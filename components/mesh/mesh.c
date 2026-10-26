/* =========================================================================
 * ESP-NOW flooding mesh — 001 ONLINE / 002 PHONE / 003 ISLAND
 * =========================================================================
 * Ported from the tested standalone mesh firmware. The state machine, channel
 * authority, three-phase search, router hold-off and RSSI election are
 * unchanged. Differences from the original, all forced by Firmware B:
 *
 *   - The relay is driven through callbacks, not GPIO here.
 *   - AWS MQTT belongs to Firmware B; a node keeps its own uplink and never
 *     steps down, so the roster is published by the app, not by this file.
 *   - The 003 WiFi phase reports the router instead of associating itself.
 *   - Sub-10 ms gaps rounded up: Firmware B runs FreeRTOS at 100 Hz, where
 *     pdMS_TO_TICKS(<10) is zero ticks.
 *   - One pending ACK slot became a table, so concurrent commands each retry.
 * ========================================================================= */

#include "mesh.h"

#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"

#include "wifi_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *M = "MESH";

/* ===================== Radio / channel ==================================== */
#define DEFAULT_CHANNEL   6
#define AP_SSID_PREFIX    "MESH-"
#define CFG_AP_PASS       "mesh1234"
#define MIN_CHANNEL       1
#define MAX_CHANNEL       13

/* ===================== Mesh timing ======================================== */
#define HB_INTERVAL_MS    1000
#define NODE_TIMEOUT_MS   25000
#define GW_TIMEOUT_MS     25000
#define ROUTER_HOLDOFF_MS 16000
#define ELECT_WINDOW_MS   800
#define INITIAL_TTL       8

#define DATA_REPEATS        5
#define DATA_REPEAT_GAP_MS  200
#define RELAY_REPEATS       2
#define RELAY_REPEAT_GAP_MS 10   /* one tick at 100 Hz; smaller rounds to 0 */
#define ACK_TIMEOUT_MS      1500
#define MAX_CMD_RETRIES     4
#define MAX_PENDING         6

/* ===================== 003 search cycle =================================== */
#define ISLAND_PHASE_MS   5000
#define PRIO_ROUNDS       3
#define PRIO_DWELL_MS     400
#define SWEEP_DWELL_MS    140

#define RSSI_EWMA_ALPHA_PCT 30
#define DEDUP_SIZE          128
#define RECV_QUEUE_DEPTH    16
#define RELAY_QUEUE_DEPTH   24
#define MESH_PAYLOAD_MAX    200
#define AP_MAX_CONN         4
#define AP_BEACON_TU        300
#define LOG_CH_HOPS         1

/* ===================== Wire format ======================================== */
#define PROTO_MAGIC 0x5A

typedef enum { MESH_ONLINE = 1, MESH_PHONE = 2, MESH_ISLAND = 3 } mesh_state_t;
typedef enum { ROLE_RELAY = 0, ROLE_GATEWAY = 1 } role_t;
enum { MSG_DATA = 1, MSG_HB = 2, MSG_ELECT = 3, MSG_ACK = 4 };

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint32_t group;
    uint8_t  origin[6];
    uint16_t seq;
    uint8_t  ttl;
    uint8_t  target[6];
    uint8_t  type;
    uint8_t  state;
    uint8_t  is_gw;
    uint8_t  score;
    uint8_t  ch;
    uint16_t ack_seq;
    uint8_t  plen;
    char     payload[MESH_PAYLOAD_MAX];
} mesh_msg_t;

#define HDR_LEN (offsetof(mesh_msg_t, payload))

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t k_prio_ch[] = {1, 6, 11};
#define PRIO_CH_COUNT (sizeof(k_prio_ch)/sizeof(k_prio_ch[0]))

static bool is_prio_ch(uint8_t ch) {
    for (size_t i = 0; i < PRIO_CH_COUNT; i++) if (k_prio_ch[i] == ch) return true;
    return false;
}

/* ===================== Globals ============================================ */
static uint8_t  s_mac[6];
static char     s_serial[MESH_SERIAL_LEN];
static uint32_t s_group = 0;
static bool     s_have_group = false;
static char     s_router_ssid[33] = {0};

static volatile mesh_state_t s_state = MESH_ISLAND;
static volatile role_t s_role = ROLE_RELAY;
static SemaphoreHandle_t s_lock;

static volatile uint8_t s_score = 0;
static SemaphoreHandle_t s_score_lock;

static volatile uint8_t s_channel = DEFAULT_CHANNEL;
static volatile bool    s_router_seen = false;
static volatile TickType_t s_router_seen_tick = 0;
static volatile TickType_t s_holdoff_start = 0;
static volatile uint8_t s_holdoff_ch = 0;

static volatile uint8_t s_announce_ch = 0;
static volatile uint8_t s_pending_ch = 0;
static volatile uint8_t s_gw_ch = 0;

static uint8_t s_gw_mac[6] = {0};
static bool    s_have_gw = false;
static TickType_t s_gw_tick = 0;
static uint8_t s_gw_state = MESH_ISLAND;
static uint8_t s_gw_score = 0;

static volatile uint16_t s_seq = 0;
static SemaphoreHandle_t s_seq_lock;

static volatile bool s_phone_connected = false;
static httpd_handle_t s_httpd = NULL;
static esp_netif_t *s_ap_netif = NULL;

static volatile bool s_cloud = false;
static volatile bool s_running = false;
static volatile bool s_radio_busy = false;
static wifi_ps_type_t s_saved_ps = WIFI_PS_MIN_MODEM;

static QueueHandle_t s_recv_q, s_relay_q;
static TaskHandle_t  s_recv_task, s_relay_task, s_state_task, s_wdog_task;

static mesh_cmd_cb_t          s_on_cmd = NULL;
static mesh_roster_cb_t       s_on_roster = NULL;
static mesh_ack_cb_t          s_on_ack = NULL;
static mesh_router_found_cb_t s_on_router = NULL;
static mesh_relay_state_cb_t  s_get_relay = NULL;
static mesh_uplink_up_cb_t    s_on_uplink_up = NULL;
static mesh_uplink_down_cb_t  s_on_uplink_down = NULL;

/* True while THIS node holds the STA association and the MQTT uplink, i.e.
 * while it is the elected 001 gateway. Edge-detected in state_task. */
static volatile bool s_uplink_on = false;

/* A command-driven stop is pending (see mesh_stop_deferred). */
static volatile bool s_stop_pending = false;
static volatile uint32_t s_stop_delay_ms = 0;

typedef struct { mesh_msg_t msg; int8_t rssi; } rx_item_t;

typedef struct {
    uint8_t mac[6];
    char    name[MESH_SERIAL_LEN];
    bool    used, active;
    TickType_t last_seen;
    int8_t  last_rssi, min_rssi;
    uint32_t rx_total;
} node_t;
static node_t s_nodes[MESH_MAX_NODES];
static SemaphoreHandle_t s_nodes_lock;

typedef struct {
    bool     used;
    uint16_t seq;
    uint8_t  target[6];
    char     cmd[MESH_PAYLOAD_MAX];
    char     command_id[32];
    int      retries;
    TickType_t next_try;
} pending_t;
static pending_t s_pending[MAX_PENDING];
static SemaphoreHandle_t s_pend_lock;

static struct { uint8_t mac[6]; uint16_t seq; bool used; } s_dedup[DEDUP_SIZE];
static int s_dedup_idx = 0;
static SemaphoreHandle_t s_dedup_lock;

static void http_start(void);
static void http_stop(void);
static void publish_role_change(void);

/* ===================== small helpers ====================================== */
static uint32_t str_hash(const char *s) {          /* FNV-1a */
    uint32_t h = 2166136261u;
    for (; *s; s++) { h ^= (uint8_t)*s; h *= 16777619u; }
    return h;
}

static void mac_to_serial(const uint8_t *m, char *out) {
    snprintf(out, MESH_SERIAL_LEN, "sig%02X:%02X:%02X:%02X:%02X:%02X",
             m[0], m[1], m[2], m[3], m[4], m[5]);
}

static bool serial_to_mac(const char *s, uint8_t *mac) {
    if (!s || strncmp(s, "sig", 3) != 0) return false;
    unsigned v[6];
    if (sscanf(s + 3, "%2X:%2X:%2X:%2X:%2X:%2X",
               &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6) return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
    return true;
}

static void ensure_mac(void) {
    static bool done = false;
    if (!done) { esp_read_mac(s_mac, ESP_MAC_WIFI_STA); mac_to_serial(s_mac, s_serial); done = true; }
}

static uint8_t rssi_to_score(int8_t rssi) {
    int v = 127 + (int)rssi;
    if (v < 1) v = 1;
    if (v > 127) v = 127;
    return (uint8_t)v;
}
static uint16_t next_seq(void) {
    xSemaphoreTake(s_seq_lock, portMAX_DELAY);
    uint16_t s = s_seq++;
    xSemaphoreGive(s_seq_lock);
    return s;
}
static void ewma_score(uint8_t sample) {
    if (sample == 0) return;
    xSemaphoreTake(s_score_lock, portMAX_DELAY);
    uint8_t prev = s_score;
    s_score = prev ? (uint8_t)((RSSI_EWMA_ALPHA_PCT * sample +
                                (100 - RSSI_EWMA_ALPHA_PCT) * prev) / 100)
                   : sample;
    xSemaphoreGive(s_score_lock);
}
static uint8_t my_score(void) {
    xSemaphoreTake(s_score_lock, portMAX_DELAY);
    uint8_t s = s_score;
    xSemaphoreGive(s_score_lock);
    return s;
}

static bool dedup_seen_or_add(const uint8_t *mac, uint16_t seq) {
    xSemaphoreTake(s_dedup_lock, portMAX_DELAY);
    for (int i = 0; i < DEDUP_SIZE; i++)
        if (s_dedup[i].used && s_dedup[i].seq == seq &&
            memcmp(s_dedup[i].mac, mac, 6) == 0) {
            xSemaphoreGive(s_dedup_lock);
            return true;
        }
    s_dedup[s_dedup_idx].used = true;
    memcpy(s_dedup[s_dedup_idx].mac, mac, 6);
    s_dedup[s_dedup_idx].seq = seq;
    s_dedup_idx = (s_dedup_idx + 1) % DEDUP_SIZE;
    xSemaphoreGive(s_dedup_lock);
    return false;
}

/* ===================== node roster ======================================== */
static int node_touch(const uint8_t *mac) {
    int free_i = -1;
    for (int i = 0; i < MESH_MAX_NODES; i++) {
        if (s_nodes[i].used && memcmp(s_nodes[i].mac, mac, 6) == 0) return i;
        if (!s_nodes[i].used && free_i < 0) free_i = i;
    }
    if (free_i < 0) return -1;
    memset(&s_nodes[free_i], 0, sizeof(node_t));
    s_nodes[free_i].used = true;
    memcpy(s_nodes[free_i].mac, mac, 6);
    mac_to_serial(mac, s_nodes[free_i].name);
    return free_i;
}

static void mark_seen(const uint8_t *mac, int8_t rssi) {
    bool joined = false;
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    int i = node_touch(mac);
    if (i >= 0) {
        if (!s_nodes[i].active) { s_nodes[i].active = true; joined = true; }
        s_nodes[i].last_seen = xTaskGetTickCount();
        s_nodes[i].last_rssi = rssi;
        if (s_nodes[i].min_rssi == 0 || rssi < s_nodes[i].min_rssi) s_nodes[i].min_rssi = rssi;
        s_nodes[i].rx_total++;
    }
    xSemaphoreGive(s_nodes_lock);
    if (joined) {
        char sn[MESH_SERIAL_LEN]; mac_to_serial(mac, sn);
        ESP_LOGI(M, "[NODE] %s joined  rssi=%d", sn, rssi);
        if (s_on_roster) s_on_roster();
    }
}

static int peers_heard_since(TickType_t since) {
    int c = 0;
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++)
        if (s_nodes[i].used && s_nodes[i].last_seen && s_nodes[i].last_seen >= since) c++;
    xSemaphoreGive(s_nodes_lock);
    return c;
}
static int peers_heard(uint32_t ms) {
    int n = 0;
    TickType_t now = xTaskGetTickCount();
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++)
        if (s_nodes[i].used && s_nodes[i].last_seen &&
            (now - s_nodes[i].last_seen) < pdMS_TO_TICKS(ms)) n++;
    xSemaphoreGive(s_nodes_lock);
    return n;
}
static bool gw_heard_since(TickType_t since) {
    bool r;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    r = s_have_gw && s_gw_tick >= since;
    xSemaphoreGive(s_lock);
    return r;
}

int mesh_node_info(mesh_node_info_t *out, int max) {
    int n = 0;
    TickType_t now = xTaskGetTickCount();
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES && n < max; i++) {
        if (!s_nodes[i].used || !s_nodes[i].active) continue;
        strncpy(out[n].serial, s_nodes[i].name, MESH_SERIAL_LEN - 1);
        out[n].serial[MESH_SERIAL_LEN - 1] = '\0';
        out[n].rssi = s_nodes[i].last_rssi;
        out[n].min_rssi = s_nodes[i].min_rssi;
        out[n].rx_total = s_nodes[i].rx_total;
        out[n].silent_ms = (s_nodes[i].last_seen > now) ? 0
            : (uint32_t)((now - s_nodes[i].last_seen) * portTICK_PERIOD_MS);
        n++;
    }
    xSemaphoreGive(s_nodes_lock);
    return n;
}

uint8_t mesh_score(void)   { return my_score(); }
uint8_t mesh_channel(void) { return s_channel; }

int mesh_roster(char out[][MESH_SERIAL_LEN], int max) {
    int n = 0;
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES && n < max; i++)
        if (s_nodes[i].used && s_nodes[i].active)
            strncpy(out[n++], s_nodes[i].name, MESH_SERIAL_LEN - 1);
    xSemaphoreGive(s_nodes_lock);
    return n;
}
void mesh_own_serial(char out[MESH_SERIAL_LEN]) { ensure_mac(); strncpy(out, s_serial, MESH_SERIAL_LEN - 1); out[MESH_SERIAL_LEN-1]='\0'; }
bool mesh_is_running(void) { return s_running; }
bool mesh_has_group(void)  { return s_have_group; }
bool mesh_is_gateway(void) { return s_role == ROLE_GATEWAY; }
uint8_t mesh_state(void)   { return (uint8_t)s_state; }

void mesh_set_group(const char *group_id) {
    if (!group_id || !group_id[0]) { s_have_group = false; s_group = 0; ESP_LOGI(M, "Group cleared"); return; }
    s_group = str_hash(group_id);
    s_have_group = true;
    ESP_LOGI(M, "Group set (hash %08X)", (unsigned)s_group);
}
void mesh_set_router_ssid(const char *ssid) {
    if (ssid) { strncpy(s_router_ssid, ssid, sizeof(s_router_ssid) - 1); s_router_ssid[sizeof(s_router_ssid)-1]='\0'; }
}
void mesh_set_home_channel(uint8_t ch) { if (ch >= MIN_CHANNEL && ch <= MAX_CHANNEL) s_channel = ch; }
void mesh_set_uplink_cbs(mesh_uplink_up_cb_t on_up, mesh_uplink_down_cb_t on_down) {
    s_on_uplink_up = on_up;
    s_on_uplink_down = on_down;
}
void mesh_set_cloud(bool up) { s_cloud = up; }

/* ===================== ESP-NOW send / recv ================================ */
static void espnow_send(const mesh_msg_t *m, int len) { esp_now_send(BCAST, (const uint8_t *)m, len); }

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!s_running || len < (int)HDR_LEN || len > (int)sizeof(mesh_msg_t)) return;
    rx_item_t it;
    memset(&it, 0, sizeof(it));
    memcpy(&it.msg, data, len);
    if (it.msg.magic != PROTO_MAGIC || it.msg.group != s_group) return;
    int avail = len - (int)HDR_LEN;
    if (avail < 0) avail = 0;
    if (it.msg.plen > avail) it.msg.plen = (uint8_t)avail;
    it.rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
    xQueueSend(s_recv_q, &it, 0);
}

static void hdr_init(mesh_msg_t *m, uint8_t type, const uint8_t *target) {
    memset(m, 0, sizeof(*m));
    m->magic = PROTO_MAGIC;
    m->group = s_group;
    memcpy(m->origin, s_mac, 6);
    m->seq = next_seq();
    m->ttl = INITIAL_TTL;
    memcpy(m->target, target ? target : BCAST, 6);
    m->type = type;
}

/* ===================== heartbeat ========================================== */
static void send_heartbeat(void) {
    mesh_msg_t m;
    hdr_init(&m, MSG_HB, BCAST);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    m.state = (uint8_t)s_state;
    m.is_gw = (s_role == ROLE_GATEWAY) ? 1 : 0;
    xSemaphoreGive(s_lock);
    m.score = my_score();
    m.ch = s_announce_ch ? s_announce_ch : s_channel;
    int plen = snprintf(m.payload, MESH_PAYLOAD_MAX, "%s", s_serial);
    m.plen = (uint8_t)plen;
    espnow_send(&m, HDR_LEN + plen + 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    espnow_send(&m, HDR_LEN + plen + 1);
}

/* ===================== relay / flood ====================================== */
static void relay_task(void *a) {
    mesh_msg_t m;
    while (1)
        if (xQueueReceive(s_relay_q, &m, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(10 + (esp_random() % 80)));
            for (int i = 0; i < RELAY_REPEATS; i++) {
                espnow_send(&m, HDR_LEN + m.plen + 1);
                if (i < RELAY_REPEATS - 1) vTaskDelay(pdMS_TO_TICKS(RELAY_REPEAT_GAP_MS));
            }
        }
}
static void enqueue_relay(const mesh_msg_t *m) {
    if (m->ttl <= 1) return;
    mesh_msg_t f = *m;
    f.ttl--;
    if (xQueueSend(s_relay_q, &f, 0) != pdTRUE) ESP_LOGW(M, "relay queue full");
}

static void send_ack(uint16_t cmd_seq) {
    mesh_msg_t m;
    hdr_init(&m, MSG_ACK, BCAST);
    m.ack_seq = cmd_seq;
    m.payload[0] = (s_get_relay && s_get_relay()) ? '1' : '0';
    m.payload[1] = '\0';
    m.plen = 1;
    for (int i = 0; i < 3; i++) { espnow_send(&m, HDR_LEN + m.plen + 1); vTaskDelay(pdMS_TO_TICKS(10)); }
}

/* ===================== channel management ================================= */
static void mesh_set_channel_r(uint8_t ch, const char *why) {
    uint8_t prev = s_channel;
    if (ch < MIN_CHANNEL || ch > MAX_CHANNEL) { ESP_LOGW(M, "[CH] reject %u (%s)", ch, why); return; }
    wifi_config_t ap;
    if (esp_wifi_get_config(WIFI_IF_AP, &ap) == ESP_OK && ap.ap.channel != ch) {
        ap.ap.channel = ch;
        esp_wifi_set_config(WIFI_IF_AP, &ap);
    }
    esp_err_t e = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (e != ESP_OK) {
        /* Do NOT update s_channel on failure — country code 01 rejects 12/13. */
        ESP_LOGW(M, "[CH] FAIL %u -> %u : %s (%s)", prev, ch, esp_err_to_name(e), why);
        return;
    }
    s_channel = ch;
    ESP_LOGI(M, "[CH] %u -> %u (%s)", prev, ch, why);
}

static void radio_hop(uint8_t ch, const char *why) {
    if (esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE) != ESP_OK) return;
#if LOG_CH_HOPS
    ESP_LOGI(M, "[HOP] ch %u (%s)", ch, why);
#endif
}

/* Announce the destination on the CURRENT channel, THEN move. Once you have
 * moved you can no longer reach the nodes you left behind. */
static void mesh_migrate(uint8_t ch) {
    if (ch < MIN_CHANNEL || ch > MAX_CHANNEL) return;
    if (ch == s_channel) { s_gw_ch = ch; return; }
    ESP_LOGI(M, "[CH] announcing move %u -> %u", s_channel, ch);
    s_announce_ch = ch;
    for (int i = 0; i < 3; i++) send_heartbeat();
    s_announce_ch = 0;
    s_gw_ch = ch;
    mesh_set_channel_r(ch, "migrate to router");
}

/* ===================== state / role ======================================= */
static void set_state_role(mesh_state_t st, role_t role) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool changed = (s_state != st) || (s_role != role);
    s_state = st;
    s_role = role;
    xSemaphoreGive(s_lock);
    if (changed) {
        static const char *sn[] = {"?", "001-ONLINE", "002-PHONE", "003-ISLAND"};
        ESP_LOGI(M, "[STATE] -> %s (%s)", sn[st], role == ROLE_GATEWAY ? "GATEWAY" : "relay");
        publish_role_change();
    }
}
static void publish_role_change(void) { if (s_on_roster) s_on_roster(); }

/* ===================== send a command into the mesh ======================= */
bool mesh_send_command(const char *target_serial, const char *command_id, const char *json) {
    if (!s_running) return false;
    uint8_t target[6];
    bool is_bcast = (target_serial == NULL);
    if (!is_bcast && !serial_to_mac(target_serial, target)) {
        ESP_LOGE(M, "Bad target serial '%s'", target_serial);
        return false;
    }
    if (is_bcast) memcpy(target, BCAST, 6);
    if (!is_bcast && memcmp(target, s_mac, 6) == 0) return false;

    mesh_msg_t m;
    hdr_init(&m, MSG_DATA, target);
    int plen = snprintf(m.payload, MESH_PAYLOAD_MAX, "%s", json);
    if (plen < 0 || plen >= MESH_PAYLOAD_MAX) { ESP_LOGE(M, "Command too long"); return false; }
    m.plen = (uint8_t)plen;
    m.ch = s_channel;

    /* ACK/retry only for a unicast aimed at another node — a node never ACKs
     * itself, and a broadcast has no single responder. */
    if (!is_bcast) {
        xSemaphoreTake(s_pend_lock, portMAX_DELAY);
        int slot = -1;
        for (int i = 0; i < MAX_PENDING; i++) if (!s_pending[i].used) { slot = i; break; }
        if (slot >= 0) {
            s_pending[slot].used = true;
            s_pending[slot].seq = m.seq;
            memcpy(s_pending[slot].target, target, 6);
            strncpy(s_pending[slot].cmd, m.payload, MESH_PAYLOAD_MAX - 1);
            s_pending[slot].cmd[MESH_PAYLOAD_MAX-1] = '\0';
            s_pending[slot].command_id[0] = '\0';
            if (command_id) strncpy(s_pending[slot].command_id, command_id, sizeof(s_pending[slot].command_id) - 1);
            s_pending[slot].retries = 0;
            s_pending[slot].next_try = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_MS);
        }
        xSemaphoreGive(s_pend_lock);
        if (slot < 0) { ESP_LOGE(M, "Pending table full"); return false; }
    }

    for (int i = 0; i < DATA_REPEATS; i++) {
        espnow_send(&m, HDR_LEN + plen + 1);
        if (i < DATA_REPEATS - 1) vTaskDelay(pdMS_TO_TICKS(DATA_REPEAT_GAP_MS));
    }
    ESP_LOGI(M, "[CMD] sent seq=%u %s", m.seq, is_bcast ? "(all)" : target_serial);
    return true;
}

/* ===================== gateway tracking =================================== */
static void note_gateway(const uint8_t *mac, uint8_t gw_state, uint8_t score) {
    xSemaphoreTake(s_lock, portMAX_DELAY);
    TickType_t now = xTaskGetTickCount();
    bool have_fresh_online = s_have_gw && s_gw_state == MESH_ONLINE &&
                             (now - s_gw_tick) < pdMS_TO_TICKS(GW_TIMEOUT_MS);
    if (!(gw_state == MESH_PHONE && have_fresh_online)) {
        memcpy(s_gw_mac, mac, 6);
        s_have_gw = true;
        s_gw_state = gw_state;
        s_gw_score = score;
        s_gw_tick = now;
    }
    xSemaphoreGive(s_lock);
}

/* ===================== election =========================================== */
static volatile bool s_elect_active = false;
static uint8_t s_best_mac[6];
static uint8_t s_best_score = 0;

static bool candidate_beats(uint8_t sa, const uint8_t *ma, uint8_t sb, const uint8_t *mb) {
    if (sa != sb) return sa > sb;
    return memcmp(ma, mb, 6) > 0;
}
static void elect_observe(const uint8_t *mac, uint8_t score) {
    if (!s_elect_active) return;
    if (candidate_beats(score, mac, s_best_score, s_best_mac)) {
        s_best_score = score;
        memcpy(s_best_mac, mac, 6);
    }
}

/* ===================== receive processing ================================= */
static void process_msg(mesh_msg_t *m, int len, int8_t rssi) {
    int plen = len - (int)HDR_LEN;
    if (plen < 0) plen = 0;
    if (plen >= MESH_PAYLOAD_MAX) plen = MESH_PAYLOAD_MAX - 1;
    m->payload[plen] = '\0';

    if (memcmp(m->origin, s_mac, 6) == 0) return;
    if (dedup_seen_or_add(m->origin, m->seq)) return;
    mark_seen(m->origin, rssi);

    switch (m->type) {
    case MSG_HB: {
        bool gw_hb = m->is_gw && (m->state == MESH_ONLINE || m->state == MESH_PHONE);
        if (gw_hb) {
            note_gateway(m->origin, m->state, m->score);
            elect_observe(m->origin, m->score);
        }
        /* CHANNEL AUTHORITY. Never follow anyone while we hold the gateway
         * role — a gateway's channel is the router's channel. */
        if (m->ch >= MIN_CHANNEL && m->ch <= MAX_CHANNEL &&
            s_role != ROLE_GATEWAY) {
            if (gw_hb) s_gw_ch = m->ch;
            bool from_our_gw;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            from_our_gw = s_have_gw && memcmp(m->origin, s_gw_mac, 6) == 0;
            xSemaphoreGive(s_lock);
            bool trust = gw_hb || from_our_gw ||
                         (s_state == MESH_ISLAND && m->score > my_score());
            if (trust && m->ch != s_channel) s_pending_ch = m->ch;
        }
        enqueue_relay(m);
        break;
    }

    case MSG_ELECT:
        elect_observe(m->origin, m->score);
        enqueue_relay(m);
        break;

    case MSG_DATA: {
        bool for_me = (memcmp(m->target, s_mac, 6) == 0) ||
                      (memcmp(m->target, BCAST, 6) == 0);
        if (for_me && s_on_cmd) {
            s_on_cmd(m->payload);
            send_ack(m->seq);
        }
        enqueue_relay(m);
        break;
    }

    case MSG_ACK: {
        char serial[MESH_SERIAL_LEN] = {0};
        char cid[32] = {0};
        bool matched = false;
        xSemaphoreTake(s_pend_lock, portMAX_DELAY);
        for (int i = 0; i < MAX_PENDING; i++)
            if (s_pending[i].used && s_pending[i].seq == m->ack_seq) {
                mac_to_serial(s_pending[i].target, serial);
                strncpy(cid, s_pending[i].command_id, sizeof(cid) - 1);
                s_pending[i].used = false;
                matched = true;
                break;
            }
        xSemaphoreGive(s_pend_lock);
        if (matched) {
            ESP_LOGI(M, "[ACK] %s -> %s", serial, m->payload[0] == '1' ? "on" : "off");
            if (s_on_ack) s_on_ack(serial, cid[0] ? cid : NULL, m->plen ? m->payload : NULL);
        }
        enqueue_relay(m);
        break;
    }
    default: break;
    }
}

static void recv_task(void *a) {
    rx_item_t it;
    while (1)
        if (xQueueReceive(s_recv_q, &it, portMAX_DELAY) == pdTRUE)
            process_msg(&it.msg, HDR_LEN + it.msg.plen + 1, it.rssi);
}

/* ===================== WiFi scan for the router =========================== */
static uint8_t wifi_scan_modem_ch(uint8_t only_ch, uint8_t *out_ch) {
    if (!s_router_ssid[0]) return 0;
    wifi_scan_config_t cfg = {0};
    cfg.ssid = (uint8_t *)s_router_ssid;
    cfg.channel = only_ch;
    cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    cfg.scan_time.active.min = 50;
    cfg.scan_time.active.max = 120;
    if (esp_wifi_scan_start(&cfg, true) != ESP_OK) return 0;

    uint16_t n = 12;
    wifi_ap_record_t *recs = calloc(n, sizeof(*recs));
    if (!recs) return 0;
    if (esp_wifi_scan_get_ap_records(&n, recs) != ESP_OK) { free(recs); return 0; }

    int8_t best = -127;
    bool found = false;
    uint8_t ch = 0;
    for (int i = 0; i < n; i++)
        if (strcmp((char *)recs[i].ssid, s_router_ssid) == 0 && recs[i].rssi > best) {
            best = recs[i].rssi; found = true; ch = recs[i].primary;
        }
    free(recs);

    uint8_t score = found ? rssi_to_score(best) : 0;
    ewma_score(score);
    if (found) {
        s_router_seen = true;
        s_router_seen_tick = xTaskGetTickCount();
        if (out_ch) *out_ch = ch;
        ESP_LOGI(M, "[SCAN] FOUND '%s' rssi=%d ch=%u score=%u", s_router_ssid, best, ch, score);
    }
    esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);
    return score;
}

static uint8_t wifi_search_router(uint8_t *out_ch) {
    ESP_LOGI(M, "[PHASE 1 WIFI] router search: 1/6/11 x%d then full sweep", PRIO_ROUNDS);
    for (int round = 0; round < PRIO_ROUNDS; round++)
        for (size_t i = 0; i < PRIO_CH_COUNT; i++) {
            uint8_t sc = wifi_scan_modem_ch(k_prio_ch[i], out_ch);
            if (sc > 0) return sc;
        }
    return wifi_scan_modem_ch(0, out_ch);
}

/* ===================== 003 phase 0: gateway listen ======================== */
static uint8_t gateway_listen_scan(void) {
    ESP_LOGI(M, "[PHASE 0 LISTEN] gw_ch=%u ch=%u peers=%d", s_gw_ch, s_channel, peers_heard(NODE_TIMEOUT_MS));
    if (s_gw_ch >= MIN_CHANNEL && s_gw_ch <= MAX_CHANNEL) {
        if (s_gw_ch != s_channel) mesh_set_channel_r(s_gw_ch, "known gateway channel");
        TickType_t t0 = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(ISLAND_PHASE_MS));
        if (gw_heard_since(t0)) { ESP_LOGI(M, "[PHASE 0] gateway heard on ch %u", s_channel); return s_channel; }
        return 0;
    }
    for (int round = 0; round < PRIO_ROUNDS; round++)
        for (size_t i = 0; i < PRIO_CH_COUNT; i++) {
            uint8_t ch = k_prio_ch[i];
            radio_hop(ch, "listen prio");
            TickType_t t0 = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(PRIO_DWELL_MS));
            if (gw_heard_since(t0)) { ESP_LOGI(M, "[PHASE 0] gateway on ch %u", ch); return ch; }
        }
    for (uint8_t ch = MIN_CHANNEL; ch <= MAX_CHANNEL; ch++) {
        if (is_prio_ch(ch)) continue;
        radio_hop(ch, "listen sweep");
        TickType_t t0 = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(SWEEP_DWELL_MS));
        if (gw_heard_since(t0)) { ESP_LOGI(M, "[PHASE 0] gateway on ch %u (sweep)", ch); return ch; }
    }
    return 0;
}

/* ===================== 003 phase 2: peer walk ============================= */
static uint8_t peer_listen_walk(int *best_n_out) {
    int best_n = -1;
    uint8_t best_ch = s_channel, gw_ch = 0;
    ESP_LOGI(M, "[PHASE 2 WALK] start (ch=%u gw_ch=%u)", s_channel, s_gw_ch);

    for (int round = 0; round < PRIO_ROUNDS && !gw_ch; round++)
        for (size_t i = 0; i < PRIO_CH_COUNT && !gw_ch; i++) {
            uint8_t ch = k_prio_ch[i];
            radio_hop(ch, "walk prio");
            TickType_t t0 = xTaskGetTickCount();
            send_heartbeat();
            vTaskDelay(pdMS_TO_TICKS(PRIO_DWELL_MS));
            if (gw_heard_since(t0)) { gw_ch = ch; break; }
            int c = peers_heard_since(t0);
            if (c > best_n || (c == best_n && ch < best_ch)) { best_n = c; best_ch = ch; }
        }

    for (uint8_t ch = MIN_CHANNEL; ch <= MAX_CHANNEL && !gw_ch; ch++) {
        if (is_prio_ch(ch)) continue;
        radio_hop(ch, "walk sweep");
        TickType_t t0 = xTaskGetTickCount();
        send_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(SWEEP_DWELL_MS));
        if (gw_heard_since(t0)) { gw_ch = ch; break; }
        int c = peers_heard_since(t0);
        if (c > best_n || (c == best_n && ch < best_ch)) { best_n = c; best_ch = ch; }
    }

    uint8_t settle;
    if (gw_ch) settle = gw_ch;
    else if (s_gw_ch >= MIN_CHANNEL && s_gw_ch <= MAX_CHANNEL) settle = s_gw_ch;
    else settle = best_ch;

    mesh_set_channel_r(settle, "walk settle");
    ESP_LOGI(M, "[PHASE 2 WALK] settled ch %u (%d peers)", s_channel, best_n < 0 ? 0 : best_n);
    if (best_n_out) *best_n_out = best_n < 0 ? 0 : best_n;
    return settle;
}

/* ===================== election (003 -> 001) ============================== */
/* Runs ON THE ROUTER'S CHANNEL, after mesh_migrate() has taken us — and, via
 * the announcement, our neighbours — there. Highest RSSI wins; MAC breaks ties. */
static void run_election(void) {
    uint8_t sc = my_score();
    if (sc == 0) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool incumbent = s_have_gw && s_gw_state == MESH_ONLINE &&
                     (xTaskGetTickCount() - s_gw_tick) < pdMS_TO_TICKS(GW_TIMEOUT_MS) &&
                     memcmp(s_gw_mac, s_mac, 6) != 0;
    xSemaphoreGive(s_lock);

    if (!incumbent) {
        ESP_LOGI(M, "[ELECT] listening on ch %u for an existing gateway", s_channel);
        TickType_t t0 = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(2200));
        incumbent = gw_heard_since(t0);
    }
    if (incumbent) {
        ESP_LOGI(M, "[ELECT] gateway already present -> relay on ch %u", s_channel);
        set_state_role(MESH_ONLINE, ROLE_RELAY);
        return;
    }

    s_best_score = sc;
    memcpy(s_best_mac, s_mac, 6);
    s_elect_active = true;

    mesh_msg_t m;
    hdr_init(&m, MSG_ELECT, BCAST);
    m.score = sc;
    m.ch = s_channel;
    m.plen = 0;
    m.payload[0] = '\0';
    for (int i = 0; i < 2; i++) { espnow_send(&m, HDR_LEN + 1); vTaskDelay(pdMS_TO_TICKS(10)); }

    vTaskDelay(pdMS_TO_TICKS(ELECT_WINDOW_MS));
    s_elect_active = false;

    if (memcmp(s_best_mac, s_mac, 6) == 0) {
        ESP_LOGI(M, "[ELECT] won (score=%u) -> 001 gateway on ch %u", sc, s_channel);
        set_state_role(MESH_ONLINE, ROLE_GATEWAY);
        s_gw_ch = s_channel;
        /* The uplink edge in state_task raises the association and MQTT — it
         * runs outside s_lock, so app callbacks never fire under the mutex. */
    } else {
        ESP_LOGI(M, "[ELECT] lost -> relay on ch %u (winner score=%u)", s_channel, s_best_score);
    }
}

/* ===================== state machine ======================================
 * Priority each tick:
 *   apply a channel another node told us to move to
 *   drive the uplink (only the 001 gateway holds the STA and MQTT)
 *   (1) phone connected            -> 002 gateway   (no searching)
 *   (2) we are the online gateway  -> maintain 001, step down if the router
 *                                     is gone (no searching)
 *   (3) fresh gateway heartbeat    -> follow it as a relay (no searching)
 *   (4) otherwise                  -> 003 island, three-phase search
 */
static void state_task(void *a) {
    TickType_t last_phase = 0;
    int phase = 0;
    vTaskDelay(pdMS_TO_TICKS(esp_random() % ISLAND_PHASE_MS));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        TickType_t now = xTaskGetTickCount();

        uint8_t pc = s_pending_ch;
        if (pc) {
            s_pending_ch = 0;
            if (s_role != ROLE_GATEWAY &&
                pc >= MIN_CHANNEL && pc <= MAX_CHANNEL && pc != s_channel)
                mesh_set_channel_r(pc, "following the mesh");
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        role_t role = s_role;
        mesh_state_t st = s_state;
        bool gw_fresh = s_have_gw && (now - s_gw_tick) < pdMS_TO_TICKS(GW_TIMEOUT_MS);
        uint8_t gwst = s_gw_state;
        xSemaphoreGive(s_lock);

        /* ---- uplink ownership ---------------------------------------- *
         * Exactly ONE node holds the STA association and the MQTT uplink:
         * the elected 001 gateway. Everyone else is an unassociated relay.
         * MESH_PHONE must NOT raise the uplink — there is no router in 002.
         * Driven OUTSIDE s_lock: these are app callbacks and must never run
         * with the mesh mutex held. */
        bool want_uplink = (role == ROLE_GATEWAY && st == MESH_ONLINE);
        if (want_uplink && !s_uplink_on) {
            s_uplink_on = true;
            s_router_seen_tick = xTaskGetTickCount();
            ESP_LOGI(M, "[UPLINK] up (ch %u)", s_channel);
            if (s_on_uplink_up) s_on_uplink_up(s_channel);
        } else if (!want_uplink && s_uplink_on) {
            s_uplink_on = false;
            ESP_LOGI(M, "[UPLINK] down");
            if (s_on_uplink_down) s_on_uplink_down();
        }

        /* ---- (1) 002 PHONE ------------------------------------------- */
        if (s_phone_connected && !(role == ROLE_GATEWAY && st == MESH_ONLINE) &&
            !(gw_fresh && gwst == MESH_ONLINE)) {
            set_state_role(MESH_PHONE, ROLE_GATEWAY);
            s_gw_ch = s_channel;
            continue;
        }
        if (role == ROLE_GATEWAY && st == MESH_PHONE && !s_phone_connected)
            set_state_role(MESH_ISLAND, ROLE_RELAY);

        /* ---- (2) 001 ONLINE gateway ---------------------------------- *
         * Only an EXISTING gateway re-evaluates. A relay follows case (3) and
         * never reconsiders — that is what keeps the role stable.          */
        if (role == ROLE_GATEWAY && st == MESH_ONLINE) {
            /* dual-gateway: deterministic tiebreak, same rule the election
             * uses. With a strict '>' two nodes at equal RSSI would each wait
             * for the other and neither would ever yield. */
            if (gw_fresh && gwst == MESH_ONLINE && memcmp(s_gw_mac, s_mac, 6) != 0 &&
                candidate_beats(s_gw_score, s_gw_mac, my_score(), s_mac)) {
                ESP_LOGI(M, "[STATE] dual-GW: peer %u vs ours %u -> yielding",
                         s_gw_score, my_score());
                /* Role FIRST. The uplink edge on the next tick takes the STA
                 * down; it must not see us still holding the gateway role. */
                set_state_role(MESH_ISLAND, ROLE_RELAY);
                continue;
            }
            wifi_ap_record_t ap;
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                ewma_score(rssi_to_score(ap.rssi));
                s_router_seen_tick = now;
                uint8_t ch = 0;
                wifi_second_chan_t sc2;
                if (esp_wifi_get_channel(&ch, &sc2) == ESP_OK && ch) s_channel = ch;
            } else if (s_router_seen_tick &&
                       (now - s_router_seen_tick) > pdMS_TO_TICKS(GW_TIMEOUT_MS)) {
                /* The router is gone. Step down: the uplink edge on the next
                 * tick releases the STA and stops MQTT, and the 003 search
                 * then owns the radio again. */
                ESP_LOGI(M, "[STATE] router gone -> stepping down from 001");
                set_state_role(MESH_ISLAND, ROLE_RELAY);
            }
            s_holdoff_start = 0;
            s_holdoff_ch = 0;
            continue;
        }

        if (role == ROLE_GATEWAY || gw_fresh) { s_holdoff_start = 0; s_holdoff_ch = 0; }

        /* ---- (3) follow a fresh gateway as a relay -------------------- */
        if (gw_fresh) { set_state_role((mesh_state_t)gwst, ROLE_RELAY); continue; }

        /* ---- (4) 003 ISLAND ------------------------------------------ */
        set_state_role(MESH_ISLAND, ROLE_RELAY);
        if ((now - last_phase) < pdMS_TO_TICKS(ISLAND_PHASE_MS)) continue;
        last_phase = now;

        switch (phase) {
        case 0: {
            uint8_t ch = gateway_listen_scan();
            if (ch) mesh_set_channel_r(ch, "gateway found by listen");
            last_phase = xTaskGetTickCount();
            break;
        }
        case 1: {
            uint8_t ch = 0;
            s_radio_busy = true;
            uint8_t sc = wifi_search_router(&ch);
            s_radio_busy = false;
            if (sc > 0 && ch) {
                TickType_t tnow = xTaskGetTickCount();
                if (s_holdoff_start == 0 || s_holdoff_ch != ch) {
                    /* First sighting this spell. Claiming the role now would
                     * make us the only candidate and drag the mesh behind us. */
                    s_holdoff_start = tnow;
                    s_holdoff_ch = ch;
                    ESP_LOGI(M, "[ELECT] router on ch %u — holding off %u ms", ch, (unsigned)ROUTER_HOLDOFF_MS);
                    /* Informational only: the app remembers the channel. It
                     * must NOT associate here — only the winner associates. */
                    if (s_on_router) s_on_router(ch);
                } else if ((tnow - s_holdoff_start) >= pdMS_TO_TICKS(ROUTER_HOLDOFF_MS)) {
                    ESP_LOGI(M, "[ELECT] hold-off elapsed -> migrating to ch %u", ch);
                    s_holdoff_start = 0;
                    s_holdoff_ch = 0;
                    mesh_migrate(ch);
                    s_radio_busy = true;
                    run_election();
                    s_radio_busy = false;
                }
            } else if (s_holdoff_start) {
                ESP_LOGI(M, "[ELECT] router lost again -> hold-off cancelled");
                s_holdoff_start = 0;
                s_holdoff_ch = 0;
            }
            last_phase = xTaskGetTickCount();
            break;
        }
        case 2: {
            int n = 0;
            s_radio_busy = true;
            peer_listen_walk(&n);
            s_radio_busy = false;
            last_phase = xTaskGetTickCount();
            break;
        }
        }
        phase = (phase + 1) % 3;
    }
}

/* ===================== watchdog: heartbeat, timeout, retry ================ */
static void wdog_task(void *a) {
    vTaskDelay(pdMS_TO_TICKS(1500));
    TickType_t last_hb = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        TickType_t now = xTaskGetTickCount();

        if (now - last_hb >= pdMS_TO_TICKS(HB_INTERVAL_MS)) {
            vTaskDelay(pdMS_TO_TICKS(10 + (esp_random() % 40)));
            send_heartbeat();
            last_hb = xTaskGetTickCount();
        }

        /* RE-READ THE CLOCK. `now` above was sampled BEFORE the heartbeat
         * block, which sleeps. TickType_t is unsigned, so a stale `now` minus
         * a newer last_seen wraps to ~4.29e9 and a healthy node is marked
         * gone — the bogus "silent 4294967s" in the logs. */
        now = xTaskGetTickCount();

        bool dropped = false;
        if (!s_radio_busy) {
            xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
            for (int i = 0; i < MESH_MAX_NODES; i++) {
                if (!s_nodes[i].used || !s_nodes[i].active) continue;
                if (s_nodes[i].last_seen > now) continue;
                if ((now - s_nodes[i].last_seen) > pdMS_TO_TICKS(NODE_TIMEOUT_MS)) {
                    s_nodes[i].active = false;
                    dropped = true;
                    ESP_LOGI(M, "[NODE] %s left (last rssi=%d, %lu frames)",
                             s_nodes[i].name, s_nodes[i].last_rssi,
                             (unsigned long)s_nodes[i].rx_total);
                }
            }
            xSemaphoreGive(s_nodes_lock);
        }
        if (dropped && s_on_roster) s_on_roster();

        for (int i = 0; i < MAX_PENDING; i++) {
            bool go = false;
            pending_t local;
            xSemaphoreTake(s_pend_lock, portMAX_DELAY);
            if (s_pending[i].used && now >= s_pending[i].next_try) {
                if (s_pending[i].retries < MAX_CMD_RETRIES) {
                    s_pending[i].retries++;
                    s_pending[i].next_try = now + pdMS_TO_TICKS(ACK_TIMEOUT_MS);
                    local = s_pending[i];
                    go = true;
                } else {
                    ESP_LOGW(M, "[CMD] max retries — dropping");
                    s_pending[i].used = false;
                }
            }
            xSemaphoreGive(s_pend_lock);
            if (go) {
                mesh_msg_t m;
                hdr_init(&m, MSG_DATA, local.target);
                m.seq = local.seq;          /* keep the seq so a late ACK matches */
                int n = snprintf(m.payload, MESH_PAYLOAD_MAX, "%s", local.cmd);
                m.plen = (uint8_t)n;
                m.ch = s_channel;
                for (int k = 0; k < DATA_REPEATS; k++) {
                    espnow_send(&m, HDR_LEN + n + 1);
                    if (k < DATA_REPEATS - 1) vTaskDelay(pdMS_TO_TICKS(DATA_REPEAT_GAP_MS));
                }
            }
        }
    }
}

/* ===================== SoftAP + HTTP control ============================== */
static const char INDEX_HTML[] =
    "<!doctype html><meta charset=utf-8>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>Mesh</title><style>body{font-family:sans-serif;background:#111;color:#eee;margin:16px}"
    "button{font-size:16px;padding:8px 14px;margin:3px;border:0;border-radius:6px;color:#fff}"
    ".on{background:#2e7d32}.off{background:#c62828}#s{margin-top:12px;color:#9e9e9e}</style>"
    "<h3>Mesh control</h3>"
    "<button class=on onclick=\"c('all on')\">ALL ON</button>"
    "<button class=off onclick=\"c('all off')\">ALL OFF</button>"
    "<div id=s>ready</div>"
    "<p><a href=/status style=color:#6cf>/status</a></p>"
    "<script>function c(x){fetch('/cmd?c='+encodeURIComponent(x)).then(r=>r.text())"
    ".then(t=>document.getElementById('s').textContent=x+' -> '+t)}</script>";

static esp_err_t h_index(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static void url_decode(char *dst, const char *src, size_t n) {
    size_t d = 0;
    for (size_t i = 0; src[i] && d + 1 < n; i++) {
        if (src[i] == '+') dst[d++] = ' ';
        else if (src[i] == '%' && isxdigit((int)src[i+1]) && isxdigit((int)src[i+2])) {
            char h[3] = {src[i+1], src[i+2], 0};
            dst[d++] = (char)strtol(h, 0, 16);
            i += 2;
        } else dst[d++] = src[i];
    }
    dst[d] = '\0';
}

/* Grammar: "<sig{MAC}|all> <on|off>" — turned into the same JSON the backend
 * sends, so the offline path and the online path execute identically. */
static esp_err_t h_cmd(httpd_req_t *r) {
    char q[96], raw[64], cmd[64];
    if (httpd_req_get_url_query_str(r, q, sizeof(q)) != ESP_OK ||
        httpd_query_key_value(q, "c", raw, sizeof(raw)) != ESP_OK) {
        httpd_resp_set_status(r, "400 Bad Request");
        return httpd_resp_sendstr(r, "no cmd");
    }
    url_decode(cmd, raw, sizeof(cmd));

    char *sp = strchr(cmd, ' ');
    if (!sp) return httpd_resp_sendstr(r, "bad");
    *sp = '\0';
    const char *target = cmd;
    const char *state  = sp + 1;
    if (strcmp(state, "on") != 0 && strcmp(state, "off") != 0)
        return httpd_resp_sendstr(r, "bad state");

    char json[128];
    snprintf(json, sizeof(json), "{\"type\":\"relay1\",\"state\":\"%s\",\"commandId\":\"http-%u\"}",
             state, (unsigned)(xTaskGetTickCount() & 0xFFFF));

    if (strcmp(target, "all") == 0) {
        if (s_on_cmd) s_on_cmd(json);          /* apply locally too */
        mesh_send_command(NULL, NULL, json);
    } else if (strcmp(target, s_serial) == 0) {
        if (s_on_cmd) s_on_cmd(json);
    } else {
        mesh_send_command(target, NULL, json);
    }
    return httpd_resp_sendstr(r, "ok");
}

static esp_err_t h_status(httpd_req_t *r) {
    char buf[320];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    mesh_state_t st = s_state;
    role_t role = s_role;
    xSemaphoreGive(s_lock);
    int len = snprintf(buf, sizeof(buf),
        "{\"name\":\"%s\",\"state\":%u,\"role\":\"%s\",\"channel\":%u,"
        "\"gw_ch\":%u,\"score\":%u,\"relay\":\"%s\",\"active\":[",
        s_serial, (unsigned)st, role == ROLE_GATEWAY ? "gateway" : "relay",
        s_channel, s_gw_ch, my_score(),
        (s_get_relay && s_get_relay()) ? "on" : "off");
    httpd_resp_set_type(r, "application/json");
    httpd_resp_send_chunk(r, buf, len);
    bool first = true;
    xSemaphoreTake(s_nodes_lock, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_NODES; i++)
        if (s_nodes[i].used && s_nodes[i].active) {
            len = snprintf(buf, sizeof(buf), "%s\"%s\"", first ? "" : ",", s_nodes[i].name);
            httpd_resp_send_chunk(r, buf, len);
            first = false;
        }
    xSemaphoreGive(s_nodes_lock);
    httpd_resp_send_chunk(r, "]}", 2);
    return httpd_resp_send_chunk(r, NULL, 0);
}

static void http_start(void) {
    if (s_httpd) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = 4;
    cfg.stack_size = 4096;
    if (httpd_start(&s_httpd, &cfg) != ESP_OK) { ESP_LOGE(M, "[HTTP] start failed"); return; }
    httpd_uri_t u[] = {
        {.uri = "/",       .method = HTTP_GET, .handler = h_index},
        {.uri = "/cmd",    .method = HTTP_GET, .handler = h_cmd},
        {.uri = "/status", .method = HTTP_GET, .handler = h_status},
    };
    for (int i = 0; i < 3; i++) httpd_register_uri_handler(s_httpd, &u[i]);
    ESP_LOGI(M, "[HTTP] up on http://192.168.4.1/ (SoftAP %s%s)", AP_SSID_PREFIX, s_serial);
}
static void http_stop(void) { if (s_httpd) { httpd_stop(s_httpd); s_httpd = NULL; } }

static void wifi_evt(void *a, esp_event_base_t base, int32_t id, void *data) {
    if (base != WIFI_EVENT) return;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        s_phone_connected = true;
        ESP_LOGI(M, "[AP] station connected -> phone present");
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_sta_list_t list;
        if (esp_wifi_ap_get_sta_list(&list) == ESP_OK && list.num == 0) {
            s_phone_connected = false;
            ESP_LOGI(M, "[AP] last station gone -> no phone");
        }
    }
}

static void softap_start(void) {
    if (!s_ap_netif) s_ap_netif = esp_netif_create_default_wifi_ap();

    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    if (mode != WIFI_MODE_APSTA) esp_wifi_set_mode(WIFI_MODE_APSTA);

    wifi_config_t ap = {0};
    int an = snprintf((char *)ap.ap.ssid, sizeof(ap.ap.ssid), "%s%s", AP_SSID_PREFIX, s_serial);
    ap.ap.ssid_len = an;
    ap.ap.channel = s_channel;
    ap.ap.max_connection = AP_MAX_CONN;
    ap.ap.beacon_interval = AP_BEACON_TU;
    if (strlen(CFG_AP_PASS) >= 8) {
        strncpy((char *)ap.ap.password, CFG_AP_PASS, sizeof(ap.ap.password) - 1);
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else ap.ap.authmode = WIFI_AUTH_OPEN;
    esp_wifi_set_config(WIFI_IF_AP, &ap);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_evt, NULL, NULL);
    http_start();
}

/* ===================== lifecycle ========================================== */
void mesh_start(mesh_cmd_cb_t on_cmd, mesh_roster_cb_t on_roster,
                mesh_ack_cb_t on_ack, mesh_router_found_cb_t on_router,
                mesh_relay_state_cb_t get_relay) {
    if (s_running) return;
    if (!s_have_group) { ESP_LOGW(M, "No group set — mesh not started"); return; }

    s_on_cmd = on_cmd;
    s_on_roster = on_roster;
    s_on_ack = on_ack;
    s_on_router = on_router;
    s_get_relay = get_relay;

    ensure_mac();

    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        s_score_lock = xSemaphoreCreateMutex();
        s_seq_lock = xSemaphoreCreateMutex();
        s_nodes_lock = xSemaphoreCreateMutex();
        s_pend_lock = xSemaphoreCreateMutex();
        s_dedup_lock = xSemaphoreCreateMutex();
    }
    memset(s_nodes, 0, sizeof(s_nodes));
    memset(s_pending, 0, sizeof(s_pending));
    memset(s_dedup, 0, sizeof(s_dedup));
    s_dedup_idx = 0;
    s_seq = esp_random() & 0xFFFF;

    s_recv_q  = xQueueCreate(RECV_QUEUE_DEPTH, sizeof(rx_item_t));
    s_relay_q = xQueueCreate(RELAY_QUEUE_DEPTH, sizeof(mesh_msg_t));
    if (!s_recv_q || !s_relay_q) { ESP_LOGE(M, "Queue alloc failed"); return; }

    /* Modem sleep parks the radio between beacons; ESP-NOW frames arriving
     * while it sleeps are simply lost. Mains powered, so no downside. */
    esp_wifi_get_ps(&s_saved_ps);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(84);   /* 21 dBm — helps the edge nodes */

    /* The mesh owns the radio from here. EVERY node starts unassociated in
     * 003 ISLAND and the election decides which one takes the router and the
     * MQTT uplink. A node that kept its association would never run the 003
     * search, which is exactly the fault this replaces. */
    s_uplink_on = false;
    s_stop_pending = false;          /* cancels a deferred stop still waiting */
    s_state = MESH_ISLAND;
    s_role  = ROLE_RELAY;
    wifi_manager_set_auto_retry(false);   /* driver stays up — ESP-NOW needs it */
    esp_wifi_disconnect();
    if (s_on_uplink_down) s_on_uplink_down();   /* app stops MQTT */
    esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);

    softap_start();

    if (esp_now_init() != ESP_OK) { ESP_LOGE(M, "esp_now_init failed"); return; }
    esp_now_register_recv_cb(on_recv);
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BCAST, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    s_running = true;
    xTaskCreate(recv_task,  "mesh_rx",    3072, NULL, 5, &s_recv_task);
    xTaskCreate(relay_task, "mesh_rly",   3072, NULL, 5, &s_relay_task);
    xTaskCreate(state_task, "mesh_state", 4096, NULL, 4, &s_state_task);
    xTaskCreate(wdog_task,  "mesh_wdog",  4096, NULL, 6, &s_wdog_task);

    ESP_LOGI(M, "Mesh started as %s on ch %u", s_serial, s_channel);
}

/* ===================== deferred stop =====================================
 * A mesh-off command arrives ON the mesh: s_on_cmd() runs on recv_task, and
 * mesh_stop() deletes recv_task. Called straight from the callback it would
 * delete the running task mid-teardown — no ACK, and the rest of mesh_stop()
 * (the other three tasks, the queues, the STA and MQTT restore) never runs.
 * So the stop is handed to a task of its own, after a delay long enough for
 * the ACK to go out and for this node to relay the command onward. */
static void stop_task(void *a) {
    (void)a;
    vTaskDelay(pdMS_TO_TICKS(s_stop_delay_ms));
    /* Cancelled: a "mesh on" arrived inside the delay window, or something
     * stopped the mesh already. Either way, do nothing. */
    if (!s_stop_pending) { vTaskDelete(NULL); return; }
    ESP_LOGI(M, "[MESH] deferred stop firing");
    s_stop_pending = false;
    mesh_stop();
    vTaskDelete(NULL);
}

void mesh_stop_deferred(uint32_t delay_ms) {
    if (!s_running || s_stop_pending) return;
    s_stop_pending = true;
    s_stop_delay_ms = delay_ms;
    ESP_LOGI(M, "[MESH] mesh off pending — stopping in %u ms", (unsigned)delay_ms);
    if (xTaskCreate(stop_task, "mesh_off", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(M, "[MESH] deferred stop task alloc failed — stopping inline");
        s_stop_pending = false;
        mesh_stop();
    }
}

void mesh_stop(void) {
    if (!s_running) return;
    s_running = false;
    s_stop_pending = false;
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_now_unregister_recv_cb();
    esp_now_del_peer(BCAST);
    esp_now_deinit();

    http_stop();
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_evt);
    esp_wifi_set_mode(WIFI_MODE_STA);

    if (s_recv_task)  { vTaskDelete(s_recv_task);  s_recv_task = NULL; }
    if (s_relay_task) { vTaskDelete(s_relay_task); s_relay_task = NULL; }
    if (s_state_task) { vTaskDelete(s_state_task); s_state_task = NULL; }
    if (s_wdog_task)  { vTaskDelete(s_wdog_task);  s_wdog_task = NULL; }
    if (s_recv_q)  { vQueueDelete(s_recv_q);  s_recv_q = NULL; }
    if (s_relay_q) { vQueueDelete(s_relay_q); s_relay_q = NULL; }

    esp_wifi_set_ps(s_saved_ps);
    s_state = MESH_ISLAND;
    s_role = ROLE_RELAY;

    /* Hand the radio back: every node returns to its own STA + MQTT, which is
     * the whole point of "mesh off". 0 = no channel hint. */
    s_uplink_on = false;
    wifi_manager_set_auto_retry(true);
    if (s_on_uplink_up) s_on_uplink_up(0);
    ESP_LOGI(M, "Mesh stopped");
}
