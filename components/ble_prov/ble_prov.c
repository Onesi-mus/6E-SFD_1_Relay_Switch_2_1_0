// On-demand BLE provisioning. See FIRMWARE_B_REFERENCE.md §2.

#include "ble_prov.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "cJSON.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_PROV";

#define SECRET_PIN "123456"   // change per deployment

// UUIDs must stay identical to the factory firmware so the same app works.
static const ble_uuid128_t svc_uuid    = BLE_UUID128_INIT(0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0xff,0x00,0x00,0x00);
static const ble_uuid128_t notify_uuid = BLE_UUID128_INIT(0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x02,0xff,0x00,0x00);
static const ble_uuid128_t recv_uuid   = BLE_UUID128_INIT(0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x01,0xff,0x00,0x00);

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_notify_val_handle;
static ble_prov_cb_t s_prov_cb = NULL;
static bool s_authenticated = false;      // cleared on disconnect
static bool s_active = false;             // NimBLE up (provisioning window open)

// Forward declarations
static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_advertising(void);
static void process_json(const char *json_str);
static int  cb_recv_write(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
static int  cb_notify(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int  cb_dsc_read(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg);

//
// GATT TABLE
//
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {   // write: PIN auth + credentials
                .uuid = &recv_uuid.u,
                .access_cb = cb_recv_write,
                .flags = BLE_GATT_CHR_F_WRITE,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    { .uuid = BLE_UUID16_DECLARE(0x2901), .att_flags = BLE_ATT_F_READ,
                      .access_cb = cb_dsc_read, .arg = "Receive Data (PIN/Wi-Fi)" },
                    {0}
                },
            },
            {   // notify: JSON status
                .uuid = &notify_uuid.u,
                .access_cb = cb_notify,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_notify_val_handle,
                .descriptors = (struct ble_gatt_dsc_def[]){
                    { .uuid = BLE_UUID16_DECLARE(0x2901), .att_flags = BLE_ATT_F_READ,
                      .access_cb = cb_dsc_read, .arg = "Send Status" },
                    {0}
                },
            },
            {0}
        }
    },
    {0}
};

static int cb_notify(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int cb_dsc_read(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const char *desc = (const char *)arg;
    return os_mbuf_append(ctxt->om, desc, strlen(desc)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int cb_recv_write(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || ctxt->om == NULL) {
        return 0;
    }
    uint16_t total_len = OS_MBUF_PKTLEN(ctxt->om);
    if (total_len == 0) {
        return 0;
    }
    char *buf = malloc(total_len + 1);
    if (!buf) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, total_len, NULL) == 0) {
        buf[total_len] = '\0';
        ESP_LOGI(TAG, "Write (%d bytes): %s", total_len, buf);
        process_json(buf);
    }
    free(buf);
    return 0;
}

//
// JSON COMMAND HANDLING
//
static void process_json(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "Invalid JSON received");
        ble_prov_send_status("{\"status\":\"error\", \"reason\":\"Invalid JSON\"}");
        return;
    }

    cJSON *pin = cJSON_GetObjectItem(root, "pin");
    if (pin) {
        if (cJSON_IsString(pin) && strcmp(pin->valuestring, SECRET_PIN) == 0) {
            s_authenticated = true;
            ESP_LOGI(TAG, "BLE client authenticated");
            ble_prov_send_status("{\"status\":\"ok\", \"reason\":\"Authenticated\"}");
        } else {
            ESP_LOGE(TAG, "BLE authentication failed, invalid PIN, disconnecting client...");
            ble_prov_send_status("{\"status\":\"error\", \"reason\":\"Invalid PIN\"}");
            ble_gap_terminate(s_conn_handle, BLE_ERR_AUTH_FAIL);
        }
        cJSON_Delete(root);
        return;
    }

    if (!s_authenticated) {
        ESP_LOGE(TAG, "APP sending json before authentication");
        ble_prov_send_status("{\"status\":\"error\", \"reason\":\"Not authenticated\"}");
        cJSON_Delete(root);
        return;
    }

    cJSON *ssid = cJSON_GetObjectItem(root, "ssid");
    cJSON *pw   = cJSON_GetObjectItem(root, "password");
    if (cJSON_IsString(ssid) && cJSON_IsString(pw)) {
        ESP_LOGI(TAG, "Wi-Fi credentials received via BLE");
        ble_prov_send_status("{\"status\":\"ok\", \"reason\":\"Credentials received\"}");
        if (s_prov_cb) {
            s_prov_cb(ssid->valuestring, pw->valuestring);
        }
    } else {
        ESP_LOGE(TAG, "Invalid json for creds...");
        ble_prov_send_status("{\"status\":\"error\", \"reason\":\"Missing credentials\"}");
    }
    cJSON_Delete(root);
}

//
// GAP / ADVERTISING
//
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE connected handle : %d", event->connect.conn_handle);
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);
            } else {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE disconnected, reason : %d", event->disconnect.reason);
            s_authenticated = false;
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            if (s_active) start_advertising();   // stay open until the window closes
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (s_active) start_advertising();
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU: %d", event->mtu.value);
            break;

        default:
            break;
    }
    return 0;
}

static void start_advertising(void) {
    uint8_t mac[6];
    char name[25];   // "Sig{base-MAC}" — same format as the factory firmware
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(name, sizeof(name), "Sig%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ble_svc_gap_device_name_set(name);

    // Main packet carries the service UUID (the app scans on it); name goes in
    // the scan response as both don't fit in one packet.
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_hs_adv_fields rsp = {0};
    const char *adv_name = ble_svc_gap_device_name();
    rsp.name = (uint8_t *)adv_name;
    rsp.name_len = strlen(adv_name);
    rsp.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as %s", adv_name);
    }
}

//
// NIMBLE HOST
//
static void on_sync_cb(void) {
    ble_hs_util_ensure_addr(0);
    start_advertising();
}

static void host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();               // returns only on nimble_port_stop()
    nimble_port_freertos_deinit();
}

//
// PUBLIC API
//
void ble_prov_enter(ble_prov_cb_t cb) {
    if (s_active) return;
    s_prov_cb = cb;
    s_authenticated = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

    nimble_port_init();
    ble_att_set_preferred_mtu(512);
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb = on_sync_cb;              // advertises once the host syncs
    s_active = true;
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE provisioning ENTERED (advertising)");
}

void ble_prov_exit(void) {
    if (!s_active) return;
    s_active = false;                             // stops re-advertise on disconnect
    ble_gap_adv_stop();
    int rc = nimble_port_stop();                  // unblocks host_task
    if (rc == 0) {
        nimble_port_deinit();                     // frees ~40 KB
    } else {
        ESP_LOGW(TAG, "nimble_port_stop rc=%d", rc);
    }
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_authenticated = false;
    ESP_LOGI(TAG, "BLE provisioning EXITED (NimBLE de-inited, heap freed)");
}

void ble_prov_send_status(const char *json) {
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(json, strlen(json));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_notify_val_handle, om);
    }
}
