/*
-SECTIONS
-PUBLIC API
*/



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "cJSON.h"


#include "mqtt_app_handler.h"
#include "app_config.h"
#include "relay_controls.h"
#include "storage_handler.h"
#include "ota.h"
#include "energy_meter.h"
#include "scheduler.h"
#include "switch_input.h"
#include "mesh.h"

//Internal states and declaration
static esp_mqtt_client_handle_t s_client = NULL;
static const char *s_serial_number = NULL;
static sig_r_s_t s_relay_state;
static bool s_state_seeded = false;   //true once app_main has filled s_relay_state at boot
static char topic_buffer_sub[64];

//MQTT topic
#define TOPIC_SIG_PUB   "sig/Sigtron%s/sub"
#define TOPIC_SIG_SUB   "sig/Sigtron%s/pub"
//Retained schedule slot wildcard: sig/Sigtron{serial}/schedule/{1..5} and
//.../schedule/once/{1..3}. Backend pushes each schedule to its own slot topic.
#define TOPIC_SIG_SCHEDULE_SUB "sig/Sigtron%s/schedule/#"
//Mesh. Contract: MESH_BACKEND_CONTRACT.md
#define TOPIC_SIG_OWNER         "sig/Sigtron%s/owner"
#define TOPIC_SIG_MESH_FEEDBACK "sig/Sigtron%s/mesh/feedback"
#define TOPIC_MESH_UPDATE       "sig/update/mesh"
//Unlink. No ACK template on purpose — the ACK topic is the command topic + "/ack".
#define TOPIC_SIG_UNLINK        "sig/Sigtron%s/unlink"
//Common topic for all devices to report state changes to the backend.
#define TOPIC_SIG_SYNC          "sig/device_id/sync"

#define ROSTER_RETRY_MS   30000
#define ROSTER_MAX_BACKOFF_MS 300000
//TEMPORARY: the backend does not ack the roster yet, so send it a fixed number
//of times and stop. Set ROSTER_ACK_REQUIRED back to 1 to restore retry-until-
//acked; the feedback topic and its handler are still in place either way.
#define ROSTER_ACK_REQUIRED 0
#define ROSTER_SENDS        2
#define ROSTER_RESEND_MS    5000


static const char *A = "APP_CONFIG";


//Forward declaration
void mqtt_app_handler_init(esp_mqtt_client_handle_t client, const char* serial_number, const sig_r_s_t *initial_state);
void mqtt_app_handler_process_message(const char* topic, const char* payload);
static void publish_relay_state(void);
static void publish_sig_message(const char *payload);
static void publish_command_feedback(const char *command_id, bool success);
static void publish_sync_state(void);
void mqtt_app_handler_on_connected(void);

//Mesh
static char s_owner_topic[64];
static char s_mesh_fb_topic[80];
static bool s_mesh_created = false;          //"created" on the first roster, then "updated"
static unsigned s_mesh_seq = 0;
static char s_roster_cmd_id[24] = {0};       //in flight, empty = nothing pending
static char *s_roster_payload = NULL;
static TickType_t s_roster_next = 0;
static uint32_t s_roster_backoff = ROSTER_RETRY_MS;
static int s_roster_sends = 0;
static SemaphoreHandle_t s_roster_lock = NULL;
static bool s_cloud_up = false;
static mqtt_router_found_cb_t s_on_router_found = NULL;

//True only while replaying a command that arrived over the MESH. A gateway
//must not re-flood something it was already handed by the mesh.
static volatile bool s_cmd_from_mesh = false;

//How long a node keeps relaying after being told "mesh off", before it
//actually leaves. The gateway waits longer so it is the LAST to go.
#define MESH_OFF_DELAY_MS          4000
#define MESH_OFF_GATEWAY_DELAY_MS  6000

static void handle_owner_message(const char *payload);
static void mesh_apply_enabled(bool enabled);
static void publish_roster(void);
static void roster_task(void *pv);
static void on_mesh_cmd(const char *json);
static void on_mesh_roster_changed(void);
static void on_mesh_ack(const char *serial, const char *command_id, const char *state_json);
static void on_mesh_router_found(uint8_t channel);
static bool mesh_get_relay(void);
static void handle_unlink(const char *topic, const char *payload);
static void publish_mesh_stat(void);
static bool s_unlink_in_progress = false;



//
//PUBLIC API
//
//Function to initialize the mqtt_app_handler
void mqtt_app_handler_init(esp_mqtt_client_handle_t client, const char* serial_number, const sig_r_s_t *initial_state){
    s_client = client;
    s_serial_number = serial_number;
    //Already filled at boot and possibly changed by the rocker since; copying
    //the boot value again here would undo that.
    if (!s_state_seeded) mqtt_app_handler_set_initial_state(initial_state);
    //Initializing the OTA
    char thing_name[48];
    snprintf(thing_name, sizeof(thing_name), "sig%s", s_serial_number);
    ota_init(s_client, thing_name);

    //Mesh comes up only if a group was claimed AND the feature is enabled.
    //Idempotent: on a meshed node app_main has already brought the mesh up
    //before MQTT ever existed, and this call simply re-applies the same state.
    mqtt_app_handler_mesh_refresh();

    ESP_LOGI(A, "App handler initialized...");
}

bool mqtt_app_handler_get_relay_state(void){
    return s_relay_state.relay_on;
}

void mqtt_app_handler_set_initial_state(const sig_r_s_t *state){
    memcpy(&s_relay_state, state, sizeof(sig_r_s_t));
    s_state_seeded = true;
}

//Function to handle the incoming messages
static bool topic_ends_with(const char *topic, const char *suffix){
    size_t lt = strlen(topic), ls = strlen(suffix);
    return lt >= ls && strcmp(topic + lt - ls, suffix) == 0;
}

void mqtt_app_handler_process_message(const char* topic, const char* payload){
    //Unlink first, before any other routing. Suffix match, not strstr — the
    //latter would also match "/unlink/ack".
    if (topic != NULL && topic_ends_with(topic, "/unlink")){
        handle_unlink(topic, payload);
        return;
    }

    //Schedule slot messages (retained, backend->device) are routed by TOPIC,
    //before any JSON parse (an empty tombstone payload won't parse). Replayed
    //schedule commands come back here with topic == NULL and fall through to the
    //normal command handling below.
    if (topic != NULL && strstr(topic, "/schedule/") != NULL){
        scheduler_handle_slot_message(topic, payload);
        return;
    }

    //OTA job topics.
    if (topic != NULL && check_job_topic(topic)){
        ESP_LOGI(A, "Received job topic with payload : %s", payload);
        ota_process_message(topic, payload);
        return;
    }

    //Claim / unclaim. Retained, so this also arrives on every reconnect.
    if (topic != NULL && s_owner_topic[0] && strcmp(topic, s_owner_topic) == 0){
        handle_owner_message(payload);
        return;
    }

    //Roster ack. Matched on commandId only.
    if (topic != NULL && s_mesh_fb_topic[0] && strcmp(topic, s_mesh_fb_topic) == 0){
        cJSON *fb = cJSON_Parse(payload);
        if (fb){
            cJSON *cid = cJSON_GetObjectItem(fb, "commandId");
            if (cJSON_IsString(cid)){
                xSemaphoreTake(s_roster_lock, portMAX_DELAY);
                if (s_roster_cmd_id[0] && strcmp(cid->valuestring, s_roster_cmd_id) == 0){
                    ESP_LOGI(A, "Roster acked : %s", s_roster_cmd_id);
                    s_roster_cmd_id[0] = '\0';
                    free(s_roster_payload);
                    s_roster_payload = NULL;
                    s_roster_backoff = ROSTER_RETRY_MS;
                }
                xSemaphoreGive(s_roster_lock);
            }
            cJSON_Delete(fb);
        }
        return;
    }

    //Everything past here is a relay command: either from the live command topic
    //or replayed by the scheduler (topic == NULL). Any other topic is ignored.
    if (topic != NULL && strcmp(topic, topic_buffer_sub) != 0){
        return;
    }

    ESP_LOGI(A, "Processing command payload : %s", payload);
    cJSON *root = cJSON_Parse(payload);
    if (!root){
        ESP_LOGE(A, "Failed to parse the payload");
        return;
    }

    //Addressed elsewhere -> hand it to the mesh. No targetMac means it is for us
    //(schedule replays, older app builds).
    bool already_flooded = false;
    cJSON *tmac = cJSON_GetObjectItem(root, "targetMac");
    if (cJSON_IsString(tmac)){
        char self[MESH_SERIAL_LEN];
        mesh_own_serial(self);
        bool is_all = (strcmp(tmac->valuestring, "all") == 0);

        if (is_all){
            //Strip targetMac before flooding. A receiver that still saw "all"
            //would re-flood it, and since every re-flood gets a fresh seq the
            //dedup cache cannot stop the storm.
            if (mesh_is_running()){
                cJSON_DeleteItemFromObject(root, "targetMac");
                char *flood = cJSON_PrintUnformatted(root);
                if (flood){
                    mesh_send_command(NULL, NULL, flood);
                    free(flood);
                    already_flooded = true;
                }
            }
            //fall through: this device applies it too
        } else if (strcmp(tmac->valuestring, self) != 0){
            cJSON *jid = cJSON_GetObjectItem(root, "commandId");
            bool ok = mesh_is_running() &&
                      mesh_send_command(tmac->valuestring,
                                        cJSON_IsString(jid) ? jid->valuestring : NULL,
                                        payload);
            if (!ok && cJSON_IsString(jid))
                publish_command_feedback(jid->valuestring, false);
            cJSON_Delete(root);
            return;
        }
    }

    cJSON *type = cJSON_GetObjectItem(root, "type");
    if (!cJSON_IsString(type)){
        ESP_LOGE(A, "Failed to get type / wrong json...");
        cJSON_Delete(root);
        return;
    }

    //Settings commands carry no "state", so handle them before the relay path.
    if (strcmp(type->valuestring, "switch_type") == 0){
        cJSON *mode = cJSON_GetObjectItem(root, "mode");
        if (cJSON_IsString(mode)){
            rocker_mode_t m = ROCKER_MODE_TOGGLE;              // default
            if (strcmp(mode->valuestring, "follow") == 0)      m = ROCKER_MODE_FOLLOW;
            else if (strcmp(mode->valuestring, "momentary") == 0) m = ROCKER_MODE_MOMENTARY;
            else if (strcmp(mode->valuestring, "toggle") == 0) m = ROCKER_MODE_TOGGLE;
            switch_input_set_mode(m);
            ESP_LOGI(A, "Switch type -> %s", mode->valuestring);
        } else {
            ESP_LOGE(A, "switch_type missing mode");
        }
        cJSON_Delete(root);
        return;
    }

    //Affects the next power-up only; does not change the relay now.
    if (strcmp(type->valuestring, "relay_status") == 0){
        cJSON *mode = cJSON_GetObjectItem(root, "mode");
        if (cJSON_IsString(mode)){
            poweron_mode_t pm = POWERON_MEMORY;
            if (strcmp(mode->valuestring, "off") == 0)       pm = POWERON_OFF;
            else if (strcmp(mode->valuestring, "on") == 0)   pm = POWERON_ON;
            else if (strcmp(mode->valuestring, "memory") == 0) pm = POWERON_MEMORY;
            storage_handler_save_poweron_mode(pm);
            ESP_LOGI(A, "Relay status (power-on) -> %s", mode->valuestring);
        } else {
            ESP_LOGE(A, "relay_status missing mode");
        }
        cJSON_Delete(root);
        return;
    }

    //Mesh on/off. Persisted; takes effect immediately.
    if (strcmp(type->valuestring, "mesh") == 0){
        cJSON *st  = cJSON_GetObjectItem(root, "state");
        cJSON *jid = cJSON_GetObjectItem(root, "commandId");
        bool ok = false;
        if (cJSON_IsString(st)){
            bool on = (strcmp(st->valuestring, "on") == 0);
            storage_handler_save_mesh_enabled(on);

            if (!on && mesh_is_running()){
                //The backend publishes "mesh off" to every device, but only the
                //gateway holds an MQTT subscription - the relays are
                //unassociated and never see it. So a gateway passes it on to
                //the whole mesh first, and leaves LAST: while it is still up it
                //keeps relaying, and each relay ACKs and re-broadcasts before
                //its own deferred stop fires.
                if (!s_cmd_from_mesh && !already_flooded && mesh_is_gateway()){
                    char *flood = cJSON_PrintUnformatted(root);
                    if (flood){
                        mesh_send_command(NULL, NULL, flood);
                        ESP_LOGI(A, "Mesh off -> flooded to the mesh, leaving last");
                        free(flood);
                    }
                    mesh_stop_deferred(MESH_OFF_GATEWAY_DELAY_MS);
                } else {
                    mesh_stop_deferred(MESH_OFF_DELAY_MS);
                }
            } else {
                mesh_apply_enabled(on);
            }
            ok = true;
        } else {
            ESP_LOGE(A, "mesh command missing state");
        }
        if (cJSON_IsString(jid)) publish_command_feedback(jid->valuestring, ok);
        cJSON_Delete(root);
        return;
    }

    //Link diagnostics on demand. There is no serial console in the field, so
    //this is how RSSI and frame counts get out.
    if (strcmp(type->valuestring, "mesh_stat") == 0){
        publish_mesh_stat();
        cJSON *jid = cJSON_GetObjectItem(root, "commandId");
        if (cJSON_IsString(jid)) publish_command_feedback(jid->valuestring, true);
        cJSON_Delete(root);
        return;
    }

    cJSON *state = cJSON_GetObjectItem(root, "state");
    if (!cJSON_IsString(state)){
        ESP_LOGE(A, "Failed to get state / wrong json...");
        cJSON_Delete(root);
        return;
    }

    //Optional commandId — echoed back as feedback once the command is applied.
    //Absent for schedule replays and older app builds; then command_id stays
    //NULL and no feedback is sent. Points into root, valid until cJSON_Delete.
    cJSON *jid = cJSON_GetObjectItem(root, "commandId");
    const char *command_id = cJSON_IsString(jid) ? jid->valuestring : NULL;

    bool turn_on = (strcmp(state->valuestring, "on") == 0);
    bool state_changed = false;
    esp_err_t exec_err = ESP_OK;

    //Backend speaks the 2-gang protocol, so accept "relay1"/"overall" too.
    if (strcmp(type->valuestring, "relay1") == 0 ||
        strcmp(type->valuestring, "overall") == 0 ||
        strcmp(type->valuestring, "relay") == 0){
        s_relay_state.relay_on = turn_on;
        exec_err = relay_set(turn_on);
        energy_meter_relay_changed(turn_on);
        state_changed = true;
    }

    if (state_changed){
        storage_handler_save_relay_state(&s_relay_state);
        publish_relay_state();
        //Report the command outcome to the app, echoing its commandId (if any).
        publish_command_feedback(command_id, exec_err == ESP_OK);
        //Rocker changes are also reported on the common sync topic.
        if (command_id && strncmp(command_id, "rocker-", 7) == 0) publish_sync_state();
    }
    cJSON_Delete(root);
}

//Function to give relay feedback
static void publish_relay_state(void){
    if (!s_client){
        ESP_LOGE(A, "No mqtt client...");
        return;
    }
    cJSON *root = cJSON_CreateObject();
    //"relay1" is what the app reads; "relay" kept for legacy consumers.
    cJSON_AddBoolToObject(root, "relay1", s_relay_state.relay_on);
    cJSON_AddBoolToObject(root, "relay", s_relay_state.relay_on);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json){
        publish_sig_message(json);
        free(json);
    }
}

//Function to publish
static void publish_sig_message(const char *payload){
    if (!s_client){
        ESP_LOGE(A, "MQTT clientt not exist...");
        return;
    }
    char topic[64];
    snprintf(topic, sizeof(topic), TOPIC_SIG_PUB, s_serial_number);
    esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 0);
    ESP_LOGI(A, "Message : %s , published to : %s", payload, topic);
}

//Function to report a command's outcome back to the app, echoing its commandId.
//Published to the same feedback topic (TOPIC_SIG_PUB) at QoS 0 (advisory status,
//not a command — losing an occasional one is harmless). No feedback is sent for
//commands without a commandId (schedule replays, non-command traffic).
static void publish_command_feedback(const char *command_id, bool success){
    if (!s_client || command_id == NULL) return;
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "commandId", command_id);
    cJSON_AddStringToObject(root, "status", success ? "SUCCESS" : "FAILED");
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;
    char topic[64];
    snprintf(topic, sizeof(topic), TOPIC_SIG_PUB, s_serial_number);
    esp_mqtt_client_publish(s_client, topic, json, 0, 0, 0);
    ESP_LOGI(A, "Feedback -> %s : %s", topic, json);
    free(json);
}

//Report the full relay1 state on the common sync topic.
static void publish_sync_state(void){
    if (!s_client || !s_serial_number) return;

    poweron_mode_t pm = storage_handler_load_poweron_mode();
    const char *pm_name = (pm == POWERON_OFF) ? "off" : (pm == POWERON_ON) ? "on" : "memory";
    rocker_mode_t rm = switch_input_get_mode();
    const char *rm_name = (rm == ROCKER_MODE_FOLLOW) ? "follow" :
                          (rm == ROCKER_MODE_MOMENTARY) ? "momentary" : "toggle";

    char serial[32];
    snprintf(serial, sizeof(serial), "Sigtron%s", s_serial_number);

    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON *devices = cJSON_AddArrayToObject(root, "devices");
    cJSON *dev = cJSON_CreateObject();
    cJSON_AddItemToArray(devices, dev);
    cJSON_AddStringToObject(dev, "serialNo", serial);
    cJSON *state = cJSON_AddObjectToObject(dev, "state");
    cJSON *relay1 = cJSON_AddObjectToObject(state, "relay1");
    cJSON_AddStringToObject(relay1, "state", s_relay_state.relay_on ? "on" : "off");
    cJSON_AddStringToObject(relay1, "powerOnState", pm_name);
    cJSON_AddStringToObject(relay1, "switchType", rm_name);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;
    esp_mqtt_client_publish(s_client, TOPIC_SIG_SYNC, json, 0, 1, 0);
    ESP_LOGI(A, "Sync -> %s : %s", TOPIC_SIG_SYNC, json);
    free(json);
}

//
//MESH
//
//Claim carries a userId; an empty payload is the unclaim tombstone.
static void handle_owner_message(const char *payload){
    if (payload == NULL || payload[0] == '\0'){
        ESP_LOGW(A, "Unclaimed — clearing mesh group");
        storage_handler_save_mesh_group("");
        mesh_set_group(NULL);
        mesh_stop();
        return;
    }
    cJSON *root = cJSON_Parse(payload);
    if (!root){ ESP_LOGE(A, "owner: bad json"); return; }

    //groupId wins if the backend sends one; otherwise the userId is the group.
    cJSON *g = cJSON_GetObjectItem(root, "groupId");
    if (!cJSON_IsString(g)) g = cJSON_GetObjectItem(root, "userId");

    if (cJSON_IsString(g) && g->valuestring[0]){
        char saved[64] = {0};
        bool same = storage_handler_load_mesh_group(saved, sizeof(saved)) &&
                    strcmp(saved, g->valuestring) == 0;
        if (!same){
            storage_handler_save_mesh_group(g->valuestring);
            ESP_LOGI(A, "Claimed — mesh group '%s'", g->valuestring);
        }
        mesh_set_group(g->valuestring);
        mesh_apply_enabled(storage_handler_load_mesh_enabled());
    } else {
        ESP_LOGE(A, "owner: no userId/groupId");
    }
    cJSON_Delete(root);
}

//Single place that decides whether the mesh should be up. Reachable with no
//MQTT present: a meshed relay never connects to the cloud at all, so gating
//this on a successful MQTT connect would leave a rebooted relay stranded.
static void mesh_apply_enabled(bool enabled){
    if (enabled && mesh_has_group()){
        if (!mesh_is_running())
            mesh_start(on_mesh_cmd, on_mesh_roster_changed, on_mesh_ack,
                       on_mesh_router_found, mesh_get_relay);
        mesh_set_cloud(s_cloud_up);
    } else {
        mesh_stop();
    }
}

//A command arrived over the mesh for this device. Replay it through the normal
//path so state, NVS, energy and feedback all behave identically.
static void on_mesh_cmd(const char *json){
    ESP_LOGI(A, "Mesh command : %s", json);
    s_cmd_from_mesh = true;
    mqtt_app_handler_process_message(NULL, json);
    s_cmd_from_mesh = false;
}

static void on_mesh_roster_changed(void){
    if (mesh_is_gateway()) publish_roster();
}

//Reported back inside a mesh ACK so the gateway can publish the node's state.
static bool mesh_get_relay(void){
    return s_relay_state.relay_on;
}

//The 003 WiFi phase found the router. Firmware B owns the STA, so the app
//re-associates; the mesh only reports.
static void on_mesh_router_found(uint8_t channel){
    ESP_LOGI(A, "Mesh found the router on ch %u — requesting reconnect", channel);
    if (s_on_router_found) s_on_router_found(channel);
}

//A forwarded command was acked. Report it as the originating command's feedback
//and republish the node's state on this gateway's own topic.
static void on_mesh_ack(const char *serial, const char *command_id, const char *state_json){
    if (command_id) publish_command_feedback(command_id, true);
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "node", serial);
    if (state_json && state_json[0]) cJSON_AddStringToObject(root, "state", state_json);
    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (js){ publish_sig_message(js); free(js); }
}

//Everything the serial [STAT] block used to print, published instead.
static void publish_mesh_stat(void){
    if (!s_client) return;
    char self[MESH_SERIAL_LEN];
    mesh_own_serial(self);

    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "serialNo", self);
    cJSON_AddBoolToObject(root, "meshRunning", mesh_is_running());
    cJSON_AddNumberToObject(root, "state", mesh_state());
    cJSON_AddStringToObject(root, "role", mesh_is_gateway() ? "gateway" : "relay");
    cJSON_AddNumberToObject(root, "channel", mesh_channel());
    cJSON_AddNumberToObject(root, "score", mesh_score());
    cJSON_AddNumberToObject(root, "freeHeap", (double)esp_get_free_heap_size());

    mesh_node_info_t info[MESH_MAX_NODES];
    int n = mesh_node_info(info, MESH_MAX_NODES);
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++){
        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "serialNo", info[i].serial);
        cJSON_AddNumberToObject(o, "rssi", info[i].rssi);
        cJSON_AddNumberToObject(o, "minRssi", info[i].min_rssi);
        cJSON_AddNumberToObject(o, "rxTotal", (double)info[i].rx_total);
        cJSON_AddNumberToObject(o, "silentMs", (double)info[i].silent_ms);
        cJSON_AddItemToArray(arr, o);
    }
    cJSON_AddItemToObject(root, "nodes", arr);

    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (js){ publish_sig_message(js); free(js); }
}

//Builds the roster and arms the retry. cJSON sizes the buffer, so a large mesh
//can't be silently truncated.
static void publish_roster(void){
    if (!s_client || !mesh_is_gateway()) return;

    char nodes[MESH_MAX_NODES][MESH_SERIAL_LEN];
    int n = mesh_roster(nodes, MESH_MAX_NODES);
    char self[MESH_SERIAL_LEN];
    mesh_own_serial(self);

    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    char cid[24];
    snprintf(cid, sizeof(cid), "mesh-%u", ++s_mesh_seq);
    cJSON_AddStringToObject(root, "commandId", cid);
    cJSON_AddStringToObject(root, "meshStatus", s_mesh_created ? "updated" : "created");
    cJSON_AddStringToObject(root, "gatewaySerialNo", self);
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++) cJSON_AddItemToArray(arr, cJSON_CreateString(nodes[i]));
    cJSON_AddItemToObject(root, "nodeSerialNos", arr);
    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!js) return;

    //A newer roster supersedes whatever was in flight.
    xSemaphoreTake(s_roster_lock, portMAX_DELAY);
    free(s_roster_payload);
    s_roster_payload = js;
    strncpy(s_roster_cmd_id, cid, sizeof(s_roster_cmd_id) - 1);
    s_roster_cmd_id[sizeof(s_roster_cmd_id) - 1] = '\0';
    s_roster_backoff = ROSTER_ACK_REQUIRED ? ROSTER_RETRY_MS : ROSTER_RESEND_MS;
    s_roster_sends = 1;
    s_roster_next = xTaskGetTickCount() + pdMS_TO_TICKS(s_roster_backoff);
    xSemaphoreGive(s_roster_lock);

    s_mesh_created = true;
    esp_mqtt_client_publish(s_client, TOPIC_MESH_UPDATE, js, 0, 1, 0);
    ESP_LOGI(A, "Roster -> %s : %s", TOPIC_MESH_UPDATE, js);
}

//Republishes the pending roster until the backend acks its commandId.
static void roster_task(void *pv){
    while (1){
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!s_client) continue;
        char *copy = NULL;
        xSemaphoreTake(s_roster_lock, portMAX_DELAY);
        bool done = false;
        if (s_roster_cmd_id[0] && s_roster_payload &&
            xTaskGetTickCount() >= s_roster_next){
            copy = strdup(s_roster_payload);
            s_roster_sends++;
            if (!ROSTER_ACK_REQUIRED && s_roster_sends >= ROSTER_SENDS){
                //Sent our quota — drop it rather than waiting for an ack.
                s_roster_cmd_id[0] = '\0';
                free(s_roster_payload);
                s_roster_payload = NULL;
                done = true;
            } else {
                if (s_roster_backoff < ROSTER_MAX_BACKOFF_MS) s_roster_backoff *= 2;
                s_roster_next = xTaskGetTickCount() + pdMS_TO_TICKS(s_roster_backoff);
            }
        }
        xSemaphoreGive(s_roster_lock);
        if (copy){
            esp_mqtt_client_publish(s_client, TOPIC_MESH_UPDATE, copy, 0, 1, 0);
            ESP_LOGI(A, "Roster resend %d/%d%s : %s", s_roster_sends, ROSTER_SENDS,
                     done ? " (done)" : "", copy);
            free(copy);
        }
    }
}

//
//UNLINK
//
//ACK goes to the topic the command arrived on + "/ack", so command and reply
//can never drift apart.
static void publish_unlink_ack(const char *cmd_topic, const char *request_id, bool ok){
    if (!s_client) return;
    char topic[160];
    snprintf(topic, sizeof(topic), "%s/ack", cmd_topic);

    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "request_id", request_id);
    cJSON_AddStringToObject(root, "status", ok ? "success" : "failed");
    if (!ok) cJSON_AddStringToObject(root, "error", "UNLINK_FAILED");
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    int msg_id = esp_mqtt_client_publish(s_client, topic, json, 0, 1, 0);
    free(json);
    if (msg_id < 0) ESP_LOGE(A, "Unlink ACK publish REJECTED by the MQTT client.");
    else            ESP_LOGI(A, "Unlink ACK: %s (msg_id=%d)", ok ? "success" : "failed", msg_id);
}

//Erasing NVS does not drop the live MQTT session, so the ACK still goes out.
//A restart does — hence erase, ACK, flush, then restart.
static void unlink_restart_task(void *pv){
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

static void handle_unlink(const char *topic, const char *payload){
    if (s_unlink_in_progress){
        ESP_LOGW(A, "Unlink already in progress; ignoring.");
        return;
    }
    cJSON *root = cJSON_Parse(payload);
    if (!root) return;

    cJSON *jid  = cJSON_GetObjectItem(root, "request_id");
    cJSON *jcmd = cJSON_GetObjectItem(root, "command");
    if (!cJSON_IsString(jid) || !cJSON_IsString(jcmd) ||
        strcmp(jcmd->valuestring, "unlink") != 0){
        ESP_LOGW(A, "Malformed unlink; dropped without ACK.");
        cJSON_Delete(root);
        return;
    }
    char request_id[64];
    strncpy(request_id, jid->valuestring, sizeof(request_id) - 1);
    request_id[sizeof(request_id) - 1] = '\0';
    cJSON_Delete(root);

    s_unlink_in_progress = true;
    ESP_LOGW(A, "UNLINK requested (%s)", request_id);

    //Visible reaction, and it leaves the load in a safe state for the next owner.
    s_relay_state.relay_on = false;
    relay_set(false);
    energy_meter_relay_changed(false);

    mesh_stop();
    bool ok = storage_handler_unlink();
    publish_unlink_ack(topic, request_id, ok);

    if (ok) xTaskCreate(unlink_restart_task, "unlink_restart", 3072, NULL, 5, NULL);
    else    s_unlink_in_progress = false;   //stay linked; let the app retry
}

//Function to run after mqtt connection
void mqtt_app_handler_on_connected(void){
    ESP_LOGI(A, "APP Handler : MQTT connected");
    snprintf(topic_buffer_sub, sizeof(topic_buffer_sub), TOPIC_SIG_SUB, s_serial_number);

    esp_mqtt_client_subscribe(s_client, topic_buffer_sub, 1);
    ESP_LOGI(A, "Successfully subscribed to : %s", topic_buffer_sub);

    //Subscribe to the retained schedule slot wildcard. Retained delivery replays
    //all currently-stored schedule slots on this subscribe.
    char topic_sched[80];
    snprintf(topic_sched, sizeof(topic_sched), TOPIC_SIG_SCHEDULE_SUB, s_serial_number);
    //Clear first, then subscribe (EMBEDDED-INTEGRATION.md §3 step 1).
    scheduler_clear_all();
    esp_mqtt_client_subscribe(s_client, topic_sched, 1);
    ESP_LOGI(A, "Subscribed to schedule wildcard : %s", topic_sched);

    //Claim state. Retained, so the current owner arrives right after subscribe.
    snprintf(s_owner_topic, sizeof(s_owner_topic), TOPIC_SIG_OWNER, s_serial_number);
    esp_mqtt_client_subscribe(s_client, s_owner_topic, 1);
    ESP_LOGI(A, "Subscribed to owner : %s", s_owner_topic);

    snprintf(s_mesh_fb_topic, sizeof(s_mesh_fb_topic), TOPIC_SIG_MESH_FEEDBACK, s_serial_number);
    esp_mqtt_client_subscribe(s_client, s_mesh_fb_topic, 1);

    char topic_unlink[96];
    snprintf(topic_unlink, sizeof(topic_unlink), TOPIC_SIG_UNLINK, s_serial_number);
    esp_mqtt_client_subscribe(s_client, topic_unlink, 1);
    ESP_LOGI(A, "Subscribed to unlink : %s", topic_unlink);

    ota_handler_on_connected();

    {
        char hello[128];
        snprintf(hello, sizeof(hello),
                 "{\"status\":\"online_and_ready\",\"mesh\":\"%s\",\"meshState\":%u}",
                 mesh_is_running() ? "on" : "off", mesh_state());
        publish_sig_message(hello);
    }

    //Report the current relay state on every (re)connect, so a rocker flip made
    //while offline reaches the app.
    publish_relay_state();

    //Start the energy meter task. Publishes V/I/P to sig/Sigtron%s/energy
    energy_meter_start(s_client, s_serial_number, s_relay_state.relay_on);

    //Cloud is up: this node is now a gateway candidate.
    s_cloud_up = true;
    mesh_set_cloud(true);
    if (mesh_is_gateway()) publish_roster();
}

void mqtt_app_handler_mesh_refresh(void){
    if (!s_roster_lock){
        s_roster_lock = xSemaphoreCreateMutex();
        xTaskCreate(roster_task, "roster", 3072, NULL, 3, NULL);
    }
    char group[64] = {0};
    if (storage_handler_load_mesh_group(group, sizeof(group))) mesh_set_group(group);
    mesh_set_home_channel(storage_handler_load_home_channel());
    mesh_apply_enabled(storage_handler_load_mesh_enabled());
}

void mqtt_app_handler_set_router_found_cb(mqtt_router_found_cb_t cb){
    s_on_router_found = cb;
}

void mqtt_app_handler_on_disconnected(void){
    s_cloud_up = false;
    mesh_set_cloud(false);
}
