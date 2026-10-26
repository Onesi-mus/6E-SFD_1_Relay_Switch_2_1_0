#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"  //add nvs_flash in requires in CMakeLists.txt,    start/initialize the NVS system
#include "esp_log.h"
#include "esp_ota_ops.h"  //add app_updates in requires in CMakeLists.txt
#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "app_config.h"
#include "wifi_manager.h"
#include "storage_handler.h"
#include "relay_controls.h"
#include "aws_mqtt.h"
#include "mqtt_app_handler.h"
#include "ota.h"
#include "energy_meter.h"
#include "scheduler.h"
#include "ble_prov.h"
#include "switch_input.h"
#include "led_indicator.h"
#include "mesh.h"
#include "esp_wifi.h"

// Boot/provisioning/rocker behaviour is documented in FIRMWARE_B_REFERENCE.md.

static const char *APP_TAG = "SIGTRON";

//How long to wait for the cloud after a Wi-Fi switch before calling it a failure.
#define CLOUD_WAIT_MS 30000

//Forward declarations
void start_aws_iot_service(const sig_r_s_t* initial_state);
void initialize_sntp(void);
static void start_time_and_scheduler(void);
static void start_services_once(void);
static void on_ble_credentials_received(const char *ssid, const char *password);
static void on_rocker_apply(bool on);
static void on_rocker_gesture(void);
static void enter_provisioning(bool with_timeout);
static void net_status_task(void *pv);
static void remember_home_channel(void);
static void on_mesh_router_found(uint8_t channel);
static void on_mesh_uplink_up(uint8_t channel);
static void on_mesh_uplink_down(void);
static void uplink_task(void *pv);

//Uplink work is serialised onto its own task: the mesh calls the callbacks
//from its state task, whose 4 KB stack cannot carry a Wi-Fi association or a
//TLS handshake, and which must not block for the 25 s connect timeout.
typedef struct { bool up; uint8_t ch; } uplink_evt_t;
static QueueHandle_t s_uplink_q = NULL;
static void exit_provisioning(void);
static void prov_timeout_cb(void *arg);

static sig_r_s_t s_initial_relay_state;

static bool s_services_started = false;
static volatile bool s_provisioning = false;
static esp_timer_handle_t s_prov_timer = NULL;


void app_main(void){
    //Initialize the storage
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK){
        ESP_LOGE(APP_TAG, "NVS initialization failed %s", esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
        abort();
    }

    storage_handler_mount();
    vTaskDelay(pdMS_TO_TICKS(100));

    //Load saved relay state or default to OFF.
    if (!storage_handler_load_relay_state(&s_initial_relay_state)) {
        ESP_LOGW(APP_TAG, "No saved relay state found. Using default (OFF).");
        s_initial_relay_state.relay_on = false;
        storage_handler_save_relay_state(&s_initial_relay_state);
    }

    //OFF/ON override the saved state; MEMORY keeps it.
    poweron_mode_t pm = storage_handler_load_poweron_mode();
    if (pm == POWERON_OFF)      s_initial_relay_state.relay_on = false;
    else if (pm == POWERON_ON)  s_initial_relay_state.relay_on = true;
    ESP_LOGI(APP_TAG, "Power-on mode: %s", pm == POWERON_OFF ? "off" : pm == POWERON_ON ? "on" : "memory");

    init_relay_module();
    relay_controls_apply_state(&s_initial_relay_state);
    mqtt_app_handler_set_initial_state(&s_initial_relay_state);   //rocker must see the same state

    energy_meter_init();
    wifi_manager_init();

    //BLE is deliberately NOT started here — only inside a provisioning window,
    //so its ~40 KB stays free for the OTA TLS handshake.
    led_indicator_init();
    xTaskCreate(net_status_task, "net_status", 2048, NULL, 2, NULL);
    switch_input_init(mqtt_app_handler_get_relay_state, on_rocker_apply, on_rocker_gesture);

    const esp_timer_create_args_t targs = {
        .callback = prov_timeout_cb,
        .name = "prov_timeout",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_prov_timer));

    char ssid[32];
    char password[64];
    if (!storage_handler_load_wifi_credentials(ssid, sizeof(ssid), password, sizeof(password))){
        ESP_LOGW(APP_TAG, "No stored wifi credentials - starting BLE provisioning.");
        enter_provisioning(false);
        return;
    }

    mesh_set_router_ssid(ssid);
    mqtt_app_handler_set_router_found_cb(on_mesh_router_found);
    s_uplink_q = xQueueCreate(4, sizeof(uplink_evt_t));
    xTaskCreate(uplink_task, "uplink", 6144, NULL, 4, NULL);
    mesh_set_uplink_cbs(on_mesh_uplink_up, on_mesh_uplink_down);

    //MESH FIRST, before any association attempt.
    //With the mesh on, exactly one elected gateway holds the STA and the MQTT
    //uplink; everyone else is an unassociated relay. Starting the mesh only
    //after a successful MQTT connect (the old order) meant a relay that
    //rebooted - or any node out of router range - never started its mesh at
    //all and sat in the reconnect loop forever.
    char mesh_group[64] = {0};
    bool mesh_on = storage_handler_load_mesh_enabled() &&
                   storage_handler_load_mesh_group(mesh_group, sizeof(mesh_group)) &&
                   mesh_group[0] != '\0';
    if (mesh_on){
        ESP_LOGI(APP_TAG, "Mesh enabled (group '%s') - starting unassociated; the election decides who takes the router", mesh_group);
        wifi_manager_start_driver();       //driver + SoftAP up, NO association
        mqtt_app_handler_mesh_refresh();   //loads group + home channel, starts the mesh
        return;
    }

    ESP_LOGI(APP_TAG, "Connecting to wifi with stored credentials......");
    if (wifi_manager_connect(ssid, password)){
        remember_home_channel();
    } else {
        ESP_LOGW(APP_TAG, "Stored credentials failed. Flip the rocker 3x to re-provision over BLE.");
    }
    start_services_once();
}

//The channel we associate on is the channel the mesh lives on. An orphan that
//can no longer reach the router parks here instead of hunting.
static void remember_home_channel(void){
    uint8_t ch = 0;
    wifi_second_chan_t sc;
    if (esp_wifi_get_channel(&ch, &sc) == ESP_OK && ch >= 1 && ch <= 13)
        storage_handler_save_home_channel(ch);
}

//Drives the status LED and keeps the mesh told about the cloud. Radio ownership
//is NOT decided here any more - the mesh owns it while it is running and drives
//wifi_manager's auto-retry itself.
static void net_status_task(void *pv){
    while (1){
        bool cloud = aws_mqtt_is_connected();
        led_indicator_set_network(cloud);
        mesh_set_cloud(cloud);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//The 003 search located the router. INFORMATIONAL ONLY - remember the channel
//so an orphan parks there. Associating here would make whichever node scanned
//first the gateway; only the election's winner associates, via on_mesh_uplink_up.
static void on_mesh_router_found(uint8_t channel){
    ESP_LOGI(APP_TAG, "Mesh found the router on ch %u (election decides who associates)", channel);
    if (channel >= 1 && channel <= 13) storage_handler_save_home_channel(channel);
}

//Serialises the uplink transitions so an association can never run on the
//mesh's own task, and an up and a down can never overlap.
static void uplink_task(void *pv){
    uplink_evt_t e;
    while (1){
        if (xQueueReceive(s_uplink_q, &e, portMAX_DELAY) != pdTRUE) continue;

        if (!e.up){
            //Lost or yielded the gateway role: drop MQTT and release the STA
            //so the mesh owns the radio again.
            ESP_LOGI(APP_TAG, "Uplink DOWN - stopping MQTT and releasing the STA");
            aws_mqtt_stop();
            wifi_manager_set_auto_retry(false);  //also disconnects; driver stays up
            continue;
        }

        //Won the election (or the mesh stopped): take the STA and bring the
        //cloud up. The channel is a hint only (0 = none) - wifi_manager_connect()
        //finds the SSID itself.
        if (s_provisioning) continue;            //BLE window owns the radio
        char ssid[32], password[64];
        if (!storage_handler_load_wifi_credentials(ssid, sizeof(ssid), password, sizeof(password))) continue;
        ESP_LOGI(APP_TAG, "Uplink UP (ch hint %u) - associating to '%s'", e.ch, ssid);
        if (wifi_manager_connect(ssid, password)){
            remember_home_channel();
            if (!aws_mqtt_resume()) start_services_once();
        } else {
            ESP_LOGW(APP_TAG, "Uplink UP: association to '%s' failed", ssid);
        }
    }
}

//Both callbacks are called from the mesh state task - enqueue only, never block.
static void on_mesh_uplink_up(uint8_t channel){
    if (!s_uplink_q) return;
    uplink_evt_t e = { .up = true, .ch = channel };
    xQueueSend(s_uplink_q, &e, 0);
}

static void on_mesh_uplink_down(void){
    if (!s_uplink_q) return;
    uplink_evt_t e = { .up = false, .ch = 0 };
    xQueueSend(s_uplink_q, &e, 0);
}

//MQTT + SNTP + scheduler, brought up once on the first successful Wi-Fi connect.
static void start_services_once(void){
    if (s_services_started) return;
    s_services_started = true;
    start_aws_iot_service(&s_initial_relay_state);
    start_time_and_scheduler();
}

//
// PHYSICAL ROCKER
//
//Routed through the normal command path so state/NVS/energy/publish all happen.
//The "rocker-N" commandId also makes the handler emit feedback, so the app updates.
static void on_rocker_apply(bool on){
    static unsigned rk_seq = 0;
    char cmd[96];
    snprintf(cmd, sizeof(cmd),
             "{\"type\":\"relay1\",\"state\":\"%s\",\"commandId\":\"rocker-%u\"}",
             on ? "on" : "off", ++rk_seq);
    mqtt_app_handler_process_message(NULL, cmd);
}

//NimBLE init allocates ~40 KB and blocks. It must not run on the switch task,
//whose stack is 3 KB, and the mesh has to release its tasks, queues, SoftAP and
//HTTP server first or there may not be a contiguous block big enough.
static void prov_enter_task(void *pv){
    bool with_timeout = (bool)(intptr_t)pv;
    if (mesh_is_running()){
        ESP_LOGI(APP_TAG, "Pausing mesh to free heap for BLE");
        mesh_stop();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    led_indicator_set_provisioning(true);
    ble_prov_enter(on_ble_credentials_received);
    esp_timer_stop(s_prov_timer);
    if (with_timeout) esp_timer_start_once(s_prov_timer, 120ULL * 1000000ULL);
    ESP_LOGI(APP_TAG, "BLE provisioning open (%s), free heap %u",
             with_timeout ? "2 min window" : "no timeout",
             (unsigned)esp_get_free_heap_size());
    vTaskDelete(NULL);
}

//with_timeout=false leaves BLE advertising indefinitely. That is the right
//behaviour for a device with no credentials - unlinked, or straight off the
//line - because there is nothing else it can do until someone provisions it.
static void enter_provisioning(bool with_timeout){
    if (s_provisioning) return;
    s_provisioning = true;
    xTaskCreate(prov_enter_task, "prov_enter", 4096,
                (void *)(intptr_t)with_timeout, 5, NULL);
}

static void on_rocker_gesture(void){
    enter_provisioning(true);
}

static void exit_provisioning(void){
    if (!s_provisioning) return;
    s_provisioning = false;
    esp_timer_stop(s_prov_timer);
    ble_prov_exit();
    led_indicator_set_provisioning(false);
    mqtt_app_handler_mesh_refresh();   //restart the mesh if it was enabled
    ESP_LOGI(APP_TAG, "Provisioning window closed. (Rocker 3x to re-open.)");
}

//NimBLE teardown blocks, so keep it off the esp_timer task.
static void prov_exit_task(void *pv){
    exit_provisioning();
    vTaskDelete(NULL);
}
static void prov_timeout_cb(void *arg){
    ESP_LOGW(APP_TAG, "Provisioning timed out (no credentials). Closing BLE, keeping old Wi-Fi.");
    xTaskCreate(prov_exit_task, "prov_exit", 3072, NULL, 5, NULL);
}

//
// BLE RE-PROVISIONING
//
typedef struct {
    char ssid[32];
    char password[64];
} wifi_prov_creds_t;

//New credentials are saved only after BOTH Wi-Fi and cloud succeed; any failure
//reverts to the stored network so a bad password can't strand the device.
static void wifi_prov_task(void *pvParameters){
    wifi_prov_creds_t *creds = (wifi_prov_creds_t *)pvParameters;

    char old_ssid[32] = {0};
    char old_password[64] = {0};
    bool have_backup = storage_handler_load_wifi_credentials(old_ssid, sizeof(old_ssid),
                                                             old_password, sizeof(old_password));

    //This task now owns the window's lifetime and always closes it below.
    esp_timer_stop(s_prov_timer);

    ESP_LOGI(APP_TAG, "Switching to new Wi-Fi network: %s", creds->ssid);
    wifi_manager_disconnect();

    //Already fleet-provisioned, so this is a plain MQTT reconnect with the
    //existing certs — no re-provisioning.
    bool success = false;
    if (wifi_manager_connect(creds->ssid, creds->password)){
        remember_home_channel();
        start_services_once();
        for (int i = 0; i < CLOUD_WAIT_MS / 500; i++){
            if (aws_mqtt_is_connected()){ success = true; break; }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    //Exactly one terminal notification to the app.
    if (success){
        storage_handler_save_wifi_credentials(creds->ssid, creds->password);
        ESP_LOGI(APP_TAG, "Connected (Wi-Fi + cloud). New credentials saved.");
        ble_prov_send_status("{\"status\":\"ok\", \"reason\":\"connected\"}");
    } else {
        ESP_LOGE(APP_TAG, "Provisioning failed. Reverting to previous network...");
        ble_prov_send_status("{\"status\":\"error\", \"reason\":\"failed to connect\"}");
        if (have_backup){
            wifi_manager_disconnect();
            if (wifi_manager_connect(old_ssid, old_password)){
                ESP_LOGI(APP_TAG, "Reverted to previous Wi-Fi: %s", old_ssid);
                start_services_once();
            } else {
                ESP_LOGW(APP_TAG, "Revert also failed.");
            }
        }
    }

    mesh_set_router_ssid(success ? creds->ssid : old_ssid);

    vTaskDelay(pdMS_TO_TICKS(700));   //let the final BLE notification flush
    exit_provisioning();              //also restarts the mesh


    //Nothing to fall back on - reopen so the installer can try again.
    if (!success && !have_backup){
        ESP_LOGW(APP_TAG, "Still unprovisioned - reopening BLE.");
        enter_provisioning(false);
    }
    free(creds);
    vTaskDelete(NULL);
}

//Runs on the NimBLE host task, so hand the slow Wi-Fi switch to its own task.
static void on_ble_credentials_received(const char *ssid, const char *password){
    ESP_LOGI(APP_TAG, "New credentials received via BLE. Spawning connection task...");
    wifi_prov_creds_t *creds = malloc(sizeof(wifi_prov_creds_t));
    if (!creds){
        ESP_LOGE(APP_TAG, "OOM allocating credentials task");
        return;
    }
    strncpy(creds->ssid, ssid, sizeof(creds->ssid) - 1);
    creds->ssid[sizeof(creds->ssid) - 1] = '\0';
    strncpy(creds->password, password, sizeof(creds->password) - 1);
    creds->password[sizeof(creds->password) - 1] = '\0';
    xTaskCreate(wifi_prov_task, "wifi_prov_task", 4096, creds, 4, NULL);
}

static void start_time_and_scheduler(void){
    static bool started = false;
    if (started) return;
    started = true;
    initialize_sntp();
    scheduler_init();
}

//Non-blocking: the scheduler skips its checks until the clock is valid.
void initialize_sntp(void){
    ESP_LOGI(APP_TAG, "Initializing SNTP (pool.ntp.org, TZ=IST-5:30)");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    setenv("TZ", "IST-5:30", 1); //change per deployment
    tzset();
}

//Uses the device certificate stored by Firmware A.
void start_aws_iot_service(const sig_r_s_t* initial_state){
    ESP_LOGI(APP_TAG, "Wifi connected , starting the iot...");
    char *device_cert = NULL;
    char *device_key = NULL;
    if (!storage_handler_load_certs(&device_cert, &device_key)){
        ESP_LOGE(APP_TAG, "No device certificate in SPIFFS. Firmware B requires it...");
        ESP_LOGE(APP_TAG, "Restarting in 5s. Reflash Firmware A if SPIFFS was wiped.");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    aws_mqtt_start(device_cert, device_key, initial_state);
    if (device_cert) free(device_cert);
    if (device_key) free(device_key);
}
