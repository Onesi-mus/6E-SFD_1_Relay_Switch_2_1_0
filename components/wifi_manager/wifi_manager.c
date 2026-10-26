#include "wifi_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "app_config.h"
#include <string.h>

static const char *WIFI_TAG = "WIFI_MANAGER";

// Bounded so a bad network from re-provisioning can't hang the caller.
#define WIFI_CONN_TIMEOUT_MS 25000

// Every failed association is a full 13-channel scan. Retrying immediately
// keeps a node that has lost the router off the mesh channel almost
// continuously, so it drops out of the roster. Back off between attempts.
#define WIFI_RETRY_BACKOFF_MS 5000

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

// Gates the auto-reconnect so a manual disconnect doesn't rejoin the old AP.
static bool s_wifi_started = false;

// Separate gate for the mesh: it needs the auto-reconnect to stop fighting its
// channel search, but the WiFi DRIVER must stay running because ESP-NOW rides
// on it. Stopping the driver would strand the mesh with no way to scan.
static volatile bool s_auto_retry = true;

//One-shot backoff timer. The retry must not block the event task.
static esp_timer_handle_t s_retry_timer = NULL;

static void wifi_retry_cb(void *arg){
    if (s_wifi_started && s_auto_retry){
        ESP_LOGW(WIFI_TAG, "Retrying Wi-Fi association...");
        esp_wifi_connect();
    }
}

static void wifi_schedule_retry(void){
    if (!s_retry_timer) return;
    esp_timer_stop(s_retry_timer);
    esp_timer_start_once(s_retry_timer, (uint64_t)WIFI_RETRY_BACKOFF_MS * 1000ULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        //Only when we are allowed to associate. With the mesh up the driver is
        //started deliberately WITHOUT connecting; the election decides who does.
        if (s_auto_retry) esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        if (s_wifi_started && s_auto_retry){   //unexpected drop, not a manual disconnect
            ESP_LOGW(WIFI_TAG, "Wifi disconnected, retrying in %d ms......", WIFI_RETRY_BACKOFF_MS);
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            wifi_schedule_retry();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ESP_LOGI(WIFI_TAG, "Got IP.....");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//Function to initialize wifi in station mode
void wifi_manager_init(void){
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));  //load wifi driver
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    const esp_timer_create_args_t targs = { .callback = wifi_retry_cb, .name = "wifi_retry" };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_retry_timer));
}

//Driver + SoftAP up, no association. This is the mesh's entry point: ESP-NOW
//rides on the driver and 002 needs the AP, but the node must stay unassociated
//until the RSSI election picks a gateway.
void wifi_manager_start_driver(void){
    if (s_wifi_started) return;
    s_auto_retry = false;                 //STA_START must not auto-connect
    //APSTA: the mesh needs the SoftAP for 002. Not fatal if the AP half is
    //rejected here - the mesh reconfigures and restarts it on start-up anyway.
    if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK)
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_err_t e = esp_wifi_start();
    if (e != ESP_OK){
        ESP_LOGW(WIFI_TAG, "APSTA start failed (%s), falling back to STA", esp_err_to_name(e));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
    s_wifi_started = true;
    ESP_LOGI(WIFI_TAG, "Wi-Fi driver started (not associated)");
}

//Suppress or resume the automatic reconnect WITHOUT stopping the driver.
void wifi_manager_set_auto_retry(bool enabled){
    if (s_auto_retry == enabled) return;
    s_auto_retry = enabled;
    ESP_LOGI(WIFI_TAG, "Auto-reconnect %s", enabled ? "resumed" : "suppressed");
    if (!enabled){
        if (s_retry_timer) esp_timer_stop(s_retry_timer);
        esp_wifi_disconnect();             //stop the attempt in flight, keep the driver
    }
}

//True once an IP is acquired, false on timeout (wrong password, AP unreachable).
bool wifi_manager_connect(const char *ssid, const char *password){
    s_auto_retry = true;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    //Keep APSTA when the mesh has the SoftAP up: dropping to STA tears the AP
    //down and takes ESP-NOW's interface with it.
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    if (mode != WIFI_MODE_APSTA) ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    if (!s_wifi_started){
        s_wifi_started = true;
        ESP_ERROR_CHECK(esp_wifi_start());  //STA_START triggers the connect
    } else {
        //Driver already running (the mesh started it, or we are the elected
        //gateway taking the STA). Reconnect in place - stopping the driver
        //here would kill ESP-NOW.
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_wifi_connect();
    }
    ESP_LOGI(WIFI_TAG, "Connecting to WIFI : %s ......... (%ds timeout)", ssid, WIFI_CONN_TIMEOUT_MS / 1000);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(WIFI_CONN_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT){
        ESP_LOGI(WIFI_TAG, "Connected to WIFI : %s", ssid);
        return true;
    }
    ESP_LOGE(WIFI_TAG, "WIFI connect timeout/failed : %s", ssid);
    return false;
}

void wifi_manager_disconnect(void){
    ESP_LOGI(WIFI_TAG, "Manual Wi-Fi disconnect requested.");
    s_wifi_started = false;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
}
