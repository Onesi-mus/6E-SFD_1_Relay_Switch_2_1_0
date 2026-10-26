/*
SECTIONS
-DEVICE MAC SECTION
-MQTT SECTIONS
-PUBLIC API
*/

#include "aws_mqtt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_tls.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mqtt_app_handler.h"

//Certificate Declarations (only CA root needed in B; device cert+key come from SPIFFS)
extern const char _binary_ca_pem_start[];

//AWS endpoint
#define AWS_IOT_ENDPOINT "a1zv6fodtw8hm-ats.iot.ap-south-1.amazonaws.com"

//Internal definitions and states
static const sig_r_s_t *s_initial_state = NULL;
static char *s_device_cert = NULL;
static char *s_device_key = NULL;
static char s_client_id[48];
static char s_serial_number[18];
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;

static char *s_response_buffer = NULL;
static int s_response_len = 0;
static char *s_topic_buffer = NULL;

static const char *I = "IOT_NET";

//Forward declaration
static void generate_client_id(void);
static esp_mqtt_client_handle_t create_mqtt_client_with_tls(const char *root_ca_pem, const char *cert_pem,
                                                            const char *key_pem, const char *client_id);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void handle_completed_message(esp_mqtt_client_handle_t client);
static void start_mqtt_with_device_creds(void);
void aws_mqtt_start(const char *cert, const char *key, const sig_r_s_t *state);


//
//DEVICE MAC SECTION
//
static void generate_client_id(void){
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_serial_number, sizeof(s_serial_number),
             "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_client_id, sizeof(s_client_id), "Sigtron%s", s_serial_number);
    ESP_LOGI(I, "Serial Number for MQTT: %s", s_serial_number);
    ESP_LOGI(I, "Client ID for MQTT: %s", s_client_id);
}


//
//MQTT SECTIONS
//
//Function to setup mqtt
static esp_mqtt_client_handle_t create_mqtt_client_with_tls(const char *root_ca_pem, const char *cert_pem,
                                                            const char *key_pem, const char *client_id){
    const char *alpn_protos[] = {"x-amzn-mqtt-ca", NULL};
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtts://" AWS_IOT_ENDPOINT ":443",
        .broker.verification.certificate = root_ca_pem,
        .broker.verification.alpn_protos = alpn_protos,
        .credentials.client_id = client_id,
        .credentials.authentication.certificate = cert_pem,
        .credentials.authentication.key = key_pem,
        .network.timeout_ms = 20000,
        .task.stack_size = 8192,
        .task.priority = 5,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    return client;
}

//MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id){
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(I, "MQTT connected");
        s_mqtt_connected = true;
        mqtt_app_handler_on_connected();
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(I, "MQTT disconnected");
        s_mqtt_connected = false;
        mqtt_app_handler_on_disconnected();
        break;
    case MQTT_EVENT_DATA:
        if (event->current_data_offset == 0){
            if (s_topic_buffer) free(s_topic_buffer);
            s_topic_buffer = strndup(event->topic, event->topic_len);
            if (!s_topic_buffer) break;
            if (s_response_buffer) free(s_response_buffer);
            s_response_buffer = malloc(event->total_data_len + 1);
            s_response_len = 0;
        }
        if (s_response_buffer){
            memcpy(s_response_buffer + s_response_len, event->data, event->data_len);
            s_response_len += event->data_len;
            if (s_response_len == event->total_data_len){
                handle_completed_message(event->client);
            }
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(I, "MQTT_EVENT_ERROR");
        break;
    default:
        break;
    }
}

//Function to handle the incoming messages. In B, all messages go straight to mqtt_app_handler.
static void handle_completed_message(esp_mqtt_client_handle_t client){
    s_response_buffer[s_response_len] = '\0';
    mqtt_app_handler_process_message(s_topic_buffer, s_response_buffer);
    free(s_topic_buffer); s_topic_buffer = NULL;
    free(s_response_buffer); s_response_buffer = NULL;
    s_response_len = 0;
}

//Function to connect mqtt with device certificate
static void start_mqtt_with_device_creds(void){
    ESP_LOGI(I, "Starting MQTT with device credentials...");
    s_mqtt = create_mqtt_client_with_tls(_binary_ca_pem_start, s_device_cert, s_device_key, s_client_id);
    mqtt_app_handler_init(s_mqtt, s_serial_number, s_initial_state);
    esp_mqtt_client_start(s_mqtt);
}


//
//PUBLIC API
//
bool aws_mqtt_is_connected(void){
    return s_mqtt_connected;
}

//Stop the client but keep it (and its credentials) allocated.
void aws_mqtt_stop(void){
    if (!s_mqtt) return;
    s_mqtt_connected = false;
    esp_mqtt_client_stop(s_mqtt);
    mqtt_app_handler_on_disconnected();
    ESP_LOGI(I, "MQTT stopped (uplink released)");
}

//Restart a previously stopped client.
bool aws_mqtt_resume(void){
    if (!s_mqtt) return false;
    ESP_LOGI(I, "MQTT resuming");
    esp_mqtt_client_start(s_mqtt);
    return true;
}

//Function to connect with the aws using the device certificate saved by Firmware A
void aws_mqtt_start(const char *cert, const char *key, const sig_r_s_t *state){
    s_initial_state = state;
    generate_client_id();
    s_device_cert = strdup(cert);
    s_device_key = strdup(key);
    start_mqtt_with_device_creds();
}
