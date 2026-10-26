#ifndef MQTT_APP_HANDLER_H
#define MQTT_APP_HANDLER_H

#include "mqtt_client.h"
#include "relay_controls.h"

#include <stdbool.h>
#include <stdint.h>

void mqtt_app_handler_init(esp_mqtt_client_handle_t client, const char* serial_number, const sig_r_s_t *initial_state);
void mqtt_app_handler_process_message(const char* topic, const char* payload);
void mqtt_app_handler_on_connected(void);
void mqtt_app_handler_on_disconnected(void);

//Called when the mesh 003 search locates the router; the app re-associates.
typedef void (*mqtt_router_found_cb_t)(uint8_t channel);
void mqtt_app_handler_set_router_found_cb(mqtt_router_found_cb_t cb);

//Re-apply the stored mesh enable/group. Used after BLE provisioning, which
//stops the mesh so it cannot fight the reconnect for the radio.
void mqtt_app_handler_mesh_refresh(void);
bool mqtt_app_handler_get_relay_state(void);
//Fill the relay state at boot, before MQTT exists, so the rocker sees the
//restored state from the first flip.
void mqtt_app_handler_set_initial_state(const sig_r_s_t *state);


#endif