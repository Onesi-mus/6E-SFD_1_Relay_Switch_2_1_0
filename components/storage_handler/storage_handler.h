#ifndef STORAGE_HANDLER_H
#define STORAGE_HANDLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "relay_controls.h"

//Power-restore behaviour. See RELAY_STATUS_POWERON.md.
typedef enum {
    POWERON_OFF    = 0,
    POWERON_ON     = 1,
    POWERON_MEMORY = 2,
} poweron_mode_t;

void storage_handler_mount(void);
//Defaults to MEMORY if never set.
poweron_mode_t storage_handler_load_poweron_mode(void);
void storage_handler_save_poweron_mode(poweron_mode_t mode);
bool storage_handler_load_wifi_credentials(char *ssid, size_t ssid_len, char *password, size_t pass_len);
bool storage_handler_save_wifi_credentials(const char* ssid, const char* password);
bool storage_handler_save_certs(const char *cert_pem, const char *key_pem);
bool storage_handler_load_certs(char **cert_pem, char **key_pem);
esp_err_t storage_handler_set_prov_status_flag(bool status);
esp_err_t storage_handler_get_prov_status_flag(bool *status);
bool storage_handler_load_relay_state(sig_r_s_t *state);
bool storage_handler_save_relay_state(const sig_r_s_t *state);

//Mesh group (the claimed userId). Empty string = unclaimed.
bool storage_handler_load_mesh_group(char *out, size_t len);
bool storage_handler_save_mesh_group(const char *group_id);
//Mesh feature flag. Defaults to false so a mesh fault can't reach the fleet.
bool storage_handler_load_mesh_enabled(void);
void storage_handler_save_mesh_enabled(bool enabled);
//Last channel we associated on, used to park an unassociated node.
uint8_t storage_handler_load_home_channel(void);
void storage_handler_save_home_channel(uint8_t ch);

//Erases owner identity (wifi, mesh group, relay state, rocker mode). Keeps the
//device certificate and the BL0937 calibration — neither is recoverable.
bool storage_handler_unlink(void);



#endif