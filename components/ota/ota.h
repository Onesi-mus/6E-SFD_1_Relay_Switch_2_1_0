#ifndef OTA_H
#define OTA_H

#include "mqtt_client.h"
#include <stdbool.h>




void ota_init(esp_mqtt_client_handle_t client, const char *thing_name);
void ota_handler_on_connected(void);
bool check_job_topic(const char *topic);
void ota_process_message(const char *topic, const char *payload);





#endif