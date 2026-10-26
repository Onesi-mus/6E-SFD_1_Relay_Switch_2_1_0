#ifndef ENERGY_METER_H
#define ENERGY_METER_H

#include <stdbool.h>
#include "mqtt_client.h"

/**
 * @brief Initialize BL0937 GPIOs/ISRs and load stored K-factors from NVS.
 *        Call once after nvs_flash_init() in app_main, before MQTT connects.
 */
void energy_meter_init(void);

/**
 * @brief Start the periodic energy task. Idempotent — safe to call again
 *        on MQTT reconnect (just refreshes state).
 *        Call from mqtt_app_handler_on_connected().
 *
 * @param client         MQTT client to publish through.
 * @param serial         Device serial (for topic "sig/Sigtron%s/energy").
 * @param relay_initial  Current state of relay (ON/OFF).
 */
void energy_meter_start(esp_mqtt_client_handle_t client,
                        const char *serial,
                        bool relay_initial);

/**
 * @brief Notify the energy meter that the relay state just changed.
 *        Call this from your relay-toggle handler. The meter queues an event
 *        and publishes a fresh reading shortly after.
 */
void energy_meter_relay_changed(bool on);

#endif // ENERGY_METER_H
