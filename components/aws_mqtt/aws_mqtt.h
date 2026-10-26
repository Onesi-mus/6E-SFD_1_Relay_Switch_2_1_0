#ifndef AWS_MQTT_H
#define AWS_MQTT_H

#include "relay_controls.h"

//Function to start MQTT with the device certificate stored from Firmware A.
//Caller must already have valid cert+key loaded from SPIFFS.
void aws_mqtt_start(const char *cert, const char *key, const sig_r_s_t *state);

#include <stdbool.h>
//True while the MQTT/cloud link is up.
bool aws_mqtt_is_connected(void);

//Stop the MQTT task WITHOUT destroying the client, so a later resume does not
//need the certificates again. Used when the mesh takes the radio away: a
//meshed relay is unassociated and has no uplink at all.
void aws_mqtt_stop(void);

//Restart a client that was stopped. False if MQTT was never started, in which
//case the caller should run its normal first-time start path.
bool aws_mqtt_resume(void);


#endif
