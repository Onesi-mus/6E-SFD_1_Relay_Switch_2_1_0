#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdbool.h>

// Status LED on GPIO4 (active-LOW).
// Blink patterns: FIRMWARE_B_REFERENCE.md §10.

void led_indicator_init(void);

// true = online (Wi-Fi + cloud up) -> solid on; false -> slow pulse.
void led_indicator_set_network(bool online);

// true = BLE provisioning window open -> fast blink; overrides network state.
void led_indicator_set_provisioning(bool on);

#endif // LED_INDICATOR_H
