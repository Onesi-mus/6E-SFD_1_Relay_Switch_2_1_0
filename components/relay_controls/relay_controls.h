#ifndef RELAY_CONTROL_H
#define RELAY_CONTROL_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct{
    bool relay_on;
} sig_r_s_t;


esp_err_t init_relay_module(void);
esp_err_t relay_set(bool on);
void relay_controls_apply_state(const sig_r_s_t *state);

#endif
