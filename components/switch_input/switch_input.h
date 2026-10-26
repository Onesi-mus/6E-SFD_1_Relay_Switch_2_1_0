#ifndef SWITCH_INPUT_H
#define SWITCH_INPUT_H

#include <stdbool.h>

// Rocker input (GPIO5), three selectable modes persisted in NVS, plus the
// 3x-flip provisioning gesture. See FIRMWARE_B_REFERENCE.md §3 and
// ROCKER_SWITCH_TYPE.md.

typedef enum {
  ROCKER_MODE_FOLLOW = 0,    // relay ON when rocker closed to GND, OFF when open
  ROCKER_MODE_TOGGLE = 1,    // each flip inverts relay state (default)
  ROCKER_MODE_MOMENTARY = 2, // toggle on press-to-GND only
} rocker_mode_t;

typedef bool (*switch_get_state_cb_t)(void); // current relay state (for TOGGLE)
typedef void (*switch_apply_cb_t)(bool on);
typedef void (*switch_gesture_cb_t)(void); // 3x flip detected

void switch_input_init(switch_get_state_cb_t get_state, switch_apply_cb_t apply,
                       switch_gesture_cb_t gesture);

// Persists to NVS.
void switch_input_set_mode(rocker_mode_t mode);
rocker_mode_t switch_input_get_mode(void);

#endif // SWITCH_INPUT_H
