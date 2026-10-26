#include "switch_input.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#define ROCKER_GPIO       GPIO_NUM_5

#define POLL_MS           10
#define DEBOUNCE_SAMPLES  5             // 50 ms stable — rejects contact bounce / EMI spikes
#define GESTURE_COUNT     3
#define GESTURE_WINDOW_MS 3000

#define NVS_NS            "rocker"
#define NVS_KEY_MODE      "mode"

// 0 = rocker only fires the provisioning gesture (for debugging input noise).
#define ROCKER_CONTROLS_RELAY 1

static const char *TAG = "SWITCH_IN";

typedef struct {
    gpio_num_t gpio;
    int stable;              // committed (debounced) level
    int cand;                // candidate level being debounced
    int deb;                 // consecutive samples of cand
    int fall_count;          // actuations-to-GND in the current window
    TickType_t first_fall;   // tick of the first fall in the window
} rocker_t;

static rocker_t s_rocker;
static rocker_mode_t s_mode = ROCKER_MODE_TOGGLE; // default TOGGLE
static switch_get_state_cb_t s_get_state;
static switch_apply_cb_t s_apply;
static switch_gesture_cb_t s_gesture_cb;

// --- NVS ------------------------------------------------------------------

static void load_mode(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return; // keep default
    uint8_t m = 0;
    if (nvs_get_u8(h, NVS_KEY_MODE, &m) == ESP_OK && m <= ROCKER_MODE_MOMENTARY) s_mode = (rocker_mode_t)m;
    nvs_close(h);
}

static const char *mode_name(rocker_mode_t m) {
    switch (m) {
        case ROCKER_MODE_FOLLOW:    return "follow";
        case ROCKER_MODE_MOMENTARY: return "momentary";
        default:                    return "toggle";
    }
}

static void save_mode(rocker_mode_t mode) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, NVS_KEY_MODE, (uint8_t)mode);
    nvs_commit(h);
    nvs_close(h);
}

// --- Edge handling --------------------------------------------------------

static void handle_edge(rocker_t *r, int level) {
#if ROCKER_CONTROLS_RELAY
    switch (s_mode) {
    case ROCKER_MODE_FOLLOW:
        if (s_apply) s_apply(level == 0);          // closed to GND = ON
        break;
    case ROCKER_MODE_MOMENTARY:
        if (level == 0 && s_apply) s_apply(!(s_get_state ? s_get_state() : false));
        break;
    case ROCKER_MODE_TOGGLE:
    default:
        if (s_apply) s_apply(!(s_get_state ? s_get_state() : false));
        break;
    }
#endif

    // 3 falling edges inside the window = provisioning gesture (all modes).
    if (level == 0) {
        TickType_t now = xTaskGetTickCount();
        if (r->fall_count == 0 ||
            (now - r->first_fall) > pdMS_TO_TICKS(GESTURE_WINDOW_MS)) {
            r->fall_count = 1;
            r->first_fall = now;
        } else {
            r->fall_count++;
        }
        if (r->fall_count >= GESTURE_COUNT) {
            r->fall_count = 0;
            ESP_LOGI(TAG, "Gesture: %dx -> provisioning", GESTURE_COUNT);
            if (s_gesture_cb) s_gesture_cb();
        }
    }
}

static void switch_task(void *pv) {
    // Seed the baseline without acting, so boot position causes no phantom flip.
    s_rocker.stable = gpio_get_level(s_rocker.gpio);
    s_rocker.cand = s_rocker.stable;
    s_rocker.deb = 0;
    s_rocker.fall_count = 0;

    while (1) {
        rocker_t *r = &s_rocker;
        int raw = gpio_get_level(r->gpio);
        if (raw == r->stable) {
            r->deb = 0;
            r->cand = raw;
        } else if (raw == r->cand) {
            if (++r->deb >= DEBOUNCE_SAMPLES) {
                r->stable = raw;
                r->deb = 0;
                handle_edge(r, raw);
            }
        } else {
            r->cand = raw;         // restart debounce
            r->deb = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// --- Public API -----------------------------------------------------------

void switch_input_init(switch_get_state_cb_t get_state,
                       switch_apply_cb_t apply,
                       switch_gesture_cb_t gesture) {
    s_get_state = get_state;
    s_apply = apply;
    s_gesture_cb = gesture;

    load_mode();

    s_rocker = (rocker_t){ .gpio = ROCKER_GPIO };

    // Pull-up so an open rocker reads a firm HIGH instead of floating.
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << ROCKER_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&c);

    xTaskCreate(switch_task, "switch_task", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "Rocker init (GPIO%d) mode: %s", ROCKER_GPIO, mode_name(s_mode));
}

void switch_input_set_mode(rocker_mode_t mode) {
    if (mode > ROCKER_MODE_MOMENTARY) return;
    s_mode = mode;
    save_mode(mode);
    ESP_LOGI(TAG, "Rocker mode set to %s", mode_name(mode));

    // Entering follow: make the relay match the rocker position now, instead of
    // waiting for the next flip.
    if (mode == ROCKER_MODE_FOLLOW && s_apply) {
        bool want_on = (s_rocker.stable == 0);   // rocker closed to GND (LOW) = ON
        bool is_on   = s_get_state ? s_get_state() : !want_on;
        if (want_on != is_on) s_apply(want_on);
    }
}

rocker_mode_t switch_input_get_mode(void) {
    return s_mode;
}
