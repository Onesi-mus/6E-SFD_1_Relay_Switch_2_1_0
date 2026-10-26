/*
 * energy_meter.c — BL0937 reader + MQTT publisher for 1-gang relay
 *
 * Topic   : sig/Sigtron<serial>/energy
 * Payload : {"state":"on|off","voltage":V,"current":I,"power":P}
 *
 * Events that trigger a publish:
 *   1. Relay state change (on/off)        → fresh reading published
 *   2. Periodic 10 s check, if relay ON   → publish if |ΔP| > 5 W
 *
 * K-factors are loaded from NVS namespace "bl0937_cal" (set by factory_test).
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs.h"
#include "cJSON.h"

#include "energy_meter.h"

static const char *TAG = "ENERGY";

/* ── BL0937 pins (1-Gang) ───────────────────────────────────── */
#define CF_GPIO   GPIO_NUM_7   /* power pulses */
#define CF1_GPIO  GPIO_NUM_6   /* voltage pulses (when SEL=HIGH) */
#define SEL_GPIO  GPIO_NUM_1   /* HIGH = voltage mode */

/* ── NVS keys (must match factory_test) ─────────────────────── */
#define NVS_NAMESPACE     "bl0937_cal"
#define NVS_KEY_VRMS_K    "vrms_k"
#define NVS_KEY_POWER_K   "power_k"

/* ── Behaviour ──────────────────────────────────────────────── */
#define SAMPLE_PERIOD_MS   10000  /* periodic threshold check */
#define MEASURE_WINDOW_MS  1000   /* pulse-counting window per measurement */
#define POWER_THRESHOLD_W  5.0f   /* publish if |P - last_pub| > this */

/* ── Pulse counters (ISR) ───────────────────────────────────── */
static volatile uint32_t g_cf_pulses  = 0;
static volatile uint32_t g_cf1_pulses = 0;

/* ── Calibration ────────────────────────────────────────────── */
static float s_vrms_k  = 0.0f;
static float s_power_k = 0.0f;

/* ── MQTT/state ─────────────────────────────────────────────── */
static esp_mqtt_client_handle_t s_client = NULL;
static const char *s_serial = NULL;
static bool s_relay_on = false;
static float s_last_pub_p = -1.0e9f;   /* sentinel — first sample always publishes */

typedef struct {
    bool on;
} relay_event_t;

static QueueHandle_t s_event_queue = NULL;
static bool s_started = false;

/* ── ISRs ───────────────────────────────────────────────────── */
static void IRAM_ATTR cf_isr (void *arg)  { g_cf_pulses++;  }
static void IRAM_ATTR cf1_isr(void *arg)  { g_cf1_pulses++; }

/* ── Helpers ────────────────────────────────────────────────── */
static bool load_k_factors(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "K-factors NVS open failed; readings will be 0");
        return false;
    }
    uint32_t v_bits = 0, p_bits = 0;
    esp_err_t e1 = nvs_get_u32(h, NVS_KEY_VRMS_K,  &v_bits);
    esp_err_t e2 = nvs_get_u32(h, NVS_KEY_POWER_K, &p_bits);
    nvs_close(h);
    if (e1 != ESP_OK || e2 != ESP_OK) {
        ESP_LOGW(TAG, "K-factors not in NVS; readings will be 0");
        return false;
    }
    memcpy(&s_vrms_k,  &v_bits, sizeof(float));
    memcpy(&s_power_k, &p_bits, sizeof(float));
    if (isnan(s_vrms_k) || isnan(s_power_k) || s_vrms_k <= 0 || s_power_k <= 0) {
        ESP_LOGE(TAG, "K-factors invalid (vrms=%f, power=%f)", s_vrms_k, s_power_k);
        s_vrms_k = s_power_k = 0;
        return false;
    }
    ESP_LOGI(TAG, "K-factors: VRMS_K=%.4f POWER_K=%.4f", s_vrms_k, s_power_k);
    return true;
}

/* Take a fresh measurement by counting pulses for MEASURE_WINDOW_MS. */
static void measure_v_i_p(float *v, float *i, float *p) {
    portDISABLE_INTERRUPTS();
    g_cf_pulses = 0; g_cf1_pulses = 0;
    portENABLE_INTERRUPTS();

    vTaskDelay(pdMS_TO_TICKS(MEASURE_WINDOW_MS));

    portDISABLE_INTERRUPTS();
    uint32_t cf  = g_cf_pulses;
    uint32_t cf1 = g_cf1_pulses;
    portENABLE_INTERRUPTS();

    float window_s = MEASURE_WINDOW_MS / 1000.0f;
    float cf_rate  = (float)cf  / window_s;
    float cf1_rate = (float)cf1 / window_s;

    *v = (s_vrms_k  > 0) ? cf1_rate / s_vrms_k  : 0.0f;
    *p = (s_power_k > 0) ? cf_rate  / s_power_k : 0.0f;
    *i = (*v > 0)        ? *p / *v              : 0.0f;
}

static void publish_event(bool on, float v, float i, float p) {
    if (!s_client || !s_serial) return;

    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    cJSON_AddStringToObject(root, "state",   on ? "on" : "off");
    cJSON_AddNumberToObject(root, "voltage", v);
    cJSON_AddNumberToObject(root, "current", i);
    cJSON_AddNumberToObject(root, "power",   p);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    char topic[64];
    snprintf(topic, sizeof(topic), "sig/Sigtron%s/energy", s_serial);
    esp_mqtt_client_publish(s_client, topic, json, 0, 1, 0);
    ESP_LOGI(TAG, "Pub[%s]: state=%s V=%.2f I=%.4f P=%.2f",
             topic, on ? "on" : "off", v, i, p);

    free(json);
}

/* Process a queued relay-change event. */
static void handle_relay_event(const relay_event_t *ev) {
    float v = 0.0f, i = 0.0f, p = 0.0f;
    if (ev->on) {
        /* Relay just turned on — measure fresh */
        measure_v_i_p(&v, &i, &p);
        s_last_pub_p = p;
    } else {
        /* Relay just turned off — report 0s */
        s_last_pub_p = 0.0f;
    }
    publish_event(ev->on, v, i, p);
}

/* Background task: periodic threshold check + event-driven publishing. */
static void energy_task(void *pv) {
    relay_event_t ev;

    /* Initial publish if relay is already ON when we start */
    if (s_relay_on) {
        float v, i, p;
        measure_v_i_p(&v, &i, &p);
        publish_event(true, v, i, p);
        s_last_pub_p = p;
    }

    TickType_t next_periodic = xTaskGetTickCount() + pdMS_TO_TICKS(SAMPLE_PERIOD_MS);

    while (1) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait = (next_periodic > now) ? (next_periodic - now) : 0;

        if (xQueueReceive(s_event_queue, &ev, wait) == pdTRUE) {
            /* Event arrived */
            handle_relay_event(&ev);
            /* Drain any other queued events too */
            while (xQueueReceive(s_event_queue, &ev, 0) == pdTRUE) {
                handle_relay_event(&ev);
            }
            next_periodic = xTaskGetTickCount() + pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
        } else {
            /* Periodic timer fired */
            if (s_relay_on) {
                float v, i, p;
                measure_v_i_p(&v, &i, &p);
                if (fabsf(p - s_last_pub_p) > POWER_THRESHOLD_W) {
                    publish_event(true, v, i, p);
                    s_last_pub_p = p;
                }
            }
            next_periodic = xTaskGetTickCount() + pdMS_TO_TICKS(SAMPLE_PERIOD_MS);
        }
    }
}

/* ── Public API ─────────────────────────────────────────────── */
void energy_meter_init(void) {
    /* SEL output (HIGH = voltage mode) */
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << SEL_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    gpio_set_level(SEL_GPIO, 1);

    /* CF/CF1 inputs with rising-edge interrupts */
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << CF_GPIO) | (1ULL << CF1_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&in);

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(err));
    }
    gpio_isr_handler_add(CF_GPIO,  cf_isr,  NULL);
    gpio_isr_handler_add(CF1_GPIO, cf1_isr, NULL);

    load_k_factors();

    s_event_queue = xQueueCreate(8, sizeof(relay_event_t));
    if (!s_event_queue) ESP_LOGE(TAG, "Failed to create event queue");

    ESP_LOGI(TAG, "Energy meter initialized");
}

void energy_meter_start(esp_mqtt_client_handle_t client,
                        const char *serial,
                        bool relay_initial) {
    s_client   = client;
    s_serial   = serial;
    s_relay_on = relay_initial;

    if (s_started) {
        ESP_LOGI(TAG, "Already started — refreshed client/state");
        return;
    }
    s_started = true;
    xTaskCreate(energy_task, "energy_meter", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "Energy meter task spawned (R=%d)", relay_initial);
}

void energy_meter_relay_changed(bool on) {
    s_relay_on = on;

    if (!s_event_queue) return;
    relay_event_t ev = {.on = on};
    xQueueSend(s_event_queue, &ev, 0);
}
