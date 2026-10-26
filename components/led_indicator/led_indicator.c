#include "led_indicator.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO      GPIO_NUM_4    // active-LOW
#define TICK_MS       50
#define FAST_PERIOD   200           // provisioning: 100 ms on / 100 ms off
#define SLOW_PERIOD   3000          // offline: 100 ms pulse every 3 s
#define PULSE_MS      100

static volatile bool s_provisioning = false;
static volatile bool s_online = false;

static inline void led_write(bool on) {
    gpio_set_level(LED_GPIO, on ? 0 : 1);
}

static void led_task(void *pv) {
    uint32_t t = 0;
    while (1) {
        bool on;
        if (s_provisioning)   on = (t % FAST_PERIOD) < (FAST_PERIOD / 2);
        else if (s_online)    on = true;
        else                  on = (t % SLOW_PERIOD) < PULSE_MS;
        led_write(on);
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        t += TICK_MS;
    }
}

void led_indicator_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    led_write(false);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
}

void led_indicator_set_network(bool online) {
    s_online = online;
}

void led_indicator_set_provisioning(bool on) {
    s_provisioning = on;
}
