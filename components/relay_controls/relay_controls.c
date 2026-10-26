#include "esp_log.h"
#include "driver/gpio.h"
#include "relay_controls.h"


//Internal declaration and states
#define RELAY_IO GPIO_NUM_3
#define RELAY_ON 1
#define RELAY_OFF 0

//Forward declaration
esp_err_t init_relay_module(void);
esp_err_t relay_set(bool on);
void relay_controls_apply_state(const sig_r_s_t *state);


static const char *R = "RELAY_CONTROLS";


//Function to initialize the relay
esp_err_t init_relay_module(void){
    ESP_LOGI(R, "Initializing the relay control, relay_io : %d", RELAY_IO);
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << RELAY_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(RELAY_IO, RELAY_OFF);
    return ESP_OK;
}

//Function to set the relay pin
esp_err_t relay_set(bool on){
    gpio_set_level(RELAY_IO, on ? RELAY_ON : RELAY_OFF);
    ESP_LOGI(R, "Relay state : %s", on ? "ON" : "OFF");
    return ESP_OK;
}

//Function to set relay saved state
void relay_controls_apply_state(const sig_r_s_t *state){
    ESP_LOGI(R, "Applying saved relay state: %s", state->relay_on ? "ON" : "OFF");
    relay_set(state->relay_on);
}
