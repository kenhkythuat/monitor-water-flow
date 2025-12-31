#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "gateway_state.h"   // enum ở gateway_core

typedef struct {
    int red_gpio;
    int blue_gpio;
    int green_gpio;
    bool active_high;
} status_led_cfg_t;

esp_err_t status_led_init(const status_led_cfg_t *cfg);
void status_led_on_state(gateway_state_t state);
void status_led_pulse_green(uint32_t ms);