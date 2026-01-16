#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON,
    LED_STATE_BLINK_FAST,
    LED_STATE_BREATH,
} led_state_t;

esp_err_t led_init(int gpio_num, bool active_low);
void led_set_state(led_state_t state);
void led_deinit(void);
