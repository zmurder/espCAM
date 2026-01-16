#include "led.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "led";
static int s_led_gpio = -1;
static bool s_active_low = true;
static led_state_t s_state = LED_STATE_OFF;
static TaskHandle_t s_task = NULL;

static const ledc_mode_t LEDC_MODE = LEDC_HIGH_SPEED_MODE;
// Use TIMER_1/CHANNEL_1 to avoid conflict with camera XCLK (which uses TIMER_0/CHANNEL_0)
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_1;
static const ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_1;
static const ledc_timer_bit_t LEDC_DUTY_RES = LEDC_TIMER_10_BIT; // 0-1023
static const int LEDC_FREQUENCY = 5000; // 5 kHz
static int s_duty_max = 0;

static void led_set_duty(int duty)
{
    if (s_led_gpio < 0) return;
    int use_duty = duty;
    if (s_active_low) {
        use_duty = s_duty_max - duty;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, use_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void led_task(void *arg)
{
    // Ensure fade functions available
    while (1) {
        led_state_t cur = s_state;
        if (cur == LED_STATE_ON) {
            led_set_duty(s_duty_max);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (cur == LED_STATE_OFF) {
            led_set_duty(0);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (cur == LED_STATE_BLINK_FAST) {
            // 100ms on, 100ms off
            led_set_duty(s_duty_max);
            vTaskDelay(pdMS_TO_TICKS(100));
            if (s_state != LED_STATE_BLINK_FAST) continue;
            led_set_duty(0);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (cur == LED_STATE_BREATH) {
            // Use LEDC fade for smooth breathing: up 1000ms, down 1000ms
            // 使用非阻塞模式，避免任务阻塞
            ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, s_duty_max, 1000);
            ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
            // 等待渐变完成
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (s_state != LED_STATE_BREATH) continue;
            ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, 0, 1000);
            ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);
            // 等待渐变完成
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

esp_err_t led_init(int gpio_num, bool active_low)
{
    if (s_task != NULL) return ESP_ERR_INVALID_STATE;
    s_led_gpio = gpio_num;
    s_active_low = active_low;

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_duty_max = (1 << LEDC_DUTY_RES) - 1;

    // Install fade service
    ledc_fade_func_install(0);

    // default off
    led_set_duty(0);

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, &s_task);
    return ESP_OK;
}

void led_set_state(led_state_t state)
{
    s_state = state;
}

void led_deinit(void)
{
    if (s_task) {
        vTaskDelete(s_task);
        s_task = NULL;
    }
    if (s_led_gpio >= 0) {
        gpio_set_level(s_led_gpio, s_active_low ? 1 : 0);
    }
}
