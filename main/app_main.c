/*
 * ESP32CAM WiFi Camera Application
 *
 * Main application entry point for WiFi camera with SoftAP/STA mode
 * and configuration portal support.
 */

#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_intr_types.h"

#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif

#include "esp_camera.h"
#include "camera_app.h"
#include "wifi_manager.h"
#include "wifi_config_manager.h"
#include "led.h"
#include "udp_camera_client.h"

static const char* TAG = "APP_MAIN";

#define WIFI_CONFIG_BUTTON_GPIO 14  // WiFi配置按钮（改为GPIO4，避免可能的GPIO中断冲突）

static void audio_player_init_task(void* arg)
{
    ESP_LOGI(TAG, "Audio player init task running on CPU core %d", xPortGetCoreID());
    esp_err_t ret = audio_player_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio player initialization failed: %s", esp_err_to_name(ret));
    }
    else {
        ESP_LOGI(TAG, "Audio player initialized successfully on CPU1");
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize status LED on GPIO2, active High */
    led_init(2, false);

    /* Initialize camera first (to allocate high-priority interrupts) */
    esp_err_t camera_err = camera_init();
    if (camera_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(camera_err));
        led_set_state(LED_STATE_BLINK_FAST);
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待100ms

    /* Initialize audio player on CPU1 to use CPU1's interrupt resources */
    // 暂时禁用 I2S 音频初始化，用于调试看门狗重启问题
    xTaskCreatePinnedToCore(audio_player_init_task, "audio_init", 4096, NULL, 5, NULL, 1);
    // ESP_LOGI(TAG, "Audio player initialization scheduled on CPU1");
    ESP_LOGI(TAG, "Audio player initialization DISABLED for debugging");

    /* Initialize WiFi manager */
    ESP_ERROR_CHECK(wifi_manager_init());

    /* Register WiFi event handlers */
    wifi_register_event_handlers(NULL);

    /* Initialize AP */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    esp_netif_t* esp_netif_ap = wifi_init_softap();

    /* Initialize STA */
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    esp_netif_t* esp_netif_sta = wifi_init_sta();

    /* Start WiFi */
    ESP_ERROR_CHECK(esp_wifi_start());

    // 初始化WiFi配置管理器，使用宏 WIFI_CONFIG_BUTTON_GPIO 指定的引脚作为配置按钮，传入事件组和AP netif
    ESP_ERROR_CHECK(wifi_config_manager_init(WIFI_CONFIG_BUTTON_GPIO, wifi_get_event_group(), esp_netif_ap));

    /*
     * If a compile-time STA SSID is configured, wait for connection result
     * (WIFI_CONNECTED_BIT or WIFI_FAIL_BIT). If no compile-time SSID is set
     * (we rely solely on NVS/portal), skip the blocking wait so device can
     * continue to run provisioning portal and services immediately.
     */
    if (strlen(WIFI_STA_SSID) > 0) {
        EventBits_t bits = xEventGroupWaitBits(wifi_get_event_group(), WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        /* xEventGroupWaitBits() returns the bits before the call returned,
         * hence we can test which event actually happened. */
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_STA_SSID, WIFI_STA_PASSWD);
            wifi_set_dns_addr(esp_netif_ap, esp_netif_sta);
        }
        else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_STA_SSID, WIFI_STA_PASSWD);
            led_set_state(LED_STATE_BLINK_FAST);
        }
        else {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
            return;
        }
    }
    else {
        ESP_LOGI(TAG, "No compile-time STA SSID configured; skipping auto-connect wait.");
        // 没有配置WiFi，LED快速闪烁提示用户需要配置
        led_set_state(LED_STATE_BLINK_FAST);
    }

    /* Set sta as the default interface */
    esp_netif_set_default_netif(esp_netif_sta);

    /* Enable napt on the AP netif */
    if (esp_netif_napt_enable(esp_netif_ap) != ESP_OK) {
        ESP_LOGE(TAG, "NAPT not enabled on the netif: %p", esp_netif_ap);
    }

    // 等待音频初始化完成（给任务一些时间）
    vTaskDelay(pdMS_TO_TICKS(100));

    // 打印最终中断分配情况
    esp_intr_dump(NULL);  // 调试：打印最终中断分配情况

    // 启动UDP图像传输（仅相机初始化成功且非配置模式）
    if (camera_err == ESP_OK && get_wifi_provisioning_mode() == false) {
        start_udp_camera();
    }
}