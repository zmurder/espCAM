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

#define WIFI_CONFIG_BUTTON_GPIO 12  // 默认使用GPIO12作为配置按钮

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

    /* Initialize status LED on GPIO33, active low */
    led_init(33, true);

    /* Initialize WiFi manager */
    ESP_ERROR_CHECK(wifi_manager_init());

    /* Register WiFi event handlers */
    ESP_ERROR_CHECK(wifi_register_event_handlers(NULL));

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
        EventBits_t bits = xEventGroupWaitBits(wifi_get_event_group(),
                                                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                pdFALSE,
                                                pdFALSE,
                                                portMAX_DELAY);

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

    // 初始化相机并检查返回值
    esp_err_t camera_err = camera_init();
    if (camera_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(camera_err));
        led_set_state(LED_STATE_BLINK_FAST);  // 相机初始化失败，快速闪烁LED
        // 不启动UDP传输，但系统继续运行
    }
    else {
        // 相机初始化成功，启动UDP图像传输
        if (get_wifi_provisioning_mode() == false)  // 仅在非配置模式下启动
        {
            start_udp_camera();
        }
    }
}
