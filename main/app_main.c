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
#include "posture_model.h"
#include "audio_player.h"

static const char* TAG = "APP_MAIN";

#define WIFI_CONFIG_BUTTON_GPIO 14

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

/**
 * @brief 坐姿检测推理任务
 */
static void posture_inference_task(void* arg)
{
    ESP_LOGI(TAG, "Posture inference task started on core %d", xPortGetCoreID());

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 初始化姿态检测模型
    ESP_LOGI(TAG, "Posture model init...");
    esp_err_t ret = posture_model_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Posture model init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Posture model init OK");

    ESP_LOGI(TAG, "Starting posture inference loop...");

#if POSTURE_TEST_MODE
    // 测试模式：test() 已在 posture_model_init() 中运行，这里直接结束任务
    ESP_LOGI(TAG, "POSTURE_TEST_MODE: Model test completed, exiting task");
    posture_model_deinit();
    vTaskDelete(NULL);
#else
    // 实际推理模式：一次捕获，同时用于推理和UDP发送
    while (1) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "Failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // 先发送图像到PC（用于调试显示）
        send_image_via_udp(fb);

        // 再进行姿态推理
        posture_output_t output;
        ret = posture_model_run_inference(fb, &output);

        // 释放帧缓冲（在推理函数外部释放，确保图像已发送）
        esp_camera_fb_return(fb);

        if (ret == ESP_OK) {
            if (output.result == POSTURE_OK) {
                ESP_LOGI(TAG, "Posture: OK (ratio=%.3f)", output.ratio);
            }
            else if (output.result == POSTURE_BAD_NECK) {
                ESP_LOGE(TAG, "Posture: BAD_NECK (ratio=%.3f)", output.ratio);
                audio_player_play_posture_alert(POSTURE_BAD_NECK);
            }
            else if (output.result == POSTURE_BAD_SHOULDER) {
                ESP_LOGE(TAG, "Posture: BAD - shoulder issue");
                audio_player_play_posture_alert(POSTURE_BAD_SHOULDER);
            }
            else if (output.result == POSTURE_NOT_DETECTED) {
                ESP_LOGW(TAG, "Posture: Keypoints not detected");
            }

            // 发送姿态结果到 PC 用于调试显示
            send_posture_result_via_udp(&output);

            ESP_LOGI(TAG, "Inference latency: %lu us", (unsigned long)posture_model_get_last_latency_us());
        }

        // 推理间隔2秒，避免过于频繁
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    posture_model_deinit();
    vTaskDelete(NULL);
#endif
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
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Initialize audio player on CPU1 to use CPU1's interrupt resources */
    xTaskCreatePinnedToCore(audio_player_init_task, "audio_init", 4096, NULL, 5, NULL, 1);
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

    // 初始化WiFi配置管理器
    ESP_ERROR_CHECK(wifi_config_manager_init(WIFI_CONFIG_BUTTON_GPIO, wifi_get_event_group(), esp_netif_ap));

    /*
     * If a compile-time STA SSID is configured, wait for connection result
     */
    if (strlen(WIFI_STA_SSID) > 0) {
        EventBits_t bits = xEventGroupWaitBits(wifi_get_event_group(), WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

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
        led_set_state(LED_STATE_BLINK_FAST);
    }

    /* Set sta as the default interface */
    esp_netif_set_default_netif(esp_netif_sta);

    /* Enable napt on the AP netif */
    if (esp_netif_napt_enable(esp_netif_ap) != ESP_OK) {
        ESP_LOGE(TAG, "NAPT not enabled on the netif: %p", esp_netif_ap);
    }

    // 等待音频初始化完成
    vTaskDelay(pdMS_TO_TICKS(100));

    // 打印最终中断分配情况
    esp_intr_dump(NULL);

    // 启动坐姿检测推理任务（图像捕获、推理、UDP发送都在此任务中完成）
    xTaskCreatePinnedToCore(posture_inference_task, "posture_inf", 32768, NULL, 5, NULL, 0);

    // 注意：UDP图像传输已合并到 posture_inference_task 中，不再单独启动
    // 如果需要独立的UDP相机任务，可以取消下面这行的注释
    // start_udp_camera();
}
