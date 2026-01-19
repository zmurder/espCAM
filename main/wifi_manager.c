/*
 * WiFi Manager Implementation
 *
 * This module handles WiFi hardware initialization and management
 * including SoftAP and Station mode setup, event handling, and DNS configuration.
 */

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#if IP_NAPT
#include "lwip/lwip_napt.h"
#endif

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif.h"

#include "wifi_manager.h"

/*DHCP server option*/
#define DHCPS_OFFER_DNS 0x02

static const char* TAG_AP = "WiFi SoftAP";
static const char* TAG_STA = "WiFi Sta";

static int s_retry_num = 0;

/* FreeRTOS event group to signal when we are connected/disconnected */
static EventGroupHandle_t s_wifi_event_group = NULL;

/* Network interface handles */
static esp_netif_t* s_esp_netif_ap = NULL;
static esp_netif_t* s_esp_netif_sta = NULL;

/* Event handler instances */
static esp_event_handler_instance_t s_app_wifi_start_inst = NULL;
static esp_event_handler_instance_t s_app_wifi_disconn_inst = NULL;
static esp_event_handler_instance_t s_app_ip_event_inst = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG_STA, "Station started");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
        ESP_LOGI(TAG_STA, "Station disconnected, reason:%d", event->reason);
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_STA, "Retrying to connect to the AP (%d/%d)", s_retry_num, WIFI_MAXIMUM_RETRY);
        }
        else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            // 播放WiFi连接失败语音提示
            // audio_player_play_wifi_status(1); // 1表示连接失败
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG_STA, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        // 播放WiFi连接成功语音提示
        // audio_player_play_wifi_status(0); // 0表示连接成功
    }
}

esp_err_t wifi_manager_init(void)
{
    /* Initialize event group */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG_STA, "Failed to create WiFi event group");
        return ESP_FAIL;
    }

    /*Initialize WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    ESP_LOGI(TAG_STA, "WiFi manager initialized");
    return ESP_OK;
}

esp_netif_t* wifi_init_softap(void)
{
    if (s_esp_netif_ap != NULL) {
        ESP_LOGW(TAG_AP, "SoftAP already initialized");
        return s_esp_netif_ap;
    }

    s_esp_netif_ap = esp_netif_create_default_wifi_ap();

    // 使用真实 MAC 的后三字节生成唯一 SSID
    char ap_ssid[64];
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(ap_ssid, sizeof(ap_ssid), "esp32cam_%02X%02X%02X", mac[3], mac[4], mac[5]);
    }
    else {
        // 回退到随机/占位后缀
        snprintf(ap_ssid, sizeof(ap_ssid), "esp32cam_%02X%02X%02X", 0x12, 0x34, 0x56);
    }

    wifi_config_t wifi_ap_config;
    memset(&wifi_ap_config, 0, sizeof(wifi_config_t));

    wifi_ap_config.ap.ssid_len = (uint8_t)strlen(ap_ssid);
    wifi_ap_config.ap.channel = WIFI_AP_CHANNEL;
    wifi_ap_config.ap.max_connection = WIFI_MAX_STA_CONN;
    wifi_ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    // 复制SSID到配置结构体
    strncpy((char*)wifi_ap_config.ap.ssid, ap_ssid, sizeof(wifi_ap_config.ap.ssid) - 1);

    // 复制密码到配置结构体
    if (strlen(WIFI_AP_PASSWD) > 0) {
        strncpy((char*)wifi_ap_config.ap.password, WIFI_AP_PASSWD, sizeof(wifi_ap_config.ap.password) - 1);
    }
    else {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
        wifi_ap_config.ap.password[0] = '\0';  // 无密码
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    ESP_LOGI(TAG_AP, "wifi_init_softap finished. SSID:%s password:%s channel:%d", ap_ssid, WIFI_AP_PASSWD, WIFI_AP_CHANNEL);

    return s_esp_netif_ap;
}

esp_netif_t* wifi_init_sta(void)
{
    if (s_esp_netif_sta != NULL) {
        ESP_LOGW(TAG_STA, "Station already initialized");
        return s_esp_netif_sta;
    }

    s_esp_netif_sta = esp_netif_create_default_wifi_sta();

    wifi_config_t wifi_sta_config = {
        .sta =
            {
                .ssid = WIFI_STA_SSID,
                .password = WIFI_STA_PASSWD,
                .scan_method = WIFI_ALL_CHANNEL_SCAN,
                .failure_retry_cnt = WIFI_MAXIMUM_RETRY,
                /* Authmode threshold resets to WPA2 as default if password
                 * matches WPA2 standards (password len => 8). If you want to
                 * connect the device to deprecated WEP/WPA networks, Please set
                 * the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set
                 * the password with length and format matching to
                 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
                 */
                .threshold.authmode = WIFI_SCAN_AUTH_MODE_THRESHOLD,
                .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));

    ESP_LOGI(TAG_STA, "wifi_init_sta finished.");

    return s_esp_netif_sta;
}

void wifi_set_dns_addr(esp_netif_t* esp_netif_ap, esp_netif_t* esp_netif_sta)
{
    esp_netif_dns_info_t dns;
    esp_netif_get_dns_info(esp_netif_sta, ESP_NETIF_DNS_MAIN, &dns);
    uint8_t dhcps_offer_option = DHCPS_OFFER_DNS;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(esp_netif_ap));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(esp_netif_ap, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_offer_option, sizeof(dhcps_offer_option)));
    ESP_ERROR_CHECK(esp_netif_set_dns_info(esp_netif_ap, ESP_NETIF_DNS_MAIN, &dns));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(esp_netif_ap));
}

esp_err_t wifi_register_event_handlers(EventGroupHandle_t event_group)
{
    /* Use the provided event group if available, otherwise use the internal one */
    if (event_group != NULL) {
        s_wifi_event_group = event_group;
    }

    /* Register Event handler and save instances for later unregistration */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START, &wifi_event_handler, NULL, &s_app_wifi_start_inst));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL, &s_app_wifi_disconn_inst));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &s_app_ip_event_inst));

    ESP_LOGI(TAG_STA, "WiFi event handlers registered");
    return ESP_OK;
}

esp_err_t wifi_unregister_event_handlers(void)
{
    if (s_app_wifi_start_inst != NULL) {
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, s_app_wifi_start_inst));
        s_app_wifi_start_inst = NULL;
    }
    if (s_app_wifi_disconn_inst != NULL) {
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, s_app_wifi_disconn_inst));
        s_app_wifi_disconn_inst = NULL;
    }
    if (s_app_ip_event_inst != NULL) {
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_app_ip_event_inst));
        s_app_ip_event_inst = NULL;
    }

    ESP_LOGI(TAG_STA, "WiFi event handlers unregistered");
    return ESP_OK;
}

EventGroupHandle_t wifi_get_event_group(void)
{
    return s_wifi_event_group;
}

esp_netif_t* wifi_get_ap_netif(void)
{
    return s_esp_netif_ap;
}

esp_netif_t* wifi_get_sta_netif(void)
{
    return s_esp_netif_sta;
}
