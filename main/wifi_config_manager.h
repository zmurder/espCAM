/*
 * WiFi Configuration Manager Header
 *
 * This module handles dynamic WiFi configuration via a captive portal.
 */

#ifndef WIFI_CONFIG_MANAGER_H
#define WIFI_CONFIG_MANAGER_H

#include "esp_err.h"
#include "esp_netif.h"
#include <stdbool.h>
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure to store WiFi credentials
 */
typedef struct {
    char ssid[64];
    char password[64];
} wifi_credentials_t;

/**
 * @brief Initialize WiFi configuration manager
 * 
 * @param button_gpio GPIO pin connected to the configuration button
 * @return esp_err_t
 */
/* Event bits (shared with app) */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

esp_err_t wifi_config_manager_init(int button_gpio, EventGroupHandle_t event_group, esp_netif_t *ap_netif);

/**
 * @brief Check if device is in configuration mode
 * 
 * @return true if in configuration mode, false otherwise
 */
bool get_wifi_provisioning_mode(void);

/**
 * @brief Save WiFi credentials to NVS
 * 
 * @param creds WiFi credentials to save
 * @return esp_err_t
 */
esp_err_t wifi_config_save_credentials(wifi_credentials_t *creds);

/**
 * @brief Load WiFi credentials from NVS
 * 
 * @param creds Pointer to wifi_credentials_t to store loaded credentials
 * @return esp_err_t
 */
esp_err_t wifi_config_load_credentials(wifi_credentials_t *creds);

/**
 * @brief Start the configuration portal (SoftAP + Captive Portal)
 * 
 * @return esp_err_t
 */
esp_err_t wifi_start_config_portal(void);

/**
 * @brief Stop the configuration portal
 * 
 * @return esp_err_t
 */
esp_err_t wifi_stop_config_portal(void);

/**
 * @brief Get the SoftAP IP address
 *
 * @return esp_netif_ip_info_t* IP information of the SoftAP
 */
const esp_netif_ip_info_t* wifi_get_ap_ip_info(void);

/**
 * @brief Clear all WiFi credentials from NVS (including ESP32 WiFi driver saved credentials)
 *
 * @return esp_err_t
 */
esp_err_t wifi_config_clear_all_credentials(void);

/**
 * @brief Stop UDP camera transmission
 */
void stop_udp_camera(void);

/**
 * @brief Restart UDP camera transmission (called after WiFi reset)
 */
void restart_udp_camera(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CONFIG_MANAGER_H