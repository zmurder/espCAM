/*
 * WiFi Manager Header
 *
 * This module handles WiFi hardware initialization and management
 * including SoftAP and Station mode setup, event handling, and DNS configuration.
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include <stdbool.h>
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Event bits for WiFi connection status */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

/* STA Configuration */
#define WIFI_STA_SSID CONFIG_ESP_WIFI_REMOTE_AP_SSID
#define WIFI_STA_PASSWD CONFIG_ESP_WIFI_REMOTE_AP_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_STA_RETRY

/* AP Configuration */
#define WIFI_AP_SSID CONFIG_ESP_WIFI_AP_SSID
#define WIFI_AP_PASSWD CONFIG_ESP_WIFI_AP_PASSWORD
#define WIFI_AP_CHANNEL CONFIG_ESP_WIFI_AP_CHANNEL
#define WIFI_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN_AP

/* Auth mode threshold configuration */
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/**
 * @brief Initialize WiFi manager
 *
 * This function initializes the WiFi hardware and event system.
 * Must be called before any other WiFi manager functions.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Initialize SoftAP mode
 *
 * Creates and configures a SoftAP with SSID based on device MAC address.
 *
 * @return esp_netif_t pointer to the AP network interface
 */
esp_netif_t* wifi_init_softap(void);

/**
 * @brief Initialize Station mode
 *
 * Configures the device as a WiFi station using compile-time SSID/password.
 *
 * @return esp_netif_t pointer to the STA network interface
 */
esp_netif_t* wifi_init_sta(void);

/**
 * @brief Set DNS address for SoftAP
 *
 * Configures the SoftAP to use the DNS server from the STA interface.
 *
 * @param esp_netif_ap Pointer to the AP network interface
 * @param esp_netif_sta Pointer to the STA network interface
 */
void wifi_set_dns_addr(esp_netif_t* esp_netif_ap, esp_netif_t* esp_netif_sta);

/**
 * @brief Register WiFi event handlers
 *
 * Registers event handlers for WiFi and IP events.
 *
 * @param event_group Event group handle for signaling connection status
 * @return ESP_OK on success
 */
esp_err_t wifi_register_event_handlers(EventGroupHandle_t event_group);

/**
 * @brief Unregister WiFi event handlers
 *
 * Unregisters previously registered event handlers.
 *
 * @return ESP_OK on success
 */
esp_err_t wifi_unregister_event_handlers(void);

/**
 * @brief Get the WiFi event group handle
 *
 * @return EventGroupHandle_t pointer to the WiFi event group
 */
EventGroupHandle_t wifi_get_event_group(void);

/**
 * @brief Get the AP network interface
 *
 * @return esp_netif_t pointer to the AP network interface
 */
esp_netif_t* wifi_get_ap_netif(void);

/**
 * @brief Get the STA network interface
 *
 * @return esp_netif_t pointer to the STA network interface
 */
esp_netif_t* wifi_get_sta_netif(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H
