/*
 * SPDX-FileCopyrightText: 2024-2026 Thomas Goettgens and contributors
 * SPDX-License-Identifier: LGPL-3.0-or-later
 *
 * This file is part of the ESP32-CH390 Arduino library.
 * Licensed under the GNU Lesser General Public License v3.0 or later.
 * See the LICENSE file in the project root for the full license text.
 */

#ifndef ESP32_CH390_H
#define ESP32_CH390_H

#include "Arduino.h"

#ifndef ESP32
#error "ESP32-CH390 Library requires ESP32 platform"
#endif

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

#ifndef ESP_IDF_VERSION
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(4, 4, 0)
#endif

#ifndef ESP_IDF_VERSION_VAL
#define ESP_IDF_VERSION_VAL(major, minor, patch)                               \
  ((major << 16) | (minor << 8) | (patch))
#endif

#if __has_include("esp_eth.h")
#include "esp_eth.h"
#define CH390_HAS_ETH_CORE 1
#else
#define CH390_HAS_ETH_CORE 0
#endif

#if __has_include("esp_eth_driver.h")
#include "esp_eth_driver.h"
#endif

#if __has_include("esp_eth_mac.h")
#include "esp_eth_mac.h"
#endif

#if __has_include("esp_eth_phy.h")
#include "esp_eth_phy.h"
#endif

#if __has_include("esp_eth_com.h")
#include "esp_eth_com.h"
#endif

#if __has_include("lwip/ip_addr.h")
#include "lwip/ip_addr.h"
#endif

#if __has_include("lwip/inet.h")
#include "lwip/inet.h"
#endif

#if __has_include("ETH.h")
#include "ETH.h"
#define CH390_HAS_ETH_LIB 1
#else
#define CH390_HAS_ETH_LIB 0
#endif

#if CH390_HAS_ETH_CORE && !__has_include("esp_eth_com.h")
typedef int esp_eth_io_cmd_t;
#endif

// Fallback ETH_CMD_* defines removed intentionally: these command names are
// enum members in esp_eth_com.h (not macros), so the old #ifndef guards always
// fired and overrode the real enum values with fabricated numbers (e.g.
// ETH_CMD_G_DUPLEX_MODE=10 vs real value 7), producing
// "esp_eth_ioctl: unknown io command: N" at runtime. Use the framework enum.

#if __has_include("hal/eth_types.h")
#include "hal/eth_types.h"
#else
#ifndef ETH_LINK_UP
#define ETH_LINK_UP ESP_ETH_LINK_UP
#endif

#ifndef ETH_LINK_DOWN
#define ETH_LINK_DOWN ESP_ETH_LINK_DOWN
#endif

#ifndef ETH_SPEED_10M
#define ETH_SPEED_10M ESP_ETH_SPEED_10M
#endif

#ifndef ETH_SPEED_100M
#define ETH_SPEED_100M ESP_ETH_SPEED_100M
#endif

#ifndef ETH_DUPLEX_HALF
#define ETH_DUPLEX_HALF ESP_ETH_DUPLEX_HALF
#endif

#ifndef ETH_DUPLEX_FULL
#define ETH_DUPLEX_FULL ESP_ETH_DUPLEX_FULL
#endif

typedef int eth_link_t;
typedef int eth_speed_t;
typedef int eth_duplex_t;
#endif

#ifndef ETHERNET_EVENT_CONNECTED
#define ETHERNET_EVENT_CONNECTED 1
#endif

#ifndef ETHERNET_EVENT_DISCONNECTED
#define ETHERNET_EVENT_DISCONNECTED 2
#endif

#ifndef ETHERNET_EVENT_START
#define ETHERNET_EVENT_START 0
#endif

#ifndef ETHERNET_EVENT_STOP
#define ETHERNET_EVENT_STOP 3
#endif

#ifndef ETH_MAC_DEFAULT_CONFIG
#define ETH_MAC_DEFAULT_CONFIG()                                               \
  {}
#endif

#ifndef ETH_PHY_DEFAULT_CONFIG
#define ETH_PHY_DEFAULT_CONFIG()                                               \
  {}
#endif

#ifndef ETH_DEFAULT_CONFIG
#define ETH_DEFAULT_CONFIG(mac, phy)                                           \
  {                                                                            \
    .mac = mac, .phy = phy, .check_link_cb = NULL, .stack_input = NULL,        \
    .on_lowlevel_init_done = NULL, .on_lowlevel_deinit_done = NULL             \
  }
#endif

#ifndef ESP_NETIF_DEFAULT_ETH
#define ESP_NETIF_DEFAULT_ETH() ESP_NETIF_BASE_DEFAULT_ETH
#endif

#ifndef INADDR_NONE
#define INADDR_NONE ((uint32_t)0xffffffffUL)
#endif

#define CH390_VENDOR_SPECIFIC_REG_1 0x10
#define CH390_VENDOR_SPECIFIC_REG_2 0x11
#define CH390_VENDOR_SPECIFIC_REG_3 0x12
#define CH390_PHY_ADDR_DEFAULT 1
#define CH390_AUTO_NEGOTIATION_TIMEOUT 5000

// CH390 configuration structure
typedef struct {
  int reset_gpio;
  int int_gpio;
  int mdc_gpio;
  int mdio_gpio;
  int phy_addr;
  bool use_internal_ethernet;
  uint8_t mac_addr[6];

  // SPI configuration
  int spi_cs_gpio;
  int spi_mosi_gpio;
  int spi_miso_gpio;
  int spi_sck_gpio;
  int spi_clock_mhz; // default: 20
  int spi_host;      // SPI host (HSPI_HOST or VSPI_HOST)
} ch390_config_t;

// Note: This library uses the standard Arduino WiFi event system.
// Register event handlers using: WiFi.onEvent(yourCallbackFunction)

#if CH390_HAS_ETH_CORE
static inline esp_err_t eth_ioctl_3param(esp_eth_handle_t handle,
                                         esp_eth_io_cmd_t cmd, void *data) {
  return esp_eth_ioctl(handle, cmd, data);
}

static inline esp_err_t eth_ioctl_4param(esp_eth_handle_t handle,
                                         esp_eth_io_cmd_t cmd, void *in,
                                         void *out) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  return esp_eth_ioctl(handle, cmd, in, out);
#else
  return esp_eth_ioctl(handle, cmd, out ? out : in);
#endif
}
#endif

class ESP32_CH390 {
public:
  ESP32_CH390();
  ~ESP32_CH390();

  bool begin(ch390_config_t config);
  bool begin(int reset_gpio = -1, int int_gpio = -1, int mdc_gpio = 23,
             int mdio_gpio = 18, int phy_addr = CH390_PHY_ADDR_DEFAULT);
  void end();

  bool config(IPAddress local_ip, IPAddress gateway, IPAddress subnet,
              IPAddress dns1 = (uint32_t)0x00000000,
              IPAddress dns2 = (uint32_t)0x00000000);
  bool enableDHCP();
  bool disableDHCP();

  bool isConnected();
  IPAddress localIP();
  IPAddress subnetMask();
  IPAddress gatewayIP();
  IPAddress dnsIP(uint8_t dns_no = 0);
  String macAddress();
  uint8_t *macAddressBytes();

  bool linkUp();
  int linkSpeed();
  bool fullDuplex();

  bool setMACAddress(const uint8_t *mac);
  bool setMACAddress(const char *mac);
  bool setHostname(const char *hostname);
  const char *getHostname();

  uint32_t readPHY(uint8_t reg);
  bool writePHY(uint8_t reg, uint32_t value);

private:
  esp_eth_handle_t eth_handle;
  esp_netif_t *eth_netif;
  bool initialized;
  bool dhcp_enabled;
  ch390_config_t ch390_config;

  static void eth_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);
  static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data);
  bool initializeEthernet();
  void deinitializeEthernet();
  esp_eth_mac_t *createMACDriver();
  esp_eth_phy_t *createPHYDriver();
};

extern ESP32_CH390 CH390;

#define CH390_DEFAULT_CONFIG()                                                 \
  {                                                                            \
    .reset_gpio = -1, .int_gpio = 4, .mdc_gpio = -1, .mdio_gpio = -1,          \
    .phy_addr = CH390_PHY_ADDR_DEFAULT, .use_internal_ethernet = false,        \
    .mac_addr = {0}, .spi_cs_gpio = 5, .spi_mosi_gpio = 23,                    \
    .spi_miso_gpio = 19, .spi_sck_gpio = 18, .spi_clock_mhz = 20,              \
    .spi_host = 1                                                              \
  }

#endif // ESP32_CH390_H