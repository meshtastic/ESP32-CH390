/*
 * SPDX-FileCopyrightText: 2024-2026 Thomas Goettgens and contributors
 * SPDX-License-Identifier: LGPL-3.0-or-later
 *
 * This file is part of the ESP32-CH390 Arduino library.
 *
 * This library is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details. You should have received a copy
 * of the GNU Lesser General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

#include "ESP32_CH390.h"
#include "WiFiGeneric.h"

#include "esp_idf_version.h"
#include "esp_system.h"
#include <string.h>

#if __has_include("esp_mac.h")
#include "esp_mac.h"
#elif __has_include("esp_wifi.h")
#include "esp_wifi.h"
#endif

#if __has_include("esp_eth_netif_glue.h")
#include "esp_eth_netif_glue.h"
#endif

#include "driver/spi_master.h"
#include "esp_eth_mac_ch390_arduino.h"
#include "esp_eth_phy_ch390_arduino.h"

ESP32_CH390 CH390;

#define CH390_PHY_BMCR_REG 0x00
#define CH390_PHY_BMSR_REG 0x01
#define CH390_PHY_IDR1_REG 0x02
#define CH390_PHY_IDR2_REG 0x03
#define CH390_PHY_ANAR_REG 0x04
#define CH390_PHY_ANLPAR_REG 0x05
#define CH390_PHY_ANER_REG 0x06

#define CH390_PHY_ID1 0x7371
#define CH390_PHY_ID2 0x9011

ESP32_CH390::ESP32_CH390() {
  eth_handle = nullptr;
  eth_netif = nullptr;
  initialized = false;
  dhcp_enabled = true;
  memset(&ch390_config, 0, sizeof(ch390_config));
}

ESP32_CH390::~ESP32_CH390() { end(); }

bool ESP32_CH390::begin(ch390_config_t cfg) {
  if (initialized) {
    return true;
  }

  ch390_config = cfg;

  if (ch390_config.spi_cs_gpio == 0)
    ch390_config.spi_cs_gpio = 5;
  if (ch390_config.spi_mosi_gpio == 0)
    ch390_config.spi_mosi_gpio = 23;
  if (ch390_config.spi_miso_gpio == 0)
    ch390_config.spi_miso_gpio = 19;
  if (ch390_config.spi_sck_gpio == 0)
    ch390_config.spi_sck_gpio = 18;
  if (ch390_config.spi_clock_mhz == 0)
    ch390_config.spi_clock_mhz = 20;
  if (ch390_config.spi_host == 0)
    ch390_config.spi_host = 1; // HSPI_HOST
  if (ch390_config.int_gpio == 0)
    ch390_config.int_gpio = 4;

  return initializeEthernet();
}

bool ESP32_CH390::begin(int reset_gpio, int int_gpio, int mdc_gpio,
                        int mdio_gpio, int phy_addr) {
  ch390_config_t cfg = CH390_DEFAULT_CONFIG();
  cfg.reset_gpio = reset_gpio;
  cfg.int_gpio = int_gpio;
  cfg.mdc_gpio = mdc_gpio;
  cfg.mdio_gpio = mdio_gpio;
  cfg.phy_addr = phy_addr;

  return begin(cfg);
}

void ESP32_CH390::end() {
  if (!initialized)
    return;

  deinitializeEthernet();
  initialized = false;
}

bool ESP32_CH390::config(IPAddress local_ip, IPAddress gateway,
                         IPAddress subnet, IPAddress dns1, IPAddress dns2) {
  if (!initialized || !eth_netif)
    return false;

  esp_netif_dhcpc_stop(eth_netif);
  dhcp_enabled = false;

  esp_netif_ip_info_t ip_info;
  ip_info.ip.addr = static_cast<uint32_t>(local_ip);
  ip_info.gw.addr = static_cast<uint32_t>(gateway);
  ip_info.netmask.addr = static_cast<uint32_t>(subnet);

  esp_err_t ret = esp_netif_set_ip_info(eth_netif, &ip_info);
  if (ret != ESP_OK) {
    return false;
  }

  if (dns1 != INADDR_NONE) {
    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = static_cast<uint32_t>(dns1);
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_MAIN, &dns_info);
  }

  if (dns2 != INADDR_NONE) {
    esp_netif_dns_info_t dns_info;
    dns_info.ip.u_addr.ip4.addr = static_cast<uint32_t>(dns2);
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_BACKUP, &dns_info);
  }

  return true;
}

bool ESP32_CH390::enableDHCP() {
  if (!initialized || !eth_netif)
    return false;

  esp_err_t ret = esp_netif_dhcpc_start(eth_netif);
  if (ret == ESP_OK || ret == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
    dhcp_enabled = true;
    return true;
  }

  return false;
}

bool ESP32_CH390::disableDHCP() {
  if (!initialized || !eth_netif)
    return false;

  esp_err_t ret = esp_netif_dhcpc_stop(eth_netif);
  if (ret == ESP_OK || ret == ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
    dhcp_enabled = false;
    return true;
  }

  return false;
}

bool ESP32_CH390::isConnected() {
  if (!initialized || !eth_netif)
    return false;

  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
    return ip_info.ip.addr != 0;
  }
  return false;
}

IPAddress ESP32_CH390::localIP() {
  if (!initialized || !eth_netif)
    return IPAddress(0, 0, 0, 0);

  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
    return IPAddress(ip_info.ip.addr);
  }
  return IPAddress(0, 0, 0, 0);
}

IPAddress ESP32_CH390::subnetMask() {
  if (!initialized || !eth_netif)
    return IPAddress(0, 0, 0, 0);

  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
    return IPAddress(ip_info.netmask.addr);
  }
  return IPAddress(0, 0, 0, 0);
}

IPAddress ESP32_CH390::gatewayIP() {
  if (!initialized || !eth_netif)
    return IPAddress(0, 0, 0, 0);

  esp_netif_ip_info_t ip_info;
  if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
    return IPAddress(ip_info.gw.addr);
  }
  return IPAddress(0, 0, 0, 0);
}

IPAddress ESP32_CH390::dnsIP(uint8_t dns_no) {
  if (!initialized || !eth_netif)
    return IPAddress(0, 0, 0, 0);

  esp_netif_dns_info_t dns_info;
  esp_netif_dns_type_t dns_type =
      (dns_no == 0) ? ESP_NETIF_DNS_MAIN : ESP_NETIF_DNS_BACKUP;

  if (esp_netif_get_dns_info(eth_netif, dns_type, &dns_info) == ESP_OK &&
      dns_info.ip.type == ESP_IPADDR_TYPE_V4) {
    return IPAddress(dns_info.ip.u_addr.ip4.addr);
  }
  return IPAddress(0, 0, 0, 0);
}

String ESP32_CH390::macAddress() {
  if (!initialized || !eth_handle)
    return String("");

  uint8_t mac[6];
  if (esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac) == ESP_OK) {
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
             mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(mac_str);
  }
  return String("");
}

uint8_t *ESP32_CH390::macAddressBytes() {
  static uint8_t mac[6];
  if (!initialized || !eth_handle) {
    memset(mac, 0, 6);
    return mac;
  }

#if CH390_HAS_ETH_CORE
  if (esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac) != ESP_OK) {
    memset(mac, 0, 6);
  }
#else
  memset(mac, 0, 6);
#endif
  return mac;
}

bool ESP32_CH390::linkUp() {
  if (!initialized || !eth_handle)
    return false;

#if CH390_HAS_ETH_CORE
  if (eth_netif) {
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {

      return ip_info.ip.addr != 0;
    }
  }

  return initialized;
#endif
  return false;
}

int ESP32_CH390::linkSpeed() {
  if (!initialized || !eth_handle)
    return 0;

#if CH390_HAS_ETH_CORE
  eth_speed_t speed;
  esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_CMD_G_SPEED, &speed);
  if (ret == ESP_OK) {
    return (speed == ETH_SPEED_10M) ? 10 : 100;
  } else {

    return 100;
  }
#endif
  return 0;
}

bool ESP32_CH390::fullDuplex() {
  if (!initialized || !eth_handle)
    return false;

#if CH390_HAS_ETH_CORE
  eth_duplex_t duplex;
  esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex);
  if (ret == ESP_OK) {
    return duplex == ETH_DUPLEX_FULL;
  } else {

    return true;
  }
#endif
  return false;
}

bool ESP32_CH390::setMACAddress(const uint8_t *mac) {
  if (!initialized || !eth_handle || !mac)
    return false;

#if CH390_HAS_ETH_CORE
  esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, (void *)mac);
  if (ret == ESP_OK) {
    memcpy(ch390_config.mac_addr, mac, 6);
    return true;
  }
#endif
  return false;
}

bool ESP32_CH390::setMACAddress(const char *mac) {
  if (!mac)
    return false;

  uint8_t mac_bytes[6];
  int values[6];

  if (sscanf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", &values[0], &values[1],
             &values[2], &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) {
      mac_bytes[i] = (uint8_t)values[i];
    }
    return setMACAddress(mac_bytes);
  }

  return false;
}

bool ESP32_CH390::setHostname(const char *hostname) {
  if (!initialized || !eth_netif || !hostname)
    return false;

  esp_err_t ret = esp_netif_set_hostname(eth_netif, hostname);
  return ret == ESP_OK;
}

const char *ESP32_CH390::getHostname() {
  if (!initialized || !eth_netif)
    return nullptr;

  const char *hostname = nullptr;
  esp_netif_get_hostname(eth_netif, &hostname);
  return hostname;
}

uint32_t ESP32_CH390::readPHY(uint8_t reg) {
  if (!initialized || !eth_handle)
    return 0;

#if CH390_HAS_ETH_CORE
#ifdef ETH_CMD_READ_PHY_REG
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

  uint32_t value = 0;
  if (eth_ioctl_4param(eth_handle, ETH_CMD_READ_PHY_REG, &reg, &value) ==
      ESP_OK) {
    return value;
  }
#else

  struct {
    uint32_t reg_addr;
    uint32_t reg_value;
  } phy_reg = {reg, 0};

  if (eth_ioctl_3param(eth_handle, ETH_CMD_READ_PHY_REG, &phy_reg) == ESP_OK) {
    return phy_reg.reg_value;
  }
#endif
#endif
#endif
  return 0;
}

bool ESP32_CH390::writePHY(uint8_t reg, uint32_t value) {
  if (!initialized || !eth_handle)
    return false;

#if CH390_HAS_ETH_CORE
#ifdef ETH_CMD_WRITE_PHY_REG
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

  esp_err_t ret =
      eth_ioctl_4param(eth_handle, ETH_CMD_WRITE_PHY_REG, &reg, &value);
  return ret == ESP_OK;
#else

  struct {
    uint32_t reg_addr;
    uint32_t reg_value;
  } phy_reg = {reg, value};

  esp_err_t ret = eth_ioctl_3param(eth_handle, ETH_CMD_WRITE_PHY_REG, &phy_reg);
  return ret == ESP_OK;
#endif
#endif
#endif
  return false;
}

bool ESP32_CH390::initializeEthernet() {
#if !CH390_HAS_ETH_CORE
  return false;
#endif

  esp_err_t err = esp_netif_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }

  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return false;
  }

  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  eth_netif = esp_netif_new(&netif_cfg);

  esp_eth_mac_t *mac = createMACDriver();
  esp_eth_phy_t *phy = createPHYDriver();

  if (!mac || !phy) {
    return false;
  }

  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

  esp_err_t ret = esp_eth_driver_install(&eth_config, &eth_handle);
  if (ret != ESP_OK) {
    return false;
  }

#if __has_include("esp_eth_netif_glue.h")
  void *glue = esp_eth_new_netif_glue(eth_handle);
  if (!glue) {
    return false;
  }
  ret = esp_netif_attach(eth_netif, glue);
  if (ret != ESP_OK) {
    return false;
  }
#else
  return false;
#endif

  err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                   &eth_event_handler, this);
  if (err != ESP_OK) {
    return false;
  }

  err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                   &got_ip_event_handler, this);
  if (err != ESP_OK) {
    return false;
  }

  ret = esp_eth_start(eth_handle);
  if (ret != ESP_OK) {
    return false;
  }

  initialized = true;
  return true;
}

void ESP32_CH390::deinitializeEthernet() {
  if (eth_handle) {
    esp_eth_stop(eth_handle);
    esp_eth_driver_uninstall(eth_handle);
    eth_handle = nullptr;
  }

  if (eth_netif) {
    esp_netif_destroy(eth_netif);
    eth_netif = nullptr;
  }

  esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler);
  esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                               &got_ip_event_handler);
}

esp_eth_mac_t *ESP32_CH390::createMACDriver() {
#if !CH390_HAS_ETH_CORE
  return nullptr;
#endif

  if (ch390_config.use_internal_ethernet) {
    return nullptr;
  }

  spi_bus_config_t buscfg = {
      .mosi_io_num = ch390_config.spi_mosi_gpio,
      .miso_io_num = ch390_config.spi_miso_gpio,
      .sclk_io_num = ch390_config.spi_sck_gpio,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
  };

  spi_host_device_t spi_host = (spi_host_device_t)ch390_config.spi_host;
  esp_err_t ret = spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    return nullptr;
  }

  static spi_device_interface_config_t devcfg = {};
  devcfg.command_bits = 1; // CH390 SPI frame: 1 opcode bit + 7 address bits
  devcfg.address_bits = 7;
  devcfg.mode = 0;
  devcfg.clock_speed_hz = ch390_config.spi_clock_mhz * 1000 * 1000;
  devcfg.spics_io_num = ch390_config.spi_cs_gpio;
  devcfg.queue_size = 20;

  eth_ch390_config_t ch390_spi_config =
      ETH_CH390_DEFAULT_CONFIG(spi_host, &devcfg);
  ch390_spi_config.int_gpio_num = ch390_config.int_gpio;

  if (ch390_config.mac_addr[0] != 0 || ch390_config.mac_addr[1] != 0 ||
      ch390_config.mac_addr[2] != 0 || ch390_config.mac_addr[3] != 0 ||
      ch390_config.mac_addr[4] != 0 || ch390_config.mac_addr[5] != 0) {
    ch390_spi_config.custom_mac_addr = ch390_config.mac_addr;
  }

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.sw_reset_timeout_ms = 100;
  mac_config.rx_task_stack_size = 2048;
  mac_config.rx_task_prio = 15;

  esp_eth_mac_t *mac =
      esp_eth_mac_new_ch390_arduino(&ch390_spi_config, &mac_config);

  return mac;
}

esp_eth_phy_t *ESP32_CH390::createPHYDriver() {
#if !CH390_HAS_ETH_CORE
  return nullptr;
#endif

  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = ch390_config.phy_addr;
  phy_config.reset_gpio_num = ch390_config.reset_gpio;

  esp_eth_phy_t *phy = esp_eth_phy_new_ch390_arduino(&phy_config);

  return phy;
}

void ESP32_CH390::eth_event_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data) {
  arduino_event_t arduino_event;
  memset(&arduino_event, 0, sizeof(arduino_event_t));

  switch (event_id) {
  case ETHERNET_EVENT_START:
    arduino_event.event_id = ARDUINO_EVENT_ETH_START;
    WiFiGenericClass::_eventCallback(&arduino_event);
    break;

  case ETHERNET_EVENT_STOP:
    arduino_event.event_id = ARDUINO_EVENT_ETH_STOP;
    WiFiGenericClass::_eventCallback(&arduino_event);
    break;

  case ETHERNET_EVENT_CONNECTED:
    arduino_event.event_id = ARDUINO_EVENT_ETH_CONNECTED;
    WiFiGenericClass::_eventCallback(&arduino_event);
    break;

  case ETHERNET_EVENT_DISCONNECTED:
    arduino_event.event_id = ARDUINO_EVENT_ETH_DISCONNECTED;
    WiFiGenericClass::_eventCallback(&arduino_event);
    break;

  default:
    break;
  }
}

void ESP32_CH390::got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                       int32_t event_id, void *event_data) {
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

  arduino_event_t arduino_event;
  memset(&arduino_event, 0, sizeof(arduino_event_t));
  arduino_event.event_id = ARDUINO_EVENT_ETH_GOT_IP;
  memcpy(&arduino_event.event_info.got_ip.ip_info, &event->ip_info,
         sizeof(esp_netif_ip_info_t));
  arduino_event.event_info.got_ip.ip_changed = event->ip_changed;
  WiFiGenericClass::_eventCallback(&arduino_event);
}