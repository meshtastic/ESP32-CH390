/*
 * Arduino-compatible port of the CH390 SPI Ethernet MAC driver.
 *
 * Derived from the ESP-IDF CH390 driver:
 *   SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *   Originally licensed under the Apache License, Version 2.0.
 *
 * Port and modifications:
 *   SPDX-FileCopyrightText: 2024-2026 Thomas Goettgens and contributors
 *
 * SPDX-License-Identifier: LGPL-3.0-or-later
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"
#include "esp_eth_com.h"
#include "esp_eth_mac.h"

/**
 * @brief CH390 specific configuration
 */
typedef struct {
  int int_gpio_num; /*!< Interrupt GPIO number, set -1 to not use interrupt */
  uint32_t poll_period_ms; /*!< Period in ms to poll RX status when interrupt is
                              not used */
  spi_host_device_t spi_host;                /*!< SPI peripheral */
  spi_device_interface_config_t *spi_devcfg; /*!< SPI device configuration */
  uint8_t *custom_mac_addr; /*!< Custom MAC address (6 bytes), set to NULL to
                               use default */
} eth_ch390_config_t;

/**
 * @brief Default CH390 specific configuration
 */
#define ETH_CH390_DEFAULT_CONFIG(spi_host, spi_devcfg_p)                       \
  {                                                                            \
    .int_gpio_num = 4, .poll_period_ms = 0, .spi_host = spi_host,              \
    .spi_devcfg = spi_devcfg_p, .custom_mac_addr = NULL,                       \
  }

/**
 * @brief Create CH390 Ethernet MAC instance
 *
 * @param ch390_config: CH390 specific configuration
 * @param mac_config: Ethernet MAC configuration
 *
 * @return
 *      - instance: create MAC instance successfully
 *      - NULL: create MAC instance failed because some error occurred
 */
esp_eth_mac_t *
esp_eth_mac_new_ch390_arduino(const eth_ch390_config_t *ch390_config,
                              const eth_mac_config_t *mac_config);

#ifdef __cplusplus
}
#endif
