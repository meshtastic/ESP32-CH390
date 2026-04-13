/*
 * Arduino-compatible port of the CH390 PHY driver.
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

#include "esp_eth_com.h"
#include "esp_eth_phy.h"

/**
 * @brief Create a PHY instance of CH390
 *
 * @param[in] config: configuration of PHY
 *
 * @return
 *      - instance: create PHY instance successfully
 *      - NULL: create PHY instance failed because some error occurred
 */
esp_eth_phy_t *esp_eth_phy_new_ch390_arduino(const eth_phy_config_t *config);

#ifdef __cplusplus
}
#endif
