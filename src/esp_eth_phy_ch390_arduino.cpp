/*
 * Arduino-compatible port of the CH390 PHY driver.
 * Simplified standalone version without the ESP-IDF 802.3 dependency.
 *
 * Derived from the ESP-IDF CH390 driver:
 *   SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *   SPDX-FileContributor:   2024 Sergey Kharenko
 *   Originally licensed under the Apache License, Version 2.0.
 *
 * Port and modifications:
 *   SPDX-FileCopyrightText: 2024-2026 Thomas Goettgens and contributors
 *
 * SPDX-License-Identifier: LGPL-3.0-or-later
 *
 * Upstream source:
 *   https://github.com/espressif/esp-eth-drivers/tree/master/ch390
 *
 * See LICENSE and NOTICE in the project root for details.
 */

#include "esp_eth_phy_ch390_arduino.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_eth_com.h"
#include "esp_eth_phy.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ch390.phy";

// CH390 PHY-specific registers (accessed via SPI, not MDIO)
#define CH390_PHY_BMCR 0x00   // Basic Mode Control Register
#define CH390_PHY_BMSR 0x01   // Basic Mode Status Register
#define CH390_PHY_PHYID1 0x02 // PHY Identifier 1 Register
#define CH390_PHY_PHYID2 0x03 // PHY Identifier 2 Register
#define CH390_PHY_ANAR 0x04   // Auto-Negotiation Advertisement Register
#define CH390_PHY_ANLPAR 0x05 // Auto-Negotiation Link Partner Ability Register

// BMCR bits
#define BMCR_RESET (1 << 15)
#define BMCR_LOOPBACK (1 << 14)
#define BMCR_SPEED_SELECT (1 << 13)
#define BMCR_AUTO_NEG_EN (1 << 12)
#define BMCR_POWER_DOWN (1 << 11)
#define BMCR_ISOLATE (1 << 10)
#define BMCR_RESTART_AUTO_NEG (1 << 9)
#define BMCR_DUPLEX_MODE (1 << 8)

// BMSR bits (register 0x01)
#define BMSR_100BASE_T4 (1 << 15)
#define BMSR_100BASE_TX_FULL (1 << 14)
#define BMSR_100BASE_TX_HALF (1 << 13)
#define BMSR_10BASE_T_FULL (1 << 12)
#define BMSR_10BASE_T_HALF (1 << 11)
#define BMSR_AUTO_NEG_COMPLETE (1 << 5)
#define BMSR_LINK_STATUS (1 << 2)

// ANLPAR bits (register 0x05) - Link Partner Ability
#define ANLPAR_PAUSE (1 << 10)
#define ANLPAR_100BASE_TX_FULL (1 << 8)
#define ANLPAR_100BASE_TX (1 << 7)
#define ANLPAR_10BASE_T_FULL (1 << 6)
#define ANLPAR_10BASE_T (1 << 5)

#define CH390_INFO_OUI 0x1CDC64
#define CH390_INFO_MODEL 0x01

typedef struct {
  esp_eth_phy_t parent;
  esp_eth_mediator_t *eth;
  uint32_t addr;
  uint32_t reset_timeout_ms;
  uint32_t autonego_timeout_ms;
  eth_link_t link_status;
  int reset_gpio_num;
} phy_ch390_t;

// Forward declarations
static esp_err_t ch390_autonego_ctrl(esp_eth_phy_t *phy, int cmd,
                                     bool *autonego_en_stat);

static esp_err_t ch390_update_link_duplex_speed(phy_ch390_t *ch390) {
  esp_err_t ret = ESP_OK;
  esp_eth_mediator_t *eth = ch390->eth;
  uint32_t bmsr = 0, bmcr = 0, anlpar = 0;
  eth_speed_t speed = ETH_SPEED_10M;
  eth_duplex_t duplex = ETH_DUPLEX_HALF;
  uint32_t peer_pause_ability = 0;

  if (!eth || !eth->phy_reg_read || !eth->on_state_changed) {
    ESP_LOGE(TAG, "mediator callbacks null: eth=%p reg_read=%p state=%p", eth,
             eth ? (void *)eth->phy_reg_read : NULL,
             eth ? (void *)eth->on_state_changed : NULL);
    return ESP_OK;
  }

  // BMSR Link Status bit is "latching low" per IEEE 802.3 - first read clears
  // any latched link-down state, second read returns actual current status
  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMSR, &bmsr);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMSR, &bmsr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "read BMSR failed");
    return ret;
  }

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_ANLPAR, &anlpar);
  if (ret != ESP_OK) {
    return ret;
  }

  eth_link_t link = (bmsr & BMSR_LINK_STATUS) ? ETH_LINK_UP : ETH_LINK_DOWN;

  // Only update if link status changed
  if (ch390->link_status != link) {
    if (link == ETH_LINK_UP) {
      // Read BMCR to get actual configured speed/duplex (not peer abilities)
      ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMCR, &bmcr);
      if (ret != ESP_OK) {
        return ret;
      }

      if (bmcr & BMCR_SPEED_SELECT) {
        speed = ETH_SPEED_100M;
      } else {
        speed = ETH_SPEED_10M;
      }

      if (bmcr & BMCR_DUPLEX_MODE) {

        duplex = ETH_DUPLEX_FULL;
      } else {
        duplex = ETH_DUPLEX_HALF;
      }

      ret = eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed);
      if (ret != ESP_OK) {
        return ret;
      }

      ret = eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex);
      if (ret != ESP_OK) {
        return ret;
      }

      // Check peer pause ability for flow control
      if (duplex == ETH_DUPLEX_FULL && (anlpar & ANLPAR_PAUSE)) {
        peer_pause_ability = 1;
      }
      ret = eth->on_state_changed(eth, ETH_STATE_PAUSE,
                                  (void *)peer_pause_ability);
      if (ret != ESP_OK) {
        return ret;
      }
    }

    ret = eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "change link failed");
      return ret;
    }
    ch390->link_status = link;
  }
  return ESP_OK;
}

static esp_err_t ch390_set_mediator(esp_eth_phy_t *phy,
                                    esp_eth_mediator_t *eth) {
  if (!phy) {
    ESP_LOGE(TAG, "PHY can't be null");
    return ESP_ERR_INVALID_ARG;
  }
  if (!eth) {
    ESP_LOGE(TAG, "mediator can't be null");
    return ESP_ERR_INVALID_ARG;
  }
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  ch390->eth = eth;
  return ESP_OK;
}

static esp_err_t ch390_get_link(esp_eth_phy_t *phy) {
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  return ch390_update_link_duplex_speed(ch390);
}

static esp_err_t ch390_reset(esp_eth_phy_t *phy) {
  esp_err_t ret = ESP_OK;
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  esp_eth_mediator_t *eth = ch390->eth;

  if (!eth) {
    return ESP_OK;
  }

  uint32_t bmcr;

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMCR, &bmcr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "read BMCR failed");
    return ret;
  }

  bmcr |= BMCR_RESET;
  ret = eth->phy_reg_write(eth, ch390->addr, CH390_PHY_BMCR, bmcr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "write BMCR failed");
    return ret;
  }

  // Wait for reset complete
  uint32_t timeout = 0;
  for (timeout = 0; timeout < ch390->reset_timeout_ms / 10; timeout++) {
    ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMCR, &bmcr);
    if (ret == ESP_OK && !(bmcr & BMCR_RESET)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (timeout >= ch390->reset_timeout_ms / 10) {
    ESP_LOGE(TAG, "PHY reset timeout");
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

static esp_err_t ch390_reset_hw(esp_eth_phy_t *phy) {
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;

  if (ch390->reset_gpio_num >= 0) {
    esp_rom_gpio_pad_select_gpio((gpio_num_t)ch390->reset_gpio_num);
    gpio_set_direction((gpio_num_t)ch390->reset_gpio_num, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)ch390->reset_gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)ch390->reset_gpio_num, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return ESP_OK;
}

static esp_err_t ch390_verify_id(phy_ch390_t *ch390) {
  esp_err_t ret = ESP_OK;
  esp_eth_mediator_t *eth = ch390->eth;

  if (!eth) {
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t phyid1, phyid2;

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_PHYID1, &phyid1);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_PHYID2, &phyid2);
  if (ret != ESP_OK) {
    return ret;
  }

  // Extract OUI and model from PHY ID registers
  // OUI is split across PHYID1 (bits 15:0 -> OUI bits 21:6) and PHYID2 (bits
  // 15:10 -> OUI bits 5:0)
  uint32_t oui = (phyid1 << 6) | ((phyid2 >> 10) & 0x3F);
  uint8_t model = (phyid2 >> 4) & 0x3F;

  if (oui != CH390_INFO_OUI || model != CH390_INFO_MODEL) {
    ESP_LOGE(TAG,
             "PHY ID mismatch: PHYID1=0x%04x PHYID2=0x%04x -> OUI=0x%06x "
             "model=0x%02x (expect OUI=0x%06x model=0x%02x)",
             (unsigned)phyid1, (unsigned)phyid2, (unsigned)oui, (unsigned)model,
             (unsigned)CH390_INFO_OUI, (unsigned)CH390_INFO_MODEL);
    return ESP_FAIL;
  }

  return ESP_OK;
}

static esp_err_t ch390_init(esp_eth_phy_t *phy) {
  esp_err_t ret = ESP_OK;
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;

  ret = ch390_reset_hw(phy);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ch390_reset_hw failed: %d", ret);
    return ret;
  }

  ret = ch390_reset(phy);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ch390_reset failed: %d", ret);
    return ret;
  }

  // Verify PHY ID
  ret = ch390_verify_id(ch390);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ch390_verify_id failed: %d", ret);
    return ret;
  }

  ret = ch390_autonego_ctrl(phy, 0, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ch390_autonego_ctrl failed: %d", ret);
    return ret;
  }

  return ESP_OK;
}

static esp_err_t ch390_deinit(esp_eth_phy_t *phy) { return ESP_OK; }

static esp_err_t ch390_negotiate(esp_eth_phy_t *phy) {
  return ch390_autonego_ctrl(phy, 0, NULL);
}

// Define our own enum for auto-negotiation commands
typedef enum {
  CH390_AUTONEGO_EN = 0,
  CH390_AUTONEGO_DIS = 1,
  CH390_AUTONEGO_G_STAT = 2
} ch390_autonego_cmd_t;

static esp_err_t ch390_autonego_ctrl(esp_eth_phy_t *phy, int cmd,
                                     bool *autonego_en_stat) {
  esp_err_t ret = ESP_OK;
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  esp_eth_mediator_t *eth = ch390->eth;

  // Check if mediator is set before using it
  if (!eth) {
    ESP_LOGW(TAG, "PHY autonego_ctrl called before set_mediator - mediator is "
                  "NULL, skipping");
    return ESP_OK; // Return OK to allow initialization to continue
  }

  uint32_t bmcr;

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMCR, &bmcr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "read BMCR failed");
    return ret;
  }

  switch (cmd) {
  case CH390_AUTONEGO_EN:
    // Validate: can't enable auto-negotiation while in loopback
    if (bmcr & BMCR_LOOPBACK) {
      return ESP_ERR_INVALID_STATE;
    }
    bmcr |= BMCR_AUTO_NEG_EN | BMCR_RESTART_AUTO_NEG;
    ret = eth->phy_reg_write(eth, ch390->addr, CH390_PHY_BMCR, bmcr);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "write BMCR failed");
      return ret;
    }
    if (autonego_en_stat) {
      *autonego_en_stat = true;
    }
    break;

  case CH390_AUTONEGO_DIS:
    bmcr &= ~BMCR_AUTO_NEG_EN;
    ret = eth->phy_reg_write(eth, ch390->addr, CH390_PHY_BMCR, bmcr);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "write BMCR failed");
      return ret;
    }
    if (autonego_en_stat) {
      *autonego_en_stat = false;
    }
    break;

  case CH390_AUTONEGO_G_STAT:
    if (autonego_en_stat) {
      *autonego_en_stat = !!(bmcr & BMCR_AUTO_NEG_EN);
    }
    break;

  default:
    return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}

static esp_err_t ch390_loopback(esp_eth_phy_t *phy, bool enable) {
  esp_err_t ret = ESP_OK;
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  esp_eth_mediator_t *eth = ch390->eth;

  if (!eth) {
    return ESP_ERR_INVALID_STATE;
  }

  // Validate: can't enable loopback while auto-negotiation is enabled
  if (enable) {
    bool auto_nego_en = false;
    ret = ch390_autonego_ctrl(phy, CH390_AUTONEGO_G_STAT, &auto_nego_en);
    if (ret != ESP_OK) {
      return ret;
    }
    if (auto_nego_en) {
      return ESP_ERR_INVALID_STATE;
    }
  }

  uint32_t bmcr;

  ret = eth->phy_reg_read(eth, ch390->addr, CH390_PHY_BMCR, &bmcr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "read BMCR failed");
    return ret;
  }

  if (enable) {
    bmcr |= BMCR_LOOPBACK;
  } else {
    bmcr &= ~BMCR_LOOPBACK;
  }

  ret = eth->phy_reg_write(eth, ch390->addr, CH390_PHY_BMCR, bmcr);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "write BMCR failed");
    return ret;
  }

  return ESP_OK;
}

static esp_err_t ch390_del(esp_eth_phy_t *phy) {
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  free(ch390);
  return ESP_OK;
}

// Stub implementations for optional callbacks that may be called even in
// ESP-IDF 4.x
static esp_err_t ch390_pwrctl(esp_eth_phy_t *phy, bool enable) {
  ESP_LOGD(TAG, "PHY pwrctl called (enable=%d) - not implemented", enable);
  return ESP_OK;
}

static esp_err_t ch390_get_addr(esp_eth_phy_t *phy, uint32_t *addr) {
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  *addr = ch390->addr;
  return ESP_OK;
}

static esp_err_t ch390_set_addr(esp_eth_phy_t *phy, uint32_t addr) {
  phy_ch390_t *ch390 = (phy_ch390_t *)phy;
  ch390->addr = addr;
  return ESP_OK;
}

static esp_err_t ch390_advertise_pause_ability(esp_eth_phy_t *phy,
                                               uint32_t ability) {
  ESP_LOGD(TAG, "PHY advertise_pause_ability called - not implemented");
  return ESP_OK;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static esp_err_t ch390_custom_ioctl(esp_eth_phy_t *phy, uint32_t cmd,
                                    void *data) {
  ESP_LOGI(TAG, "PHY custom_ioctl called (cmd=%u) - not implemented", cmd);
  return ESP_ERR_NOT_SUPPORTED;
}
#endif

esp_eth_phy_t *esp_eth_phy_new_ch390_arduino(const eth_phy_config_t *config) {
  phy_ch390_t *ch390 = NULL;

  if (!config) {
    ESP_LOGE(TAG, "config can't be null");
    return NULL;
  }

  ch390 = (phy_ch390_t *)calloc(1, sizeof(phy_ch390_t));
  if (!ch390) {
    ESP_LOGE(TAG, "no mem for PHY instance");
    return NULL;
  }

  ch390->addr = config->phy_addr;
  ch390->reset_timeout_ms = config->reset_timeout_ms;
  ch390->autonego_timeout_ms = config->autonego_timeout_ms;
  ch390->reset_gpio_num = config->reset_gpio_num;
  ch390->link_status = ETH_LINK_DOWN;

  ch390->parent.set_mediator = ch390_set_mediator;
  ch390->parent.reset = ch390_reset;
  ch390->parent.reset_hw = ch390_reset_hw;
  ch390->parent.init = ch390_init;
  ch390->parent.deinit = ch390_deinit;
  ch390->parent.negotiate = ch390_negotiate;
  ch390->parent.get_link = ch390_get_link;
  ch390->parent.pwrctl = ch390_pwrctl;
  ch390->parent.set_addr = ch390_set_addr;
  ch390->parent.get_addr = ch390_get_addr;
  ch390->parent.advertise_pause_ability = ch390_advertise_pause_ability;
  ch390->parent.loopback = ch390_loopback;
  ch390->parent.del = ch390_del;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  ch390->parent.autonego_ctrl = ch390_autonego_ctrl;
  ch390->parent.custom_ioctl = ch390_custom_ioctl;
#endif

  return &ch390->parent;
}
