/*
 * Arduino-compatible port of the CH390 SPI Ethernet MAC driver.
 *
 * Derived from the ESP-IDF CH390 driver:
 *   SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *   SPDX-FileContributor:   2024-2025 Sergey Kharenko
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
 * The Apache-2.0 license permits redistribution of derivative works under
 * the GNU LGPL v3; the combined work in this file is distributed under
 * LGPL-3.0-or-later. See LICENSE and NOTICE in the project root for details.
 */

#include "esp_eth_mac_ch390_arduino.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_eth_com.h"
#include "esp_eth_mac.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ch390.mac";

#define GOTO_ON_ERROR(expr, label)                                             \
  do {                                                                         \
    esp_err_t err_rc_ = (expr);                                                \
    if (err_rc_ != ESP_OK) {                                                   \
      ret = err_rc_;                                                           \
      goto label;                                                              \
    }                                                                          \
  } while (0)

#define GOTO_ON_FALSE(cond, err_code, label)                                   \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ret = (err_code);                                                        \
      goto label;                                                              \
    }                                                                          \
  } while (0)

// CH390 SPI Commands
#define CH390_SPI_WR 1 // SPI write opcode (1 bit, placed in trans.cmd)
#define CH390_SPI_RD 0 // SPI read opcode

// CH390 Registers
#define CH390_NCR 0x00     // Network Control Register
#define CH390_NSR 0x01     // Network Status Register
#define CH390_TCR 0x02     // TX Control Register
#define CH390_WCR 0x03     // Wakeup Control Register
#define CH390_RCR 0x05     // RX Control Register
#define CH390_RSR 0x06     // RX Status Register
#define CH390_EPCR 0x0B    // EEPROM & PHY Control Register
#define CH390_EPAR 0x0C    // EEPROM & PHY Address Register
#define CH390_EPDRL 0x0D   // EEPROM & PHY Low Data Register
#define CH390_EPDRH 0x0E   // EEPROM & PHY High Data Register
#define CH390_PAR 0x10     // Physical Address Register (6 bytes)
#define CH390_MAR 0x16     // Multicast Address Register (8 bytes)
#define CH390_BCASTCR 0x53 // Broadcast Control Register
#define CH390_GPCR 0x1E    // General Purpose Control Register
#define CH390_GPR 0x1F     // General Purpose Register
#define CH390_TCR2 0x2D    // TX Control Register 2
#define CH390_BPTR 0x08    // Backoff Preset Time Register
#define CH390_FCTR 0x09    // Flow Control Threshold Register
#define CH390_FCR 0x0A     // Flow Control Register
#define CH390_TCSCR 0x31   // TX Checksum Control Register
#define CH390_RCSCSR 0x32  // RX Checksum Control Status Register
#define CH390_MPTRCR 0x55  // MAC Pause Time Register
#define CH390_ISR 0x7E     // Interrupt Status Register
#define CH390_IMR 0x7F     // Interrupt Mask Register
#define CH390_INTCR 0x39   // Interrupt Control Register
#define CH390_INTCKCR 0x54 // Interrupt Clock Control Register
#define CH390_RLENCR 0x52  // RX Length Control Register
#define CH390_VIDL 0x28    // Vendor ID Low byte
#define CH390_VIDH 0x29    // Vendor ID High byte
#define CH390_PIDL 0x2A    // Product ID Low byte
#define CH390_PIDH 0x2B    // Product ID High byte
#define CH390_MRRH 0x75    // MAC RX Result High byte
#define CH390_MRRL 0x74    // MAC RX Result Low byte
#define CH390_TXPLL 0x7C   // TX Packet Length Low byte
#define CH390_TXPLH 0x7D   // TX Packet Length High byte
#define CH390_MWCMDX 0x76  // MAC Write Command Extended
#define CH390_MWCMD 0x78   // MAC Write Command
#define CH390_MRCMDX 0x70  // MAC Read Command Extended
#define CH390_MRCMD 0x72   // MAC Read Command

// NCR Register bits
#define NCR_RST (1 << 0)    // Software reset
#define NCR_LBK (3 << 1)    // Loopback mode
#define NCR_FDX (1 << 3)    // Full duplex
#define NCR_WAKEEN (1 << 6) // Wakeup enable

// NSR Register bits
#define NSR_RXOV (1 << 1)   // RX FIFO overflow
#define NSR_TX1END (1 << 2) // TX packet 1 complete
#define NSR_TX2END (1 << 3) // TX packet 2 complete
#define NSR_WAKEST (1 << 5) // Wakeup event
#define NSR_LINKST (1 << 6) // Link status
#define NSR_SPEED (1 << 7)  // Speed (0=10M, 1=100M)

// TCR Register bits
#define TCR_TXREQ (1 << 0) // TX request

// TCR2 Register bits
#define TCR2_RLCP (1 << 0) // Retry late collision packet

// TCSCR Register bits
#define TCSCR_IPCSE (1 << 0)  // IP checksum generation
#define TCSCR_TCPCSE (1 << 1) // TCP checksum generation
#define TCSCR_UDPCSE (1 << 2) // UDP checksum generation

// RLENCR Register bits
#define RLENCR_RXLEN_EN (1 << 7) // RX length limit enable
#define RLENCR_RXLEN_DEFAULT 24  // Default length (1536 bytes = 64*24)

// RCR Register bits
#define RCR_RXEN (1 << 0)     // RX enable
#define RCR_PRMSC (1 << 1)    // Promiscuous mode
#define RCR_RUNT (1 << 2)     // Accept runt packet
#define RCR_ALL (1 << 3)      // Accept all multicast
#define RCR_DIS_CRC (1 << 4)  // Discard CRC error packet
#define RCR_DIS_LONG (1 << 5) // Discard long packet

// RSR Register bits
#define RSR_AE (1 << 2)                // Alignment error
#define RSR_MF (1 << 6)                // Multicast frame
#define RSR_RF (1 << 7)                // Runt frame
#define RSR_ERR_MASK (RSR_AE | RSR_RF) // Error mask

// EPCR Register bits
#define EPCR_ERRE (1 << 0)  // EEPROM/PHY access status (busy when 1)
#define EPCR_ERPRW (1 << 1) // EEPROM/PHY write access
#define EPCR_ERPRR (1 << 2) // EEPROM/PHY read access
#define EPCR_EPOS (1 << 3)  // EEPROM/PHY operation select
#define EPCR_WEP (1 << 4)   // Write enable protection

#define MPTRCR_RST_RX (1 << 0) // Reset RX FIFO pointer

#define ISR_PR (1 << 0)  // Packet received
#define ISR_PT (1 << 1)  // Packet transmitted
#define ISR_ROS (1 << 2) // RX over size
#define ISR_ROO (1 << 3) // RX overrun
#define ISR_CLR_STATUS                                                         \
  (ISR_PR | ISR_PT | ISR_ROS | ISR_ROO) // Clear status bits

#define IMR_PAR (1 << 7) // PHY auto-negotiation result
#define IMR_PRI (1 << 0) // Packet received interrupt

#define FCR_FLOW_ENABLE (0x39)
#define FCTR_HWOT(x) (((x)&0x0F) << 4)
#define FCTR_LWOT(x) ((x)&0x0F)

// PHY register base address
#define CH390_PHY 0x40 // Internal PHY base address

#define CH390_TX_FIFO_SIZE 3072
#define CH390_RX_FIFO_SIZE 16384

#define CH390_PKT_RDY 0x01
#define CH390_PKT_ERR 0xFE

#define CH390_SPI_LOCK_TIMEOUT_MS 50

typedef struct {
  uint8_t flag; // new ?
  uint8_t status;
  uint8_t length_low;
  uint8_t length_high;
  // uint8_t reserved;
} ch390_rx_header_t;

typedef struct {
  esp_eth_mac_t parent;
  esp_eth_mediator_t *eth;
  spi_device_handle_t spi_hdl;
  SemaphoreHandle_t spi_lock; // new ?
  TaskHandle_t rx_task_hdl;
  uint32_t sw_reset_timeout_ms;
  int int_gpio_num;
  uint8_t addr[6];
  bool packets_remain;
  bool flow_ctrl_enabled;
  uint8_t *rx_buffer; // new ?
} emac_ch390_t;

// Forward declarations
static esp_err_t ch390_write_reg(emac_ch390_t *emac, uint8_t reg,
                                 uint8_t value);
static esp_err_t ch390_read_reg(emac_ch390_t *emac, uint8_t reg,
                                uint8_t *value);

static esp_err_t emac_ch390_start(esp_eth_mac_t *mac);
static esp_err_t emac_ch390_stop(esp_eth_mac_t *mac);

static inline bool ch390_spi_lock(emac_ch390_t *emac) {
  return xSemaphoreTake(emac->spi_lock,
                        pdMS_TO_TICKS(CH390_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline void ch390_spi_unlock(emac_ch390_t *emac) {
  xSemaphoreGive(emac->spi_lock);
}

static esp_err_t ch390_write_reg(emac_ch390_t *emac, uint8_t reg,
                                 uint8_t value) {
  esp_err_t ret = ESP_OK;
  spi_transaction_t trans = {.flags = SPI_TRANS_USE_TXDATA,
                             .cmd = CH390_SPI_WR,
                             .addr = (uint64_t)reg,
                             .length = 8,
                             .tx_data = {value}};
  if (!ch390_spi_lock(emac)) {
    return ESP_ERR_TIMEOUT;
  }
  ret = spi_device_polling_transmit(emac->spi_hdl, &trans);
  ch390_spi_unlock(emac);
  return ret;
}

static esp_err_t ch390_read_reg(emac_ch390_t *emac, uint8_t reg,
                                uint8_t *value) {
  esp_err_t ret = ESP_OK;
  spi_transaction_t trans = {
      .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
      .cmd = CH390_SPI_RD,
      .addr = (uint64_t)reg,
      .length = 8,
  };
  if (!ch390_spi_lock(emac)) {
    return ESP_ERR_TIMEOUT;
  }
  ret = spi_device_polling_transmit(emac->spi_hdl, &trans);
  ch390_spi_unlock(emac);
  *value = trans.rx_data[0];
  return ret;
}

static esp_err_t ch390_write_mem(emac_ch390_t *emac, uint8_t *buffer,
                                 uint32_t len) {
  esp_err_t ret = ESP_OK;
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));
  trans.cmd = CH390_SPI_WR;
  trans.addr = CH390_MWCMD;
  trans.length = len * 8;
  trans.tx_buffer = buffer;
  if (!ch390_spi_lock(emac)) {
    return ESP_ERR_TIMEOUT;
  }
  ret = spi_device_polling_transmit(emac->spi_hdl, &trans);
  ch390_spi_unlock(emac);
  return ret;
}

static esp_err_t ch390_read_mem(emac_ch390_t *emac, uint8_t *buffer,
                                uint32_t len) {
  esp_err_t ret = ESP_OK;
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(trans));
  trans.cmd = CH390_SPI_RD;
  trans.addr = CH390_MRCMD;
  trans.length = len * 8;
  trans.rx_buffer = buffer;
  if (!ch390_spi_lock(emac)) {
    return ESP_ERR_TIMEOUT;
  }
  ret = spi_device_polling_transmit(emac->spi_hdl, &trans);
  ch390_spi_unlock(emac);
  return ret;
}

static esp_err_t ch390_set_mediator(esp_eth_mac_t *mac,
                                    esp_eth_mediator_t *eth) {
  if (!mac) {
    ESP_LOGE(TAG, "MAC can't be null");
    return ESP_ERR_INVALID_ARG;
  }
  if (!eth) {
    ESP_LOGE(TAG, "mediator can't be null");
    return ESP_ERR_INVALID_ARG;
  }
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  emac->eth = eth;
  return ESP_OK;
}

static esp_err_t ch390_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr,
                                     uint32_t phy_reg, uint32_t reg_value) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  uint8_t epcr_val;
  uint32_t timeout = 0;
  uint8_t epar_val;
  uint8_t epcr_cmd;

  epar_val = CH390_PHY | (phy_reg & 0x1F);
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_EPAR, epar_val), err, TAG,
                    "write EPAR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_EPDRL, reg_value & 0xFF), err,
                    TAG, "write EPDRL failed");
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_EPDRH, (reg_value >> 8) & 0xFF),
                    err, TAG, "write EPDRH failed");

  epcr_cmd = EPCR_EPOS | EPCR_ERPRW;
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_EPCR, epcr_cmd), err, TAG,
                    "write EPCR failed");

  timeout = 0;
  do {
    ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_EPCR, &epcr_val), err, TAG,
                      "read EPCR failed");
    if (!(epcr_val & EPCR_ERRE)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    timeout++;
  } while (timeout < 100);

  if (timeout >= 100) {
    ret = ESP_ERR_TIMEOUT;
    goto err;
  }

  return ESP_OK;

err:
  return ret;
}

static esp_err_t ch390_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr,
                                    uint32_t phy_reg, uint32_t *reg_value) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  uint8_t epcr_val;
  uint32_t timeout = 0;
  uint8_t epar_val;
  uint8_t epcr_cmd;
  uint8_t data_low, data_high;

  epar_val = CH390_PHY | (phy_reg & 0x1F);
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_EPAR, epar_val), err, TAG,
                    "write EPAR failed");

  epcr_cmd = EPCR_EPOS | EPCR_ERPRR;
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_EPCR, epcr_cmd), err, TAG,
                    "write EPCR failed");

  timeout = 0;
  do {
    ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_EPCR, &epcr_val), err, TAG,
                      "read EPCR failed");
    if (!(epcr_val & EPCR_ERRE)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    timeout++;
  } while (timeout < 100);

  if (timeout >= 100) {
    ret = ESP_ERR_TIMEOUT;
    goto err;
  }

  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_EPDRL, &data_low), err, TAG,
                    "read EPDRL failed");
  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_EPDRH, &data_high), err, TAG,
                    "read EPDRH failed");

  *reg_value = (data_high << 8) | data_low;
  return ESP_OK;

err:
  *reg_value = 0;
  return ret;
}

static esp_err_t ch390_set_addr(esp_eth_mac_t *mac, uint8_t *addr) {
  esp_err_t ret = ESP_OK;
  if (!addr) {
    ESP_LOGE(TAG, "addr can't be null");
    return ESP_ERR_INVALID_ARG;
  }

  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  memcpy(emac->addr, addr, 6);

  // Write MAC address to CH390
  for (int i = 0; i < 6; i++) {
    ret = ch390_write_reg(emac, CH390_PAR + i, addr[i]);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "write PAR failed");
      return ret;
    }
  }
  return ESP_OK;
}

static esp_err_t ch390_get_addr(esp_eth_mac_t *mac, uint8_t *addr) {
  if (!addr) {
    ESP_LOGE(TAG, "addr can't be null");
    return ESP_ERR_INVALID_ARG;
  }
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  memcpy(addr, emac->addr, 6);
  return ESP_OK;
}

static esp_err_t ch390_set_link(esp_eth_mac_t *mac, eth_link_t link) {
  esp_err_t ret = ESP_OK;

  switch (link) {
  case ETH_LINK_UP:
    GOTO_ON_ERROR(emac_ch390_start(mac), err);
    break;
  case ETH_LINK_DOWN:
    GOTO_ON_ERROR(emac_ch390_stop(mac), err);
    break;
  default:
    ret = ESP_ERR_INVALID_ARG;
    return ret;
  }
  return ESP_OK;
err:
  return ret;
}

static esp_err_t ch390_set_speed(esp_eth_mac_t *mac, eth_speed_t speed) {
  switch (speed) {
  case ETH_SPEED_10M:
  case ETH_SPEED_100M:
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

static esp_err_t ch390_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex) {
  switch (duplex) {
  case ETH_DUPLEX_HALF:
  case ETH_DUPLEX_FULL:
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

static esp_err_t ch390_set_promiscuous(esp_eth_mac_t *mac, bool enable) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  uint8_t rcr;
  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_RCR, &rcr), err, TAG,
                    "read RCR failed");
  if (enable) {
    rcr |= RCR_PRMSC;
  } else {
    rcr &= ~RCR_PRMSC;
  }
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_RCR, rcr), err, TAG,
                    "write RCR failed");
err:
  return ret;
}

static __attribute__((unused)) esp_err_t
ch390_enable_flow_ctrl(emac_ch390_t *emac, bool enable) {
  esp_err_t ret = ESP_OK;
  if (enable) {
    GOTO_ON_ERROR(ch390_write_reg(emac, CH390_BPTR, 0x3F), err);
    GOTO_ON_ERROR(
        ch390_write_reg(emac, CH390_FCTR, FCTR_HWOT(3) | FCTR_LWOT(8)), err);
    GOTO_ON_ERROR(ch390_write_reg(emac, CH390_FCR, FCR_FLOW_ENABLE), err);
  } else {
    GOTO_ON_ERROR(ch390_write_reg(emac, CH390_FCR, 0), err);
  }
err:
  return ret;
}

static esp_err_t ch390_drop_frame(emac_ch390_t *emac, uint16_t length) {
  esp_err_t ret = ESP_OK;
  uint8_t mrrh, mrrl;
  uint16_t addr = 0;

  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_MRRH, &mrrh), err);
  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_MRRL, &mrrl), err);

  addr = (mrrh << 8) | mrrl;
  addr += length;

  // Handle wraparound at 0x4000 boundary
  if (addr >= 0x4000) {
    addr -= 0x3400;
  }

  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_MRRH, addr >> 8), err);
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_MRRL, addr & 0xFF), err);
err:
  return ret;
}

static esp_err_t emac_ch390_start(esp_eth_mac_t *mac) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);

  // Reset RX memory pointer
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_MPTRCR, MPTRCR_RST_RX), err);

  // Clear interrupt status
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_ISR, ISR_CLR_STATUS), err);

  // Enable RX related interrupts
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_IMR, IMR_PAR | IMR_PRI), err);

  // Enable RX
  uint8_t rcr;
  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_RCR, &rcr), err);
  rcr |= RCR_RXEN;
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_RCR, rcr), err);

err:
  return ret;
}

static esp_err_t emac_ch390_stop(esp_eth_mac_t *mac) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);

  // Disable interrupts
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_IMR, 0x00), err);

  // Disable RX
  uint8_t rcr;
  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_RCR, &rcr), err);
  rcr &= ~RCR_RXEN;
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_RCR, rcr), err);

err:
  return ret;
}

static esp_err_t ch390_transmit(esp_eth_mac_t *mac, uint8_t *buf,
                                uint32_t length) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  uint8_t tcr;
  uint32_t wait = 0;

  if (length > 1522) {
    return ESP_ERR_INVALID_ARG;
  }

  // Copy data to TX memory first (chip has dual TX buffer)
  GOTO_ON_ERROR(ch390_write_mem(emac, buf, length), err);

  // Wait for previous transmit to complete (timeout ~1ms)
  do {
    GOTO_ON_ERROR(ch390_read_reg(emac, CH390_TCR, &tcr), err);
    if (!(tcr & TCR_TXREQ)) {
      break;
    }
    esp_rom_delay_us(10);
    wait += 10;
  } while (wait < 1000);

  if (tcr & TCR_TXREQ) {
    ESP_LOGE(TAG, "last transmit still in progress, cannot send.");
    return ESP_ERR_INVALID_STATE;
  }

  // Set TX length
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_TXPLL, length & 0xFF), err);
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_TXPLH, (length >> 8) & 0xFF), err);

  // Issue TX request (preserve existing TCR bits)
  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_TCR, &tcr), err);
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_TCR, tcr | TCR_TXREQ), err, TAG,
                    "write TCR failed");

err:
  return ret;
}

static esp_err_t ch390_receive(esp_eth_mac_t *mac, uint8_t *buf,
                               uint32_t *length) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  uint8_t ready;
  __attribute__((aligned(4))) ch390_rx_header_t header;
  uint16_t rx_len;

  // Dummy read to get updated status, then read actual value
  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_MRCMDX, &ready), err);
  GOTO_ON_ERROR(ch390_read_reg(emac, CH390_MRCMDX, &ready), err);

  // Check for packet error - reset MAC if detected
  if (ready & CH390_PKT_ERR) {
    emac_ch390_stop(mac);
    vTaskDelay(pdMS_TO_TICKS(1));
    emac_ch390_start(mac);
    *length = 0;
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Check if packet is ready
  if (!(ready & CH390_PKT_RDY)) {
    *length = 0;
    return ESP_OK;
  }

  // Read RX header
  GOTO_ON_ERROR(ch390_read_mem(emac, (uint8_t *)&header, sizeof(header)), err);

  rx_len = (header.length_high << 8) | header.length_low;

  // Check for frame errors
  if (header.status & RSR_ERR_MASK) {
    ch390_drop_frame(emac, rx_len);
    *length = 0;
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Check frame size
  if (rx_len > *length || rx_len > 1522) {
    // Reset RX memory pointer on oversized frame
    ch390_write_reg(emac, CH390_MPTRCR, MPTRCR_RST_RX);

    *length = 0;
    return ESP_ERR_INVALID_SIZE;
  }

  // Read frame data
  GOTO_ON_ERROR(ch390_read_mem(emac, buf, rx_len), err);

  // Strip CRC
  *length = rx_len - ETH_CRC_LEN;
  return ESP_OK;

err:
  return ret;
}

static esp_err_t ch390_verify_id(emac_ch390_t *emac) {
  esp_err_t ret = ESP_OK;
  uint8_t vidl, vidh, pidl, pidh;

  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_VIDL, &vidl), err, TAG,
                    "read VIDL failed");
  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_VIDH, &vidh), err, TAG,
                    "read VIDH failed");

  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_PIDL, &pidl), err, TAG,
                    "read PIDL failed");
  ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_PIDH, &pidh), err, TAG,
                    "read PIDH failed");

  if (vidh != 0x1C || vidl != 0x00) {
    ret = ESP_ERR_INVALID_VERSION;
    goto err;
  }

  if (pidh != 0x91 || pidl != 0x51) {
    ret = ESP_ERR_INVALID_VERSION;
    goto err;
  }

  return ESP_OK;

err:
  return ret;
}

static esp_err_t ch390_clear_multicast_table(emac_ch390_t *emac) {
  esp_err_t ret = ESP_OK;

  // RX broadcast packet control
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_BCASTCR, 0x00), err);

  // Clear MAR 0-6
  for (int i = 0; i < 7; i++) {
    GOTO_ON_ERROR(ch390_write_reg(emac, CH390_MAR + i, 0x00), err);
  }

  // Enable broadcast reception (MAR7 bit 7)
  GOTO_ON_ERROR(ch390_write_reg(emac, CH390_MAR + 7, 0x80), err);

err:
  return ret;
}

static esp_err_t ch390_setup_default(emac_ch390_t *emac) {
  esp_err_t ret = ESP_OK;

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_NCR, 0x00), err, TAG,
                    "write NCR failed");
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_WCR, 0x00), err, TAG,
                    "write WCR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_TCR, 0x00), err, TAG,
                    "write TCR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_RCR, RCR_DIS_CRC | RCR_ALL),
                    err, TAG, "write RCR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_TCR2, TCR2_RLCP), err, TAG,
                    "write TCR2 failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_TCSCR,
                                    TCSCR_IPCSE | TCSCR_TCPCSE | TCSCR_UDPCSE),
                    err, TAG, "write TCSCR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_RCSCSR, 0x00), err, TAG,
                    "write RCSCSR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_INTCR, 0x00), err, TAG,
                    "write INTCR failed");
  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_INTCKCR, 0x00), err, TAG,
                    "write INTCKCR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_RLENCR,
                                    RLENCR_RXLEN_EN | RLENCR_RXLEN_DEFAULT),
                    err, TAG, "write RLENCR failed");

  ESP_GOTO_ON_ERROR(
      ch390_write_reg(emac, CH390_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END),
      err, TAG, "write NSR failed");

  return ESP_OK;

err:
  return ret;
}

static esp_err_t ch390_init(esp_eth_mac_t *mac) {
  esp_err_t ret = ESP_OK;
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  esp_eth_mediator_t *eth = emac->eth;
  uint8_t ncr;
  uint32_t timeout = 0;

  if (eth) {
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err,
                      TAG, "lowlevel init failed");
  }

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_NCR, NCR_RST), err, TAG,
                    "write NCR failed");
  vTaskDelay(pdMS_TO_TICKS(10));

  do {
    ESP_GOTO_ON_ERROR(ch390_read_reg(emac, CH390_NCR, &ncr), err, TAG,
                      "read NCR failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    timeout += 10;
  } while ((ncr & NCR_RST) && timeout < emac->sw_reset_timeout_ms);
  ESP_GOTO_ON_FALSE(!(ncr & NCR_RST), ESP_ERR_TIMEOUT, err, TAG,
                    "reset timeout");

  ESP_GOTO_ON_ERROR(ch390_verify_id(emac), err, TAG,
                    "chip ID verification failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_GPCR, 0x00), err, TAG,
                    "write GPCR failed");

  ESP_GOTO_ON_ERROR(ch390_write_reg(emac, CH390_GPR, 0x00), err, TAG,
                    "write GPR failed");
  vTaskDelay(pdMS_TO_TICKS(100));

  ESP_GOTO_ON_ERROR(ch390_setup_default(emac), err, TAG,
                    "ch390_setup_default failed");

  ESP_GOTO_ON_ERROR(ch390_set_addr(mac, emac->addr), err, TAG,
                    "set MAC address failed");

  GOTO_ON_ERROR(ch390_clear_multicast_table(emac), err);

  return ESP_OK;

err:
  if (eth) {
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
  }
  return ret;
}

static esp_err_t ch390_deinit(esp_eth_mac_t *mac) {
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  esp_eth_mediator_t *eth = emac->eth;

  emac_ch390_stop(mac);

  if (eth) {
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
  }
  return ESP_OK;
}

// enable_flow_ctrl and set_peer_pause_ability exist in IDF 4.4.7 (Arduino ABI).
// custom_ioctl is IDF 5.0+ only.
static esp_err_t ch390_mac_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable) {
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  emac->flow_ctrl_enabled = enable;
  // CH390 doesn't have hardware flow control, so this is a no-op
  return ESP_OK;
}

static esp_err_t ch390_set_peer_pause_ability(esp_eth_mac_t *mac,
                                              uint32_t ability) {
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
  if (emac->flow_ctrl_enabled && ability) {
    ch390_enable_flow_ctrl(emac, true);
  } else {
    ch390_enable_flow_ctrl(emac, false);
  }
  return ESP_OK;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static esp_err_t ch390_custom_ioctl(esp_eth_mac_t *mac, uint32_t cmd,
                                    void *data) {
  // No custom ioctl commands supported
  return ESP_ERR_NOT_SUPPORTED;
}
#endif

static esp_err_t ch390_del(esp_eth_mac_t *mac) {
  emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);

  if (emac->rx_task_hdl) {
    vTaskDelete(emac->rx_task_hdl);
  }

  if (emac->spi_lock) {
    vSemaphoreDelete(emac->spi_lock);
  }

  if (emac->spi_hdl) {
    spi_bus_remove_device(emac->spi_hdl);
  }

  if (emac->rx_buffer) {
    heap_caps_free(emac->rx_buffer);
  }

  free(emac);
  return ESP_OK;
}

static IRAM_ATTR void ch390_isr_handler(void *arg) {
  emac_ch390_t *emac = (emac_ch390_t *)arg;
  BaseType_t high_task_wakeup = pdFALSE;
  vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
  if (high_task_wakeup != pdFALSE) {
    portYIELD_FROM_ISR();
  }
}

static void emac_ch390_task(void *arg) {
  emac_ch390_t *emac = (emac_ch390_t *)arg;
  uint8_t status = 0;
  uint8_t *buffer;
  uint32_t rx_len;

  while (1) {
    if (emac->int_gpio_num >= 0) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&
          gpio_get_level((gpio_num_t)emac->int_gpio_num) == 0) {
        continue;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Clear interrupt status
    ch390_read_reg(emac, CH390_ISR, &status);
    ch390_write_reg(emac, CH390_ISR, status);

    // Process received packets
    if (status & ISR_PR) {
      do {
        rx_len = 1522;
        if (emac->parent.receive(&emac->parent, emac->rx_buffer, &rx_len) ==
            ESP_OK) {
          if (rx_len == 0) {
            break;
          }

          buffer = (uint8_t *)malloc(rx_len);
          if (buffer == NULL) {
            continue;
          }

          memcpy(buffer, emac->rx_buffer, rx_len);
          emac->eth->stack_input(emac->eth, buffer, rx_len);
        } else {
          break;
        }
      } while (1);
    }
  }
  vTaskDelete(NULL);
}

esp_eth_mac_t *
esp_eth_mac_new_ch390_arduino(const eth_ch390_config_t *ch390_config,
                              const eth_mac_config_t *mac_config) {
  emac_ch390_t *emac = NULL;

  if (!ch390_config || !mac_config) {
    ESP_LOGE(TAG, "invalid argument");
    return NULL;
  }

  emac = (emac_ch390_t *)calloc(1, sizeof(emac_ch390_t));
  if (!emac) {
    ESP_LOGE(TAG, "no mem for MAC instance");
    return NULL;
  }

  // Create SPI mutex
  emac->spi_lock = xSemaphoreCreateMutex();
  if (!emac->spi_lock) {
    free(emac);
    return NULL;
  }

  // Bind methods
  emac->parent.set_mediator = ch390_set_mediator;
  emac->parent.init = ch390_init;
  emac->parent.deinit = ch390_deinit;
  emac->parent.start = emac_ch390_start;
  emac->parent.stop = emac_ch390_stop;
  emac->parent.del = ch390_del;
  emac->parent.write_phy_reg = ch390_write_phy_reg;
  emac->parent.read_phy_reg = ch390_read_phy_reg;
  emac->parent.set_addr = ch390_set_addr;
  emac->parent.get_addr = ch390_get_addr;
  emac->parent.set_link = ch390_set_link;
  emac->parent.set_speed = ch390_set_speed;
  emac->parent.set_duplex = ch390_set_duplex;
  emac->parent.set_promiscuous = ch390_set_promiscuous;
  emac->parent.transmit = ch390_transmit;
  emac->parent.receive = ch390_receive;

  // enable_flow_ctrl and set_peer_pause_ability exist in IDF 4.4.7+ (Arduino
  // ABI) — must be assigned unconditionally, as the upstream W5500 driver does.
  emac->parent.enable_flow_ctrl = ch390_mac_enable_flow_ctrl;
  emac->parent.set_peer_pause_ability = ch390_set_peer_pause_ability;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  emac->parent.custom_ioctl = ch390_custom_ioctl;
#endif

  // Configure SPI device
  spi_device_interface_config_t spi_devcfg = *ch390_config->spi_devcfg;
  if (spi_devcfg.command_bits == 0 && spi_devcfg.address_bits == 0) {
    spi_devcfg.command_bits = 1;
    spi_devcfg.address_bits = 7;
  } else if (spi_devcfg.command_bits != 1 || spi_devcfg.address_bits != 7) {
    ESP_LOGE(TAG, "incorrect SPI frame format: command_bits must be 1, "
                  "address_bits must be 7");
    vSemaphoreDelete(emac->spi_lock);
    free(emac);
    return NULL;
  }

  esp_err_t spi_ret =
      spi_bus_add_device(ch390_config->spi_host, &spi_devcfg, &emac->spi_hdl);
  if (spi_ret != ESP_OK) {
    vSemaphoreDelete(emac->spi_lock);
    ESP_LOGE(TAG, "adding SPI device failed: %s (0x%x)",
             esp_err_to_name(spi_ret), spi_ret);
    free(emac);
    return NULL;
  }

  emac->int_gpio_num = ch390_config->int_gpio_num;
  emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
  emac->flow_ctrl_enabled = false; // Flow control disabled by default

  // Set MAC address: use custom if provided, otherwise get default from ESP32
  if (ch390_config->custom_mac_addr != NULL) {
    memcpy(emac->addr, ch390_config->custom_mac_addr, 6);
  } else {
    esp_read_mac(emac->addr, ESP_MAC_ETH);
  }
  // Allocate DMA-capable RX buffer
  emac->rx_buffer = (uint8_t *)heap_caps_malloc(1522, MALLOC_CAP_DMA);
  if (!emac->rx_buffer) {
    spi_bus_remove_device(emac->spi_hdl);
    vSemaphoreDelete(emac->spi_lock);
    free(emac);
    return NULL;
  }

  // Configure interrupt GPIO if specified
  if (emac->int_gpio_num >= 0) {
    gpio_set_direction((gpio_num_t)emac->int_gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)emac->int_gpio_num, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type((gpio_num_t)emac->int_gpio_num, GPIO_INTR_POSEDGE);
    gpio_intr_enable((gpio_num_t)emac->int_gpio_num);
    gpio_isr_handler_add((gpio_num_t)emac->int_gpio_num, ch390_isr_handler,
                         emac);
  }

  // Create RX task
  BaseType_t xReturned = xTaskCreatePinnedToCore(
      emac_ch390_task, "ch390_tsk", mac_config->rx_task_stack_size, emac,
      mac_config->rx_task_prio, &emac->rx_task_hdl, tskNO_AFFINITY);
  if (xReturned != pdPASS) {
    if (emac->int_gpio_num >= 0) {
      gpio_isr_handler_remove((gpio_num_t)emac->int_gpio_num);
    }
    heap_caps_free(emac->rx_buffer);
    spi_bus_remove_device(emac->spi_hdl);
    vSemaphoreDelete(emac->spi_lock);
    free(emac);
    return NULL;
  }

  return &emac->parent;
}
