# ESP32-CH390 Arduino Library

Arduino library for the WCH CH390 SPI Ethernet controller. The CH390 is a
complete 10/100 Mbps Ethernet solution (MAC + PHY) accessed over SPI.

This library is an Arduino port of the ESP-IDF CH390 driver (based on IDF
v4.4.7) and integrates with the Arduino-ESP32 network stack so it behaves like
the built-in `ETH` class.

## Hardware

CH390 is a SPI-attached MAC+PHY. The PHY is accessed internally through the MAC
via SPI, so no MDC/MDIO lines are required.

### Default Pin Configuration

| Signal | ESP32 Pin | Description             |
| ------ | --------- | ----------------------- |
| CS     | GPIO5     | SPI Chip Select         |
| MOSI   | GPIO23    | SPI Master Out Slave In |
| MISO   | GPIO19    | SPI Master In Slave Out |
| SCK    | GPIO18    | SPI Clock               |
| INT    | GPIO4     | Interrupt (optional)    |

The SPI frame format used by the driver is fixed at 1 command bit + 7 address
bits. This is configured automatically; do not override it.

## Quick Start

```cpp
#include "ESP32_CH390.h"
#include "WiFi.h"

void WiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_ETH_GOT_IP) {
    Serial.print("Ethernet IP: ");
    Serial.println(CH390.localIP());
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.onEvent(WiFiEvent);
  CH390.begin();
}

void loop() {
  delay(1000);
}
```

## Configuration

### Custom SPI Pins

```cpp
ch390_config_t config = CH390_DEFAULT_CONFIG();
config.spi_cs_gpio   = 15;
config.spi_mosi_gpio = 13;
config.spi_miso_gpio = 12;
config.spi_sck_gpio  = 14;
config.spi_clock_mhz = 20;  // SPI speed in MHz (default 20)
config.spi_host      = 2;   // 1 = HSPI_HOST, 2 = VSPI_HOST / SPI2_HOST
config.int_gpio      = 4;   // Optional interrupt pin, set -1 to poll

CH390.begin(config);
```

### Static IP

```cpp
IPAddress ip(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

CH390.begin();
CH390.config(ip, gateway, subnet, dns);
```

### Custom MAC Address

```cpp
uint8_t mac[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
ch390_config_t config = CH390_DEFAULT_CONFIG();
memcpy(config.mac_addr, mac, 6);
CH390.begin(config);
```

If `mac_addr` is left zeroed, the ESP32's factory Ethernet MAC is used.

## API

### Initialization

- `bool begin()`
- `bool begin(ch390_config_t config)`
- `void end()`

### Network

- `bool config(IPAddress ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = 0, IPAddress dns2 = 0)`
- `bool enableDHCP()` / `bool disableDHCP()`
- `bool setHostname(const char *hostname)` / `const char *getHostname()`

### Status

- `bool isConnected()` returns true when a valid IP is assigned
- `bool linkUp()`
- `IPAddress localIP()`, `subnetMask()`, `gatewayIP()`, `dnsIP(uint8_t n)`
- `String macAddress()`, `uint8_t *macAddressBytes()`
- `int linkSpeed()` (10 or 100)
- `bool fullDuplex()`

### PHY Access

- `uint32_t readPHY(uint8_t reg)`
- `bool writePHY(uint8_t reg, uint32_t value)`

### Events

The library uses the standard Arduino-ESP32 event system. Register handlers
with `WiFi.onEvent()`:

```cpp
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:        break;
    case ARDUINO_EVENT_ETH_CONNECTED:    break;
    case ARDUINO_EVENT_ETH_GOT_IP:       break;
    case ARDUINO_EVENT_ETH_DISCONNECTED: break;
    case ARDUINO_EVENT_ETH_STOP:         break;
  }
}
```

## Examples

- `Basic`: DHCP with event handling on the default SPI pins.
- `StaticIP`: Static IP configuration and hostname.
- `Advanced`: Custom SPI pins and MAC, PHY register dump, HTTP client.

## Compatibility

- Platform: ESP32, ESP32-S2, ESP32-S3, ESP32-C3 (any ESP32 with SPI master)
- Framework: Arduino-ESP32 2.0.0 or newer (ESP-IDF 4.4.x or 5.x)
- Interface: Hardware SPI, up to 33 MHz (20 MHz default)

## Notes

- CH390 is a complete SPI Ethernet chip (MAC + PHY); no external PHY wiring is
  needed.
- Hardware interrupt is optional. When `int_gpio` is set to `-1`, RX is polled
  from the driver task.
- The driver shares one SPI bus mutex across all register and memory access,
  so it is safe to share the SPI host with other devices that use the
  Arduino-ESP32 SPI master.
- Pause/flow control is advertised but only takes effect on IDF 5.0 or newer.

## License

This library is licensed under the GNU Lesser General Public License v3.0
or later (`LGPL-3.0-or-later`). See the `LICENSE` file for the full license
text.

It is derived from the ESP-IDF CH390 driver, which is licensed under the
Apache License, Version 2.0. The upstream source is maintained at:

    https://github.com/espressif/esp-eth-drivers/tree/master/ch390

Apache-2.0 permits redistribution of derivative works under LGPLv3; see the
`NOTICE` file for attribution details.
