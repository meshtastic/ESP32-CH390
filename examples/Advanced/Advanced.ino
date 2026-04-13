/*
 * ESP32-CH390 Arduino Library - Advanced Example
 *
 * Demonstrates:
 *   - Custom SPI pin configuration
 *   - Custom MAC address
 *   - PHY register access via readPHY()
 *   - HTTP client over Ethernet
 *   - Link and duplex monitoring
 *   - Arduino-ESP32 WiFi event system
 *
 * Custom hardware pins used below:
 *   CS   = GPIO15
 *   MOSI = GPIO13
 *   MISO = GPIO12
 *   SCK  = GPIO14
 *   INT  = GPIO4
 */

#include "ESP32_CH390.h"
#include "WiFi.h"
#include "HTTPClient.h"

// Custom MAC address
uint8_t customMAC[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};

bool networkReady = false;

void printDetailedNetworkInfo() {
  Serial.println("\n=== Network Information ===");
  Serial.printf("IP Address:    %s\n", CH390.localIP().toString().c_str());
  Serial.printf("Subnet Mask:   %s\n", CH390.subnetMask().toString().c_str());
  Serial.printf("Gateway:       %s\n", CH390.gatewayIP().toString().c_str());
  Serial.printf("DNS Primary:   %s\n", CH390.dnsIP(0).toString().c_str());
  Serial.printf("DNS Secondary: %s\n", CH390.dnsIP(1).toString().c_str());
  Serial.printf("MAC Address:   %s\n", CH390.macAddress().c_str());
  Serial.printf("Hostname:      %s\n", CH390.getHostname() ? CH390.getHostname() : "Not Set");
  Serial.println("===========================\n");
}

void printPHYRegisters() {
  Serial.println("\n--- PHY Register Status ---");
  Serial.printf("BMCR (0x00):   0x%04X - Basic Mode Control Register\n", CH390.readPHY(0x00));
  Serial.printf("BMSR (0x01):   0x%04X - Basic Mode Status Register\n", CH390.readPHY(0x01));
  Serial.printf("PHYID1 (0x02): 0x%04X - PHY Identifier 1\n", CH390.readPHY(0x02));
  Serial.printf("PHYID2 (0x03): 0x%04X - PHY Identifier 2\n", CH390.readPHY(0x03));
  Serial.printf("ANAR (0x04):   0x%04X - Auto-Negotiation Advertisement\n", CH390.readPHY(0x04));
  Serial.printf("ANLPAR (0x05): 0x%04X - Auto-Negotiation Link Partner\n", CH390.readPHY(0x05));
  Serial.println("---------------------------\n");
}

void performHTTPTest() {
  if (!networkReady) {
    Serial.println("Network not ready for HTTP test");
    return;
  }

  Serial.println("Performing HTTP Test...");

  HTTPClient http;
  http.begin("http://httpbin.org/ip");
  http.setTimeout(5000);

  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.printf("HTTP Response Code: %d\n", httpResponseCode);
    Serial.printf("Response: %s\n", response.c_str());
  } else {
    Serial.printf("HTTP Error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet Started");
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet Stopped");
      networkReady = false;
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet Link Connected");
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet Link Disconnected");
      networkReady = false;
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("Ethernet Got IP Address");
      printDetailedNetworkInfo();
      networkReady = true;
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("     ESP32-CH390 Advanced Example      ");
  Serial.println("       with Custom SPI Configuration   ");
  Serial.println("========================================\n");

  // Register event handler using standard WiFi event system
  WiFi.onEvent(WiFiEvent);

  // Advanced SPI configuration with custom pins
  ch390_config_t config = CH390_DEFAULT_CONFIG();
  config.spi_cs_gpio = 15;      // Custom CS pin
  config.spi_mosi_gpio = 13;    // Custom MOSI pin
  config.spi_miso_gpio = 12;    // Custom MISO pin
  config.spi_sck_gpio = 14;     // Custom SCK pin
  config.spi_clock_mhz = 20;    // 20 MHz SPI clock
  config.spi_host = 2;          // SPI2_HOST
  config.int_gpio = 4;          // Interrupt pin
  memcpy(config.mac_addr, customMAC, 6);

  // Initialize CH390
  Serial.println("Initializing CH390 Ethernet...");
  if (CH390.begin(config)) {
    Serial.println("CH390 initialized successfully");

    // Set hostname
    if (CH390.setHostname("esp32-ch390-advanced")) {
      Serial.println("Hostname set to: esp32-ch390-advanced");
    }

    // Enable DHCP
    if (CH390.enableDHCP()) {
      Serial.println("DHCP enabled");
    }

  } else {
    Serial.println("CH390 initialization failed!");
    return;
  }

  Serial.println("\nWaiting for network connection...");
}

void loop() {
  static unsigned long lastStatusCheck = 0;
  static unsigned long lastHTTPTest = 0;
  static unsigned long lastPHYCheck = 0;

  // Status check every 10 seconds
  if (millis() - lastStatusCheck > 10000) {
    lastStatusCheck = millis();

    Serial.println("\n--- Link Status Check ---");
    Serial.printf("Link Status: %s\n", CH390.linkUp() ? "UP" : "DOWN");
    Serial.printf("Speed: %d Mbps\n", CH390.linkSpeed());
    Serial.printf("Duplex: %s\n", CH390.fullDuplex() ? "Full" : "Half");
    Serial.printf("IP Connected: %s\n", CH390.isConnected() ? "YES" : "NO");

    if (CH390.isConnected()) {
      Serial.printf("Current IP: %s\n", CH390.localIP().toString().c_str());
    }
    Serial.println("-------------------------");
  }

  // HTTP test every 30 seconds
  if (networkReady && (millis() - lastHTTPTest > 30000)) {
    lastHTTPTest = millis();
    performHTTPTest();
  }

  // PHY register check every 60 seconds
  if (millis() - lastPHYCheck > 60000) {
    lastPHYCheck = millis();
    printPHYRegisters();
  }

  delay(100);
}
