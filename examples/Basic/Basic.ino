/*
 * ESP32-CH390 Arduino Library - Basic Example
 *
 * Demonstrates default DHCP configuration of the CH390 SPI Ethernet
 * controller using the standard Arduino-ESP32 WiFi event system.
 *
 * Default hardware pins (see CH390_DEFAULT_CONFIG):
 *   CS   = GPIO5
 *   MOSI = GPIO23
 *   MISO = GPIO19
 *   SCK  = GPIO18
 *   INT  = GPIO4 (set int_gpio to -1 to poll instead)
 */

#include "ESP32_CH390.h"
#include "WiFi.h"

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet Started");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet Connected!");
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet Disconnected!");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("Ethernet Got IP!");
      Serial.print("IP Address: ");
      Serial.println(CH390.localIP());
      Serial.print("Subnet Mask: ");
      Serial.println(CH390.subnetMask());
      Serial.print("Gateway: ");
      Serial.println(CH390.gatewayIP());
      Serial.print("DNS: ");
      Serial.println(CH390.dnsIP());
      Serial.print("MAC Address: ");
      Serial.println(CH390.macAddress());
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CH390 Ethernet Example");

  // Register event handler using standard WiFi event system
  WiFi.onEvent(WiFiEvent);

  // Initialize CH390 with default settings
  // Default SPI pins: CS=5, MOSI=23, MISO=19, SCK=18
  if (CH390.begin()) {
    Serial.println("CH390 initialized successfully");
  } else {
    Serial.println("CH390 initialization failed!");
    return;
  }

  // Enable DHCP (default)
  CH390.enableDHCP();
}

void loop() {
  // Check connection status every 5 seconds
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();

    if (CH390.isConnected()) {
      Serial.println("=== Ethernet Status ===");
      Serial.printf("Connected: %s\n", CH390.isConnected() ? "Yes" : "No");
      Serial.printf("Link Up: %s\n", CH390.linkUp() ? "Yes" : "No");
      Serial.printf("Speed: %d Mbps\n", CH390.linkSpeed());
      Serial.printf("Duplex: %s\n", CH390.fullDuplex() ? "Full" : "Half");
      Serial.printf("IP: %s\n", CH390.localIP().toString().c_str());
      Serial.println("=====================");
    } else {
      Serial.println("Ethernet not connected");
    }
  }

  delay(100);
}
