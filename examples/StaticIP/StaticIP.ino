/*
 * ESP32-CH390 Arduino Library - Static IP Example
 *
 * Configures the CH390 SPI Ethernet controller with a static IP address
 * and a custom hostname, using the Arduino-ESP32 WiFi event system.
 *
 * Default hardware pins (see CH390_DEFAULT_CONFIG):
 *   CS   = GPIO5
 *   MOSI = GPIO23
 *   MISO = GPIO19
 *   SCK  = GPIO18
 *   INT  = GPIO4
 */

#include "ESP32_CH390.h"
#include "WiFi.h"

// Network configuration
IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

void printNetworkInfo() {
  Serial.println("=== Network Configuration ===");
  Serial.print("IP Address: ");
  Serial.println(CH390.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(CH390.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(CH390.gatewayIP());
  Serial.print("Primary DNS: ");
  Serial.println(CH390.dnsIP(0));
  Serial.print("Secondary DNS: ");
  Serial.println(CH390.dnsIP(1));
  Serial.print("MAC Address: ");
  Serial.println(CH390.macAddress());
  Serial.println("============================");
}

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
      printNetworkInfo();
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CH390 Static IP Example");

  // Register event handler using standard WiFi event system
  WiFi.onEvent(WiFiEvent);

  // Initialize CH390 with default SPI configuration
  // Default pins: CS=5, MOSI=23, MISO=19, SCK=18, INT=4
  if (CH390.begin()) {
    Serial.println("CH390 initialized successfully");
  } else {
    Serial.println("CH390 initialization failed!");
    return;
  }

  // Configure static IP
  if (CH390.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Static IP configuration successful");
  } else {
    Serial.println("Static IP configuration failed!");
  }

  // Set hostname
  CH390.setHostname("esp32-ch390");
}

void loop() {
  // Monitor connection and perform network test
  static unsigned long lastTest = 0;
  if (millis() - lastTest > 10000) {  // Every 10 seconds
    lastTest = millis();

    if (CH390.isConnected()) {
      Serial.println("\n--- Network Test ---");
      printNetworkInfo();

      // Additional link status
      Serial.printf("Link Speed: %d Mbps\n", CH390.linkSpeed());
      Serial.printf("Duplex Mode: %s\n", CH390.fullDuplex() ? "Full" : "Half");

      Serial.println("Network is ready for communication");
      Serial.println("-------------------\n");
    } else {
      Serial.println("Waiting for network connection...");
    }
  }

  delay(100);
}
