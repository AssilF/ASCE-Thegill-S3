#include "network_setup.h"

#include <Arduino.h>
#include <WiFi.h>
#include <string.h>

void ConfigureWiFi(const char *hostname, const char *ssid, const char *password, uint8_t channel) {
  Serial.printf("Configuring WiFi (hostname=%s, channel=%u)\n", hostname, channel);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(hostname);
  Serial.printf("Starting SoftAP \"%s\" with password length %u\n", ssid, strlen(password));
  bool apResult = WiFi.softAP(ssid, password, channel);
  if (!apResult) {
    Serial.println("Failed to start access point");
  } else {
    Serial.printf("Access point \"%s\" started on channel %u\n", ssid, channel);
  }
}

bool InitializeEspNow(esp_now_recv_cb_t callback) {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return false;
  }
  esp_now_register_recv_cb(callback);
  Serial.println("ESP-NOW initialized and receive callback registered");
  Serial.println("Waiting for ILITE pairing handshake");
  return true;
}

void ConfigureOta(const char *hostname, const char *password, const char *ssid,
                  const std::function<void()> &onStart, const std::function<void()> &onEnd,
                  const std::function<void(unsigned int, unsigned int)> &onProgress,
                  const std::function<void(ota_error_t)> &onError) {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(password);

  if (onStart) {
    ArduinoOTA.onStart(onStart);
  }
  if (onEnd) {
    ArduinoOTA.onEnd(onEnd);
  }
  if (onProgress) {
    ArduinoOTA.onProgress(onProgress);
  }
  if (onError) {
    ArduinoOTA.onError(onError);
  }

  ArduinoOTA.begin();
  Serial.printf("OTA ready on AP %s\n", ssid);
}

void HandleOtaLoop() { ArduinoOTA.handle(); }

