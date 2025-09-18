#include "network_setup.h"

#include <Arduino.h>

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

