#pragma once

#include <ArduinoOTA.h>

#include <functional>

void ConfigureOta(const char *hostname, const char *password, const char *ssid,
                  const std::function<void()> &onStart, const std::function<void()> &onEnd,
                  const std::function<void(unsigned int, unsigned int)> &onProgress,
                  const std::function<void(ota_error_t)> &onError);
void HandleOtaLoop();

