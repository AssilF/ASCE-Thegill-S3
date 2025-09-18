#pragma once

#include <ArduinoOTA.h>
#include <esp_now.h>

#include <functional>

void ConfigureWiFi(const char *hostname, const char *ssid, const char *password, uint8_t channel);
bool InitializeEspNow(esp_now_recv_cb_t callback);
void ConfigureOta(const char *hostname, const char *password, const char *ssid,
                  const std::function<void()> &onStart,
                  const std::function<void()> &onEnd,
                  const std::function<void(unsigned int, unsigned int)> &onProgress,
                  const std::function<void(ota_error_t)> &onError);
void HandleOtaLoop();

