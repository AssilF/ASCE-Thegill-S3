#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "control_protocol.h"
#include "device_config.h"

namespace Comms {

struct DriveCommand {
  uint32_t sequence;
  uint8_t version;
  int16_t motorDuty[config::kMotorCount];
  uint16_t flags;
};

bool init(const char *ssid, const char *password, uint8_t channel);
bool receiveCommand(DriveCommand &cmd);
bool paired();
uint32_t lastCommandTimestamp();
const uint8_t *controllerMac();
const char *controllerIdentity();

extern const uint8_t kBroadcastMac[6];

} // namespace Comms

