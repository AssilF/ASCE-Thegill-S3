#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "device_config.h"

namespace Comms {

constexpr uint32_t kDriveCommandMagic = 0xA1B2C3D4;

struct DriveCommand {
  uint32_t magic = kDriveCommandMagic;
  uint16_t throttle = 0;
  int8_t pitchAngle = 0;
  int8_t rollAngle = 0;
  int8_t yawAngle = 0;
  bool armMotors = false;
} __attribute__((packed));

bool init(const char *ssid, const char *password, uint8_t channel);
bool receiveCommand(DriveCommand &cmd);
bool paired();
uint32_t lastCommandTimestamp();
const uint8_t *controllerMac();
const char *controllerIdentity();

extern const uint8_t kBroadcastMac[6];

} // namespace Comms

