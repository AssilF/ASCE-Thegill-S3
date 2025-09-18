#pragma once

#include <cstdint>

namespace Comms {

constexpr uint32_t kDrivePacketMagic = 0x54474C4C; // 'TGLL'
constexpr uint8_t kDriveFlagBrake = 0x01;
constexpr uint8_t kDriveFlagHonk = 0x02;

struct DriveCommand {
  uint32_t magic;
  int16_t leftFront;
  int16_t leftRear;
  int16_t rightFront;
  int16_t rightRear;
  float easingRate;
  uint8_t mode;
  uint8_t easing;
  uint8_t flags;
  uint8_t reserved;
} __attribute__((packed));

enum PairingType : uint8_t {
  kScanRequest = 0x01,
  kDroneIdentity = 0x02,
  kControllerIdentity = 0x03,
  kDroneAck = 0x04,
};

struct IdentityMessage {
  uint8_t type;
  char identity[16];
  uint8_t mac[6];
} __attribute__((packed));

bool init(const char *ssid, const char *password, uint8_t channel);
bool paired();
bool receiveCommand(DriveCommand &cmd);
uint32_t lastCommandTimestamp();
const uint8_t *controllerMac();
const char *controllerIdentity();

} // namespace Comms

