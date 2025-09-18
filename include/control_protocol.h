#pragma once

#include <cstddef>
#include <cstdint>

#include "device_config.h"

namespace protocol {

enum class MessageType : uint8_t {
  kScanRequest = 0x01,
  kDroneIdentity = 0x02,
  kControllerIdentity = 0x03,
  kDroneAck = 0x04,
  kControlCommand = 0x10,
};

#pragma pack(push, 1)
struct IdentityMessage {
  uint8_t type;
  char identity[16];
  uint8_t mac[6];
};

struct ControlMessage {
  uint8_t type;
  uint8_t version;
  uint32_t sequence;
  int16_t motorDuty[config::kMotorCount];
  uint16_t flags;
};

struct GillControlPacket {
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
};
#pragma pack(pop)

static_assert(sizeof(IdentityMessage) == 23, "IdentityMessage must match ILITE discovery payload size");
static_assert(sizeof(ControlMessage) == 16, "ControlMessage layout must stay byte-packed for ESP-NOW");
static_assert(sizeof(GillControlPacket) == 20, "GillControlPacket layout must match ILITE controller output");

constexpr uint8_t kControlProtocolVersion = 1;
constexpr uint16_t kControlFlagHonk = 0x0001;
constexpr uint16_t kControlFlagBrakeBase = 0x0010;

constexpr uint32_t kGillPacketMagic = 0x54474C4C; // 'TGLL'
constexpr uint8_t kGillFlagBrake = 0x01;
constexpr uint8_t kGillFlagHonk = 0x02;

inline constexpr uint16_t BrakeFlagForMotor(std::size_t index) {
  return static_cast<uint16_t>(kControlFlagBrakeBase << index);
}

} // namespace protocol

