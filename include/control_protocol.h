#pragma once

#include <cstddef>
#include <cstdint>

#include "device_config.h"

namespace protocol {

constexpr uint8_t kControlProtocolVersion = 1;

enum class MessageType : uint8_t {
  kScanRequest = 0,
  kDroneIdentity = 1,
  kControllerIdentity = 2,
  kDroneAck = 3,
  kControlCommand = 4,
};

constexpr uint16_t kControlFlagHonk = 1u << 0;

constexpr uint16_t BrakeFlagForMotor(std::size_t motorIndex) {
  return static_cast<uint16_t>(1u << (motorIndex + 1));
}

struct IdentityMessage {
  uint8_t type = 0;
  char identity[32] = {};
  uint8_t mac[6] = {};
};

struct ControlMessage {
  uint8_t type = 0;
  uint8_t version = 0;
  uint32_t sequence = 0;
  int16_t motorDuty[config::kMotorCount] = {};
  uint16_t flags = 0;
};

constexpr uint16_t kGillPacketMagic = 0x474c; // 'GL'

constexpr uint16_t kGillFlagBrake = 1u << 0;
constexpr uint16_t kGillFlagHonk = 1u << 1;

struct GillControlPacket {
  uint16_t magic = kGillPacketMagic;
  uint8_t flags = 0;
  int16_t leftFront = 0;
  int16_t leftRear = 0;
  int16_t rightFront = 0;
  int16_t rightRear = 0;
};

} // namespace protocol

