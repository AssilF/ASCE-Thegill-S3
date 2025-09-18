#pragma once

#include <cstddef>
#include <cstdint>

namespace config {

constexpr const char kDeviceIdentity[] = "Thegiller-PT12";
constexpr const char kAccessPointSsid[] = "Thegill PORT";
constexpr const char kAccessPointPassword[] = "ASCE321#";
constexpr uint8_t kEspNowChannel = 6;
constexpr std::size_t kMotorCount = 4;
constexpr uint32_t kControlTimeoutMs = 500;

struct MotorPinConfig {
  uint8_t forwardPin;
  uint8_t reversePin;
  uint8_t forwardChannel;
  uint8_t reverseChannel;
  bool inverted;
};

constexpr MotorPinConfig kMotorPins[kMotorCount] = {
    {4, 5, 0, 1, false},   // Motor A
    {6, 7, 2, 3, false},   // Motor B
    {8, 9, 4, 5, false},   // Motor C
    {10, 11, 6, 7, false}, // Motor D
};

constexpr uint8_t kBuzzerPin = 47;
constexpr uint8_t kBuzzerChannel = 14;
constexpr uint8_t kBuzzerResolutionBits = 12;

constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint16_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;

constexpr uint8_t kStatusLedPin = 48;

} // namespace config
