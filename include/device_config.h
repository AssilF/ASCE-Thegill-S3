#pragma once

#include <cstddef>
#include <cstdint>

namespace config {

constexpr const char kDeviceIdentity[] = "THEGILL";
constexpr const char kAccessPointSsid[] = "Thegill PORT";
constexpr const char kAccessPointPassword[] = "ASCE321#";
constexpr uint8_t kEspNowChannel = 0;
constexpr std::size_t kMotorCount = 4;
constexpr uint32_t kControlTimeoutMs = 500;

struct MotorPinConfig {
  uint8_t forwardPin;
  uint8_t reversePin;
  uint8_t forwardChannel;
  uint8_t reverseChannel;
  uint8_t timerIndex;
  bool inverted;
};

constexpr float kMotorCommandDeadband = 0.01f;
constexpr float kMotorFrequencyThresholdLow = 0.05f;
constexpr float kMotorFrequencyThresholdMidLow = 0.2f;
constexpr float kMotorFrequencyThresholdMidHigh = 0.5f;
constexpr uint32_t kMotorPwmFrequencyLow = 400;
constexpr uint32_t kMotorPwmFrequencyMidLow = 800;
constexpr uint32_t kMotorPwmFrequencyMidHigh = 2000;
constexpr uint32_t kMotorPwmFrequencyHigh = 4000;

constexpr MotorPinConfig kMotorPins[kMotorCount] = {
    // Left side motors share orientation, right side are mirrored and inverted.
    {16, 15, 0, 1, 0, false},  // Front left motor (H-bridge A1/A2)
    {18, 17, 2, 3, 1, false},  // Rear left motor (H-bridge B1/B2)
    {13, 14, 4, 5, 2, true},   // Front right motor (H-bridge C1/C2)
    {11, 12, 6, 7, 3, true},   // Rear right motor (H-bridge D1/D2)
};

constexpr uint8_t kBuzzerPin = 47;
constexpr uint8_t kBuzzerChannel = 0;
constexpr uint8_t kBuzzerResolutionBits = 10;
constexpr uint8_t kBuzzerMinResolutionBits = 4;

constexpr uint16_t kBuzzerRetuneGuardMs = 6;
constexpr uint16_t kBuzzerRetuneGuardMinDeltaHz = 12;
constexpr uint8_t kBuzzerRetuneGuardDeltaPercent = 6;


constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint16_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;

constexpr uint8_t kStatusLedPin = 48;

} // namespace config
