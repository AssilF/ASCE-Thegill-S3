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
constexpr uint32_t kMotorPwmFrequencyLow = 6000;
constexpr uint32_t kMotorPwmFrequencyMidLow = 10000;
constexpr uint32_t kMotorPwmFrequencyMidHigh = 16000;
constexpr uint32_t kMotorPwmFrequencyHigh = 22000;

constexpr MotorPinConfig kMotorPins[kMotorCount] = {
    {19, 20, 0, 1, 0, false},  // Front left motor
    {5, 4, 2, 3, 1, false},    // Rear left motor
    {18, 17, 4, 5, 2, true},   // Front right motor
    {7, 6, 6, 7, 3, true},     // Rear right motor
};

constexpr uint8_t kBuzzerPin = 11;
constexpr uint8_t kBuzzerChannel = 5;
constexpr uint8_t kBuzzerResolutionBits = 10;
constexpr uint8_t kBuzzerMinResolutionBits = 4;

constexpr uint16_t kBuzzerRetuneGuardMs = 6;
constexpr uint16_t kBuzzerRetuneGuardMinDeltaHz = 12;
constexpr uint8_t kBuzzerRetuneGuardDeltaPercent = 6;


constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint16_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;

constexpr uint8_t kStatusLedPin = 48;

constexpr uint8_t kShiftRegisterDataPin = 21;
constexpr uint8_t kShiftRegisterClockPin = 36;
constexpr uint8_t kShiftRegisterLatchPin = 35;
constexpr uint32_t kShiftRegisterPwmFrequencyHz = 100000;
constexpr uint8_t kShiftRegisterPwmResolutionBits = 8;

constexpr uint8_t kArmBasePotPin = 1;
constexpr uint8_t kArmExtensionPotPin = 2;
constexpr uint8_t kAnalogMultiplexerInputPin = 3;

constexpr uint8_t kServoShoulderPin = 13;
constexpr uint8_t kServoElbowPin = 12;
constexpr uint8_t kServoGripperPitchPin = 40;
constexpr uint8_t kServoRollPin = 39;
constexpr uint8_t kServoYawPin = 14;

namespace arm {

constexpr uint16_t kBaseAdcMin = 0;
constexpr uint16_t kBaseAdcMax = 4095;
constexpr bool kBaseInvertFeedback = false;
constexpr bool kBaseInvertDirection = false;

constexpr uint16_t kExtensionAdcMin = 0;
constexpr uint16_t kExtensionAdcMax = 4095;
constexpr bool kExtensionInvertFeedback = false;
constexpr bool kExtensionInvertDirection = false;

constexpr float kBaseKp = 4.5f;
constexpr float kBaseKi = 0.35f;
constexpr float kBaseKd = 0.12f;

constexpr float kExtensionKp = 4.0f;
constexpr float kExtensionKi = 0.30f;
constexpr float kExtensionKd = 0.10f;

constexpr float kIntegralLimit = 1.5f;
constexpr float kOutputLimit = 0.9f;
constexpr float kDeadband = 0.015f;
constexpr float kSampleAlpha = 0.25f;

} // namespace arm

} // namespace config
