#include "motor_driver.h"

#include <math.h>

void MotorDriver::begin(const config::MotorPinConfig &config) {
  config_ = config;
  pinMode(config_.forwardPin, OUTPUT);
  pinMode(config_.reversePin, OUTPUT);

  ledcSetup(config_.forwardChannel, config::kMotorPwmFrequencyHigh, config::kPwmResolutionBits);
  ledcSetup(config_.reverseChannel, config::kMotorPwmFrequencyHigh, config::kPwmResolutionBits);
  ledcAttachPin(config_.forwardPin, config_.forwardChannel);
  ledcAttachPin(config_.reversePin, config_.reverseChannel);
  writeDuty(0, 0);
  currentFrequency_ = config::kMotorPwmFrequencyHigh;
}

void MotorDriver::applyCommand(float command, bool brake) {
  if (brake) {
    writeDuty(config::kPwmMaxDuty, config::kPwmMaxDuty);
    return;
  }

  float constrained = constrain(command, -1.0f, 1.0f);
  if (config_.inverted) {
    constrained = -constrained;
  }

  const float magnitude = fabsf(constrained);
  const uint32_t frequency = selectFrequency(magnitude);
  if (frequency != currentFrequency_) {
    ledcChangeFrequency(config_.forwardChannel, frequency, config::kPwmResolutionBits);
    ledcChangeFrequency(config_.reverseChannel, frequency, config::kPwmResolutionBits);
    currentFrequency_ = frequency;
  }

  const uint16_t dutyValue = static_cast<uint16_t>(roundf(magnitude * config::kPwmMaxDuty));

  if (constrained > config::kMotorCommandDeadband) {
    writeDuty(dutyValue, 0);
  } else if (constrained < -config::kMotorCommandDeadband) {
    writeDuty(0, dutyValue);
  } else {
    writeDuty(0, 0);
  }
}

void MotorDriver::stop() { writeDuty(0, 0); }

uint32_t MotorDriver::selectFrequency(float magnitude) {
  if (magnitude < config::kMotorFrequencyThresholdLow) {
    return config::kMotorPwmFrequencyLow;
  }
  if (magnitude < config::kMotorFrequencyThresholdMidLow) {
    return config::kMotorPwmFrequencyMidLow;
  }
  if (magnitude < config::kMotorFrequencyThresholdMidHigh) {
    return config::kMotorPwmFrequencyMidHigh;
  }
  return config::kMotorPwmFrequencyHigh;
}

void MotorDriver::writeDuty(uint16_t forwardDuty, uint16_t reverseDuty) {
  ledcWrite(config_.forwardChannel, forwardDuty);
  ledcWrite(config_.reverseChannel, reverseDuty);
}

