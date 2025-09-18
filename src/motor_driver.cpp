#include "motor_driver.h"

#include <math.h>

void MotorDriver::begin(const config::MotorPinConfig &config) {
  config_ = config;
  pinMode(config_.forwardPin, OUTPUT);
  pinMode(config_.reversePin, OUTPUT);

  ledcSetup(config_.forwardChannel, 4000, config::kPwmResolutionBits);
  ledcSetup(config_.reverseChannel, 4000, config::kPwmResolutionBits);
  ledcAttachPin(config_.forwardPin, config_.forwardChannel);
  ledcAttachPin(config_.reversePin, config_.reverseChannel);
  writeDuty(0, 0);
  currentFrequency_ = 4000;
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

  if (constrained > 0.01f) {
    writeDuty(dutyValue, 0);
  } else if (constrained < -0.01f) {
    writeDuty(0, dutyValue);
  } else {
    writeDuty(0, 0);
  }
}

void MotorDriver::stop() { writeDuty(0, 0); }

uint32_t MotorDriver::selectFrequency(float magnitude) {
  if (magnitude < 0.05f) {
    return 400;
  }
  if (magnitude < 0.2f) {
    return 800;
  }
  if (magnitude < 0.5f) {
    return 2000;
  }
  return 4000;
}

void MotorDriver::writeDuty(uint16_t forwardDuty, uint16_t reverseDuty) {
  ledcWrite(config_.forwardChannel, forwardDuty);
  ledcWrite(config_.reverseChannel, reverseDuty);
}

