#pragma once

#include <Arduino.h>
#include <cstdint>

#include "device_config.h"

class MotorDriver {
public:
  MotorDriver() = default;

  void begin(const config::MotorPinConfig &config);
  void applyCommand(float command, bool brake);
  void stop();

private:
  static uint32_t selectFrequency(float magnitude);
  void writeDuty(uint16_t forwardDuty, uint16_t reverseDuty);

  config::MotorPinConfig config_{};
  uint32_t currentFrequency_ = 4000;
};

