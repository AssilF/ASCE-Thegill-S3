#pragma once

#include <Arduino.h>
#include <cstdint>
#include <esp32-hal-timer.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "device_config.h"

class MotorDriver {
public:
  MotorDriver() = default;

  void begin(const config::MotorPinConfig &config);
  void applyCommand(float command, bool brake);
  void stop();
  void handleTimerInterrupt();

private:
  void configurePwmUnlocked(float magnitude, bool forward, uint32_t frequency);
  void setBrakeUnlocked();
  void releaseBrakeUnlocked();
  void stopUnlocked();
  void setActivePinHighUnlocked();
  void setPinsLowUnlocked();
  void setBothPinsHighUnlocked();


  static uint32_t selectFrequency(float magnitude);

  config::MotorPinConfig config_{};
  hw_timer_t *timer_ = nullptr;
  portMUX_TYPE timerMux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t currentFrequency_ = config::kMotorPwmFrequencyHigh;
  volatile uint32_t periodTicks_ = 0;
  volatile uint32_t highTicks_ = 0;
  volatile bool pwmActive_ = false;
  volatile bool phaseHigh_ = false;
  volatile bool forwardActive_ = true;
  volatile bool brakeActive_ = false;

  friend void IRAM_ATTR MotorDriverTimerDispatch(uint8_t timerIndex);
};

