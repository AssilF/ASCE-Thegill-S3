#include "motor_driver.h"

#include <algorithm>
#include <math.h>
#include <soc/gpio_struct.h>

namespace {
constexpr uint32_t kTimerPrescaler = 80;
constexpr uint32_t kTimerBaseFrequencyHz = 80000000UL / kTimerPrescaler; // 1 MHz tick resolution
constexpr size_t kHardwareTimerCount = 4;

inline void IRAM_ATTR writePinHigh(uint8_t pin) {
  if (pin < 32) {
    GPIO.out_w1ts = (1UL << pin);
  } else {
    GPIO.out1_w1ts.val = (1UL << (pin - 32));
  }
}

inline void IRAM_ATTR writePinLow(uint8_t pin) {
  if (pin < 32) {
    GPIO.out_w1tc = (1UL << pin);
  } else {
    GPIO.out1_w1tc.val = (1UL << (pin - 32));
  }
}

MotorDriver *g_timerOwners[kHardwareTimerCount] = {nullptr};

void IRAM_ATTR TimerTrampoline0() { MotorDriverTimerDispatch(0); }
void IRAM_ATTR TimerTrampoline1() { MotorDriverTimerDispatch(1); }
void IRAM_ATTR TimerTrampoline2() { MotorDriverTimerDispatch(2); }
void IRAM_ATTR TimerTrampoline3() { MotorDriverTimerDispatch(3); }

using TimerIsr = void (*)();
constexpr TimerIsr kTimerTrampolines[kHardwareTimerCount] = {
    TimerTrampoline0,
    TimerTrampoline1,
    TimerTrampoline2,
    TimerTrampoline3,
};
} // namespace

void IRAM_ATTR MotorDriverTimerDispatch(uint8_t timerIndex) {
  if (timerIndex >= kHardwareTimerCount) {
    return;
  }
  MotorDriver *driver = g_timerOwners[timerIndex];
  if (driver != nullptr) {
    driver->handleTimerInterrupt();
  }
}

void MotorDriver::begin(const config::MotorPinConfig &config) {
  config_ = config;
  pinMode(config_.forwardPin, OUTPUT);
  pinMode(config_.reversePin, OUTPUT);
  setPinsLowUnlocked();

  if (config_.timerIndex >= kHardwareTimerCount) {
    return;
  }

  timer_ = timerBegin(config_.timerIndex, kTimerPrescaler, true);
  if (timer_ != nullptr) {
    g_timerOwners[config_.timerIndex] = this;
    timerAttachInterrupt(timer_, kTimerTrampolines[config_.timerIndex], true);
    timerAlarmDisable(timer_);
    timerWrite(timer_, 0);
  }

  currentFrequency_ = config::kMotorPwmFrequencyHigh;
  pwmActive_ = false;
  phaseHigh_ = false;
  brakeActive_ = false;
}

void MotorDriver::applyCommand(float command, bool brake) {
  if (timer_ == nullptr) {
    return;
  }

  portENTER_CRITICAL(&timerMux_);

  if (brake) {
    setBrakeUnlocked();
    portEXIT_CRITICAL(&timerMux_);
    return;
  }

  releaseBrakeUnlocked();

  float constrained = constrain(command, -1.0f, 1.0f);
  if (config_.inverted) {
    constrained = -constrained;
  }

  const float magnitude = fabsf(constrained);
  if (magnitude <= config::kMotorCommandDeadband) {
    stopUnlocked();
    portEXIT_CRITICAL(&timerMux_);
    return;
  }

  const uint32_t frequency = selectFrequency(magnitude);
  currentFrequency_ = frequency;
  const bool forward = (constrained >= 0.0f);
  configurePwmUnlocked(magnitude, forward, frequency);

  portEXIT_CRITICAL(&timerMux_);
}

void MotorDriver::stop() {
  if (timer_ == nullptr) {
    return;
  }
  portENTER_CRITICAL(&timerMux_);
  releaseBrakeUnlocked();
  stopUnlocked();
  portEXIT_CRITICAL(&timerMux_);
}

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

void MotorDriver::configurePwmUnlocked(float magnitude, bool forward, uint32_t frequency) {
  forwardActive_ = forward;

  const uint32_t periodTicks = std::max<uint32_t>(1, kTimerBaseFrequencyHz / std::max<uint32_t>(frequency, 1));
  uint32_t highTicks = static_cast<uint32_t>(roundf(magnitude * static_cast<float>(periodTicks)));

  if (highTicks == 0) {
    stopUnlocked();
    return;
  }

  if (highTicks >= periodTicks) {
    pwmActive_ = false;
    phaseHigh_ = false;
    if (timer_ != nullptr) {
      timerAlarmDisable(timer_);
      timerWrite(timer_, 0);
    }
    setActivePinHighUnlocked();
    return;
  }

  periodTicks_ = periodTicks;
  highTicks_ = highTicks;
  pwmActive_ = true;
  phaseHigh_ = true;

  setActivePinHighUnlocked();

  timerWrite(timer_, 0);
  timerAlarmWrite(timer_, highTicks_, false);
  timerAlarmEnable(timer_);
}

void MotorDriver::setBrakeUnlocked() {
  if (timer_ != nullptr) {
    timerAlarmDisable(timer_);
    timerWrite(timer_, 0);
  }
  pwmActive_ = false;
  phaseHigh_ = false;
  brakeActive_ = true;
  setBothPinsHighUnlocked();
}

void MotorDriver::releaseBrakeUnlocked() {
  if (!brakeActive_) {
    return;
  }
  brakeActive_ = false;
  setPinsLowUnlocked();
}

void MotorDriver::stopUnlocked() {
  if (timer_ != nullptr) {
    timerAlarmDisable(timer_);
    timerWrite(timer_, 0);
  }
  pwmActive_ = false;
  phaseHigh_ = false;
  setPinsLowUnlocked();
}

void MotorDriver::setActivePinHighUnlocked() {
  if (forwardActive_) {
    writePinLow(config_.reversePin);
    writePinHigh(config_.forwardPin);
  } else {
    writePinLow(config_.forwardPin);
    writePinHigh(config_.reversePin);
  }
}

void MotorDriver::setPinsLowUnlocked() {
  writePinLow(config_.forwardPin);
  writePinLow(config_.reversePin);
}

void MotorDriver::setBothPinsHighUnlocked() {
  writePinHigh(config_.forwardPin);
  writePinHigh(config_.reversePin);
}

void MotorDriver::handleTimerInterrupt() {
  if (!pwmActive_ || timer_ == nullptr) {
    return;
  }

  portENTER_CRITICAL_ISR(&timerMux_);

  if (phaseHigh_) {
    setPinsLowUnlocked();
    phaseHigh_ = false;
    const uint32_t lowTicks = periodTicks_ - highTicks_;
    if (lowTicks == 0) {
      pwmActive_ = false;
      timerAlarmDisable(timer_);
      portEXIT_CRITICAL_ISR(&timerMux_);
      return;
    }
    timerAlarmWrite(timer_, lowTicks, false);
    timerAlarmEnable(timer_);
  } else {
    setActivePinHighUnlocked();
    phaseHigh_ = true;
    timerAlarmWrite(timer_, highTicks_, false);
    timerAlarmEnable(timer_);
  }

  portEXIT_CRITICAL_ISR(&timerMux_);
}

