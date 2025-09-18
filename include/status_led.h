#pragma once

#include <Adafruit_NeoPixel.h>
#include <cstdint>

class StatusLed {
public:
  enum class Mode {
    kBoot,
    kPairing,
    kConnected,
    kFailsafe,
  };

  StatusLed();

  void begin(uint8_t pin);
  void setMode(Mode mode);
  void setDriveBalance(float leftIntensity, float rightIntensity);
  void update();

private:
  void showColor(uint8_t red, uint8_t green, uint8_t blue);

  Adafruit_NeoPixel pixel_;
  Mode mode_ = Mode::kBoot;
  uint32_t modeChangeMs_ = 0;
  float leftIntensity_ = 0.0f;
  float rightIntensity_ = 0.0f;
  uint8_t currentRed_ = 0;
  uint8_t currentGreen_ = 0;
  uint8_t currentBlue_ = 0;
};

