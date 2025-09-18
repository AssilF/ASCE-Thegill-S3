#include "status_led.h"

#include <Arduino.h>
#include <math.h>

StatusLed::StatusLed() : pixel_(1, -1, NEO_GRB + NEO_KHZ800) {}

void StatusLed::begin(uint8_t pin) {
  pixel_.setPin(pin);
  pixel_.begin();
  pixel_.show();
  modeChangeMs_ = millis();
}

void StatusLed::setMode(Mode mode) {
  if (mode_ == mode) {
    return;
  }
  mode_ = mode;
  modeChangeMs_ = millis();
}

void StatusLed::setDriveBalance(float leftIntensity, float rightIntensity) {
  leftIntensity_ = constrain(leftIntensity, 0.0f, 1.0f);
  rightIntensity_ = constrain(rightIntensity, 0.0f, 1.0f);
}

void StatusLed::update() {
  const uint32_t now = millis();
  switch (mode_) {
  case Mode::kBoot: {
    float phase = fmodf(static_cast<float>(now - modeChangeMs_), 2000.0f) / 2000.0f;
    float brightness = 0.3f + 0.7f * (0.5f - 0.5f * cosf(phase * 2.0f * PI));
    showColor(0, 0, static_cast<uint8_t>(brightness * 255.0f));
    break;
  }
  case Mode::kPairing: {
    float phase = fmodf(static_cast<float>(now - modeChangeMs_), 1500.0f) / 1500.0f;
    float brightness = 0.2f + 0.8f * fabsf(sinf(phase * PI));
    showColor(static_cast<uint8_t>(brightness * 40.0f), static_cast<uint8_t>(brightness * 200.0f),
              static_cast<uint8_t>(brightness * 120.0f));
    break;
  }
  case Mode::kConnected: {
    const float left = leftIntensity_;
    const float right = rightIntensity_;
    const uint8_t red = static_cast<uint8_t>(constrain(right, 0.0f, 1.0f) * 255.0f);
    const uint8_t green = static_cast<uint8_t>(constrain(left, 0.0f, 1.0f) * 255.0f);
    const uint8_t blue = (left < 0.05f && right < 0.05f) ? 30 : 0;
    showColor(red, green, blue);
    break;
  }
  case Mode::kFailsafe: {
    bool on = ((now - modeChangeMs_) / 200) % 2 == 0;
    showColor(on ? 255 : 0, 0, 0);
    break;
  }
  }
}

void StatusLed::showColor(uint8_t red, uint8_t green, uint8_t blue) {
  if (currentRed_ == red && currentGreen_ == green && currentBlue_ == blue) {
    return;
  }
  currentRed_ = red;
  currentGreen_ = green;
  currentBlue_ = blue;
  pixel_.setPixelColor(0, pixel_.Color(red, green, blue));
  pixel_.show();
}

