#include "status_led.h"

#include <Arduino.h>
#include <algorithm>
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
  const uint32_t elapsed = now - modeChangeMs_;

  switch (mode_) {
  case Mode::kBoot: {
    const uint32_t cycle = 2800;
    const uint32_t t = elapsed % cycle;
    if (t < 700) {
      const float progress = static_cast<float>(t) / 700.0f;
      const uint8_t blue = static_cast<uint8_t>(80.0f + 175.0f * progress);
      const uint8_t green = static_cast<uint8_t>(20.0f + 100.0f * progress);
      showColor(0, green, blue);
    } else if (t < 1400) {
      const float progress = static_cast<float>(t - 700) / 700.0f;
      const float pulse = 0.5f + 0.5f * sinf(progress * PI);
      showColor(static_cast<uint8_t>(40.0f * pulse), static_cast<uint8_t>(80.0f + 120.0f * pulse),
                static_cast<uint8_t>(100.0f + 120.0f * pulse));
    } else if (t < 2000) {
      const float progress = static_cast<float>(t - 1400) / 600.0f;
      const float intensity = 1.0f - fabsf(2.0f * progress - 1.0f);
      const uint8_t value = static_cast<uint8_t>(180.0f + 75.0f * intensity);
      showColor(value, value, value);
    } else {
      float progress = static_cast<float>(t - 2000) / 800.0f;
      progress = constrain(progress, 0.0f, 1.0f);
      const uint8_t red = static_cast<uint8_t>(120.0f + 135.0f * progress);
      const uint8_t green = static_cast<uint8_t>(25.0f * (1.0f - progress));
      const uint8_t blue = static_cast<uint8_t>(30.0f * (1.0f - progress));
      showColor(red, green, blue);
    }
    break;
  }
  case Mode::kPairing: {
    const float sweepPhase = fmodf(static_cast<float>(elapsed), 1100.0f) / 1100.0f;
    const float sweep = 0.5f + 0.5f * sinf(sweepPhase * 2.0f * PI);
    const float sparkle = 0.5f + 0.5f * sinf(static_cast<float>(elapsed % 220) / 220.0f * 2.0f * PI);
    const uint8_t red = static_cast<uint8_t>(60.0f + 160.0f * sweep);
    const uint8_t green = static_cast<uint8_t>(40.0f + 80.0f * sparkle);
    const uint8_t blue = static_cast<uint8_t>(90.0f + 130.0f * (1.0f - sweep));
    showColor(red, green, blue);
    break;
  }
  case Mode::kConnected: {
    const float left = constrain(leftIntensity_, 0.0f, 1.0f);
    const float right = constrain(rightIntensity_, 0.0f, 1.0f);
    const float thrust = constrain((left + right) * 0.5f, 0.0f, 1.0f);
    const float difference = constrain(right - left, -1.0f, 1.0f);
    const float accent = 0.5f + 0.5f * sinf(static_cast<float>(elapsed % 1400) / 1400.0f * 2.0f * PI);
    const uint8_t red = static_cast<uint8_t>(80.0f + 140.0f * std::max(difference, 0.0f) + 60.0f * thrust * accent);
    const uint8_t green = static_cast<uint8_t>(80.0f + 140.0f * std::max(-difference, 0.0f) + 60.0f * thrust * accent);
    const uint8_t blue = static_cast<uint8_t>(30.0f + 100.0f * (1.0f - thrust) * accent);
    showColor(red, green, blue);
    break;
  }
  case Mode::kFailsafe: {
    const uint32_t cycle = 900;
    const uint32_t t = elapsed % cycle;
    const bool on = (t < 150) || (t >= 240 && t < 390);
    const uint8_t red = on ? 255 : 40;
    const uint8_t blue = on ? 50 : 0;
    showColor(red, 0, blue);
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

