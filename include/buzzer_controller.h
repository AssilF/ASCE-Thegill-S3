#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "device_config.h"

struct ToneStep {
  uint16_t frequencyHz;
  uint16_t durationMs;
  uint16_t pauseMs;
};

template <std::size_t N>
constexpr std::size_t ToneSequenceLength(const ToneStep (&)[N]) {
  return N;
}

class BuzzerController {
public:
  void begin(uint8_t pin);
  void update();
  void playSequence(const ToneStep *sequence, std::size_t length, bool loop = false);
  void playBootSequence(bool loop = false);
  void stop();
  bool isPlaying() const;

private:
  void loadNextStep();
  void startTone(uint16_t frequencyHz);
  void stopToneOutput();
  bool configureToneTimer(uint32_t frequencyHz);

  uint8_t pinNumber_ = 0;
  bool playing_ = false;
  bool looping_ = false;
  bool inPause_ = false;
  const ToneStep *sequence_ = nullptr;
  std::size_t length_ = 0;
  std::size_t currentIndex_ = 0;
  uint32_t nextChangeMs_ = 0;
  uint16_t pendingPauseMs_ = 0;
  uint8_t channel_ = config::kBuzzerChannel;
  bool initialized_ = false;
  bool timerConfigured_ = false;
  uint32_t requestedFrequencyHz_ = 0;
  uint32_t currentFrequencyHz_ = 0;
  uint8_t currentResolutionBits_ = config::kBuzzerResolutionBits;
  bool pendingGuardStep_ = false;
  ToneStep guardStep_{};
  uint32_t guardReleaseMs_ = 0;
};

