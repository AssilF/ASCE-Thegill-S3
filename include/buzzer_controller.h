#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

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
  void begin(uint8_t pin, uint8_t channel, uint8_t resolutionBits);
  void update();
  void playSequence(const ToneStep *sequence, std::size_t length, bool loop = false);
  void playBootSequence(bool loop = false);
  void stop();
  bool isPlaying() const;

private:
  void loadNextStep();

  uint8_t channel_ = 0;
  uint8_t resolutionBits_ = 0;
  bool playing_ = false;
  bool looping_ = false;
  bool inPause_ = false;
  const ToneStep *sequence_ = nullptr;
  std::size_t length_ = 0;
  std::size_t currentIndex_ = 0;
  uint32_t nextChangeMs_ = 0;
  uint16_t pendingPauseMs_ = 0;
};

