#include "buzzer_controller.h"

namespace {
constexpr ToneStep kDefaultBootSequence[] = {
    {0, 80, 40},
    {220, 160, 20},  // A3
    {0, 40, 0},
    {277, 160, 20},  // C#4
    {0, 40, 0},
    {330, 200, 20},  // E4
    {392, 200, 30},  // G4
    {523, 240, 60},  // C5
    {0, 100, 0},
    {698, 220, 20},  // F5
    {880, 260, 40},  // A5
    {1108, 320, 220}, // C#6
};
} // namespace

void BuzzerController::begin(uint8_t pin) {
  pinNumber_ = pin;
  ledcSetup(channel_, 2000, config::kBuzzerResolutionBits);
  ledcAttachPin(pinNumber_, channel_);
  ledcWrite(channel_, 0);
}

void BuzzerController::update() {
  const uint32_t now = millis();
  if (!playing_ || sequence_ == nullptr) {
    return;
  }

  if (now >= nextChangeMs_) {
    if (inPause_) {
      inPause_ = false;
      loadNextStep();
    } else if (pendingPauseMs_ > 0) {
      stopToneOutput();
      nextChangeMs_ = now + pendingPauseMs_;
      pendingPauseMs_ = 0;
      inPause_ = true;
    } else {
      loadNextStep();
    }
  }
}

void BuzzerController::playSequence(const ToneStep *sequence, std::size_t length, bool loop) {
  sequence_ = sequence;
  length_ = length;
  looping_ = loop;
  currentIndex_ = 0;
  playing_ = true;
  inPause_ = false;
  pendingPauseMs_ = 0;
  nextChangeMs_ = millis();
  loadNextStep();
}

void BuzzerController::stop() {
  if (!playing_) {
    return;
  }
  stopToneOutput();
  playing_ = false;
  sequence_ = nullptr;
  currentIndex_ = 0;
  pendingPauseMs_ = 0;
  inPause_ = false;
}

bool BuzzerController::isPlaying() const { return playing_; }

void BuzzerController::playBootSequence(bool loop) {
  playSequence(kDefaultBootSequence, ToneSequenceLength(kDefaultBootSequence), loop);
}

void BuzzerController::loadNextStep() {
  if (sequence_ == nullptr || length_ == 0) {
    stop();
    return;
  }

  if (currentIndex_ >= length_) {
    if (looping_) {
      currentIndex_ = 0;
    } else {
      stop();
      return;
    }
  }

  const ToneStep &step = sequence_[currentIndex_++];
  if (step.frequencyHz == 0) {
    stopToneOutput();
  } else {
    startTone(step.frequencyHz);
  }
  nextChangeMs_ = millis() + step.durationMs;

  if (step.pauseMs > 0) {
    pendingPauseMs_ = step.pauseMs;
    if (step.durationMs == 0) {
      stopToneOutput();
      nextChangeMs_ = millis() + pendingPauseMs_;
      pendingPauseMs_ = 0;
    }
  } else {
    pendingPauseMs_ = 0;
  }
}

void BuzzerController::startTone(uint16_t frequencyHz) {
  if (frequencyHz == 0) {
    stopToneOutput();
    return;
  }

  ledcWriteTone(channel_, static_cast<double>(frequencyHz));
  const uint32_t duty = 1U << (config::kBuzzerResolutionBits - 1U);
  ledcWrite(channel_, duty);
}

void BuzzerController::stopToneOutput() {
  ledcWriteTone(channel_, 0);
  ledcWrite(channel_, 0);
}

