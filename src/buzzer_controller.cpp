#include "buzzer_controller.h"

void BuzzerController::begin(uint8_t pin, uint8_t channel, uint8_t resolutionBits) {
  pinMode(pin, OUTPUT);
  ledcSetup(channel, 2000, resolutionBits);
  ledcAttachPin(pin, channel);
  channel_ = channel;
  resolutionBits_ = resolutionBits;
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
      ledcWriteTone(channel_, 0);
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
  ledcWriteTone(channel_, 0);
  playing_ = false;
  sequence_ = nullptr;
  currentIndex_ = 0;
  pendingPauseMs_ = 0;
  inPause_ = false;
}

bool BuzzerController::isPlaying() const { return playing_; }

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
    ledcWriteTone(channel_, 0);
  } else {
    ledcWriteTone(channel_, step.frequencyHz);
  }
  nextChangeMs_ = millis() + step.durationMs;

  if (step.pauseMs > 0) {
    pendingPauseMs_ = step.pauseMs;
    if (step.durationMs == 0) {
      ledcWriteTone(channel_, 0);
      nextChangeMs_ = millis() + pendingPauseMs_;
      pendingPauseMs_ = 0;
    }
  } else {
    pendingPauseMs_ = 0;
  }
}

