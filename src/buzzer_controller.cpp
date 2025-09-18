#include "buzzer_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
  pinMode(pinNumber_, OUTPUT);
  digitalWrite(pinNumber_, LOW);

  const double initialFrequency = 2000.0;
  ledcSetup(channel_, initialFrequency, config::kBuzzerResolutionBits);
  ledcAttachPin(pinNumber_, channel_);
  ledcWrite(channel_, 0);

  initialized_ = true;
  timerConfigured_ = false;
  requestedFrequencyHz_ = 0;
  currentFrequencyHz_ = 0;
  currentResolutionBits_ = config::kBuzzerResolutionBits;
  pendingGuardStep_ = false;
  guardReleaseMs_ = 0;
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
  pendingGuardStep_ = false;
  guardReleaseMs_ = 0;
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
  pendingGuardStep_ = false;
  guardReleaseMs_ = 0;
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

  if (pendingGuardStep_ && guardReleaseMs_ != 0) {
    const uint32_t nowGuard = millis();
    if (nowGuard < guardReleaseMs_) {
      inPause_ = true;
      nextChangeMs_ = guardReleaseMs_;
      return;
    }
  }

  ToneStep step{};
  const uint32_t now = millis();
  if (pendingGuardStep_) {
    step = guardStep_;
  } else {
    step = sequence_[currentIndex_];
    guardStep_ = step;
  }

  if (!pendingGuardStep_ && requestedFrequencyHz_ != 0 && step.frequencyHz != 0) {
    const uint32_t previous = requestedFrequencyHz_;
    const uint32_t next = step.frequencyHz;
    const uint32_t delta = (previous > next) ? (previous - next) : (next - previous);
    const uint32_t maxFrequency = std::max(previous, next);
    const bool deltaLargeEnough = delta >= config::kBuzzerRetuneGuardMinDeltaHz;
    const bool ratioLargeEnough =
        (maxFrequency > 0) && (delta * 100U >= maxFrequency * config::kBuzzerRetuneGuardDeltaPercent);
    if (deltaLargeEnough && ratioLargeEnough) {
      stopToneOutput();
      pendingGuardStep_ = true;
      guardReleaseMs_ = now + config::kBuzzerRetuneGuardMs;
      inPause_ = true;
      nextChangeMs_ = guardReleaseMs_;
      return;
    }
  }

  pendingGuardStep_ = false;
  guardReleaseMs_ = 0;
  ++currentIndex_;

  if (step.frequencyHz == 0) {
    stopToneOutput();
  } else {
    startTone(step.frequencyHz);
  }
  nextChangeMs_ = now + step.durationMs;

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
  if (!initialized_) {
    return;
  }

  if (frequencyHz == 0) {
    stopToneOutput();
    return;
  }

  if (!configureToneTimer(frequencyHz)) {
    stopToneOutput();
    return;
  }

  if (currentResolutionBits_ == 0) {
    stopToneOutput();
    return;
  }

  const uint32_t maxDuty = (1UL << currentResolutionBits_) - 1UL;
  const uint32_t duty = (maxDuty + 1UL) / 2UL;
  ledcWrite(channel_, duty);
}

void BuzzerController::stopToneOutput() {
  if (!initialized_) {
    return;
  }
  ledcWrite(channel_, 0);
  timerConfigured_ = false;
  requestedFrequencyHz_ = 0;
  currentFrequencyHz_ = 0;
}

bool BuzzerController::configureToneTimer(uint32_t frequencyHz) {
  if (!initialized_ || frequencyHz == 0) {
    return false;
  }

  if (timerConfigured_ && requestedFrequencyHz_ == frequencyHz) {
    return true;
  }

  const uint8_t previousResolution = currentResolutionBits_;
  const uint32_t previousFrequency = currentFrequencyHz_;
  const uint32_t previousRequest = requestedFrequencyHz_;
  const bool hadPreviousConfig = timerConfigured_;

  ledcWrite(channel_, 0);

  const double target = static_cast<double>(frequencyHz);
  const uint8_t maxResolution = config::kBuzzerResolutionBits;
  const uint8_t minResolution = std::max<uint8_t>(1, config::kBuzzerMinResolutionBits);
  constexpr double kExactMatchTolerance = 1e-6;

  uint8_t bestResolution = 0;
  uint32_t bestFrequency = 0;
  double bestError = std::numeric_limits<double>::infinity();

  for (int bits = static_cast<int>(maxResolution); bits >= static_cast<int>(minResolution); --bits) {
    const uint32_t actual = ledcSetup(channel_, target, static_cast<uint8_t>(bits));
    if (actual == 0) {
      continue;
    }

    const double relativeError = std::fabs(static_cast<double>(actual) - target) / target;
    if (relativeError < bestError) {
      bestError = relativeError;
      bestFrequency = actual;
      bestResolution = static_cast<uint8_t>(bits);
      if (relativeError <= kExactMatchTolerance) {
        break;
      }
    }
  }

  if (bestResolution == 0) {
    if (hadPreviousConfig && previousFrequency != 0) {
      ledcSetup(channel_, static_cast<double>(previousFrequency), previousResolution);
    }
    timerConfigured_ = hadPreviousConfig;
    currentResolutionBits_ = previousResolution;
    currentFrequencyHz_ = previousFrequency;
    requestedFrequencyHz_ = previousRequest;
    return false;
  }

  const uint32_t applied = ledcSetup(channel_, target, bestResolution);
  if (applied == 0) {
    if (hadPreviousConfig && previousFrequency != 0) {
      ledcSetup(channel_, static_cast<double>(previousFrequency), previousResolution);
    }
    timerConfigured_ = hadPreviousConfig;
    currentResolutionBits_ = previousResolution;
    currentFrequencyHz_ = previousFrequency;
    requestedFrequencyHz_ = previousRequest;
    return false;
  }

  currentResolutionBits_ = bestResolution;
  currentFrequencyHz_ = applied;
  requestedFrequencyHz_ = frequencyHz;
  timerConfigured_ = true;
  return true;
}

