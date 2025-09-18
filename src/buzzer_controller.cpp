#include "buzzer_controller.h"

namespace {
constexpr ToneStep kDefaultBootSequence[] = {
    {784, 784, 784},  // G5
    {988, 988, 988},  // B5
    {1568, 1568, 1568},  // G6
};
} // namespace

void BuzzerController::begin(uint8_t pin) {
  pinNumber_ = pin;
  gpioPin_ = static_cast<gpio_num_t>(pin);
  pinMode(pinNumber_, OUTPUT);
  digitalWrite(pinNumber_, LOW);

  if (toneTimer_ == nullptr) {
    esp_timer_create_args_t args{};
    args.callback = &BuzzerController::TimerCallback;
    args.arg = this;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "buzzer";

    const esp_err_t result = esp_timer_create(&args, &toneTimer_);
    if (result != ESP_OK) {
      Serial.printf("Failed to create buzzer timer (err=%d)\n", static_cast<int>(result));
    }
  }
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
  if (frequencyHz == 0 || toneTimer_ == nullptr || gpioPin_ == GPIO_NUM_NC) {
    stopToneOutput();
    return;
  }

  uint32_t periodUs = frequencyHz > 0 ? static_cast<uint32_t>(1000000UL / frequencyHz) : 0UL;
  if (periodUs == 0) {
    periodUs = 1;
  }
  const uint64_t halfPeriodUs = static_cast<uint64_t>(periodUs) / 2ULL;
  const uint64_t intervalUs = halfPeriodUs > 0 ? halfPeriodUs : 1ULL;

  esp_timer_stop(toneTimer_);
  pinState_.store(false, std::memory_order_relaxed);
  gpio_set_level(gpioPin_, 0);

  toneActive_.store(true, std::memory_order_release);
  if (esp_timer_start_periodic(toneTimer_, intervalUs) != ESP_OK) {
    toneActive_.store(false, std::memory_order_release);
    Serial.println("Failed to start buzzer timer");
  }
}

void BuzzerController::stopToneOutput() {
  toneActive_.store(false, std::memory_order_release);
  if (toneTimer_ != nullptr) {
    esp_timer_stop(toneTimer_);
  }
  pinState_.store(false, std::memory_order_relaxed);
  if (gpioPin_ != GPIO_NUM_NC) {
    gpio_set_level(gpioPin_, 0);
  }
}

void BuzzerController::TimerCallback(void *arg) {
  if (arg == nullptr) {
    return;
  }
  static_cast<BuzzerController *>(arg)->handleTimerCallback();
}

void BuzzerController::handleTimerCallback() {
  if (!toneActive_.load(std::memory_order_acquire) || gpioPin_ == GPIO_NUM_NC) {
    return;
  }

  const bool newState = !pinState_.load(std::memory_order_relaxed);
  pinState_.store(newState, std::memory_order_relaxed);
  gpio_set_level(gpioPin_, newState ? 1 : 0);
}

