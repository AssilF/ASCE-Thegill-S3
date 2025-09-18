#pragma once

#include <Arduino.h>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <esp_timer.h>

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
  static void TimerCallback(void *arg);
  void handleTimerCallback();

  uint8_t pinNumber_ = 0;
  gpio_num_t gpioPin_ = GPIO_NUM_NC;
  bool playing_ = false;
  bool looping_ = false;
  bool inPause_ = false;
  const ToneStep *sequence_ = nullptr;
  std::size_t length_ = 0;
  std::size_t currentIndex_ = 0;
  uint32_t nextChangeMs_ = 0;
  uint16_t pendingPauseMs_ = 0;
  esp_timer_handle_t toneTimer_ = nullptr;
  std::atomic<bool> toneActive_{false};
  std::atomic<bool> pinState_{false};
};

