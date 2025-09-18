#include <device_config.h>
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#ifdef ARDUINO_ARCH_ESP32
#include <sdkconfig.h>
#endif
#include <buzzer_controller.h>
#include <status_led.h>
#include <comms.h>
#include <network_setup.h>
#include <control_system.h>

namespace {
constexpr ToneStep kPairingSequence[] = {
    {523, 160, 30},   // C5
    {0, 60, 0},
    {659, 200, 30},   // E5
    {0, 60, 0},
    {784, 260, 120},  // G5
};

constexpr ToneStep kConnectedSequence[] = {
    {784, 160, 20},   // G5
    {988, 160, 30},   // B5
    {1175, 240, 160}, // D6
};

constexpr ToneStep kFailsafeSequence[] = {
    {880, 200, 20},  // A5
    {698, 220, 20},  // F5
    {523, 420, 200}, // C5
};

constexpr UBaseType_t kUpdateTaskPriority = 2;
constexpr uint32_t kUpdateTaskStackWords = 4096;
constexpr TickType_t kUpdateTaskDelayTicks = pdMS_TO_TICKS(10);
#if defined(ARDUINO_ARCH_ESP32) && !(defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE)
constexpr BaseType_t kUpdateTaskCore = 1; // APP CPU keeps Wi-Fi/ESPNOW on core 0 responsive
#else
constexpr BaseType_t kUpdateTaskCore = tskNO_AFFINITY;
#endif
} // namespace

void ControlSystem::begin() {
  Serial.println("ControlSystem initialization started");

  statusLed_.begin(config::kStatusLedPin);
  statusLed_.setMode(StatusLed::Mode::kBoot);
  statusLed_.setDriveBalance(0.0f, 0.0f);

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    motors_[i].begin(config::kMotorPins[i]);
    lastMotorCommands_[i] = 0.0f;
  }

  buzzer_.begin(config::kBuzzerPin);
  buzzer_.playBootSequence();

  Serial.printf("Starting communications on SSID %s channel %u\n", config::kAccessPointSsid,
                static_cast<unsigned>(config::kEspNowChannel));
  if (!Comms::init(config::kAccessPointSsid, config::kAccessPointPassword, config::kEspNowChannel)) {
    Serial.println("Failed to initialize communications");
  }
  
  WiFi.softAPmacAddress(selfMac_);
  Serial.printf("SoftAP MAC address %02X:%02X:%02X:%02X:%02X:%02X\n", selfMac_[0], selfMac_[1], selfMac_[2],
                selfMac_[3], selfMac_[4], selfMac_[5]);

  ConfigureOta(
      config::kDeviceIdentity, config::kAccessPointPassword, config::kAccessPointSsid,
      []() { Serial.println("OTA update started"); },
      [this]() {
        Serial.println("OTA update finished");
        buzzer_.playSequence(kPairingSequence, ToneSequenceLength(kPairingSequence));
      },
      [](unsigned int progress, unsigned int total) {
        Serial.printf("OTA progress: %u%%\n", (progress * 100U) / total);
      },
      [](ota_error_t error) { Serial.printf("OTA Error[%u]\n", error); });

  if (updateTaskHandle_ == nullptr) {
#if defined(ARDUINO_ARCH_ESP32) && !(defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE)
    BaseType_t result = xTaskCreatePinnedToCore(UpdateTaskTrampoline, "ControlUpdate", kUpdateTaskStackWords, this,
                                                kUpdateTaskPriority, &updateTaskHandle_, kUpdateTaskCore);
#else
    BaseType_t result = xTaskCreate(UpdateTaskTrampoline, "ControlUpdate", kUpdateTaskStackWords, this,
                                    kUpdateTaskPriority, &updateTaskHandle_);
#endif
    if (result != pdPASS) {
      Serial.println("Failed to create control update task");
      updateTaskHandle_ = nullptr;
    }
  }

  pendingPairingTone_ = false;
  paired_ = false;
  failsafeActive_ = false;
  bootGreetingComplete_ = false;
  lastCommandTimestamp_ = 0;

  Serial.println("ControlSystem initialization complete");
}

void ControlSystem::loop() { vTaskDelay(kUpdateTaskDelayTicks); }

void ControlSystem::UpdateTaskTrampoline(void *param) {
  auto *self = static_cast<ControlSystem *>(param);
  if (self != nullptr) {
    self->updateTask();
  }
  vTaskDelete(nullptr);
}

void ControlSystem::updateTask() {
  Comms::DriveCommand command{};
  uint32_t lastAppliedTimestamp = 0;

  while (true) {
    buzzer_.update();
    statusLed_.update();

    if (!bootGreetingComplete_) {
      if (!buzzer_.isPlaying()) {
        bootGreetingComplete_ = true;
        if (paired_) {
          updateStatusForConnection();
        } else {
          updateStatusForPairing();
        }
      }
    } else if (pendingPairingTone_ && !buzzer_.isPlaying()) {
      buzzer_.playSequence(kPairingSequence, ToneSequenceLength(kPairingSequence));
      pendingPairingTone_ = false;
    }

    HandleOtaLoop();

    if (!paired_ && Comms::paired()) {
      paired_ = true;
      pendingPairingTone_ = false;
      stopAllMotors();
      const uint8_t *mac = Comms::controllerMac();
      const char *identity = Comms::controllerIdentity();
      Serial.printf("Paired with controller %s (%02X:%02X:%02X:%02X:%02X:%02X)\n",
                    (identity != nullptr) ? identity : "",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      buzzer_.playSequence(kConnectedSequence, ToneSequenceLength(kConnectedSequence));
      updateStatusForConnection();
    }

    const uint32_t commandTimestamp = Comms::lastCommandTimestamp();
    if (commandTimestamp != 0 && commandTimestamp != lastAppliedTimestamp &&
        Comms::receiveCommand(command)) {
      applyDriveCommand(command, commandTimestamp);
      lastAppliedTimestamp = commandTimestamp;
    }

    if (paired_) {
      const uint32_t now = millis();
      if (lastCommandTimestamp_ == 0 || (now - lastCommandTimestamp_) > config::kControlTimeoutMs) {
        enterFailsafe();
      }
    }

    vTaskDelay(kUpdateTaskDelayTicks);
  }
}

void ControlSystem::applyDriveCommand(const Comms::DriveCommand &command, uint32_t timestamp) {
  if (command.magic != Comms::kDriveCommandMagic) {
    return;
  }

  exitFailsafe();

  const bool armed = command.armMotors;

  if (!armed) {
    stopAllMotors();
    lastCommandTimestamp_ = timestamp;
    updateStatusForConnection();
    return;
  }

  auto clamp = [](float value) {
    if (value > 1.0f) {
      return 1.0f;
    }
    if (value < -1.0f) {
      return -1.0f;
    }
    return value;
  };

  const float throttle = (static_cast<int32_t>(command.throttle) - 500) / 500.0f;
  const float yaw = static_cast<float>(command.yawAngle) / 90.0f;
  const float pitch = static_cast<float>(command.pitchAngle) / 90.0f;
  const float roll = static_cast<float>(command.rollAngle) / 90.0f;

  const float leftBase = throttle - yaw;
  const float rightBase = throttle + yaw;
  const float frontAdjust = pitch;
  const float rearAdjust = -pitch;
  const float leftAdjust = -roll;
  const float rightAdjust = roll;

  const float mixes[4] = {
      clamp(leftBase + frontAdjust + leftAdjust),
      clamp(leftBase + rearAdjust + leftAdjust),
      clamp(rightBase + frontAdjust + rightAdjust),
      clamp(rightBase + rearAdjust + rightAdjust),
  };

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    const float duty = (i < 4) ? mixes[i] : clamp(throttle);
    motors_[i].applyCommand(duty, false);
    lastMotorCommands_[i] = duty;
  }

  lastCommandTimestamp_ = timestamp;
  updateStatusForConnection();
}

void ControlSystem::stopAllMotors() {
  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    motors_[i].stop();
    lastMotorCommands_[i] = 0.0f;
  }
  statusLed_.setDriveBalance(0.0f, 0.0f);
}

void ControlSystem::enterFailsafe() {
  if (failsafeActive_) {
    return;
  }

  stopAllMotors();
  buzzer_.playSequence(kFailsafeSequence, ToneSequenceLength(kFailsafeSequence));
  pendingPairingTone_ = false;
  failsafeActive_ = true;
  statusLed_.setMode(StatusLed::Mode::kFailsafe);
}

void ControlSystem::exitFailsafe() {
  if (!failsafeActive_) {
    return;
  }

  failsafeActive_ = false;
  buzzer_.stop();
  if (paired_) {
    updateStatusForConnection();
  } else {
    updateStatusForPairing();
  }
}

void ControlSystem::updateDriveIndicator() {
  const std::size_t motorsPerSide = config::kMotorCount / 2;
  float leftSum = 0.0f;
  float rightSum = 0.0f;

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    const float magnitude = fabsf(lastMotorCommands_[i]);
    if (i < motorsPerSide) {
      leftSum += magnitude;
    } else {
      rightSum += magnitude;
    }
  }

  const float leftAvg = leftSum / static_cast<float>(motorsPerSide);
  const float rightAvg = rightSum / static_cast<float>(motorsPerSide);
  statusLed_.setDriveBalance(leftAvg, rightAvg);
}

void ControlSystem::updateStatusForPairing() {
  statusLed_.setMode(StatusLed::Mode::kPairing);
  statusLed_.setDriveBalance(0.0f, 0.0f);
  if (bootGreetingComplete_) {
    pendingPairingTone_ = true;
  }
}

void ControlSystem::updateStatusForConnection() {
  pendingPairingTone_ = false;
  statusLed_.setMode(StatusLed::Mode::kConnected);
  updateDriveIndicator();
}

