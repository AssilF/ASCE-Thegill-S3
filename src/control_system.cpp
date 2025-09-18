#include <device_config.h>
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <buzzer_controller.h>
#include <status_led.h>
#include <comms.h>
#include <network_setup.h>
#include <control_protocol.h>
#include <control_system.h>

namespace {
constexpr ToneStep kPairingSequence[] = {
    {500, 4000, 2000}, // E6
    {4000, 2000, 3000},  // A6
};

constexpr ToneStep kConnectedSequence[] = {
    {500, 4000, 2000}, // E6
    {4000, 2000, 3000},  // A6
};

constexpr ToneStep kHonkSequence[] = {
    {500, 4000, 2000}, // E6
    {4000, 2000, 3000},  // A6
};

constexpr ToneStep kFailsafeSequence[] = {
    {500, 4000, 2000}, // E6
    {4000, 2000, 3000},  // A6
};

constexpr UBaseType_t kUpdateTaskPriority = 2;
constexpr uint32_t kUpdateTaskStackWords = 4096;
constexpr TickType_t kUpdateTaskDelayTicks = pdMS_TO_TICKS(10);
} // namespace

void ControlSystem::begin() {
  Serial.println("ControlSystem initialization started");

  statusLed_.begin(config::kStatusLedPin);
  statusLed_.setMode(StatusLed::Mode::kPairing);

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
    BaseType_t result = xTaskCreate(UpdateTaskTrampoline, "ControlUpdate", kUpdateTaskStackWords, this,
                                    kUpdateTaskPriority, &updateTaskHandle_);
    if (result != pdPASS) {
      Serial.println("Failed to create control update task");
      updateTaskHandle_ = nullptr;
    }
  }

  pendingPairingTone_ = true;
  paired_ = false;
  failsafeActive_ = false;
  lastCommandTimestamp_ = 0;
  updateStatusForPairing();

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

    if (pendingPairingTone_ && !buzzer_.isPlaying()) {
      buzzer_.playSequence(kPairingSequence, ToneSequenceLength(kPairingSequence));
      pendingPairingTone_ = false;
    }

    HandleOtaLoop();

    if (!paired_ && Comms::paired()) {
      paired_ = true;
      pendingPairingTone_ = false;
      stopAllMotors();
      const uint8_t *mac = Comms::BroadcastMac;
      const char *identity = Comms::DRONE_IDENTITY;
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
  if (command.version != protocol::kControlProtocolVersion) {
    return;
  }

  const bool honk = (command.flags & protocol::kControlFlagHonk) != 0;

  exitFailsafe();

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    const float duty = static_cast<float>(command.motorDuty[i]) / 1000.0f;
    const bool brake = (command.flags & protocol::BrakeFlagForMotor(i)) != 0;
    motors_[i].applyCommand(duty, brake);
    lastMotorCommands_[i] = brake ? 0.0f : duty;
  }

  lastCommandTimestamp_ = timestamp;
  updateStatusForConnection();

  if (honk) {
    buzzer_.playSequence(kHonkSequence, ToneSequenceLength(kHonkSequence));
  }
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
}

void ControlSystem::updateStatusForConnection() {
  statusLed_.setMode(StatusLed::Mode::kConnected);
  updateDriveIndicator();
}

