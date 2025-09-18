#include "control_system.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

#include "network_setup.h"

namespace {
constexpr ToneStep kPairingSequence[] = {
    {1319, 120, 20}, // E6
    {1760, 120, 0},  // A6
};

constexpr ToneStep kConnectedSequence[] = {
    {880, 120, 10}, // A5
    {1175, 200, 0}, // D6
};

constexpr ToneStep kHonkSequence[] = {
    {392, 300, 20}, // G4
    {311, 300, 0},  // D#4
};

constexpr ToneStep kFailsafeSequence[] = {
    {220, 180, 20}, // A3
    {0, 120, 0},
    {220, 180, 0},
};

constexpr uint32_t kHandshakeCooldownMs = 500;
constexpr UBaseType_t kEspNowTaskPriority = 3;
constexpr UBaseType_t kUpdateTaskPriority = 2;
constexpr std::size_t kEspNowQueueLength = 10;
constexpr uint32_t kEspNowTaskStackWords = 4096;
constexpr uint32_t kUpdateTaskStackWords = 4096;
constexpr TickType_t kUpdateTaskDelayTicks = pdMS_TO_TICKS(10);

const char *MessageTypeName(protocol::MessageType type) {
  switch (type) {
  case protocol::MessageType::kScanRequest:
    return "scan request";
  case protocol::MessageType::kDroneIdentity:
    return "drone identity";
  case protocol::MessageType::kControllerIdentity:
    return "controller identity";
  case protocol::MessageType::kDroneAck:
    return "drone ack";
  case protocol::MessageType::kControlCommand:
    return "control command";
  default:
    return "unknown";
  }
}
} // namespace

ControlSystem *ControlSystem::instance_ = nullptr;

void ControlSystem::begin() {
  instance_ = this;

  Serial.println("ControlSystem initialization started");
  statusLed_.begin(config::kStatusLedPin);
  statusLed_.setMode(StatusLed::Mode::kPairing);

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    Serial.printf("Initializing motor %u on pin %u\n", static_cast<unsigned>(i),
                  static_cast<unsigned>(config::kMotorPins[i]));
    motors_[i].begin(config::kMotorPins[i]);
    lastMotorCommands_[i] = 0.0f;
  }

  buzzer_.begin(config::kBuzzerPin, config::kBuzzerChannel, config::kBuzzerResolutionBits);
  buzzer_.playBootSequence();

  ConfigureWiFi(config::kDeviceIdentity, config::kAccessPointSsid, config::kAccessPointPassword,
                config::kEspNowChannel);
  WiFi.softAPmacAddress(selfMac_);
  Serial.printf("SoftAP MAC address %02X:%02X:%02X:%02X:%02X:%02X\n", selfMac_[0], selfMac_[1], selfMac_[2],
                selfMac_[3], selfMac_[4], selfMac_[5]);
  lastHandshakeTimestamp_ = 0;
  if (!InitializeEspNow(&ControlSystem::EspNowReceiveTrampoline)) {
    Serial.println("ESP-NOW callback registration failed");
  }

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

  if (espNowQueue_ == nullptr) {
    espNowQueue_ = xQueueCreate(kEspNowQueueLength, sizeof(EspNowPacket));
    if (espNowQueue_ == nullptr) {
      Serial.println("Failed to allocate ESP-NOW queue");
    }
  }

  if (espNowQueue_ != nullptr && espNowTaskHandle_ == nullptr) {
    BaseType_t result = xTaskCreate(EspNowTaskTrampoline, "ESPNowRx", kEspNowTaskStackWords, this,
                                    kEspNowTaskPriority, &espNowTaskHandle_);
    if (result != pdPASS) {
      Serial.println("Failed to create ESP-NOW processing task");
      espNowTaskHandle_ = nullptr;
    }
  }

  if (updateTaskHandle_ == nullptr) {
    BaseType_t result =
        xTaskCreate(UpdateTaskTrampoline, "ControlUpdate", kUpdateTaskStackWords, this, kUpdateTaskPriority,
                    &updateTaskHandle_);
    if (result != pdPASS) {
      Serial.println("Failed to create control update task");
      updateTaskHandle_ = nullptr;
    }
  }

  pendingPairingTone_ = true;
  Serial.println("Pairing tone scheduled");
  updateStatusForPairing();
  failsafeActive_ = false;
  pairingState_ = PairingState{};
  lastControlTimestamp_ = 0;
}

void ControlSystem::loop() { vTaskDelay(kUpdateTaskDelayTicks); }

void ControlSystem::onEspNowData(const uint8_t *mac, const uint8_t *data, int len) {
  bool handled = false;

  Serial.printf("ESP-NOW packet (%d bytes) from %02X:%02X:%02X:%02X:%02X:%02X\n", len, mac[0], mac[1], mac[2], mac[3],
                mac[4], mac[5]);

  if (len == static_cast<int>(sizeof(protocol::IdentityMessage))) {
    const auto &message = *reinterpret_cast<const protocol::IdentityMessage *>(data);
    const protocol::MessageType type = static_cast<protocol::MessageType>(message.type);
    Serial.printf("Identity payload detected: type=%s (%u) senderName=%s\n", MessageTypeName(type),
                  static_cast<unsigned>(message.type), message.identity);
    switch (type) {
    case protocol::MessageType::kScanRequest:
      handleScanRequest(mac);
      handled = true;
      break;
    case protocol::MessageType::kControllerIdentity:
      handleControllerIdentity(mac, message);
      handled = true;
      break;
    case protocol::MessageType::kDroneIdentity:
    case protocol::MessageType::kDroneAck:
      handled = true;
      break;
    default:
      break;
    }
  }

  if (!handled && len >= static_cast<int>(sizeof(protocol::GillControlPacket))) {
    const auto &packet = *reinterpret_cast<const protocol::GillControlPacket *>(data);
    if (packet.magic == protocol::kGillPacketMagic) {
      Serial.println("Gill control packet received");
      handleGillControlPacket(mac, packet);
      handled = true;
    }
  }

  if (!handled && len >= static_cast<int>(sizeof(protocol::ControlMessage))) {
    const auto &packet = *reinterpret_cast<const protocol::ControlMessage *>(data);
    if (packet.type == static_cast<uint8_t>(protocol::MessageType::kControlCommand)) {
      Serial.println("ILITE control packet received");
      handleControlPacket(mac, packet);
    }
  }
}

void ControlSystem::EspNowReceiveTrampoline(const uint8_t *mac, const uint8_t *data, int len) {
  if (instance_ != nullptr) {
    instance_->enqueueEspNowPacketFromIsr(mac, data, len);
  }
}

void ControlSystem::EspNowTaskTrampoline(void *param) {
  auto *self = static_cast<ControlSystem *>(param);
  if (self != nullptr) {
    self->espNowTask();
  }
  vTaskDelete(nullptr);
}

void ControlSystem::UpdateTaskTrampoline(void *param) {
  auto *self = static_cast<ControlSystem *>(param);
  if (self != nullptr) {
    self->updateTask();
  }
  vTaskDelete(nullptr);
}

void ControlSystem::enqueueEspNowPacketFromIsr(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < 0) {
    return;
  }

  if (espNowQueue_ == nullptr) {
    droppedPacketCount_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  EspNowPacket packet{};
  memcpy(packet.mac, mac, sizeof(packet.mac));
  uint16_t copyLength = static_cast<uint16_t>(len);
  if (copyLength > ESP_NOW_MAX_DATA_LEN) {
    copyLength = ESP_NOW_MAX_DATA_LEN;
    droppedPacketCount_.fetch_add(1, std::memory_order_relaxed);
  }
  if (copyLength > 0 && data != nullptr) {
    memcpy(packet.payload, data, copyLength);
  }
  packet.length = copyLength;

  BaseType_t higherPriorityTaskWoken = pdFALSE;
  BaseType_t result = xQueueSendFromISR(espNowQueue_, &packet, &higherPriorityTaskWoken);
  if (result != pdTRUE) {
    droppedPacketCount_.fetch_add(1, std::memory_order_relaxed);
  }
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void ControlSystem::espNowTask() {
  EspNowPacket packet{};
  while (true) {
    if (espNowQueue_ == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (xQueueReceive(espNowQueue_, &packet, portMAX_DELAY) == pdTRUE) {
      onEspNowData(packet.mac, packet.payload, static_cast<int>(packet.length));
    }
  }
}

void ControlSystem::updateTask() {
  while (true) {
    buzzer_.update();
    statusLed_.update();

    if (pendingPairingTone_ && !buzzer_.isPlaying()) {
      buzzer_.playSequence(kPairingSequence, ToneSequenceLength(kPairingSequence));
      pendingPairingTone_ = false;
    }

    HandleOtaLoop();

    const uint32_t now = millis();
    if (pairingState_.paired && (now - lastControlTimestamp_ > config::kControlTimeoutMs)) {
      enterFailsafe();
    }

    const uint32_t dropped = droppedPacketCount_.exchange(0, std::memory_order_relaxed);
    if (dropped != 0) {
      Serial.printf("Dropped %lu ESP-NOW packets\n", static_cast<unsigned long>(dropped));
    }

    vTaskDelay(kUpdateTaskDelayTicks);
  }
}

void ControlSystem::handleScanRequest(const uint8_t *mac) {
  const uint32_t now = millis();
  if (now - lastHandshakeTimestamp_ < kHandshakeCooldownMs) {
    Serial.printf(
        "Ignoring scan request from %02X:%02X:%02X:%02X:%02X:%02X due to cooldown (%lu ms since last handshake)\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        static_cast<unsigned long>(now - lastHandshakeTimestamp_));
    return;
  }

  Serial.printf("Discovery scan from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4],
                mac[5]);
  if (sendIdentityMessage(mac, protocol::MessageType::kDroneIdentity)) {
    Serial.println("Drone identity response sent");
    lastHandshakeTimestamp_ = now;
  }
}

void ControlSystem::handleControllerIdentity(const uint8_t *mac,
                                             const protocol::IdentityMessage &message) {
  const uint32_t now = millis();

  bool payloadMacValid = false;
  for (std::size_t i = 0; i < sizeof(message.mac); ++i) {
    if (message.mac[i] != 0) {
      payloadMacValid = true;
      break;
    }
  }


  const bool addressedToUs = payloadMacValid && memcmp(message.mac, selfMac_, sizeof(selfMac_)) == 0;
  Serial.printf(
      "Controller identity from %02X:%02X:%02X:%02X:%02X:%02X name=%s payloadMac=%02X:%02X:%02X:%02X:%02X:%02X addressedToUs=%s\n",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], message.identity, message.mac[0], message.mac[1],
      message.mac[2], message.mac[3], message.mac[4], message.mac[5], addressedToUs ? "yes" : "no");
  if (payloadMacValid && !addressedToUs) {
    Serial.println("Controller identity payload MAC does not match this drone");
  }

  const bool sameController = pairingState_.paired && memcmp(pairingState_.controllerMac, mac, 6) == 0;


  if (!sameController) {
    memcpy(pairingState_.controllerMac, mac, 6);
    strlcpy(pairingState_.controllerName, message.identity, sizeof(pairingState_.controllerName));
    pairingState_.paired = true;
    lastControlTimestamp_ = now;
    pendingPairingTone_ = false;
    exitFailsafe();
    stopAllMotors();
    ensurePeer(mac);
    Serial.printf("Paired with controller %s (%02X:%02X:%02X:%02X:%02X:%02X)\n", pairingState_.controllerName, mac[0],
                  mac[1], mac[2], mac[3], mac[4], mac[5]);

    buzzer_.playSequence(kConnectedSequence, ToneSequenceLength(kConnectedSequence));
    updateStatusForConnection();
  } else {
    if (strncmp(pairingState_.controllerName, message.identity, sizeof(pairingState_.controllerName)) != 0) {
      Serial.printf("Controller %02X:%02X:%02X:%02X:%02X:%02X updated name to %s\n", mac[0], mac[1], mac[2], mac[3],
                    mac[4], mac[5], message.identity);
      strlcpy(pairingState_.controllerName, message.identity, sizeof(pairingState_.controllerName));
    }
    lastControlTimestamp_ = now;
    Serial.println("Refreshed pairing timestamp for existing controller");
  }


  if (sendIdentityMessage(mac, protocol::MessageType::kDroneAck)) {
    Serial.println("Sent drone acknowledgement to controller");
    lastHandshakeTimestamp_ = now;
  }
}

void ControlSystem::handleControlPacket(const uint8_t *mac, const protocol::ControlMessage &packet) {
  if (packet.version != protocol::kControlProtocolVersion) {
    Serial.printf("Ignoring control packet with incompatible version: %u\n", packet.version);
    return;
  }

  if (!pairingState_.paired || memcmp(mac, pairingState_.controllerMac, 6) != 0) {
    Serial.printf("Ignoring control packet from unpaired controller %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1],
                  mac[2], mac[3], mac[4], mac[5]);
    return;
  }

  Serial.printf("Applying control packet seq=%lu flags=0x%04X\n", static_cast<unsigned long>(packet.sequence),
                packet.flags);
  lastControlTimestamp_ = millis();
  exitFailsafe();

  const std::size_t motorsPerSide = config::kMotorCount / 2;
  float leftSum = 0.0f;
  float rightSum = 0.0f;

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    const float command = static_cast<float>(packet.motorDuty[i]) / 1000.0f;
    const bool brake = (packet.flags & protocol::BrakeFlagForMotor(i)) != 0;
    motors_[i].applyCommand(command, brake);
    lastMotorCommands_[i] = command;

    const float magnitude = brake ? 0.0f : fabsf(command);
    if (i < motorsPerSide) {
      leftSum += magnitude;
    } else {
      rightSum += magnitude;
    }
  }

  const float leftAvg = leftSum / static_cast<float>(motorsPerSide);
  const float rightAvg = rightSum / static_cast<float>(motorsPerSide);
  statusLed_.setDriveBalance(leftAvg, rightAvg);
  updateStatusForConnection();

  if (packet.flags & protocol::kControlFlagHonk) {
    buzzer_.playSequence(kHonkSequence, ToneSequenceLength(kHonkSequence));
  }
}

void ControlSystem::handleGillControlPacket(const uint8_t *mac, const protocol::GillControlPacket &packet) {
  if (packet.magic != protocol::kGillPacketMagic) {
    Serial.println("Ignoring Thegill packet with invalid magic");
    return;
  }

  if (!pairingState_.paired || memcmp(mac, pairingState_.controllerMac, 6) != 0) {
    Serial.printf("Ignoring Thegill packet from unpaired controller %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1],
                  mac[2], mac[3], mac[4], mac[5]);
    return;
  }

  Serial.printf("Applying Gill packet flags=0x%02X mode=%u easing=%u rate=%.3f\n", packet.flags, packet.mode,
                packet.easing, static_cast<double>(packet.easingRate));
  lastControlTimestamp_ = millis();
  exitFailsafe();

  const bool brakeAll = (packet.flags & protocol::kGillFlagBrake) != 0;
  const bool honk = (packet.flags & protocol::kGillFlagHonk) != 0;

  const int16_t motorValues[] = {packet.leftFront, packet.leftRear, packet.rightFront, packet.rightRear};
  constexpr std::size_t kGillMotorCount = sizeof(motorValues) / sizeof(motorValues[0]);

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    float command = 0.0f;
    if (i < kGillMotorCount) {
      command = static_cast<float>(motorValues[i]) / 1000.0f;
    }
    motors_[i].applyCommand(command, brakeAll);
    lastMotorCommands_[i] = brakeAll ? 0.0f : command;
  }

  updateStatusForConnection();

  if (honk) {
    buzzer_.playSequence(kHonkSequence, ToneSequenceLength(kHonkSequence));
  }
}

void ControlSystem::ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    Serial.printf("ESP-NOW peer already exists for %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3],
                  mac[4], mac[5]);
    return;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = config::kEspNowChannel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_AP;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  } else {
    Serial.printf("Added ESP-NOW peer %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4],
                  mac[5]);
  }
}

bool ControlSystem::sendIdentityMessage(const uint8_t *mac, protocol::MessageType type) {
  protocol::IdentityMessage message{};
  message.type = static_cast<uint8_t>(type);
  strlcpy(message.identity, config::kDeviceIdentity, sizeof(message.identity));
  memcpy(message.mac, selfMac_, sizeof(selfMac_));
  ensurePeer(mac);
  if (esp_now_send(mac, reinterpret_cast<uint8_t *>(&message), sizeof(message)) != ESP_OK) {
    Serial.println("Failed to send identity message");
    return false;
  }
  Serial.printf("Sent %s message to %02X:%02X:%02X:%02X:%02X:%02X\n", MessageTypeName(type), mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);
  return true;
}

void ControlSystem::stopAllMotors() {
  Serial.println("Stopping all motors");
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
  Serial.println("Entering failsafe mode");
  stopAllMotors();
  buzzer_.playSequence(kFailsafeSequence, ToneSequenceLength(kFailsafeSequence));
  failsafeActive_ = true;
  statusLed_.setMode(StatusLed::Mode::kFailsafe);
}

void ControlSystem::exitFailsafe() {
  if (!failsafeActive_) {
    return;
  }
  Serial.println("Exiting failsafe mode");
  failsafeActive_ = false;
  buzzer_.stop();
  if (pairingState_.paired) {
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
  Serial.println("Status LED set for pairing mode");
  statusLed_.setMode(StatusLed::Mode::kPairing);
  statusLed_.setDriveBalance(0.0f, 0.0f);
}

void ControlSystem::updateStatusForConnection() {
  Serial.println("Status LED set for connected mode");
  statusLed_.setMode(StatusLed::Mode::kConnected);
  updateDriveIndicator();
}

