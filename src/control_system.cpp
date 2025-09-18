#include "control_system.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <string.h>

#include "network_setup.h"

namespace {
constexpr ToneStep kBootSequence[] = {
    {784, 200, 30},  // G5
    {988, 160, 30},  // B5
    {1568, 180, 0},  // G6
};

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
} // namespace

ControlSystem *ControlSystem::instance_ = nullptr;

void ControlSystem::begin() {
  instance_ = this;

  statusLed_.begin(config::kStatusLedPin);
  statusLed_.setMode(StatusLed::Mode::kBoot);

  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    motors_[i].begin(config::kMotorPins[i]);
    lastMotorCommands_[i] = 0.0f;
  }

  buzzer_.begin(config::kBuzzerPin, config::kBuzzerChannel, config::kBuzzerResolutionBits);
  buzzer_.playSequence(kBootSequence, ToneSequenceLength(kBootSequence));

  ConfigureWiFi(config::kDeviceIdentity, config::kAccessPointSsid, config::kAccessPointPassword,
                config::kEspNowChannel);
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

  pendingPairingTone_ = true;
  updateStatusForPairing();
  failsafeActive_ = false;
  pairingState_ = PairingState{};
  lastControlTimestamp_ = 0;
}

void ControlSystem::loop() {
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
}

void ControlSystem::onEspNowData(const uint8_t *mac, const uint8_t *data, int len) {
  bool handled = false;

  if (len == static_cast<int>(sizeof(protocol::IdentityMessage))) {
    const auto &message = *reinterpret_cast<const protocol::IdentityMessage *>(data);
    const protocol::MessageType type = static_cast<protocol::MessageType>(message.type);
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

  if (!handled && len >= static_cast<int>(sizeof(protocol::ControlMessage))) {
    const auto &packet = *reinterpret_cast<const protocol::ControlMessage *>(data);
    if (packet.type == static_cast<uint8_t>(protocol::MessageType::kControlCommand)) {
      handleControlPacket(mac, packet);
    }
  }
}

void ControlSystem::EspNowReceiveTrampoline(const uint8_t *mac, const uint8_t *data, int len) {
  if (instance_ != nullptr) {
    instance_->onEspNowData(mac, data, len);
  }
}

void ControlSystem::handleScanRequest(const uint8_t *mac) {
  Serial.printf("Discovery scan from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4],
                mac[5]);
  sendIdentityMessage(mac, protocol::MessageType::kDroneIdentity);
}

void ControlSystem::handleControllerIdentity(const uint8_t *mac,
                                             const protocol::IdentityMessage &message) {
  const bool sameController = pairingState_.paired && memcmp(mac, pairingState_.controllerMac, 6) == 0;

  if (!sameController) {
    memcpy(pairingState_.controllerMac, mac, 6);
    strlcpy(pairingState_.controllerName, message.identity, sizeof(pairingState_.controllerName));
    pairingState_.paired = true;
    lastControlTimestamp_ = millis();
    pendingPairingTone_ = false;
    exitFailsafe();
    stopAllMotors();
    Serial.printf("Paired with controller %s (%02X:%02X:%02X:%02X:%02X:%02X)\n", pairingState_.controllerName,
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    buzzer_.playSequence(kConnectedSequence, ToneSequenceLength(kConnectedSequence));
    updateStatusForConnection();
  } else {
    if (strncmp(pairingState_.controllerName, message.identity, sizeof(pairingState_.controllerName)) != 0) {
      strlcpy(pairingState_.controllerName, message.identity, sizeof(pairingState_.controllerName));
    }
    lastControlTimestamp_ = millis();
  }

  sendIdentityMessage(mac, protocol::MessageType::kDroneAck);
}

void ControlSystem::handleControlPacket(const uint8_t *mac, const protocol::ControlMessage &packet) {
  if (packet.version != protocol::kControlProtocolVersion) {
    Serial.printf("Ignoring control packet with incompatible version: %u\n", packet.version);
    return;
  }

  if (!pairingState_.paired || memcmp(mac, pairingState_.controllerMac, 6) != 0) {
    Serial.println("Ignoring control packet from unpaired controller");
    return;
  }

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

void ControlSystem::ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  }
}

void ControlSystem::sendIdentityMessage(const uint8_t *mac, protocol::MessageType type) {
  protocol::IdentityMessage message{};
  message.type = static_cast<uint8_t>(type);
  strlcpy(message.identity, config::kDeviceIdentity, sizeof(message.identity));
  WiFi.macAddress(message.mac);
  ensurePeer(mac);
  if (esp_now_send(mac, reinterpret_cast<uint8_t *>(&message), sizeof(message)) != ESP_OK) {
    Serial.println("Failed to send identity message");
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
  statusLed_.setMode(StatusLed::Mode::kPairing);
  statusLed_.setDriveBalance(0.0f, 0.0f);
}

void ControlSystem::updateStatusForConnection() {
  statusLed_.setMode(StatusLed::Mode::kConnected);
  updateDriveIndicator();
}

