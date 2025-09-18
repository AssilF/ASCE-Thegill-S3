#include "comms.h"
#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <esp_now.h>
#include <device_config.h>
#include <control_protocol.h>
#include <motor_driver.h>

namespace Comms {
namespace {
constexpr uint8_t kIdentityStringLength = sizeof(protocol::IdentityMessage::identity);
constexpr uint32_t kHandshakeCooldownMs = 1000; // Cooldown in milliseconds

bool g_paired = false;
DriveCommand g_lastCommand{};
uint32_t g_lastTimestamp = 0;
uint8_t g_controllerMac[6] = {0};
char g_controllerIdentity[kIdentityStringLength] = {0};
uint8_t g_channel = 0;
uint32_t g_lastHandshakeMs = 0;
uint32_t g_syntheticSequence = 0;

void resetCommandState() {
  std::memset(&g_lastCommand, 0, sizeof(g_lastCommand));
  g_lastTimestamp = 0;
  g_syntheticSequence = 0;
}

void resetPairingState() {
  g_paired = false;
  g_lastHandshakeMs = 0;
  std::memset(g_controllerMac, 0, sizeof(g_controllerMac));
  std::memset(g_controllerIdentity, 0, sizeof(g_controllerIdentity));
  resetCommandState();
}

void ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo{};
  std::memcpy(peerInfo.peer_addr, mac, sizeof(peerInfo.peer_addr));
  peerInfo.channel = g_channel;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void sendIdentity(const uint8_t *mac, protocol::MessageType type) {
  protocol::IdentityMessage response{};
  response.type = static_cast<uint8_t>(type);
  std::strncpy(response.identity, config::kDeviceIdentity, sizeof(response.identity));
  response.identity[sizeof(response.identity) - 1] = '\0';
  WiFi.macAddress(response.mac);
  ensurePeer(mac);
  esp_now_send(mac, reinterpret_cast<const uint8_t *>(&response), sizeof(response));
  g_lastHandshakeMs = millis();
}

void handleScanRequest(const uint8_t *mac) {
  const uint32_t now = millis();
  if (g_lastHandshakeMs != 0 && (now - g_lastHandshakeMs) < kHandshakeCooldownMs) {
    return;
  }

  Serial.println("Pairing request detected");
  sendIdentity(mac, protocol::MessageType::kDroneIdentity);
}

void handleControllerIdentity(const uint8_t *mac, const protocol::IdentityMessage &message) {
  std::memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
  std::strncpy(g_controllerIdentity, message.identity, sizeof(g_controllerIdentity));
  g_controllerIdentity[sizeof(g_controllerIdentity) - 1] = '\0';
  ensurePeer(mac);
  sendIdentity(mac, protocol::MessageType::kDroneAck);
  resetCommandState();
  g_paired = true;
  Serial.println("Controller paired");
}

void handleIdentityMessage(const uint8_t *mac, const protocol::IdentityMessage &message) {
  switch (static_cast<protocol::MessageType>(message.type)) {
  case protocol::MessageType::kScanRequest:
    handleScanRequest(mac);
    break;
  case protocol::MessageType::kControllerIdentity:
    handleControllerIdentity(mac, message);
    break;
  default:
    break;
  }
}

void storeDriveCommand(const DriveCommand &command) {
  g_lastCommand = command;
  g_lastTimestamp = millis();
}

DriveCommand convertControlMessage(const protocol::ControlMessage &message) {
  DriveCommand cmd{};
  cmd.sequence = message.sequence;
  cmd.version = message.version;
  std::memcpy(cmd.motorDuty, message.motorDuty, sizeof(cmd.motorDuty));
  cmd.flags = message.flags;
  return cmd;
}

DriveCommand convertGillPacket(const protocol::GillControlPacket &packet) {
  DriveCommand cmd{};
  cmd.version = protocol::kControlProtocolVersion;
  cmd.sequence = ++g_syntheticSequence;
  cmd.flags = 0;

  const bool brake = (packet.flags & protocol::kGillFlagBrake) != 0;
  if (brake) {
    for (std::size_t i = 0; i < config::kMotorCount; ++i) {
      cmd.motorDuty[i] = 0;
      cmd.flags |= protocol::BrakeFlagForMotor(i);
    }
  } else {
    const int16_t motors[] = {packet.leftFront, packet.leftRear, packet.rightFront, packet.rightRear};
    constexpr std::size_t kGillMotorCount = sizeof(motors) / sizeof(motors[0]);
    for (std::size_t i = 0; i < config::kMotorCount; ++i) {
      cmd.motorDuty[i] = (i < kGillMotorCount) ? motors[i] : 0;
    }
  }

  if (packet.flags & protocol::kGillFlagHonk) {
    cmd.flags |= protocol::kControlFlagHonk;
  }

  return cmd;
}

void handleControlMessage(const uint8_t *mac, const protocol::ControlMessage &message) {
  if (message.type != static_cast<uint8_t>(protocol::MessageType::kControlCommand)) {
    return;
  }
  if (message.version != protocol::kControlProtocolVersion) {
    return;
  }
  if (!g_paired || std::memcmp(mac, g_controllerMac, sizeof(g_controllerMac)) != 0) {
    return;
  }

  storeDriveCommand(convertControlMessage(message));
}

void handleGillPacket(const uint8_t *mac, const protocol::GillControlPacket &packet) {
  if (packet.magic != protocol::kGillPacketMagic) {
    return;
  }
  if (!g_paired || std::memcmp(mac, g_controllerMac, sizeof(g_controllerMac)) != 0) {
    return;
  }

  storeDriveCommand(convertGillPacket(packet));
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }

  if (len == static_cast<int>(sizeof(protocol::IdentityMessage))) {
    handleIdentityMessage(mac, *reinterpret_cast<const protocol::IdentityMessage *>(incomingData));
    return;
  }

  if (!g_paired) {
    return;
  }

  if (len == static_cast<int>(sizeof(protocol::GillControlPacket))) {
    handleGillPacket(mac, *reinterpret_cast<const protocol::GillControlPacket *>(incomingData));
    return;
  }

  if (len == static_cast<int>(sizeof(protocol::ControlMessage))) {
    handleControlMessage(mac, *reinterpret_cast<const protocol::ControlMessage *>(incomingData));
  }
}

} // namespace

const uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

bool init(const char *ssid, const char *password, uint8_t channel) {
  g_channel = channel;
  resetPairingState();

  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, channel);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP now failed to init..");
    return false;
  }

  ensurePeer(kBroadcastMac);
  esp_now_register_recv_cb(onDataRecv);
  return true;
}

bool receiveCommand(DriveCommand &cmd) {
  if (!g_paired || g_lastTimestamp == 0) {
    return false;
  }

  cmd = g_lastCommand;
  return true;
}

bool paired() { return g_paired; }

uint32_t lastCommandTimestamp() { return g_lastTimestamp; }

const uint8_t *controllerMac() { return g_controllerMac; }

const char *controllerIdentity() { return g_controllerIdentity; }

} // namespace Comms

