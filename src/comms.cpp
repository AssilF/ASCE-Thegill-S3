#include "comms.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include <cstring>

#include "device_config.h"
#include "control_protocol.h"

namespace Comms {
namespace {
volatile uint32_t g_lastCommandTimestamp = 0;
DriveCommand g_lastCommand{};
bool g_paired = false;
uint8_t g_controllerMac[6] = {0};
char g_controllerIdentity[sizeof(IdentityMessage::identity)] = {0};
uint8_t g_channel = 0;
uint16_t g_lastBrakeMask = 0;
bool g_lastFromIlite = false;
portMUX_TYPE g_commandMutex = portMUX_INITIALIZER_UNLOCKED;

void ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = g_channel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_AP;
  esp_now_add_peer(&peerInfo);
}

void addBroadcastPeer() {
  static const uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  ensurePeer(kBroadcastMac);
}

void respondWithIdentity(const uint8_t *mac, PairingType type) {
  IdentityMessage response{};
  response.type = static_cast<uint8_t>(type);
  std::strncpy(response.identity, config::kDeviceIdentity, sizeof(response.identity));
  response.identity[sizeof(response.identity) - 1] = '\0';
  WiFi.softAPmacAddress(response.mac);
  ensurePeer(mac);
  esp_now_send(mac, reinterpret_cast<const uint8_t *>(&response), sizeof(response));
}

void handleGillControlPacket(const uint8_t *mac, const protocol::GillControlPacket &packet) {
  if (packet.magic != protocol::kGillPacketMagic) {
    return;
  }

  if (g_paired && std::memcmp(g_controllerMac, mac, sizeof(g_controllerMac)) != 0) {
    return;
  }

  const uint32_t now = millis();
  portENTER_CRITICAL(&g_commandMutex);
  g_lastCommand = {packet.magic,
                   packet.leftFront,
                   packet.leftRear,
                   packet.rightFront,
                   packet.rightRear,
                   packet.easingRate,
                   packet.mode,
                   packet.easing,
                   packet.flags,
                   packet.reserved};
  g_lastBrakeMask = 0;
  g_lastFromIlite = false;
  g_lastCommandTimestamp = now;
  portEXIT_CRITICAL(&g_commandMutex);

  if (!g_paired) {
    std::memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
    g_paired = true;
  }
}

void handleIliteControlPacket(const uint8_t *mac, const protocol::ControlMessage &packet) {
  if (packet.type != static_cast<uint8_t>(protocol::MessageType::kControlCommand) ||
      packet.version != protocol::kControlProtocolVersion) {
    return;
  }

  if (g_paired && std::memcmp(g_controllerMac, mac, sizeof(g_controllerMac)) != 0) {
    return;
  }

  DriveCommand translated{};
  translated.magic = kDrivePacketMagic;
  translated.leftFront = packet.motorDuty[0];
  translated.leftRear = packet.motorDuty[1];
  translated.rightFront = packet.motorDuty[2];
  translated.rightRear = packet.motorDuty[3];
  translated.easingRate = 0.0f;
  translated.mode = 0;
  translated.easing = 0;
  translated.flags = 0;
  if (packet.flags & protocol::kControlFlagHonk) {
    translated.flags |= kDriveFlagHonk;
  }
  translated.reserved = 0;

  uint16_t brakeMask = 0;
  for (std::size_t i = 0; i < config::kMotorCount; ++i) {
    if (packet.flags & protocol::BrakeFlagForMotor(i)) {
      brakeMask |= static_cast<uint16_t>(1U << i);
    }
  }

  const uint32_t now = millis();
  portENTER_CRITICAL(&g_commandMutex);
  g_lastCommand = translated;
  g_lastBrakeMask = brakeMask;
  g_lastFromIlite = true;
  g_lastCommandTimestamp = now;
  portEXIT_CRITICAL(&g_commandMutex);

  if (!g_paired) {
    std::memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
    g_paired = true;
  }
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }

  if (len == static_cast<int>(sizeof(IdentityMessage))) {
    const auto *msg = reinterpret_cast<const IdentityMessage *>(incomingData);
    switch (static_cast<PairingType>(msg->type)) {
    case kScanRequest:
      respondWithIdentity(mac, kDroneIdentity);
      return;
    case kControllerIdentity:
      memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
      std::strncpy(g_controllerIdentity, msg->identity, sizeof(g_controllerIdentity));
      g_controllerIdentity[sizeof(g_controllerIdentity) - 1] = '\0';
      ensurePeer(mac);
      respondWithIdentity(mac, kDroneAck);
      g_paired = true;
      return;
    default:
      break;
    }
  }

  if (len == static_cast<int>(sizeof(protocol::GillControlPacket))) {
    handleGillControlPacket(mac, *reinterpret_cast<const protocol::GillControlPacket *>(incomingData));
    return;
  }

  if (len == static_cast<int>(sizeof(DriveCommand))) {
    const auto *cmd = reinterpret_cast<const DriveCommand *>(incomingData);
    if (cmd->magic != kDrivePacketMagic) {
      return;
    }
    if (g_paired && std::memcmp(g_controllerMac, mac, sizeof(g_controllerMac)) != 0) {
      return;
    }

    const uint32_t now = millis();
    portENTER_CRITICAL(&g_commandMutex);
    g_lastCommand = *cmd;
    g_lastBrakeMask = 0;
    g_lastFromIlite = false;
    g_lastCommandTimestamp = now;
    portEXIT_CRITICAL(&g_commandMutex);

    if (!g_paired) {
      memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
      g_paired = true;
    }
    return;
  }

  if (len == static_cast<int>(sizeof(protocol::ControlMessage))) {
    handleIliteControlPacket(mac, *reinterpret_cast<const protocol::ControlMessage *>(incomingData));
  }
}

} // namespace

bool init(const char *ssid, const char *password, uint8_t channel) {
  g_channel = channel;
  g_paired = false;
  g_lastCommandTimestamp = 0;
  std::memset(&g_lastCommand, 0, sizeof(g_lastCommand));
  std::memset(g_controllerMac, 0, sizeof(g_controllerMac));
  std::memset(g_controllerIdentity, 0, sizeof(g_controllerIdentity));
  g_lastBrakeMask = 0;
  g_lastFromIlite = false;

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(config::kDeviceIdentity);
  WiFi.softAP(ssid, password, channel);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  addBroadcastPeer();
  esp_now_register_recv_cb(onDataRecv);
  return true;
}

bool paired() { return g_paired; }

bool receiveCommand(Command &command) {
  portENTER_CRITICAL(&g_commandMutex);
  command.drive = g_lastCommand;
  command.brakeMask = g_lastBrakeMask;
  command.fromIlite = g_lastFromIlite;
  const bool valid = g_paired && g_lastCommand.magic == kDrivePacketMagic && g_lastCommandTimestamp != 0;
  portEXIT_CRITICAL(&g_commandMutex);
  return valid;
}

uint32_t lastCommandTimestamp() { return g_lastCommandTimestamp; }

const uint8_t *controllerMac() { return g_controllerMac; }

const char *controllerIdentity() { return g_controllerIdentity; }

} // namespace Comms

