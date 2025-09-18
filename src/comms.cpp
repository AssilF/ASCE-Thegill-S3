#include "comms.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <esp_now.h>

#include "device_config.h"

namespace Comms {
namespace {

enum class PairingType : uint8_t {
  kScanRequest = 0x01,
  kDroneIdentity = 0x02,
  kControllerIdentity = 0x03,
  kDroneAck = 0x04,
};

struct IdentityMessage {
  uint8_t type = 0;
  char identity[16] = {};
  uint8_t mac[6] = {};
} __attribute__((packed));

bool g_paired = false;
DriveCommand g_lastCommand{};
uint32_t g_lastTimestamp = 0;
uint8_t g_controllerMac[6] = {0};
char g_controllerIdentity[sizeof(IdentityMessage::identity)] = {0};
uint8_t g_channel = 0;

void resetState() {
  g_paired = false;
  g_lastCommand = DriveCommand{};
  g_lastTimestamp = 0;
  std::memset(g_controllerMac, 0, sizeof(g_controllerMac));
  std::memset(g_controllerIdentity, 0, sizeof(g_controllerIdentity));
}

void ensurePeer(const uint8_t *mac) {
  if (mac == nullptr || esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo{};
  std::memcpy(peerInfo.peer_addr, mac, sizeof(peerInfo.peer_addr));
  peerInfo.channel = g_channel;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}


void sendIdentity(const uint8_t *mac, PairingType type) {
  if (mac == nullptr) {
    return;
  }

  IdentityMessage response{};
  response.type = static_cast<uint8_t>(type);
  std::strncpy(response.identity, config::kDeviceIdentity, sizeof(response.identity));
  response.identity[sizeof(response.identity) - 1] = '\0';
  WiFi.softAPmacAddress(response.mac);

  ensurePeer(mac);
  esp_now_send(mac, reinterpret_cast<const uint8_t *>(&response), sizeof(response));
}

void handleScanRequest(const uint8_t *mac) {
  Serial.println("Pair request received; advertising THEGILL");
  sendIdentity(mac, PairingType::kDroneIdentity);
}

void handleControllerIdentity(const uint8_t *mac, const IdentityMessage &message) {
  if (mac == nullptr) {
    return;
  }

  ensurePeer(mac);


  std::memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
  std::memcpy(g_controllerIdentity, message.identity, sizeof(message.identity));
  g_controllerIdentity[sizeof(g_controllerIdentity) - 1] = '\0';

  IdentityMessage ack{};
  ack.type = static_cast<uint8_t>(PairingType::kDroneAck);
  std::strncpy(ack.identity, config::kDeviceIdentity, sizeof(ack.identity));
  ack.identity[sizeof(ack.identity) - 1] = '\0';
  WiFi.softAPmacAddress(ack.mac);

  esp_now_send(mac, reinterpret_cast<const uint8_t *>(&ack), sizeof(ack));

  g_lastCommand = DriveCommand{};
  g_lastTimestamp = 0;
  g_paired = true;
  Serial.printf("Paired with controller %s\n", g_controllerIdentity);
}


void handleIdentityMessage(const uint8_t *mac, const IdentityMessage &message) {
  const auto type = static_cast<PairingType>(message.type);
  switch (type) {
  case PairingType::kScanRequest:
    handleScanRequest(mac);
    break;
  case PairingType::kControllerIdentity:
    handleControllerIdentity(mac, message);
    break;
  default:
    break;
  }
}

void storeDriveCommand(const uint8_t *mac, const DriveCommand &command) {
  if (!g_paired || mac == nullptr || std::memcmp(mac, g_controllerMac, sizeof(g_controllerMac)) != 0) {

    return;
  }

  if (command.magic != kDriveCommandMagic) {
    return;
  }

  g_lastCommand = command;
  g_lastTimestamp = millis();
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }


  if (len == static_cast<int>(sizeof(IdentityMessage))) {
    handleIdentityMessage(mac, *reinterpret_cast<const IdentityMessage *>(incomingData));

    return;
  }

  if (len == static_cast<int>(sizeof(DriveCommand))) {
    storeDriveCommand(mac, *reinterpret_cast<const DriveCommand *>(incomingData));
  }
}

} // namespace

const uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

bool init(const char *ssid, const char *password, uint8_t channel) {
  g_channel = channel;

  resetState();


  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, channel);

  if (esp_now_init() != ESP_OK) {

    Serial.println("Failed to initialize ESP-NOW");

    return false;
  }

  if (!esp_now_is_peer_exist(kBroadcastMac)) {
    esp_now_peer_info_t peerInfo{};
    std::memcpy(peerInfo.peer_addr, kBroadcastMac, sizeof(peerInfo.peer_addr));
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_AP;
    esp_now_add_peer(&peerInfo);
  }

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

