#include "comms.h"

#include <cstring>

namespace Comms {
namespace {
constexpr uint8_t kIdentityStringLength = sizeof(protocol::IdentityMessage::identity);

bool g_paired = false;
protocol::ControlMessage g_lastMessage{};
uint32_t g_lastTimestamp = 0;
uint8_t g_controllerMac[6] = {0};
char g_controllerIdentity[kIdentityStringLength] = {0};
uint8_t g_channel = 0;

void ensurePeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo{};
  std::memcpy(peerInfo.peer_addr, mac, sizeof(peerInfo.peer_addr));
  peerInfo.channel = g_channel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_AP;
  esp_now_add_peer(&peerInfo);
}

void sendIdentity(const uint8_t *mac, protocol::MessageType type) {
  protocol::IdentityMessage response{};
  response.type = static_cast<uint8_t>(type);
  std::strncpy(response.identity, config::kDeviceIdentity, sizeof(response.identity));
  response.identity[sizeof(response.identity) - 1] = '\0';
  WiFi.softAPmacAddress(response.mac);
  ensurePeer(mac);
  esp_now_send(mac, reinterpret_cast<const uint8_t *>(&response), sizeof(response));
}

void handleIdentityMessage(const uint8_t *mac, const protocol::IdentityMessage &message) {
  const auto msgType = static_cast<protocol::MessageType>(message.type);
  switch (msgType) {
  case protocol::MessageType::kScanRequest:
    sendIdentity(mac, protocol::MessageType::kDroneIdentity);
    break;
  case protocol::MessageType::kControllerIdentity:
    std::memcpy(g_controllerMac, mac, sizeof(g_controllerMac));
    std::strncpy(g_controllerIdentity, message.identity, sizeof(g_controllerIdentity));
    g_controllerIdentity[sizeof(g_controllerIdentity) - 1] = '\0';
    ensurePeer(mac);
    sendIdentity(mac, protocol::MessageType::kDroneAck);
    g_paired = true;
    break;
  default:
    break;
  }
}

void handleControlMessage(const uint8_t *mac, const protocol::ControlMessage &message) {
  (void)mac;
  if (message.type != static_cast<uint8_t>(protocol::MessageType::kControlCommand)) {
    return;
  }
  if (message.version != protocol::kControlProtocolVersion) {
    return;
  }

  g_lastMessage = message;
  g_lastTimestamp = millis();
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (mac == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }

  if (len == static_cast<int>(sizeof(protocol::IdentityMessage))) {
    handleIdentityMessage(mac, *reinterpret_cast<const protocol::IdentityMessage *>(incomingData));
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
  g_paired = false;
  g_lastTimestamp = 0;
  std::memset(&g_lastMessage, 0, sizeof(g_lastMessage));
  std::memset(g_controllerMac, 0, sizeof(g_controllerMac));
  std::memset(g_controllerIdentity, 0, sizeof(g_controllerIdentity));

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.softAP(ssid, password, channel);

  if (esp_now_init() != ESP_OK) {
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

  cmd.sequence = g_lastMessage.sequence;
  cmd.version = g_lastMessage.version;
  std::memcpy(cmd.motorDuty, g_lastMessage.motorDuty, sizeof(cmd.motorDuty));
  cmd.flags = g_lastMessage.flags;
  return true;
}

bool paired() { return g_paired; }

uint32_t lastCommandTimestamp() { return g_lastTimestamp; }

const uint8_t *controllerMac() { return g_controllerMac; }

const char *controllerIdentity() { return g_controllerIdentity; }

} // namespace Comms

