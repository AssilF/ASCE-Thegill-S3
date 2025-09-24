#include "comms.h"

#include <WiFi.h>
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace Comms {
namespace {

constexpr size_t kMaxDiscoveryEntries = 8;
constexpr char kDefaultDeviceType[] = "THEGILL";
constexpr char kDefaultPlatform[] = "ESP32";

DeviceRole g_role = DEVICE_ROLE;
TargetSelector g_targetSelector = nullptr;
bool g_initialised = false;

char g_platformName[sizeof(Identity::platform)] = {0};
char g_customId[sizeof(Identity::customId)] = {0};
char g_deviceTypeOverride[sizeof(Identity::deviceType)] = {0};

DiscoveryInfo g_discovery[kMaxDiscoveryEntries];
size_t g_discoveryCount = 0;

LinkStatus g_linkStatus{};
ControlPacket g_lastCommand{};
bool g_commandValid = false;
uint32_t g_lastCommandTimestamp = 0;

uint32_t g_lastBroadcastMs = 0;
uint32_t g_lastCleanupMs = 0;

struct PendingConfirm {
    bool active = false;
    Identity identity{};
    uint8_t mac[6] = {0};
    uint32_t lastSentMs = 0;
} g_pendingConfirm;

esp_now_recv_cb_t g_userCallback = nullptr;

portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_commandMux = portMUX_INITIALIZER_UNLOCKED;

void copyString(char *dest, size_t destSize, const char *src) {
    if (!dest || destSize == 0) {
        return;
    }
    if (!src) {
        dest[0] = '\0';
        return;
    }
    std::strncpy(dest, src, destSize - 1);
    dest[destSize - 1] = '\0';
}

bool isZeroMac(const uint8_t mac[6]) {
    for (size_t i = 0; i < 6; ++i) {
        if (mac[i] != 0) {
            return false;
        }
    }
    return true;
}

int findDiscoveryIndex(const uint8_t mac[6]) {
    for (size_t i = 0; i < g_discoveryCount; ++i) {
        if (macEqual(g_discovery[i].identity.mac, mac)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void removeDiscoveryIndex(size_t index) {
    if (index >= g_discoveryCount) {
        return;
    }
    for (size_t i = index; i + 1 < g_discoveryCount; ++i) {
        g_discovery[i] = g_discovery[i + 1];
    }
    if (g_discoveryCount > 0) {
        --g_discoveryCount;
    }
}

void upsertDiscovery(const Identity &identity, uint32_t nowMs) {
    int existing = findDiscoveryIndex(identity.mac);
    if (existing >= 0) {
        g_discovery[existing].identity = identity;
        g_discovery[existing].lastSeenMs = nowMs;
        return;
    }

    if (g_discoveryCount < kMaxDiscoveryEntries) {
        g_discovery[g_discoveryCount].identity = identity;
        g_discovery[g_discoveryCount].lastSeenMs = nowMs;
        ++g_discoveryCount;
        return;
    }

    size_t oldestIndex = 0;
    uint32_t oldestTime = g_discovery[0].lastSeenMs;
    for (size_t i = 1; i < g_discoveryCount; ++i) {
        if (g_discovery[i].lastSeenMs < oldestTime) {
            oldestIndex = i;
            oldestTime = g_discovery[i].lastSeenMs;
        }
    }

    g_discovery[oldestIndex].identity = identity;
    g_discovery[oldestIndex].lastSeenMs = nowMs;
}

void clearCommandState() {
    portENTER_CRITICAL(&g_commandMux);
    g_lastCommand = ControlPacket{};
    g_lastCommandTimestamp = 0;
    g_commandValid = false;
    portEXIT_CRITICAL(&g_commandMux);
}

void clearLinkState() {
    portENTER_CRITICAL(&g_stateMux);
    g_linkStatus = LinkStatus{};
    portEXIT_CRITICAL(&g_stateMux);
    g_pendingConfirm = PendingConfirm{};
    clearCommandState();
}

void setLastCommandMs(uint32_t nowMs) {
    portENTER_CRITICAL(&g_stateMux);
    g_linkStatus.lastCommandMs = nowMs;
    g_linkStatus.lastActivityMs = nowMs;
    portEXIT_CRITICAL(&g_stateMux);
}

void setPaired(const Identity &identity, const uint8_t mac[6], uint32_t nowMs) {
    portENTER_CRITICAL(&g_stateMux);
    g_linkStatus.paired = true;
    g_linkStatus.peerIdentity = identity;
    std::memcpy(g_linkStatus.peerMac, mac, sizeof(g_linkStatus.peerMac));
    g_linkStatus.lastActivityMs = nowMs;
    portEXIT_CRITICAL(&g_stateMux);
    g_pendingConfirm = PendingConfirm{};
}

void sendPacket(const uint8_t mac[6], MessageType type, uint32_t nowMs) {
    Packet packet{};
    packet.version = PROTOCOL_VERSION;
    packet.type = type;
    fillSelfIdentity(packet.id);
    packet.monotonicMs = nowMs;
    if (!macEqual(mac, BroadcastMac)) {
        ensurePeer(mac);
    }
    esp_now_send(mac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

void handlePairRequest(const uint8_t *mac, const Packet &, uint32_t nowMs) {
    if (g_role != DeviceRole::Controlled) {
        return;
    }
    sendPacket(mac, MessageType::MSG_IDENTITY_REPLY, nowMs);
}

void handleIdentityReply(const uint8_t *mac, Packet packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controller) {
        return;
    }
    std::memcpy(packet.id.mac, mac, sizeof(packet.id.mac));
    upsertDiscovery(packet.id, nowMs);
}

void handlePairConfirm(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controlled) {
        return;
    }
    ensurePeer(mac);
    setPaired(packet.id, mac, nowMs);
    sendPacket(mac, MessageType::MSG_PAIR_ACK, nowMs);
}

void handlePairAck(const uint8_t *mac, const Packet &packet, uint32_t nowMs) {
    if (g_role != DeviceRole::Controller) {
        return;
    }
    ensurePeer(mac);
    if (g_pendingConfirm.active && macEqual(mac, g_pendingConfirm.mac)) {
        g_pendingConfirm = PendingConfirm{};
    }
    setPaired(packet.id, mac, nowMs);
}

void handleControlPacket(const uint8_t *mac, const ControlPacket &packet, uint32_t nowMs) {
    bool accept = false;
    portENTER_CRITICAL(&g_stateMux);
    if (g_linkStatus.paired && macEqual(g_linkStatus.peerMac, mac)) {
        accept = true;
    }
    portEXIT_CRITICAL(&g_stateMux);

    if (!accept) {
        return;
    }

    portENTER_CRITICAL(&g_commandMux);
    g_lastCommand = packet;
    g_lastCommandTimestamp = nowMs;
    g_commandValid = true;
    portEXIT_CRITICAL(&g_commandMux);

    setLastCommandMs(nowMs);
}

void IRAM_ATTR onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    uint32_t nowMs = millis();

    if (len == static_cast<int>(sizeof(Packet))) {
        Packet packet;
        std::memcpy(&packet, incomingData, sizeof(packet));
        if (packet.version == PROTOCOL_VERSION) {
            switch (packet.type) {
                case MessageType::MSG_PAIR_REQ:
                    handlePairRequest(mac, packet, nowMs);
                    break;
                case MessageType::MSG_IDENTITY_REPLY:
                    handleIdentityReply(mac, packet, nowMs);
                    break;
                case MessageType::MSG_PAIR_CONFIRM:
                    handlePairConfirm(mac, packet, nowMs);
                    break;
                case MessageType::MSG_PAIR_ACK:
                    handlePairAck(mac, packet, nowMs);
                    break;
                default:
                    break;
            }
        }
    } else if (len == static_cast<int>(sizeof(ControlPacket))) {
        ControlPacket packet;
        std::memcpy(&packet, incomingData, sizeof(packet));
        handleControlPacket(mac, packet, nowMs);
    }

    if (g_userCallback) {
        g_userCallback(mac, incomingData, len);
    }
}

void broadcastIdentity(uint32_t nowMs) {
    Packet packet{};
    packet.version = PROTOCOL_VERSION;
    packet.type = (g_role == DeviceRole::Controller) ? MessageType::MSG_PAIR_REQ
                                                     : MessageType::MSG_IDENTITY_REPLY;
    fillSelfIdentity(packet.id);
    packet.monotonicMs = nowMs;
    esp_now_send(BroadcastMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

void trySendPairConfirm(uint32_t nowMs) {
    if (g_role != DeviceRole::Controller || paired() || g_discoveryCount == 0) {
        return;
    }

    if (g_pendingConfirm.active) {
        int index = findDiscoveryIndex(g_pendingConfirm.mac);
        if (index < 0) {
            g_pendingConfirm = PendingConfirm{};
            return;
        }
        if (nowMs - g_pendingConfirm.lastSentMs < BROADCAST_INTERVAL_MS) {
            return;
        }
        sendPacket(g_pendingConfirm.mac, MessageType::MSG_PAIR_CONFIRM, nowMs);
        g_pendingConfirm.lastSentMs = nowMs;
        return;
    }

    int selectedIndex = 0;
    if (g_targetSelector) {
        selectedIndex = g_targetSelector(g_discovery, g_discoveryCount);
    }
    if (selectedIndex < 0 || static_cast<size_t>(selectedIndex) >= g_discoveryCount) {
        selectedIndex = 0;
    }

    Identity identity = g_discovery[selectedIndex].identity;
    sendPacket(identity.mac, MessageType::MSG_PAIR_CONFIRM, nowMs);
    g_pendingConfirm.active = true;
    g_pendingConfirm.identity = identity;
    std::memcpy(g_pendingConfirm.mac, identity.mac, sizeof(g_pendingConfirm.mac));
    g_pendingConfirm.lastSentMs = nowMs;
}

void pruneDiscovery(uint32_t nowMs) {
    for (size_t i = 0; i < g_discoveryCount;) {
        if (nowMs - g_discovery[i].lastSeenMs > DEVICE_TTL_MS) {
            removeDiscoveryIndex(i);
        } else {
            ++i;
        }
    }
}

} // namespace

const uint8_t BroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void setRole(DeviceRole role) {
    g_role = role;
    g_discoveryCount = 0;
    clearLinkState();
}

DeviceRole getRole() {
    return g_role;
}

void setPlatform(const char *platformName) {
    copyString(g_platformName, sizeof(g_platformName), platformName);
}

void setCustomId(const char *customId) {
    copyString(g_customId, sizeof(g_customId), customId);
}

void setDeviceTypeOverride(const char *deviceTypeName) {
    copyString(g_deviceTypeOverride, sizeof(g_deviceTypeOverride), deviceTypeName);
}

void setTargetSelector(TargetSelector selector) {
    g_targetSelector = selector;
}

void fillSelfIdentity(Identity &outIdentity) {
    std::memset(&outIdentity, 0, sizeof(outIdentity));
    const char *deviceType = g_deviceTypeOverride[0] ? g_deviceTypeOverride : kDefaultDeviceType;
    const char *platform = g_platformName[0] ? g_platformName : kDefaultPlatform;
    copyString(outIdentity.deviceType, sizeof(outIdentity.deviceType), deviceType);
    copyString(outIdentity.platform, sizeof(outIdentity.platform), platform);
    copyString(outIdentity.customId, sizeof(outIdentity.customId), g_customId);
    WiFi.macAddress(outIdentity.mac);
}

String macToString(const uint8_t mac[6]) {
    char buffer[18];
    std::snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buffer);
}

bool macEqual(const uint8_t lhs[6], const uint8_t rhs[6]) {
    return std::memcmp(lhs, rhs, 6) == 0;
}

bool ensurePeer(const uint8_t mac[6]) {
    if (!mac || isZeroMac(mac)) {
        return false;
    }
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t info{};
    std::memcpy(info.peer_addr, mac, 6);
    info.channel = WIFI_CHANNEL;
    info.encrypt = false;
    return esp_now_add_peer(&info) == ESP_OK;
}

bool init(const char *ssid, const char *password, int tcpPort) {
    return init(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    (void)tcpPort;
    g_userCallback = recvCallback;
    g_discoveryCount = 0;
    g_lastBroadcastMs = 0;
    g_lastCleanupMs = 0;
    clearLinkState();

    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_18_5dBm);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t info{};
    std::memcpy(info.peer_addr, BroadcastMac, sizeof(info.peer_addr));
    info.channel = WIFI_CHANNEL;
    info.encrypt = false;
    if (!esp_now_is_peer_exist(BroadcastMac)) {
        if (esp_now_add_peer(&info) != ESP_OK) {
            return false;
        }
    }

    esp_now_register_recv_cb(onDataRecv);
    g_initialised = true;
    return true;
}

void loop() {
    if (!g_initialised) {
        return;
    }

    uint32_t nowMs = millis();

    if (nowMs - g_lastBroadcastMs >= BROADCAST_INTERVAL_MS) {
        broadcastIdentity(nowMs);
        g_lastBroadcastMs = nowMs;
    }

    if (nowMs - g_lastCleanupMs >= BROADCAST_INTERVAL_MS) {
        pruneDiscovery(nowMs);
        g_lastCleanupMs = nowMs;
    }

    trySendPairConfirm(nowMs);

    LinkStatus status = getLinkStatus();
    if (status.paired && status.lastActivityMs > 0 && nowMs - status.lastActivityMs > LINK_TIMEOUT_MS) {
        if (!isZeroMac(status.peerMac) && esp_now_is_peer_exist(status.peerMac)) {
            esp_now_del_peer(status.peerMac);
        }
        clearLinkState();
    }
}

bool receiveCommand(ControlPacket &cmd, uint32_t *timestampMs) {
    bool hasCommand = false;
    portENTER_CRITICAL(&g_commandMux);
    cmd = g_lastCommand;
    if (timestampMs) {
        *timestampMs = g_lastCommandTimestamp;
    }
    hasCommand = g_commandValid;
    g_commandValid = false;
    portEXIT_CRITICAL(&g_commandMux);
    return hasCommand;
}

uint32_t lastCommandTimestamp() {
    uint32_t timestamp = 0;
    portENTER_CRITICAL(&g_commandMux);
    timestamp = g_lastCommandTimestamp;
    portEXIT_CRITICAL(&g_commandMux);
    return timestamp;
}

LinkStatus getLinkStatus() {
    LinkStatus status;
    portENTER_CRITICAL(&g_stateMux);
    status = g_linkStatus;
    portEXIT_CRITICAL(&g_stateMux);
    return status;
}

bool paired() {
    portENTER_CRITICAL(&g_stateMux);
    bool value = g_linkStatus.paired;
    portEXIT_CRITICAL(&g_stateMux);
    return value;
}

} // namespace Comms
