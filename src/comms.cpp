#include "comms.h"
#include "thegill.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

namespace Comms {
namespace {

constexpr size_t MAX_DISCOVERY_ENTRIES = 8;

DeviceRole g_role = DEVICE_ROLE;
TargetSelector g_targetSelector = nullptr;

char g_platform[sizeof(Identity::platform)] = "Thegill";
char g_customId[sizeof(Identity::customId)] = "ThegillA14";
char g_deviceTypeOverride[sizeof(Identity::deviceType)] = "controlled";

std::array<DiscoveryInfo, MAX_DISCOVERY_ENTRIES> g_discoveries{};
size_t g_discoveryCount = 0;

LinkStatus g_linkStatus{};

ThegillCommand g_lastThegillCommand{};
bool g_pendingThegillCommand = false;
uint32_t g_lastThegillCommandTimestamp = 0;

ArmControlCommand g_lastArmCommand{};
bool g_pendingArmCommand = false;
uint32_t g_lastArmCommandTimestamp = 0;

PeripheralCommand g_lastPeripheralCommand{};
bool g_pendingPeripheralCommand = false;
uint32_t g_lastPeripheralCommandTimestamp = 0;

ConfigurationPacket g_lastConfigurationPacket{};
bool g_pendingConfigurationPacket = false;
uint32_t g_lastConfigurationPacketTimestamp = 0;

SettingsPacket g_lastSettingsPacket{};
bool g_pendingSettingsPacket = false;
uint32_t g_lastSettingsPacketTimestamp = 0;

uint32_t g_lastPeerHeartbeatTimestamp = 0;

bool g_initialized = false;
uint32_t g_lastBroadcastMs = 0;
uint32_t g_lastDiscoveryCleanupMs = 0;

bool g_waitingForAck = false;
uint8_t g_pendingMac[6] = {0};
Identity g_pendingIdentity{};
uint32_t g_lastPairAttemptMs = 0;

esp_now_recv_cb_t g_userRecvCb = nullptr;

constexpr const char *DEFAULT_CONTROLLER_TYPE = "ILITE";
constexpr const char *DEFAULT_CONTROLLED_TYPE = "Thegill";

bool macEqualBytes(const uint8_t lhs[6], const uint8_t rhs[6]) {
    return std::memcmp(lhs, rhs, 6) == 0;
}

void composeIdentity(Identity &outIdentity) {
    std::memset(&outIdentity, 0, sizeof(outIdentity));

    const char *deviceType = g_deviceTypeOverride[0] != '\0'
                                 ? g_deviceTypeOverride
                                 : (g_role == DeviceRole::Controller ? DEFAULT_CONTROLLER_TYPE
                                                                     : DEFAULT_CONTROLLED_TYPE);

    std::strncpy(outIdentity.deviceType, deviceType, sizeof(outIdentity.deviceType) - 1);
    if (g_platform[0] != '\0') {
        std::strncpy(outIdentity.platform, g_platform, sizeof(outIdentity.platform) - 1);
    }
    if (g_customId[0] != '\0') {
        std::strncpy(outIdentity.customId, g_customId, sizeof(outIdentity.customId) - 1);
    }

    WiFi.macAddress(outIdentity.mac);
}

void sendPacket(const uint8_t *mac, MessageType type) {
    Packet packet{};
    packet.version = PROTOCOL_VERSION;
    packet.type = type;
    composeIdentity(packet.id);
    packet.monotonicMs = millis();
    packet.reserved = 0;
    esp_now_send(mac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
}

void addOrUpdateDiscovery(const Identity &identity, const uint8_t mac[6], uint32_t timestampMs) {
    for (size_t i = 0; i < g_discoveryCount; ++i) {
        if (macEqualBytes(g_discoveries[i].identity.mac, mac)) {
            g_discoveries[i].identity = identity;
            std::memcpy(g_discoveries[i].identity.mac, mac, 6);
            g_discoveries[i].lastSeenMs = timestampMs;
            return;
        }
    }

    DiscoveryInfo entry{};
    entry.identity = identity;
    std::memcpy(entry.identity.mac, mac, 6);
    entry.lastSeenMs = timestampMs;

    if (g_discoveryCount < g_discoveries.size()) {
        g_discoveries[g_discoveryCount++] = entry;
        return;
    }

    auto oldest = std::min_element(g_discoveries.begin(), g_discoveries.end(),
                                   [](const DiscoveryInfo &lhs, const DiscoveryInfo &rhs) {
                                       return lhs.lastSeenMs < rhs.lastSeenMs;
                                   });
    if (oldest != g_discoveries.end()) {
        *oldest = entry;
    }
}

void pruneDiscoveries(uint32_t nowMs) {
    if (nowMs - g_lastDiscoveryCleanupMs < BROADCAST_INTERVAL_MS) {
        return;
    }
    g_lastDiscoveryCleanupMs = nowMs;

    size_t writeIndex = 0;
    for (size_t i = 0; i < g_discoveryCount; ++i) {
        if (nowMs - g_discoveries[i].lastSeenMs <= DEVICE_TTL_MS) {
            if (writeIndex != i) {
                g_discoveries[writeIndex] = g_discoveries[i];
            }
            ++writeIndex;
        }
    }
    g_discoveryCount = writeIndex;
}

void clearLinkStatus() {
    if (g_linkStatus.paired) {
        if (!macEqualBytes(g_linkStatus.peerMac, BroadcastMac) && esp_now_is_peer_exist(g_linkStatus.peerMac)) {
            esp_now_del_peer(g_linkStatus.peerMac);
        }
    }
    g_linkStatus = LinkStatus{};
    g_pendingThegillCommand = false;
    g_pendingArmCommand = false;
    g_pendingPeripheralCommand = false;
    g_pendingConfigurationPacket = false;
    g_pendingSettingsPacket = false;
    g_lastThegillCommandTimestamp = 0;
    g_lastArmCommandTimestamp = 0;
    g_lastPeripheralCommandTimestamp = 0;
    g_lastConfigurationPacketTimestamp = 0;
    g_lastSettingsPacketTimestamp = 0;
    g_lastPeerHeartbeatTimestamp = 0;
}

void handlePairAck(uint32_t nowMs, const uint8_t mac[6], const Packet &packet) {
    if (!g_waitingForAck || !macEqualBytes(mac, g_pendingMac)) {
        return;
    }
    g_waitingForAck = false;
    g_linkStatus.paired = true;
    g_linkStatus.peerIdentity = packet.id;
    std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
    std::memcpy(g_linkStatus.peerMac, g_pendingMac, 6);
    g_linkStatus.lastActivityMs = nowMs;
    g_linkStatus.lastCommandMs = 0;
    g_lastPeerHeartbeatTimestamp = nowMs;
    ensurePeer(mac);
}

void handlePairConfirm(uint32_t nowMs, const uint8_t mac[6], const Packet &packet) {
    if (!macEqualBytes(mac, packet.id.mac)) {
        // Ensure we know which MAC sent the packet even if the payload differs.
        Identity corrected = packet.id;
        std::memcpy(corrected.mac, mac, 6);
        addOrUpdateDiscovery(corrected, mac, nowMs);
    }

    g_linkStatus.paired = true;
    g_linkStatus.peerIdentity = packet.id;
    std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
    std::memcpy(g_linkStatus.peerMac, mac, 6);
    g_linkStatus.lastActivityMs = nowMs;
    g_linkStatus.lastCommandMs = 0;
    g_lastPeerHeartbeatTimestamp = nowMs;

    ensurePeer(mac);
    sendPacket(mac, MessageType::MSG_PAIR_ACK);
}

void handleIdentityReply(uint32_t nowMs, const uint8_t mac[6], const Packet &packet) {
    Identity identity = packet.id;
    std::memcpy(identity.mac, mac, 6);
    addOrUpdateDiscovery(identity, mac, nowMs);

    if (g_role != DeviceRole::Controller || g_linkStatus.paired) {
        return;
    }

    int selection = -1;
    if (g_targetSelector) {
        selection = g_targetSelector(g_discoveries.data(), g_discoveryCount);
    }
    if (selection < 0 || static_cast<size_t>(selection) >= g_discoveryCount) {
        selection = g_discoveryCount > 0 ? 0 : -1;
    }
    if (selection < 0) {
        return;
    }

    const DiscoveryInfo &target = g_discoveries[selection];
    ensurePeer(target.identity.mac);
    sendPacket(target.identity.mac, MessageType::MSG_PAIR_CONFIRM);

    g_waitingForAck = true;
    std::memcpy(g_pendingMac, target.identity.mac, 6);
    g_pendingIdentity = target.identity;
    g_lastPairAttemptMs = nowMs;
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    uint32_t nowMs = millis();

    if (len == static_cast<int>(sizeof(Packet))) {
        const Packet *packet = reinterpret_cast<const Packet *>(incomingData);
        if (packet->version != PROTOCOL_VERSION) {
            if (g_userRecvCb) {
                g_userRecvCb(mac, incomingData, len);
            }
            return;
        }

        switch (packet->type) {
        case MessageType::MSG_PAIR_REQ:
            if (g_role == DeviceRole::Controlled && !g_linkStatus.paired) {
                sendPacket(mac, MessageType::MSG_IDENTITY_REPLY);
            }
            break;
        case MessageType::MSG_IDENTITY_REPLY:
            handleIdentityReply(nowMs, mac, *packet);
            break;
        case MessageType::MSG_PAIR_CONFIRM:
            if (g_role == DeviceRole::Controlled) {
                handlePairConfirm(nowMs, mac, *packet);
            }
            break;
        case MessageType::MSG_PAIR_ACK:
            if (g_role == DeviceRole::Controller) {
                handlePairAck(nowMs, mac, *packet);
            }
            break;
        }

        g_linkStatus.lastActivityMs = nowMs;
        if (g_linkStatus.paired && macEqualBytes(mac, g_linkStatus.peerMac)) {
            g_lastPeerHeartbeatTimestamp = nowMs;
        }
    } else if (len == static_cast<int>(sizeof(PeripheralCommand))) {
        const uint32_t magic = *reinterpret_cast<const uint32_t *>(incomingData);
        if (magic == THEGILL_PERIPHERAL_MAGIC) {
            const PeripheralCommand *packet = reinterpret_cast<const PeripheralCommand *>(incomingData);
            bool fromBroadcast = macEqualBytes(mac, BroadcastMac);
            bool fromPeer = g_linkStatus.paired && macEqualBytes(mac, g_linkStatus.peerMac);

            if (!g_linkStatus.paired && !fromBroadcast) {
                g_linkStatus.paired = true;
                g_linkStatus.peerIdentity = Identity{};
                std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
                std::memcpy(g_linkStatus.peerMac, mac, 6);
                g_pendingThegillCommand = false;
                g_pendingArmCommand = false;
                g_pendingPeripheralCommand = false;
                g_pendingConfigurationPacket = false;
                g_pendingSettingsPacket = false;
                ensurePeer(mac);
                fromPeer = true;
            }

            if (fromPeer || (fromBroadcast && g_linkStatus.paired)) {
                g_lastPeripheralCommand = *packet;
                g_pendingPeripheralCommand = true;
                g_lastPeripheralCommandTimestamp = nowMs;
                g_linkStatus.lastCommandMs = nowMs;
                g_linkStatus.lastActivityMs = nowMs;
                if (fromPeer) {
                    g_lastPeerHeartbeatTimestamp = nowMs;
                }
            }
        } else if (magic == THEGILL_CONFIG_MAGIC) {
            const ConfigurationPacket *packet = reinterpret_cast<const ConfigurationPacket *>(incomingData);
            bool fromBroadcast = macEqualBytes(mac, BroadcastMac);
            bool fromPeer = g_linkStatus.paired && macEqualBytes(mac, g_linkStatus.peerMac);

            if (!g_linkStatus.paired && !fromBroadcast) {
                g_linkStatus.paired = true;
                g_linkStatus.peerIdentity = Identity{};
                std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
                std::memcpy(g_linkStatus.peerMac, mac, 6);
                g_pendingThegillCommand = false;
                g_pendingArmCommand = false;
                g_pendingPeripheralCommand = false;
                g_pendingConfigurationPacket = false;
                g_pendingSettingsPacket = false;
                ensurePeer(mac);
                fromPeer = true;
            }

            if (fromPeer || (fromBroadcast && g_linkStatus.paired)) {
                g_lastConfigurationPacket = *packet;
                g_pendingConfigurationPacket = true;
                g_lastConfigurationPacketTimestamp = nowMs;
                g_linkStatus.lastCommandMs = nowMs;
                g_linkStatus.lastActivityMs = nowMs;
                if (fromPeer) {
                    g_lastPeerHeartbeatTimestamp = nowMs;
                }
            }
        }
    } else if (len >= static_cast<int>(sizeof(SettingsPacket))) {
        const SettingsPacket *packet = reinterpret_cast<const SettingsPacket *>(incomingData);
        if (packet->magic == THEGILL_SETTINGS_MAGIC) {
            bool fromBroadcast = macEqualBytes(mac, BroadcastMac);
            bool fromPeer = g_linkStatus.paired && macEqualBytes(mac, g_linkStatus.peerMac);

            if (!g_linkStatus.paired && !fromBroadcast) {
                g_linkStatus.paired = true;
                g_linkStatus.peerIdentity = Identity{};
                std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
                std::memcpy(g_linkStatus.peerMac, mac, 6);
                g_pendingThegillCommand = false;
                g_pendingArmCommand = false;
                g_pendingPeripheralCommand = false;
                g_pendingConfigurationPacket = false;
                g_pendingSettingsPacket = false;
                ensurePeer(mac);
                fromPeer = true;
            }

            if (fromPeer || (fromBroadcast && g_linkStatus.paired)) {
                g_lastSettingsPacket = *packet;
                g_pendingSettingsPacket = true;
                g_lastSettingsPacketTimestamp = nowMs;
                g_linkStatus.lastCommandMs = nowMs;
                g_linkStatus.lastActivityMs = nowMs;
                if (fromPeer) {
                    g_lastPeerHeartbeatTimestamp = nowMs;
                }
            }
        }
    } else if (len >= static_cast<int>(sizeof(ArmControlCommand))) {
        const ArmControlCommand *packet = reinterpret_cast<const ArmControlCommand *>(incomingData);
        if (packet->magic == ARM_COMMAND_MAGIC) {
            bool fromBroadcast = macEqualBytes(mac, BroadcastMac);
            bool fromPeer = g_linkStatus.paired && macEqualBytes(mac, g_linkStatus.peerMac);

            if (!g_linkStatus.paired && !fromBroadcast) {
                g_linkStatus.paired = true;
                g_linkStatus.peerIdentity = Identity{};
                std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
                std::memcpy(g_linkStatus.peerMac, mac, 6);
                g_pendingThegillCommand = false;
                g_pendingArmCommand = false;
                g_pendingPeripheralCommand = false;
                g_pendingConfigurationPacket = false;
                g_pendingSettingsPacket = false;
                ensurePeer(mac);
                fromPeer = true;
            }

            if (fromPeer || (fromBroadcast && g_linkStatus.paired)) {
                g_lastArmCommand = *packet;
                g_pendingArmCommand = true;
                g_lastArmCommandTimestamp = nowMs;
                g_linkStatus.lastCommandMs = nowMs;
                g_linkStatus.lastActivityMs = nowMs;
                if (fromPeer) {
                    g_lastPeerHeartbeatTimestamp = nowMs;
                }
            }
        }
    } else if (len >= static_cast<int>(sizeof(ThegillCommand))) {
        const ThegillCommand *packet = reinterpret_cast<const ThegillCommand *>(incomingData);
        if (packet->magic == THEGILL_PACKET_MAGIC) {
            bool fromBroadcast = macEqualBytes(mac, BroadcastMac);
            bool fromPeer = g_linkStatus.paired && macEqualBytes(mac, g_linkStatus.peerMac);

            if (!g_linkStatus.paired && !fromBroadcast) {
                g_linkStatus.paired = true;
                g_linkStatus.peerIdentity = Identity{};
                std::memcpy(g_linkStatus.peerIdentity.mac, mac, 6);
                std::memcpy(g_linkStatus.peerMac, mac, 6);
                g_pendingArmCommand = false;
                g_pendingPeripheralCommand = false;
                g_pendingConfigurationPacket = false;
                g_pendingSettingsPacket = false;
                ensurePeer(mac);
                fromPeer = true;
            }

            if (fromPeer || (fromBroadcast && g_linkStatus.paired)) {
                g_lastThegillCommand = *packet;
                g_pendingThegillCommand = true;
                g_lastThegillCommandTimestamp = nowMs;
                g_linkStatus.lastCommandMs = nowMs;
                g_linkStatus.lastActivityMs = nowMs;
                if (fromPeer) {
                    g_lastPeerHeartbeatTimestamp = nowMs;
                }
            }
        }
    }

    if (g_userRecvCb) {
        g_userRecvCb(mac, incomingData, len);
    }
}

} // namespace

const uint8_t BroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setRole(DeviceRole role) {
    g_role = role;
}

DeviceRole getRole() {
    return g_role;
}

void setPlatform(const char *platformName) {
    if (!platformName) {
        g_platform[0] = '\0';
        return;
    }
    std::strncpy(g_platform, platformName, sizeof(g_platform) - 1);
    g_platform[sizeof(g_platform) - 1] = '\0';
}

void setCustomId(const char *customId) {
    if (!customId) {
        g_customId[0] = '\0';
        return;
    }
    std::strncpy(g_customId, customId, sizeof(g_customId) - 1);
    g_customId[sizeof(g_customId) - 1] = '\0';
}

void setDeviceTypeOverride(const char *deviceTypeName) {
    if (!deviceTypeName) {
        g_deviceTypeOverride[0] = '\0';
        return;
    }
    std::strncpy(g_deviceTypeOverride, deviceTypeName, sizeof(g_deviceTypeOverride) - 1);
    g_deviceTypeOverride[sizeof(g_deviceTypeOverride) - 1] = '\0';
}

void setTargetSelector(TargetSelector selector) {
    g_targetSelector = selector;
}

void fillSelfIdentity(Identity &outIdentity) {
    composeIdentity(outIdentity);
}

String macToString(const uint8_t mac[6]) {
    char buffer[18];
    std::snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buffer);
}

bool macEqual(const uint8_t lhs[6], const uint8_t rhs[6]) {
    return macEqualBytes(lhs, rhs);
}

bool ensurePeer(const uint8_t mac[6]) {
    if (!mac || macEqualBytes(mac, BroadcastMac)) {
        return false;
    }
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }

    esp_now_peer_info_t peerInfo{};
    std::memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

bool init(const char *ssid, const char *password, int tcpPort) {
    return init(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    (void)tcpPort;

    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    if (!esp_now_is_peer_exist(BroadcastMac)) {
        esp_now_peer_info_t broadcastPeer{};
        std::memcpy(broadcastPeer.peer_addr, BroadcastMac, 6);
        broadcastPeer.channel = WIFI_CHANNEL;
        broadcastPeer.encrypt = false;
        if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
            esp_now_deinit();
            return false;
        }
    }

    g_userRecvCb = recvCallback;
    esp_now_register_recv_cb(onDataRecv);

    g_initialized = true;
    g_discoveryCount = 0;
    g_lastBroadcastMs = millis();
    g_lastDiscoveryCleanupMs = g_lastBroadcastMs;
    g_waitingForAck = false;
    std::memset(g_pendingMac, 0, sizeof(g_pendingMac));
    g_pendingIdentity = Identity{};
    clearLinkStatus();

    return true;
}

void loop() {
    if (!g_initialized) {
        return;
    }

    uint32_t nowMs = millis();

    if (!g_linkStatus.paired) {
        if (nowMs - g_lastBroadcastMs >= BROADCAST_INTERVAL_MS) {
            g_lastBroadcastMs = nowMs;
            if (g_role == DeviceRole::Controller) {
                sendPacket(BroadcastMac, MessageType::MSG_PAIR_REQ);
            } else {
                sendPacket(BroadcastMac, MessageType::MSG_IDENTITY_REPLY);
            }
        }

        if (g_role == DeviceRole::Controller && g_waitingForAck &&
            nowMs - g_lastPairAttemptMs > LINK_TIMEOUT_MS) {
            g_waitingForAck = false;
        }
    }

    pruneDiscoveries(nowMs);
}

bool receiveArmCommand(ArmControlCommand &cmd, uint32_t *timestampMs) {
    if (!g_pendingArmCommand) {
        return false;
    }

    cmd = g_lastArmCommand;
    if (timestampMs) {
        *timestampMs = g_lastArmCommandTimestamp;
    }
    g_pendingArmCommand = false;
    return true;
}

bool receiveThegillCommand(ThegillCommand &cmd, uint32_t *timestampMs) {
    if (!g_pendingThegillCommand) {
        return false;
    }

    cmd = g_lastThegillCommand;
    if (timestampMs) {
        *timestampMs = g_lastThegillCommandTimestamp;
    }
    g_pendingThegillCommand = false;
    return true;
}

bool receivePeripheralCommand(PeripheralCommand &cmd, uint32_t *timestampMs) {
    if (!g_pendingPeripheralCommand) {
        return false;
    }

    cmd = g_lastPeripheralCommand;
    if (timestampMs) {
        *timestampMs = g_lastPeripheralCommandTimestamp;
    }
    g_pendingPeripheralCommand = false;
    return true;
}

bool receiveConfigurationPacket(ConfigurationPacket &cmd, uint32_t *timestampMs) {
    if (!g_pendingConfigurationPacket) {
        return false;
    }

    cmd = g_lastConfigurationPacket;
    if (timestampMs) {
        *timestampMs = g_lastConfigurationPacketTimestamp;
    }
    g_pendingConfigurationPacket = false;
    return true;
}

bool receiveSettingsPacket(SettingsPacket &cmd, uint32_t *timestampMs) {
    if (!g_pendingSettingsPacket) {
        return false;
    }

    cmd = g_lastSettingsPacket;
    if (timestampMs) {
        *timestampMs = g_lastSettingsPacketTimestamp;
    }
    g_pendingSettingsPacket = false;
    return true;
}

uint32_t lastCommandTimestamp() {
    uint32_t latest = std::max(g_lastThegillCommandTimestamp, g_lastArmCommandTimestamp);
    latest = std::max(latest, g_lastPeripheralCommandTimestamp);
    return latest;
}

uint32_t lastThegillCommandTimestamp() {
    return g_lastThegillCommandTimestamp;
}

uint32_t lastArmCommandTimestamp() {
    return g_lastArmCommandTimestamp;
}

uint32_t lastConfigurationCommandTimestamp() {
    return g_lastConfigurationPacketTimestamp;
}

uint32_t lastSettingsCommandTimestamp() {
    return g_lastSettingsPacketTimestamp;
}

uint32_t lastPeerHeartbeatTimestamp() {
    return g_lastPeerHeartbeatTimestamp;
}

LinkStatus getLinkStatus() {
    return g_linkStatus;
}

bool paired() {
    return g_linkStatus.paired;
}

bool sendStatusPacket(const StatusPacket &packet) {
    if (!g_initialized || !g_linkStatus.paired) {
        return false;
    }
    if (macEqualBytes(g_linkStatus.peerMac, BroadcastMac)) {
        return false;
    }
    esp_err_t result = esp_now_send(g_linkStatus.peerMac,
                                    reinterpret_cast<const uint8_t *>(&packet),
                                    sizeof(packet));
    return result == ESP_OK;
}

bool sendArmStatePacket(const ArmStatePacket &packet) {
    if (!g_initialized || !g_linkStatus.paired) {
        return false;
    }
    if (macEqualBytes(g_linkStatus.peerMac, BroadcastMac)) {
        return false;
    }
    esp_err_t result = esp_now_send(g_linkStatus.peerMac,
                                    reinterpret_cast<const uint8_t *>(&packet),
                                    sizeof(packet));
    return result == ESP_OK;
}

} // namespace Comms


