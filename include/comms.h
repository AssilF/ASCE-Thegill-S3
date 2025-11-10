#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "thegill.h"

namespace Comms {

enum class DeviceRole : uint8_t {
    Controller = 0,
    Controlled = 1,
};

#ifndef DEVICE_ROLE
#define DEVICE_ROLE Comms::DeviceRole::Controlled
#endif

enum class MessageType : uint8_t {
    MSG_PAIR_REQ = 0x01,
    MSG_IDENTITY_REPLY = 0x02,
    MSG_PAIR_CONFIRM = 0x03,
    MSG_PAIR_ACK = 0x04,
};


constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t WIFI_CHANNEL = 0;
constexpr uint32_t BROADCAST_INTERVAL_MS = 500;
constexpr uint32_t DEVICE_TTL_MS = 5000;
constexpr uint32_t LINK_TIMEOUT_MS = 3000;

#pragma pack(push, 1)
struct Identity {
    char deviceType[12];
    char platform[16];
    char customId[32];
    uint8_t mac[6];
};

struct Packet {
    uint8_t version;
    MessageType type;
    Identity id;
    uint32_t monotonicMs;
    uint32_t reserved;
};

struct ControlPacket {
    uint8_t replyIndex;
    int8_t speed;
    uint8_t motionState;
    uint8_t buttonStates[3];
    uint8_t ledPwm[3];       // PWM duty (0-255) for each high-power LED
    uint8_t pumpIntensity;   // Pump PWM duty (0-255)
    uint8_t commandByte;     // Bitmask for auxiliary user outputs/commands
};
#pragma pack(pop)

struct DiscoveryInfo {
    Identity identity;
    uint32_t lastSeenMs;
};

struct LinkStatus {
    bool paired = false;
    Identity peerIdentity{};
    uint8_t peerMac[6] = {0};
    uint32_t lastActivityMs = 0;
    uint32_t lastCommandMs = 0;
};

using TargetSelector = int (*)(const DiscoveryInfo *entries, size_t count);

void setRole(DeviceRole role);
DeviceRole getRole();
void setPlatform(const char *platformName);
void setCustomId(const char *customId);
void setDeviceTypeOverride(const char *deviceTypeName);
void setTargetSelector(TargetSelector selector);

void fillSelfIdentity(Identity &outIdentity);
String macToString(const uint8_t mac[6]);
bool macEqual(const uint8_t lhs[6], const uint8_t rhs[6]);
bool ensurePeer(const uint8_t mac[6]);

bool init(const char *ssid, const char *password, int tcpPort);
bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback);
void loop();

bool receiveCommand(ControlPacket &cmd, uint32_t *timestampMs = nullptr);
bool receiveThegillCommand(ThegillCommand &cmd, uint32_t *timestampMs = nullptr);
bool receiveArmCommand(ArmControlCommand &cmd, uint32_t *timestampMs = nullptr);
uint32_t lastCommandTimestamp();
uint32_t lastThegillCommandTimestamp();
uint32_t lastArmCommandTimestamp();
LinkStatus getLinkStatus();
bool paired();

extern const uint8_t BroadcastMac[6];

} // namespace Comms

