#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

namespace Comms {
struct DriveCommand {
  
  uint32_t sequence = 0;
  uint8_t version = 0;
  int16_t motorDuty[4] = {0};
  uint16_t flags = 0;

}__attribute__((packed));

struct TelemetryPacket {
  uint32_t magic;                // Should be PACKET_MAGIC
  float pitch, roll, yaw;        // Orientation in degrees
  float pitchCorrection, rollCorrection, yawCorrection; // PID outputs
  uint16_t throttle;             // Current throttle command
  int8_t pitchAngle, rollAngle, yawAngle; // Commanded angles
  float verticalAcc;             // Vertical acceleration in m/s^2
  uint32_t commandAge;           // Age of last command in ms
} __attribute__((packed));

enum PairingType : uint8_t {
    SCAN_REQUEST = 0x01,
    DRONE_IDENTITY = 0x02,
    ILITE_IDENTITY = 0x03,
    DRONE_ACK = 0x04,
};

struct IdentityMessage {
    uint8_t type;
    char identity[16];
    uint8_t mac[6];
} __attribute__((packed));

void init(const char *ssid, const char *password, int tcpPort);
bool receiveCommand(ThrustCommand &cmd);
bool paired();
extern const uint8_t BroadcastMac[6];
}

