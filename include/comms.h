#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "thegill.h"

namespace Comms {
struct TelemetryPacket {
  uint32_t magic;                // Should be PACKET_MAGIC
  float pitch, roll, yaw;        // Orientation in degrees
  float leftFrontActual, leftRearActual, rightFrontActual; // Normalized motor outputs
  uint16_t rightRearActual;      // Encoded right-rear output (0-2000 => -1000..1000)
  int8_t leftFrontTarget, rightFrontTarget, leftRearTarget; // Target commands compressed to int8
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

bool init(const char *ssid, const char *password, int tcpPort);
bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback);
bool receiveCommand(ThegillCommand &cmd);
bool paired();
extern const uint8_t BroadcastMac[6];
}

