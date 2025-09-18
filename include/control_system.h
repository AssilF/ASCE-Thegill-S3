#pragma once

#include <atomic>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <esp_now.h>

#include "buzzer_controller.h"
#include "control_protocol.h"
#include "device_config.h"
#include "motor_driver.h"
#include "status_led.h"

class ControlSystem {
public:
  void begin();
  void loop();
  void onEspNowData(const uint8_t *mac, const uint8_t *data, int len);

private:
  struct EspNowPacket {
    uint8_t mac[6];
    uint16_t length;
    uint8_t payload[ESP_NOW_MAX_DATA_LEN];
  };

  struct PairingState {
    bool paired = false;
    uint8_t controllerMac[6] = {0};
    char controllerName[16] = {0};
  };

  static void EspNowReceiveTrampoline(const uint8_t *mac, const uint8_t *data, int len);
  static void EspNowTaskTrampoline(void *param);
  static void UpdateTaskTrampoline(void *param);

  void enqueueEspNowPacketFromIsr(const uint8_t *mac, const uint8_t *data, int len);
  void espNowTask();
  void updateTask();
  void handleScanRequest(const uint8_t *mac);
  void handleControllerIdentity(const uint8_t *mac, const protocol::IdentityMessage &message);
  void handleControlPacket(const uint8_t *mac, const protocol::ControlMessage &packet);
  void handleGillControlPacket(const uint8_t *mac, const protocol::GillControlPacket &packet);
  void ensurePeer(const uint8_t *mac);
  bool sendIdentityMessage(const uint8_t *mac, protocol::MessageType type);
  void stopAllMotors();
  void enterFailsafe();
  void exitFailsafe();
  void updateDriveIndicator();
  void updateStatusForPairing();
  void updateStatusForConnection();

  MotorDriver motors_[config::kMotorCount];
  BuzzerController buzzer_;
  StatusLed statusLed_;
  PairingState pairingState_;
  uint8_t selfMac_[6] = {0};
  uint32_t lastControlTimestamp_ = 0;
  uint32_t lastHandshakeTimestamp_ = 0;
  bool failsafeActive_ = false;
  bool pendingPairingTone_ = false;
  float lastMotorCommands_[config::kMotorCount] = {0};

  QueueHandle_t espNowQueue_ = nullptr;
  TaskHandle_t espNowTaskHandle_ = nullptr;
  TaskHandle_t updateTaskHandle_ = nullptr;
  std::atomic<uint32_t> droppedPacketCount_{0};

  static ControlSystem *instance_;
};

