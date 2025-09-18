#pragma once

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "buzzer_controller.h"
#include "comms.h"
#include "device_config.h"
#include "motor_driver.h"
#include "network_setup.h"
#include "status_led.h"

class ControlSystem {
public:
  void begin();
  void loop();

private:
  static void UpdateTaskTrampoline(void *param);

  void updateTask();
  void applyDriveCommand(const Comms::Command &command, uint32_t timestamp);
  void stopAllMotors();
  void enterFailsafe();
  void exitFailsafe();
  void updateDriveIndicator();
  void updateStatusForPairing();
  void updateStatusForConnection();

  MotorDriver motors_[config::kMotorCount];
  BuzzerController buzzer_;
  StatusLed statusLed_;
  uint8_t selfMac_[6] = {0};
  uint32_t lastCommandTimestamp_ = 0;
  bool failsafeActive_ = false;
  bool pendingPairingTone_ = false;
  bool paired_ = false;
  float lastMotorCommands_[config::kMotorCount] = {0};

  TaskHandle_t updateTaskHandle_ = nullptr;
};

