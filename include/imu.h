#pragma once
#include <Arduino.h>

namespace IMU {
void init();
void update();
void zero();
void applyOffsetFromCurrent();
float pitch();
float roll();
float yaw();
float verticalAcc();
float gyroX();
float gyroY();
float gyroZ();
}

