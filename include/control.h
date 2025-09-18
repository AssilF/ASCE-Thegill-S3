#pragma once
#include <Arduino.h>

struct ControlOutputs {
    float roll;
    float pitch;
    float yaw;
    float vertical;
};

void computeCorrections(float pitchSetpoint,
                        float rollSetpoint,
                        float yawSetpoint,
                        float pitch,
                        float roll,
                        float yaw,
                        float gyroX,
                        float gyroY,
                        float gyroZ,
                        float verticalAcc,
                        bool throttleStable,
                        bool yawEnabled,
                        ControlOutputs &out);
