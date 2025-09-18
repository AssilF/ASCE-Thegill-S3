#include "control.h"

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
                        ControlOutputs &out) {
    // Simple PD stabilizer inspired by open-source DIY controllers like
    // MultiWii and the Crazyflie firmware.  Angles are corrected with a
    // proportional term while gyro rates provide damping.
    const float ANGLE_KP = 4.0f;   // proportional gain on angle error
    const float RATE_KD  = 0.1f;   // damping from gyro rate
    const float VERT_KP  = 20.0f;  // throttle units per m/s^2 of vertical acceleration

    float rollError  = rollSetpoint  - roll;
    float pitchError = pitchSetpoint - pitch;
    float yawError   = yawSetpoint   - yaw;

    if (yawError > 180) yawError -= 360;
    else if (yawError < -180) yawError += 360;

    out.roll  = ANGLE_KP * rollError  - RATE_KD * gyroX;
    out.pitch = ANGLE_KP * pitchError - RATE_KD * gyroY;
    out.yaw   = yawEnabled ? ANGLE_KP * yawError - RATE_KD * gyroZ : 0.0f;
    out.vertical = throttleStable ? VERT_KP * verticalAcc : 0.0f;
}
