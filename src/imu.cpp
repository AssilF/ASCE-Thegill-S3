#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// A minimal and robust IMU reading system. Raw accelerometer/gyro
// data are combined using a complementary filter and then passed
// through a second smoothing stage to reject high‑frequency motor
// noise. No Kalman or other complex filters are used to avoid
// unexpected resets to zero.

namespace IMU {
static MPU6050 mpu;

// Nested (fast and slow) filtered angles in degrees
static float pitchFast = 0, rollFast = 0, yawFast = 0;
static float pitchSlow = 0, rollSlow = 0, yawSlow = 0;
static float g_pitch = 0, g_roll = 0, g_yaw = 0;
static float g_verticalAcc = 0;
static float g_gx = 0, g_gy = 0, g_gz = 0;
static float pitchOffset = 0, rollOffset = 0, yawOffset = 0;
static float verticalAccOffset = 0; // offset to remove initial bias
static unsigned long lastUpdate = 0;

void init() {
    Wire.begin();
    mpu.initialize();
    zero();
}

void update() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    unsigned long now = millis();
    float dt = (now - lastUpdate)/1000.0f;
    if (dt <= 0 || dt > 0.1f) { lastUpdate = now; return; }
    lastUpdate = now;

    // Convert to physical units
    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay*ay+az*az)) * RAD_TO_DEG;
    float gyroXrate = gx/131.0f;
    float gyroYrate = gy/131.0f;
    float gyroZrate = gz/131.0f;

    g_gx = gyroXrate;
    g_gy = gyroYrate;
    g_gz = gyroZrate;

    // First stage complementary filter
    rollFast  = 0.98f*(rollFast  + gyroXrate*dt) + 0.02f*rollAcc;
    pitchFast = 0.98f*(pitchFast + gyroYrate*dt) + 0.02f*pitchAcc;
    yawFast   = yawFast + gyroZrate*dt; // no accel reference for yaw

    // Second stage smoothing to reject vibrations
    rollSlow  = 0.9f*rollSlow  + 0.1f*rollFast;
    pitchSlow = 0.9f*pitchSlow + 0.1f*pitchFast;
    yawSlow   = 0.9f*yawSlow   + 0.1f*yawFast;

    g_roll  = rollSlow  - rollOffset;
    g_pitch = pitchSlow - pitchOffset;
    g_yaw   = yawSlow   - yawOffset;
    if (g_yaw > 180) g_yaw -= 360; else if (g_yaw < -180) g_yaw += 360;

    const float ACC_SCALE = 16384.0f;
    float ax_ms2 = (float)ax/ACC_SCALE*9.81f;
    float ay_ms2 = (float)ay/ACC_SCALE*9.81f;
    float az_ms2 = (float)az/ACC_SCALE*9.81f;
    float pitchRad = g_pitch*DEG_TO_RAD;
    float rollRad  = g_roll*DEG_TO_RAD;
    // Transform body acceleration into world-frame Z. Using the standard
    // rotation (roll then pitch), world Z is
    //   cos(roll)*cos(pitch)*az + sin(roll)*cos(pitch)*ay - sin(pitch)*ax
    float worldZ = cos(rollRad)*cos(pitchRad)*az_ms2 +
                   sin(rollRad)*cos(pitchRad)*ay_ms2 -
                   sin(pitchRad)*ax_ms2;
    // Positive values correspond to upward acceleration.
    g_verticalAcc = (worldZ - 9.81f) - verticalAccOffset;
}

void zero() {
    // Collect IMU readings for at least 3 seconds before determining
    // offsets. This allows the sensors to settle and avoids repeated
    // recalibration if startup readings are far from the steady state.
    unsigned long start = millis();
    float sumRoll = 0, sumPitch = 0, sumYaw = 0, sumVert = 0;
    int samples = 0;
    while (millis() - start < 3000) {
        update();
        sumRoll  += rollSlow;
        sumPitch += pitchSlow;
        sumYaw   += yawSlow;
        sumVert  += g_verticalAcc;
        samples++;
        delay(5);
    }

    rollOffset = sumRoll / samples;
    pitchOffset = sumPitch / samples;
    yawOffset = sumYaw / samples;
    verticalAccOffset = sumVert / samples;

    // Run one more update so that g_* values reflect the new offsets.
    update();
}

void applyOffsetFromCurrent() {
    // Assume update() has been called recently so that the filtered
    // angles represent the current craft attitude. Rebase the offsets
    // to treat the latest orientation as the new zero without any
    // lengthy calibration cycle.
    rollOffset = rollSlow;
    pitchOffset = pitchSlow;
    yawOffset = yawSlow;

    // g_verticalAcc represents (worldZ - 9.81f) - verticalAccOffset.
    // Adjust the stored offset so that the current reading becomes
    // zeroed, then clear the cached value for immediate feedback.
    verticalAccOffset = g_verticalAcc + verticalAccOffset;
    g_roll = 0;
    g_pitch = 0;
    g_yaw = 0;
    g_verticalAcc = 0;
}

float pitch() { return g_pitch; }
float roll()  { return g_roll; }
float yaw()   { return g_yaw; }
float verticalAcc() { return g_verticalAcc; }
float gyroX() { return g_gx; }
float gyroY() { return g_gy; }
float gyroZ() { return g_gz; }
}

