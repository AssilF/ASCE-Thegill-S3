#include "arm_servos.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <cmath>

#include "device_config.h"

namespace ArmServos {
namespace {

struct ServoConfig {
    uint8_t pin;
    uint8_t channel;
    float minDegrees;
    float maxDegrees;
    uint16_t minPulseMicros;
    uint16_t maxPulseMicros;
    float defaultDegrees;
};

constexpr size_t kServoCount = static_cast<size_t>(ServoId::Count);

// 50 Hz hobby servo PWM using LEDC (low-speed mode). Use channels that avoid the buzzer (channel 7).
constexpr uint32_t kServoPwmFrequencyHz = 50;
constexpr uint32_t kServoPwmPeriodUs = 1000000UL / kServoPwmFrequencyHz;
constexpr uint8_t kServoResolutionBits = 14; // ESP32-S3 LEDC supports up to 14 bits
constexpr uint32_t kServoMaxDuty = (1u << kServoResolutionBits) - 1;

constexpr ServoConfig kConfigs[kServoCount] = {
    // Constrain pulses to a tighter 50 Hz window (approx 1.05-1.95 ms) for stability
    {config::kServoShoulderPin, 0, 0.0f, 180.0f, 1050, 1950, 90.0f},
    {config::kServoElbowPin, 1, 0.0f, 180.0f, 1050, 1950, 90.0f},
    {config::kServoGripperPitchPin, 2, 0.0f, 180.0f, 1050, 1950, 90.0f},
    {config::kServoRollPin, 3, 0.0f, 180.0f, 1050, 1950, 90.0f},
    {config::kServoYawPin, 4, 0.0f, 180.0f, 1050, 1950, 90.0f},
};

float gTargets[kServoCount];
float gFilteredTargets[kServoCount];
bool gEnabled = true;
bool gInitialised = false;

uint8_t gPins[kServoCount];
bool gActive[kServoCount];

constexpr float clampf(float value, float minValue, float maxValue) {
    return (value < minValue) ? minValue : (value > maxValue ? maxValue : value);
}

constexpr size_t toIndex(ServoId id) {
    return static_cast<size_t>(id);
}

uint32_t degreesToMicros(size_t index, float degrees) {
    const ServoConfig &cfg = kConfigs[index];
    float d = clampf(degrees, cfg.minDegrees, cfg.maxDegrees);
    float spanDeg = (cfg.maxDegrees - cfg.minDegrees);
    if (spanDeg <= 0.0f) spanDeg = 180.0f;
    float t = (d - cfg.minDegrees) / spanDeg; // 0..1
    uint32_t spanUs = static_cast<uint32_t>(cfg.maxPulseMicros - cfg.minPulseMicros);
    return static_cast<uint32_t>(cfg.minPulseMicros + t * spanUs);
}

uint32_t pulseUsToDuty(uint32_t pulseUs) {
    if (pulseUs > kServoPwmPeriodUs) {
        pulseUs = kServoPwmPeriodUs;
    }
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(pulseUs) * kServoMaxDuty + (kServoPwmPeriodUs / 2)) /
        kServoPwmPeriodUs);
}

void attachServo(size_t index) {
    if (index >= kServoCount) {
        return;
    }
    const ServoConfig &cfg = kConfigs[index];
    gPins[index] = cfg.pin;
    pinMode(gPins[index], OUTPUT);
    digitalWrite(gPins[index], LOW);
    double actual = ledcSetup(cfg.channel, kServoPwmFrequencyHz, kServoResolutionBits);
    if (actual <= 0) {
        gActive[index] = false;
        return;
    }
    ledcAttachPin(gPins[index], cfg.channel);
    gActive[index] = true;
}

void attachAllServos() {
    for (size_t i = 0; i < kServoCount; ++i) {
        attachServo(i);
    }
}

void detachAllServos() {
    for (size_t i = 0; i < kServoCount; ++i) {
        ledcDetachPin(gPins[i]);
        gActive[i] = false;
    }
}

void ensureAttachedMissing() {
    for (size_t i = 0; i < kServoCount; ++i) {
        if (!gActive[i]) {
            attachServo(i);
        }
    }
}

void applyServoOutput(size_t index, float targetDegrees) {
    if (!gEnabled || index >= kServoCount) {
        return;
    }
    if (!gActive[index]) {
        attachServo(index);
        if (!gActive[index]) {
            return;
        }
    }
    const ServoConfig &cfg = kConfigs[index];
    float clampedDeg = clampf(targetDegrees, cfg.minDegrees, cfg.maxDegrees);
    uint32_t pw = degreesToMicros(index, clampedDeg);
    if (pw < cfg.minPulseMicros) pw = cfg.minPulseMicros;
    if (pw > cfg.maxPulseMicros) pw = cfg.maxPulseMicros;
    uint32_t duty = pulseUsToDuty(pw);
    if (duty > kServoMaxDuty) duty = kServoMaxDuty;
    ledcWrite(cfg.channel, duty);
}

} // namespace

void init() {
    // Release JTAG defaults on S3 (39/40/41/42) so we can drive them as GPIO
    const gpio_num_t jtagPins[] = {GPIO_NUM_39, GPIO_NUM_40, GPIO_NUM_41, GPIO_NUM_42};
    for (gpio_num_t pin : jtagPins) {
        if (pin != GPIO_NUM_NC) {
            gpio_reset_pin(pin);
        }
    }

    attachAllServos();
    for (size_t i = 0; i < kServoCount; ++i) {
        gTargets[i] = kConfigs[i].defaultDegrees;
        gFilteredTargets[i] = gTargets[i];
    }
    gInitialised = true;
    for (size_t i = 0; i < kServoCount; ++i) {
        applyServoOutput(i, gFilteredTargets[i]);
    }
}

void setEnabled(bool enabled) {
    if (gEnabled == enabled) {
        if (gEnabled && gInitialised) {
            ensureAttachedMissing();
        }
        return;
    }
    gEnabled = enabled;
    if (!gInitialised) {
        return;
    }
    if (!gEnabled) {
        detachAllServos();
    } else {
        attachAllServos();
        for (size_t i = 0; i < kServoCount; ++i) {
            applyServoOutput(i, gFilteredTargets[i]);
        }
    }
}

bool enabled() {
    return gEnabled;
}

bool setTargetDegrees(ServoId id, float degrees) {
    size_t index = toIndex(id);
    if (index >= kServoCount) {
        return false;
    }
    float minDeg = kConfigs[index].minDegrees;
    float maxDeg = kConfigs[index].maxDegrees;
    if (maxDeg <= minDeg) {
        maxDeg = minDeg + 1.0f;
    }
    degrees = clampf(degrees, minDeg, maxDeg);
    gTargets[index] = degrees;
    // Light filtering on updates to reduce jitter from noisy commands
    constexpr float kFilterAlpha = 0.2f;
    float filtered = gFilteredTargets[index] + kFilterAlpha * (degrees - gFilteredTargets[index]);
    gFilteredTargets[index] = filtered;
    applyServoOutput(index, filtered);
    return true;
}

bool setTargetNormalized(ServoId id, float normalized) {
    size_t index = toIndex(id);
    if (index >= kServoCount) {
        return false;
    }
    normalized = clampf(normalized, 0.0f, 1.0f);
    const ServoConfig &cfg = kConfigs[index];
    float range = cfg.maxDegrees - cfg.minDegrees;
    float degrees = cfg.minDegrees + normalized * range;
    return setTargetDegrees(id, degrees);
}

float targetDegrees(ServoId id) {
    size_t index = toIndex(id);
    if (index >= kServoCount) {
        return 0.0f;
    }
    return gTargets[index];
}

float normalizedTarget(ServoId id) {
    size_t index = toIndex(id);
    if (index >= kServoCount) {
        return 0.0f;
    }
    const ServoConfig &cfg = kConfigs[index];
    float range = cfg.maxDegrees - cfg.minDegrees;
    if (range <= 0.0f) {
        return 0.0f;
    }
    return (gTargets[index] - cfg.minDegrees) / range;
}

ServoStatus status(ServoId id) {
    size_t index = toIndex(id);
    ServoStatus info{};
    if (index >= kServoCount) {
        info.id = ServoId::Shoulder;
        return info;
    }
    info.id = id;
    info.enabled = gEnabled;
    info.attached = gEnabled && gActive[index];
    info.targetDegrees = gTargets[index];
    info.minDegrees = kConfigs[index].minDegrees;
    info.maxDegrees = kConfigs[index].maxDegrees;
    return info;
}

const char *name(ServoId id) {
    switch (id) {
    case ServoId::Shoulder:
        return "shoulder";
    case ServoId::Elbow:
        return "elbow";
    case ServoId::Pitch:
        return "pitch";
    case ServoId::Roll:
        return "roll";
    case ServoId::Yaw:
        return "yaw";
    default:
        return "servo";
    }
}

} // namespace ArmServos
