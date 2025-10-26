#include "arm_servos.h"

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#include <cmath>

#include "device_config.h"

namespace ArmServos {
namespace {

struct ServoConfig {
    uint8_t pin;
    float minDegrees;
    float maxDegrees;
    uint16_t minPulseMicros;
    uint16_t maxPulseMicros;
    float defaultDegrees;
};

constexpr size_t kServoCount = static_cast<size_t>(ServoId::Count);

constexpr ServoConfig kConfigs[kServoCount] = {
    {config::kServoShoulderPin, 0.0f, 180.0f, 500, 2500, 90.0f},
    {config::kServoElbowPin, 0.0f, 180.0f, 500, 2500, 90.0f},
    {config::kServoGripperPitchPin, 0.0f, 180.0f, 500, 2500, 90.0f},
    {config::kServoRollPin, 0.0f, 180.0f, 500, 2500, 90.0f},
    {config::kServoYawPin, 0.0f, 180.0f, 500, 2500, 90.0f},
};

#if defined(ARDUINO_ARCH_ESP32)
Servo gServos[kServoCount];
#else
Servo gServos[kServoCount];
#endif

float gTargets[kServoCount];
bool gAttached[kServoCount];
bool gEnabled = true;
bool gInitialised = false;

constexpr float clampf(float value, float minValue, float maxValue) {
    return (value < minValue) ? minValue : (value > maxValue ? maxValue : value);
}

constexpr size_t toIndex(ServoId id) {
    return static_cast<size_t>(id);
}

void applyTarget(size_t index) {
    if (!gEnabled || !gAttached[index]) {
        return;
    }
    gServos[index].write(static_cast<int>(std::lround(gTargets[index])));
}

void attachServo(size_t index) {
    if (gAttached[index]) {
        return;
    }
#if defined(ARDUINO_ARCH_ESP32)
    gServos[index].setPeriodHertz(50);
#endif
    gServos[index].attach(kConfigs[index].pin,
                          kConfigs[index].minPulseMicros,
                          kConfigs[index].maxPulseMicros);
    gAttached[index] = gServos[index].attached();
    applyTarget(index);
}

void detachServo(size_t index) {
    if (!gAttached[index]) {
        return;
    }
    gServos[index].detach();
    gAttached[index] = false;
}

} // namespace

void init() {
#if defined(ARDUINO_ARCH_ESP32)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
#endif

    for (size_t i = 0; i < kServoCount; ++i) {
        pinMode(kConfigs[i].pin, OUTPUT);
        gTargets[i] = kConfigs[i].defaultDegrees;
        gAttached[i] = false;
    }

    gInitialised = true;

    if (gEnabled) {
        for (size_t i = 0; i < kServoCount; ++i) {
            attachServo(i);
        }
    }
}

void setEnabled(bool enabled) {
    if (gEnabled == enabled) {
        return;
    }
    gEnabled = enabled;
    if (!gInitialised) {
        return;
    }
    if (gEnabled) {
        for (size_t i = 0; i < kServoCount; ++i) {
            attachServo(i);
        }
    } else {
        for (size_t i = 0; i < kServoCount; ++i) {
            detachServo(i);
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
    if (gEnabled && gInitialised) {
        if (!gAttached[index]) {
            attachServo(index);
        } else {
            applyTarget(index);
        }
    }
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
    info.attached = gAttached[index] && gServos[index].attached();
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
