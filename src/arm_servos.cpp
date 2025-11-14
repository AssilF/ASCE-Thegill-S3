#include "arm_servos.h"

#include <Arduino.h>
#include <esp_timer.h>
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

float gTargets[kServoCount];
bool gEnabled = true;
bool gInitialised = false;

// Servo pulse scheduling (50 Hz frame)
constexpr uint32_t kFramePeriodUs = 20000; // 20 ms
esp_timer_handle_t gFrameTimer = nullptr;  // periodic frame timer
esp_timer_handle_t gOffTimers[kServoCount] = {nullptr}; // per-servo oneshot to drive pin low
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

void IRAM_ATTR offTimerCallback(void *arg) {
    size_t idx = reinterpret_cast<size_t>(arg);
    if (idx < kServoCount) {
        digitalWrite(gPins[idx], LOW);
    }
}

void frameTimerCallback(void *arg) {
    (void)arg;
    if (!gEnabled) return;
    // Start-of-frame: drive all active servo pins HIGH, then arm oneshots to pull LOW at each pulse width
    for (size_t i = 0; i < kServoCount; ++i) {
        if (!gActive[i]) continue;
        digitalWrite(gPins[i], HIGH);
        uint32_t pw = degreesToMicros(i, gTargets[i]);
        // Clamp pulse to sane range (500..2500 us)
        const ServoConfig &cfg = kConfigs[i];
        if (pw < cfg.minPulseMicros) pw = cfg.minPulseMicros;
        if (pw > cfg.maxPulseMicros) pw = cfg.maxPulseMicros;
        if (gOffTimers[i]) {
            esp_timer_stop(gOffTimers[i]);
            esp_timer_start_once(gOffTimers[i], pw);
        }
    }
}

void ensureTimersCreated() {
    if (!gFrameTimer) {
        esp_timer_create_args_t fa{};
        fa.callback = &frameTimerCallback;
        fa.arg = nullptr;
        fa.dispatch_method = ESP_TIMER_TASK;
        fa.name = "servo_frame";
        esp_timer_create(&fa, &gFrameTimer);
    }
    for (size_t i = 0; i < kServoCount; ++i) {
        if (!gOffTimers[i]) {
            esp_timer_create_args_t oa{};
            oa.callback = &offTimerCallback;
            oa.arg = reinterpret_cast<void*>(i);
            oa.dispatch_method = ESP_TIMER_TASK;
            oa.name = "servo_off";
            esp_timer_create(&oa, &gOffTimers[i]);
        }
    }
}

} // namespace

void init() {
    for (size_t i = 0; i < kServoCount; ++i) {
        gPins[i] = kConfigs[i].pin;
        pinMode(gPins[i], OUTPUT);
        digitalWrite(gPins[i], LOW);
        gTargets[i] = kConfigs[i].defaultDegrees;
        gActive[i] = true;
    }

    ensureTimersCreated();
    gInitialised = true;
    if (gFrameTimer) {
        esp_timer_start_periodic(gFrameTimer, kFramePeriodUs);
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
    if (!gEnabled) {
        for (size_t i = 0; i < kServoCount; ++i) {
            if (gOffTimers[i]) {
                esp_timer_stop(gOffTimers[i]);
            }
            digitalWrite(gPins[i], LOW);
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
