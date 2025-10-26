#include "arm_control.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

#include "device_config.h"
#include "shift_register.h"

namespace ArmControl {
namespace {

struct AxisPins {
    ShiftRegister::Output pwm;
    ShiftRegister::Output in1;
    ShiftRegister::Output in2;
};

struct AxisConfig {
    AxisPins pins;
    uint8_t analogPin;
    bool invertFeedback;
    bool invertDirection;
    uint16_t adcMin;
    uint16_t adcMax;
    float kp;
    float ki;
    float kd;
};

struct AxisState {
    AxisConfig config;
    float target = 0.0f;
    float filteredPosition = 0.0f;
    float rawPosition = 0.0f;
    uint16_t lastAdc = 0;
    float integral = 0.0f;
    float lastError = 0.0f;
    float effort = 0.0f;
    bool active = false;
    bool filterInitialised = false;

    AxisState() = default;
    explicit AxisState(const AxisConfig &cfg) : config(cfg) {}
};

constexpr AxisConfig kBaseConfig{
    {ShiftRegister::Output::RotationPwmA,
     ShiftRegister::Output::RotationAIn1,
     ShiftRegister::Output::RotationAIn2},
    config::kArmBasePotPin,
    config::arm::kBaseInvertFeedback,
    config::arm::kBaseInvertDirection,
    config::arm::kBaseAdcMin,
    config::arm::kBaseAdcMax,
    config::arm::kBaseKp,
    config::arm::kBaseKi,
    config::arm::kBaseKd};

constexpr AxisConfig kExtensionConfig{
    {ShiftRegister::Output::ExtensionPwmB,
     ShiftRegister::Output::ExtensionBIn1,
     ShiftRegister::Output::ExtensionBIn2},
    config::kArmExtensionPotPin,
    config::arm::kExtensionInvertFeedback,
    config::arm::kExtensionInvertDirection,
    config::arm::kExtensionAdcMin,
    config::arm::kExtensionAdcMax,
    config::arm::kExtensionKp,
    config::arm::kExtensionKi,
    config::arm::kExtensionKd};

AxisState gBase{kBaseConfig};
AxisState gExtension{kExtensionConfig};

bool gOutputsEnabled = true;
bool gInitialised = false;

float clampf(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

float clamp01(float value) {
    return clampf(value, 0.0f, 1.0f);
}

float readAxisSample(AxisState &axis) {
    uint16_t raw = static_cast<uint16_t>(analogRead(axis.config.analogPin));
    axis.lastAdc = raw;
    float span = static_cast<float>(static_cast<int32_t>(axis.config.adcMax) -
                                    static_cast<int32_t>(axis.config.adcMin));
    float value = 0.0f;
    if (span > 1.0f) {
        value = (static_cast<float>(raw) - static_cast<float>(axis.config.adcMin)) / span;
    }
    value = clamp01(value);
    if (axis.config.invertFeedback) {
        value = 1.0f - value;
    }
    return clamp01(value);
}

void configureAnalogPin(uint8_t pin) {
    pinMode(pin, INPUT);
#ifdef ARDUINO_ARCH_ESP32
    adcAttachPin(pin);
    analogSetPinAttenuation(pin, ADC_11db);
#endif
}

void idleAxis(AxisState &axis) {
    axis.active = false;
    axis.effort = 0.0f;
    if (!ShiftRegister::initialized()) {
        return;
    }
    ShiftRegister::writeChannelPwm(axis.config.pins.pwm, 0);
    ShiftRegister::writeChannel(axis.config.pins.in1, false);
    ShiftRegister::writeChannel(axis.config.pins.in2, false);
}

void applyAxis(AxisState &axis) {
    if (!ShiftRegister::initialized()) {
        axis.active = false;
        return;
    }

    float magnitude = std::fabs(axis.effort);
    bool engaged = gOutputsEnabled && magnitude >= config::arm::kDeadband;

    if (!engaged) {
        idleAxis(axis);
        return;
    }

    bool positive = axis.effort >= 0.0f;
    if (axis.config.invertDirection) {
        positive = !positive;
    }

    float limited = std::min(magnitude, config::arm::kOutputLimit);
    uint8_t duty = static_cast<uint8_t>(std::round(limited * 255.0f));
    if (duty > 255) {
        duty = 255;
    }

    ShiftRegister::writeChannel(axis.config.pins.in1, positive);
    ShiftRegister::writeChannel(axis.config.pins.in2, !positive);
    ShiftRegister::writeChannelPwm(axis.config.pins.pwm, duty);

    axis.active = duty > 0;
}

void updateAxis(AxisState &axis, float dtSeconds) {
    if (dtSeconds <= 0.0f) {
        return;
    }

    float sample = readAxisSample(axis);
    axis.rawPosition = sample;
    if (!axis.filterInitialised) {
        axis.filteredPosition = sample;
        axis.filterInitialised = true;
    } else {
        axis.filteredPosition += config::arm::kSampleAlpha * (sample - axis.filteredPosition);
    }

    float error = axis.target - axis.filteredPosition;
    axis.integral += error * dtSeconds;
    axis.integral = clampf(axis.integral,
                           -config::arm::kIntegralLimit,
                           config::arm::kIntegralLimit);

    float derivative = (error - axis.lastError) / dtSeconds;
    axis.lastError = error;

    float effort = axis.config.kp * error +
                   axis.config.ki * axis.integral +
                   axis.config.kd * derivative;

    axis.effort = clampf(effort, -config::arm::kOutputLimit, config::arm::kOutputLimit);
}

AxisStatus buildAxisStatus(const AxisState &axis) {
    AxisStatus status{};
    status.position = axis.filteredPosition;
    status.raw = axis.rawPosition;
    status.adc = axis.lastAdc;
    status.target = axis.target;
    status.error = axis.target - axis.filteredPosition;
    status.effort = axis.effort;
    status.active = axis.active && gOutputsEnabled;
    return status;
}

} // namespace

void init() {
    analogReadResolution(12);
    configureAnalogPin(gBase.config.analogPin);
    configureAnalogPin(gExtension.config.analogPin);

    gBase.filterInitialised = false;
    gExtension.filterInitialised = false;

    gBase.integral = 0.0f;
    gExtension.integral = 0.0f;
    gBase.lastError = 0.0f;
    gExtension.lastError = 0.0f;

    gBase.target = readAxisSample(gBase);
    gExtension.target = readAxisSample(gExtension);
    gBase.filteredPosition = gBase.target;
    gExtension.filteredPosition = gExtension.target;
    gBase.filterInitialised = true;
    gExtension.filterInitialised = true;

    if (ShiftRegister::initialized()) {
        ShiftRegister::writeChannel(ShiftRegister::Output::ArmStandby, gOutputsEnabled);
        idleAxis(gBase);
        idleAxis(gExtension);
    }

    gInitialised = true;
}

void update(float dtSeconds) {
    if (!gInitialised) {
        return;
    }

    updateAxis(gBase, dtSeconds);
    updateAxis(gExtension, dtSeconds);

    if (!gOutputsEnabled) {
        idleAxis(gBase);
        idleAxis(gExtension);
        if (ShiftRegister::initialized()) {
        ShiftRegister::writeChannel(ShiftRegister::Output::ArmStandby, false);
        }
        return;
    }

    applyAxis(gBase);
    applyAxis(gExtension);

    if (ShiftRegister::initialized()) {
        bool anyActive = gBase.active || gExtension.active;
        ShiftRegister::writeChannel(ShiftRegister::Output::ArmStandby, anyActive || gOutputsEnabled);
    }
}

void setOutputsEnabled(bool enabled) {
    if (enabled == gOutputsEnabled) {
        return;
    }
    gOutputsEnabled = enabled;
    gBase.integral = 0.0f;
    gExtension.integral = 0.0f;
    gBase.lastError = 0.0f;
    gExtension.lastError = 0.0f;

    if (!gInitialised) {
        return;
    }

    if (!enabled) {
        idleAxis(gBase);
        idleAxis(gExtension);
    }

    if (ShiftRegister::initialized()) {
        ShiftRegister::writeChannel(ShiftRegister::Output::ArmStandby, enabled);
    }
}

bool outputsEnabled() {
    return gOutputsEnabled;
}

void setBaseTargetNormalized(float value) {
    gBase.target = clamp01(value);
    gBase.integral = 0.0f;
    gBase.lastError = gBase.target - gBase.filteredPosition;
}

void setExtensionTargetNormalized(float value) {
    gExtension.target = clamp01(value);
    gExtension.integral = 0.0f;
    gExtension.lastError = gExtension.target - gExtension.filteredPosition;
}

float baseTargetNormalized() {
    return gBase.target;
}

float extensionTargetNormalized() {
    return gExtension.target;
}

float basePositionNormalized() {
    return gBase.filteredPosition;
}

float extensionPositionNormalized() {
    return gExtension.filteredPosition;
}

Status status() {
    Status result{};
    result.base = buildAxisStatus(gBase);
    result.extension = buildAxisStatus(gExtension);
    result.outputsEnabled = gOutputsEnabled;
    return result;
}

} // namespace ArmControl
