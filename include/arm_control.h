#pragma once

#include <cstdint>

namespace ArmControl {

struct AxisStatus {
    float position;
    float raw;
    uint16_t adc;
    float target;
    float error;
    float effort;
    bool active;
};

struct Status {
    AxisStatus base;
    AxisStatus extension;
    bool outputsEnabled;
};

void init();
void update(float dtSeconds);

void setOutputsEnabled(bool enabled);
bool outputsEnabled();

void setBaseTargetNormalized(float value);
void setExtensionTargetNormalized(float value);
void setBaseTargetDegrees(float degrees);
void setExtensionTargetCentimeters(float centimeters);

float baseTargetNormalized();
float extensionTargetNormalized();
float baseTargetDegrees();
float extensionTargetCentimeters();

float basePositionNormalized();
float extensionPositionNormalized();
float basePositionDegrees();
float extensionPositionCentimeters();

Status status();

} // namespace ArmControl
