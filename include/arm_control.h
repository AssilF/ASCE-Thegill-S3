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

float baseTargetNormalized();
float extensionTargetNormalized();

float basePositionNormalized();
float extensionPositionNormalized();

Status status();

} // namespace ArmControl
