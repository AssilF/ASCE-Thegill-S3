#pragma once

#include <Arduino.h>
#include <cstddef>

namespace Motor {
struct Outputs {
    int16_t leftFront;
    int16_t leftRear;
    int16_t rightFront;
    int16_t rightRear;
    void constrainAll();
};

struct DriverPins {
    int forwardPin;
    int reversePin;
    int enablePin;
};

struct EncoderMeasurement {
    int32_t totalTicks;
    float metersTravelled;
    float metersPerSecond;
    bool valid;
};

bool init(const DriverPins &lf, const DriverPins &lr,
          const DriverPins &rf, const DriverPins &rr);
void calibrate();
void update(bool enabled, bool brake, Outputs &current, const Outputs &target);
void stop();
void setDynamicFrequencyEnabled(bool enabled);
bool dynamicFrequencyEnabled();

bool encoderMeasurement(std::size_t index, EncoderMeasurement &out);
std::size_t encoderMeasurements(EncoderMeasurement *out, std::size_t maxCount);

} 

