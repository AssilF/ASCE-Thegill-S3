#pragma once

#include <Arduino.h>

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

bool init(const DriverPins &lf, const DriverPins &lr,
          const DriverPins &rf, const DriverPins &rr);
void calibrate();
void update(bool enabled, bool brake, Outputs &current, const Outputs &target);
void stop();
}

