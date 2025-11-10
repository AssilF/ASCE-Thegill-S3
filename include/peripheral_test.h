#pragma once

#include <stdint.h>

namespace PeripheralTest {

struct Status {
    bool active = false;
    bool autoStopEnabled = false;
    uint32_t autoStopMs = 0;
    uint32_t elapsedMs = 0;
};

bool active();
bool overridesPeripheralOutputs();
bool startBreathingTest(uint32_t durationMs = 0);
bool stopBreathingTest(uint32_t timeoutMs = 500);
Status status();

} // namespace PeripheralTest
