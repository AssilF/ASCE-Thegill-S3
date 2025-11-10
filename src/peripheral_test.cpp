#include "peripheral_test.h"

#include <Arduino.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "shift_register.h"

namespace {

struct ChannelPattern {
    ShiftRegister::Output output;
    float periodMs;
    float phaseOffsetMs;
    uint8_t minDuty;
    uint8_t maxDuty;
};

constexpr ChannelPattern kPatterns[] = {
    {ShiftRegister::Output::HighPowerLed1, 3200.0f,   0.0f, 15, 255},
    {ShiftRegister::Output::HighPowerLed2, 2100.0f, 180.0f, 15, 255},
    {ShiftRegister::Output::HighPowerLed3, 1400.0f, 360.0f, 15, 255},
    {ShiftRegister::Output::PumpControl,   2800.0f, 520.0f,  0, 180},
};

constexpr size_t kPatternCount = sizeof(kPatterns) / sizeof(kPatterns[0]);
constexpr float kTwoPi = 6.28318530718f;
constexpr TickType_t kLoopDelay = pdMS_TO_TICKS(15);

portMUX_TYPE gTestMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t gTaskHandle = nullptr;
bool gStopRequested = false;
uint32_t gAutoStopMs = 0;
uint32_t gStartMillis = 0;

bool testActiveUnsafe() {
    return gTaskHandle != nullptr;
}

bool stopRequested() {
    portENTER_CRITICAL(&gTestMux);
    bool requested = gStopRequested;
    portEXIT_CRITICAL(&gTestMux);
    return requested;
}

void resetState() {
    portENTER_CRITICAL(&gTestMux);
    gTaskHandle = nullptr;
    gStopRequested = false;
    gAutoStopMs = 0;
    gStartMillis = 0;
    portEXIT_CRITICAL(&gTestMux);
}

void PeripheralBreathTask(void *pvParameters) {
    const uint32_t autoStopDurationMs = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(pvParameters));
    const bool autoStopEnabled = autoStopDurationMs > 0;
    const uint64_t startUs = esp_timer_get_time();
    float lastDuty[kPatternCount] = {0.0f};

    while (true) {
        if (stopRequested()) {
            break;
        }

        uint64_t nowUs = esp_timer_get_time();
        for (size_t idx = 0; idx < kPatternCount; ++idx) {
            const auto &pattern = kPatterns[idx];
            float currentMs = static_cast<float>(nowUs) / 1000.0f + pattern.phaseOffsetMs;
            if (pattern.periodMs <= 0.0f) {
                lastDuty[idx] = static_cast<float>(pattern.minDuty);
                ShiftRegister::writeChannelPwm(pattern.output, pattern.minDuty);
                continue;
            }
            float cycles = currentMs / pattern.periodMs;
            float fractional = cycles - floorf(cycles);
            float angle = fractional * kTwoPi;
            float breath = 0.5f * (1.0f - cosf(angle));
            float duty = pattern.minDuty + breath * static_cast<float>(pattern.maxDuty - pattern.minDuty);
            duty = constrain(duty, 0.0f, 255.0f);
            lastDuty[idx] = duty;
            ShiftRegister::writeChannelPwm(pattern.output, static_cast<uint8_t>(duty + 0.5f));
        }

        if (autoStopEnabled) {
            uint64_t elapsedMs = (esp_timer_get_time() - startUs) / 1000ULL;
            if (elapsedMs >= autoStopDurationMs) {
                break;
            }
        }

        vTaskDelay(kLoopDelay);
    }

    constexpr uint8_t kRampSteps = 8;
    for (int step = kRampSteps; step >= 0; --step) {
        float scale = static_cast<float>(step) / static_cast<float>(kRampSteps);
        for (size_t idx = 0; idx < kPatternCount; ++idx) {
            uint8_t duty = static_cast<uint8_t>(lastDuty[idx] * scale + 0.5f);
            ShiftRegister::writeChannelPwm(kPatterns[idx].output, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for (const auto &pattern : kPatterns) {
        ShiftRegister::writeChannelPwm(pattern.output, 0);
    }

    resetState();
    vTaskDelete(nullptr);
}

} // namespace

namespace PeripheralTest {

bool active() {
    portENTER_CRITICAL(&gTestMux);
    bool state = testActiveUnsafe();
    portEXIT_CRITICAL(&gTestMux);
    return state;
}

bool overridesPeripheralOutputs() {
    return active();
}

bool startBreathingTest(uint32_t durationMs) {
    if (!ShiftRegister::initialized()) {
        return false;
    }

    bool alreadyRunning = false;
    {
        portENTER_CRITICAL(&gTestMux);
        alreadyRunning = (gTaskHandle != nullptr);
        portEXIT_CRITICAL(&gTestMux);
    }
    if (alreadyRunning) {
        if (durationMs == 0) {
            return true;
        }
        if (!stopBreathingTest()) {
            return false;
        }
    }

    portENTER_CRITICAL(&gTestMux);
    gStopRequested = false;
    gAutoStopMs = durationMs;
    gStartMillis = millis();
    portEXIT_CRITICAL(&gTestMux);

    TaskHandle_t handle = nullptr;
    BaseType_t created = xTaskCreatePinnedToCore(
        PeripheralBreathTask,
        "PeripheralTest",
        2048,
        reinterpret_cast<void *>(static_cast<uintptr_t>(durationMs)),
        1,
        &handle,
        1);

    if (created != pdPASS || handle == nullptr) {
        resetState();
        return false;
    }

    portENTER_CRITICAL(&gTestMux);
    gTaskHandle = handle;
    portEXIT_CRITICAL(&gTestMux);
    return true;
}

bool stopBreathingTest(uint32_t timeoutMs) {
    {
        portENTER_CRITICAL(&gTestMux);
        if (gTaskHandle == nullptr) {
            portEXIT_CRITICAL(&gTestMux);
            return false;
        }
        gStopRequested = true;
        portEXIT_CRITICAL(&gTestMux);
    }

    const TickType_t pollDelay = pdMS_TO_TICKS(10);
    const TickType_t timeoutTicks = timeoutMs > 0 ? pdMS_TO_TICKS(timeoutMs) : 0;
    const TickType_t startTicks = xTaskGetTickCount();

    while (true) {
        portENTER_CRITICAL(&gTestMux);
        bool stillRunning = (gTaskHandle != nullptr);
        portEXIT_CRITICAL(&gTestMux);
        if (!stillRunning) {
            return true;
        }
        if (timeoutMs > 0) {
            TickType_t elapsedTicks = xTaskGetTickCount() - startTicks;
            if (elapsedTicks >= timeoutTicks) {
                return false;
            }
        }
        vTaskDelay(pollDelay);
    }
}

Status status() {
    Status snapshot{};
    uint32_t startMs = 0;
    portENTER_CRITICAL(&gTestMux);
    snapshot.active = testActiveUnsafe();
    snapshot.autoStopEnabled = gAutoStopMs > 0;
    snapshot.autoStopMs = gAutoStopMs;
    startMs = gStartMillis;
    portEXIT_CRITICAL(&gTestMux);

    if (snapshot.active && startMs != 0) {
        uint32_t now = millis();
        snapshot.elapsedMs = (now >= startMs) ? (now - startMs) : 0;
    } else {
        snapshot.elapsedMs = 0;
    }
    return snapshot;
}

} // namespace PeripheralTest
