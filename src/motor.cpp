#include "motor.h"

#include <Arduino.h>
#include <cmath>
#include <algorithm>

#include "device_config.h"
#include "shift_register.h"
#include "pcint_encoder.h"

namespace Motor {
namespace {

struct DriverState {
    DriverPins pins;
    bool initialised;
};

struct PwmState {
    volatile uint32_t periodTicks;
    volatile uint32_t onTicks;
    volatile uint64_t cycleStart;
    volatile uint64_t nextEvent;
    volatile uint8_t phase;
    volatile uint8_t activePin;
};

constexpr size_t kMotorCount = config::kMotorCount;
constexpr uint32_t kTimerBaseFrequencyHz = 1000000;  // 1 MHz timer base
constexpr uint32_t kPwmFrequencyMin = config::kMotorPwmFrequencyLow;
constexpr uint32_t kDefaultIdleInterval =
    kTimerBaseFrequencyHz / (kPwmFrequencyMin > 0 ? kPwmFrequencyMin : 1);

constexpr uint8_t kPhaseStart = 0;
constexpr uint8_t kPhaseEnd = 1;
constexpr uint8_t kPhaseHoldHigh = 2;

constexpr uint8_t kActiveNone = 0;
constexpr uint8_t kActiveForward = 1;
constexpr uint8_t kActiveReverse = 2;

constexpr float kCommandScale = 1000.0f;
constexpr float kCommandDeadband = config::kMotorCommandDeadband;
constexpr float kCommandDeadbandCounts = kCommandDeadband * kCommandScale;
constexpr float kMetersPerTick = config::kMetersPerEncoderTick;
constexpr uint32_t kFixedPwmFrequencyHz = config::kMotorPwmFixedFrequency;
constexpr float kMaxSlewTimeSeconds = 0.5f;  // limit to reach a new target in <= 0.5s
constexpr uint32_t kMaxSlewTimeMicros = 500000;  // mirror of 0.5s for integer math
constexpr uint32_t kFallbackDtMicros = 2000;     // ~500 Hz expected loop
constexpr bool kSlewLimitingEnabled = false;     // Disable slew limiter to avoid pre-stop spikes
constexpr uint32_t kMinAlarmIntervalTicks = 25;  // Clamp timer alarm (~25 us) to avoid ISR storms

DriverState drivers[kMotorCount];
PwmState pwmStates[kMotorCount];
bool subsystemInitialised = false;
hw_timer_t *pwmTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t currentTicks = 0;
volatile uint32_t alarmInterval = kDefaultIdleInterval;
volatile bool dynamicPwmEnabled = config::kMotorDynamicPwmEnabledDefault;

struct EncoderSample {
    EncoderMeasurement measurement{};
    int32_t lastTicks{0};
    uint64_t lastMicros{0};
    bool initialised{false};
};

EncoderSample encoderSamples[kMotorCount];
uint64_t lastUpdateMicros = 0;

inline int16_t applyCommandDeadband(int16_t value) {
    if (fabsf(static_cast<float>(value)) <= kCommandDeadbandCounts) {
        return 0;
    }
    return value;
}

inline void setPinLevel(int pin, int level) {
    if (pin >= 0) {
        digitalWrite(pin, level ? HIGH : LOW);
    }
}

uint32_t sanitizeFrequency(uint32_t frequency) {
    if (frequency == 0) {
        return kPwmFrequencyMin == 0 ? 1 : kPwmFrequencyMin;
    }
    return frequency;
}

uint32_t chooseFrequency(float magnitude) {
    if (!dynamicPwmEnabled) {
        return sanitizeFrequency(kFixedPwmFrequencyHz);
    }

    if (magnitude >= config::kMotorFrequencyThresholdMidHigh) {
        return sanitizeFrequency(config::kMotorPwmFrequencyHigh);
    }
    if (magnitude >= config::kMotorFrequencyThresholdMidLow) {
        return sanitizeFrequency(config::kMotorPwmFrequencyMidHigh);
    }
    if (magnitude >= config::kMotorFrequencyThresholdLow) {
        return sanitizeFrequency(config::kMotorPwmFrequencyMidLow);
    }
    return sanitizeFrequency(config::kMotorPwmFrequencyLow);
}

uint32_t computePeriodTicks(uint32_t frequency) {
    if (frequency == 0) {
        return kDefaultIdleInterval;
    }
    uint32_t ticks = kTimerBaseFrequencyHz / frequency;
    return ticks == 0 ? 1 : ticks;
}

inline void IRAM_ATTR setAlarmIntervalLocked(uint32_t interval) {
    if (!pwmTimer) {
        return;
    }
    if (interval < kMinAlarmIntervalTicks) {
        interval = kMinAlarmIntervalTicks;
    }
    alarmInterval = interval;
    timerWrite(pwmTimer, 0);
    timerAlarmWrite(pwmTimer, interval, false);
    timerAlarmEnable(pwmTimer);
}

void IRAM_ATTR handleDriverEvents(size_t index, uint64_t now) {
    PwmState &pwm = pwmStates[index];
    if (pwm.activePin == kActiveNone || pwm.periodTicks == 0) {
        return;
    }

    const DriverPins &pins = drivers[index].pins;

    // Process only a limited number of steps to avoid long ISR lockouts that can trip the watchdog
    constexpr uint8_t kMaxStepsPerIsr = 8;
    uint8_t steps = 0;
    while (pwm.nextEvent <= now && steps < kMaxStepsPerIsr) {
        if (pwm.phase == kPhaseStart) {
            pwm.cycleStart = pwm.nextEvent;
            setPinLevel(pins.forwardPin, LOW);
            setPinLevel(pins.reversePin, LOW);

            if (pwm.onTicks == 0) {
                pwm.cycleStart += pwm.periodTicks;
                pwm.nextEvent = pwm.cycleStart;
                continue;
            }

            if (pwm.activePin == kActiveForward) {
                setPinLevel(pins.forwardPin, HIGH);
            } else {
                setPinLevel(pins.reversePin, HIGH);
            }

            if (pwm.onTicks >= pwm.periodTicks) {
                pwm.phase = kPhaseHoldHigh;
                pwm.nextEvent = pwm.cycleStart + pwm.periodTicks;
            } else {
                pwm.phase = kPhaseEnd;
                pwm.nextEvent = pwm.cycleStart + pwm.onTicks;
            }
        } else if (pwm.phase == kPhaseEnd) {
            if (pwm.activePin == kActiveForward) {
                setPinLevel(pins.forwardPin, LOW);
            } else {
                setPinLevel(pins.reversePin, LOW);
            }
            pwm.phase = kPhaseStart;
            pwm.cycleStart += pwm.periodTicks;
            pwm.nextEvent = pwm.cycleStart;
        } else {  // kPhaseHoldHigh
            pwm.phase = kPhaseStart;
            pwm.cycleStart += pwm.periodTicks;
            pwm.nextEvent = pwm.cycleStart;
        }
        ++steps;
    }

    // If we are still behind schedule, resync to the current time boundary and drop outputs safely
    if (pwm.nextEvent <= now) {
        pwm.phase = kPhaseStart;
        pwm.cycleStart = now;
        pwm.nextEvent = pwm.cycleStart + pwm.periodTicks;
        setPinLevel(pins.forwardPin, LOW);
        setPinLevel(pins.reversePin, LOW);
    }
}

void IRAM_ATTR onPwmTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    uint32_t interval = alarmInterval;
    if (interval == 0) {
        interval = 1;
    }
    currentTicks += interval;

    uint32_t minDelta = UINT32_MAX;
    for (size_t i = 0; i < kMotorCount; ++i) {
        handleDriverEvents(i, currentTicks);
        PwmState &pwm = pwmStates[i];
        if (pwm.activePin == kActiveNone || pwm.periodTicks == 0) {
            continue;
        }

        if (pwm.nextEvent <= currentTicks) {
            minDelta = 1;
        } else {
            uint32_t delta = static_cast<uint32_t>(pwm.nextEvent - currentTicks);
            if (delta < minDelta) {
                minDelta = delta;
            }
        }
    }

    if (minDelta == UINT32_MAX) {
        setAlarmIntervalLocked(kDefaultIdleInterval);
    } else {
        setAlarmIntervalLocked(minDelta);
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

void configureDriver(size_t index, const DriverPins &pins) {
    DriverState &state = drivers[index];
    state.pins = pins;
    state.initialised = true;

    pinMode(pins.forwardPin, OUTPUT);
    digitalWrite(pins.forwardPin, LOW);
    pinMode(pins.reversePin, OUTPUT);
    digitalWrite(pins.reversePin, LOW);

    if (pins.enablePin >= 0) {
        pinMode(pins.enablePin, OUTPUT);
        digitalWrite(pins.enablePin, LOW);
    }

    uint32_t baseFrequency = chooseFrequency(0.0f);
    PwmState &pwm = pwmStates[index];
    pwm.periodTicks = computePeriodTicks(baseFrequency);
    pwm.onTicks = 0;
    pwm.cycleStart = 0;
    pwm.nextEvent = kDefaultIdleInterval;
    pwm.phase = kPhaseStart;
    pwm.activePin = kActiveNone;
}

void applyOutput(size_t index, int16_t command, bool outputsEnabled, bool brake) {
    DriverState &state = drivers[index];
    if (!state.initialised) {
        return;
    }

    int16_t filteredCommand = outputsEnabled ? applyCommandDeadband(command) : 0;
    float magnitude = fabsf(static_cast<float>(filteredCommand)) / kCommandScale;
    magnitude = constrain(magnitude, 0.0f, 1.0f);

    uint32_t frequency = chooseFrequency(magnitude);
    uint32_t periodTicks = computePeriodTicks(frequency);
    if (periodTicks == 0) {
        periodTicks = 1;
    }

    if (!pwmTimer) {
        // Coast by default: both direction pins LOW
        setPinLevel(state.pins.forwardPin, LOW);
        setPinLevel(state.pins.reversePin, LOW);

        if (filteredCommand > 0) {
            setPinLevel(state.pins.forwardPin, HIGH);
        } else if (filteredCommand < 0) {
            setPinLevel(state.pins.reversePin, HIGH);
        }

        if (state.pins.enablePin >= 0) {
            if (brake || filteredCommand == 0) {
                digitalWrite(state.pins.enablePin, LOW);
            } else {
                digitalWrite(state.pins.enablePin, HIGH);
            }
        }
        return;
    }

    bool driverActive = false;

    portENTER_CRITICAL(&timerMux);
    PwmState &pwm = pwmStates[index];
    pwm.periodTicks = periodTicks;
    pwm.phase = kPhaseStart;
    pwm.cycleStart = currentTicks;

    if (brake) {
        pwm.onTicks = 0;
        pwm.activePin = kActiveNone;
        pwm.nextEvent = currentTicks + periodTicks;
        // Coast on brake: both direction lines low
        setPinLevel(state.pins.forwardPin, LOW);
        setPinLevel(state.pins.reversePin, LOW);
    } else if (filteredCommand == 0) {
        pwm.onTicks = 0;
        pwm.activePin = kActiveNone;
        pwm.nextEvent = currentTicks + periodTicks;
        setPinLevel(state.pins.forwardPin, LOW);
        setPinLevel(state.pins.reversePin, LOW);
    } else {
        uint32_t onTicks = static_cast<uint32_t>(
            lroundf(static_cast<float>(periodTicks) * magnitude));
        if (onTicks > periodTicks) {
            onTicks = periodTicks;
        }

        if (onTicks == 0) {
            pwm.onTicks = 0;
            pwm.activePin = kActiveNone;
            pwm.nextEvent = currentTicks + periodTicks;
            setPinLevel(state.pins.forwardPin, LOW);
            setPinLevel(state.pins.reversePin, LOW);
        } else {
    pwm.onTicks = onTicks;
    pwm.activePin =
        (filteredCommand > 0) ? kActiveForward : kActiveReverse;
    // Always drop both lines before setting the active one to avoid shoot-through
    setPinLevel(state.pins.forwardPin, LOW);
            setPinLevel(state.pins.reversePin, LOW);
            pwm.nextEvent = currentTicks;
            setAlarmIntervalLocked(1);
            driverActive = true;
        }
    }
    portEXIT_CRITICAL(&timerMux);

    if (state.pins.enablePin >= 0) {
        // Enable only when actively driving
        digitalWrite(state.pins.enablePin, (driverActive && !brake) ? HIGH : LOW);
    }
}

inline float ticksToMeters(int32_t ticks) {
    return kMetersPerTick * static_cast<float>(ticks);
}

void updateEncoderSample(std::size_t index, uint64_t nowMicros) {
    if (!PcintEncoder::configured() || index >= PcintEncoder::encoderCount()) {
        encoderSamples[index].measurement.valid = false;
        encoderSamples[index].initialised = false;
        return;
    }

    PcintEncoder::EncoderReading reading = PcintEncoder::read(index);
    EncoderSample &sample = encoderSamples[index];
    sample.measurement.totalTicks = reading.total;
    sample.measurement.metersTravelled = ticksToMeters(reading.total);
    sample.measurement.valid = reading.valid && PcintEncoder::enabled();

    if (!sample.measurement.valid) {
        sample.initialised = false;
        sample.measurement.metersPerSecond = 0.0f;
        return;
    }

    if (!sample.initialised) {
        sample.lastTicks = reading.total;
        sample.lastMicros = nowMicros;
        sample.measurement.metersPerSecond = 0.0f;
        sample.initialised = true;
        return;
    }

    uint64_t deltaMicros = nowMicros - sample.lastMicros;
    int32_t deltaTicks = reading.total - sample.lastTicks;

    float deltaSeconds =
        (deltaMicros == 0) ? 0.0f : static_cast<float>(deltaMicros) * 1e-6f;
    float deltaMeters = ticksToMeters(deltaTicks);
    if (deltaSeconds > 0.0f) {
        sample.measurement.metersPerSecond = deltaMeters / deltaSeconds;
    } else {
        sample.measurement.metersPerSecond = 0.0f;
    }

    sample.lastTicks = reading.total;
    sample.lastMicros = nowMicros;
}

void updateEncoderSamples() {
    if (kMetersPerTick == 0.0f) {
        for (std::size_t i = 0; i < kMotorCount; ++i) {
            encoderSamples[i].measurement.valid = false;
            encoderSamples[i].initialised = false;
        }
        return;
    }

    uint64_t nowMicros = micros();
    for (std::size_t i = 0; i < kMotorCount; ++i) {
        updateEncoderSample(i, nowMicros);
    }
}

}  // namespace

void Outputs::constrainAll() {
    leftFront = constrain(leftFront, -1000, 1000);
    leftRear = constrain(leftRear, -1000, 1000);
    rightFront = constrain(rightFront, -1000, 1000);
    rightRear = constrain(rightRear, -1000, 1000);
}

bool init(const DriverPins &lf, const DriverPins &lr,
          const DriverPins &rf, const DriverPins &rr) {
    configureDriver(0, lf);
    configureDriver(1, lr);
    configureDriver(2, rf);
    configureDriver(3, rr);

    currentTicks = 0;
    alarmInterval = kDefaultIdleInterval;

    pwmTimer = timerBegin(0, 80, true);  // 1 MHz clock
    if (!pwmTimer) {
        return false;
    }
    timerAttachInterrupt(pwmTimer, &onPwmTimer, true);
    setAlarmIntervalLocked(kDefaultIdleInterval);

    subsystemInitialised = true;
    return true;
}

void calibrate() {}

void stop() {
    if (!subsystemInitialised) {
        return;
    }

    for (size_t i = 0; i < kMotorCount; ++i) {
        applyOutput(i, 0, false, false);
    }
}

void setDynamicFrequencyEnabled(bool enabled) {
    dynamicPwmEnabled = enabled;
}

bool dynamicFrequencyEnabled() {
    return dynamicPwmEnabled;
}

void update(bool enabled, bool brake, Outputs &current, const Outputs &target) {
    if (!subsystemInitialised) {
        return;
    }

    Outputs previous = current;  // capture last applied values for slew limiting

    Outputs next{};
    if (brake) {
        next.leftFront = next.leftRear = next.rightFront = next.rightRear = 0;
    } else if (enabled) {
        next = target;
        next.constrainAll();
    } else {
        next.leftFront = next.leftRear = next.rightFront = next.rightRear = 0;
    }

    bool outputsEnabled = enabled && !brake;
    int16_t lf = next.leftFront;
    int16_t lr = next.leftRear;
    int16_t rf = next.rightFront;
    int16_t rr = next.rightRear;

    if (outputsEnabled) {
        lf = applyCommandDeadband(lf);
        lr = applyCommandDeadband(lr);
        rf = applyCommandDeadband(rf);
        rr = applyCommandDeadband(rr);
    } else {
        lf = lr = rf = rr = 0;
    }

    if (kSlewLimitingEnabled) {
        // Time-aware slew limiting to avoid sudden changes; guarantee <=0.5s to reach new target
        uint64_t nowMicros = micros();
        uint32_t dtMicros = kFallbackDtMicros;
        if (lastUpdateMicros != 0 && nowMicros > lastUpdateMicros) {
            uint64_t delta = nowMicros - lastUpdateMicros;
            dtMicros = static_cast<uint32_t>(std::min<uint64_t>(delta, kMaxSlewTimeMicros));
        }
        lastUpdateMicros = nowMicros;

        auto slewChannel = [&](int16_t previousValue, int16_t desired) -> int16_t {
            int32_t delta = static_cast<int32_t>(desired) - static_cast<int32_t>(previousValue);
            if (delta == 0) {
                return previousValue;
            }
            uint32_t mag = static_cast<uint32_t>(delta > 0 ? delta : -delta);
            // step = ceil(mag * dt / maxTime)
            uint32_t numer = mag * dtMicros + (kMaxSlewTimeMicros - 1);
            uint32_t step = numer / kMaxSlewTimeMicros;
            if (step == 0) {
                step = 1;
            }
            if (step > mag) {
                step = mag;
            }
            return static_cast<int16_t>(previousValue + (delta > 0 ? static_cast<int32_t>(step) : -static_cast<int32_t>(step)));
        };

        lf = slewChannel(previous.leftFront, lf);
        lr = slewChannel(previous.leftRear, lr);
        rf = slewChannel(previous.rightFront, rf);
        rr = slewChannel(previous.rightRear, rr);
    } else {
        lastUpdateMicros = micros();
    }

    applyOutput(0, lf, outputsEnabled, brake);
    applyOutput(1, lr, outputsEnabled, brake);
    applyOutput(2, rf, outputsEnabled, brake);
    applyOutput(3, rr, outputsEnabled, brake);

    next.leftFront = lf;
    next.leftRear = lr;
    next.rightFront = rf;
    next.rightRear = rr;

    current = next;

    updateEncoderSamples();
}

}  // namespace Motor

bool Motor::encoderMeasurement(std::size_t index, EncoderMeasurement &out) {
    if (index >= kMotorCount) {
        out = {};
        return false;
    }
    out = encoderSamples[index].measurement;
    return out.valid;
}

std::size_t Motor::encoderMeasurements(EncoderMeasurement *out,
                                       std::size_t maxCount) {
    if (!out || maxCount == 0) {
        return 0;
    }
    std::size_t count =
        std::min<std::size_t>(kMotorCount, maxCount);
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = encoderSamples[i].measurement;
    }
    return count;
}
