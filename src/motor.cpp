#include "motor.h"

#include <Arduino.h>
#include <cmath>

#include "device_config.h"

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

DriverState drivers[kMotorCount];
PwmState pwmStates[kMotorCount];
bool subsystemInitialised = false;
hw_timer_t *pwmTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint64_t currentTicks = 0;
volatile uint32_t alarmInterval = kDefaultIdleInterval;

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

uint32_t chooseFrequency(float magnitude) {
    if (magnitude >= config::kMotorFrequencyThresholdMidHigh) {
        return config::kMotorPwmFrequencyHigh;
    }
    if (magnitude >= config::kMotorFrequencyThresholdMidLow) {
        return config::kMotorPwmFrequencyMidHigh;
    }
    if (magnitude >= config::kMotorFrequencyThresholdLow) {
        return config::kMotorPwmFrequencyMidLow;
    }
    return config::kMotorPwmFrequencyLow;
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
    if (interval == 0) {
        interval = 1;
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

    while (pwm.nextEvent <= now) {
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

    PwmState &pwm = pwmStates[index];
    pwm.periodTicks = computePeriodTicks(config::kMotorPwmFrequencyLow);
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
        if (brake) {
            setPinLevel(state.pins.forwardPin, HIGH);
            setPinLevel(state.pins.reversePin, HIGH);
        } else if (filteredCommand == 0) {
            setPinLevel(state.pins.forwardPin, LOW);
            setPinLevel(state.pins.reversePin, LOW);
        } else if (filteredCommand > 0) {
            setPinLevel(state.pins.forwardPin, HIGH);
            setPinLevel(state.pins.reversePin, LOW);
        } else {
            setPinLevel(state.pins.forwardPin, LOW);
            setPinLevel(state.pins.reversePin, HIGH);
        }

        if (state.pins.enablePin >= 0) {
            if (brake) {
                digitalWrite(state.pins.enablePin, HIGH);
            } else {
                digitalWrite(state.pins.enablePin,
                             filteredCommand != 0 ? HIGH : LOW);
            }
        }
        return;
    }

    portENTER_CRITICAL(&timerMux);
    PwmState &pwm = pwmStates[index];
    pwm.periodTicks = periodTicks;
    pwm.phase = kPhaseStart;
    pwm.cycleStart = currentTicks;

    if (brake) {
        pwm.onTicks = 0;
        pwm.activePin = kActiveNone;
        pwm.nextEvent = currentTicks + periodTicks;
        setPinLevel(state.pins.forwardPin, HIGH);
        setPinLevel(state.pins.reversePin, HIGH);
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
        if (magnitude > 0.0f && onTicks == 0) {
            onTicks = 1;
        }

        pwm.onTicks = onTicks;
        pwm.activePin = (filteredCommand > 0) ? kActiveForward : kActiveReverse;
        setPinLevel(state.pins.forwardPin, LOW);
        setPinLevel(state.pins.reversePin, LOW);
        pwm.nextEvent = currentTicks;
        setAlarmIntervalLocked(1);
    }
    portEXIT_CRITICAL(&timerMux);

    if (state.pins.enablePin >= 0) {
        if (brake) {
            digitalWrite(state.pins.enablePin, HIGH);
        } else {
            digitalWrite(state.pins.enablePin,
                         filteredCommand != 0 ? HIGH : LOW);
        }
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

void update(bool enabled, bool brake, Outputs &current, const Outputs &target) {
    if (!subsystemInitialised) {
        return;
    }

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

    applyOutput(0, lf, outputsEnabled, brake);
    applyOutput(1, lr, outputsEnabled, brake);
    applyOutput(2, rf, outputsEnabled, brake);
    applyOutput(3, rr, outputsEnabled, brake);

    next.leftFront = lf;
    next.leftRear = lr;
    next.rightFront = rf;
    next.rightRear = rr;


    current = next;
}

}  // namespace Motor

