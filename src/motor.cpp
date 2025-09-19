#include "motor.h"
#include <math.h>
#include <driver/gpio.h>

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

constexpr uint32_t PWM_MAX_FREQ = 20000;
constexpr uint32_t PWM_MIN_FREQ = 400;
constexpr uint32_t TIMER_BASE_FREQ = 1000000; // 1 MHz base for scheduling
constexpr uint32_t DEFAULT_IDLE_INTERVAL = 1000; // microseconds

constexpr uint8_t PHASE_START = 0;
constexpr uint8_t PHASE_END = 1;
constexpr uint8_t PHASE_HOLD_HIGH = 2;

constexpr uint8_t ACTIVE_NONE = 0;
constexpr uint8_t ACTIVE_FORWARD = 1;
constexpr uint8_t ACTIVE_REVERSE = 2;

static DriverState drivers[4];
static PwmState pwmStates[4];
static bool subsystemInitialised = false;
static float smoothingRate = 4.0f;

static hw_timer_t *pwmTimer = nullptr;
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t currentTicks = 0;
static volatile uint32_t alarmInterval = DEFAULT_IDLE_INTERVAL;

inline void IRAM_ATTR setPinLevel(int pin, int level)
{
    if (pin >= 0) {
        gpio_set_level(static_cast<gpio_num_t>(pin), level ? 1 : 0);
    }
}

uint32_t computeFrequency(float magnitude)
{
    magnitude = constrain(magnitude, 0.0f, 1.0f);
    return PWM_MIN_FREQ + static_cast<uint32_t>((PWM_MAX_FREQ - PWM_MIN_FREQ) * magnitude);
}

uint32_t computePeriodTicks(uint32_t freq)
{
    if (freq == 0) return DEFAULT_IDLE_INTERVAL;
    uint32_t ticks = TIMER_BASE_FREQ / freq;
    return ticks == 0 ? 1 : ticks;
}

inline void IRAM_ATTR setAlarmIntervalLocked(uint32_t interval)
{
    if (!pwmTimer) return;
    if (interval == 0) interval = 1;
    alarmInterval = interval;
    timerWrite(pwmTimer, 0);
    timerAlarmWrite(pwmTimer, interval, false);
    timerAlarmEnable(pwmTimer);
}

void IRAM_ATTR handleDriverEvents(size_t index, uint64_t now)
{
    PwmState &pwm = pwmStates[index];
    if (pwm.activePin == ACTIVE_NONE || pwm.periodTicks == 0) {
        return;
    }

    const DriverPins &pins = drivers[index].pins;

    while (pwm.nextEvent <= now) {
        if (pwm.phase == PHASE_START) {
            pwm.cycleStart = pwm.nextEvent;
            setPinLevel(pins.forwardPin, LOW);
            setPinLevel(pins.reversePin, LOW);

            if (pwm.onTicks == 0) {
                pwm.cycleStart += pwm.periodTicks;
                pwm.nextEvent = pwm.cycleStart;
                continue;
            }

            if (pwm.activePin == ACTIVE_FORWARD) {
                setPinLevel(pins.forwardPin, HIGH);
            } else {
                setPinLevel(pins.reversePin, HIGH);
            }

            if (pwm.onTicks >= pwm.periodTicks) {
                pwm.phase = PHASE_HOLD_HIGH;
                pwm.nextEvent = pwm.cycleStart + pwm.periodTicks;
            } else {
                pwm.phase = PHASE_END;
                pwm.nextEvent = pwm.cycleStart + pwm.onTicks;
            }
        } else if (pwm.phase == PHASE_END) {
            if (pwm.activePin == ACTIVE_FORWARD) {
                setPinLevel(pins.forwardPin, LOW);
            } else {
                setPinLevel(pins.reversePin, LOW);
            }
            pwm.phase = PHASE_START;
            pwm.cycleStart += pwm.periodTicks;
            pwm.nextEvent = pwm.cycleStart;
        } else { // PHASE_HOLD_HIGH
            pwm.phase = PHASE_START;
            pwm.cycleStart += pwm.periodTicks;
            pwm.nextEvent = pwm.cycleStart;
        }
    }
}

void IRAM_ATTR onPwmTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    uint32_t interval = alarmInterval;
    if (interval == 0) interval = 1;
    currentTicks += interval;

    uint32_t minDelta = UINT32_MAX;
    for (size_t i = 0; i < 4; ++i) {
        handleDriverEvents(i, currentTicks);
        PwmState &pwm = pwmStates[i];
        if (pwm.activePin == ACTIVE_NONE || pwm.periodTicks == 0) continue;

        if (pwm.nextEvent <= currentTicks) {
            minDelta = 1;
        } else {
            uint32_t delta = static_cast<uint32_t>(pwm.nextEvent - currentTicks);
            if (delta < minDelta) minDelta = delta;
        }
    }

    if (minDelta == UINT32_MAX) {
        setAlarmIntervalLocked(DEFAULT_IDLE_INTERVAL);
    } else {
        setAlarmIntervalLocked(minDelta);
    }

    portEXIT_CRITICAL_ISR(&timerMux);
}

int16_t easeToward(int16_t current, int16_t target)
{
    float stepF = constrain(smoothingRate * 40.0f, 10.0f, 400.0f);
    int16_t step = static_cast<int16_t>(stepF);
    if (current < target) {
        int16_t next = current + step;
        return next > target ? target : next;
    }
    int16_t next = current - step;
    return next < target ? target : next;
}

void configureDriver(size_t index, const DriverPins &pins)
{
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

    pwmStates[index].periodTicks = computePeriodTicks(PWM_MIN_FREQ);
    pwmStates[index].onTicks = 0;
    pwmStates[index].cycleStart = 0;
    pwmStates[index].nextEvent = DEFAULT_IDLE_INTERVAL;
    pwmStates[index].phase = PHASE_START;
    pwmStates[index].activePin = ACTIVE_NONE;
}

void applyOutput(size_t index, int16_t command)
{
    DriverState &state = drivers[index];
    if (!state.initialised) return;

    float magnitude = fabsf(static_cast<float>(command) / 1000.0f);
    magnitude = constrain(magnitude, 0.0f, 1.0f);
    uint32_t freq = computeFrequency(magnitude);
    uint32_t periodTicks = computePeriodTicks(freq);
    uint32_t onTicks = static_cast<uint32_t>(static_cast<float>(periodTicks) * magnitude);
    if (onTicks > periodTicks) onTicks = periodTicks;
    if (magnitude > 0.0f && onTicks == 0) onTicks = 1;

    uint8_t activePin = ACTIVE_NONE;
    if (command > 0) {
        activePin = ACTIVE_FORWARD;
    } else if (command < 0) {
        activePin = ACTIVE_REVERSE;
    }

    if (state.pins.enablePin >= 0) {
        digitalWrite(state.pins.enablePin,
                     (activePin != ACTIVE_NONE && onTicks > 0) ? HIGH : LOW);
    }

    if (!pwmTimer) {
        setPinLevel(state.pins.forwardPin,
                    (activePin == ACTIVE_FORWARD && onTicks > 0) ? HIGH : LOW);
        setPinLevel(state.pins.reversePin,
                    (activePin == ACTIVE_REVERSE && onTicks > 0) ? HIGH : LOW);
        return;
    }

    portENTER_CRITICAL(&timerMux);
    PwmState &pwm = pwmStates[index];
    pwm.periodTicks = periodTicks;
    pwm.onTicks = (activePin == ACTIVE_NONE) ? 0 : onTicks;
    pwm.phase = PHASE_START;
    pwm.cycleStart = currentTicks;
    pwm.activePin = (activePin == ACTIVE_NONE || onTicks == 0) ? ACTIVE_NONE : activePin;

    setPinLevel(state.pins.forwardPin, LOW);
    setPinLevel(state.pins.reversePin, LOW);

    if (pwm.activePin == ACTIVE_NONE) {
        pwm.nextEvent = currentTicks + periodTicks;
    } else {
        pwm.nextEvent = currentTicks;
        setAlarmIntervalLocked(1);
    }
    portEXIT_CRITICAL(&timerMux);
}

} // namespace

void Outputs::constrainAll()
{
    leftFront = constrain(leftFront, -1000, 1000);
    leftRear = constrain(leftRear, -1000, 1000);
    rightFront = constrain(rightFront, -1000, 1000);
    rightRear = constrain(rightRear, -1000, 1000);
}

bool init(const DriverPins &lf, const DriverPins &lr,
          const DriverPins &rf, const DriverPins &rr)
{
    configureDriver(0, lf);
    configureDriver(1, lr);
    configureDriver(2, rf);
    configureDriver(3, rr);

    currentTicks = 0;
    alarmInterval = DEFAULT_IDLE_INTERVAL;

    pwmTimer = timerBegin(0, 80, true); // 1 MHz
    if (!pwmTimer) {
        return false;
    }
    timerAttachInterrupt(pwmTimer, &onPwmTimer, true);
    setAlarmIntervalLocked(DEFAULT_IDLE_INTERVAL);

    subsystemInitialised = true;
    return true;
}

void calibrate() {}

void stop()
{
    for (size_t i = 0; i < 4; ++i) {
        if (!drivers[i].initialised) continue;
        applyOutput(i, 0);
    }
}

void update(bool enabled, Outputs &current, const Outputs &target)
{
    if (!subsystemInitialised) return;
    Outputs next = current;

    if (!enabled) {
        next.leftFront = next.leftRear = next.rightFront = next.rightRear = 0;
    } else {
        next.leftFront = easeToward(current.leftFront, target.leftFront);
        next.leftRear = easeToward(current.leftRear, target.leftRear);
        next.rightFront = easeToward(current.rightFront, target.rightFront);
        next.rightRear = easeToward(current.rightRear, target.rightRear);
        next.constrainAll();
    }

    applyOutput(0, next.leftFront);
    applyOutput(1, next.leftRear);
    applyOutput(2, next.rightFront);
    applyOutput(3, next.rightRear);

    current = next;
}

void setEasingRate(float rate)
{
    smoothingRate = constrain(rate, 0.5f, 20.0f);
}

} // namespace Motor

