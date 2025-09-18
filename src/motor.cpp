#include "motor.h"
#include <math.h>

namespace Motor {
namespace {
struct DriverState {
    DriverPins pins;
    uint8_t forwardChannel;
    uint8_t reverseChannel;
    uint8_t timer;
    bool initialised;
};

static DriverState drivers[4];
static bool subsystemInitialised = false;
static float smoothingRate = 4.0f;

constexpr uint32_t LEDC_BASE_FREQ = 20000;
constexpr uint32_t LEDC_MIN_FREQ = 400;
constexpr uint8_t LEDC_RESOLUTION = 12; // 12-bit resolution (0-4095)

uint32_t computeFrequency(float magnitude)
{
    magnitude = constrain(magnitude, 0.0f, 1.0f);
    return LEDC_MIN_FREQ + static_cast<uint32_t>((LEDC_BASE_FREQ - LEDC_MIN_FREQ) * magnitude);
}

uint32_t computeDuty(float magnitude)
{
    magnitude = constrain(magnitude, 0.0f, 1.0f);
    const uint32_t maxDuty = (1u << LEDC_RESOLUTION) - 1u;
    return static_cast<uint32_t>(maxDuty * magnitude);
}

void applyOutput(DriverState &state, int16_t command)
{
    float magnitude = fabsf(static_cast<float>(command) / 1000.0f);
    uint32_t freq = computeFrequency(magnitude);
    ledcChangeFrequency(state.timer, freq, LEDC_RESOLUTION);

    uint32_t duty = computeDuty(magnitude);
    if (command >= 0) {
        ledcWrite(state.forwardChannel, duty);
        ledcWrite(state.reverseChannel, 0);
    } else {
        ledcWrite(state.forwardChannel, 0);
        ledcWrite(state.reverseChannel, duty);
    }

    if (state.pins.enablePin >= 0) {
        digitalWrite(state.pins.enablePin, duty > 0 ? HIGH : LOW);
    }
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

void configureDriver(DriverState &state, const DriverPins &pins,
                     uint8_t forwardChannel, uint8_t reverseChannel, uint8_t timer)
{
    state.pins = pins;
    state.forwardChannel = forwardChannel;
    state.reverseChannel = reverseChannel;
    state.timer = timer;
    state.initialised = true;

    if (pins.enablePin >= 0) {
        pinMode(pins.enablePin, OUTPUT);
        digitalWrite(pins.enablePin, LOW);
    }

    ledcSetup(timer, LEDC_BASE_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(pins.forwardPin, forwardChannel);
    ledcAttachPin(pins.reversePin, reverseChannel);
    ledcWrite(forwardChannel, 0);
    ledcWrite(reverseChannel, 0);
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
    configureDriver(drivers[0], lf, 0, 1, 0);
    configureDriver(drivers[1], lr, 2, 3, 1);
    configureDriver(drivers[2], rf, 4, 5, 2);
    configureDriver(drivers[3], rr, 6, 7, 3);

    subsystemInitialised = true;
    return true;
}

void calibrate() {}

void stop()
{
    for (auto &driver : drivers) {
        if (!driver.initialised) continue;
        ledcWrite(driver.forwardChannel, 0);
        ledcWrite(driver.reverseChannel, 0);
        if (driver.pins.enablePin >= 0) {
            digitalWrite(driver.pins.enablePin, LOW);
        }
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

    applyOutput(drivers[0], next.leftFront);
    applyOutput(drivers[1], next.leftRear);
    applyOutput(drivers[2], next.rightFront);
    applyOutput(drivers[3], next.rightRear);

    current = next;
}

void setEasingRate(float rate)
{
    smoothingRate = constrain(rate, 0.5f, 20.0f);
}

} // namespace Motor

