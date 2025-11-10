#include "shift_register.h"

#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>
#include <cstring>

namespace ShiftRegister {
namespace {

// State
Pins gPins{};
SPIClass *gSpi = nullptr;         // Dedicated HSPI bus instance
esp_timer_handle_t gTimer = nullptr; // Refresh timer
bool gInitialised = false;

// PWM model
volatile uint32_t gStaticBits = 0;                 // Outputs that are statically HIGH
volatile uint32_t gPwmMask = 0;                    // Bitmask of outputs driven by PWM
volatile uint8_t gPwmDuty[kTotalOutputs] = {0};    // Duty in steps (0..gResolutionSteps-1)
volatile uint16_t gPhase = 0;                      // Current PWM phase
uint8_t gResolutionBits = 8;                       // Default 8-bit phase resolution
uint16_t gResolutionSteps = 1 << 8;                // 256 steps

// Frame buffering and synchronization
volatile uint32_t gFrameShadow = 0;                // Last composed frame we attempted to send
uint8_t gTxBuffers[2][kRegisterCount] = {{0}};     // Two 4-byte buffers (double-buffer)
volatile uint8_t gActiveTx = 0;                    // Index of buffer last filled
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;  // Protects shared state

// Latch pulse via GPIO (fast) after SPI shift
struct GpioRegs {
    volatile uint32_t *setReg{nullptr};
    volatile uint32_t *clrReg{nullptr};
    uint32_t mask{0};
};
GpioRegs gLatchRegs{};

inline bool configureFastGpio(uint8_t pin, GpioRegs &out) {
    // ESP32 family: choose correct W1TS/W1TC region
    if (pin < 32) {
        out.mask = (1u << pin);
        out.setReg = (volatile uint32_t *)GPIO_OUT_W1TS_REG;
        out.clrReg = (volatile uint32_t *)GPIO_OUT_W1TC_REG;
        return true;
    }
#if SOC_GPIO_PIN_COUNT > 32
    out.mask = (1u << (pin - 32));
    out.setReg = (volatile uint32_t *)GPIO_OUT1_W1TS_REG;
    out.clrReg = (volatile uint32_t *)GPIO_OUT1_W1TC_REG;
    return true;
#else
    (void)pin; (void)out; return false;
#endif
}

inline void IRAM_ATTR latchPulse() {
    if (!gLatchRegs.setReg) return;
    *gLatchRegs.setReg = gLatchRegs.mask;
    *gLatchRegs.clrReg = gLatchRegs.mask;
}

inline uint8_t scaleDutyToSteps(uint8_t duty) {
    if (gResolutionSteps <= 1) {
        return duty > 0 ? 1 : 0;
    }
    const uint16_t maxStep = gResolutionSteps - 1;
    const uint32_t scaled = (static_cast<uint32_t>(duty) * maxStep + 127) / 255;
    return static_cast<uint8_t>(scaled > maxStep ? maxStep : scaled);
}

inline uint8_t scaleStepsToDuty(uint8_t steps) {
    if (gResolutionSteps <= 1) {
        return steps ? 255 : 0;
    }
    const uint16_t maxStep = gResolutionSteps - 1;
    return static_cast<uint8_t>((static_cast<uint32_t>(steps) * 255 + maxStep / 2) / maxStep);
}

inline uint32_t composeFrame(uint16_t phase) {
    uint32_t frame = gStaticBits;
    const uint32_t pwmMask = gPwmMask;
    if (pwmMask == 0) {
        return frame;
    }

    for (uint8_t i = 0; i < kTotalOutputs; ++i) {
        const uint32_t bit = (1u << i);
        if ((pwmMask & bit) == 0) continue;
        if (gPwmDuty[i] > phase) {
            frame |= bit;
        } else {
            frame &= ~bit;
        }
    }
    return frame;
}

inline void frameToBytes(uint32_t frame, uint8_t out[kRegisterCount]) {
    // Maintain previous ordering: send MSB byte first (register k-1 .. 0)
    out[0] = static_cast<uint8_t>((frame >> 24) & 0xFF);
    out[1] = static_cast<uint8_t>((frame >> 16) & 0xFF);
    out[2] = static_cast<uint8_t>((frame >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>((frame >> 0) & 0xFF);
}

inline void spiWriteBytes(const uint8_t *data, size_t len) {
    if (!gSpi) return;
    gSpi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); // 10 MHz
    gSpi->writeBytes(const_cast<uint8_t*>(data), len);
    gSpi->endTransaction();
}

void refreshOnce() {
    // Compute phase and frame under lock
    uint32_t frame;
    portENTER_CRITICAL(&gMux);
    uint16_t next = gPhase + 1;
    if (next >= gResolutionSteps) next = 0;
    gPhase = next;
    frame = composeFrame(gPhase);
    gFrameShadow = frame;

    // Double buffer pack
    uint8_t nextBuf = gActiveTx ^ 1;
    frameToBytes(frame, gTxBuffers[nextBuf]);
    gActiveTx = nextBuf;
    portEXIT_CRITICAL(&gMux);

    // Transmit and latch
    spiWriteBytes(gTxBuffers[gActiveTx], kRegisterCount);
    latchPulse();
}

void IRAM_ATTR timerCallback(void *arg) {
    (void)arg;
    // Runs in esp_timer task context; keep it brief
    refreshOnce();
}

void clampResolution(uint8_t &bits) {
    if (bits < 1) bits = 1;
    if (bits > 8) bits = 8; // 8-bit is plenty for LED dimming at 8 kHz update
}

} // namespace

bool init(const Pins &pins, uint32_t updateFrequencyHz, uint8_t pwmResolutionBits) {
    if (gInitialised) {
        return true;
    }

    clampResolution(pwmResolutionBits);
    gResolutionBits = pwmResolutionBits;
    gResolutionSteps = static_cast<uint16_t>(1u << gResolutionBits);

    gPins = pins;
    pinMode(pins.data, OUTPUT);
    pinMode(pins.clock, OUTPUT);
    pinMode(pins.latch, OUTPUT);
    digitalWrite(pins.data, LOW);
    digitalWrite(pins.clock, LOW);
    digitalWrite(pins.latch, LOW);
    configureFastGpio(pins.latch, gLatchRegs);

    // Dedicated HSPI instance with given pins (MISO unused)
#if defined(HSPI)
    gSpi = new SPIClass(HSPI);
#else
    gSpi = new SPIClass(VSPI);
#endif
    if (!gSpi) return false;
    gSpi->begin(pins.clock, -1 /*MISO*/, pins.data, -1 /*SS unused*/);

    // Clear state
    portENTER_CRITICAL(&gMux);
    gStaticBits = 0;
    gPwmMask = 0;
    std::memset((void*)gPwmDuty, 0, sizeof(gPwmDuty));
    gPhase = 0;
    gFrameShadow = 0;
    gActiveTx = 0;
    frameToBytes(0, gTxBuffers[0]);
    frameToBytes(0, gTxBuffers[1]);
    portEXIT_CRITICAL(&gMux);

    // Prime outputs to zero
    spiWriteBytes(gTxBuffers[gActiveTx], kRegisterCount);
    latchPulse();

    // Setup refresh timer
    if (updateFrequencyHz == 0) updateFrequencyHz = 8000; // default 8 kHz
    const uint64_t period_us = 1000000ull / updateFrequencyHz;
    esp_timer_create_args_t args{};
    args.callback = &timerCallback;
    args.arg = nullptr;
    args.dispatch_method = ESP_TIMER_TASK; // runs in timer task context
    args.name = "sr_refresh";
    if (esp_timer_create(&args, &gTimer) != ESP_OK) {
        return false;
    }
    if (esp_timer_start_periodic(gTimer, period_us) != ESP_OK) {
        return false;
    }

    gInitialised = true;
    return true;
}

bool initialized() {
    return gInitialised;
}

void writeChannel(uint8_t index, bool high) {
    if (!gInitialised || index >= kTotalOutputs) return;
    const uint32_t bit = (1u << index);
    portENTER_CRITICAL(&gMux);
    gPwmMask &= ~bit; // disable PWM for this channel
    if (high) {
        gStaticBits |= bit;
        gPwmDuty[index] = gResolutionSteps > 1 ? static_cast<uint8_t>(gResolutionSteps - 1) : 1;
    } else {
        gStaticBits &= ~bit;
        gPwmDuty[index] = 0;
    }
    // Compose a new frame now to reduce latency
    gFrameShadow = composeFrame(gPhase);
    frameToBytes(gFrameShadow, gTxBuffers[gActiveTx]);
    portEXIT_CRITICAL(&gMux);
}

void writeChannelPwm(uint8_t index, uint8_t duty) {
    if (!gInitialised || index >= kTotalOutputs) return;
    const uint32_t bit = (1u << index);
    portENTER_CRITICAL(&gMux);
    const uint8_t scaled = scaleDutyToSteps(duty);
    const uint16_t maxStep = gResolutionSteps > 0 ? (gResolutionSteps - 1) : 0;
    if (scaled == 0) {
        gPwmMask &= ~bit;
        gStaticBits &= ~bit;
        gPwmDuty[index] = 0;
    } else if (maxStep > 0 && scaled >= maxStep) {
        gPwmMask &= ~bit;
        gStaticBits |= bit;
        gPwmDuty[index] = static_cast<uint8_t>(maxStep);
    } else {
        gPwmMask |= bit;
        gStaticBits &= ~bit;
        gPwmDuty[index] = scaled;
    }
    portEXIT_CRITICAL(&gMux);
}

void disableChannelPwm(uint8_t index) {
    if (!gInitialised || index >= kTotalOutputs) return;
    const uint32_t bit = (1u << index);
    portENTER_CRITICAL(&gMux);
    gPwmMask &= ~bit;
    gStaticBits &= ~bit;
    gPwmDuty[index] = 0;
    portEXIT_CRITICAL(&gMux);
}

uint8_t channelPwmDuty(uint8_t index) {
    if (!gInitialised || index >= kTotalOutputs) return 0;
    portENTER_CRITICAL(&gMux);
    const uint8_t steps = gPwmDuty[index];
    const bool usesPwm = (gPwmMask & (1u << index)) != 0;
    const bool staticHigh = (gStaticBits & (1u << index)) != 0;
    portEXIT_CRITICAL(&gMux);
    if (!usesPwm) return staticHigh ? 255 : 0;
    return scaleStepsToDuty(steps);
}

void writeUserMask(uint8_t value, uint8_t mask) {
    if (!gInitialised) return;
    portENTER_CRITICAL(&gMux);
    for (uint8_t i = 0; i < 8; ++i) {
        if ((mask & (1u << i)) == 0) continue;
        const uint8_t idx = kUserOutputBase + i;
        const uint32_t bit = (1u << idx);
        gPwmMask &= ~bit;
        if (value & (1u << i)) {
            gStaticBits |= bit;
            gPwmDuty[idx] = gResolutionSteps > 1 ? static_cast<uint8_t>(gResolutionSteps - 1) : 1;
        } else {
            gStaticBits &= ~bit;
            gPwmDuty[idx] = 0;
        }
    }
    portEXIT_CRITICAL(&gMux);
}

uint8_t readUserMask() {
    if (!gInitialised) return 0;
    portENTER_CRITICAL(&gMux);
    uint32_t frame = gFrameShadow;
    portEXIT_CRITICAL(&gMux);
    frame >>= kUserOutputBase;
    return static_cast<uint8_t>(frame & 0xFFu);
}

uint32_t frameState() {
    if (!gInitialised) return 0;
    portENTER_CRITICAL(&gMux);
    uint32_t frame = gFrameShadow;
    portEXIT_CRITICAL(&gMux);
    return frame;
}

void clearAll() {
    // Immediately force all outputs low and stop PWM masking
    portENTER_CRITICAL(&gMux);
    gStaticBits = 0;
    gPwmMask = 0;
    std::memset((void*)gPwmDuty, 0, sizeof(gPwmDuty));
    gPhase = 0;
    gFrameShadow = 0;
    frameToBytes(0, gTxBuffers[0]);
    frameToBytes(0, gTxBuffers[1]);
    portEXIT_CRITICAL(&gMux);

    // Push zeros right now if possible
    spiWriteBytes(gTxBuffers[0], kRegisterCount);
    latchPulse();
}

} // namespace ShiftRegister

