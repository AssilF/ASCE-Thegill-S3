#include "pcint_encoder.h"

#include <Arduino.h>
#include <algorithm>
#include <driver/gpio.h>

namespace PcintEncoder {
namespace {

constexpr std::size_t kMaxEncoders = config::kEncoderCount;
static_assert(kMaxEncoders > 0, "Encoder array must be non-zero");

struct EncoderRuntime {
    PinConfig config{};
    volatile int32_t total{0};
    volatile int32_t delta{0};
    volatile uint32_t lastTransitionMicros{0};
    volatile uint8_t lastState{0};
    bool attached{false};
};

struct ChannelContext {
    uint8_t encoderIndex{0};
};

inline bool hasChannelA(const PinConfig &cfg) {
    return cfg.channelA != config::kInvalidPin;
}

inline bool hasChannelB(const PinConfig &cfg) {
    return cfg.channelB != config::kInvalidPin;
}

inline uint8_t stateMask(const PinConfig &cfg) {
    return hasChannelB(cfg) ? 0x03 : 0x01;
}

EncoderRuntime g_encoders[kMaxEncoders];
ChannelContext g_channelContexts[kMaxEncoders * 2];
std::size_t g_encoderCount = 0;
bool g_configured = false;
volatile bool g_enabled = false;
portMUX_TYPE g_isrMux = portMUX_INITIALIZER_UNLOCKED;

constexpr int8_t kTransitionTable[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
   -1, 0, 0, 1,
    0, 1, -1, 0,
};

inline uint8_t readState(const PinConfig &cfg) {
    uint8_t state = 0;
    if (hasChannelA(cfg)) {
        state |= gpio_get_level(static_cast<gpio_num_t>(cfg.channelA)) ? 0x01 : 0x00;
    }
    if (hasChannelB(cfg)) {
        state |= gpio_get_level(static_cast<gpio_num_t>(cfg.channelB)) ? 0x02 : 0x00;
    }
    return state & stateMask(cfg);
}

inline void configurePin(int pin, bool pullup) {
    if (pin == config::kInvalidPin) {
        return;
    }
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
}

inline void detachPin(int pin) {
    if (pin == config::kInvalidPin) {
        return;
    }
    detachInterrupt(pin);
}

inline void detachAll() {
    for (std::size_t i = 0; i < kMaxEncoders; ++i) {
        EncoderRuntime &rt = g_encoders[i];
        if (!rt.attached) {
            continue;
        }
        if (hasChannelA(rt.config)) {
            detachPin(rt.config.channelA);
        }
        if (hasChannelB(rt.config) && rt.config.channelB != rt.config.channelA) {
            detachPin(rt.config.channelB);
        }
        rt.attached = false;
    }
}

inline uint32_t IRAM_ATTR currentMicros() {
    return static_cast<uint32_t>(micros());
}

inline void IRAM_ATTR applyDelta(uint8_t index, int8_t delta, uint8_t newState) {
    EncoderRuntime &rt = g_encoders[index];
    portENTER_CRITICAL_ISR(&g_isrMux);
    rt.total += delta;
    rt.delta += delta;
    rt.lastState = newState & 0x03;
    rt.lastTransitionMicros = currentMicros();
    portEXIT_CRITICAL_ISR(&g_isrMux);
}

inline void IRAM_ATTR updateSingleChannel(uint8_t index, uint8_t newState) {
    EncoderRuntime &rt = g_encoders[index];
    uint8_t previous = rt.lastState & 0x01;
    if (newState == previous) {
        return;
    }
    if (previous == 0 && newState == 1) {
        int8_t delta = rt.config.inverted ? -1 : 1;
        applyDelta(index, delta, newState & 0x01);
        return;
    }
    // Falling edge: update stored state without accumulating delta
    rt.lastState = newState & 0x01;
}

inline void IRAM_ATTR updateFromState(uint8_t index, uint8_t rawState) {
    EncoderRuntime &rt = g_encoders[index];
    uint8_t maskedState = rawState & stateMask(rt.config);
    if (!hasChannelB(rt.config)) {
        updateSingleChannel(index, maskedState & 0x01);
        return;
    }
    uint8_t previous = rt.lastState & 0x03;
    if (maskedState == previous) {
        return;
    }
    int8_t delta = kTransitionTable[(previous << 2) | (maskedState & 0x03)];
    if (delta == 0) {
        rt.lastState = maskedState & 0x03;
        return;
    }
    if (rt.config.inverted) {
        delta = -delta;
    }
    applyDelta(index, delta, maskedState);
}

void IRAM_ATTR handleChannelEdge(void *arg) {
    auto *ctx = static_cast<ChannelContext *>(arg);
    if (!ctx || ctx->encoderIndex >= g_encoderCount) {
        return;
    }
    if (!g_enabled) {
        return;
    }
    EncoderRuntime &rt = g_encoders[ctx->encoderIndex];
    if (!rt.attached) {
        return;
    }
    uint8_t state = readState(rt.config);
    updateFromState(ctx->encoderIndex, state);
}

} // namespace

bool init(const PinConfig *configs, std::size_t count) {
    detachAll();
    g_encoderCount = 0;
    g_configured = false;
    g_enabled = false;

    if (!configs || count == 0) {
        return false;
    }

    const std::size_t limit = std::min(count, kMaxEncoders);
    std::size_t attachedCount = 0;

    for (std::size_t i = 0; i < limit; ++i) {
        EncoderRuntime &rt = g_encoders[i];
        rt.config = configs[i];
        rt.total = 0;
        rt.delta = 0;
        rt.lastTransitionMicros = 0;
        rt.lastState = 0;
        rt.attached = false;

        bool hasA = hasChannelA(rt.config);
        bool hasB = hasChannelB(rt.config);
        if (!hasA) {
            continue;
        }

        configurePin(rt.config.channelA, rt.config.usePullup);
        if (hasB && rt.config.channelB != rt.config.channelA) {
            configurePin(rt.config.channelB, rt.config.usePullup);
        }

        rt.lastState = readState(rt.config);
        g_channelContexts[i * 2].encoderIndex = static_cast<uint8_t>(i);
        g_channelContexts[i * 2 + 1].encoderIndex = static_cast<uint8_t>(i);

        attachInterruptArg(rt.config.channelA, handleChannelEdge,
                           &g_channelContexts[i * 2], CHANGE);
        if (hasB && rt.config.channelB != rt.config.channelA) {
            attachInterruptArg(rt.config.channelB, handleChannelEdge,
                               &g_channelContexts[i * 2 + 1], CHANGE);
        }

        rt.attached = true;
        ++attachedCount;
    }

    g_encoderCount = limit;
    g_configured = true;
    g_enabled = attachedCount > 0;
    return attachedCount > 0;
}

bool initDefault() {
    return init(config::kEncoderPins, config::kEncoderCount);
}

bool configured() {
    return g_configured;
}

bool enabled() {
    return g_enabled && g_configured;
}

std::size_t encoderCount() {
    return g_encoderCount;
}

void setEnabled(bool enable) {
    g_enabled = enable && g_configured;
}

EncoderReading read(std::size_t index) {
    EncoderReading reading{0, 0, 0, false};
    if (index >= g_encoderCount) {
        return reading;
    }

    portENTER_CRITICAL(&g_isrMux);
    reading.total = g_encoders[index].total;
    reading.delta = g_encoders[index].delta;
    reading.lastTransitionMicros = g_encoders[index].lastTransitionMicros;
    reading.valid = g_encoders[index].attached;
    portEXIT_CRITICAL(&g_isrMux);
    return reading;
}

std::size_t readAll(EncoderReading *out, std::size_t maxCount) {
    if (!out || maxCount == 0) {
        return 0;
    }
    std::size_t limit = std::min(g_encoderCount, maxCount);

    portENTER_CRITICAL(&g_isrMux);
    for (std::size_t i = 0; i < limit; ++i) {
        out[i].total = g_encoders[i].total;
        out[i].delta = g_encoders[i].delta;
        out[i].lastTransitionMicros = g_encoders[i].lastTransitionMicros;
        out[i].valid = g_encoders[i].attached;
    }
    portEXIT_CRITICAL(&g_isrMux);
    return limit;
}

int32_t totalCount(std::size_t index) {
    if (index >= g_encoderCount) {
        return 0;
    }
    portENTER_CRITICAL(&g_isrMux);
    int32_t value = g_encoders[index].total;
    portEXIT_CRITICAL(&g_isrMux);
    return value;
}

int32_t consumeDelta(std::size_t index) {
    if (index >= g_encoderCount) {
        return 0;
    }
    portENTER_CRITICAL(&g_isrMux);
    int32_t value = g_encoders[index].delta;
    g_encoders[index].delta = 0;
    portEXIT_CRITICAL(&g_isrMux);
    return value;
}

void clearDelta(std::size_t index) {
    if (index >= g_encoderCount) {
        return;
    }
    portENTER_CRITICAL(&g_isrMux);
    g_encoders[index].delta = 0;
    portEXIT_CRITICAL(&g_isrMux);
}

void reset(std::size_t index) {
    if (index >= g_encoderCount) {
        return;
    }

    EncoderRuntime &rt = g_encoders[index];
    uint8_t state = rt.attached ? readState(rt.config) : 0;

    portENTER_CRITICAL(&g_isrMux);
    rt.total = 0;
    rt.delta = 0;
    rt.lastTransitionMicros = rt.attached ? currentMicros() : 0;
    rt.lastState = state;
    portEXIT_CRITICAL(&g_isrMux);
}

void resetAll() {
    for (std::size_t i = 0; i < g_encoderCount; ++i) {
        reset(i);
    }
}

} // namespace PcintEncoder
