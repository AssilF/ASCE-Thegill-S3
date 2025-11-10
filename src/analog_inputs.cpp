#include "analog_inputs.h"

#include <Arduino.h>
#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#ifdef ARDUINO_ARCH_ESP32
#include "esp_rom_sys.h"
#endif

#include "shift_register.h"

namespace AnalogInputs {
namespace {

constexpr uint32_t kMuxSettleTimeUs = 15;

constexpr std::array<ShiftRegister::Output, 3> kSelectBits = {
    ShiftRegister::Output::AnalogSelect1,
    ShiftRegister::Output::AnalogSelect2,
    ShiftRegister::Output::AnalogSelect3,
};

std::array<ChannelConfig, kMuxChannelCount> gChannelConfigs{};
bool gInitialised = false;
int8_t gSelectedChannel = -1;
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint16_t kDefaultDigitalThreshold =
    static_cast<uint16_t>(1u << (config::kAdcResolutionBits - 1));

inline void settleDelay() {
#ifdef ARDUINO_ARCH_ESP32
    esp_rom_delay_us(kMuxSettleTimeUs);
#else
    delayMicroseconds(kMuxSettleTimeUs);
#endif
}

void configureAnalogPin(uint8_t pin) {
    pinMode(pin, INPUT);
#ifdef ARDUINO_ARCH_ESP32
    adcAttachPin(pin);
    analogSetPinAttenuation(pin, ADC_11db);
#endif
}

void resetSelectBits() {
    if (!ShiftRegister::initialized()) {
        return;
    }
    for (auto bit : kSelectBits) {
        ShiftRegister::writeChannel(bit, false);
    }
    gSelectedChannel = -1;
}

bool selectChannelLocked(uint8_t channel) {
    if (channel >= kMuxChannelCount) {
        return false;
    }
    if (!ShiftRegister::initialized()) {
        return false;
    }
    if (gSelectedChannel == static_cast<int8_t>(channel)) {
        return true;
    }

    for (uint8_t bit = 0; bit < kSelectBits.size(); ++bit) {
        bool level = ((channel >> bit) & 0x1u) != 0;
        ShiftRegister::writeChannel(kSelectBits[bit], level);
    }
    gSelectedChannel = static_cast<int8_t>(channel);
    return true;
}

uint16_t readMuxAdcLocked() {
    return static_cast<uint16_t>(analogRead(config::kAnalogMultiplexerInputPin));
}

float rawToVoltage(uint16_t raw, float ratio) {
    constexpr float maxCounts =
        static_cast<float>((1u << config::kAdcResolutionBits) - 1u);
    if (maxCounts <= 0.0f) {
        return 0.0f;
    }
    float normalized = static_cast<float>(raw) / maxCounts;
    return normalized * config::kAdcReferenceVoltage * ratio;
}

} // namespace

bool init() {
    configureAnalogPin(config::kAnalogMultiplexerInputPin);
    configureAnalogPin(config::kBatterySensePin);
    configureAnalogPin(config::kLeakSensePin);

    for (auto &cfg : gChannelConfigs) {
        cfg = ChannelConfig{};
    }
    gInitialised = true;
    gSelectedChannel = -1;
    resetSelectBits();
    return true;
}

bool configureChannel(uint8_t channel, const ChannelConfig &config) {
    if (channel >= kMuxChannelCount) {
        return false;
    }
    gChannelConfigs[channel] = config;
    return true;
}

ChannelConfig getChannelConfig(uint8_t channel) {
    if (channel >= kMuxChannelCount) {
        return ChannelConfig{};
    }
    return gChannelConfigs[channel];
}

bool readMuxAnalog(uint8_t channel, uint16_t &value) {
    if (!gInitialised || channel >= kMuxChannelCount) {
        return false;
    }
    if (gChannelConfigs[channel].mode == ChannelMode::Disabled) {
        return false;
    }

    bool success = false;
    portENTER_CRITICAL(&gMux);
    if (selectChannelLocked(channel)) {
        settleDelay();
        value = readMuxAdcLocked();
        success = true;
    }
    portEXIT_CRITICAL(&gMux);
    return success;
}

bool readMuxDigital(uint8_t channel, bool &level) {
    if (!gInitialised || channel >= kMuxChannelCount) {
        return false;
    }
    const ChannelConfig cfg = gChannelConfigs[channel];
    if (cfg.mode != ChannelMode::Digital) {
        return false;
    }

    uint16_t sample = 0;
    if (!readMuxAnalog(channel, sample)) {
        return false;
    }
    uint16_t threshold = cfg.digitalThreshold != 0 ? cfg.digitalThreshold
                                                   : kDefaultDigitalThreshold;
    bool value = sample >= threshold;
    if (cfg.invertDigital) {
        value = !value;
    }
    level = value;
    return true;
}

uint16_t readBatteryRaw() {
    if (!gInitialised) {
        return 0;
    }
    return static_cast<uint16_t>(analogRead(config::kBatterySensePin));
}

float readBatteryVoltage() {
    return rawToVoltage(readBatteryRaw(), config::kBatteryDividerRatio);
}

uint16_t readLeakRaw() {
    if (!gInitialised) {
        return 0;
    }
    return static_cast<uint16_t>(analogRead(config::kLeakSensePin));
}

float readLeakVoltage() {
    return rawToVoltage(readLeakRaw(), config::kLeakDividerRatio);
}

} // namespace AnalogInputs
