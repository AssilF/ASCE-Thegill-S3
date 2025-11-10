#pragma once

#include <cstdint>

#include "device_config.h"

namespace AnalogInputs {

constexpr uint8_t kMuxChannelCount = config::kAnalogMuxChannelCount;

enum class ChannelMode : uint8_t {
    Disabled = 0,
    Analog,
    Digital,
};

struct ChannelConfig {
    ChannelMode mode = ChannelMode::Analog;
    uint16_t digitalThreshold = 2048; // Raw ADC threshold for digital HIGH
    bool invertDigital = false;
};

bool init();

bool configureChannel(uint8_t channel, const ChannelConfig &config);
ChannelConfig getChannelConfig(uint8_t channel);

bool readMuxAnalog(uint8_t channel, uint16_t &value);
bool readMuxDigital(uint8_t channel, bool &level);

uint16_t readBatteryRaw();
float readBatteryVoltage();

uint16_t readLeakRaw();
float readLeakVoltage();

} // namespace AnalogInputs
