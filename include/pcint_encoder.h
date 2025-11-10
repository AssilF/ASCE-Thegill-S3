#pragma once

#include <cstddef>
#include <cstdint>

#include "device_config.h"

namespace PcintEncoder {

using PinConfig = config::EncoderPinConfig;

struct EncoderReading {
    int32_t total;
    int32_t delta;
    uint32_t lastTransitionMicros;
    bool valid;
};

bool init(const PinConfig *configs, std::size_t count);
bool initDefault();

bool configured();
bool enabled();
std::size_t encoderCount();

EncoderReading read(std::size_t index);
std::size_t readAll(EncoderReading *out, std::size_t maxCount);

int32_t totalCount(std::size_t index);
int32_t consumeDelta(std::size_t index);
void clearDelta(std::size_t index);

void reset(std::size_t index);
void resetAll();

void setEnabled(bool enable);

} // namespace PcintEncoder

