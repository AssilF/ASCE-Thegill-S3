#pragma once

#include <cstddef>
#include <cstdint>

namespace ShiftRegister {

constexpr std::size_t kRegisterCount = 4;
constexpr std::size_t kOutputsPerRegister = 8;
constexpr std::size_t kTotalOutputs = kRegisterCount * kOutputsPerRegister;
constexpr uint8_t kUserOutputBase = 24;

struct Pins {
    uint8_t data;
    uint8_t clock;
    uint8_t latch;
};

enum class Output : uint8_t {
    PumpControl = 0,
    HighPowerLed3 = 1,
    HighPowerLed2 = 2,
    HighPowerLed1 = 3,
    AnalogSelect3 = 4,
    AnalogSelect2 = 5,
    AnalogSelect1 = 6,
    SystemIndicatorLed = 7,

    GripperIndicatorLed = 8,
    GripperPwmA = 9,
    GripperAIn2 = 10,
    GripperAIn1 = 11,
    GripperStandby = 12,
    GripperBIn1 = 13,
    GripperBIn2 = 14,
    GripperPwmB = 15,

    ArmIndicatorLed = 16,
    RotationPwmA = 17,
    RotationAIn2 = 18,
    RotationAIn1 = 19,
    ArmStandby = 20,
    ExtensionBIn1 = 21,
    ExtensionBIn2 = 22,
    ExtensionPwmB = 23,

    User0 = kUserOutputBase + 0,
    User1 = kUserOutputBase + 1,
    User2 = kUserOutputBase + 2,
    User3 = kUserOutputBase + 3,
    User4 = kUserOutputBase + 4,
    User5 = kUserOutputBase + 5,
    User6 = kUserOutputBase + 6,
    User7 = kUserOutputBase + 7,
};

constexpr uint8_t outputIndex(Output output) {
    return static_cast<uint8_t>(output);
}

bool init(const Pins &pins,
          uint32_t pwmFrequencyHz = 100000,
          uint8_t pwmResolutionBits = 8);

bool initialized();

void writeChannel(uint8_t index, bool high);
inline void writeChannel(Output output, bool high) {
    writeChannel(outputIndex(output), high);
}
inline void SRWrite(uint8_t index, bool high) {
    writeChannel(index, high);
}
inline void SRWrite(Output output, bool high) {
    writeChannel(output, high);
}

void writeChannelPwm(uint8_t index, uint8_t duty);
inline void writeChannelPwm(Output output, uint8_t duty) {
    writeChannelPwm(outputIndex(output), duty);
}
inline void SRWritePwm(uint8_t index, uint8_t duty) {
    writeChannelPwm(index, duty);
}
inline void SRWritePwm(Output output, uint8_t duty) {
    writeChannelPwm(output, duty);
}

void disableChannelPwm(uint8_t index);
inline void disableChannelPwm(Output output) {
    disableChannelPwm(outputIndex(output));
}
inline void SRDisablePwm(uint8_t index) {
    disableChannelPwm(index);
}
inline void SRDisablePwm(Output output) {
    disableChannelPwm(output);
}

uint8_t channelPwmDuty(uint8_t index);
inline uint8_t channelPwmDuty(Output output) {
    return channelPwmDuty(outputIndex(output));
}
inline uint8_t SRPwmDuty(uint8_t index) {
    return channelPwmDuty(index);
}
inline uint8_t SRPwmDuty(Output output) {
    return channelPwmDuty(outputIndex(output));
}

void writeUserMask(uint8_t value, uint8_t mask = 0xFF);
uint8_t readUserMask();

uint32_t frameState();
void clearAll();

} // namespace ShiftRegister
