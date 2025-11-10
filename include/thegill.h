#pragma once

#include <Arduino.h>

constexpr uint32_t THEGILL_PACKET_MAGIC = 0x54474C4C; // 'TGLL'

enum class GillMode : uint8_t {
    Default = 0,
    Differential = 1,
};

enum class GillEasing : uint8_t {
    Linear = 0,
    EaseIn,
    EaseOut,
    EaseInOut,
    Sine,
};

constexpr uint8_t GILL_FLAG_BRAKE = 0x01;
constexpr uint8_t GILL_FLAG_HONK  = 0x02;

struct ThegillCommand {
    uint32_t magic;
    int16_t leftFront;
    int16_t leftRear;
    int16_t rightFront;
    int16_t rightRear;
    float easingRate;
    GillMode mode;
    GillEasing easing;
    uint8_t flags;
    uint8_t reserved;
} __attribute__((packed));

struct ThegillTelemetry {
    float targetLeftFront;
    float targetLeftRear;
    float targetRightFront;
    float targetRightRear;
    float actualLeftFront;
    float actualLeftRear;
    float actualRightFront;
    float actualRightRear;
    float easingRate;
    bool brakeActive;
    bool honkActive;
};

float applyEasingCurve(GillEasing mode, float t);

constexpr uint32_t ARM_COMMAND_MAGIC = 0x54474152; // 'TGAR'

namespace ArmCommandMask {
constexpr uint16_t Extension  = 1u << 0;
constexpr uint16_t Shoulder   = 1u << 1;
constexpr uint16_t Elbow      = 1u << 2;
constexpr uint16_t Pitch      = 1u << 3;
constexpr uint16_t Roll       = 1u << 4;
constexpr uint16_t Yaw        = 1u << 5;
constexpr uint16_t Gripper1   = 1u << 6;
constexpr uint16_t Gripper2   = 1u << 7;
constexpr uint16_t AllServos  = Shoulder | Elbow | Pitch | Roll | Yaw;
constexpr uint16_t AllGrippers = Gripper1 | Gripper2;
constexpr uint16_t AllOutputs = Extension | AllServos | AllGrippers;
} // namespace ArmCommandMask

namespace ArmCommandFlag {
constexpr uint8_t EnableOutputs  = 0x01;
constexpr uint8_t DisableOutputs = 0x02;
constexpr uint8_t EnableServos   = 0x04;
constexpr uint8_t DisableServos  = 0x08;
} // namespace ArmCommandFlag

struct ArmControlCommand {
    uint32_t magic;
    float extensionMillimeters;
    float shoulderDegrees;
    float elbowDegrees;
    float pitchDegrees;
    float rollDegrees;
    float yawDegrees;
    float gripper1Degrees;
    float gripper2Degrees;
    uint16_t validMask;
    uint8_t flags;
    uint8_t reserved;
} __attribute__((packed));
