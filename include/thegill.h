#pragma once

#include <Arduino.h>

constexpr uint32_t THEGILL_PACKET_MAGIC = 0x54474C4C; // 'TGLL'

constexpr uint8_t GILL_FLAG_BRAKE = 0x01;
constexpr uint8_t GILL_FLAG_HONK  = 0x02;

namespace GillSystemCommand {
constexpr uint8_t EnableTelemetry = 1u << 0;
constexpr uint8_t DisableTelemetry = 1u << 1;
constexpr uint8_t EnableBuzzer = 1u << 2;
constexpr uint8_t DisableBuzzer = 1u << 3;
constexpr uint8_t RequestStatus = 1u << 4;
constexpr uint8_t RequestArmState = 1u << 5;
} // namespace GillSystemCommand

struct ThegillCommand {
    uint32_t magic;
    int16_t leftFront;
    int16_t leftRear;
    int16_t rightFront;
    int16_t rightRear;
    uint8_t flags;
    uint8_t system;
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
    bool brakeActive;
    bool honkActive;
};

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
constexpr uint16_t Base       = 1u << 8;
constexpr uint16_t AllServos  = Shoulder | Elbow | Pitch | Roll | Yaw;
constexpr uint16_t AllGrippers = Gripper1 | Gripper2;
constexpr uint16_t AllOutputs = Extension | Base | AllServos | AllGrippers;
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
    float baseDegrees;
    float shoulderDegrees;
    float elbowDegrees;
    float pitchDegrees;
    float rollDegrees;
    float yawDegrees;
    float gripper1Degrees;
    float gripper2Degrees;
    uint16_t validMask;
    uint8_t flags;
} __attribute__((packed));

constexpr size_t THEGILL_ARM_SERVO_COUNT = 5;

enum class DriveEasingMode : uint8_t {
    None = 0,
    SlewRate = 1,
    Exponential = 2,
};

namespace ConfigFlag {
constexpr uint8_t MuteAudio        = 1u << 0;
constexpr uint8_t DriveEnabled     = 1u << 1;
constexpr uint8_t ArmOutputsEnable = 1u << 2;
constexpr uint8_t FailsafeEnable   = 1u << 3;
} // namespace ConfigFlag

namespace SafetyFlag {
constexpr uint8_t BatterySafetyEnable = 1u << 0;
} // namespace SafetyFlag

constexpr uint32_t THEGILL_CONFIG_MAGIC = 0x54474346;    // 'TGCF'
constexpr uint32_t THEGILL_SETTINGS_MAGIC = 0x54475350;  // 'TGSP'
constexpr uint32_t THEGILL_ARM_STATE_MAGIC = 0x54474153; // 'TGAS'

struct ConfigurationPacket {
    uint32_t magic;
    uint8_t easingMode;      // DriveEasingMode
    uint8_t easingRate;      // 0 disables, units depend on mode
    uint8_t controlFlags;    // ConfigFlag bits
    uint8_t safetyFlags;     // SafetyFlag bits
    uint16_t batteryCutoffMillivolts;
    uint16_t batteryRecoverMillivolts;
} __attribute__((packed));

struct SettingsPacket {
    uint32_t magic;
    char robotName[16];
    char customId[32];
    char wifiSsid[32];
    char wifiPassword[32];
} __attribute__((packed));

struct ArmStatePacket {
    uint32_t magic;
    float baseDegrees;
    float extensionCentimeters;
    float servoDegrees[THEGILL_ARM_SERVO_COUNT];
    uint8_t servoEnabledMask;
    uint8_t servoAttachedMask;
    uint8_t flags; // bit0 Arm outputs enabled, bit1 servos enabled
} __attribute__((packed));
