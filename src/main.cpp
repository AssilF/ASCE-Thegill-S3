#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "analog_inputs.h"
#include "comms.h"
#include "commands.h"
#include "thegill.h"
#include "motor.h"
#include "device_config.h"
#include "shift_register.h"
#include "pcint_encoder.h"
#include "arm_control.h"
#include "arm_servos.h"

// ==================== BOARD CONFIGURATION ====================
// ESP32-S3 pin mappings for BTS7960 bridge drivers and task sizes
const Motor::DriverPins PINS_LEFT_FRONT  = {19, 20, -1};
const Motor::DriverPins PINS_LEFT_REAR   = {5, 4, -1};
const Motor::DriverPins PINS_RIGHT_FRONT = {18, 17, -1};
const Motor::DriverPins PINS_RIGHT_REAR  = {7, 6, -1};
const int BUZZER_PIN = config::kBuzzerPin; // optional piezo buzzer
const int BUZZER_CHANNEL = config::kBuzzerChannel;
const ShiftRegister::Pins SHIFT_REG_PINS = {
    config::kShiftRegisterDataPin,
    config::kShiftRegisterClockPin,
    config::kShiftRegisterLatchPin
};
const uint32_t CPU_FREQ_MHZ = 240;
const uint16_t FAST_TASK_STACK = 4096;
const uint16_t COMM_TASK_STACK = 8192;
const uint16_t FAILSAFE_TASK_STACK = 2048;
const uint16_t TELEMETRY_TASK_STACK = 4096;
const uint16_t OTA_TASK_STACK = 2048;
const uint16_t BUZZER_TASK_STACK = 1024;
const uint16_t STATUS_LED_TASK_STACK = 1024;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, handle, core)


const int STATUS_LED_PIN = config::kStatusLedPin;
constexpr auto SR_STATUS_LED_PRIMARY = ShiftRegister::Output::SystemIndicatorLed;
constexpr auto SR_STATUS_LED_SECONDARY = ShiftRegister::Output::GripperIndicatorLed;
constexpr auto SR_STATUS_LED_DEBUG = ShiftRegister::Output::ArmIndicatorLed;

constexpr ShiftRegister::Output kHighPowerLedOutputs[3] = {
    ShiftRegister::Output::HighPowerLed1,
    ShiftRegister::Output::HighPowerLed2,
    ShiftRegister::Output::HighPowerLed3,
};

struct PeripheralState {
    uint8_t ledPwm[3] = {0, 0, 0};
    uint8_t pumpDuty = 0;
    uint8_t userMask = 0;
};

static PeripheralState gPeripheralState{};

static void publishStatus();
static void applyConfigurationPacket(const ConfigurationPacket &packet);
static void applySettingsPacket(const SettingsPacket &packet);
static void sendArmStateSnapshot();
static Motor::Outputs filterDriveCommand(const Motor::Outputs &target, float dtSeconds, bool allowEasing);
static void resetDriveEasing(const Motor::Outputs &value);
static void updateBatterySafety(uint16_t batteryMillivolts);
static void reconfigureSoftAp(const char *ssid, const char *password);


/// ==================== CONSTANTS ====================
char WIFI_SSID[33] = "Thegill Telemetry";
char WIFI_PASSWORD[65] = "ASCEpec@2025";
const int TCP_PORT = 8000;

// Motor and control constants
const int16_t MOTOR_MIN = -1000;
const int16_t MOTOR_MAX = 1000;
const unsigned long HEARTBEAT_TIMEOUT_MS = 2000;  // ms without controller traffic
const unsigned long TELEMETRY_INTERVAL = 50; // ms
bool failsafe_enable = false;
bool isArmed = true;
static DriveEasingMode g_driveEasingMode = DriveEasingMode::None;
static uint8_t g_driveEasingRate = 0;
static Motor::Outputs g_easedOutputs{0, 0, 0, 0};
static bool g_batterySafetyEnabled = false;
static bool g_batteryProtectionLatched = false;
static uint16_t g_batteryCutoffMv = 15000;
static uint16_t g_batteryRecoverMv = 15500;
static uint16_t g_lastBatteryMillivolts = 0;

char DRONE_ID[33] = "Thegill";
char ROBOT_NAME[32] = "ThegillS3";

struct BuzzerCommand {
    uint16_t durationMs;
    uint8_t freqCount;
    uint16_t freqs[3];
};

// ==================== GLOBAL VARIABLES ====================
// Hardware
WiFiServer server(TCP_PORT);
WiFiClient client;
ThegillCommand command = {THEGILL_PACKET_MAGIC, 0, 0, 0, 0, 0, 0};
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
Motor::Outputs currentOutputs{0, 0, 0, 0};
Motor::Outputs targetOutputs{0, 0, 0, 0};
ThegillTelemetry gillTelemetry{};
unsigned long lastTelemetry = 0;
QueueHandle_t buzzerQueue = nullptr;
bool buzzerMuted = false;

// ==================== COMMUNICATION FUNCTIONS ====================
bool telemetryEnabled = false; // Serial/TCP telemetry disabled by default
const int MAX_MESSAGE_LENGTH = 256;
char messageBuffer[MAX_MESSAGE_LENGTH + 1] = {0};
size_t messageLength = 0;
bool serialActive = false; // Tracks if a Serial session is currently open
bool armControlEnabled = true;

// ==================== IMPROVED COMMUNICATION FUNCTIONS ====================

static inline ThegillCommand loadCommandSnapshot()
{
    portENTER_CRITICAL(&commandMux);
    ThegillCommand snapshot = command;
    portEXIT_CRITICAL(&commandMux);
    return snapshot;
}

static inline void storeCommandSnapshot(const ThegillCommand &value)
{
    portENTER_CRITICAL(&commandMux);
    command = value;
    portEXIT_CRITICAL(&commandMux);
}

static inline void storeCommandSnapshotFromISR(const ThegillCommand &value)
{
    portENTER_CRITICAL_ISR(&commandMux);
    command = value;
    portEXIT_CRITICAL_ISR(&commandMux);
}

static inline Motor::Outputs commandToMotorOutputs(const ThegillCommand &cmd)
{
    Motor::Outputs outputs{};
    outputs.leftFront = static_cast<int16_t>(constrain(cmd.leftFront, MOTOR_MIN, MOTOR_MAX));
    outputs.leftRear = static_cast<int16_t>(constrain(cmd.leftRear, MOTOR_MIN, MOTOR_MAX));
    outputs.rightFront = static_cast<int16_t>(constrain(cmd.rightFront, MOTOR_MIN, MOTOR_MAX));
    outputs.rightRear = static_cast<int16_t>(constrain(cmd.rightRear, MOTOR_MIN, MOTOR_MAX));
    return outputs;
}

static inline bool copyBoundedString(char *dest, size_t destSize, const char *src, size_t srcSize)
{
    if (!dest || destSize == 0)
    {
        return false;
    }
    if (!src || srcSize == 0)
    {
        return false;
    }
    size_t copyLen = 0;
    while (copyLen < srcSize && src[copyLen] != '\0')
    {
        ++copyLen;
    }
    if (copyLen == 0)
    {
        return false;
    }
    if (copyLen >= destSize)
    {
        copyLen = destSize - 1;
    }
    std::memcpy(dest, src, copyLen);
    dest[copyLen] = '\0';
    return true;
}

static void resetDriveEasing(const Motor::Outputs &value)
{
    g_easedOutputs = value;
}

static Motor::Outputs filterDriveCommand(const Motor::Outputs &target, float dtSeconds, bool allowEasing)
{
    if (!allowEasing || g_driveEasingMode == DriveEasingMode::None || g_driveEasingRate == 0)
    {
        resetDriveEasing(target);
        return target;
    }

    Motor::Outputs eased = g_easedOutputs;
    switch (g_driveEasingMode)
    {
        case DriveEasingMode::SlewRate:
        {
            const float countsPerSecond = static_cast<float>(g_driveEasingRate) * 50.0f;
            float maxStep = countsPerSecond * dtSeconds;
            if (maxStep < 1.0f)
            {
                maxStep = 1.0f;
            }
            auto applyChannel = [&](int16_t &current, int16_t desired) {
                float delta = static_cast<float>(desired - current);
                if (delta > maxStep)
                {
                    delta = maxStep;
                }
                else if (delta < -maxStep)
                {
                    delta = -maxStep;
                }
                current = static_cast<int16_t>(roundf(static_cast<float>(current) + delta));
            };
            applyChannel(eased.leftFront, target.leftFront);
            applyChannel(eased.leftRear, target.leftRear);
            applyChannel(eased.rightFront, target.rightFront);
            applyChannel(eased.rightRear, target.rightRear);
            break;
        }
        case DriveEasingMode::Exponential:
        {
            float alpha = static_cast<float>(g_driveEasingRate) / 255.0f;
            alpha = constrain(alpha, 0.0f, 1.0f);
            if (alpha <= 0.0f)
            {
                resetDriveEasing(target);
                return target;
            }
            auto blendChannel = [&](int16_t &current, int16_t desired) {
                float blended = alpha * static_cast<float>(desired) + (1.0f - alpha) * static_cast<float>(current);
                current = static_cast<int16_t>(roundf(blended));
            };
            blendChannel(eased.leftFront, target.leftFront);
            blendChannel(eased.leftRear, target.leftRear);
            blendChannel(eased.rightFront, target.rightFront);
            blendChannel(eased.rightRear, target.rightRear);
            break;
        }
        default:
            eased = target;
            break;
    }

    g_easedOutputs = eased;
    return eased;
}

static void reconfigureSoftAp(const char *ssid, const char *password)
{
    if (!ssid || !password)
    {
        return;
    }
    WiFi.softAP(ssid, password);
}

static void sendArmStateSnapshot()
{
    if (!Comms::paired())
    {
        return;
    }

    ArmStatePacket packet{};
    packet.magic = THEGILL_ARM_STATE_MAGIC;
    packet.baseDegrees = ArmControl::basePositionDegrees();
    packet.extensionCentimeters = ArmControl::extensionPositionCentimeters();

    uint8_t enabledMask = 0;
    uint8_t attachedMask = 0;
    for (size_t idx = 0; idx < THEGILL_ARM_SERVO_COUNT; ++idx)
    {
        ArmServos::ServoId id = static_cast<ArmServos::ServoId>(idx);
        ArmServos::ServoStatus status = ArmServos::status(id);
        packet.servoDegrees[idx] = status.targetDegrees;
        if (status.enabled)
        {
            enabledMask |= static_cast<uint8_t>(1u << idx);
        }
        if (status.attached)
        {
            attachedMask |= static_cast<uint8_t>(1u << idx);
        }
    }
    packet.servoEnabledMask = enabledMask;
    packet.servoAttachedMask = attachedMask;
    packet.flags = 0;
    if (ArmControl::outputsEnabled())
    {
        packet.flags |= 0x01;
    }
    if (ArmServos::enabled())
    {
        packet.flags |= 0x02;
    }

    Comms::sendArmStatePacket(packet);
}

static void applySettingsPacket(const SettingsPacket &packet)
{
    if (copyBoundedString(ROBOT_NAME, sizeof(ROBOT_NAME), packet.robotName, sizeof(packet.robotName)))
    {
        Comms::setPlatform(ROBOT_NAME);
    }
    if (copyBoundedString(DRONE_ID, sizeof(DRONE_ID), packet.customId, sizeof(packet.customId)))
    {
        Comms::setCustomId(DRONE_ID);
    }
    bool updatedSsid = copyBoundedString(WIFI_SSID, sizeof(WIFI_SSID), packet.wifiSsid, sizeof(packet.wifiSsid));
    bool updatedPassword = copyBoundedString(WIFI_PASSWORD, sizeof(WIFI_PASSWORD), packet.wifiPassword, sizeof(packet.wifiPassword));
    if (updatedSsid || updatedPassword)
    {
        reconfigureSoftAp(WIFI_SSID, WIFI_PASSWORD);
    }
}

static void updateBatterySafety(uint16_t batteryMillivolts)
{
    g_lastBatteryMillivolts = batteryMillivolts;
    if (!g_batterySafetyEnabled)
    {
        g_batteryProtectionLatched = false;
        return;
    }

    if (!g_batteryProtectionLatched && batteryMillivolts > 0 && batteryMillivolts <= g_batteryCutoffMv)
    {
        g_batteryProtectionLatched = true;
        isArmed = false;

        ThegillCommand safe = loadCommandSnapshot();
        safe.leftFront = safe.leftRear = safe.rightFront = safe.rightRear = 0;
        safe.flags |= GILL_FLAG_BRAKE;
        storeCommandSnapshot(safe);

        Motor::Outputs outputs = commandToMotorOutputs(safe);
        targetOutputs = outputs;
        resetDriveEasing(outputs);
        Motor::update(false, true, currentOutputs, targetOutputs);

        ArmControl::setOutputsEnabled(false);
        ArmServos::setEnabled(false);
    }
    else if (g_batteryProtectionLatched && batteryMillivolts >= g_batteryRecoverMv)
    {
        g_batteryProtectionLatched = false;
        ArmControl::setOutputsEnabled(armControlEnabled);
        ArmServos::setEnabled(armControlEnabled);
    }
}

static void applyConfigurationPacket(const ConfigurationPacket &packet)
{
    DriveEasingMode mode = DriveEasingMode::None;
    if (packet.easingMode <= static_cast<uint8_t>(DriveEasingMode::Exponential))
    {
        mode = static_cast<DriveEasingMode>(packet.easingMode);
    }
    g_driveEasingMode = mode;
    g_driveEasingRate = packet.easingRate;
    Motor::Outputs snapshotOutputs = commandToMotorOutputs(loadCommandSnapshot());
    resetDriveEasing(snapshotOutputs);

    bool mute = (packet.controlFlags & ConfigFlag::MuteAudio) != 0;
    buzzerMuted = mute;

    bool driveEnabled = (packet.controlFlags & ConfigFlag::DriveEnabled) != 0;
    if (isArmed != driveEnabled)
    {
        isArmed = driveEnabled;
        if (!driveEnabled)
        {
            ThegillCommand safe = loadCommandSnapshot();
            safe.leftFront = safe.leftRear = safe.rightFront = safe.rightRear = 0;
            safe.flags |= GILL_FLAG_BRAKE;
            storeCommandSnapshot(safe);

            Motor::Outputs outputs = commandToMotorOutputs(safe);
            targetOutputs = outputs;
            resetDriveEasing(outputs);
            Motor::update(false, true, currentOutputs, targetOutputs);
        }
    }
    if (g_batteryProtectionLatched)
    {
        isArmed = false;
    }

    bool manipEnabled = (packet.controlFlags & ConfigFlag::ArmOutputsEnable) != 0;
    if (armControlEnabled != manipEnabled)
    {
        armControlEnabled = manipEnabled;
        ArmControl::setOutputsEnabled(armControlEnabled);
        ArmServos::setEnabled(armControlEnabled);
    }

    failsafe_enable = (packet.controlFlags & ConfigFlag::FailsafeEnable) != 0;

    g_batterySafetyEnabled = (packet.safetyFlags & SafetyFlag::BatterySafetyEnable) != 0;
    if (packet.batteryCutoffMillivolts != 0)
    {
        g_batteryCutoffMv = packet.batteryCutoffMillivolts;
    }
    if (packet.batteryRecoverMillivolts != 0)
    {
        g_batteryRecoverMv = packet.batteryRecoverMillivolts;
    }
    if (g_batteryRecoverMv <= g_batteryCutoffMv)
    {
        g_batteryRecoverMv = g_batteryCutoffMv + 200;
    }
    if (!g_batterySafetyEnabled)
    {
        g_batteryProtectionLatched = false;
    }
}

static void syncPeripheralOutputs()
{
    if (!ShiftRegister::initialized())
    {
        return;
    }

    for (uint8_t i = 0; i < 3; ++i)
    {
        ShiftRegister::writeChannelPwm(kHighPowerLedOutputs[i], gPeripheralState.ledPwm[i]);
    }

    ShiftRegister::writeChannelPwm(ShiftRegister::Output::PumpControl, gPeripheralState.pumpDuty);
    ShiftRegister::writeUserMask(gPeripheralState.userMask);
}

static void applyPeripheralCommand(const Comms::PeripheralCommand &packet)
{
    for (uint8_t i = 0; i < 3; ++i)
    {
        gPeripheralState.ledPwm[i] = packet.ledPwm[i];
    }
    gPeripheralState.pumpDuty = packet.pumpDuty;
    gPeripheralState.userMask = packet.userMask;
    syncPeripheralOutputs();
}

static void clearPeripheralState()
{
    gPeripheralState = PeripheralState{};
    syncPeripheralOutputs();
}

static void handleSystemCommandBits(uint8_t bits)
{
    if (bits & GillSystemCommand::EnableTelemetry)
    {
        telemetryEnabled = true;
    }
    if (bits & GillSystemCommand::DisableTelemetry)
    {
        telemetryEnabled = false;
    }
    if (bits & GillSystemCommand::EnableBuzzer)
    {
        buzzerMuted = false;
    }
    if (bits & GillSystemCommand::DisableBuzzer)
    {
        buzzerMuted = true;
    }
    if (bits & GillSystemCommand::RequestStatus)
    {
        lastTelemetry = 0;
        publishStatus();
    }
    if (bits & GillSystemCommand::RequestArmState)
    {
        sendArmStateSnapshot();
    }
}

static void applyArmControlCommand(const ArmControlCommand &packet, uint32_t timestampMs)
{
    (void)timestampMs;

    bool requestEnableOutputs = (packet.flags & ArmCommandFlag::EnableOutputs) != 0;
    bool requestDisableOutputs = (packet.flags & ArmCommandFlag::DisableOutputs) != 0;
    if (requestEnableOutputs && !requestDisableOutputs)
    {
        armControlEnabled = true;
        ArmControl::setOutputsEnabled(true);
    }
    else if (requestDisableOutputs && !requestEnableOutputs)
    {
        armControlEnabled = false;
        ArmControl::setOutputsEnabled(false);
    }

    bool requestEnableServos = (packet.flags & ArmCommandFlag::EnableServos) != 0;
    bool requestDisableServos = (packet.flags & ArmCommandFlag::DisableServos) != 0;
    if (requestEnableServos && !requestDisableServos)
    {
        ArmServos::setEnabled(true);
    }
    else if (requestDisableServos && !requestEnableServos)
    {
        ArmServos::setEnabled(false);
    }

    if (packet.validMask & ArmCommandMask::Extension)
    {
        float centimeters = packet.extensionMillimeters * 0.1f;
        ArmControl::setExtensionTargetCentimeters(centimeters);
    }
    if (packet.validMask & ArmCommandMask::Shoulder)
    {
        ArmServos::setTargetDegrees(ArmServos::ServoId::Shoulder, packet.shoulderDegrees);
    }
    if (packet.validMask & ArmCommandMask::Elbow)
    {
        ArmServos::setTargetDegrees(ArmServos::ServoId::Elbow, packet.elbowDegrees);
    }
    if (packet.validMask & ArmCommandMask::Pitch)
    {
        ArmServos::setTargetDegrees(ArmServos::ServoId::Pitch, packet.pitchDegrees);
    }
    if (packet.validMask & ArmCommandMask::Roll)
    {
        ArmServos::setTargetDegrees(ArmServos::ServoId::Roll, packet.rollDegrees);
    }
    if (packet.validMask & ArmCommandMask::Yaw)
    {
        ArmServos::setTargetDegrees(ArmServos::ServoId::Yaw, packet.yawDegrees);
    }

    armControlEnabled = ArmControl::outputsEnabled();
}

static void applyThegillCommandPayload(const ThegillCommand &packet, uint32_t timestampMs)
{
    (void)timestampMs;
    if (packet.magic != THEGILL_PACKET_MAGIC)
    {
        return;
    }

    if (packet.system != 0)
    {
        handleSystemCommandBits(packet.system);
    }

    ThegillCommand updated = packet;
    updated.leftFront = static_cast<int16_t>(constrain(updated.leftFront, MOTOR_MIN, MOTOR_MAX));
    updated.leftRear = static_cast<int16_t>(constrain(updated.leftRear, MOTOR_MIN, MOTOR_MAX));
    updated.rightFront = static_cast<int16_t>(constrain(updated.rightFront, MOTOR_MIN, MOTOR_MAX));
    updated.rightRear = static_cast<int16_t>(constrain(updated.rightRear, MOTOR_MIN, MOTOR_MAX));

    storeCommandSnapshot(updated);
}

static inline void resetMessageBuffer()
{
    messageLength = 0;
    messageBuffer[0] = '\0';
}

static void dispatchBufferedMessage()
{
    if (messageLength == 0)
        return;

    messageBuffer[messageLength] = '\0';
    Commands::handleCommand(String(messageBuffer));
    resetMessageBuffer();
}

static void handleBufferedCharacter(char c)
{
    if (c == '\n' || c == '\r')
    {
        dispatchBufferedMessage();
        return;
    }

    if (messageLength < MAX_MESSAGE_LENGTH)
    {
        messageBuffer[messageLength++] = c;
        return;
    }

    resetMessageBuffer();
    Commands::sendLine("ERROR: Message too long");
}

static void enqueueChord(const uint16_t *freqs, size_t count, uint16_t durationMs)
{
    if (BUZZER_PIN < 0 || !buzzerQueue || !freqs || count == 0 || buzzerMuted)
        return;

    BuzzerCommand cmd{};
    cmd.durationMs = durationMs;
    cmd.freqCount = count > 3 ? 3 : static_cast<uint8_t>(count);
    for (size_t i = 0; i < cmd.freqCount; ++i)
    {
        cmd.freqs[i] = freqs[i];
    }

    xQueueSend(buzzerQueue, &cmd, 0);
}

void beep(uint16_t freq, uint16_t duration)
{
    const uint16_t single[1] = {freq};
    enqueueChord(single, 1, duration);
}

void BuzzerTask(void *pvParameters)
{
    BuzzerCommand cmd;
    while (xQueueReceive(buzzerQueue, &cmd, portMAX_DELAY))
    {
        if (cmd.freqCount == 0 || cmd.durationMs == 0)
        {
            ledcWrite(BUZZER_CHANNEL, 0);
            continue;
        }

        if (cmd.freqCount == 1)
        {
            ledcWriteTone(BUZZER_CHANNEL, cmd.freqs[0]);
            vTaskDelay(pdMS_TO_TICKS(cmd.durationMs));
        }
        else
        {
            TickType_t totalTicks = pdMS_TO_TICKS(cmd.durationMs);
            if (totalTicks == 0)
                totalTicks = 1;
            TickType_t slice = pdMS_TO_TICKS(10);
            if (slice == 0)
                slice = 1;
            TickType_t elapsed = 0;
            uint8_t index = 0;
            while (elapsed < totalTicks)
            {
                ledcWriteTone(BUZZER_CHANNEL, cmd.freqs[index]);
                TickType_t remaining = totalTicks - elapsed;
                TickType_t waitTicks = remaining < slice ? remaining : slice;
                vTaskDelay(waitTicks);
                elapsed += waitTicks;
                index = (index + 1) % cmd.freqCount;
            }
        }

        ledcWrite(BUZZER_CHANNEL, 0);
    }
}

static void runBuzzerSelfTest()
{
    if (BUZZER_PIN < 0 || buzzerMuted)
    {
        return;
    }

    Serial.println("Running buzzer self-test...");
    constexpr uint16_t kSweep[] = {440, 660, 880, 660};
    constexpr uint16_t kToneDurationMs = 140;
    constexpr uint16_t kGapMs = 60;

    for (uint8_t i = 0; i < sizeof(kSweep) / sizeof(kSweep[0]); ++i)
    {
        ledcWriteTone(BUZZER_CHANNEL, kSweep[i]);
        delay(kToneDurationMs);
        ledcWrite(BUZZER_CHANNEL, 0);
        delay(kGapMs);
    }

    Serial.println("Buzzer self-test complete");
}

static void playImmediateStartupTone()
{
    for(int i=0l; i <100;i++)
    {
    digitalWrite(BUZZER_PIN,1);
    delay(2);
    digitalWrite(BUZZER_PIN,0);
    delay(2);
    }
}

static void runMotorStartupTest()
{
    Serial.println("Running motor startup self-test...");

    const int16_t maxTestOutput = 300;
    const int16_t step = 50;
    const uint16_t stepDelayMs = 50;

    Motor::Outputs outputs{0, 0, 0, 0};

    for (int16_t value = 0; value <= maxTestOutput; value += step)
    {
        outputs.leftFront = outputs.leftRear = outputs.rightFront = outputs.rightRear = value;
        targetOutputs = outputs;
        Motor::update(true, false, currentOutputs, targetOutputs);
        delay(stepDelayMs);
    }

    delay(200);

    for (int16_t value = maxTestOutput; value >= 0; value -= step)
    {
        outputs.leftFront = outputs.leftRear = outputs.rightFront = outputs.rightRear = value;
        targetOutputs = outputs;
        Motor::update(true, false, currentOutputs, targetOutputs);
        delay(stepDelayMs);
    }

    targetOutputs = {0, 0, 0, 0};
    Motor::update(false, false, currentOutputs, targetOutputs);
    resetDriveEasing(targetOutputs);
    Serial.println("Motor startup self-test complete.");
}

void publishStatus()
{
    if (millis() - lastTelemetry < TELEMETRY_INTERVAL)
        return;

    lastTelemetry = millis();

    uint32_t nowMs = millis();
    uint32_t lastCommandMs = Comms::lastCommandTimestamp();
    uint32_t commandAge = (lastCommandMs != 0 && nowMs >= lastCommandMs) ? (nowMs - lastCommandMs) : 0;

    Comms::StatusPacket packet{};
    packet.magic = Comms::THEGILL_STATUS_MAGIC;

    constexpr size_t kWheelCount = sizeof(packet.wheelSpeedMmPerSec) / sizeof(packet.wheelSpeedMmPerSec[0]);
    Motor::EncoderMeasurement measurements[kWheelCount] = {};
    std::size_t measurementCount = Motor::encoderMeasurements(measurements, kWheelCount);

    for (size_t i = 0; i < kWheelCount; ++i)
    {
        float mmps = 0.0f;
        if (i < measurementCount && measurements[i].valid)
        {
            mmps = measurements[i].metersPerSecond * 1000.0f;
        }
        if (mmps > 32767.0f)
        {
            mmps = 32767.0f;
        }
        else if (mmps < -32768.0f)
        {
            mmps = -32768.0f;
        }
        packet.wheelSpeedMmPerSec[i] = static_cast<int16_t>(mmps);
    }

    float batteryVolts = AnalogInputs::readBatteryVoltage();
    int32_t batteryMillivolts = static_cast<int32_t>(batteryVolts * 1000.0f + 0.5f);
    if (batteryMillivolts < 0)
    {
        batteryMillivolts = 0;
    }
    else if (batteryMillivolts > 0xFFFF)
    {
        batteryMillivolts = 0xFFFF;
    }
    packet.batteryMillivolts = static_cast<uint16_t>(batteryMillivolts);
    updateBatterySafety(packet.batteryMillivolts);

    std::memcpy(packet.ledPwm, gPeripheralState.ledPwm, sizeof(packet.ledPwm));
    packet.pumpDuty = gPeripheralState.pumpDuty;
    packet.userMask = gPeripheralState.userMask;

    packet.flags = 0;
    if (ArmControl::outputsEnabled())
    {
        packet.flags |= Comms::StatusFlag::ArmOutputsEnabled;
    }
    if (failsafe_enable)
    {
        packet.flags |= Comms::StatusFlag::FailsafeArmed;
    }
    if (telemetryEnabled)
    {
        packet.flags |= Comms::StatusFlag::TelemetryStreaming;
    }
    if (client && client.connected())
    {
        packet.flags |= Comms::StatusFlag::TcpClientActive;
    }
    if (serialActive)
    {
        packet.flags |= Comms::StatusFlag::SerialActive;
    }
    if (Comms::paired())
    {
        packet.flags |= Comms::StatusFlag::PairedLink;
    }
    if (g_batterySafetyEnabled && g_batteryProtectionLatched)
    {
        packet.flags |= Comms::StatusFlag::BatterySafeActive;
    }
    if (isArmed)
    {
        packet.flags |= Comms::StatusFlag::DriveArmed;
    }

    if (commandAge > 0xFFFF)
    {
        commandAge = 0xFFFF;
    }
    packet.commandAgeMs = static_cast<uint16_t>(commandAge);

    Comms::sendStatusPacket(packet);

    bool tcpActive = client && client.connected();
    if (!(telemetryEnabled && (serialActive || tcpActive)))
        return;

    String telemetry = String("STAT mv=") + String(packet.batteryMillivolts) +
                       " mmps=" + String(packet.wheelSpeedMmPerSec[0]) + "," +
                       String(packet.wheelSpeedMmPerSec[1]) + "," +
                       String(packet.wheelSpeedMmPerSec[2]) + "," +
                       String(packet.wheelSpeedMmPerSec[3]) +
                       " pump=" + String(packet.pumpDuty) +
                       " leds=" + String(packet.ledPwm[0]) + "," +
                       String(packet.ledPwm[1]) + "," +
                       String(packet.ledPwm[2]) +
                       " user=0x" + String(packet.userMask, HEX) +
                       " flags=0x" + String(packet.flags, HEX) +
                       " age=" + String(packet.commandAgeMs);

    if (serialActive)
        Serial.println(telemetry);
    if (tcpActive)
    {
        client.println(telemetry);
        client.flush();
    }
}
void handleIncomingData()
{
    // Update serial connection status based on host presence
#if ARDUINO_USB_CDC_ON_BOOT
    serialActive = Serial; // DTR not available on some boards
#else
    // Without USB-CDC we can't detect port status; start once data arrives
    if (Serial.available())
        serialActive = true;
#endif

    // Handle TCP client connection
    if (!client || !client.connected())
    {
        WiFiClient newClient = server.available();
        if (newClient)
        {
            if (client)
                client.stop(); // Close old connection
            client = newClient;
            Commands::sendLine("ACK: TCP client connected");
        }
    }

    // Process TCP data with buffering
    while (client && client.available())
    {
        char c = client.read();
        handleBufferedCharacter(c);
    }

    // Process Serial data with buffering
    while (Serial.available())
    {
        char c = Serial.read();
        handleBufferedCharacter(c);
    }
}

// ==================== FAILSAFE LOGIC ====================

void checkFailsafe() {
    if (!failsafe_enable)
        return;

    static bool rampingDown = false;
    static uint32_t lastRampUpdate = 0;

    uint32_t nowMs = millis();
    uint32_t lastHeartbeatMs = Comms::lastPeerHeartbeatTimestamp();
    bool heartbeatRecent = (lastHeartbeatMs != 0 && nowMs >= lastHeartbeatMs &&
                            (nowMs - lastHeartbeatMs) <= HEARTBEAT_TIMEOUT_MS);

    if (heartbeatRecent) {
        rampingDown = false;
        return;
    }

    if (!rampingDown) {
        rampingDown = true;
        lastRampUpdate = nowMs;
        // Immediately drop all shift-register outputs (LEDs, pump, aux) to avoid unintended actuation
        if (ShiftRegister::initialized()) {
            ShiftRegister::clearAll();
        }
        clearPeripheralState();
        // Disable arm outputs so STBY stays low and PID idles
        ArmControl::setOutputsEnabled(false);
    }

    const uint32_t rampIntervalMs = 50;
    if (nowMs - lastRampUpdate < rampIntervalMs)
        return;
    lastRampUpdate = nowMs;

    ThegillCommand safe = loadCommandSnapshot();
    bool changed = false;

    auto rampValue = [&](int16_t *value) {
        int16_t original = *value;
        if (*value > 0) {
            int16_t next = *value - 20;
            *value = next < 0 ? 0 : next;
        } else if (*value < 0) {
            int16_t next = *value + 20;
            *value = next > 0 ? 0 : next;
        }
        if (*value != original)
            changed = true;
    };

    rampValue(&safe.leftFront);
    rampValue(&safe.leftRear);
    rampValue(&safe.rightFront);
    rampValue(&safe.rightRear);

    if (changed) {
        safe.flags |= GILL_FLAG_BRAKE;
        storeCommandSnapshot(safe);
        targetOutputs = commandToMotorOutputs(safe);
        resetDriveEasing(targetOutputs);
    } else {
        rampingDown = false;
    }
}


// ==================== CONTROL LOOP ====================

void FastTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(2); // 500 Hz control loop
    constexpr float kLoopDtSeconds = 0.002f;
    while (true) {
        ThegillCommand currentCommand = loadCommandSnapshot();

        Motor::Outputs desired = commandToMotorOutputs(currentCommand);

        bool brake = (currentCommand.flags & GILL_FLAG_BRAKE) != 0;
        bool honkFlag = (currentCommand.flags & GILL_FLAG_HONK) != 0;
        if (!isArmed || brake) {
            desired.leftFront = 0;
            desired.leftRear = 0;
            desired.rightFront = 0;
            desired.rightRear = 0;
        }

        desired = filterDriveCommand(desired, kLoopDtSeconds, isArmed && !brake);

        targetOutputs = desired;
        Motor::update(isArmed, brake, currentOutputs, targetOutputs);
        ArmControl::setOutputsEnabled(armControlEnabled);
        ArmControl::update(kLoopDtSeconds);

        gillTelemetry.targetLeftFront = desired.leftFront / 1000.0f;
        gillTelemetry.targetLeftRear = desired.leftRear / 1000.0f;
        gillTelemetry.targetRightFront = desired.rightFront / 1000.0f;
        gillTelemetry.targetRightRear = desired.rightRear / 1000.0f;
        gillTelemetry.actualLeftFront = currentOutputs.leftFront / 1000.0f;
        gillTelemetry.actualLeftRear = currentOutputs.leftRear / 1000.0f;
        gillTelemetry.actualRightFront = currentOutputs.rightFront / 1000.0f;
        gillTelemetry.actualRightRear = currentOutputs.rightRear / 1000.0f;
        gillTelemetry.brakeActive = brake;

        static bool lastHonkFlag = false;
        static bool honkToggleState = false;
        if (lastHonkFlag && !honkFlag) {
            honkToggleState = !honkToggleState;
            if (BUZZER_PIN >= 0) {
                const uint16_t honkChord[] = {523, 659, 784};
                enqueueChord(honkChord, sizeof(honkChord) / sizeof(honkChord[0]), honkToggleState ? 350 : 200);
            }
        }
        lastHonkFlag = honkFlag;
        gillTelemetry.honkActive = honkToggleState;

        vTaskDelayUntil(&lastWake, interval);
    }
}


static void StatusLedTask(void *pvParameters)
{
    (void)pvParameters;

    struct LedPattern {
        ShiftRegister::Output output;
        float periodMs;
        float phaseOffsetMs;
    };

    constexpr LedPattern kPatterns[] = {
        {SR_STATUS_LED_PRIMARY,   3200.0f,   0.0f},  // Slow breath
        {SR_STATUS_LED_SECONDARY, 1600.0f, 240.0f},  // Medium breath
        {SR_STATUS_LED_DEBUG,      900.0f, 480.0f},  // Fast breath
    };

    constexpr float kTwoPi = 6.28318530718f;
    const TickType_t sleepTicks = pdMS_TO_TICKS(15);

    TickType_t boardLastToggle = xTaskGetTickCount();
    bool boardLevel = false;
    digitalWrite(STATUS_LED_PIN, LOW);

    while (true) {
        uint64_t nowUs = esp_timer_get_time();
        for (const auto &pattern : kPatterns) {
            float effectiveMs = static_cast<float>(nowUs) / 1000.0f + pattern.phaseOffsetMs;
            if (pattern.periodMs <= 0.0f) {
                ShiftRegister::writeChannel(pattern.output, false);
                continue;
            }
            float cycles = effectiveMs / pattern.periodMs;
            float fractional = cycles - floorf(cycles);
            float angle = fractional * kTwoPi;
            float intensity = 0.5f * (1.0f - cosf(angle));
            uint8_t duty = static_cast<uint8_t>(intensity * 255.0f + 0.5f);
            ShiftRegister::writeChannelPwm(pattern.output, duty);
        }

        TickType_t nowTicks = xTaskGetTickCount();
        if (nowTicks - boardLastToggle >= pdMS_TO_TICKS(500)) {
            boardLevel = !boardLevel;
            digitalWrite(STATUS_LED_PIN, boardLevel ? HIGH : LOW);
            boardLastToggle = nowTicks;
        }

        vTaskDelay(sleepTicks);
    }
}

void CommTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(5);
    bool lastPaired = false;
    uint8_t lastPeerMac[6] = {0};

    while (true) {
        handleIncomingData();
        Comms::loop();

        Comms::LinkStatus status = Comms::getLinkStatus();
        if (status.paired) {
            bool peerChanged = !lastPaired || !Comms::macEqual(status.peerMac, lastPeerMac);
            if (peerChanged) {
                memcpy(lastPeerMac, status.peerMac, sizeof(lastPeerMac));
                if (BUZZER_PIN >= 0) {
                    beep(2000, 200);
                }
            }
        } else {
            memset(lastPeerMac, 0, sizeof(lastPeerMac));
        }
        lastPaired = status.paired;

        uint32_t timestampMs = 0;
        ConfigurationPacket configPacket{};
        if (Comms::receiveConfigurationPacket(configPacket, &timestampMs)) {
            (void)timestampMs;
            applyConfigurationPacket(configPacket);
        }

        timestampMs = 0;
        SettingsPacket settingsPacket{};
        if (Comms::receiveSettingsPacket(settingsPacket, &timestampMs)) {
            (void)timestampMs;
            applySettingsPacket(settingsPacket);
        }

        timestampMs = 0;
        ArmControlCommand armCommand{};
        if (Comms::receiveArmCommand(armCommand, &timestampMs)) {
            applyArmControlCommand(armCommand, timestampMs);
        }

        timestampMs = 0;
        ThegillCommand directCommand{};
        if (Comms::receiveThegillCommand(directCommand, &timestampMs)) {
            applyThegillCommandPayload(directCommand, timestampMs);
        }

        timestampMs = 0;
        Comms::PeripheralCommand peripheralCommand{};
        if (Comms::receivePeripheralCommand(peripheralCommand, &timestampMs)) {
            (void)timestampMs;
            applyPeripheralCommand(peripheralCommand);
        }

        vTaskDelayUntil(&lastWake, interval);
    }
}


void FailsafeTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(10);
    while (true) {
        checkFailsafe();
        vTaskDelayUntil(&lastWake, interval); // ~10 Hz
    }
}

void TelemetryTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(10);
    while (true) {
        publishStatus();
        vTaskDelayUntil(&lastWake, interval);
    }
}

void OTATask(void *pvParameters) {
    while (true) {
        ArduinoOTA.handle();  // OTA can be run slowly
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Ensure SR is cleared on system shutdown/reset to avoid stray outputs
static void SRShutdownHandler() {
    ShiftRegister::clearAll();
    clearPeripheralState();
}


void setup()
{
    playImmediateStartupTone();
    Serial.begin(115200);
    Serial.println("Thegill S3 controller starting...");
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    if (BUZZER_PIN >= 0) {
        ledcSetup(BUZZER_CHANNEL, 2000, 8);
        ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
        ledcWrite(BUZZER_CHANNEL, 0);
        buzzerQueue = xQueueCreate(5, sizeof(BuzzerCommand));
        CREATE_TASK(
            BuzzerTask,
            "BuzzerTask",
            BUZZER_TASK_STACK,
            1,
            NULL,
            1
        );
        const uint16_t bootNotes[] = {523, 659, 784};
        for (uint8_t i = 0; i < sizeof(bootNotes) / sizeof(bootNotes[0]); ++i) {
            beep(bootNotes[i], 160);
        }
    }

    if (!ShiftRegister::init(
            SHIFT_REG_PINS,
            config::kShiftRegisterPwmFrequencyHz,
            config::kShiftRegisterPwmResolutionBits)) {
        Serial.println("Shift register init failed");
        while (true) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(120);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(120);
        }
    }

    clearPeripheralState();
    // Register shutdown clear
    esp_register_shutdown_handler(&SRShutdownHandler);

    if (!AnalogInputs::init()) {
        Serial.println("Analog input subsystem init failed");
    }

    if (PcintEncoder::initDefault()) {
        Serial.printf("Wheel encoder subsystem ready (%u channels)\n",
                      static_cast<unsigned>(PcintEncoder::encoderCount()));
    } else {
        Serial.println("Wheel encoder subsystem skipped (no valid PCINT pins)");
    }

    ArmControl::init();
    ArmControl::setOutputsEnabled(armControlEnabled);
    ArmServos::init();
    ArmServos::setEnabled(armControlEnabled);

    setCpuFrequencyMhz(CPU_FREQ_MHZ);

    Comms::setRole(Comms::DeviceRole::Controlled);
    Comms::setPlatform(ROBOT_NAME);
    Comms::setCustomId(DRONE_ID);
    Comms::setDeviceTypeOverride("THEGILL");

    if (!Comms::init(WIFI_SSID, WIFI_PASSWORD, TCP_PORT)) {
        Serial.println("ESP-NOW init failed");
        while (true) {
            beep(2000, 500);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ArduinoOTA.begin();
    server.begin();

    Comms::Identity selfIdentity{};
    Comms::fillSelfIdentity(selfIdentity);
    Serial.print("ESP-NOW initialized as ");
    Serial.println(Comms::macToString(selfIdentity.mac));
    Serial.println("OTA service started");

    // Initialize motor outputs
    if (!Motor::init(PINS_LEFT_FRONT, PINS_LEFT_REAR, PINS_RIGHT_FRONT, PINS_RIGHT_REAR)) {
        Serial.println("Motor init failed");
        while (true) {
            beep(2000, 500);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ArmControl::setBaseTargetDegrees(180);
    ArmControl::setExtensionTargetCentimeters(11);

    ArmServos::setTargetDegrees(ArmServos::ServoId::Elbow,90);
    ArmServos::setTargetDegrees(ArmServos::ServoId::Shoulder,90);
    ArmServos::setTargetDegrees(ArmServos::ServoId::Pitch,90);
    ArmServos::setTargetDegrees(ArmServos::ServoId::Yaw,90);
    ArmServos::setTargetDegrees(ArmServos::ServoId::Roll,90);

    Motor::calibrate();
    // ensure motors are disarmed after calibration
    ThegillCommand initialCommand = loadCommandSnapshot();
    initialCommand.leftFront = 0;
    initialCommand.leftRear = 0;
    initialCommand.rightFront = 0;
    initialCommand.rightRear = 0;
    initialCommand.flags |= GILL_FLAG_BRAKE;
    storeCommandSnapshot(initialCommand);
    targetOutputs = {0, 0, 0, 0};
    Motor::update(false, false, currentOutputs, targetOutputs);
    // runMotorStartupTest();

    Serial.println("System ready for drive!");
    delay(200);

    CREATE_TASK(
        FastTask,
        "FastTask",
        FAST_TASK_STACK,
        5,
        NULL,
        1
    );

    CREATE_TASK(
        CommTask,
        "CommTask",
        COMM_TASK_STACK,
        3,
        NULL,
        0 // Pin to core 0 for Wi-Fi operations
    );

    CREATE_TASK(
        StatusLedTask,
        "StatusLED",
        STATUS_LED_TASK_STACK,
        1,
        NULL,
        1
    );

    CREATE_TASK(
        FailsafeTask,
        "FailsafeTask",
        FAILSAFE_TASK_STACK,
        1,
        NULL,
        1
    );

    CREATE_TASK(
        TelemetryTask,
        "TelemetryTask",
        TELEMETRY_TASK_STACK,
        2,
        NULL,
        0
    );

    CREATE_TASK(
        OTATask,
        "OTATask",
        OTA_TASK_STACK,
        1,
        NULL,
        0
    );
    Serial.println("Tasks are here boi!");
}
// ==================== MAIN LOOP ====================

void loop() {
    vTaskDelete(NULL); // Kill the default task
}

