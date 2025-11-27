#include "commands.h"

#include <WiFi.h>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <cstddef>
#include <cstdint>

#include "arm_control.h"
#include "arm_servos.h"
#include "motor.h"
#include "thegill.h"
#include "device_config.h"
#include "shift_register.h"

extern WiFiClient client;
extern bool failsafe_enable;
extern bool telemetryEnabled;
extern bool serialActive;
extern bool isArmed;
extern Motor::Outputs currentOutputs;
extern Motor::Outputs targetOutputs;
extern ThegillTelemetry gillTelemetry;
extern bool armControlEnabled;

namespace {

bool parseArmTargetValue(const String &input, float &normalized) {
    String value = input;
    value.trim();
    if (value.length() == 0) {
        return false;
    }

    bool percent = false;
    if (value.endsWith("%")) {
        percent = true;
        value.remove(value.length() - 1);
        value.trim();
    }

    char *endPtr = nullptr;
    float parsed = strtof(value.c_str(), &endPtr);
    if (endPtr == value.c_str()) {
        return false;
    }
    while (*endPtr && isspace(static_cast<unsigned char>(*endPtr))) {
        ++endPtr;
    }
    if (*endPtr != '\0') {
        return false;
    }

    if (percent || parsed > 1.0f) {
        if (!percent && parsed > 1.0f && parsed <= 100.0f) {
            percent = true;
        }
        if (percent) {
            parsed = parsed / 100.0f;
        } else if (parsed > 100.0f) {
            parsed = parsed / 4095.0f;
        }
    }

    normalized = constrain(parsed, 0.0f, 1.0f);
    return true;
}

bool parseServoTargetValue(const String &input, float &degrees, float minDegrees, float maxDegrees) {
    String value = input;
    value.trim();
    if (value.length() == 0) {
        return false;
    }

    bool percent = false;
    bool explicitDegrees = false;

    if (value.endsWith("%")) {
        percent = true;
        value.remove(value.length() - 1);
        value.trim();
    } else if (value.endsWith("deg")) {
        explicitDegrees = true;
        value.remove(value.length() - 3);
        value.trim();
    } else if (value.endsWith("°")) {
        explicitDegrees = true;
        value.remove(value.length() - 1);
        value.trim();
    }

    char *endPtr = nullptr;
    float parsed = strtof(value.c_str(), &endPtr);
    if (endPtr == value.c_str()) {
        return false;
    }
    while (*endPtr && isspace(static_cast<unsigned char>(*endPtr))) {
        ++endPtr;
    }
    if (*endPtr != '\0') {
        return false;
    }

    float range = maxDegrees - minDegrees;
    if (range <= 0.0f) {
        range = 180.0f;
        minDegrees = 0.0f;
        maxDegrees = minDegrees + range;
    }

    if (percent) {
        parsed = parsed / 100.0f;
        parsed = constrain(parsed, 0.0f, 1.0f);
        degrees = minDegrees + parsed * range;
        return true;
    }

    if (!explicitDegrees && parsed >= 0.0f && parsed <= 1.0f) {
        degrees = minDegrees + parsed * range;
        return true;
    }

    degrees = constrain(parsed, minDegrees, maxDegrees);
    return true;
}

bool parseServoName(const String &raw, ArmServos::ServoId &id) {
    String name = raw;
    name.trim();
    name.toLowerCase();
    if (name == "shoulder" || name == "base") {
        id = ArmServos::ServoId::Shoulder;
        return true;
    }
    if (name == "elbow") {
        id = ArmServos::ServoId::Elbow;
        return true;
    }
    if (name == "pitch" || name == "gripper" || name == "gripperpitch") {
        id = ArmServos::ServoId::Pitch;
        return true;
    }
    if (name == "roll") {
        id = ArmServos::ServoId::Roll;
        return true;
    }
    if (name == "yaw") {
        id = ArmServos::ServoId::Yaw;
        return true;
    }
    return false;
}

void printServoStatus() {
    bool globalEnabled = ArmServos::enabled();
    String header = String("Servos: ") + (globalEnabled ? "enabled" : "disabled");
    Commands::sendLine(header);

    for (uint8_t idx = 0; idx < static_cast<uint8_t>(ArmServos::ServoId::Count); ++idx) {
        ArmServos::ServoId id = static_cast<ArmServos::ServoId>(idx);
        ArmServos::ServoStatus st = ArmServos::status(id);
        char line[160];
        snprintf(line, sizeof(line),
                 " %s  tgt=%.1f° range=[%.1f°, %.1f°] attached=%s",
                 ArmServos::name(id),
                 st.targetDegrees,
                 st.minDegrees,
                 st.maxDegrees,
                 st.attached ? "yes" : "no");
        Commands::sendLine(line);
    }
}

} // namespace

namespace Commands {

void sendLine(const String &line) {
    Serial.println(line);
    if (client && client.connected()) {
        client.println(line);
        client.flush();
    }
}

static unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;

void sendHeartbeat() {
    if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        lastHeartbeat = millis();
        sendLine("HEARTBEAT:" + String(millis()));
    }
}

void handleCommand(const String &cmd) {
    String trimmed = cmd;
    trimmed.trim();
    if (trimmed.length() == 0)
        return;

    if (trimmed == "status") {
        sendLine("System: " + String(millis()) + "ms uptime");
        sendLine("Targets LF/LR/RF/RR: " + String(targetOutputs.leftFront) + ", " +
                 String(targetOutputs.leftRear) + ", " + String(targetOutputs.rightFront) + ", " +
                 String(targetOutputs.rightRear));
        sendLine("Actual  LF/LR/RF/RR: " + String(currentOutputs.leftFront) + ", " +
                 String(currentOutputs.leftRear) + ", " + String(currentOutputs.rightFront) + ", " +
                 String(currentOutputs.rightRear));
        sendLine(String("Armed: ") + (isArmed ? "yes" : "no") +
                 String(" Brake: ") + (gillTelemetry.brakeActive ? "on" : "off") +
                 String(" Honk: ") + (gillTelemetry.honkActive ? "yes" : "no") +
                 String(" PWM: ") + (Motor::dynamicFrequencyEnabled() ? "dynamic" : "fixed"));
    } else if (trimmed.equalsIgnoreCase("enc") || trimmed.equalsIgnoreCase("encoders")) {
        Motor::EncoderMeasurement readings[config::kMotorCount];
        std::size_t count = Motor::encoderMeasurements(readings, config::kMotorCount);
        if (count == 0) {
            sendLine("Encoders: no samples available");
        } else {
            char line[196];
            for (std::size_t idx = 0; idx < count; ++idx) {
                const auto &m = readings[idx];
                snprintf(line, sizeof(line),
                         "Enc%u ticks=%ld dist=%.3fm speed=%.3fm/s %s",
                         static_cast<unsigned>(idx),
                         static_cast<long>(m.totalTicks),
                         m.metersTravelled,
                         m.metersPerSecond,
                         m.valid ? "" : "(inactive)");
                sendLine(line);
            }
        }
    } else if (trimmed == "failsafe on") {
        failsafe_enable = true;
        sendLine("Enabled failsafe mode");
    } else if (trimmed == "failsafe off") {
        failsafe_enable = false;
        sendLine("Disabled failsafe mode");
    } else if (trimmed == "telemetry on") {
        telemetryEnabled = true;
        sendLine("ACK: Telemetry enabled");
    } else if (trimmed == "telemetry off") {
        telemetryEnabled = false;
        sendLine("ACK: Telemetry disabled");
    } else if (trimmed.equalsIgnoreCase("pwm dyn on") || trimmed.equalsIgnoreCase("pwm dynamic on")) {
        Motor::setDynamicFrequencyEnabled(true);
        sendLine("ACK: Dynamic motor PWM enabled");
    } else if (trimmed.equalsIgnoreCase("pwm dyn off") || trimmed.equalsIgnoreCase("pwm dynamic off")) {
        Motor::setDynamicFrequencyEnabled(false);
        sendLine("ACK: Dynamic motor PWM disabled (fixed frequency)");
    } else if (trimmed.equalsIgnoreCase("pwm dyn") || trimmed.equalsIgnoreCase("pwm dynamic")) {
        sendLine(String("PWM mode: ") + (Motor::dynamicFrequencyEnabled() ? "dynamic" : "fixed"));
    } else if (trimmed == "ping") {
        sendLine("PONG");
    } else if (trimmed == "arm") {
        isArmed = true;
        Motor::update(isArmed, false, currentOutputs, targetOutputs);
        sendLine("ACK: Motors armed");
    } else if (trimmed == "disarm") {
        isArmed = false;
        Motor::update(isArmed, false, currentOutputs, targetOutputs);
        sendLine("ACK: Motors disarmed");
    } else if (trimmed.startsWith("arm ")) {
        String args = trimmed.substring(4);
        args.trim();
        if (args.length() == 0 || args.equalsIgnoreCase("help")) {
            sendLine("Arm commands: arm status | arm enable | arm disable | arm base <value> | arm extend <value>");
            sendLine("Targets accept 0-1, percentage (e.g. 45%), or raw ADC up to 4095.");
            return;
        }

        String lower = args;
        lower.toLowerCase();

        if (lower == "status") {
            ArmControl::Status status = ArmControl::status();
            String headline = String("Arm outputs: ") +
                              (ArmControl::outputsEnabled() ? "enabled" : "disabled") +
                              String(" (user ") + (armControlEnabled ? "enabled" : "disabled") + ")";
            sendLine(headline);

            char line[160];
            snprintf(line, sizeof(line),
                     " Base    tgt=%.3f pos=%.3f raw=%.3f adc=%u effort=%.3f active=%s",
                     status.base.target,
                     status.base.position,
                     status.base.raw,
                     status.base.adc,
                     status.base.effort,
                     status.base.active ? "yes" : "no");
            sendLine(line);
            snprintf(line, sizeof(line),
                     " Extend  tgt=%.3f pos=%.3f raw=%.3f adc=%u effort=%.3f active=%s",
                     status.extension.target,
                     status.extension.position,
                     status.extension.raw,
                     status.extension.adc,
                     status.extension.effort,
                     status.extension.active ? "yes" : "no");
            sendLine(line);
            printServoStatus();
            return;
        }

        if (lower == "enable") {
            armControlEnabled = true;
            ArmControl::setOutputsEnabled(true);
            ArmServos::setEnabled(true);
            sendLine("ACK: Arm outputs enabled");
            return;
        }

        if (lower == "disable") {
            armControlEnabled = false;
            ArmControl::setOutputsEnabled(false);
            ArmServos::setEnabled(false);
            sendLine("ACK: Arm outputs disabled");
            return;
        }

        if (lower.startsWith("base")) {
            String value = args.substring(4);
            value.trim();
            float target = 0.0f;
            if (!parseArmTargetValue(value, target)) {
                sendLine("ERROR: Invalid base target value");
                return;
            }
            ArmControl::setBaseTargetNormalized(target);
            char buffer[96];
            snprintf(buffer, sizeof(buffer), "ACK: Base target set to %.3f", target);
            sendLine(buffer);
            return;
        }

        if (lower.startsWith("extend") || lower.startsWith("extension")) {
            size_t prefixLen = lower.startsWith("extension") ? 9 : 6;
            String value = args.substring(prefixLen);
            value.trim();
            float target = 0.0f;
            if (!parseArmTargetValue(value, target)) {
                sendLine("ERROR: Invalid extension target value");
                return;
            }
            ArmControl::setExtensionTargetNormalized(target);
            char buffer[96];
            snprintf(buffer, sizeof(buffer), "ACK: Extension target set to %.3f", target);
            sendLine(buffer);
            return;
        }

        sendLine("ERROR: Unknown arm command");
    } else if (trimmed.startsWith("servo")) {
        String args = trimmed.substring(5);
        args.trim();
        if (args.length() == 0 || args.equalsIgnoreCase("help")) {
            sendLine("Servo commands: servo status | servo enable | servo disable | servo <name> <value>");
            sendLine("Values accept degrees (e.g. 90 or 45deg), percentages, or 0-1 normalized.");
            return;
        }

        String lower = args;
        lower.toLowerCase();
        if (lower == "status") {
            printServoStatus();
            return;
        }
        if (lower == "enable") {
            ArmServos::setEnabled(true);
            sendLine("ACK: Servos enabled");
            return;
        }
        if (lower == "disable") {
            ArmServos::setEnabled(false);
            sendLine("ACK: Servos disabled");
            return;
        }

        int space = args.indexOf(' ');
        if (space < 0) {
            sendLine("ERROR: Missing servo value");
            return;
        }
        String nameToken = args.substring(0, space);
        String valueToken = args.substring(space + 1);
        valueToken.trim();

        ArmServos::ServoId id;
        if (!parseServoName(nameToken, id)) {
            sendLine("ERROR: Unknown servo name");
            return;
        }

        ArmServos::ServoStatus st = ArmServos::status(id);
        float degrees = st.targetDegrees;
        if (!parseServoTargetValue(valueToken, degrees, st.minDegrees, st.maxDegrees)) {
            sendLine("ERROR: Invalid servo target value");
            return;
        }

        if (!ArmServos::setTargetDegrees(id, degrees)) {
            sendLine("ERROR: Failed to set servo");
            return;
        }

        char buffer[128];
        snprintf(buffer, sizeof(buffer), "ACK: %s target set to %.1f°",
                 ArmServos::name(id), degrees);
        sendLine(buffer);
    } else {
        sendLine("ERROR: Unknown command");
    }
}

} // namespace Commands
