#include "commands.h"
#include <WiFi.h>
#include "motor.h"
#include "thegill.h"

extern WiFiClient client;
extern float pitch, roll, yaw;
extern bool enableFilters;
extern bool enableQuadFilters;
extern bool failsafe_enable;
extern bool telemetryEnabled;
extern bool serialActive;
extern bool isArmed;
extern Motor::Outputs currentOutputs;
extern Motor::Outputs targetOutputs;
extern ThegillTelemetry gillTelemetry;
extern bool requestIMUZero();

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
        sendLine("Pitch:" + String(pitch, 2) + " Roll:" + String(roll, 2) + " Yaw:" + String(yaw, 2));
        sendLine("Targets LF/LR/RF/RR: " + String(targetOutputs.leftFront) + ", " +
                 String(targetOutputs.leftRear) + ", " + String(targetOutputs.rightFront) + ", " +
                 String(targetOutputs.rightRear));
        sendLine("Actual  LF/LR/RF/RR: " + String(currentOutputs.leftFront) + ", " +
                 String(currentOutputs.leftRear) + ", " + String(currentOutputs.rightFront) + ", " +
                 String(currentOutputs.rightRear));
        sendLine(String("Armed: ") + (isArmed ? "yes" : "no") +
                 String(" Brake: ") + (gillTelemetry.brakeActive ? "on" : "off") +
                 String(" Honk: ") + (gillTelemetry.honkActive ? "yes" : "no"));
    } else if(trimmed == "failsafe on"){failsafe_enable=1; sendLine("Enabled failsafe mode");}
    else if(trimmed == "failsafe off"){failsafe_enable=0; sendLine("Disabled failsafe mode");}
    else if(trimmed == "filters on"){enableFilters = true; sendLine("Enabled filters");}
    else if(trimmed == "filters off"){enableFilters = false; sendLine("Disabled filters");}
    else if(trimmed == "quadfilters on"){enableQuadFilters = true; sendLine("Enabled quad filters");}
    else if(trimmed == "quadfilters off"){enableQuadFilters = false; sendLine("Disabled quad filters");}
    else if (trimmed.equalsIgnoreCase("imu zero")) {
        bool queued = requestIMUZero();
        if (queued) {
            sendLine("ACK: IMU zero requested");
        } else {
            sendLine("WARN: IMU zero already pending");
        }
    } else if (trimmed == "telemetry on") {
        telemetryEnabled = true;
        sendLine("ACK: Telemetry enabled");
    } else if (trimmed == "telemetry off") {
        telemetryEnabled = false;
        sendLine("ACK: Telemetry disabled");
    } else if (trimmed == "ping") {
        sendLine("PONG");
    } else if (trimmed == "arm") {
        isArmed = true;
        Motor::update(isArmed, currentOutputs, targetOutputs);
        sendLine("ACK: Motors armed");
    } else if (trimmed == "disarm") {
        isArmed = false;
        Motor::update(isArmed, currentOutputs, targetOutputs);
        sendLine("ACK: Motors disarmed");
    } else {
        sendLine("ERROR: Unknown command");
    }
}

} // namespace Commands
