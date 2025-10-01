#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include <cstring>
#include <cstdlib>
#include "comms.h"
#include "commands.h"
#include "thegill.h"
#include "motor.h"
#include "device_config.h"

// ==================== BOARD CONFIGURATION ====================
// ESP32-S3 pin mappings for BTS7960 bridge drivers and task sizes
const Motor::DriverPins PINS_LEFT_FRONT  = {6,   5, -1};
const Motor::DriverPins PINS_LEFT_REAR   = { 10, 9, -1};
const Motor::DriverPins PINS_RIGHT_FRONT = { 7,  15, -1};
const Motor::DriverPins PINS_RIGHT_REAR  = {   47, 21,  -1};
const int BUZZER_PIN = 13; // optional piezo buzzer
const uint32_t CPU_FREQ_MHZ = 240;
const uint16_t FAST_TASK_STACK = 4096;
const uint16_t COMM_TASK_STACK = 8192;
const uint16_t FAILSAFE_TASK_STACK = 2048;
const uint16_t TELEMETRY_TASK_STACK = 4096;
const uint16_t OTA_TASK_STACK = 2048;
const uint16_t BUZZER_TASK_STACK = 1024;
const uint16_t STATUS_LED_TASK_STACK = 1024;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, handle, core)


// Use LEDC channel 5 for an optional buzzer. This channel operates
// independently of the MCPWM timers driving the motors.

const int BUZZER_CHANNEL = 5;
const int STATUS_LED_PIN = config::kStatusLedPin;


/// ==================== CONSTANTS ====================
const char *WIFI_SSID = "Thegill Telemetry";
const char *WIFI_PASSWORD = "ASCEpec@2025";
const int TCP_PORT = 8000;

// Motor and control constants
const int16_t MOTOR_MIN = -1000;
const int16_t MOTOR_MAX = 1000;
const unsigned long FAILSAFE_TIMEOUT = 500;  // ms
const unsigned long TELEMETRY_INTERVAL = 50; // ms
bool failsafe_enable = false;
bool isArmed = true;

const char *DRONE_ID = "Thegill";
const uint32_t PACKET_MAGIC = THEGILL_PACKET_MAGIC;

struct BuzzerCommand {
    uint16_t durationMs;
    uint8_t freqCount;
    uint16_t freqs[3];
};

// ==================== GLOBAL VARIABLES ====================
// Hardware
WiFiServer server(TCP_PORT);
WiFiClient client;
ThegillCommand command = {THEGILL_PACKET_MAGIC, 0, 0, 0, 0, 4.0f, GillMode::Default, GillEasing::EaseInOut, 0, 0};
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
Motor::Outputs currentOutputs{0, 0, 0, 0};
Motor::Outputs targetOutputs{0, 0, 0, 0};
ThegillTelemetry gillTelemetry{};
unsigned long lastTelemetry = 0;
QueueHandle_t buzzerQueue = nullptr;

// ==================== COMMUNICATION FUNCTIONS ====================
bool telemetryEnabled = false; // Serial/TCP telemetry disabled by default
const int MAX_MESSAGE_LENGTH = 256;
char messageBuffer[MAX_MESSAGE_LENGTH + 1] = {0};
size_t messageLength = 0;
bool serialActive = false; // Tracks if a Serial session is currently open

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
    // The ELITE/ILITE control payload orders motors as front-left, front-right,
    // rear-left, rear-right.  Our Motor::Outputs structure expects them in
    // left-front, left-rear, right-front, right-rear order.  Re-map the values
    // here so every caller can work with physical motor positions.
    Motor::Outputs outputs{};
    outputs.leftFront = static_cast<int16_t>(constrain(cmd.leftFront, MOTOR_MIN, MOTOR_MAX));
    outputs.leftRear = static_cast<int16_t>(constrain(cmd.rightFront, MOTOR_MIN, MOTOR_MAX));
    outputs.rightFront = static_cast<int16_t>(constrain(cmd.leftRear, MOTOR_MIN, MOTOR_MAX));
    outputs.rightRear = static_cast<int16_t>(constrain(cmd.rightRear, MOTOR_MIN, MOTOR_MAX));
    return outputs;
}

static int16_t decodeMotionBits(uint8_t encoded, int16_t magnitude, bool &brakeRequested)
{
    switch (encoded & 0x03)
    {
    case 0x00:
        return 0;
    case 0x01:
        return -magnitude;
    case 0x02:
        return magnitude;
    case 0x03:
    default:
        brakeRequested = true;
        return 0;
    }
}

static void applyControlPacket(const Comms::ControlPacket &packet, uint32_t timestampMs)
{
    (void)timestampMs;
    ThegillCommand updated = loadCommandSnapshot();
    updated.magic = THEGILL_PACKET_MAGIC;

    const int16_t magnitude = static_cast<int16_t>(constrain(static_cast<int32_t>(abs(packet.speed)) * 10, 0L, static_cast<long>(MOTOR_MAX)));
    bool brakeRequested = false;

    auto decode = [&](uint8_t shift) {
        return decodeMotionBits((packet.motionState >> shift) & 0x03, magnitude, brakeRequested);
    };

    updated.leftFront = decode(6);
    updated.leftRear = decode(4);
    updated.rightFront = decode(2);
    updated.rightRear = decode(0);

    uint8_t flags = 0;
    if (brakeRequested || (packet.buttonStates[0] & 0x01))
    {
        flags |= GILL_FLAG_BRAKE;
    }
    if (packet.buttonStates[0] & 0x02)
    {
        flags |= GILL_FLAG_HONK;
    }
    updated.flags = flags;

    storeCommandSnapshot(updated);
}

static void applyThegillCommandPayload(const ThegillCommand &packet, uint32_t timestampMs)
{
    (void)timestampMs;
    if (packet.magic != THEGILL_PACKET_MAGIC)
    {
        return;
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
    if (BUZZER_PIN < 0 || !buzzerQueue || !freqs || count == 0)
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
    Serial.println("Motor startup self-test complete.");
}

#pragma pack(push, 1)
struct GillTelemetryPacket {
    uint32_t magic;
    float pitch;
    float roll;
    float yaw;
    float pitchCorrection;
    float rollCorrection;
    float yawCorrection;
    uint16_t throttle;
    int8_t pitchCommand;
    int8_t rollCommand;
    int8_t yawCommand;
    float altitude;
    float verticalAcc;
    uint32_t commandAge;
};
#pragma pack(pop)

void streamTelemetry()
{
    if (millis() - lastTelemetry < TELEMETRY_INTERVAL)
        return;

    lastTelemetry = millis();

    ThegillCommand currentCommand = loadCommandSnapshot();
    uint32_t nowMs = millis();
    uint32_t lastCommandMs = Comms::lastCommandTimestamp();
    uint32_t commandAge = (lastCommandMs != 0 && nowMs >= lastCommandMs) ? (nowMs - lastCommandMs) : 0;

    int32_t throttleValue = currentOutputs.rightRear + 1000;
    if (throttleValue < 0)
    {
        throttleValue = 0;
    }
    else if (throttleValue > 2000)
    {
        throttleValue = 2000;
    }

    GillTelemetryPacket packet{
        PACKET_MAGIC,
        0.0f, 0.0f, 0.0f,
        currentOutputs.leftFront / 1000.0f,
        currentOutputs.leftRear / 1000.0f,
        currentOutputs.rightFront / 1000.0f,
        static_cast<uint16_t>(throttleValue),
        static_cast<int8_t>(constrain(targetOutputs.leftFront / 8, -128, 127)),
        static_cast<int8_t>(constrain(targetOutputs.rightFront / 8, -128, 127)),
        static_cast<int8_t>(constrain(targetOutputs.leftRear / 8, -128, 127)),
        0.0f,
        0.0f,
        commandAge
    };

    Comms::LinkStatus status = Comms::getLinkStatus();
    if (status.paired && !Comms::macEqual(status.peerMac, Comms::BroadcastMac))
    {
        esp_now_send(status.peerMac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    }

    bool tcpActive = client && client.connected();
    if (!(telemetryEnabled && (serialActive || tcpActive)))
        return;

    String telemetry = "TG:0.00 0.00 0.00 " +
                       String(currentOutputs.leftFront) + " " + String(currentOutputs.leftRear) + " " +
                       String(currentOutputs.rightFront) + " " + String(currentOutputs.rightRear) + " " +
                       String(currentCommand.leftFront) + " " + String(currentCommand.leftRear) + " " +
                       String(currentCommand.rightFront) + " " + String(currentCommand.rightRear) + " " +
                       String(commandAge);

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
    uint32_t lastCommandMs = Comms::lastCommandTimestamp();
    bool commandRecent = (lastCommandMs != 0 && nowMs >= lastCommandMs && (nowMs - lastCommandMs) <= FAILSAFE_TIMEOUT);

    if (commandRecent) {
        rampingDown = false;
        return;
    }

    if (!rampingDown) {
        rampingDown = true;
        lastRampUpdate = nowMs;
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
    } else {
        rampingDown = false;
    }
}


// ==================== CONTROL LOOP ====================

void FastTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(2); // 500 Hz control loop
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

        targetOutputs = desired;
        Motor::update(isArmed, brake, currentOutputs, targetOutputs);

        gillTelemetry.targetLeftFront = desired.leftFront / 1000.0f;
        gillTelemetry.targetLeftRear = desired.leftRear / 1000.0f;
        gillTelemetry.targetRightFront = desired.rightFront / 1000.0f;
        gillTelemetry.targetRightRear = desired.rightRear / 1000.0f;
        gillTelemetry.actualLeftFront = currentOutputs.leftFront / 1000.0f;
        gillTelemetry.actualLeftRear = currentOutputs.leftRear / 1000.0f;
        gillTelemetry.actualRightFront = currentOutputs.rightFront / 1000.0f;
        gillTelemetry.actualRightRear = currentOutputs.rightRear / 1000.0f;
        gillTelemetry.easingRate = currentCommand.easingRate;
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

    enum class LedPattern : uint8_t {
        Off,
        Solid,
        BlinkSlow,
        BlinkMedium,
        BlinkFast
    };

    constexpr unsigned long kRecentCommandWindowMs = 200;
    const TickType_t slowPeriod = pdMS_TO_TICKS(1000);
    const TickType_t mediumPeriod = pdMS_TO_TICKS(500);
    const TickType_t fastPeriod = pdMS_TO_TICKS(150);
    const TickType_t pollInterval = pdMS_TO_TICKS(50);

    LedPattern previousPattern = LedPattern::Off;
    bool ledState = false;
    TickType_t lastToggle = xTaskGetTickCount();

    digitalWrite(STATUS_LED_PIN, LOW);

    while (true)
    {
        unsigned long nowMs = millis();
        uint32_t lastCommand = Comms::lastCommandTimestamp();
        bool paired = Comms::paired();
        unsigned long ageMs = lastCommand ? (nowMs - static_cast<unsigned long>(lastCommand)) : (FAILSAFE_TIMEOUT + 1UL);

        bool commandFresh = paired && lastCommand && ageMs <= kRecentCommandWindowMs;
        bool failsafeActive = paired && ageMs > FAILSAFE_TIMEOUT;
        bool idlePaired = paired && !commandFresh && !failsafeActive;

        LedPattern pattern;
        if (!paired)
        {
            pattern = LedPattern::BlinkSlow;
        }
        else if (failsafeActive)
        {
            pattern = LedPattern::BlinkFast;
        }
        else if (commandFresh)
        {
            pattern = LedPattern::Solid;
        }
        else if (idlePaired)
        {
            pattern = LedPattern::BlinkMedium;
        }
        else
        {
            pattern = LedPattern::Off;
        }

        if (pattern != previousPattern)
        {
            previousPattern = pattern;
            lastToggle = xTaskGetTickCount();
            switch (pattern)
            {
            case LedPattern::Solid:
                ledState = true;
                digitalWrite(STATUS_LED_PIN, HIGH);
                break;
            case LedPattern::Off:
            default:
                ledState = false;
                digitalWrite(STATUS_LED_PIN, LOW);
                break;
            }
        }

        TickType_t nowTicks = xTaskGetTickCount();
        auto handleBlink = [&](TickType_t period) {
            if (nowTicks - lastToggle >= period)
            {
                ledState = !ledState;
                digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
                lastToggle = nowTicks;
            }
        };

        switch (pattern)
        {
        case LedPattern::BlinkSlow:
            handleBlink(slowPeriod);
            break;
        case LedPattern::BlinkMedium:
            handleBlink(mediumPeriod);
            break;
        case LedPattern::BlinkFast:
            handleBlink(fastPeriod);
            break;
        case LedPattern::Solid:
            if (!ledState)
            {
                ledState = true;
                digitalWrite(STATUS_LED_PIN, HIGH);
            }
            break;
        case LedPattern::Off:
            if (ledState)
            {
                ledState = false;
                digitalWrite(STATUS_LED_PIN, LOW);
            }
            break;
        }

        vTaskDelay(pollInterval);
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
        ThegillCommand directCommand{};
        Comms::ControlPacket packet{};
        if (Comms::receiveThegillCommand(directCommand, &timestampMs)) {
            applyThegillCommandPayload(directCommand, timestampMs);
        } else if (Comms::receiveCommand(packet, &timestampMs)) {
            applyControlPacket(packet, timestampMs);
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
        streamTelemetry();
        vTaskDelayUntil(&lastWake, interval);
    }
}

void OTATask(void *pvParameters) {
    while (true) {
        ArduinoOTA.handle();  // OTA can be run slowly
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void setup()
{
    Serial.begin(115200);
    Serial.println("Thegill S3 controller starting...");
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    if (BUZZER_PIN >= 0) {
        // Use a standard 2 kHz buzzer tone with 8-bit resolution to avoid
        // disturbing the motor PWM timers.
        ledcSetup(BUZZER_CHANNEL, 2000, 8);
        ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
        buzzerQueue = xQueueCreate(5, sizeof(BuzzerCommand));
        CREATE_TASK(
            BuzzerTask,
            "BuzzerTask",
            BUZZER_TASK_STACK,
            1,
            NULL,
            1
        );
        beep(1000, 200);
        beep(1180, 200);
    }

    setCpuFrequencyMhz(CPU_FREQ_MHZ);

    Comms::setRole(Comms::DeviceRole::Controlled);
    Comms::setPlatform("ThegillS3");
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
}
// ==================== MAIN LOOP ====================

void loop() {
    vTaskDelete(NULL); // Kill the default task
}

