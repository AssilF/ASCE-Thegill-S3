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
#include "comms.h"
#include "commands.h"
#include "thegill.h"
#include "motor.h"

// ==================== BOARD CONFIGURATION ====================
// ESP32-S3 pin mappings for BTS7960 bridge drivers and task sizes
const Motor::DriverPins PINS_LEFT_FRONT  = {4, 5, -1};
const Motor::DriverPins PINS_LEFT_REAR   = {6, 7, -1};
const Motor::DriverPins PINS_RIGHT_FRONT = {8, 9, -1};
const Motor::DriverPins PINS_RIGHT_REAR  = {10, 11, -1};
const int BUZZER_PIN = 12; // optional piezo buzzer
const uint32_t CPU_FREQ_MHZ = 240;
const uint16_t FAST_TASK_STACK = 4096;
const uint16_t COMM_TASK_STACK = 8192;
const uint16_t FAILSAFE_TASK_STACK = 2048;
const uint16_t TELEMETRY_TASK_STACK = 4096;
const uint16_t OTA_TASK_STACK = 2048;
const uint16_t BUZZER_TASK_STACK = 1024;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, handle, core)


// Use LEDC channel 5 for an optional buzzer. This channel operates
// independently of the MCPWM timers driving the motors.

const int BUZZER_CHANNEL = 5;


/// ==================== CONSTANTS ====================
const char *WIFI_SSID = "Thegill Telemetry";
const char *WIFI_PASSWORD = "ASCEpec@2025";
const int TCP_PORT = 8000;

// Motor and control constants
const int16_t MOTOR_MIN = -1000;
const int16_t MOTOR_MAX = 1000;
const unsigned long FAILSAFE_TIMEOUT = 500;  // ms
const unsigned long TELEMETRY_INTERVAL = 50; // ms
bool failsafe_enable = true;
bool isArmed = false;

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
portMUX_TYPE commsStateMux = portMUX_INITIALIZER_UNLOCKED;
Motor::Outputs currentOutputs{0, 0, 0, 0};
Motor::Outputs targetOutputs{0, 0, 0, 0};
ThegillTelemetry gillTelemetry{};
unsigned long lastTelemetry = 0;
QueueHandle_t buzzerQueue = nullptr;

// ==================== COMMUNICATION FUNCTIONS ====================
// Time in ms before we consider the controller disconnected
const unsigned long CONNECTION_TIMEOUT = 1000;
// Minimum delay between handshake responses to avoid spamming
const unsigned long HANDSHAKE_COOLDOWN = 500;
unsigned long lastHandshakeSent = 0;
bool telemetryEnabled = false; // Serial/TCP telemetry disabled by default
const int MAX_MESSAGE_LENGTH = 256;
char messageBuffer[MAX_MESSAGE_LENGTH + 1] = {0};
size_t messageLength = 0;
bool serialActive = false; // Tracks if a Serial session is currently open

// Dynamic ESP-NOW pairing
static uint8_t iliteMac[6] = {0};
uint8_t selfMac[6];
static uint8_t commandPeer[6] = {0};
static bool ilitePaired = false;
static bool commandPeerSet = false;
static uint32_t lastCommandTimeMs = 0;
unsigned long lastDiscoveryTime = 0;


// ==================== IMPROVED COMMUNICATION FUNCTIONS ====================

struct LinkStateSnapshot {
    bool ilitePaired = false;
    bool commandPeerSet = false;
    uint8_t iliteMac[6] = {0};
    uint8_t commandPeer[6] = {0};
    uint32_t lastCommandTimeMs = 0;
};

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

static inline LinkStateSnapshot loadLinkStateSnapshot()
{
    LinkStateSnapshot snapshot;
    portENTER_CRITICAL(&commsStateMux);
    snapshot.ilitePaired = ilitePaired;
    snapshot.commandPeerSet = commandPeerSet;
    memcpy(snapshot.iliteMac, iliteMac, sizeof(iliteMac));
    memcpy(snapshot.commandPeer, commandPeer, sizeof(commandPeer));
    snapshot.lastCommandTimeMs = lastCommandTimeMs;
    portEXIT_CRITICAL(&commsStateMux);
    return snapshot;
}

static inline void setLastCommandTimeMs(uint32_t value)
{
    portENTER_CRITICAL(&commsStateMux);
    lastCommandTimeMs = value;
    portEXIT_CRITICAL(&commsStateMux);
}

static inline void setLastCommandTimeMsFromISR(uint32_t value)
{
    portENTER_CRITICAL_ISR(&commsStateMux);
    lastCommandTimeMs = value;
    portEXIT_CRITICAL_ISR(&commsStateMux);
}

static inline void setIlitePeerFromISR(const uint8_t *mac)
{
    portENTER_CRITICAL_ISR(&commsStateMux);
    memcpy(iliteMac, mac, sizeof(iliteMac));
    ilitePaired = true;
    portEXIT_CRITICAL_ISR(&commsStateMux);
}

static inline bool copyIliteMacFromISR(uint8_t dest[6])
{
    portENTER_CRITICAL_ISR(&commsStateMux);
    bool paired = ilitePaired;
    if (paired)
    {
        memcpy(dest, iliteMac, sizeof(iliteMac));
    }
    portEXIT_CRITICAL_ISR(&commsStateMux);
    return paired;
}

static inline bool updateCommandPeerFromISR(const uint8_t *mac)
{
    portENTER_CRITICAL_ISR(&commsStateMux);
    bool changed = !commandPeerSet || memcmp(commandPeer, mac, sizeof(commandPeer)) != 0;
    if (changed)
    {
        memcpy(commandPeer, mac, sizeof(commandPeer));
        commandPeerSet = true;
    }
    portEXIT_CRITICAL_ISR(&commsStateMux);
    return changed;
}

static inline LinkStateSnapshot clearLinkState()
{
    LinkStateSnapshot previous;
    portENTER_CRITICAL(&commsStateMux);
    previous.ilitePaired = ilitePaired;
    previous.commandPeerSet = commandPeerSet;
    memcpy(previous.iliteMac, iliteMac, sizeof(iliteMac));
    memcpy(previous.commandPeer, commandPeer, sizeof(commandPeer));
    previous.lastCommandTimeMs = lastCommandTimeMs;
    ilitePaired = false;
    commandPeerSet = false;
    memset(iliteMac, 0, sizeof(iliteMac));
    memset(commandPeer, 0, sizeof(commandPeer));
    lastCommandTimeMs = 0;
    portEXIT_CRITICAL(&commsStateMux);
    return previous;
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
        Motor::update(true, currentOutputs, targetOutputs);
        delay(stepDelayMs);
    }

    delay(200);

    for (int16_t value = maxTestOutput; value >= 0; value -= step)
    {
        outputs.leftFront = outputs.leftRear = outputs.rightFront = outputs.rightRear = value;
        targetOutputs = outputs;
        Motor::update(true, currentOutputs, targetOutputs);
        delay(stepDelayMs);
    }

    targetOutputs = {0, 0, 0, 0};
    Motor::update(false, currentOutputs, targetOutputs);
    Serial.println("Motor startup self-test complete.");
}

void streamTelemetry()
{
    if (millis() - lastTelemetry < TELEMETRY_INTERVAL)
        return;

    lastTelemetry = millis();

    ThegillCommand currentCommand = loadCommandSnapshot();
    LinkStateSnapshot linkState = loadLinkStateSnapshot();
    uint32_t commandAge = (linkState.lastCommandTimeMs > 0) ? (millis() - linkState.lastCommandTimeMs) : 0;

    Comms::TelemetryPacket packet = {
        PACKET_MAGIC,
        0.0f, 0.0f, 0.0f,
        currentOutputs.leftFront / 1000.0f,
        currentOutputs.leftRear / 1000.0f,
        currentOutputs.rightFront / 1000.0f,
        static_cast<uint16_t>(currentOutputs.rightRear + 1000),
        static_cast<int8_t>(constrain(targetOutputs.leftFront / 8, -128, 127)),
        static_cast<int8_t>(constrain(targetOutputs.rightFront / 8, -128, 127)),
        static_cast<int8_t>(constrain(targetOutputs.leftRear / 8, -128, 127)),
        0.0f,
        commandAge
    };

    // Send telemetry to ILITE ground station if paired
    if (linkState.ilitePaired)
    {
        esp_now_send(linkState.iliteMac, (uint8_t *)&packet, sizeof(packet));
    }

    // Also send to the last command peer if different
    if (linkState.commandPeerSet && (!linkState.ilitePaired || memcmp(linkState.commandPeer, linkState.iliteMac, 6) != 0))
    {
        esp_now_send(linkState.commandPeer, (uint8_t *)&packet, sizeof(packet));
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

// Detect dropped connections and cleanup peers
void monitorConnection() {
    LinkStateSnapshot state = loadLinkStateSnapshot();
    if (!state.ilitePaired)
        return;

    uint32_t now = millis();
    if (state.lastCommandTimeMs > 0 && now - state.lastCommandTimeMs > CONNECTION_TIMEOUT)
    {
        LinkStateSnapshot cleared = clearLinkState();
        if (cleared.ilitePaired)
        {
            esp_now_del_peer(cleared.iliteMac);
        }
    }
}

// ==================== ESP-NOW CALLBACK ====================
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(Comms::IdentityMessage))
    {
        Comms::IdentityMessage msg;
        memcpy(&msg, incomingData, sizeof(msg));
        unsigned long now = millis();
        if (msg.type == Comms::SCAN_REQUEST)
        {
            if (now - lastHandshakeSent > HANDSHAKE_COOLDOWN)
            {
                if (!esp_now_is_peer_exist(mac))
                {
                    esp_now_peer_info_t peerInfo = {};
                    memcpy(peerInfo.peer_addr, mac, 6);
                    peerInfo.channel = 0;
                    peerInfo.encrypt = false;
                    esp_now_add_peer(&peerInfo);
                }
                Comms::IdentityMessage resp = {};
                resp.type = Comms::DRONE_IDENTITY;
                strncpy(resp.identity, DRONE_ID, sizeof(resp.identity));
                memcpy(resp.mac, selfMac, 6);
                esp_now_send(mac, (uint8_t *)&resp, sizeof(resp));
                lastHandshakeSent = now;
            }
        }


        else if (msg.type == Comms::ILITE_IDENTITY)
        {
            uint8_t existingMac[6];
            bool wasPaired = copyIliteMacFromISR(existingMac);
            bool isNewPeer = !wasPaired || memcmp(existingMac, msg.mac, sizeof(existingMac)) != 0;
            if (isNewPeer)
            {
                setIlitePeerFromISR(msg.mac);
                updateCommandPeerFromISR(msg.mac);
            }
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, msg.mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (!esp_now_is_peer_exist(msg.mac))
            {
                esp_now_add_peer(&peerInfo);
            }
            Comms::IdentityMessage ack = {};
            ack.type = Comms::DRONE_ACK;
            strncpy(ack.identity, DRONE_ID, sizeof(ack.identity));
            memcpy(ack.mac, selfMac, 6);
            esp_now_send(msg.mac, (uint8_t *)&ack, sizeof(ack));
            lastHandshakeSent = now;
            if (BUZZER_PIN >= 0 && isNewPeer)
            {
                beep(2000, 200); // short beep on pairing
            }
        }

        return;
    }

    if (len == sizeof(ThegillCommand))
    {
        ThegillCommand incoming;
        memcpy(&incoming, incomingData, sizeof(incoming));

        // Ignore commands from unknown devices or with wrong magic
        uint8_t pairedMac[6];
        bool paired = copyIliteMacFromISR(pairedMac);
        if (!paired || memcmp(mac, pairedMac, 6) != 0 || incoming.magic != THEGILL_PACKET_MAGIC)
        {
            return;
        }

        storeCommandSnapshotFromISR(incoming);
        uint32_t nowMs = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
        setLastCommandTimeMsFromISR(nowMs);

        bool peerChanged = updateCommandPeerFromISR(mac);
        if (peerChanged)
        {
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (!esp_now_is_peer_exist(mac))
            {
                esp_now_add_peer(&peerInfo);
            }
        }
    }
}

void checkFailsafe() {
    if (!failsafe_enable)
        return;

    static bool rampingDown = false;
    static uint32_t lastRampUpdate = 0;
    LinkStateSnapshot linkState = loadLinkStateSnapshot();
    uint32_t nowMs = millis();
    uint32_t lastCommandMs = linkState.lastCommandTimeMs;

    if (lastCommandMs != 0 && nowMs - lastCommandMs <= FAILSAFE_TIMEOUT) {
        rampingDown = false;
        return;
    }

    if (!rampingDown)
    {
        rampingDown = true;
        lastRampUpdate = nowMs;
    }

    const uint32_t rampIntervalMs = 50;
    if (nowMs - lastRampUpdate < rampIntervalMs)
        return;
    lastRampUpdate = nowMs;

    ThegillCommand safe = loadCommandSnapshot();
    bool changed = false;

    auto rampValue = [&](int16_t &value) {
        int16_t original = value;
        if (value > 0) {
            int16_t next = value - 20;
            value = next < 0 ? 0 : next;
        } else if (value < 0) {
            int16_t next = value + 20;
            value = next > 0 ? 0 : next;
        }
        if (value != original)
            changed = true;
    };

    rampValue(safe.leftFront);
    rampValue(safe.leftRear);
    rampValue(safe.rightFront);
    rampValue(safe.rightRear);

    if (changed) {
        storeCommandSnapshot(safe);
        Motor::Outputs newOutputs{safe.leftFront, safe.leftRear, safe.rightFront, safe.rightRear};
        targetOutputs = newOutputs;
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

        Motor::Outputs desired{
            static_cast<int16_t>(constrain(currentCommand.leftFront, MOTOR_MIN, MOTOR_MAX)),
            static_cast<int16_t>(constrain(currentCommand.leftRear, MOTOR_MIN, MOTOR_MAX)),
            static_cast<int16_t>(constrain(currentCommand.rightFront, MOTOR_MIN, MOTOR_MAX)),
            static_cast<int16_t>(constrain(currentCommand.rightRear, MOTOR_MIN, MOTOR_MAX))
        };

        bool brake = (currentCommand.flags & GILL_FLAG_BRAKE) != 0;
        bool honkFlag = (currentCommand.flags & GILL_FLAG_HONK) != 0;
        if (!isArmed || brake) {
            desired.leftFront = 0;
            desired.leftRear = 0;
            desired.rightFront = 0;
            desired.rightRear = 0;
        }

        targetOutputs = desired;
        Motor::update(isArmed && !brake, currentOutputs, targetOutputs);

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

void CommTask(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(5);
    while (true) {
        handleIncomingData();
        monitorConnection();
        LinkStateSnapshot state = loadLinkStateSnapshot();
        if (!state.ilitePaired && millis() - lastDiscoveryTime > 1000) {
            Comms::IdentityMessage msg = {};
            msg.type = Comms::DRONE_IDENTITY;
            strncpy(msg.identity, DRONE_ID, sizeof(msg.identity));
            memcpy(msg.mac, selfMac, 6);
            esp_now_send(Comms::BroadcastMac, (uint8_t *)&msg, sizeof(msg));
            lastDiscoveryTime = millis();
        }
        vTaskDelayUntil(&lastWake, interval); // ~200 Hz for responsiveness
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

    if (!Comms::init(WIFI_SSID, WIFI_PASSWORD, TCP_PORT, onReceive)) {
        Serial.println("ESP-NOW init failed");
        while (true) {
            beep(2000, 500);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ArduinoOTA.begin();
    server.begin();
    WiFi.macAddress(selfMac);
    Serial.println("ESP-NOW initialized");
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
    Motor::update(false, currentOutputs, targetOutputs);

    runMotorStartupTest();
    setLastCommandTimeMs(millis());

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
