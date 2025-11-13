# ESP-NOW Protocol Reference

This note captures every packet exchanged between an ILITE handheld controller (ESP-NOW *controller* role) and a ThegillS3 rover (ESP-NOW *controlled* role). All payloads are little-endian and marked `#pragma pack(push, 1)` inside the firmware.

## Identity and Pairing

Each ESP-NOW packet begins with a `Packet` envelope:

| Field | Size | Notes |
| --- | --- | --- |
| `version` | 1 | `PROTOCOL_VERSION` (currently `1`). |
| `type` | 1 | `MessageType` (`MSG_PAIR_REQ`, `MSG_IDENTITY_REPLY`, `MSG_PAIR_CONFIRM`, `MSG_PAIR_ACK`). |
| `id` | 60 | `Identity` (device type, platform, custom ID, MAC address). |
| `monotonicMs` | 4 | Milliseconds since boot of the sender. |
| `reserved` | 4 | Reserved for future use (0 for now). |

Pairing flow:

1. **Discovery** – The controller broadcasts `MSG_PAIR_REQ`. Every rover that hears it responds with `MSG_IDENTITY_REPLY` so the operator can pick a target.
2. **Selection** – The controller chooses one MAC and unicasts `MSG_PAIR_CONFIRM`.
3. **Acknowledgement** – The rover replies with `MSG_PAIR_ACK`. Both peers now treat that MAC as the active link.

All discovery traffic uses the broadcast MAC `FF:FF:FF:FF:FF:FF`. Either side can restart pairing by broadcasting another `MSG_PAIR_REQ` or `MSG_IDENTITY_REPLY`.

## Command Packets

### Drive Command (`ThegillCommand`)

```cpp
struct ThegillCommand {
    uint32_t magic;      // THEGILL_PACKET_MAGIC (0x54474C4C, "TGLL")
    int16_t leftFront;   // -1000..1000
    int16_t leftRear;
    int16_t rightFront;
    int16_t rightRear;
    uint8_t flags;       // bit0 brake, bit1 honk
    uint8_t system;      // GillSystemCommand bitfield
};
```

If any motor command exceeds ±1000 it is clamped. Packets are ignored unless `magic == THEGILL_PACKET_MAGIC`. Each wheel channel maps directly to the physical wheel on the rover (front-left, rear-left, front-right, rear-right respectively); the legacy GillMode drive abstraction has been removed now that the controller can send individual wheel targets.

`system` adds controller → rover toggles:

| Bit | Meaning |
| --- | --- |
| `EnableTelemetry (0x01)` | Turn on TCP/Serial telemetry streaming. |
| `DisableTelemetry (0x02)` | Turn telemetry streaming off. |
| `EnableBuzzer (0x04)` | Allow buzzer tones/alerts. |
| `DisableBuzzer (0x08)` | Mute the buzzer immediately. |
| `RequestStatus (0x10)` | Force an immediate `StatusPacket` push. |
| `RequestArmState (0x20)` | Ask the rover to send a single `ArmStatePacket` with arm axis/servo telemetry. |

Multiple bits can be OR'd together in one command; leave unused bits at `0` for forward compatibility.

### Configuration Packet (`ConfigurationPacket`)

```cpp
constexpr uint32_t THEGILL_CONFIG_MAGIC = 0x54474346; // 'TGCF'

struct ConfigurationPacket {
    uint32_t magic;
    uint8_t easingMode;   // DriveEasingMode enum
    uint8_t easingRate;   // Interpretation defined by easingMode
    uint8_t controlFlags; // ConfigFlag bits
    uint8_t safetyFlags;  // SafetyFlag bits
    uint16_t batteryCutoffMillivolts;
    uint16_t batteryRecoverMillivolts;
};
```

The configuration packet moves all one-off toggles and drive-smoothing parameters out of the recurring control stream:

- **`easingMode`** selects how the rover smooths incoming wheel targets:
  - `0` (`None`): no easing, values are applied immediately.
  - `1` (`SlewRate`): limit the change rate. Each `easingRate` unit represents ~50 motor counts/second (so `5` ≈ 250 counts/sec, `20` ≈ 1 000 counts/sec).
  - `2` (`Exponential`): first-order low-pass filter where `easingRate/255.0` is the immediate weighting (`0` = no filtering, `255` = heavy smoothing).
- **`easingRate`** scales the active easing mode. Set both `easingMode` and `easingRate` to `0` when the handheld already applies its own easing, so the rover relays values verbatim.
- **`controlFlags`** (bitfield):

| Bit | Meaning |
| --- | --- |
| `MuteAudio (0x01)` | Keep the buzzer silent until cleared. |
| `DriveEnabled (0x02)` | When `0`, the rover disarms motors and applies the brake. |
| `ArmOutputsEnable (0x04)` | Gate the arm H-bridges and servos. |
| `FailsafeEnable (0x08)` | Enables/disables the link-loss failsafe task. |

- **`safetyFlags`** (bitfield):

| Bit | Meaning |
| --- | --- |
| `BatterySafetyEnable (0x01)` | Monitor pack voltage and disarm if it drops below the configured cutoff. |

- **Battery thresholds**: `batteryCutoffMillivolts` trips the battery safety latch (motors disarm, brake asserted). `batteryRecoverMillivolts` clears the latch once voltage recovers; if it is ≤ cutoff, the firmware automatically sets it to `cutoff + 200 mV`.

### Settings Packet (`SettingsPacket`)

```cpp
constexpr uint32_t THEGILL_SETTINGS_MAGIC = 0x54475350; // 'TGSP'

struct SettingsPacket {
    uint32_t magic;
    char robotName[16];
    char customId[32];
    char wifiSsid[32];
    char wifiPassword[32];
};
```

Settings packets update operator-facing strings without rebuilding firmware:

- `robotName` is applied to the ESP-NOW identity/platform name (and displayed during pairing).
- `customId` becomes the unique ID reported in discovery packets.
- `wifiSsid` / `wifiPassword` reconfigure the SoftAP used for OTA/TCP telemetry.

Each field is treated as a fixed-length ASCII blob; empty strings leave the existing value unchanged. After the SSID/password change, the SoftAP restarts with the new credentials.

### Arm State Packet (`ArmStatePacket`)

```cpp
constexpr uint32_t THEGILL_ARM_STATE_MAGIC = 0x54474153; // 'TGAS'

struct ArmStatePacket {
    uint32_t magic;
    float baseDegrees;
    float extensionCentimeters;
    float servoDegrees[5];    // Shoulder, Elbow, Pitch, Roll, Yaw in order
    uint8_t servoEnabledMask; // Bit-per-servo if the target is enabled
    uint8_t servoAttachedMask;// Bit-per-servo if the PWM signal is attached
    uint8_t flags;            // bit0 -> ArmControl outputs enabled, bit1 -> servos globally enabled
};
```

The rover sends an `ArmStatePacket` whenever it sees `GillSystemCommand::RequestArmState`. It captures the latest PID-derived base/extension readings plus each hobby-servo target, enabling the controller UI to mirror the actual arm pose.

### Manipulator Command (`ArmControlCommand`)

```cpp
struct ArmControlCommand {
    uint32_t magic;                 // 0x54474152 ("TGAR")
    float extensionMillimeters;
    float shoulderDegrees;
    float elbowDegrees;
    float pitchDegrees;
    float rollDegrees;
    float yawDegrees;
    float gripper1Degrees;
    float gripper2Degrees;
    uint16_t validMask;             // ArmCommandMask bits
    uint8_t flags;                  // ArmCommandFlag bits
};
```

- Set the relevant `ArmCommandMask` bit for each value you want applied.
- Use `ArmCommandFlag::EnableOutputs/DisableOutputs` (and the servo equivalents) to power down the arm safely.
- The rover will also honor broadcast arm packets, enabling “one-to-many” teleoperation rigs.

### Peripheral Command (`PeripheralCommand`)

```cpp
constexpr uint32_t THEGILL_PERIPHERAL_MAGIC = 0x54475043; // 'TGPC'

struct PeripheralCommand {
    uint32_t magic;     // must be THEGILL_PERIPHERAL_MAGIC
    uint8_t  ledPwm[3]; // System / gripper / arm indicators (0-255)
    uint8_t  pumpDuty;  // Pump PWM (0-255)
    uint8_t  userMask;  // Bit 0 -> User0 ... Bit 7 -> User7 shift-register outputs
    uint8_t  reserved[3];
};
```

Every packet fully overwrites the LED trio, pump duty, and user-mask bits, so always transmit the desired steady-state values.

## Status Packet (Rover → Controller)

The rover periodically returns its health via `StatusPacket` (every `TELEMETRY_INTERVAL`, 50 ms, whenever the link is paired):

```cpp
constexpr uint32_t THEGILL_STATUS_MAGIC = 0x54475354; // 'TGST'

struct StatusPacket {
    uint32_t magic;
    int16_t wheelSpeedMmPerSec[4];
    uint16_t batteryMillivolts;
    uint8_t ledPwm[3];
    uint8_t pumpDuty;
    uint8_t userMask;
    uint8_t flags;        // StatusFlag bits
    uint16_t commandAgeMs;
};
```

`StatusFlag` bits:

| Bit | Meaning |
| --- | --- |
| `ArmOutputsEnabled (0x01)` | Arm H-bridges currently enabled. |
| `FailsafeArmed (0x02)` | Link-loss failsafe is active. |
| `TelemetryStreaming (0x04)` | Console telemetry streaming is enabled. |
| `TcpClientActive (0x08)` | Wi-Fi console has an active TCP client. |
| `SerialActive (0x10)` | USB serial console is active. |
| `PairedLink (0x20)` | ESP-NOW link is paired. |
| `BatterySafeActive (0x40)` | Battery protection latch is holding the drivetrain. |
| `DriveArmed (0x80)` | Motors are currently armed and can accept new targets. |

Wheel speeds come from the encoder pipeline (in mm/s). Battery voltage is already scaled to the real pack voltage (ADC reading × 15.15). The `commandAgeMs` field gives the age of the most recent command (capped at 65 535 ms).

## Discovery Recap

1. Controller broadcasts `MSG_PAIR_REQ`.
2. Rovers respond with `MSG_IDENTITY_REPLY` (device type, platform, ID, MAC).
3. Controller picks a MAC and sends `MSG_PAIR_CONFIRM`.
4. Rover acknowledges with `MSG_PAIR_ACK`.
5. Once paired, the controller sends `ThegillCommand`, `ArmControlCommand`, and `PeripheralCommand` frames. The rover responds with `StatusPacket` frames.

## Recommended Update Rates

| Packet | Direction | Typical Rate |
| --- | --- | --- |
| `ThegillCommand` | Controller → Rover | 50–100 Hz |
| `PeripheralCommand` | Controller → Rover | On change (≤ 10 Hz) |
| `ArmControlCommand` | Controller → Rover | As needed (10–25 Hz during motion) |
| `StatusPacket` | Rover → Controller | 20 Hz (fixed by firmware) |

The Wi‑Fi TCP console and USB Serial terminal still provide textual diagnostics when you issue `telemetry on` through the CLI, but all machine-to-machine control is expected to use the ESP‑NOW packets described above.
