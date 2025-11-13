# ThegillS3 Platform Overview

ThegillS3 is an ESP32-S3 based drive-and-manipulation controller tailored for ASCE mobile robots. It combines high-current motor outputs, an articulated arm, rich peripheral I/O, and dual-mode communications into a single firmware image (`src/main.cpp`). This document summarizes what the system can do, how it is controlled, and how to extend it.

## Core Capabilities

- **Mobility:** Four BTS7960 half-bridges driven through `Motor::update`, with calibrated PWM mixing, encoder feedback hooks, honk/brake flags, and a configurable failsafe ramp (`checkFailsafe`).
- **Manipulator:** Dual closed-loop axes (rotation and extension) plus five hobby servos controlled via `ArmControl` and `ArmServos`, remotely enableable through `ArmControlCommand`.
- **Peripherals:** Shift-register PWM for system/arm/gripper indicators, three high-power LEDs, pump output, and eight user bits exposed through `Comms::PeripheralCommand`.
- **Sensing:** Analog mux (`AnalogInputs`) for eight user-selectable analog/digital channels, dedicated Vbat (GPIO8) and leak (GPIO9) measurements, wheel encoder scaffolding, and telemetry streaming.
- **Communications:** ESP-NOW pairing, dedicated command payloads (`ThegillCommand`, `PeripheralCommand`, `ArmControlCommand`), status packets back to the controller, OTA updates, and Wi-Fi telemetry/TCP console.

## Subsystem Highlights

### Mobility Stack

- `Motor::DriverPins` map each wheel to BTS7960 direction/PWM channels (GPIO 19/20, 5/4, 18/17, 7/6).
- `commandToMotorOutputs` clamps incoming wheel vectors to ±1000 and maps them directly to the physical wheel order.
- Failsafe task zeroes motors, clears shift-register outputs, and enforces brake flags when `Comms::lastCommandTimestamp()` exceeds `FAILSAFE_TIMEOUT`.
- `filterDriveCommand` applies the configured easing mode (slew-rate or exponential) from `ConfigurationPacket`, so motion ramps can be tuned per mission without reflashing firmware.
- Encoder infrastructure (`PcintEncoder`) is initialized in `setup()` to provide future closed-loop speed estimation.

### Arm & Servo Control

- `ArmControl::update` runs PID loops for rotation and extension using sensor feedback on ADC pins 1 and 2, with per-axis inversion, deadbands, anti-windup, and output gating through `ArmControl::setOutputsEnabled`.
- `ArmServos` controls shoulder, elbow, pitch, roll, and yaw servos (GPIO 13/12/40/39/14). Arm commands can toggle servo power or set joint targets via `ArmCommandMask`.
- Shift-register outputs (`RotationPwmA`, `ExtensionPwmB`, etc.) handle H-bridge control, keeping wiring centralized.
- `ArmStatePacket` publishes the latest base/extension pose plus each servo target whenever `GillSystemCommand::RequestArmState` is set, keeping handheld dashboards synchronized without opening another telemetry channel.

### Peripherals & Lighting

- Three breathing status LEDs are rendered via `StatusLedTask`, using shift-register PWM on `SystemIndicatorLed`, `GripperIndicatorLed`, and `ArmIndicatorLed`.
- Pump intensity maps to `ShiftRegister::Output::PumpControl`, while `PeripheralCommand::ledPwm[]` drives `HighPowerLed1..3`. The `userMask` byte fans out to `ShiftRegister::writeUserMask` for user-defined accessories.
- Buzzer support (`BuzzerTask`) provides connection tones and honk chords triggered through command flags.

### Sensor Ingestion

- `AnalogInputs` abstracts the 3-bit mux select (shift-register outputs `AnalogSelect1..3`), allowing each of the eight channels to operate in analog or digital mode with per-channel thresholds.
- Helpers: `readMuxAnalog`, `readMuxDigital`, `readBatteryVoltage`, and `readLeakVoltage` supply instant readings without reimplementing select logic.
- Battery/leak scaling is controlled via `config::kBatteryDividerRatio` / `kLeakDividerRatio`, ensuring telemetry can report physical voltages once the resistor network is known.

### Communications & Control Payloads

- ESP-NOW discovery/pairing is handled inside `Comms::init`/`Comms::loop`, supporting controller/controlled roles and broadcasting identity packets.
- `Comms::PeripheralCommand` updates the three indicator LEDs, pump duty, and the eight user outputs in one shot, while `ThegillCommand` covers per-wheel drive targets, honk/brake flags, and system-level toggles (telemetry, buzzer mute, status requests).
- `Comms::StatusPacket` flows in the opposite direction, reporting wheel speeds, battery voltage, peripheral state, and system flags so the handheld controller can display rover health in real time.
- `ThegillCommand` enables direct wheel/throttle/easing control for higher-bandwidth senders; `ArmControlCommand` covers manipulator targets and enable flags.
- `ConfigurationPacket` centralizes drive easing, buzzer mute, drive/arm enablement, failsafe toggles, and the battery-protection thresholds. When the handheld already smooths wheel values it should keep `easingMode`/`easingRate` at zero so the rover forwards the commands verbatim.
- `SettingsPacket` lets UIs rename the rover, tweak the ESP-NOW custom ID, and update the SoftAP credentials on demand, while `ArmStatePacket` streams arm-axis/servo telemetry whenever `GillSystemCommand::RequestArmState` is asserted.
- OTA (ArduinoOTA) and Wi-Fi (TCP console + telemetry) run concurrently with ESP-NOW, so developers can debug over Wi-Fi while driving over ESP-NOW.

## Control Scheme Overview

1. **Link establishment:** Controller broadcasts `MSG_PAIR_REQ`. ThegillS3 replies with `MSG_IDENTITY_REPLY`, leading to ACK/CONFIRM handshakes (`Comms::onDataRecv`).
2. **Command ingestion:** Incoming commands populate `g_lastCommand`, `g_lastThegillCommand`, or `g_lastArmCommand` and are consumed inside `CommTask`.
3. **Drive application:** `applyThegillCommandPayload` ingests `ThegillCommand` packets for wheel targets/flags, while `applyPeripheralCommand` updates LED, pump, and user outputs from `PeripheralCommand`.
4. **Arm application:** `applyArmControlCommand` toggles outputs/servos and sets PID targets when `ArmCommandMask` bits are present.
5. **Failsafe:** `FailsafeTask` monitors `Comms::lastCommandTimestamp()` and ramps motors (plus clears shift-register state) when control is lost.

Developers can mix control sources (e.g., `ThegillCommand`/`PeripheralCommand` for driving + `ArmControlCommand` for manipulator) because timestamps are tracked separately.

## Example Applications

### 1. Teleoperated Sprayer

- Use a handheld controller sending `ThegillCommand` drive updates at ~50 Hz and `Comms::PeripheralCommand` packets whenever lighting/pump states change.
- Map `PeripheralCommand::userMask` bit 0 to toggle a sprayer valve connected to `ShiftRegister::User0`, while still using `ThegillCommand::flags` for honk/brake.
- Drive the pump intensity via `PeripheralCommand::pumpDuty` to modulate flow rate, and monitor the leak sensor through `AnalogInputs::readLeakVoltage` to auto-shutdown on leakage.

### 2. Inspection Rover with Sensor Pod

- Attach multiple analog sensors (temperature, gas, moisture) to the mux inputs.
- Configure each channel at startup with `AnalogInputs::configureChannel(channel, ChannelConfig{ChannelMode::Analog})`.
- Poll the mux periodically, stream readings over the TCP console, and use `PeripheralCommand::ledPwm[]` to color-code status (e.g., red LED brightens with higher gas concentration).

### 3. Assistive Manipulator & Lighting Rig

- Send `ArmControlCommand` packets from a joystick UI to position the arm for sample collection.
- Drive `HighPowerLed1..3` via `PeripheralCommand::ledPwm[]` for adaptive illumination.
- Use `PeripheralCommand::userMask` bit 2 to engage a vacuum gripper tied to `ShiftRegister::User2`, and read a limit switch via `AnalogInputs::readMuxDigital` to confirm sample capture.

## Expansion Ideas

- **Autonomy:** Combine encoder data (`PcintEncoder`) with IMU input to add velocity/pose estimation and close the loop on `Motor::update`.
- **Sensor Services:** Build a dedicated task that polls mux channels, debounces digital inputs, and publishes a structured telemetry packet for higher-level planners.
- **Power Management:** Feed `readBatteryVoltage()` into a watchdog that modulates LED colors, buzzer alerts, or pump disable thresholds based on pack health.
- **Configurable I/O:** Expose a TCP/ESP-NOW command channel to adjust mux channel modes, digital thresholds, or user-mask mappings at runtime and persist them to NVS.
- **Automation Scripts:** Layer a macro system that maps `commandByte` bits to sequences (e.g., prime pump + flash LED + run actuator) for reproducible field operations.

## Useful References

- `src/main.cpp`: Task orchestration, command handling, failsafe logic, and subsystem initialization.
- `include/comms.h` / `src/comms.cpp`: ESP-NOW protocol, packet structs, discovery/pairing, and command buffering.
- `include/shift_register.h` / `src/shift_register.cpp`: Shift-register pin map, PWM engine, and helper APIs.
- `include/analog_inputs.h` / `src/analog_inputs.cpp`: Multiplexer configuration, analog/digital reads, battery/leak helpers.
- `include/device_config.h`: Hardware pin assignments, ADC scaling constants, and PID tuning parameters.
- `src/arm_control.cpp` / `src/arm_servos.cpp`: Manipulator PID loops, servo control, and enable/disable plumbing.

With these building blocks, ThegillS3 can serve as a teleoperated rover, a mobile manipulator, or a sensor-rich automation platform—all from the same firmware base.
