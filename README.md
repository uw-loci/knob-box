# Knob Box Firmware Overview

This repository contains the two firmware programs that implement the software behavior of the Knob Box:

- `monitor_firmware.cpp`
- `logic_arduino.cpp`

Although the Knob Box contains five Arduino Mega 2560 boards, there are only two firmware codebases:

- One shared monitoring firmware image, reused on all four monitoring Arduinos
- One dedicated logic/interlock firmware image, used only on the Logic Arduino

This README explains the software architecture of the full Knob Box system, the role of each firmware program, and how the firmware interacts with the user, the Dashboard, the Logic Arduino, the `+3 kV` monitor Arduino, the Beam Controller, CCS, and the high-voltage power supplies.

## 1. System Role

The Knob Box is both:

- A remote operator control and monitoring panel for the `+1 kV`, `-1 kV`, `+3 kV`, and `+20 kV` power supplies
- A local protection/interlock device that can remove beam permission and cut CCS power without waiting for the Dashboard

At a system level, the firmware is intentionally split into two domains:

- A slower `monitoring / display / reporting` domain
- A faster `interlock / protection / output-authority` domain

That split is central to understanding the software:

- The monitoring Arduinos measure and report
- The Logic Arduino decides whether beams and CCS are actually allowed

## 2. Firmware Inventory

### `monitor_firmware.cpp`

Used on four Arduino Mega boards, one per power supply:

- `ps_id = 1`: `+1 kV Matsusada`
- `ps_id = 2`: `-1 kV Matsusada`
- `ps_id = 3`: `+20 kV Bertan`
- `ps_id = 4`: `+3 kV Bertan`

Core jobs:

- Read `Vset`, `Vmon`, and `Imon` through an ADS1115
- Read local threshold trim pots through the Mega ADC
- Update the local `20x4` LCD
- Serve telemetry to the Dashboard over `RS-485 / Modbus RTU`
- On the Matsusada variants, infer a likely internal reset-required condition
- On the `+3 kV` variant, act as the software gateway between the Logic Arduino and the Dashboard

### `logic_arduino.cpp`

Used on one dedicated Arduino Mega board:

- The `Logic Arduino`

Core jobs:

- Sample the hardwired comparator outputs for all four HV supplies
- Sample the operator intent switches used by the interlock path
- Decide whether the system is in `Beam Interlock`, `Normal Operation`, or `3 kV Timer`
- Drive the real `CCS Power Enable`, `Arm Beams`, and `3 kV Enable` outputs
- Publish live logic state and latched interlock information to the `+3 kV` monitor Arduino

## 3. System Architecture


User
  - front-panel knobs, switches, reset button

HV Power Supplies
  - provide Vset / Vmon / Imon monitor signals to monitoring Arduinos
  - provide monitor signals to comparators for fast fault detection

Monitoring Arduinos (4x)
  - show local LCD data
  - expose telemetry to Dashboard over RS-485 / Modbus
  
+3 kV Monitoring Arduino also:
  - monitors its own supply
  - reads Logic Arduino outputs, flags, and raw switch-intent lines
  - republishes that logic state to the Dashboard via Modbus

Logic Arduino
  - reads comparators + interlock-related switches
  - controls actual CCS, Beam Enable, and 3 kV enable outputs
  - exports interlock status to the +3 kV monitor Arduino

Dashboard
  - remote visibility, polling, logging, and operator awareness
  - not the primary protection path

## 4. Who Talks To Whom

| System / Actor | Firmware interaction |
|---|---|
| User | Moves HV enable switches, `Arm 80 kV`, `Arm Beams`, `CCS Power`, and `Reset Interlocks`; adjusts voltage knobs and threshold pots; reads LCDs and panel indicators |
| Dashboard | Polls the monitoring Arduinos over `RS-485 / Modbus RTU`; receives telemetry, switch state, interlock state, and `+3 kV` logic status |
| HV power supplies | Provide analog telemetry (`Vset`, `Vmon`, `Imon`); receive enable control signals; Matsusadas also have reset handling via the panel |
| Logic Arduino | Consumes comparator and switch inputs; drives actual safety-critical outputs; publishes interlock status to the `+3 kV` monitor |
| `+3 kV` monitor Arduino | Reads Logic Arduino output mirrors, live flags, latched flags, raw switch states, and heartbeat handshake pins; republishes them to the Dashboard |
| Beam Controller / BCON | Receives the real beam-enable / beam-interlock signal from the Logic Arduino |
| CCS relay path | Receives the real CCS enable signal from the Logic Arduino |
| Glassman `+80 kV` interlock path | User-controlled arm switch is read by the Logic Arduino and is required before entering normal operation |

## 5. Monitoring Firmware Role

The monitoring firmware is the operator-facing and Dashboard-facing side of the Knob Box software.

### Common monitoring behavior

All four monitoring variants do the following:

- Sample `Vset`, `Vmon`, and `Imon` from an ADS1115
- Convert raw readings into engineering values using supply-specific full-scale ratings
- Read the current and voltage threshold trim pots from `A0` and `A1`
- Update a local `20x4` LCD
- Populate a Modbus register map
- Serve that register map on `Serial1` through the RS-485 transceiver

The current implementation uses:

- `timer.every(150, read_value)`
- `timer.every(200, display_value)`
- `slave.poll(modbus_regs, TOTAL_REG_COUNT)` in the main loop

That means the Dashboard path is polling-based and slower than the Logic Arduino interlock loop, by design.

### Supply-specific monitoring behavior

| Variant | Extra behavior |
|---|---|
| `+1 kV Matsusada` | Infers a likely Matsusada internal reset-required state and drives the reset-state LED |
| `-1 kV Matsusada` | Same reset-state inference behavior as the `+1 kV` unit |
| `+20 kV Bertan` | Uses different LCD formatting |
| `+3 kV Bertan` | Acts as the Logic Arduino's Modbus bridge to the Dashboard |

### Modbus role

The monitoring firmware is the only firmware that speaks Modbus.

Important consequence:

- The Logic Arduino does not talk directly to the Dashboard
- The Dashboard only sees Logic Arduino state because the `+3 kV` monitor Arduino reads it and republishes it

In the current code, the Modbus slave address is the `ps_id`, so the four boards are intended to appear as four addressed devices on the shared RS-485 bus.

## 6. Logic Firmware Role

The Logic Arduino is the fast local authority for beam permission and interlock enforcement.

### What it reads

The Logic Arduino reads:

- `8` comparator outputs, one current and one voltage comparator for each HV supply
- `3 kV Enable` switch
- `Arm Beams` switch
- `CCS Power Allow` switch
- `Arm 80 kV` switch
- `Reset Interlocks` button
- The flag-acknowledge line coming back from the `+3 kV` monitor Arduino

Comparator behavior is intentionally fail-safe:

- `LOW = safe`
- `HIGH = fault / interlock`
- Open or disconnected comparator wiring is treated as fault because the Logic Arduino pull-up makes the input read high

### What it controls

The Logic Arduino drives the real output authority signals:

- `A0`: `CCS Power Enable`
- `A1`: `Beam Enable` / `Arm Beams` to BCON
- `A2`: `3 kV HV Enable`

These are the outputs that matter electrically. The front-panel switches for `CCS Power` and `Arm Beams` are requests, not guarantees.

### Current state machine

| State | Behavior |
|---|---|
| `STATE_INTERLOCK` | Default safe/interlocked state; CCS off, beams off, `3 kV` output follows the `3 kV Enable` switch |
| `STATE_NOM_OP` | Only state in which `CCS Power` and `Arm Beams` requests are honored |
| `STATE_3KV_TIMER` | Special `+3 kV` lockout state; CCS off, beams off, `3 kV` forced off for `100 ms` |

### Entry / exit behavior

Current code behavior:

- Startup enters `STATE_INTERLOCK`
- Enter `STATE_NOM_OP` only when:
  - all comparators are safe
  - `Arm 80 kV` is asserted
  - `3 kV Enable` is asserted
  - the user presses `Reset Interlocks`
- Any interlock fault drops the system out of `STATE_NOM_OP`
- `3 kV` overcurrent in interlock state can retrigger the `3 kV` timer
- `3 kV` undervoltage or overcurrent in normal operation sends the system to the `3 kV` timer

This matches the operating-manual concept that the Knob Box starts interlocked, must be intentionally reset into nominal operation, and can momentarily cycle the `3 kV` enable to help quench a suspected arc.

## 7. Logic Arduino and `+3 kV` Monitor Arduino Interaction

This is the most important software interface in the system.

### High-level purpose

The Logic Arduino uses the `+3 kV` monitor Arduino as its indirect communication path to the Dashboard via Modbus.

That arrangement exists because:

- The Logic Arduino is optimized for fast deterministic digital sampling and output control
- Direct serial / Modbus traffic would work against that goal
- The `+3 kV` monitor Arduino can tolerate slower periodic polling and reporting

### Signals exported from the Logic Arduino

The Logic Arduino exports two kinds of information to the `+3 kV` monitor:

#### Live state outputs

- `D22-D24`: mirrors of the current output states for `CCS`, `Arm Beams`, and `3 kV Enable`
- `D25`: `Nom Op` flag

#### Latched event / fault flags

- `D26`: latched `3 kV Timer` event flag
- `D27-D29`: latched switch-related flags
- `D30-D37`: latched comparator fault flags

In the current implementation, the latched flags on `D26-D37` persist until the `+3 kV` monitor acknowledges that it has read them. `D25` remains live.

The current `+3 kV` monitor firmware uses the latched `D26` timer-event flag internally to maintain its `3 kV` timer/reset-event counter.

### Raw switch intent lines also seen by the `+3 kV` monitor

The `+3 kV` monitor directly reads:

- `Arm 80 kV`
- `3 kV Enable`
- `Arm Beams`
- `CCS Power`

In the revised wiring, the raw `3 kV Enable` request is read on monitor `D7` through a direct physical connection rather than through Logic Arduino `D26`.

This is important because it lets the Dashboard distinguish:

- what the user requested at the panel
- from what the Logic Arduino is actually allowing at the outputs

### ACK / ACK-back handshake

The two boards use a simple handshake so the monitor can both clear latched flags and confirm that the Logic Arduino is still alive.

#### `D14`: ACK from monitor to logic

The `+3 kV` monitor toggles the acknowledge line after it reads the Logic Arduino status.

Current implementation detail:

- The monitor alternates `D14` between `LOW` output and `INPUT` high-impedance
- The Logic Arduino treats any change on `D14` as an ACK edge

#### `D9`: ACK-back from logic to monitor

When the Logic Arduino sees the ACK edge:

- it clears the latched event history
- it toggles `D9`

The `+3 kV` monitor samples `D9` on its periodic cycle. If `D9` changed since the previous sample, it sets the `logic alive` status in the Modbus map.

### Why this matters

This interface gives the Dashboard three classes of logic-related information:

- Real-time logic output state
- Latched recent interlock history
- A proof-of-life indication that the Logic Arduino is still executing

## 8. Software Meaning of the Front Panel

The code and operating manual both make a key distinction:

- Some controls are direct local operator intent
- Some states are actual logic-authorized outputs

### Direct operator intent

- HV enable switches
- `Arm 80 kV`
- `Arm Beams`
- `CCS Power`
- Reset buttons
- Threshold trim pots
- Voltage set knobs

### Logic-authorized outputs

- Actual `CCS Power Enable`
- Actual `Arm Beams` / BCON permission
- Actual `3 kV` enable

This means the user can place `Arm Beams` or `CCS Power` switches in the `ON` position while the system still behaves as OFF because the Logic Arduino is holding the safe state.

That distinction is one of the main reasons the `+3 kV` monitor forwards both raw switch state and logic output state to the Dashboard.

## 9. Interaction With External Systems

### User

The user interacts with the firmware through:

- Front-panel switches
- Front-panel knobs and threshold trim pots
- The `Reset Interlocks` button
- LCDs and panel indicators
- The remote Dashboard

The user requests operating intent. The Logic Arduino decides whether the request is allowed to become a real output.

### Dashboard

The Dashboard is the remote visibility and logging path.

The firmware supports the Dashboard by:

- Reporting per-supply telemetry from all four monitoring Arduinos
- Reporting `logic arduino` logic/interlock state through the `+3 kV` monitor's Modbus register map
- Exposing global and per-channel status with slower, periodic updates

The Dashboard is not the first-line protection path. The firmware is designed so that interlock action occurs locally first and Dashboard awareness follows.

### Power supplies

The firmware interacts with the power supplies in two different ways:

- Monitoring Arduinos read analog status signals (`Vset`, `Vmon`, `Imon`)
- Logic firmware evaluates fast comparator outputs derived from those signals

### CCS

CCS power permission is software-controlled through the Logic Arduino.

The practical behavior is:

- User switch asks for CCS power
- Logic Arduino only asserts the real CCS enable output when the system is in normal operation
- Any interlock removes that enable

### Beam Controller / BCON

The Beam Controller receives the beam-permission signal from the Logic Arduino.

Through this, the Knob Box can rapidly force beam suppression on an interlock event.

### Glassman `+80 kV`

The `Arm 80 kV` switch is a required interlock input to the Logic Arduino.

The system must see this armed condition before it will enter normal operation.
