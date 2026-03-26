# Firmware for Monitoring Arduino in High‑Voltage Power‑Supply Monitor (aka Knob Box) 

Firmware for an **Arduino Mega 2560** that  

* Reads **set‑voltage**, **measured voltage**, and **measured current** from one of four power supplies (Bertan 205B‑3 kV, Bertan 205B‑20 kV, Matsusada 1 kV / 30 mA, or Matsusada -1 kV / 30 mA).  
* Displays the values on a 20 x 4 I²C LCD.  
* Streams the same readings, using RS-485, back to the E-Beam dashboard.  
* Runs for days without crashing thanks to: non‑blocking timers, zero heap allocations, an AVR watchdog, and a periodic LCD refresh.

The same `monitor_firmware.cpp` source is used for four monitor variants:

- `SELECTED_PS_ID = PS_POS1KV`: `+1 kV Matsusada`
- `SELECTED_PS_ID = PS_NEG1KV`: `-1 kV Matsusada`
- `SELECTED_PS_ID = PS_20KV`: `+20 kV Bertan`
- `SELECTED_PS_ID = PS_3KV`: `+3 kV Bertan`

The `+3 kV` monitor has extra firmware responsibilities. In addition to monitoring its own supply, it also reads the Logic Arduino interface signals and forwards that state to the Dashboard through its Modbus register map.

---

## Hardware Pinout

### Common Pins

| Arduino Pin | Purpose | Notes |
|-------------|---------|-------|
| `A0` | Current threshold pot input | Internal ADC |
| `A1` | Voltage threshold pot input | Internal ADC |
| `D6` | Matsusada reset LED | Used on `PS_POS1KV` and `PS_NEG1KV` only |
| `D7` | HV enable switch input | Common firmware input; for `+3 kV` this is the raw `3 kV Enable` switch request and is reported through the common HV-enable register |
| `D17` | RS-485 direction control | `low = receive`, transceiver controlled by firmware |
| `D18` | `TX1` | Modbus RTU transmit |
| `D19` | `RX1` | Modbus RTU receive |
| `D20` / `D21` | I2C `SDA` / `SCL` | ADS1115 + LCD |

### `+3 kV`-Only Interface Pins

| Arduino Pin | Purpose |
|-------------|---------|
| `D8` | Arm 80 kV switch input |
| `D9` | Logic Arduino ack-back input |
| `D11` | Arm Beams switch input |
| `D12` | CCS Power Allow switch input |
| `D14` | Logic Arduino flags acknowledge |
| `D22` | Logic Arduino `CCS Power` output state |
| `D23` | Logic Arduino `Arm Beams` output state |
| `D24` | Logic Arduino `3 kV Enable` output state |
| `D25` | Logic Arduino `Nom Op` flag |
| `D26` | Logic Arduino latched `3 kV Timer Event` flag |
| `D27-D29` | Logic Arduino latched switch-history flags |
| `D30-D37` | Logic Arduino latched comparator-history flags |

> Only the `+3 kV` monitor reads Logic Arduino status. The other three monitor Arduinos only report their local supply telemetry and enable state.

---

## Software Dependencies

The current source includes:

- `arduino-timer` — <https://github.com/contrem/arduino-timer>
- `Wire`
- `avr/wdt`
- `LiquidCrystal_I2C`
- `Adafruit_ADS1X15`
- `ModbusRtu.h` — <https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino>

Target board: `Arduino Mega 2560 Rev 3`

---

## Firmware Overview

### Power Supply Selection

Set `SELECTED_PS_ID` before compiling. Do not enter raw numbers:

```cpp
/**
 * POWER SUPPLY IDENTIFIER
 * Edit SELECTED_PS_ID only. Do not enter raw numbers here.
 *      - PS_POS1KV: +1kV Matsusada
 *      - PS_NEG1KV: -1kV Matsusada
 *      - PS_20KV: +20kV Bertan
 *      - PS_3KV: +3kV Bertan
 */
#define PS_POS1KV 1
#define PS_NEG1KV 2
#define PS_20KV 3
#define PS_3KV 4
#define SELECTED_PS_ID PS_POS1KV

#if SELECTED_PS_ID != PS_POS1KV && SELECTED_PS_ID != PS_NEG1KV && \
    SELECTED_PS_ID != PS_20KV && SELECTED_PS_ID != PS_3KV
#error "Invalid SELECTED_PS_ID. Use PS_POS1KV, PS_NEG1KV, PS_20KV, or PS_3KV."
#endif

const uint8_t ps_id = SELECTED_PS_ID;
```

The selected `SELECTED_PS_ID` controls:

- Voltage and current full-scale ratings
- LCD formatting
- Whether Matsusada reset logic is enabled
- Whether the `+3 kV` Logic Arduino interface is enabled
- The Modbus slave address used on RS-485

### Startup Sequence

`setup()` performs the following:

1. Starts the USB debug serial port at `9600`
2. Initializes I2C, the ADS1115, and the LCD
3. Configures common input pins
4. Selects supply-specific ratings from `SELECTED_PS_ID`
5. For the `+3 kV` firmware variant, enables all Logic Arduino interface inputs
6. Starts the Modbus RTU slave on `Serial1` at `9600`
7. Registers periodic timer callbacks
8. Re-enables the AVR watchdog with an `8 s` timeout near the end of `setup()`

The firmware uses the AVR watchdog in two stages:

- Early startup (`.init3`): capture `MCUSR`, clear it, and disable any watchdog inherited from a prior reset before normal Arduino startup runs.
- Runtime: enable the `8 s` watchdog near the end of `setup()` after peripheral initialization is complete, then refresh it once per `loop()`.

### Supply Ratings Used by the Code

| `SELECTED_PS_ID` | Supply | `ratedHV_V` | `ratedI_mA` |
|------------------|--------|-------------|-------------|
| `PS_POS1KV` | `+1 kV Matsusada` | `1000.0` | `30.0` |
| `PS_NEG1KV` | `-1 kV Matsusada` | `1000.0` | `30.0` |
| `PS_20KV` | `+20 kV Bertan` | `20000.0` | `1.0` |
| `PS_3KV` | `+3 kV Bertan` | `3000.0` | `10.0` |

### Main Loop

The main loop is intentionally small:

```cpp
void loop()
{
  wdt_reset(); // Feed dog

  int8_t pollResult = slave.poll(modbus_regs, TOTAL_REG_COUNT); // poll for requests from dashboard

  if (ps_id == PS_3KV && pollResult > 4) {
    clearPending = true;
  }

  timer.tick();
}
```

Key point: the current firmware does not have a separate `transmit_data()` task. Dashboard communication happens when the Modbus slave is polled.
For the `+3 kV` variant, a successful Modbus reply also clears the monitor-side sticky copy of the Logic Arduino fault flags after that response has been sent.

---

## Timing Model

The current source registers these timer callbacks:

| Callback | Period | Purpose |
|----------|--------|---------|
| `read_value()` | `150 ms` | Sample ADCs, update engineering values, update Modbus registers |
| `display_value()` | `200 ms` | Refresh LCD contents |
| `clear_display()` | `30 min` | Periodic LCD clear to avoid stale characters |

The older README's `transmit_data()` slot is no longer present in the current implementation.

---

## ADC Sampling and Scaling

The firmware reads three ADS1115 single-ended channels:

- `CH_VSET = 0`
- `CH_IMON = 1`
- `CH_VMON = 2`

The code configures:

- `GAIN_TWOTHIRDS`
- `RATE_ADS1115_860SPS`

It converts raw ADC counts to volts using:

- `VOLTS_PER_COUNT = 0.1875 mV/count`

Then scales to engineering units with the selected supply full-scale values:

- `programmedHV_V = (vsetVolts / 5.0) * ratedHV_V`
- `measuredHV_V = (vmonVolts / 5.0) * ratedHV_V`
- `measuredI_mA = (imonVolts / 5.0) * ratedI_mA`

Threshold potentiometers are read from the Mega's internal ADC:

- `A0` -> current threshold
- `A1` -> voltage threshold

Those are also scaled back to supply-level engineering units for LCD display and Dashboard telemetry.

The code clamps negative or over-range ADS1115 raw values before scaling and clamps outgoing Modbus values to `uint16_t`.

---

## Modbus Register Map

The current firmware exposes one contiguous register array:

- `IREG_COUNT = 4`
- `DINPUT_COUNT = 2`
- `TOTAL_REG_COUNT = 6`

### Common Input Registers

| Address | Name | Meaning |
|---------|------|---------|
| `0` | `IREG_V_SET_ADDR` | Programmed HV, rounded to integer volts |
| `1` | `IREG_V_READ_ADDR` | Measured HV, rounded to integer volts |
| `2` | `IREG_I_READ_ADDR` | Measured current, rounded to integer microamps |
| `3` | `IREG_3KV_RESET_COUNT_ADDR` | `+3 kV` timer/reset-event counter |

### Packed DINPUT Registers

| Address | Name | Meaning |
|---------|------|---------|
| `4` | `DINPUT_UNLATCHED_SIGNALS_ADDR` | Packed unlatched signals word |
| `5` | `DINPUT_LATCHED_FLAGS_ADDR` | Packed monitor-latched flags word |

`DINPUT_UNLATCHED_SIGNALS_ADDR` bit layout:

| Bit | Name | Source | Meaning |
|-----|------|--------|---------|
| `0` | `HV Enable` | `D7` | HV enable switch state |
| `1` | `Reset State 1kV` | internal state | Matsusada reset-state indication (`ps_id = PS_POS1KV` or `PS_NEG1KV`) |
| `2` | `Arm 80kV Enable` | `D8` | Arm 80 kV enable switch (`ps_id = PS_3KV`) |
| `3` | `CCS Power Enable` | `D22` | CCS Power Enable output signal (`ps_id = PS_3KV`) |
| `4` | `Arm Beams Enable` | `D23` | Arm Beams Enable output signal (`ps_id = PS_3KV`) |
| `5` | `3kV HV Enable` | `D24` | `3 kV` HV Enable output signal (`ps_id = PS_3KV`) |
| `6` | `Nom Op` | `D25` | Nom Op signal (`ps_id = PS_3KV`) |
| `7` | `Logic Alive` | derived from `D9` edge detect | Logic Arduino alive signal (`ps_id = PS_3KV`) |

`DINPUT_LATCHED_FLAGS_ADDR` bit layout:

| Bit | Pin | Meaning |
|-----|-----|---------|
| `4` | `D26` | 3 kV Timer State |
| `5` | `D27` | Arm Beams Switch |
| `6` | `D28` | CCS Power Allow Switch |
| `7` | `D29` | Arm 80 kV Switch |
| `8` | `D30` | `+1 kV` V Comparator |
| `9` | `D31` | `+1 kV` I Comparator |
| `10` | `D32` | `-1 kV` V Comparator |
| `11` | `D33` | `-1 kV` I Comparator |
| `12` | `D34` | `20 kV` V Comparator |
| `13` | `D35` | `20 kV` I Comparator |
| `14` | `D36` | `3 kV` V Comparator |
| `15` | `D37` | `3 kV` I Comparator |

Bits `0-3` are currently unused and remain `0`.

For `ps_id = PS_3KV`, the monitor samples the raw Logic Arduino latch pins on each `read_value()` cycle, ORs those bits into its own sticky `latchedFlags` word, and publishes that word in register `5`. After the dashboard request is answered successfully, the monitor clears that sticky word so the next request reports only newly sampled events.

Per-supply use of the packed DINPUT registers:

- `ps_id = PS_POS1KV` or `PS_NEG1KV`: unlatched bits `0-1` are used; latched word remains `0`
- `ps_id = PS_20KV`: unlatched bit `0` is used; latched word remains `0`
- `ps_id = PS_3KV`: unlatched bits `0`, `2-7` are used; latched bits `4-15` are used

---

## Supply-Specific Firmware Behavior

### Matsusada Variants (`ps_id = PS_POS1KV` or `PS_NEG1KV`)

The Matsusada variants drive unlatched-signals bit `1` through `checkMatsusadaResetState()`.

The firmware marks a potential reset state when:

- The HV enable switch is on
- The programmed voltage is above `1.0 V`
- Measured voltage is below `0.3 V`
- Measured current is below `0.3 mA`

It clears that state when the supply appears to recover:

- Measured voltage rises above `1.0 V`, or
- Measured current rises above `1.0 mA`

The state is reported both on the Matsusada reset LED (`D6`) and in unlatched-signals bit `1`.

### `+20 kV` Bertan (`ps_id = PS_20KV`)

The `+20 kV` variant uses `kV` formatting on the LCD:

- Set voltage shown with `0.01 kV` formatting
- Measured voltage shown with `0.01 kV` formatting
- Threshold voltage shown with `0.1 kV` formatting

For `DINPUT_UNLATCHED_SIGNALS_ADDR` bit `0`, the current code treats the `+20 kV` HV enable telemetry as active-high.

### `+3 kV` Bertan (`ps_id = PS_3KV`)

The `+3 kV` variant extends the normal monitor behavior with Logic Arduino telemetry:

- Uses `DINPUT_UNLATCHED_SIGNALS_ADDR` for the shared HV-enable input on `D7`, the raw Arm 80 kV switch on `D8`, the live Logic Arduino output signals on `D22-D25`, and the logic-alive heartbeat edge detect derived from `D9`
- Uses `DINPUT_LATCHED_FLAGS_ADDR` for a monitor-latched copy of the Logic Arduino flags on `D26-D37`, keeping `D26 -> bit 4` through `D37 -> bit 15`
- Keeps `D25` (`Nom Op`) in the unlatched word and `D26-D37` in the latched word
- Maps the existing ack-back edge-detect behavior on `D9` to unlatched-signals bit `7`
- Toggles the flags acknowledge line on `D14`
- Clears the monitor-latched flags only after a successful Modbus reply to the dashboard

The current code configures raw `Arm Beams` and `CCS Power Allow` switch inputs on `D11` and `D12`, but the published Modbus map currently exposes the Logic Arduino output-state lines on `D22` and `D23` for those functions.

The dedicated `3kV_HVEnable_Flag` Modbus register has been removed. The raw `3 kV Enable` switch request on `D7` is now reported only through unlatched-signals bit `0`.

It also tracks a `3 kV` timer/reset-event counter in Modbus register `3`.

Current implementation detail: the counter increments when:

- The latched `D26` timer-event flag rises

It resets to `0` when `Nom Op` rises.
The counter continues to use the raw sampled `D26` latch input rather than the monitor-latched Modbus register.

---

## LCD Behavior

`display_value()` updates the LCD with:

- Programmed HV
- Measured HV
- Measured current
- Threshold current and threshold voltage

Formatting differs slightly by supply:

- Matsusadas show signed volt values (`+` or `-`)
- `+20 kV` shows voltages in `kV`
- `+3 kV` shows voltages in `V`

The firmware also clears the LCD every `30 minutes` to avoid stale characters.

---

## Build Notes

Compile for:

- Board: `arduino:avr:mega:cpu=atmega2560`

This repository currently contains the firmware source file, but not a complete Arduino project or PlatformIO manifest, so the exact build command depends on how you package the sketch locally.

## Branching and Pull Request Strategy

The repository uses two primary branches:

- `main`: protected and used for approved milestones
- `develop`: default branch for ongoing work

Recommended workflow:

1. Branch from `develop`
2. Keep changes focused
3. Open a PR for review before merging
