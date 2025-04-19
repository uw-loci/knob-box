# High‑Voltage Power‑Supply Monitor (aka Knob Box) Firmware for E-Beam System

Firmware for an **Arduino Mega 2560** that  

* Reads **set‑voltage**, **measured voltage**, and **measured current** from one of three power supplies (Bertan 205B‑3 kV, Bertan 205B‑20 kV, or Matsusada 1 kV / 30 mA).  
* Displays the values on a 16 × 2 I²C LCD.  
* Streams the same readings over USB serial.  
* Runs for days without crashing thanks to: non‑blocking timers, zero heap allocations, an AVR watchdog, and a periodic LCD refresh.

---

## Hardware Pin‑out

| Arduino Pin | Function | Connected To |
|-------------|----------|--------------|
| **48 (L3)** | `SET_BERTAN_3KV` jumper | Pull **LOW** if 3 kV PSU is attached |
| **50 (B3)** | `SET_BERTAN_20KV` jumper | Pull **LOW** if 20 kV PSU is attached |
| **52 (B1)** | `SET_MATSUSADA_1KV` jumper | Pull **LOW** if 1 kV / 30 mA PSU is attached |
| **A0**      | `V‑MON`  | 0–5 V voltage‑monitor output |
| **A1**      | `POT`    | 0–5 V set‑point potentiometer |
| **A2**      | `I‑MON`  | 0–5 V current‑monitor output |
| **SDA/SCL** | I²C bus  | ADS1115 + LCD backpack |
| 5 V / GND   | Power    | As usual |

> **Note:** Only *one* of the three “SET_…” pins should be low at power‑up.  
> Leave the others floating (internal pull‑ups are enabled).

---

## Software Dependencies

Install via the Arduino Library Manager or add to `platformio.ini`:

* **arduino‑timer** — <https://github.com/contrem/arduino-timer>  
* **Adafruit ADS1X15**  
* **LiquidCrystal_I2C**

Target board: **Arduino Mega 2560** (ATmega2560).

---

## How the Firmware Works

### Startup (`setup()`)

1. Initialise Serial, I²C, ADS1115, and LCD.  
2. Auto‑detect PSU via pins 48/50/52 and set multipliers:

| Model | Full‑scale V | Full‑scale I | `voltage_multiplier` | `current_multiplier` |
|-------|--------------|--------------|----------------------|----------------------|
| Bertan 205B‑03R | 3 000 V | 10 mA | 3000 | 10 |
| Bertan 205B‑20R | 20 000 V | 1 mA | 20000 | 1 |
| Matsusada 1 kV | 1 000 V | 30 mA | 1000 | 30 |

3. Register three non‑blocking timers (arduino‑timer):

| Slot | Callback | Period | Purpose |
|------|----------|--------|---------|
| 0 | `read_value()` | 150 ms | Read ADC & compute engineering units |
| 1 | `display_value()` | 200 ms | Update LCD and Serial |
| 2 | `clear_display()` | 30 min | `lcd.clear()` to wipe any ghost chars |

4. Enable the AVR watchdog (`8 s`).

### Main Loop (`loop()`)

```cpp
void loop() {
  wdt_reset();     // kick watchdog
  timer.tick();    // run any due callbacks
}
```

No delay() calls — MCU stays responsive.

Memory‑Safety Highlights
* Zero run‑time heap — no String objects after setup().

* Fixed‑size char[] buffers with snprintf() prevent overflow.

* ADC samples kept as int16_t, matching the Adafruit driver.

ADC & Scaling
* ADS1115 at ±6.144 V → 1 LSB = 0.1875 mV.

* Sample rate 860 SPS ensures a fresh reading every 150 ms.

* Conversions:
  - HV_set (V)  = potVolts / 5 × V_multiplier  
  - HV_meas (V) = vmonVolts / 5 × V_multiplier  
  - I_meas (mA) = imonVolts / 5 × I_multiplier  

Serial Log Format
Set:  750 V,  HV:  742 V,  I: 0.15 mA
One line every 200 ms, newline‑terminated.

### Building & Uploading
#### Arduino CLI
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 .
arduino-cli upload  -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 .
(Or use PlatformIO: platformio run -t upload.)

#### Extending the Firmware
* Add a PSU – reserve a new input pin, duplicate the jumper‑detect block, set new multipliers.
* Change refresh rates – adjust the two timer periods; keep display ≥ read.
* Alter LCD layout – edit display_value() only; rest of the logic is unaffected.

## Branching and Pull Request Strategy

Our repository uses a structured branching strategy with two primary branches:

### `main` Branch

- Protected by a ruleset requiring pull requests for all updates.
- Commits on `main` are tagged for important milestones or deployments, allowing easy reference later.
  - Example: `v1.0` corresponds to the version used in the 3kV High Voltage Test.

### `develop` Branch

- The default branch for ongoing development.
- No enforced ruleset, but all changes to `develop` should use pull requests for clarity and review.

### Workflow

1. Always branch from `develop`.
   - **Bug fixes:** `bugfix/your-fix-description`
   - **New features:** `feature/your-feature-description`

2. Create a draft pull request early to facilitate discussion and feedback during development.
3. Pull requests must be reviewed and approved by at least one other coder before merging.

### Pull Request Guidelines

- Keep pull requests focused and clean:
  - Avoid unrelated changes (e.g., unnecessary whitespace or formatting changes) unless explicitly intended.
  - Clearly document your changes to aid review.

Following this structure helps maintain clear version history, effective collaboration, and high-quality code.