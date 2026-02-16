# Knob Box — Logic Arduino (Mega 2560 Rev 3)

This firmware (`logic_arduino.cpp`) implements the **Logic Arduino** portion of the Knob Box system. It runs a quick response state machine that:

- Reads **interlock comparators** (open-drain) and **front-panel switches**
- Controls three hardware enables:
  - **CCS Enable** (A0 / PF0)
  - **Beam Enable** (A1 / PF1)
  - **3kV Enable** (A2 / PF2)
- Publishes **status + event flags** on two 8‑bit ports for the 3kV Monitoring Arduino to read.
- Uses an **ACK toggle input** to clear latched event flags after 3kV reads flags.

---

## Safety model and assumptions

### Comparator inputs (D42–D49, PORTL) are open-drain (wired-safe)

Comparator lines are treated as open-drain/open-collector:

- **SAFE:** comparator drives **LOW**
- **FAULT / INTERLOCK:** comparator is **Hi‑Z**, Logic Arduino pullup makes input read **HIGH**
- **Disconnected wire:** reads **HIGH** → treated as **FAULT**

**In firmware:** comparator bits are read from `PINL` and **not inverted**.  
A `1` bit means **FAULT**.

---

## Tunable parameters

These constants are near the top of the file:

### `TIMER_3KV_MS` (default: `100`)
```cpp
static constexpr uint32_t TIMER_3KV_MS = 100;
```
3kV lockout duration (ms) after a 3kV trip.

### `DEBOUNCE_BITS`
```cpp
static constexpr uint8_t  DEBOUNCE_BITS = 16; // 1..31
static constexpr uint32_t MASK_DEBOUNCE = (uint32_t)((1u << DEBOUNCE_BITS) - 1u);
```
Debounce length in **number of consecutive samples** (sample occurs every ~25 us).

> **Important:** `DEBOUNCE_BITS` must remain in `1..31`. Values outside that range can produce undefined behavior due to shifting.

---

## Hardware pin mapping

### Inputs

| Signal | Arduino Pin | AVR Port/Bit | Pullup | Polarity in Code | Notes |
|---|---:|---|---|---|---|
| 3kV HV Output Enable Switch | D10 | PB4 | Yes | asserted = 1 | Active-low at pin, inverted in `switchesAssertPortB` |
| Arm Beams Switch | D11 | PB5 | Yes | asserted = 1 | Active-low at pin, inverted in `switchesAssertPortB` |
| CCS Power Allow Switch | D12 | PB6 | Yes | asserted = 1 | Active-low at pin, inverted in `switchesAssertPortB` |
| Arm 80kV Switch | D13 | PB7 | Yes | asserted = 1 | Active-low at pin, inverted in `switchesAssertPortB` |
| ACK (toggle-to-clear) | D14 | PJ1 | Yes | high = 1 | Any level change clears latched flags |
| Reset Button | D15 | PJ0 | Yes | pressed = 1 | Active-low at pin; debounced and rising-edge detected |
| +1kV V Comparator | D42 | PL7 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| +1kV I Comparator | D43 | PL6 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| -1kV V Comparator | D44 | PL5 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| -1kV I Comparator | D45 | PL4 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| 20kV V Comparator | D46 | PL3 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| 20kV I Comparator | D47 | PL2 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| 3kV V Comparator | D48 | PL1 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |
| 3kV I Comparator | D49 | PL0 | Yes | FAULT = 1 | Open-drain: LOW = safe, Hi-Z → HIGH = fault |

#### Switch and reset inversion
Switches and reset are physically **active-low** at the pin (pulled high, asserted by pulling to GND). Sampling converts them to **asserted = 1**:

```cpp
s.switchesAssertPortB = (uint8_t)(~PINB) & MASK_SWITCHES_PORTB;
s.resetAsserted       = (PINJ & MASK_RESET_BTN) == 0;
```

ACK is **not inverted**:

```cpp
s.ackLevel = (PINJ & MASK_ACK) != 0;
```

---

### Outputs (hardware enables + LED)

| Output | Arduino Pin | AVR Port/Bit | Active level | Meaning |
|---|---:|---|---|---|
| CCS Enable | A0 | PF0 | HIGH enables CCS Power | `out.ccsPowerEnable` |
| Beam Enable | A1 | PF1 | HIGH enables Beams via BCON | `out.armBeamsEnable` |
| 3kV Enable | A2 | PF2 | HIGH enables HV Output | `out.enable3kV` |
| Interlock LED | D16 | PH1 | HIGH drives LED | ON when NOT in NOM_OP |
---

## Flag outputs (status + latched events)

Two 8‑bit ports are dedicated to flag outputs:

### PORTA (D22–D29): live outputs + NomOp + latched switch events
`PORTA` is rebuilt and written every `step()`:

| Flag Name | Flag Pin | AVR Port/Bit | Source | Latched? | Meaning |
|---|---:|---|---|---|---|
| CCS Power Enable | D22 | PA0 | `out.ccsPowerEnable` | No | Mirrors current CCS enable output |
| Beam Enable | D23 | PA1 | `out.armBeamsEnable` | No | Mirrors current Beam enable output |
| 3kV HV Enable | D24 | PA2 | `out.enable3kV` | No | Mirrors current 3kV enable output |
| Nom Op | D25 | PA3 | `out.nomOp` | No | 1 = in `STATE_NOM_OP` |
| 3kV HV Output Enable Switch | D26 | PA4 | `prevFlagsSwitches` (PB4) | Yes | Switch asserted at least once since last ACK toggle |
| Arm Beams Switch | D27 | PA5 | `prevFlagsSwitches` (PB5) | Yes | Switch asserted at least once since last ACK toggle |
| CCS Power Allow Switch | D28 | PA6 | `prevFlagsSwitches` (PB6) | Yes | Switch asserted at least once since last ACK toggle |
| Arm 80kV Switch | D29 | PA7 | `prevFlagsSwitches` (PB7) | Yes | Switch asserted at least once since last ACK toggle |

> **Code reality (important):** `prevFlagsSwitches` is latched from **raw sampled switches**, not the debounced switch values:
>
> ```cpp
> prevFlagsSwitches |= (uint8_t)(sample.switchesAssertPortB & MASK_SWITCHES_PORTB);
> ```
>
> The file header comment says “latched debounced switches”, but the implementation latches **raw** (potentially bouncy) assertions. If you want flags to be debounced, latch `switchesDebounced` instead.

### PORTC (D30–D37): latched comparator fault events
`PORTC` reflects `prevFlagsComparators`, a sticky OR of `PINL` since last ACK toggle:

```cpp
prevFlagsComparators |= sample.comparators; // sample.comparators = PINL
PORTC = prevFlagsComparators;
```

**Bit-ordering note:** `PINL` bits are copied into `PORTC` bits directly. Physical Arduino pin numbering on PORTC is reversed relative to bit indices:

|> PORTC contains latched comparator fault events (sticky OR since last ACK toggle).

| Comparator Name | Comparator Input Pin | Comparator AVR Port/Bit | Flag Pin | Flag AVR Port/Bit | Latched? | Meaning |
|---|---:|---|---:|---|---|---|
| +1kV V Comparator | D42 | PL7 | D30 | PC7 | Yes | 1 if this comparator faulted since last ACK toggle |
| +1kV I Comparator | D43 | PL6 | D31 | PC6 | Yes | 1 if this comparator faulted since last ACK toggle |
| -1kV V Comparator | D44 | PL5 | D32 | PC5 | Yes | 1 if this comparator faulted since last ACK toggle |
| -1kV I Comparator | D45 | PL4 | D33 | PC4 | Yes | 1 if this comparator faulted since last ACK toggle |
| 20kV V Comparator | D46 | PL3 | D34 | PC3 | Yes | 1 if this comparator faulted since last ACK toggle |
| 20kV I Comparator | D47 | PL2 | D35 | PC2 | Yes | 1 if this comparator faulted since last ACK toggle |
| 3kV V Comparator | D48 | PL1 | D36 | PC1 | Yes | 1 if this comparator faulted since last ACK toggle |
| 3kV I Comparator | D49 | PL0 | D37 | PC0 | Yes | 1 if this comparator faulted since last ACK toggle |

---

## ACK toggle-to-clear protocol (D14 / PJ1)

`write_flags()` uses an ACK level change to clear both latched event flag bytes:

- `prevFlagsComparators` (latched comparator faults)
- `prevFlagsSwitches` (latched switch assertions)

Any edge (low→high or high→low) clears:

```cpp
if (sample.ackLevel != prevAck) {
  prevFlagsComparators = 0;
  prevFlagsSwitches    = 0;
}
prevAck = sample.ackLevel;
```

This supports a simple dashboard/test harness handshake: **read flags → toggle ACK → flags clear**.

> On first call after boot, `prevAck` starts as `false`. If the external pullup makes ACK high at boot, the first call will detect a “toggle” (false→true) and clear latches (already 0). This is harmless but worth knowing.

---

## Debounce implementation

Debounce uses a shift-register history per signal:

- 4 switch histories: `switchHist[4]`
- reset button history: `resetButtonHist`

On each sample, history shifts left and inserts the newest sample bit in the LSB. If the last `DEBOUNCE_BITS` samples are all 0 or all 1, the stable value is updated; otherwise it holds.

```cpp
hist = (hist << 1) | (sample ? 1u : 0u);
maskedHist = hist & MASK_DEBOUNCE;
if (maskedHist == 0) stable = false;
else if (maskedHist == MASK_DEBOUNCE) stable = true;
```

**Reset edge detection** is performed on the debounced reset signal:

```cpp
const bool resetButtonEdge = resetButtonDb && !prevResetButtonDb;
prevResetButtonDb = resetButtonDb;
```

---

## State machine

### State enum
```cpp
enum class State : uint8_t {
  STATE_INTERLOCK  = 0,
  STATE_NOM_OP     = 1,
  STATE_3KV_TIMER  = 2
};
```

### Comparator masks used in logic
- `MASK_COMP_3KV_I` = `PL0` (Arduino D49) — 3kV **current** fault
- `MASK_COMP_3KV`   = `PL0 | PL1` (D49 + D48) — 3kV **current OR voltage** fault

```cpp
static constexpr uint8_t MASK_COMP_3KV      = _BV(PL0) | _BV(PL1);
static constexpr uint8_t MASK_COMP_3KV_I    = _BV(PL0);
```

---

### Overview of `step()`

Each `loop()` iteration calls `step()`:

1. `sample_inputs()` reads raw pins into a `Sample`
2. Switches + reset are debounced
3. State transitions are evaluated based on:
   - comparator faults (immediate)
   - debounced switches
   - reset **edge**
4. Outputs are assigned from state + debounced switches
5. Flags are updated (including ACK-cleared latches)
6. Outputs are driven (register writes only if changed)

---

## State behavior (as implemented)

### `STATE_INTERLOCK` (BI / Interlock)
**Transitions:**
- If **3kV overcurrent** fault (`comparators & MASK_COMP_3KV_I`) → enter `STATE_3KV_TIMER`, set `timerEnterMs = millis()`, and stop evaluating other transitions this step.
- Else if **resetButtonEdge** and **all comparators safe** and required switches asserted:
  - `comparators == 0`
  - `sw_arm_80kv == true` (D13)
  - `sw_3kv_enable == true` (D10)  
  → enter `STATE_NOM_OP`

**Outputs:**
- CCS enable: OFF
- Beam enable: OFF
- 3kV enable: follows **debounced** 3kV switch (`sw_3kv_enable`)
- `nomOp`: 0

---

### `STATE_NOM_OP` (Nominal Operation)
**Transitions (evaluated in this order):**
1. If **3kV V/I fault** (`comparators & MASK_COMP_3KV`) → enter `STATE_3KV_TIMER` and set `timerEnterMs`.
2. Else if any of the following:
   - any comparator fault (`comparators != 0`)
   - Arm 80kV switch deasserted (`!sw_arm_80kv`)
   - 3kV enable switch deasserted (`!sw_3kv_enable`)  
   → return to `STATE_INTERLOCK`

**Outputs:**
- CCS enable: follows debounced CCS allow switch (`sw_ccs_allow`, D12)
- Beam enable: follows debounced arm beams switch (`sw_arm_beams`, D11)
- 3kV enable: follows debounced 3kV switch (`sw_3kv_enable`, D10)
- `nomOp`: 1

---

### `STATE_3KV_TIMER` (3kV lockout)
**Transition:**
- If elapsed time is at least `TIMER_3KV_MS`, return to `STATE_INTERLOCK`:
  ```cpp
  if ((uint32_t)(millis() - timerEnterMs) >= TIMER_3KV_MS)
      currentState = State::STATE_INTERLOCK;
  ```

**Outputs (forced safe):**
- CCS enable: OFF
- Beam enable: OFF
- 3kV enable: OFF
- `nomOp`: 0

---

## Mermaid diagram (logic-equivalent)

```mermaid
stateDiagram-v2
  [*] --> INTERLOCK

  INTERLOCK --> TIMER_3KV : (comparators & PL0) != 0
  INTERLOCK --> NOM_OP    : reset_edge && comparators==0 && sw80k && sw3k

  NOM_OP --> TIMER_3KV    : (comparators & (PL0|PL1)) != 0
  NOM_OP --> INTERLOCK    : comparators!=0 || !sw80k || !sw3k

  TIMER_3KV --> INTERLOCK : (millis()-enter_ms) >= TIMER_3KV_MS
```

---

## Initialization (`setup()`)

`setup()` performs:

- `io_init_registers()`:
  - configures DDR registers
  - enables pullups on inputs (switches, ack/reset, comparators)
  - forces safe posture on outputs (PF0–PF2 OFF; LED ON)
  - initializes PORTA/PORTC to 0
- initializes `prevPORTF` / `prevPORTH` to current output registers
- resets state to `STATE_INTERLOCK`
- clears debounce histories
- generates an initial flag image (`write_flags(rawSample, allOffOut)`)
- re-applies safe outputs (`write_outputs(allOffOut)`)

---

## Known discrepancies / logic notes (from code review)

1. **Switch event flags are NOT debounced.**  
   The header comment claims “latched debounced switches”, but `write_flags()` latches **raw sampled switch assertions**. If you see spurious latched switch flags, this is the likely cause.

2. **Potential 3kV “one-cycle pulse” when timer expires while fault persists.**  
   In `STATE_3KV_TIMER`, when the timer expires the firmware sets `currentState = INTERLOCK` and proceeds to output assignment **without re-checking** `MASK_COMP_3KV_I` that same step.  
   If the 3kV overcurrent fault (PL0) remains asserted at that moment and the 3kV enable switch is asserted, then for **one `step()` iteration** the `INTERLOCK` output assignment can re-enable 3kV before the next iteration re-enters the timer state.

   If this matters for your hardware, consider one of these fixes:
   - Keep the state in `STATE_3KV_TIMER` until the 3kV fault clears (in addition to time expiring).
   - Or, in `STATE_INTERLOCK` output assignment, force 3kV OFF when `MASK_COMP_3KV_I` is high.
   - Or, after changing `currentState` to `INTERLOCK` inside the timer state, immediately re-run/inline the interlock 3kI check before output assignment.

3. **Debounce time depends on loop rate.**  
   `DEBOUNCE_BITS` is a sample count; any change in loop timing changes debounce time.

---

## Extending the firmware safely

If you add new states or signals:

1. Update the `State` enum
2. Add transition logic in the state machine switch
3. Add explicit output assignments for the new state(s)
4. Decide whether the new signal should be:
   - **live status** (reflect current state), or
   - **latched event** (sticky until ACK toggles)

**Rule of thumb:** safety-relevant “events” should latch; operational “status” should reflect live state.

---

## File of record

- `logic_arduino.cpp` — main implementation
- Arduino entry points:
  - `setup()` initializes registers and safe posture
  - `loop()` calls `step()` continuously
