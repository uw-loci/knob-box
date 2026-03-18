# Knob Box Operating User Manual

## 1. Purpose

The Knob Box is a remote control and monitoring interface for the high-voltage hardware used during melt-metal and related beam experiments. It allows an operator outside the lead enclosure to:

- Set and monitor the output of the `+1 kV` Matsusada power supply
- Set and monitor the output of the `-1 kV` Matsusada power supply
- Set and monitor the output of the `+20 kV` Bertan power supply
- Set and monitor the output of the `+3 kV` Bertan power supply
- Arm the `+80 kV` Glassman interlock
- Control `Arm Beams`
- Control `CCS Power`
- View local status on front-panel LCDs and indicators
- Send system status to the Dashboard
- Automatically force the beams off and remove CCS power when an interlock or fault is detected

The Knob Box is not just a convenience panel. It is also part of the beam interlock chain.

## 2. What the Knob Box Does

The Knob Box combines two major functions:

- Slow control and monitoring
  - Each high-voltage supply has a monitoring Arduino that reads set voltage, measured voltage, measured current, threshold settings, and enable state.
  - These values are shown on the local LCD for that channel and reported to the Dashboard over RS-485.
- Fast fault response
  - A dedicated Logic Arduino watches the fast interlock signals.
  - If a fault is detected, it immediately forces the system to a beam-interlocked condition.
  - This means the Knob Box can shut off CCS power and command the Beam Controller to keep the grids negative without waiting for the Dashboard.

## 3. What the Knob Box Does Not Do

- It does not directly control the Glassman output voltage.
  - The Glassman voltage is controlled through its OEM software.
- It does not guarantee that AC mains power to every HV supply is off when the Knob Box is off.
  - The Knob Box power switch disables the controlled HV enable paths.
  - It does not replace the HV subpanel shutdown sequence.
- It is not a substitute for the experiment procedure, Glassman operating procedure, or lab safety requirements.

## 4. Controlled Equipment

The Knob Box interfaces with:

- `+1 kV Matsusada`
- `-1 kV Matsusada`
- `+20 kV Bertan`
- `+3 kV Bertan`
- `+80 kV Glassman` interlock/arm path
- `CCS` power shutoff relay
- `BCON` / Beam Controller interlock input
- Control workstation Dashboard

## 5. Front-Panel Controls and Indicators

The operator should expect the following controls and indicators.

### 5.1 Main Box Controls

- `Knob Box Power Switch`
  - Powers the electronics inside the Knob Box.
  - When this switch is off, the controlled HV enable lines default to off.
- `Power LED`
  - Indicates that the Knob Box itself is powered.
- `Interlocks Tripped LED`
  - Indicates that the Logic Arduino is not in normal operation.
  - When this LED is on, the system is beam-interlocked.
- `Reset Interlocks Button`
  - Used to move the Logic Arduino into normal operation, but only if all interlocks are currently clear.

  ### 5.2 System Switches

- `Arm 80 kV`
  - Arms the Glassman interlock path
  - Does not itself ramp or program the Glassman output
- `CCS Power`
  - Requests power to CCS through the CCS relay
- `Arm Beams`
  - Requests that BCON be allowed to make the grids positive

### 5.3 Per-Supply Controls

Each monitored power-supply section includes the following:

- `HV Enable Switch`
- `Voltage Control Knob`
- `Voltage Threshold Trim Pot`
- `Current Threshold Trim Pot`
- `20x4 LCD`

The LCD displays the channel's:

- Set voltage
- Measured voltage
- Measured current
- Trip thresholds
    - Interlock trips if measured current is **above** threshold current
    - Interlock trips if measured voltage is **below** threshold voltage

### 5.4 Matsusada-Specific Controls

Each Matsusada section also includes:

- `Overcurrent Reset Button`
  - Sends the Matsusada reset signal after a Matsusada internal overcurrent lockout
- `Reset-State LED`
  - Indicates that the supply appears to be in a Matsusada reset/lockout condition

Important:

- This reset-state indication is inferred from operating behavior.
- It is not a direct status pin from the Matsusada.

## 6. Key Operating Concept: Switch Intent vs Actual Output

Some front-panel switches represent operator intent, not a guaranteed live output.

In particular:

- `Arm Beams` can be switched on while the Logic Arduino still forces beams off
- `CCS Power` can be switched on while the Logic Arduino still keeps CCS power off
- `3 kV Enable` can be switched on while the Logic Arduino temporarily disables the actual 3 kV output during fault handling

If the Knob Box is not in normal operation, the Logic Arduino overrides those requests and forces the safe state.

Practical rule:

- If `Interlocks Tripped` is on, assume `beams are off` and `CCS is off` no matter what the related switch LEDs say.

## 7. Logic States

The Logic Arduino operates in three user-visible modes.

### 7.1 Beam Interlock State

This is the default startup state and the fallback fault state.

Behavior:

- CCS output forced off
- Beam enable forced off
- 3 kV output follows its special interlock logic
- Interlocks Tripped LED on

This state is expected:

- At startup
- During ramp-up before all supplies are at nominal values
- After a fault
- After intentionally taking a required interlock out of spec

### 7.2 Normal Operation State

This is the only state in which the `Arm Beams` and `CCS Power` requests are allowed to take effect.

Requirements to enter normal operation:

- All monitored interlocks must be clear
- `Arm 80 kV` must be on
- `3 kV Enable` must be on
- The operator must press `Reset Interlocks`

Once in normal operation:

- CCS enable follows the `CCS Power` switch
- Beam enable follows the `Arm Beams` switch
- 3 kV output follows the `3 kV Enable` switch unless a 3 kV trip occurs

### 7.3 3 kV Timer State

This state is unique to the `+3 kV Bertan`.

Behavior:

- CCS forced off
- Beam enable forced off
- 3 kV enable forced off
- A `100 ms` timeout is applied to help extinguish a possible arc

Current intended behavior:

- A `3 kV` overcurrent or undervoltage trip during normal operation sends the system into the 3 kV timer behavior.
- While the system is already interlocked, `3 kV undervoltage` alone is expected during startup or ramping and does not retrigger the timer.
- While interlocked, a continuing `3 kV overcurrent` can retrigger the timer behavior, resulting in repeated `100 ms` disable cycles until the overcurrent condition clears.

This is expected behavior, not a malfunction.

## 8. What Counts as an Interlock or Fault

The Knob Box continuously monitors:

- `+1 kV` undervoltage
- `+1 kV` overcurrent
- `-1 kV` undervoltage
- `-1 kV` overcurrent
- `+20 kV` undervoltage
- `+20 kV` overcurrent
- `+3 kV` undervoltage
- `+3 kV` overcurrent
- `Arm 80 kV` status
- `3 kV Enable` status

These conditions feed the Logic Arduino and determine whether the box is allowed to stay in normal operation.

## 9. What Happens on a Fault

If an interlock trips:

1. The Logic Arduino leaves normal operation.
2. `Arm Beams` output is forced off.
3. `CCS Power` output is forced off.
4. The `Interlocks Tripped` LED turns on.
5. The relevant fault indications are reported to the `3 kV` monitoring Arduino and then to the Dashboard.

If the fault is a `3 kV` fault:

- The `3 kV` output is also disabled for about `100 ms`.
- If overcurrent persists, additional timer cycles can occur.

To recover:

1. Identify and correct the cause.
2. Confirm the interlock condition has cleared.
3. Press `Reset Interlocks`.

If the underlying fault is still present, pressing reset will not restore normal operation.

## 10. Matsusada Reset-State Behavior

The Matsusadas have their own internal protection behavior.

If a Matsusada internally trips on overcurrent:

- The supply may disable its own HV output
- The Knob Box may see:
  - HV enable requested
  - Set voltage above near-zero
  - Measured voltage near zero
  - Measured current near zero

When that pattern appears:

- The Matsusada reset-state LED turns on
- The channel should be treated as likely needing a manual reset
- The global interlock will also likely trip because the voltage is too low

Recovery:

1. Verify the cause of the overcurrent has been addressed.
2. Press the correct Matsusada `Overcurrent Reset` button on the Knob Box.
3. Confirm the reset-state LED goes away.
4. Re-establish normal conditions.
5. Press `Reset Interlocks` if needed.

Important quirk:

- The Matsusada reset-state LED is a smart inference, not a direct hardware truth signal.

## 11. Dashboard Behavior

The Dashboard is the main remote status view for the operator.

It is expected to show:

- Per-supply measured voltage
- Per-supply measured current
- Per-supply set voltage
- HV enable state
- `Arm 80 kV` status
- `Arm Beams` status
- `CCS Power` status
- Global interlock state
- Individual interlock indications
- Event history / logging

Important notes:

- Threshold settings are local to the Knob Box and are shown on the LCDs.
- Interlock handling happens in hardware/firmware before the Dashboard updates.
- The Dashboard is for monitoring and logging, not first-line protection.

## 12. LCD Behavior

Each monitoring Arduino updates a local `20x4` LCD for its power supply.

Typical display items:

- Set voltage
- Measured voltage
- Measured current
- Threshold line

Formatting differs by channel:

- `+1 kV`, `-1 kV`, and `+3 kV` are shown in volts
- `+20 kV` uses kilovolt formatting for voltage

The LCD is the easiest place to set and verify local threshold trim pots during bring-up.

## 13. Normal Startup and Bring-Up Procedure

This section assumes the system is being prepared for normal operation in the melt-metal configuration.

### 13.1 Before Applying Power

Verify:

- All required cables are connected:
  - Both Matsusadas
  - Both Bertans
  - Glassman interlock connection
  - CCS shutoff cable
  - Beam Controller cable
  - Dashboard/workstation connection
- Knob Box grounding is in place
- All HV enable switches are off
- All front-panel voltage knobs are at zero
- `CCS Power` is off
- `Arm Beams` is off
- `80 kV` Glassman breaker is open/off

### 13.2 Energize the HV Subpanel

Turn on the HV subpanel through the normal facility procedure.

Expected behavior:

- Most HV supplies may now have AC mains available
- The `80 kV` Glassman remains unpowered if its breaker is still open
- HV outputs remain disabled because the Knob Box has not yet been used to enable them

### 13.3 Power On the Knob Box

Turn on the Knob Box.

Expected behavior:

- The box powers up in `Beam Interlock State`
- Dashboard begins reading values and switch states
- HV outputs remain off until each one is intentionally enabled

This startup interlocked condition is normal.

### 13.4 Ramp the Four Main Supplies Sequentially

Recommended order:

1. `-1 kV Matsusada`
2. `+1 kV Matsusada`
3. `+3 kV Bertan`
4. `+20 kV Bertan`

For each channel:

1. Confirm voltage set knob is at zero before enabling.
2. Adjust current and voltage threshold trim pot values displayed on the channel LCD.
3. Turn the channel HV enable switch on.
4. Slowly raise the voltage to the desired setpoint.
5. Confirm expected readings on the Dashboard and LCD.

Notes:

- Ramp slowly.
- The system may remain interlocked during ramp-up because undervoltage conditions are expected before nominal voltage is reached.
- That is normal and does not mean anything is wrong.

### 13.5 Arm the Glassman Before Applying Its AC Power

Once the other supplies are at nominal values:

1. Turn `Arm 80 kV` on at the Knob Box.
2. Confirm the armed state is visible locally and on the Dashboard.

Important:

- The Glassman should be armed before applying its `220 VAC`.
- Breaking the intended order can require a manual reset at the Glassman.

### 13.6 Optional Pulse Test Before Glassman AC Power

With:

- the other supplies at nominal values
- Glassman armed
- Glassman AC still off

you may perform the recommended pulse test.

During this step:

- `CCS Power` must remain off
- `Arm Beams` can be turned on for the pulse test
- `Interlocks Tripped` may still be on from the ramp-up history
- Press `Reset Interlocks` once all required conditions are actually satisfied

Expected result:

- Individual beam pulses can be tested while keeping CCS off

### 13.7 Bring Up the Glassman

After the pre-pulse checks:

1. Apply the Glassman AC power using the correct breaker.
2. Start the Glassman OEM software.
3. Confirm communications.
4. Set low voltage first.
5. Enable HV in the OEM software.
6. Slowly ramp toward the desired operating voltage.

The Knob Box does not replace the Glassman OEM control path for output programming.

## 14. Entering Full Operating Condition

When all required HV supplies are at nominal values and all interlocks are clear:

1. Confirm `Arm 80 kV` is on.
2. Confirm `3 kV Enable` is on.
3. Press `Reset Interlocks`.

If successful:

- `Interlocks Tripped` turns off
- The Knob Box is now in normal operation
- `Arm Beams` and `CCS Power` requests are allowed to take effect

To begin cathode heating:

1. Turn `CCS Power` on
2. Confirm CCS communication and expected status

To allow beam operation:

1. Turn `Arm Beams` on
2. Confirm the expected beam-control status

## 15. Expected Behavior During Normal Operation

During normal operation:

- The Knob Box continuously watches for undervoltage and overcurrent events
- If a monitored HV channel goes out of limits, the system immediately interlocks
  - BCON is forced to keep the grids negative
  - CCS power is removed

This behavior is intentional and protective.

The operator should expect:

- Fast protective action from the box itself
- Slightly slower status updates on the Dashboard
- Logged evidence of which interlocks were active

## 16. Expected Behavior During Ramping

During initial ramp-up or after intentional changes:

- The box may show an interlocked condition until all required channels are within threshold
- The `Interlocks Tripped` LED may remain on even though you are still in a normal setup sequence
- This is expected because low voltage during ramping looks the same as undervoltage from the logic point of view

Normal operator response:

1. Finish bringing the system to the intended condition.
2. Confirm all required channels are truly within limits.
3. Press `Reset Interlocks`.

## 17. Expected Behavior of the 3 kV Channel

The `+3 kV Bertan` is special.

Important behaviors:

- Its physical switch is not the final output authority.
- The Logic Arduino controls the actual `3 kV` enable path.
- A `3 kV` trip can temporarily turn the output off even if the physical switch remains on.

What the operator may observe:

- A `3 kV` fault causes immediate interlocking of beams and CCS
- The `3 kV` output is dropped for about `100 ms`
- If overcurrent persists, the Knob Box may continue repeating the timeout cycle

What this means in practice:

- Repeated 3 kV enable cycling usually points to a continuing overcurrent/arc condition
- Do not treat this as random behavior
- Investigate the cause before trying to force the system back into normal operation

## 18. Expected Behavior of Arm Beams and CCS Power

`Arm Beams` and `CCS Power` are both controlled by the Logic Arduino.

That means:

- Their switches can be on while their real outputs are off
- Their indicator lights can represent the switch request even when the logic is suppressing the output

Always interpret them together with the interlock state:

- `Interlocks Tripped` off:
  - switch request can take effect
- `Interlocks Tripped` on:
  - outputs are suppressed regardless of switch position

## 19. Shutdown Procedure

Recommended shutdown order:

1. Turn `Arm Beams` off.
2. Turn `CCS Power` off after cathodes have cooled enough.
3. Ramp down the Glassman using OEM software.
4. Disable Glassman output in OEM software.
5. De-arm `Arm 80 kV`.
6. Ramp down the remaining supplies in decreasing-voltage order.
7. Turn off each HV enable switch after its voltage reaches zero.
8. Optionally leave the Knob Box on for a short period to continue monitoring.
9. Turn off the Knob Box.
10. Turn off the HV subpanel / open the Glassman breaker as required.

Important:

- Turning off the Knob Box is not a substitute for bringing voltages to zero in a controlled way.

## 20. Common Operator Questions

### 20.1 Why is the Interlocks Tripped LED on during startup?

Because the box starts interlocked by design. It does not start in beam-enabled mode.

### 20.2 Why is the Interlocks Tripped LED on while I am ramping supplies up?

Because undervoltage during ramp-up is treated as out-of-spec until the channel reaches its threshold.

### 20.3 Why doesn't pressing Reset Interlocks do anything?

Because at least one required interlock is still active.

### 20.4 Why can the Arm Beams or CCS switch look on while the system still behaves off?

Because those switches are requests. The Logic Arduino can override them when the system is interlocked.

### 20.5 Why did the 3 kV output turn off even though I did not touch its switch?

Because the Logic Arduino detected a `3 kV` fault and intentionally cycled the output to extinguish a possible arc.

### 20.6 Why is a Matsusada reset LED on?

Because the monitoring firmware believes that Matsusada may be in its internal lockout/reset-required state.

## 21. Troubleshooting Guide

### 21.1 Interlocks Tripped Will Not Clear

Check:

- Are all four monitored supplies actually at their intended voltage?
- Is `Arm 80 kV` on?
- Is `3 kV Enable` on?
- Is a comparator threshold set too aggressively?
- Is a Matsusada in a reset-required state?
- Is the `3 kV` channel still indicating a continuing fault?

Then:

1. Correct the issue.
2. Verify it on LCD/Dashboard.
3. Press `Reset Interlocks`.

### 21.2 3 kV Seems to Be Cycling Repeatedly

Likely causes:

- Persistent `3 kV` overcurrent
- Continuing arc behavior
- Bad threshold setting
- Wiring or grounding issue

Recommended response:

1. Stop trying to operate normally.
2. Reduce or disable the relevant HV.
3. Inspect for arc causes or bad settings.
4. Only reset after the cause is understood.

### 21.3 Matsusada Has Enable On but No Output

Possible cause:

- Matsusada internal overcurrent trip / reset-required state

Recommended response:

1. Verify the voltage is commanded above zero.
2. Verify measured voltage and current remain near zero.
3. Press the corresponding Matsusada reset button.
4. Re-check output and reset LED.

### 21.4 Dashboard and Panel Seem Briefly Out of Sync

This can happen because:

- The fast logic interlock reacts first
- Monitoring and communications update afterward

If there is disagreement during a fast event:

- Trust the protective behavior first
- Then confirm the post-event state on the Dashboard

## 22. Feature Quirks and Non-Obvious Behaviors

- The Knob Box starts in an interlocked state every time it powers up.
- Ramping a supply from zero to nominal will usually create a temporary undervoltage condition that must later be cleared with `Reset Interlocks`.
- `Arm 80 kV` is required before entering normal operation.
- The `+3 kV` channel is intentionally special and may be cycled automatically by the logic.
- A Matsusada reset-state indication is inferred, not directly reported by the supply.
- The LCDs are the local place to read threshold settings.
- The Dashboard is slightly slower than the fast interlock path.
- If the Knob Box loses power, the controlled HV enable paths default to off.

## 23. Practical Good Habits

- Always begin with all HV enable switches off and all voltage knobs at zero.
- Ramp each supply slowly and deliberately.
- Use the LCDs while adjusting thresholds.
- Do not treat an interlock as a nuisance alarm; find the cause.
- Keep `CCS Power` off during the recommended pre-Glassman pulse test.
- Always arm the Glassman before applying its AC power.
- When in doubt, return the system to a known low-energy state before troubleshooting.

## 24. Summary

The Knob Box is the operator-facing control point for the HV supplies, beam arming logic, and CCS shutoff path. It is designed to let the operator work remotely while also providing automatic protection against undervoltage, overcurrent, and configuration errors.

To use it correctly:

- Think of the front-panel switches as requests
- Think of the Logic Arduino as the final authority for beam and CCS permission
- Expect interlocks during startup and ramping
- Use `Reset Interlocks` only after the system is truly in-range
- Treat repeated `3 kV` cycling or Matsusada reset indications as meaningful diagnostic information
