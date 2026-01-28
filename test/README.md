# Hardware Setup for Power Supply Testing

Firmware for an **Arduino Mega 2650** that 

* Tests reading **set‑voltage**, **measured voltage**, and **measured current** from one of three power supplies (Bertan 205B‑3 kV, Bertan 205B‑20 kV, Matsusada 1 kV / 30 mA).
* Displays the values on a 16 x 2 I²C LCD. 

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
> **Note:** This pinout is not consistent with the finalized Knob Box firmware.

---