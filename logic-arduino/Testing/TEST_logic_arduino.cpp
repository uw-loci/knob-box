/*
  Knob Box - Logic Arduino Test Harness (Mega 2560 Rev 3)

  PURPOSE
  - Drives every Logic Arduino input (switches, comparators, ACK, RESET) safely
    using open-drain style: drive LOW to assert, Hi-Z to release.
  - Monitors every Logic Arduino output (flags ports, Interlock LED, A0-A2 outputs).
  - Runs automated test suites (expanded, per-switch + per-comparator + flag latching).
  - Offers an interactive manual mode where user can control inputs and observe response.
  - Keeps a structured RAM log that can be dumped after tests.

  ELECTRICAL SAFETY NOTES
  - This tester NEVER drives HIGH on lines that the Logic Arduino pull-ups.
     * It only drives LOW (assert) or Hi-Z (release), preventing contention.
  - Ensure BOTH boards share GND.

  SERIAL UI
  - 115200 baud.
  - Type "help" for commands.

  INPUT/OUTPUT MAP (matches your Logic Arduino final design)
  Latched timer-event flag on PORTA:
    3kV Timer Event              N/A -> Flag D26 (PA4)

  Switches (Logic input, asserted=LOW) latched to PORTA upper bits:
    Arm Beams Switch             D11 -> Flag D27 (PA5)
    CCS Power Allow Switch       D12 -> Flag D28 (PA6)
    Arm 80kV Switch              D13 -> Flag D29 (PA7)

  Output mirror flags (CURRENT, not latched):
    CCS Power Enable (A0)        N/A -> Flag D22 (PA0)
    Beam Enable (A1)             N/A -> Flag D23 (PA1)
    3kV HV Enable (A2)           N/A -> Flag D24 (PA2)

  NomOp flag:
    Nom Op (state flag)          N/A -> Flag D25 (PA3)

  Comparator inputs (Logic comparator input, FAULT = Hi-Z -> pulled HIGH) latch to PORTC:
    +1kV V Comparator            D42 -> Flag D30 (PC7)
    +1kV I Comparator            D43 -> Flag D31 (PC6)
    -1kV V Comparator            D44 -> Flag D32 (PC5)
    -1kV I Comparator            D45 -> Flag D33 (PC4)
    20kV V Comparator            D46 -> Flag D34 (PC3)
    20kV I Comparator            D47 -> Flag D35 (PC2)
    3kV V Comparator             D48 -> Flag D36 (PC1)
    3kV I Comparator             D49 -> Flag D37 (PC0)

  Outputs :
    A0 – CCS Power Enable Signal
    A1 – Beam Enable Signal
    A2 – 3kV HV Enable Signal
    D16 - Interlock LED
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

struct Observe;
struct LogEntry;

// ========================= Configuration =========================
static constexpr uint32_t BAUD = 115200;
static constexpr uint16_t LOG_CAP = 256;

static constexpr uint32_t LOGIC_TIMER_3KV_MS = 100;
static constexpr uint16_t DEFAULT_SETTLE_MS = 5;
static constexpr uint16_t GLITCH_PULSE_US   = 50;

static constexpr uint8_t  COMP_IDX_3KV_V = 6; // D48
static constexpr uint8_t  COMP_IDX_3KV_I = 7; // D49

// DO NOT CHANGE per user requirement
static constexpr uint16_t RESET_PULSE_MS_NOMOP = 20;

// ========================= Pin Map =========================
struct PinMap {
  // Logic inputs we DRIVE
  uint8_t sw3kv;     // Logic D10
  uint8_t swBeams;   // Logic D11
  uint8_t swCCS;     // Logic D12
  uint8_t sw80kv;    // Logic D13
  uint8_t ack;       // Logic D14
  uint8_t reset;     // Logic D15

  // Comparator inputs Logic D42..D49
  // Index 0 = D42 ... Index 7 = D49
  uint8_t comp[8];

  // Logic outputs we READ
  uint8_t led;       // Logic D16
  uint8_t flagA[8];  // Logic D22..D29 (PA0..PA7)
  uint8_t flagC[8];  // Logic D30..D37 (PC7..PC0), array order is D30..D37
  uint8_t outA0;     // Logic A0
  uint8_t outA1;     // Logic A1
  uint8_t outA2;     // Logic A2
};

static const PinMap P = {
  10, 11, 12, 13, 14, 15,
  {42, 43, 44, 45, 46, 47, 48, 49},
  16,
  {22, 23, 24, 25, 26, 27, 28, 29},
  {30, 31, 32, 33, 34, 35, 36, 37},
  A0, A1, A2
};

// ========================= PROGMEM name strings (fixes F() global-init issue) =========================
static const char SW_NAME_3KV[]   PROGMEM = "3kV HV Output Enable Switch";
static const char SW_NAME_BEAMS[] PROGMEM = "Arm Beams Switch";
static const char SW_NAME_CCS[]   PROGMEM = "CCS Power Allow Switch";
static const char SW_NAME_80KV[]  PROGMEM = "Arm 80kV Switch";

static const char CP_NAME_P1KV_V[] PROGMEM = "+1kV V Comparator";
static const char CP_NAME_P1KV_I[] PROGMEM = "+1kV I Comparator";
static const char CP_NAME_N1KV_V[] PROGMEM = "-1kV V Comparator";
static const char CP_NAME_N1KV_I[] PROGMEM = "-1kV I Comparator";
static const char CP_NAME_20KV_V[] PROGMEM = "20kV V Comparator";
static const char CP_NAME_20KV_I[] PROGMEM = "20kV I Comparator";
static const char CP_NAME_3KV_V[]  PROGMEM = "3kV V Comparator";
static const char CP_NAME_3KV_I[]  PROGMEM = "3kV I Comparator";

static inline void printPgm(PGM_P p) {
  Serial.print(reinterpret_cast<const __FlashStringHelper*>(p));
}

// ========================= Signal Metadata (for friendly prints) =========================
struct SwitchMeta { PGM_P name; uint8_t inPin; uint8_t flagPin; uint8_t portaBit; };
struct CompMeta   { PGM_P name; uint8_t inPin; uint8_t flagPin; uint8_t compIdx; uint8_t portcBitPacked; };

static constexpr uint8_t PORTA_BIT_NOMOP = 3;
static constexpr uint8_t PORTA_BIT_3KVTIMER = 4;
static constexpr uint8_t MASK_SWITCH_LATCH_BITS = 0xE0;

static const SwitchMeta SWITCH_LATCHES[3] = {
  { SW_NAME_BEAMS, 11, 27, 5 },
  { SW_NAME_CCS,   12, 28, 6 },
  { SW_NAME_80KV,  13, 29, 7 }
};

static const CompMeta COMPS[8] = {
  { CP_NAME_P1KV_V, 42, 30, 0, 0 },
  { CP_NAME_P1KV_I, 43, 31, 1, 1 },
  { CP_NAME_N1KV_V, 44, 32, 2, 2 },
  { CP_NAME_N1KV_I, 45, 33, 3, 3 },
  { CP_NAME_20KV_V, 46, 34, 4, 4 },
  { CP_NAME_20KV_I, 47, 35, 5, 5 },
  { CP_NAME_3KV_V,  48, 36, 6, 6 },
  { CP_NAME_3KV_I,  49, 37, 7, 7 }
};

// ========================= Open-drain Drive Helpers =========================
static inline void driveLow(uint8_t pin) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
static inline void releaseHiZ(uint8_t pin) { pinMode(pin, INPUT); }

// DO NOT CHANGE: comparator faults should be Hi-Z (open drain semantics)
static inline void compSafe(uint8_t idx)  { driveLow(P.comp[idx]); }
static inline void compFault(uint8_t idx) { releaseHiZ(P.comp[idx]); }

// Switch semantics: asserted = LOW, deasserted = Hi-Z
static inline void swOn(uint8_t pin)  { driveLow(pin); }
static inline void swOff(uint8_t pin) { releaseHiZ(pin); }

static inline void pulseLowMs(uint8_t pin, uint16_t ms) { driveLow(pin); delay(ms); releaseHiZ(pin); }
static inline void pulseLowUs(uint8_t pin, uint16_t us) { driveLow(pin); delayMicroseconds(us); releaseHiZ(pin); }

// ACK edge generation (open-drain safe)
static inline void ackEdge() {
  driveLow(P.ack);
  delayMicroseconds(60);
  releaseHiZ(P.ack);
  delayMicroseconds(60);
}
static inline void toggleAckEdge() { ackEdge(); }

// Reset helpers (NomOp entry pulse is fixed at 20ms per requirement)
static inline void resetPulseNomOp() {
  pulseLowMs(P.reset, RESET_PULSE_MS_NOMOP);
  delay(10);
}

// ========================= Observations (read Logic outputs) =========================
struct Observe {
  uint8_t porta;  // D22..D29 packed bit0..7
  uint8_t portc;  // D30..D37 packed bit0..7
  bool led;       // D16
  bool outA0;     // A0
  bool outA1;     // A1
  bool outA2;     // A2
};

static inline uint8_t pack8(const uint8_t pins[8]) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; i++) if (digitalRead(pins[i]) == HIGH) v |= (uint8_t)(1u << i);
  return v;
}

static inline Observe observeLogic() {
  Observe o;
  o.porta = pack8(P.flagA);
  o.portc = pack8(P.flagC);
  o.led   = (digitalRead(P.led) == HIGH);
  o.outA0 = (digitalRead(P.outA0) == HIGH);
  o.outA1 = (digitalRead(P.outA1) == HIGH);
  o.outA2 = (digitalRead(P.outA2) == HIGH);
  return o;
}

static inline bool isNomOpFromFlags(const Observe& o) {
  return (o.porta & (1u << PORTA_BIT_NOMOP)) != 0; // D25 = bit3
}

static inline bool is3kVTimerFromFlags(const Observe& o) {
  return (o.porta & (1u << PORTA_BIT_3KVTIMER)) != 0; // D26 = bit4, latched timer-event flag
}

// Friendly OBS print (pins + register bits + names)
static void printObserveDetailed(const Observe& o) {
  Serial.println(F("\n--- OBSERVE LOGIC ARDUINO OUTPUTS ---"));
  Serial.print(F("Raw Flags: PORTA (D22..D29)=0x")); Serial.print(o.porta, HEX);
  Serial.print(F("  PORTC (D30..D37)=0x")); Serial.println(o.portc, HEX);

  Serial.println(F("\nPORTA / D22..D29 (Flags):"));
  Serial.print(F("  D22 (PA0 bit0): A0 mirror (CCS Enable) = ")); Serial.println((o.porta & (1u<<0)) ? F("1") : F("0"));
  Serial.print(F("  D23 (PA1 bit1): A1 mirror (Beam Enable) = ")); Serial.println((o.porta & (1u<<1)) ? F("1") : F("0"));
  Serial.print(F("  D24 (PA2 bit2): A2 mirror (3kV Enable) = ")); Serial.println((o.porta & (1u<<2)) ? F("1") : F("0"));
  Serial.print(F("  D25 (PA3 bit3): Nom Op flag = "));
  Serial.println((o.porta & (1u<<3)) ? F("1 (NOM_OP)") : F("0 (INTERLOCK/TIMER)"));
  Serial.print(F("  D26 (PA4 bit4): latched 3kV timer-event flag = "));
  Serial.println((o.porta & (1u<<4)) ? F("1 (timer event seen since ACK)") : F("0"));

  for (uint8_t i = 0; i < 3; i++) {
    const uint8_t bit = SWITCH_LATCHES[i].portaBit;
    Serial.print(F("  D")); Serial.print(SWITCH_LATCHES[i].flagPin);
    Serial.print(F(" (PA")); Serial.print(bit);
    Serial.print(F(" bit")); Serial.print(bit);
    Serial.print(F("): "));
    printPgm(SWITCH_LATCHES[i].name);
    Serial.print(F(" (latched) = "));
    Serial.println((o.porta & (1u<<bit)) ? F("1") : F("0"));
  }

  Serial.println(F("\nPORTC / D30..D37 (Comparator latches):"));
  for (uint8_t i = 0; i < 8; i++) {
    const uint8_t bit = i;
    const uint8_t dPin = (uint8_t)(30 + i);
    const CompMeta &cm = COMPS[i];
    Serial.print(F("  D")); Serial.print(dPin);
    Serial.print(F(" (packed bit")); Serial.print(bit);
    Serial.print(F("): "));
    printPgm(cm.name);
    Serial.print(F(" (Logic input D")); Serial.print(cm.inPin);
    Serial.print(F(", compIdx ")); Serial.print(cm.compIdx);
    Serial.print(F(") latched = "));
    Serial.println((o.portc & (1u<<bit)) ? F("1 (FAULT latched)") : F("0"));
  }

  Serial.println(F("\nLogic Outputs:"));
  Serial.print(F("  LED D16 (PH1): ")); Serial.println(o.led ? F("1 (ON)") : F("0 (OFF)"));
  Serial.print(F("  A0 (CCS Power Enable): ")); Serial.println(o.outA0 ? F("1") : F("0"));
  Serial.print(F("  A1 (Beam Enable):      ")); Serial.println(o.outA1 ? F("1") : F("0"));
  Serial.print(F("  A2 (3kV HV Enable):    ")); Serial.println(o.outA2 ? F("1") : F("0"));
  Serial.println(F("--- END OBS ---\n"));
}

// ========================= Logging (user-friendly + ties to test case) =========================
enum LogType : uint8_t { LOG_INFO, LOG_ACTION, LOG_OBS, LOG_CHECK, LOG_FAIL, LOG_PASS };

struct LogEntry {
  uint32_t t;
  uint16_t tc;
  LogType  type;
  uint32_t tag;
  uint8_t  a;
  uint8_t  b;
  uint16_t x;
};

static LogEntry logBuf[LOG_CAP];
static uint16_t logHead = 0;
static uint16_t logCount = 0;
static volatile uint16_t gTestCase = 0;

static constexpr uint32_t TAG4(char a, char b, char c, char d) {
  return ((uint32_t)(uint8_t)a << 24) |
         ((uint32_t)(uint8_t)b << 16) |
         ((uint32_t)(uint8_t)c << 8)  |
         (uint32_t)(uint8_t)d;
}

static inline void printTag(uint32_t tag) {
  char s[5];
  s[0] = (char)((tag >> 24) & 0xFF);
  s[1] = (char)((tag >> 16) & 0xFF);
  s[2] = (char)((tag >> 8) & 0xFF);
  s[3] = (char)(tag & 0xFF);
  s[4] = '\0';
  if (isPrintable((uint8_t)s[0]) && isPrintable((uint8_t)s[1]) && isPrintable((uint8_t)s[2]) && isPrintable((uint8_t)s[3])) {
    Serial.print(s);
  } else {
    Serial.print(F("0x")); Serial.print(tag, HEX);
  }
}

static inline void logPush(LogType type, uint32_t tag, uint8_t a=0, uint8_t b=0, uint16_t x=0) {
  logBuf[logHead] = { millis(), gTestCase, type, tag, a, b, x };
  logHead = (uint16_t)((logHead + 1) % LOG_CAP);
  if (logCount < LOG_CAP) logCount++;
}

static void dumpDecodedLine(const LogEntry &e) {
  if (e.tag == TAG4('O','B','S','0')) {
    Serial.print(F("  Decoded OBS: PORTA=0x")); Serial.print(e.a, HEX);
    Serial.print(F(" PORTC=0x")); Serial.print(e.b, HEX);
    Serial.print(F(" LED=")); Serial.print((e.x & (1u<<3)) ? 1 : 0);
    Serial.print(F(" A0=")); Serial.print((e.x & (1u<<0)) ? 1 : 0);
    Serial.print(F(" A1=")); Serial.print((e.x & (1u<<1)) ? 1 : 0);
    Serial.print(F(" A2=")); Serial.println((e.x & (1u<<2)) ? 1 : 0);
  }
}

static void logDump() {
  Serial.println(F("\n---- LOG DUMP (oldest -> newest) ----"));
  Serial.println(F("Format: <time>ms | TC=<###> | <TYPE> | tag=<TAG> | a=<..> b=<..> x=<..>"));

  uint16_t start = (logCount == LOG_CAP) ? logHead : 0;
  for (uint16_t i = 0; i < logCount; i++) {
    uint16_t idx = (uint16_t)((start + i) % LOG_CAP);
    const LogEntry &e = logBuf[idx];

    Serial.print(e.t); Serial.print(F(" ms | TC="));
    if (e.tc < 100) Serial.print(F("0"));
    if (e.tc < 10)  Serial.print(F("0"));
    Serial.print(e.tc);
    Serial.print(F(" | "));

    switch (e.type) {
      case LOG_INFO:   Serial.print(F("INFO  ")); break;
      case LOG_ACTION: Serial.print(F("ACT   ")); break;
      case LOG_OBS:    Serial.print(F("OBS   ")); break;
      case LOG_CHECK:  Serial.print(F("CHECK ")); break;
      case LOG_FAIL:   Serial.print(F("FAIL  ")); break;
      case LOG_PASS:   Serial.print(F("PASS  ")); break;
      default:         Serial.print(F("UNK   ")); break;
    }

    Serial.print(F("| tag=")); printTag(e.tag);
    Serial.print(F(" | a=")); Serial.print(e.a);
    Serial.print(F(" b=")); Serial.print(e.b);
    Serial.print(F(" x=0x")); Serial.println(e.x, HEX);

    dumpDecodedLine(e);
  }
  Serial.println(F("---- END LOG ----\n"));
}

// ========================= Harness state =========================
static bool autoRunning = false;

// ========================= Init / Safe baseline =========================
static void initPins() {
  releaseHiZ(P.sw3kv);
  releaseHiZ(P.swBeams);
  releaseHiZ(P.swCCS);
  releaseHiZ(P.sw80kv);
  releaseHiZ(P.ack);
  releaseHiZ(P.reset);
  for (uint8_t i = 0; i < 8; i++) releaseHiZ(P.comp[i]);

  pinMode(P.led, INPUT);
  for (uint8_t i = 0; i < 8; i++) pinMode(P.flagA[i], INPUT);
  for (uint8_t i = 0; i < 8; i++) pinMode(P.flagC[i], INPUT);
  pinMode(P.outA0, INPUT);
  pinMode(P.outA1, INPUT);
  pinMode(P.outA2, INPUT);
}

static void setAllSafeIdle() {
  swOff(P.sw3kv);
  swOff(P.swBeams);
  swOff(P.swCCS);
  swOff(P.sw80kv);

  for (uint8_t i = 0; i < 8; i++) compSafe(i);

  releaseHiZ(P.ack);
  releaseHiZ(P.reset);

  logPush(LOG_ACTION, TAG4('I','D','L','E'));
}

static void suiteStart() {
  setAllSafeIdle();
  ackEdge(); delay(2);
  ackEdge(); delay(2);
}

static inline void settle(uint16_t ms = DEFAULT_SETTLE_MS) { delay(ms); }

// ========================= Test Case helpers (numbered) =========================
static void beginSuite(uint16_t /*suiteBase*/, const __FlashStringHelper* name) {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("SUITE: ")); Serial.println(name);
  Serial.println(F("========================================"));
  logPush(LOG_INFO, TAG4('S','U','I','T'));
}

static void beginCase(uint16_t tc, const __FlashStringHelper* name) {
  gTestCase = tc;
  Serial.print(F("\nT"));
  if (tc < 100) Serial.print(F("0"));
  if (tc < 10)  Serial.print(F("0"));
  Serial.print(tc);
  Serial.print(F(": "));
  Serial.println(name);
  logPush(LOG_INFO, TAG4('T','C','A','S'));
}

// ========================= Expectations / Check helpers =========================

static bool expectOutputMirrorFlags(bool a0, bool a1, bool a2, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();

  const uint8_t want = (uint8_t)((a0<<0) | (a1<<1) | (a2<<2));
  const uint8_t got  = (uint8_t)(o.porta & 0x07);

  logPush(LOG_CHECK, TAG4('F','O','U','0'), want, got, o.porta);

  if (got != want) {
    logPush(LOG_FAIL, TAG4('F','O','U','F'), want, got, o.porta);
    Serial.print(F("FAIL: Output-mirror flags mismatch (D22..D24). Want 0b"));
    Serial.print(want, BIN);
    Serial.print(F(" got 0b"));
    Serial.println(got, BIN);
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('F','O','U','P'), want, got, o.porta);
  return true;
}

static bool expectNomOp(bool wantNomOp, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const bool gotNomOp = isNomOpFromFlags(o);

  logPush(LOG_CHECK, TAG4('N','O','M','0'), (uint8_t)wantNomOp, (uint8_t)gotNomOp, o.porta);

  if (gotNomOp != wantNomOp) {
    logPush(LOG_FAIL, TAG4('N','O','M','F'), (uint8_t)wantNomOp, (uint8_t)gotNomOp, o.porta);
    Serial.println(F("FAIL: NomOp mismatch"));
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('N','O','M','P'), (uint8_t)wantNomOp, (uint8_t)gotNomOp, o.porta);
  return true;
}

static bool expectTimerFlag(bool wantTimer, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const bool gotTimer = is3kVTimerFromFlags(o);

  logPush(LOG_CHECK, TAG4('T','M','R','0'), (uint8_t)wantTimer, (uint8_t)gotTimer, o.porta);

  if (gotTimer != wantTimer) {
    logPush(LOG_FAIL, TAG4('T','M','R','F'), (uint8_t)wantTimer, (uint8_t)gotTimer, o.porta);
    Serial.println(F("FAIL: latched 3kV timer-event flag mismatch"));
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('T','M','R','P'), (uint8_t)wantTimer, (uint8_t)gotTimer, o.porta);
  return true;
}

static bool expectOutputs(bool a0, bool a1, bool a2, bool led, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();

  const bool okOut = (o.outA0==a0) && (o.outA1==a1) && (o.outA2==a2) && (o.led==led);
  const uint8_t wantOut = (uint8_t)((a0<<0)|(a1<<1)|(a2<<2)|(led<<3));
  const uint8_t gotOut  = (uint8_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));

  // Output-mirror flags are CURRENT state (NOT latched)
  const uint8_t wantFlags = (uint8_t)((a0<<0)|(a1<<1)|(a2<<2));
  const uint8_t gotFlags  = (uint8_t)(o.porta & 0x07);
  const bool okFlags = (wantFlags == gotFlags);

  logPush(LOG_CHECK, TAG4('O','U','T','0'), wantOut, gotOut, (uint16_t)((o.porta<<8) | o.portc));
  logPush(LOG_CHECK, TAG4('F','O','U','0'), wantFlags, gotFlags, o.porta);

  if (!okOut || !okFlags) {
    if (!okOut)   logPush(LOG_FAIL, TAG4('O','U','T','F'), wantOut, gotOut, (uint16_t)((o.porta<<8) | o.portc));
    if (!okFlags) logPush(LOG_FAIL, TAG4('F','O','U','F'), wantFlags, gotFlags, o.porta);

    Serial.println(F("FAIL: Output and/or output-mirror flag mismatch"));
    if (!okOut) {
      Serial.print(F("  Outputs want 0b")); Serial.print(wantOut, BIN);
      Serial.print(F(" got 0b")); Serial.println(gotOut, BIN);
    }
    if (!okFlags) {
      Serial.print(F("  Flags D22..D24 want 0b")); Serial.print(wantFlags, BIN);
      Serial.print(F(" got 0b")); Serial.println(gotFlags, BIN);
    }
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('O','U','T','P'), wantOut, gotOut, (uint16_t)((o.porta<<8) | o.portc));
  logPush(LOG_PASS, TAG4('F','O','U','P'), wantFlags, gotFlags, o.porta);
  return true;
}

static uint8_t expectedSwitchLatchBits(bool beams, bool ccs, bool sw80) {
  uint8_t v = 0;
  if (beams) v |= (1u << 5);
  if (ccs)   v |= (1u << 6);
  if (sw80)  v |= (1u << 7);
  return v;
}

static bool checkSwitchLatchExact(uint8_t wantLatchBits, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const uint8_t got = (uint8_t)(o.porta & MASK_SWITCH_LATCH_BITS);

  logPush(LOG_CHECK, TAG4('S','L','A','0'), wantLatchBits, got, o.porta);

  if (got != wantLatchBits) {
    logPush(LOG_FAIL, TAG4('S','L','A','F'), wantLatchBits, got, o.porta);
    Serial.print(F("FAIL: Switch latch mismatch. Want 0x"));
    Serial.print(wantLatchBits, HEX);
    Serial.print(F(" got 0x"));
    Serial.println(got, HEX);
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('S','L','A','P'), wantLatchBits, got, o.porta);
  return true;
}

static inline uint8_t bitForCompIdx(uint8_t compIdx) { return (uint8_t)(1u << compIdx); }

static bool checkComparatorLatchExact(uint8_t want, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const uint8_t got = o.portc;

  logPush(LOG_CHECK, TAG4('C','L','A','0'), want, got, o.portc);

  if (got != want) {
    logPush(LOG_FAIL, TAG4('C','L','A','F'), want, got, o.portc);
    Serial.print(F("FAIL: Comparator latch mismatch. Want 0x"));
    Serial.print(want, HEX);
    Serial.print(F(" got 0x"));
    Serial.println(got, HEX);
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('C','L','A','P'), want, got, o.portc);
  return true;
}

static bool checkComparatorLatchHas(uint8_t wantMask, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const uint8_t got = o.portc;

  logPush(LOG_CHECK, TAG4('C','L','H','0'), wantMask, got, o.portc);

  if ((got & wantMask) != wantMask) {
    logPush(LOG_FAIL, TAG4('C','L','H','F'), wantMask, got, o.portc);
    Serial.print(F("FAIL: Comparator latch missing bits. Want mask 0x"));
    Serial.print(wantMask, HEX);
    Serial.print(F(" got 0x"));
    Serial.println(got, HEX);
    printObserveDetailed(o);
    return false;
  }

  logPush(LOG_PASS, TAG4('C','L','H','P'), wantMask, got, o.portc);
  return true;
}

// ========================= Automated Test Suites (numbered cases) =========================

static void testSuiteEnterExitNomOp() {
  beginSuite(100, F("ENTER/EXIT NOM_OP"));
  suiteStart();

  beginCase(101, F("Baseline: all switches OFF -> INTERLOCK, outputs OFF, LED ON"));
  swOff(P.sw3kv); swOff(P.sw80kv); swOff(P.swBeams); swOff(P.swCCS);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,false,true)) return;

  beginCase(102, F("Interlock: enable 3kV switch only -> A2 follows (ON)"));
  swOn(P.sw3kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  beginCase(103, F("Enter NOM_OP: sw80kv ON + sw3kv ON + RESET edge (20ms) + all comparators SAFE"));
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;

  beginCase(104, F("Exit NOM_OP: drop sw80kv -> return to INTERLOCK"));
  swOff(P.sw80kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  beginCase(105, F("Re-enter NOM_OP and exit by dropping sw3kv"));
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  swOff(P.sw3kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,false,true)) return;
}

static void testSuiteEachSwitchBehaviorInNomOp() {
  beginSuite(200, F("EACH SWITCH BEHAVIOR IN NOM_OP"));
  suiteStart();

  beginCase(201, F("Enter NOM_OP baseline"));
  swOn(P.sw3kv);
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;

  beginCase(202, F("Toggle CCS allow in NOM_OP: stay NOM_OP, only A0 changes"));
  swOn(P.swCCS);
  if (!expectNomOp(true)) return;
  if (!expectOutputs(true,false,true,false)) return;
  swOff(P.swCCS);
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;

  beginCase(203, F("Toggle Arm Beams in NOM_OP: stay NOM_OP, only A1 changes"));
  swOn(P.swBeams);
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,true,true,false)) return;
  swOff(P.swBeams);
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;

  beginCase(204, F("Drop sw80kv in NOM_OP: MUST go to INTERLOCK"));
  swOff(P.sw80kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  beginCase(205, F("Re-enter NOM_OP then drop sw3kv: MUST go to INTERLOCK"));
  swOn(P.sw80kv);
  swOn(P.sw3kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  swOff(P.sw3kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,false,true)) return;
}

static void testSuiteSwitchLatchStackingAndAckClear() {
  beginSuite(300, F("SWITCH LATCH STACKING + ACK CLEAR"));
  suiteStart();

  beginCase(301, F("ACK clear -> switch latch bits D27..D29 should be 0x00"));
  ackEdge();
  if (!checkSwitchLatchExact(0x00)) return;

  beginCase(302, F("sw3kv is not latched on PORTA; D27..D29 accumulate as remaining switches assert"));
  swOn(P.sw3kv);   if (!checkSwitchLatchExact(0x00)) return;
  swOn(P.swBeams); if (!checkSwitchLatchExact(expectedSwitchLatchBits(true,false,false))) return;
  swOn(P.swCCS);   if (!checkSwitchLatchExact(expectedSwitchLatchBits(true,true,false))) return;
  swOn(P.sw80kv);  if (!checkSwitchLatchExact(expectedSwitchLatchBits(true,true,true))) return;

  beginCase(303, F("Release all -> latch remains set until ACK"));
  swOff(P.sw3kv); swOff(P.swBeams); swOff(P.swCCS); swOff(P.sw80kv);
  if (!checkSwitchLatchExact(expectedSwitchLatchBits(true,true,true))) return;

  beginCase(304, F("ACK edge clears switch latch"));
  ackEdge();
  if (!checkSwitchLatchExact(0x00)) return;

  beginCase(305, F("Post-clear re-latch single switch (sw80kv)"));
  swOn(P.sw80kv);
  if (!checkSwitchLatchExact(expectedSwitchLatchBits(false,false,true))) return;
}

static void testSuiteComparatorLatchStackingAndAckClear() {
  beginSuite(400, F("COMPARATOR LATCH STACKING + ACK CLEAR (EXACT)"));
  suiteStart();

  beginCase(401, F("ACK clear -> comparator latch should be 0x00"));
  ackEdge();
  if (!checkComparatorLatchExact(0x00)) return;

  beginCase(402, F("Trip two comparators -> latch ORs them; clearing physical fault does not clear latch"));
  const uint8_t m0 = bitForCompIdx(0);
  const uint8_t m3 = bitForCompIdx(3);
  compFault(0);
  compFault(3);
  if (!checkComparatorLatchHas((uint8_t)(m0 | m3))) return;
  compSafe(0);
  compSafe(3);
  if (!checkComparatorLatchHas((uint8_t)(m0 | m3))) return;

  beginCase(403, F("Add another comparator fault later -> latch stacks"));
  const uint8_t m7 = bitForCompIdx(7);
  compFault(7);
  if (!checkComparatorLatchHas((uint8_t)(m0 | m3 | m7))) return;

  beginCase(404, F("ACK clears comparator latch"));
  compSafe(7);
  ackEdge();
  if (!checkComparatorLatchExact(0x00)) return;

  
}

static void testSuitePerComparatorIndividual() {
  beginSuite(500, F("PER-COMPARATOR INDIVIDUAL (EXACT)"));
  suiteStart();

  for (uint8_t i = 0; i < 8; i++) {
    beginCase((uint16_t)(501 + i), F("Trip one comparator -> latch bit sets; clear physical -> latch stays; ACK clears"));

    for (uint8_t k = 0; k < 8; k++) compSafe(k);

    ackEdge();
    if (!checkComparatorLatchExact(0x00)) return;

    compFault(i);
    const uint8_t want = bitForCompIdx(i);
    if (!checkComparatorLatchExact(want)) return;

    compSafe(i);
    if (!checkComparatorLatchExact(want)) return;

    ackEdge();
    if (!checkComparatorLatchExact(0x00)) return;
  }
}

static void testSuite3kVSpecific() {
  beginSuite(600, F("3kV I/V SPECIFIC"));
  suiteStart();

  beginCase(601, F("Latch bits for 3kV I and 3kV V individually"));
  ackEdge();
  if (!checkComparatorLatchExact(0x00)) return;

  compFault(COMP_IDX_3KV_I);
  if (!checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_I))) return;
  compSafe(COMP_IDX_3KV_I);
  if (!checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_I))) return;
  ackEdge();
  if (!checkComparatorLatchExact(0x00)) return;

  delay(LOGIC_TIMER_3KV_MS + 10); // wait until timer state ends
  ackEdge();
  if (!expectTimerFlag(false)) return;

  compFault(COMP_IDX_3KV_V);
  if (!checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_V))) return;
  compSafe(COMP_IDX_3KV_V);
  if (!checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_V))) return;
  ackEdge();
  if (!checkComparatorLatchExact(0x00)) return;



  beginCase(602, F("INTERLOCK: timer entry only on 3kV I (3kV V alone does not force timer)"));
  swOn(P.sw3kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  compFault(COMP_IDX_3KV_V);
  delay(2);
  if (!expectNomOp(false)) return;
  if (!expectTimerFlag(false)) return;
  if (!expectOutputs(false,false,true,true)) return;
  compSafe(COMP_IDX_3KV_V);

  compFault(COMP_IDX_3KV_I);
  delay(2);
  compSafe(COMP_IDX_3KV_I);
  if (!expectNomOp(false)) return;
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,false,true)) return;
  delay(LOGIC_TIMER_3KV_MS + 10);
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,true,true)) return;
  ackEdge();
  if (!expectTimerFlag(false)) return;

  beginCase(603, F("NOM_OP: timer entry on 3kV V OR 3kV I"));
  swOn(P.sw80kv);
  swOn(P.sw3kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;

  compFault(COMP_IDX_3KV_V);
  delay(2);
  compSafe(COMP_IDX_3KV_V);
  if (!expectNomOp(false)) return;
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,false,true)) return;
  delay(LOGIC_TIMER_3KV_MS + 10);
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,true,true)) return;
  ackEdge();
  if (!expectTimerFlag(false)) return;

  swOn(P.sw80kv);
  swOn(P.sw3kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;

  compFault(COMP_IDX_3KV_I);
  delay(2);
  compSafe(COMP_IDX_3KV_I);
  if (!expectNomOp(false)) return;
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,false,true)) return;
  delay(LOGIC_TIMER_3KV_MS + 10);
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,true,true)) return;
  ackEdge();
  if (!expectTimerFlag(false)) return;

  suiteStart();

  beginCase(604, F("ACK during active 3kV timer clears D26 until the next timer-state entry"));
  swOn(P.sw3kv);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;
  if (!expectTimerFlag(false)) return;

  compFault(COMP_IDX_3KV_I);
  delay(2);
  compSafe(COMP_IDX_3KV_I);
  if (!expectNomOp(false)) return;
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,false,true)) return;

  ackEdge();
  if (!expectTimerFlag(false)) return;
  if (!expectOutputs(false,false,false,true)) return;

  delay(LOGIC_TIMER_3KV_MS + 10);
  if (!expectTimerFlag(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  compFault(COMP_IDX_3KV_I);
  delay(2);
  compSafe(COMP_IDX_3KV_I);
  if (!expectTimerFlag(true)) return;
  if (!expectOutputs(false,false,false,true)) return;
  ackEdge();
  if (!expectTimerFlag(false)) return;
}

static void testSuiteNomOpEntryBlockedCases() {
  beginSuite(700, F("NOM_OP ENTRY BLOCKED CASES"));
  suiteStart();

  beginCase(701, F("Missing sw80kv -> reset edge should NOT enter NOM_OP"));
  swOn(P.sw3kv);
  swOff(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(false)) return;

  beginCase(702, F("Missing sw3kv -> reset edge should NOT enter NOM_OP"));
  swOff(P.sw3kv);
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(false)) return;

  beginCase(703, F("Comparator fault present -> reset edge should NOT enter NOM_OP"));
  swOn(P.sw3kv);
  swOn(P.sw80kv);
  compFault(0);
  resetPulseNomOp();
  if (!expectNomOp(false)) return;
  compSafe(0);

  beginCase(704, F("All conditions satisfied -> DOES enter NOM_OP"));
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;
}

static void testSuiteGlitchesInformational() {
  beginSuite(800, F("GLITCHES (INFORMATIONAL)"));
  suiteStart();

  beginCase(801, F("Enter NOM_OP then glitch sw80kv briefly"));
  swOn(P.sw3kv);
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;

  swOff(P.sw80kv);
  delayMicroseconds(GLITCH_PULSE_US);
  swOn(P.sw80kv);
  delay(10);
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;


  beginCase(802, F("Glitch 3kV I comparator briefly"));
  compFault(COMP_IDX_3KV_I);
  delayMicroseconds(GLITCH_PULSE_US);
  compSafe(COMP_IDX_3KV_I);
  delay(10);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,false,true)) return;

  delay(LOGIC_TIMER_3KV_MS + 10); // make sure we are out of 3k timer state
  suiteStart();
}


static void testSuiteOutputMirrorFlags() {
  beginSuite(900, F("OUTPUT MIRROR FLAGS (D22..D24) ARE CURRENT, NOT LATCHED"));
  suiteStart();

  beginCase(901, F("Baseline INTERLOCK: outputs OFF, LED ON; mirror flags 000"));
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,false,true)) return;

  beginCase(902, F("INTERLOCK: sw3kv ON -> A2 ON; mirror flags track immediately"));
  swOn(P.sw3kv);
  delay(1);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  beginCase(903, F("ACK toggle must NOT clear output mirror flags (still tracks current outputs)"));
  ackEdge();
  if (!expectOutputs(false,false,true,true)) return;

  beginCase(904, F("Enter NOM_OP, then toggle A0/A1 via switches; mirror flags track current (not latched)"));
  swOn(P.sw80kv);
  resetPulseNomOp();
  delay(1);
  if (!expectNomOp(true)) return;
  if (!expectOutputs(false,false,true,false)) return;

  swOn(P.swCCS);
  delay(1);
  if (!expectOutputs(true,false,true,false)) return;
  swOff(P.swCCS);
  delay(1);
  if (!expectOutputs(false,false,true,false)) return;

  swOn(P.swBeams);
  delay(1);
  if (!expectOutputs(false,true,true,false)) return;
  swOff(P.swBeams);
  delay(1);
  if (!expectOutputs(false,false,true,false)) return;

  beginCase(905, F("Mirror flags are NOT latched: rapid A0 toggles must always reflect CURRENT state"));
  for (uint8_t i = 0; i < 4; i++) {
    swOn(P.swCCS);
    delay(1);  
    if (!expectOutputs(true,false,true,false, 2)) return;
    swOff(P.swCCS); 
    delay(1);
    if (!expectOutputs(false,false,true,false, 2)) return;
  }

  beginCase(906, F("Exit NOM_OP via non-3kV comparator fault: outputs go safe and mirror flags MUST drop even though latches may stay set"));
  // Turn on A0 and A1 first
  swOn(P.swCCS);
  swOn(P.swBeams);
  delay(1);
  if (!expectOutputs(true,true,true,false)) return;

  // Trip a non-3k comparator (idx 0) -> should return to INTERLOCK (not timer)
  compFault(0);
  delay(2);
  compSafe(0);

  // Expect INTERLOCK now: A0/A1 OFF, A2 follows sw3kv (still ON), LED ON
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  beginCase(907, F("3kV I timer state: A2 forced OFF and mirror flags reflect OFF, then recover to follow sw3kv"));
  // Ensure sw3kv is ON and we are in interlock
  swOn(P.sw3kv);
  swOff(P.sw80kv);
  delay(1);
  if (!expectNomOp(false)) return;
  if (!expectOutputs(false,false,true,true)) return;

  compFault(COMP_IDX_3KV_I);
  delay(2);
  compSafe(COMP_IDX_3KV_I);

  // Timer entry should force A2 OFF
  if (!expectOutputs(false,false,false,true)) return;

  delay(LOGIC_TIMER_3KV_MS + 10);
  // Back to INTERLOCK, A2 follows sw3kv = ON
  if (!expectOutputs(false,false,true,true)) return;
}


static void runAllAutoTests() {
  autoRunning = true;
  gTestCase = 0;
  logPush(LOG_INFO, TAG4('A','L','L','0'));

  testSuiteEnterExitNomOp();
  testSuiteEachSwitchBehaviorInNomOp();
  testSuiteSwitchLatchStackingAndAckClear();
  testSuiteComparatorLatchStackingAndAckClear();
  testSuitePerComparatorIndividual();
  testSuite3kVSpecific();
  testSuiteNomOpEntryBlockedCases();
  testSuiteGlitchesInformational();
  testSuiteOutputMirrorFlags();

  autoRunning = false;
  Serial.println(F("\n=== Auto tests complete ==="));
}

// ========================= Manual Mode Commands =========================
static void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  help                       - show commands"));
  Serial.println(F("  obs                        - observe logic outputs now (labeled)"));
  Serial.println(F("  dump                       - dump RAM log (includes test-case numbers)"));
  Serial.println(F("  idle                       - set all inputs to safe/idle baseline"));
  Serial.println(F("  auto                       - run automated test suite"));
  Serial.println(F("  map                        - print wiring map"));
  Serial.println(F("  sw <3kv|beams|ccs|80kv> <on|off>"));
  Serial.println(F("  comp <0..7> <safe|fault>"));
  Serial.println(F("  ack toggle"));
  Serial.println(F("  reset pulse <ms>"));
  Serial.println(F("  glitch sw <3kv|beams|ccs|80kv>"));
  Serial.println(F("  glitch comp <0..7>"));
}

static void printMap() {
  Serial.println(F("\n--- MAP (Logic inputs -> Logic flags) ---"));
  Serial.println(F("PORTA / D22..D29:"));
  Serial.println(F("  D22 (PA0 bit0): A0 mirror (CCS Power Enable output)"));
  Serial.println(F("  D23 (PA1 bit1): A1 mirror (Beam Enable output)"));
  Serial.println(F("  D24 (PA2 bit2): A2 mirror (3kV Enable output)"));
  Serial.println(F("  D25 (PA3 bit3): Nom Op flag"));
  Serial.println(F("  D26 (PA4 bit4): latched 3kV timer-event flag"));
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F("  D")); Serial.print(SWITCH_LATCHES[i].flagPin);
    Serial.print(F(" (PA")); Serial.print(SWITCH_LATCHES[i].portaBit);
    Serial.print(F(" bit")); Serial.print(SWITCH_LATCHES[i].portaBit);
    Serial.print(F("): "));
    printPgm(SWITCH_LATCHES[i].name);
    Serial.print(F("  (Logic input D")); Serial.print(SWITCH_LATCHES[i].inPin);
    Serial.println(F(", latched)"));
  }
  Serial.print(F("  "));
  printPgm(SW_NAME_3KV);
  Serial.println(F(" raw state is no longer latched on PORTA."));

  Serial.println(F("\nPORTC / D30..D37 (packed bit0..7 corresponds to D30..D37):"));
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("  D")); Serial.print(COMPS[i].flagPin);
    Serial.print(F(" (packed bit")); Serial.print(i);
    Serial.print(F("): "));
    printPgm(COMPS[i].name);
    Serial.print(F("  (Logic input D")); Serial.print(COMPS[i].inPin);
    Serial.println(F(")"));
  }

  Serial.println(F("\nLogic outputs: A0=CCS enable, A1=Beam enable, A2=3kV enable, LED=D16"));
}

static void cmdSetSwitch(const String& which, const String& val) {
  uint8_t pin = 0xFF;
  if (which == "3kv") pin = P.sw3kv;
  else if (which == "beams") pin = P.swBeams;
  else if (which == "ccs") pin = P.swCCS;
  else if (which == "80kv") pin = P.sw80kv;

  if (pin == 0xFF) { Serial.println(F("Unknown switch")); return; }

  gTestCase = 0;
  if (val == "on")  { swOn(pin);  logPush(LOG_ACTION, TAG4('S','W','O','N'), pin, 1); }
  else if (val == "off") { swOff(pin); logPush(LOG_ACTION, TAG4('S','W','O','F'), pin, 0); }
  else { Serial.println(F("Use on/off")); return; }

  settle();
  Observe o = observeLogic();
  printObserveDetailed(o);
  uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
  logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
}

static void cmdSetComp(int idx, const String& val) {
  if (idx < 0 || idx > 7) { Serial.println(F("comp idx must be 0..7")); return; }

  gTestCase = 0;
  if (val == "safe")  { compSafe((uint8_t)idx);  logPush(LOG_ACTION, TAG4('C','S','A','F'), (uint8_t)idx, 0); }
  else if (val == "fault") { compFault((uint8_t)idx); logPush(LOG_ACTION, TAG4('C','F','L','T'), (uint8_t)idx, 1); }
  else { Serial.println(F("Use safe/fault")); return; }

  settle();
  Observe o = observeLogic();
  printObserveDetailed(o);
  uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
  logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
}

static void cmdResetPulse(int ms) {
  if (ms < 1) ms = 1;
  if (ms > 1000) ms = 1000;

  gTestCase = 0;
  pulseLowMs(P.reset, (uint16_t)ms);
  logPush(LOG_ACTION, TAG4('R','P','L','S'), (uint8_t)ms);

  settle();
  Observe o = observeLogic();
  printObserveDetailed(o);
  uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
  logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
}

// ========================= Simple Line Parser =========================
static String readLine() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = line;
      line = "";
      out.trim();
      return out;
    }
    line += c;
    if (line.length() > 140) { line = ""; }
  }
  return "";
}

static void handleCommand(const String& line) {
  if (line.length() == 0) return;

  String t[5];
  int n = 0;
  int start = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ' ') {
      if (i > start && n < 5) t[n++] = line.substring(start, i);
      start = i + 1;
    }
  }

  if (t[0] == "help") { printHelp(); return; }
  if (t[0] == "map")  { printMap(); return; }

  if (t[0] == "obs")  {
    gTestCase = 0;
    Observe o = observeLogic();
    printObserveDetailed(o);
    uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
    logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
    return;
  }

  if (t[0] == "dump") { logDump(); return; }
  if (t[0] == "idle") { gTestCase = 0; setAllSafeIdle(); Observe o = observeLogic(); printObserveDetailed(o); return; }
  if (t[0] == "auto") { runAllAutoTests(); return; }

  if (t[0] == "sw" && n >= 3) { cmdSetSwitch(t[1], t[2]); return; }
  if (t[0] == "comp" && n >= 3) { cmdSetComp(t[1].toInt(), t[2]); return; }

  if (t[0] == "ack" && n >= 2 && t[1] == "toggle") {
    gTestCase = 0;
    ackEdge();
    logPush(LOG_ACTION, TAG4('A','C','K','E'));
    settle();
    Observe o = observeLogic();
    printObserveDetailed(o);
    uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
    logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
    return;
  }

  if (t[0] == "reset" && n >= 3 && t[1] == "pulse") {
    cmdResetPulse(t[2].toInt());
    return;
  }

  if (t[0] == "glitch" && n >= 3 && t[1] == "sw") {
    uint8_t pin = 0xFF;
    if (t[2] == "3kv") pin = P.sw3kv;
    else if (t[2] == "beams") pin = P.swBeams;
    else if (t[2] == "ccs") pin = P.swCCS;
    else if (t[2] == "80kv") pin = P.sw80kv;
    if (pin == 0xFF) { Serial.println(F("Unknown switch")); return; }

    gTestCase = 0;
    swOff(pin); delayMicroseconds(GLITCH_PULSE_US); swOn(pin);
    logPush(LOG_ACTION, TAG4('G','L','S','W'), pin);
    delay(10);
    Observe o = observeLogic();
    printObserveDetailed(o);
    uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
    logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
    return;
  }

  if (t[0] == "glitch" && n >= 3 && t[1] == "comp") {
    int idx = t[2].toInt();
    if (idx < 0 || idx > 7) { Serial.println(F("comp idx must be 0..7")); return; }

    gTestCase = 0;
    compFault((uint8_t)idx); delayMicroseconds(GLITCH_PULSE_US); compSafe((uint8_t)idx);
    logPush(LOG_ACTION, TAG4('G','L','C','P'), (uint8_t)idx);
    delay(10);
    Observe o = observeLogic();
    printObserveDetailed(o);
    uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
    logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
}

// ========================= Setup / Loop =========================
void setup() {
  Serial.begin(BAUD);
  while (!Serial) { /* USB */ }

  initPins();
  setAllSafeIdle();

  Serial.println(F("\nKnob Box Logic Arduino - Test Harness Ready"));
  Serial.println(F("Type 'help' for commands. Type 'auto' to run full test suite."));
  printMap();

  Observe o = observeLogic();
  printObserveDetailed(o);
  uint16_t outBits = (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
  gTestCase = 0;
  logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, outBits);
}

void loop() {
  String line = readLine();
  if (line.length()) handleCommand(line);
}
