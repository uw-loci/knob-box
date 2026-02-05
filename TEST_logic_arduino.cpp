/*
  Knob Box - Logic Arduino Test Harness (Mega 2560 Rev 3)

  PURPOSE
  - Drives every Logic Arduino input (switches, comparators, ACK, RESET) safely
    using open-drain style: drive LOW to assert, Hi-Z to release.
  - Monitors every Logic Arduino output (flags ports, LED, A0-A2 outputs).
  - Runs automated test suites (expanded, per-switch + per-comparator + stacking/latching).
  - Offers an interactive manual mode where user can control inputs and observe response.
  - Keeps a structured RAM log that can be dumped after tests.

  ELECTRICAL SAFETY NOTES
  - This tester NEVER drives HIGH on lines that the Logic Arduino pull-ups.
    It only drives LOW (assert) or Hi-Z (release), preventing contention.
  - Strongly recommended: ~1k series resistors on each driven line for extra protection.
  - Ensure BOTH boards share GND.

  SERIAL UI
  - 115200 baud.
  - Type "help" for commands.

  WIRING DEFAULT (tester pin -> logic pin)
  Inputs we drive (Logic inputs):
    Switches: D10..D13
    ACK:      D14
    RESET:    D15
    Comparators: D42..D49 (PORTL PL7..PL0)

  Outputs we read (Logic outputs):
    LED:       D16
    Flags A:   D22..D29  (PORTA PA0..PA7)
    Flags C:   D30..D37  (PORTC PC7..PC0)  NOTE: physical D30..D37 order is reversed vs PC bit index
    Outputs:   A0/A1/A2

  IMPORTANT BIT MAPPING (Comparator latch -> observed D30..D37)
  - Logic writes: PORTC_bit_n = comparator_bit_n (PLn)
  - Mega pin mapping: PC7=D30 ... PC0=D37
  - Therefore:
      PL7 -> PC7 -> D30
      PL6 -> PC6 -> D31
      ...
      PL1 -> PC1 -> D36
      PL0 -> PC0 -> D37
  - This harness packs D30..D37 into a byte where bit0 = D30, bit7 = D37.
    So packedPortC bit = (7 - PL_index).
  - With comparator indices defined as:
      compIdx 0 = D42 = PL7
      compIdx 7 = D49 = PL0  (3kV I)
      compIdx 6 = D48 = PL1  (3kV V)
    the expected packedPortC bit is:
      expectedBit = compIdx (because compIdx0->bit0 ... compIdx7->bit7).
*/

#include <Arduino.h>
#include <avr/io.h>
struct Observe;

// ========================= Configuration =========================
static constexpr uint32_t BAUD = 115200;

// Log entries kept in RAM (ring buffer)
static constexpr uint16_t LOG_CAP = 512;

// Logic Arduino timer spec (used for expectations)
static constexpr uint32_t LOGIC_TIMER_3KV_MS = 100;

// Settling between stimulus and observation
static constexpr uint16_t DEFAULT_SETTLE_MS = 5;

// Glitch pulse widths
static constexpr uint16_t GLITCH_PULSE_US   = 50;

// 3kV comparator indices in our wiring map (see PinMap below)
static constexpr uint8_t COMP_IDX_3KV_V = 6; // D48 = PL1
static constexpr uint8_t COMP_IDX_3KV_I = 7; // D49 = PL0

// Reset pulse for NomOp entry (user-requested fixed length)
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

  // Comparator inputs Logic D42..D49 (PORTL PL7..PL0)
  // Index 0 = D42 (PL7) ... Index 7 = D49 (PL0)
  uint8_t comp[8];

  // Logic outputs we READ
  uint8_t led;       // Logic D16
  uint8_t flagA[8];  // Logic D22..D29 (PA0..PA7)
  uint8_t flagC[8];  // Logic D30..D37 (PC7..PC0)  array order is D30..D37
  uint8_t outA0;     // Logic A0 (PF0)
  uint8_t outA1;     // Logic A1 (PF1)
  uint8_t outA2;     // Logic A2 (PF2)
};

static const PinMap P = {
  10, 11, 12, 13, 14, 15,
  {42, 43, 44, 45, 46, 47, 48, 49},
  16,
  {22, 23, 24, 25, 26, 27, 28, 29},      // D22..D29
  {30, 31, 32, 33, 34, 35, 36, 37},      // D30..D37
  A0, A1, A2
};

// ========================= Open-drain Drive Helpers =========================
// Assert = drive LOW (OUTPUT LOW)
// Release = Hi-Z (INPUT), allowing Logic pullups to pull HIGH

static inline void driveLow(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

static inline void releaseHiZ(uint8_t pin) {
  pinMode(pin, INPUT); // Hi-Z
  // Do NOT enable pullup here; Logic side determines idle level.
}

static inline void pulseLowMs(uint8_t pin, uint16_t ms) {
  driveLow(pin);
  delay(ms);
  releaseHiZ(pin);
}

static inline void pulseLowUs(uint8_t pin, uint16_t us) {
  driveLow(pin);
  delayMicroseconds(us);
  releaseHiZ(pin);
}

// Comparator semantics:
// Logic: pullups ON, SAFE = LOW, FAULT = HIGH (Hi-Z causes pullup HIGH)
static inline void compSafe(uint8_t idx)  { driveLow(P.comp[idx]); }
static inline void compFault(uint8_t idx) { releaseHiZ(P.comp[idx]); }

// Switch semantics for your logic:
// Logic: pullups ON, switch asserted = LOW (logic inverts PINB)
// so "ON/ASSERT" = LOW, "OFF" = Hi-Z
static inline void swOn(uint8_t pin)  { driveLow(pin); }
static inline void swOff(uint8_t pin) { releaseHiZ(pin); }

// ACK edge generation (open-drain safe):
// Always produces a LOW->HiZ transition with a guaranteed edge.
static inline void ackEdge() {
  driveLow(P.ack);
  delayMicroseconds(60);
  releaseHiZ(P.ack);
  delayMicroseconds(60);
}

// Legacy name used throughout tests/commands
static inline void toggleAckEdge() {
  ackEdge();
}

// Reset helpers
static inline void resetPulseNomOp() {
  // User requirement: keep this at 20ms for NomOp entry tests.
  pulseLowMs(P.reset, RESET_PULSE_MS_NOMOP);
  // Small post-release delay helps ensure the Logic Arduino samples the rising edge cleanly.
  delay(10);
}

// ========================= Logging =========================
enum LogType : uint8_t {
  LOG_INFO   = 0,
  LOG_ACTION = 1,
  LOG_OBS    = 2,
  LOG_CHECK  = 3,
  LOG_FAIL   = 4,
  LOG_PASS   = 5
};

struct LogEntry {
  uint32_t t;
  LogType  type;
  uint32_t tag;   // identifies the check/action (FOURCC or hex)
  uint8_t  a;
  uint8_t  b;
  uint16_t x;
};

static LogEntry logBuf[LOG_CAP];
static uint16_t logHead = 0;
static uint16_t logCount = 0;

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

  if (isPrintable((uint8_t)s[0]) &&
      isPrintable((uint8_t)s[1]) &&
      isPrintable((uint8_t)s[2]) &&
      isPrintable((uint8_t)s[3])) {
    Serial.print(s);
  } else {
    Serial.print(F("0x"));
    Serial.print(tag, HEX);
  }
}

static inline void logPush(LogType type, uint32_t tag, uint8_t a=0, uint8_t b=0, uint16_t x=0) {
  logBuf[logHead] = { millis(), type, tag, a, b, x };
  logHead = (uint16_t)((logHead + 1) % LOG_CAP);
  if (logCount < LOG_CAP) logCount++;
}

static void logDump() {
  Serial.println(F("---- LOG DUMP (oldest -> newest) ----"));
  uint16_t start = (logCount == LOG_CAP) ? logHead : 0;
  for (uint16_t i = 0; i < logCount; i++) {
    uint16_t idx = (uint16_t)((start + i) % LOG_CAP);
    const LogEntry &e = logBuf[idx];
    Serial.print(e.t); Serial.print(F(" ms | "));
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
    Serial.print(F(" a=")); Serial.print(e.a);
    Serial.print(F(" b=")); Serial.print(e.b);
    Serial.print(F(" x=0x")); Serial.println(e.x, HEX);
  }
  Serial.println(F("---- END LOG ----"));
}

// ========================= Observations (read Logic outputs) =========================
struct Observe {
  uint8_t porta;  // D22..D29 packed bits 0..7 correspond to D22..D29
  uint8_t portc;  // D30..D37 packed bits 0..7 correspond to D30..D37
  bool led;       // D16
  bool outA0;     // A0 (digital read)
  bool outA1;     // A1 (digital read)
  bool outA2;     // A2 (digital read)
};

static inline uint8_t pack8(const uint8_t pins[8]) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (digitalRead(pins[i]) == HIGH) v |= (uint8_t)(1u << i);
  }
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

static void printObserve(const Observe& o) {
  Serial.print(F("LED=")); Serial.print(o.led);
  Serial.print(F(" A0=")); Serial.print(o.outA0);
  Serial.print(F(" A1=")); Serial.print(o.outA1);
  Serial.print(F(" A2=")); Serial.print(o.outA2);

  Serial.print(F("  PORTA(D22..D29)=0x")); Serial.print(o.porta, HEX);
  Serial.print(F("  PORTC(D30..D37)=0x")); Serial.println(o.portc, HEX);
}

static inline bool isNomOpFromFlags(const Observe& o) {
  // D25 is bit index 3 in packed PORTA (D22->bit0, D25->bit3)
  return (o.porta & (1u << 3)) != 0;
}

// ========================= Harness state =========================
static bool autoRunning = false;

// ========================= Init / Safe baseline =========================
static void initPins() {
  // Driven pins default to Hi-Z
  releaseHiZ(P.sw3kv);
  releaseHiZ(P.swBeams);
  releaseHiZ(P.swCCS);
  releaseHiZ(P.sw80kv);
  releaseHiZ(P.ack);
  releaseHiZ(P.reset);
  for (uint8_t i = 0; i < 8; i++) releaseHiZ(P.comp[i]);

  // Read pins as inputs (no pullups)
  pinMode(P.led, INPUT);
  for (uint8_t i = 0; i < 8; i++) pinMode(P.flagA[i], INPUT);
  for (uint8_t i = 0; i < 8; i++) pinMode(P.flagC[i], INPUT);
  pinMode(P.outA0, INPUT);
  pinMode(P.outA1, INPUT);
  pinMode(P.outA2, INPUT);
}

static void setAllSafeIdle() {
  // Switches OFF (released)
  swOff(P.sw3kv);
  swOff(P.swBeams);
  swOff(P.swCCS);
  swOff(P.sw80kv);

  // Comparators SAFE = LOW
  for (uint8_t i = 0; i < 8; i++) compSafe(i);

  // ACK/RESET released (HIGH via logic pullups)
  releaseHiZ(P.ack);
  releaseHiZ(P.reset);

  logPush(LOG_ACTION, TAG4('I','D','L','E'));
}

// Make suite starts consistent even if previous suite returned early on a failure.
static void suiteStart() {
  setAllSafeIdle();
  // Clear any latched flags deterministically
  ackEdge();
  delay(2);
  ackEdge();
  delay(2);
}

// ========================= Expectations / Check helpers =========================
static inline void settle(uint16_t ms = DEFAULT_SETTLE_MS) { delay(ms); }

static bool expectNomOp(bool wantNomOp, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const bool gotNomOp = isNomOpFromFlags(o);
  logPush(LOG_CHECK, TAG4('N','O','M','0'), (uint8_t)wantNomOp, (uint8_t)gotNomOp, o.porta);
  if (gotNomOp != wantNomOp) {
    logPush(LOG_FAIL, TAG4('N','O','M','F'), (uint8_t)wantNomOp, (uint8_t)gotNomOp, o.porta);
    Serial.println(F("FAIL: NomOp mismatch"));
    printObserve(o);
    return false;
  }
  logPush(LOG_PASS, TAG4('N','O','M','P'), (uint8_t)wantNomOp, (uint8_t)gotNomOp, o.porta);
  return true;
}

static bool expectOutputs(bool a0, bool a1, bool a2, bool led, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const bool ok = (o.outA0==a0) && (o.outA1==a1) && (o.outA2==a2) && (o.led==led);
  const uint8_t want = (uint8_t)((a0<<0)|(a1<<1)|(a2<<2)|(led<<3));
  const uint8_t got  = (uint8_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3));
  logPush(LOG_CHECK, TAG4('O','U','T','0'), want, got, (uint16_t)((o.porta<<8) | o.portc));
  if (!ok) {
    logPush(LOG_FAIL, TAG4('O','U','T','F'), want, got, (uint16_t)((o.porta<<8) | o.portc));
    Serial.println(F("FAIL: Output mismatch"));
    printObserve(o);
    return false;
  }
  logPush(LOG_PASS, TAG4('O','U','T','P'), want, got, (uint16_t)((o.porta<<8) | o.portc));
  return true;
}

// Switch latch expectation:
// Logic latches PB4..PB7 into PA4..PA7 => on our packed PORTA bits 4..7.
static uint8_t expectedSwitchLatchBits(bool sw3, bool beams, bool ccs, bool sw80) {
  uint8_t v = 0;
  if (sw3)   v |= (1u << 4);
  if (beams) v |= (1u << 5);
  if (ccs)   v |= (1u << 6);
  if (sw80)  v |= (1u << 7);
  return v;
}

static bool checkSwitchLatchExact(uint8_t wantUpperNibble, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const uint8_t got = (uint8_t)(o.porta & 0xF0);
  logPush(LOG_CHECK, TAG4('S','L','A','0'), wantUpperNibble, got, o.porta);
  if (got != wantUpperNibble) {
    logPush(LOG_FAIL, TAG4('S','L','A','F'), wantUpperNibble, got, o.porta);
    Serial.print(F("FAIL: Switch latch mismatch. Want 0x"));
    Serial.print(wantUpperNibble, HEX);
    Serial.print(F(" got 0x"));
    Serial.println(got, HEX);
    printObserve(o);
    return false;
  }
  logPush(LOG_PASS, TAG4('S','L','A','P'), wantUpperNibble, got, o.porta);
  return true;
}

// Comparator latch exact check:
// With the mapping described at top, comparator idx i => packed PORTC bit i.
static inline uint8_t bitForCompIdx(uint8_t compIdx) {
  return (uint8_t)(1u << compIdx);
}

static bool checkComparatorLatchExact(uint8_t want, uint16_t settleMs = DEFAULT_SETTLE_MS) {
  settle(settleMs);
  Observe o = observeLogic();
  const uint8_t got = o.portc;
  logPush(LOG_CHECK, TAG4('C','L','A','0'), want, got, o.portc);
  if (got != want) {
    logPush(LOG_FAIL, TAG4('C','L','A','F'), want, got, o.portc);
    Serial.print(F("FAIL: Comparator latch exact mismatch. Want 0x"));
    Serial.print(want, HEX);
    Serial.print(F(" got 0x"));
    Serial.println(got, HEX);
    printObserve(o);
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
    printObserve(o);
    return false;
  }
  logPush(LOG_PASS, TAG4('C','L','H','P'), wantMask, got, o.portc);
  return true;
}

// ========================= Automated Test Suites =========================

// Enter/exit NomOp baseline tests
static void testSuiteEnterExitNomOp() {
  Serial.println(F("\n=== Auto Test: ENTER/EXIT NOM_OP ==="));
  logPush(LOG_INFO, TAG4('T','1','0','0'));

  suiteStart();

  // Start: all switches OFF => INTERLOCK, outputs off, LED on
  swOff(P.sw3kv); swOff(P.sw80kv); swOff(P.swBeams); swOff(P.swCCS);
  expectNomOp(false);
  expectOutputs(false,false,false,true);

  // Turn on 3kV only => INTERLOCK, A2 follows
  swOn(P.sw3kv);
  expectNomOp(false);
  expectOutputs(false,false,true,true);

  // Enter NomOp requires sw80kv + sw3kv + reset edge + all comps safe
  swOn(P.sw80kv);
  resetPulseNomOp();
  expectNomOp(true);
  expectOutputs(false,false,true,false);

  // Exit NomOp: drop required switch 80kV
  swOff(P.sw80kv);
  expectNomOp(false);
  expectOutputs(false,false,true,true);

  // Re-enter NomOp again
  swOn(P.sw80kv);
  resetPulseNomOp();
  expectNomOp(true);
  expectOutputs(false,false,true,false);

  // Exit NomOp: drop required switch 3kV enable
  swOff(P.sw3kv);
  expectNomOp(false);
  expectOutputs(false,false,false,true);
}

// Per-switch behavior in NomOp (and confirming CCS/Beams do NOT force interlock)
static void testSuiteEachSwitchBehaviorInNomOp() {
  Serial.println(F("\n=== Auto Test: EACH SWITCH BEHAVIOR IN NOM_OP ==="));
  logPush(LOG_INFO, TAG4('T','2','0','0'));

  suiteStart();

  // Enter NomOp
  swOn(P.sw3kv);
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;

  // CCS allow toggle => stay NomOp, only A0 changes
  swOn(P.swCCS);
  expectNomOp(true);
  expectOutputs(true,false,true,false);

  swOff(P.swCCS);
  expectNomOp(true);
  expectOutputs(false,false,true,false);

  // Beams toggle => stay NomOp, only A1 changes
  swOn(P.swBeams);
  expectNomOp(true);
  expectOutputs(false,true,true,false);

  swOff(P.swBeams);
  expectNomOp(true);
  expectOutputs(false,false,true,false);

  // 80kV drop => MUST interlock
  swOff(P.sw80kv);
  expectNomOp(false);
  expectOutputs(false,false,true,true);

  // Re-enter NomOp
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;

  // 3kV enable drop => MUST interlock
  swOff(P.sw3kv);
  expectNomOp(false);
  expectOutputs(false,false,false,true);
}

// Switch latch stacking and ACK clear
static void testSuiteSwitchLatchStackingAndAckClear() {
  Serial.println(F("\n=== Auto Test: SWITCH LATCH STACKING + ACK CLEAR ==="));
  logPush(LOG_INFO, TAG4('T','3','0','0'));

  suiteStart();

  // Clear any old latches
  ackEdge();
  checkSwitchLatchExact(0x00);

  // Assert each switch one-by-one, latch accumulates
  swOn(P.sw3kv);   checkSwitchLatchExact(expectedSwitchLatchBits(true,false,false,false));
  swOn(P.swBeams); checkSwitchLatchExact(expectedSwitchLatchBits(true,true,false,false));
  swOn(P.swCCS);   checkSwitchLatchExact(expectedSwitchLatchBits(true,true,true,false));
  swOn(P.sw80kv);  checkSwitchLatchExact(expectedSwitchLatchBits(true,true,true,true));

  // Release all; latch should remain set
  swOff(P.sw3kv); swOff(P.swBeams); swOff(P.swCCS); swOff(P.sw80kv);
  checkSwitchLatchExact(expectedSwitchLatchBits(true,true,true,true));

  // ACK clears latch
  ackEdge();
  checkSwitchLatchExact(0x00);

  // Re-latch a single switch to ensure works after clear
  swOn(P.sw80kv);
  checkSwitchLatchExact(expectedSwitchLatchBits(false,false,false,true));
}

// Comparator latch stacking and ACK clear (exact bit mapping)
static void testSuiteComparatorLatchStackingAndAckClear() {
  Serial.println(F("\n=== Auto Test: COMPARATOR LATCH STACKING + ACK CLEAR (EXACT) ==="));
  logPush(LOG_INFO, TAG4('T','4','0','0'));

  suiteStart();

  // Clear any old latch
  ackEdge();
  checkComparatorLatchExact(0x00);

  // Trip two comparators => latch should be OR of their bits
  const uint8_t m0 = bitForCompIdx(0);
  const uint8_t m3 = bitForCompIdx(3);
  compFault(0);
  compFault(3);
  checkComparatorLatchHas((uint8_t)(m0 | m3));

  // Clear physical faults (SAFE) => latch remains until ACK
  compSafe(0);
  compSafe(3);
  checkComparatorLatchHas((uint8_t)(m0 | m3));

  // Add a third fault later => latch stacks further
  const uint8_t m7 = bitForCompIdx(7);
  compFault(7);
  checkComparatorLatchHas((uint8_t)(m0 | m3 | m7));

  // ACK clears comparator latch
  ackEdge();
  checkComparatorLatchExact(0x00);

  // Cleanup
  compSafe(7);
}

// Per-comparator tests (each comparator individually): set -> latch bit set, clear physical -> latch stays, ACK -> clears
static void testSuitePerComparatorIndividual() {
  Serial.println(F("\n=== Auto Test: PER-COMPARATOR INDIVIDUAL (EXACT) ==="));
  logPush(LOG_INFO, TAG4('T','5','0','0'));

  suiteStart();

  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(F("  Comparator idx ")); Serial.print(i);
    Serial.print(F(" (Logic D")); Serial.print(P.comp[i]); Serial.println(F(")"));

    // Ensure all safe
    for (uint8_t k = 0; k < 8; k++) compSafe(k);

    // Clear latch
    ackEdge();
    if (!checkComparatorLatchExact(0x00)) return;

    // Trip just this comparator
    compFault(i);
    const uint8_t want = bitForCompIdx(i);
    if (!checkComparatorLatchExact(want)) return;

    // Clear physical fault, latch should remain
    compSafe(i);
    if (!checkComparatorLatchExact(want)) return;

    // ACK clears
    ackEdge();
    if (!checkComparatorLatchExact(0x00)) return;
  }
}

// 3kV-specific behavior tests (I and V): timer transitions and latch bits
static void testSuite3kVSpecific() {
  Serial.println(F("\n=== Auto Test: 3kV I/V SPECIFIC ==="));
  logPush(LOG_INFO, TAG4('T','6','0','0'));

  suiteStart();

  // 1) Verify latch bits for 3kV I and 3kV V individually
  ackEdge();
  checkComparatorLatchExact(0x00);

  compFault(COMP_IDX_3KV_I);
  checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_I));
  compSafe(COMP_IDX_3KV_I);
  checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_I));
  ackEdge();
  checkComparatorLatchExact(0x00);

  compFault(COMP_IDX_3KV_V);
  checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_V));
  compSafe(COMP_IDX_3KV_V);
  checkComparatorLatchHas(bitForCompIdx(COMP_IDX_3KV_V));
  ackEdge();
  checkComparatorLatchExact(0x00);

  // 2) INTERLOCK timer entry is ONLY on 3kV I
  swOn(P.sw3kv);
  expectNomOp(false);
  expectOutputs(false,false,true,true);

  // Trip 3kV V ONLY: should NOT enter timer from interlock
  compFault(COMP_IDX_3KV_V);
  delay(2);
  expectNomOp(false);
  expectOutputs(false,false,true,true);
  compSafe(COMP_IDX_3KV_V);

  // Trip 3kV I: should enter timer, force A2 off
  compFault(COMP_IDX_3KV_I);
  delay(2);
  expectNomOp(false);
  expectOutputs(false,false,false,true);

  delay(LOGIC_TIMER_3KV_MS / 2);
  expectOutputs(false,false,false,true);

  delay(LOGIC_TIMER_3KV_MS / 2 + 10);
  expectNomOp(false);
  expectOutputs(false,false,true,true);
  compSafe(COMP_IDX_3KV_I);

  // 3) NOM_OP timer entry on 3kV V or I
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;
  expectOutputs(false,false,true,false);

  // Trip 3kV V => timer
  compFault(COMP_IDX_3KV_V);
  delay(2);
  expectNomOp(false);
  expectOutputs(false,false,false,true);
  delay(LOGIC_TIMER_3KV_MS + 10);
  expectNomOp(false);
  expectOutputs(false,false,true,true);
  compSafe(COMP_IDX_3KV_V);

  // Re-enter NomOp
  swOn(P.sw80kv);
  swOn(P.sw3kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;

  // Trip 3kV I => timer
  compFault(COMP_IDX_3KV_I);
  delay(2);
  expectNomOp(false);
  expectOutputs(false,false,false,true);
  delay(LOGIC_TIMER_3KV_MS + 10);
  expectNomOp(false);
  expectOutputs(false,false,true,true);
  compSafe(COMP_IDX_3KV_I);
}

// NomOp entry blocked cases (missing conditions / fault present)
static void testSuiteNomOpEntryBlockedCases() {
  Serial.println(F("\n=== Auto Test: NOM_OP ENTRY BLOCKED CASES ==="));
  logPush(LOG_INFO, TAG4('T','7','0','0'));

  suiteStart();

  // Case 1: reset edge but missing sw80kv -> should NOT enter NomOp
  swOn(P.sw3kv);
  swOff(P.sw80kv);
  resetPulseNomOp();
  expectNomOp(false);

  // Case 2: reset edge but missing sw3kv -> should NOT enter NomOp
  swOff(P.sw3kv);
  swOn(P.sw80kv);
  resetPulseNomOp();
  expectNomOp(false);

  // Case 3: reset edge but comparator fault present -> should NOT enter NomOp
  swOn(P.sw3kv);
  swOn(P.sw80kv);
  compFault(0);
  resetPulseNomOp();
  expectNomOp(false);
  compSafe(0);

  // Case 4: ensure it DOES enter when all conditions correct
  resetPulseNomOp();
  expectNomOp(true);
  expectOutputs(false,false,true,false);
}

// “Strange” case: glitch pulses (informational)
static void testSuiteGlitchesInformational() {
  Serial.println(F("\n=== Auto Test: GLITCHES (INFORMATIONAL) ==="));
  logPush(LOG_INFO, TAG4('T','8','0','0'));

  suiteStart();

  // Enter NomOp
  swOn(P.sw3kv);
  swOn(P.sw80kv);
  resetPulseNomOp();
  if (!expectNomOp(true)) return;

  // Glitch sw80kv OFF briefly then ON
  swOff(P.sw80kv);
  delayMicroseconds(GLITCH_PULSE_US);
  swOn(P.sw80kv);
  delay(10);
  Observe o = observeLogic();
  Serial.print(F("After sw80kv glitch: "));
  printObserve(o);
  logPush(LOG_OBS, TAG4('G','8','0','1'), o.porta, o.portc, (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3)));

  // Comparator glitch: 3kV I briefly FAULT then SAFE
  compFault(COMP_IDX_3KV_I);
  delayMicroseconds(GLITCH_PULSE_US);
  compSafe(COMP_IDX_3KV_I);
  delay(10);
  o = observeLogic();
  Serial.print(F("After 3kI glitch: "));
  printObserve(o);
  logPush(LOG_OBS, TAG4('G','8','0','2'), o.porta, o.portc, (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3)));

  // Restore baseline at end of informational suite to avoid impacting manual use
  suiteStart();
}

static void runAllAutoTests() {
  autoRunning = true;
  logPush(LOG_INFO, TAG4('A','L','L','0'));

  testSuiteEnterExitNomOp();
  testSuiteEachSwitchBehaviorInNomOp();
  testSuiteSwitchLatchStackingAndAckClear();
  testSuiteComparatorLatchStackingAndAckClear();
  testSuitePerComparatorIndividual();
  testSuite3kVSpecific();
  testSuiteNomOpEntryBlockedCases();
  testSuiteGlitchesInformational();

  autoRunning = false;
  Serial.println(F("\n=== Auto tests complete ==="));
}

// ========================= Manual Mode Commands =========================

static void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  help                       - show commands"));
  Serial.println(F("  obs                        - observe logic outputs now"));
  Serial.println(F("  dump                       - dump RAM log"));
  Serial.println(F("  idle                       - set all inputs to safe/idle baseline"));
  Serial.println(F("  auto                       - run automated test suite"));
  Serial.println(F("  map                        - print comparator index <-> D-pin mapping + PORTC bit mapping"));
  Serial.println(F("  sw <3kv|beams|ccs|80kv> <on|off>"));
  Serial.println(F("       on  = drive LOW (assert)"));
  Serial.println(F("       off = Hi-Z (release)"));
  Serial.println(F("  comp <0..7> <safe|fault>    - safe=LOW, fault=Hi-Z"));
  Serial.println(F("       comp idx maps to D42..D49 as 0..7"));
  Serial.println(F("  ack toggle                 - create an ACK edge"));
  Serial.println(F("  reset pulse <ms>           - pulse reset button low for ms"));
  Serial.println(F("  glitch sw <...>            - quick glitch demo"));
  Serial.println(F("  glitch comp <idx>          - quick glitch demo"));
}

static void printMap() {
  Serial.println(F("\nComparator Mapping:"));
  Serial.println(F("  idx  D-pin  PL-bit  Expected PORTC packed bit (D30..D37 packing)"));
  for (uint8_t i = 0; i < 8; i++) {
    // idx0 = D42 = PL7, idx7 = D49 = PL0
    const uint8_t dPin = P.comp[i];
    const int pl = 7 - i; // D42 is PL7
    const uint8_t packedBit = i; // as described at top
    Serial.print(F("   ")); Serial.print(i);
    Serial.print(F("    D")); Serial.print(dPin);
    Serial.print(F("     PL")); Serial.print(pl);
    Serial.print(F("      bit")); Serial.println(packedBit);
  }
  Serial.print(F("3kV V idx=")); Serial.print(COMP_IDX_3KV_V);
  Serial.print(F(" (D")); Serial.print(P.comp[COMP_IDX_3KV_V]); Serial.println(F(")"));
  Serial.print(F("3kV I idx=")); Serial.print(COMP_IDX_3KV_I);
  Serial.print(F(" (D")); Serial.print(P.comp[COMP_IDX_3KV_I]); Serial.println(F(")"));
}

static void cmdSetSwitch(const String& which, const String& val) {
  uint8_t pin = 0xFF;
  if (which == "3kv") pin = P.sw3kv;
  else if (which == "beams") pin = P.swBeams;
  else if (which == "ccs") pin = P.swCCS;
  else if (which == "80kv") pin = P.sw80kv;

  if (pin == 0xFF) { Serial.println(F("Unknown switch")); return; }

  if (val == "on")  { swOn(pin);  logPush(LOG_ACTION, TAG4('S','W','0','1'), pin, 1); }
  else if (val == "off") { swOff(pin); logPush(LOG_ACTION, TAG4('S','W','0','0'), pin, 0); }
  else { Serial.println(F("Use on/off")); return; }

  settle();
  printObserve(observeLogic());
}

static void cmdSetComp(int idx, const String& val) {
  if (idx < 0 || idx > 7) { Serial.println(F("comp idx must be 0..7")); return; }
  if (val == "safe")  { compSafe((uint8_t)idx);  logPush(LOG_ACTION, TAG4('C','P','5','0'), (uint8_t)idx, 0); }
  else if (val == "fault") { compFault((uint8_t)idx); logPush(LOG_ACTION, TAG4('C','P','F','1'), (uint8_t)idx, 1); }
  else { Serial.println(F("Use safe/fault")); return; }

  settle();
  printObserve(observeLogic());
}

static void cmdResetPulse(int ms) {
  if (ms < 1) ms = 1;
  if (ms > 1000) ms = 1000;
  pulseLowMs(P.reset, (uint16_t)ms);
  logPush(LOG_ACTION, TAG4('R','P','0','0'), (uint8_t)ms);
  settle();
  printObserve(observeLogic());
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

  // Tokenize by spaces
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
  if (t[0] == "obs")  { Observe o = observeLogic(); printObserve(o); logPush(LOG_OBS, TAG4('O','B','S','0'), o.porta, o.portc, (uint16_t)((o.outA0<<0)|(o.outA1<<1)|(o.outA2<<2)|(o.led<<3))); return; }
  if (t[0] == "dump") { logDump(); return; }
  if (t[0] == "idle") { setAllSafeIdle(); Observe o = observeLogic(); printObserve(o); return; }
  if (t[0] == "auto") { runAllAutoTests(); return; }

  if (t[0] == "sw" && n >= 3) { cmdSetSwitch(t[1], t[2]); return; }
  if (t[0] == "comp" && n >= 3) { cmdSetComp(t[1].toInt(), t[2]); return; }

  if (t[0] == "ack" && n >= 2 && t[1] == "toggle") {
    ackEdge();
    logPush(LOG_ACTION, TAG4('A','C','K','E'));
    settle();
    printObserve(observeLogic());
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
    swOff(pin); delayMicroseconds(GLITCH_PULSE_US); swOn(pin);
    logPush(LOG_ACTION, TAG4('G','L','S','1'), pin);
    delay(10);
    printObserve(observeLogic());
    return;
  }

  if (t[0] == "glitch" && n >= 3 && t[1] == "comp") {
    int idx = t[2].toInt();
    if (idx < 0 || idx > 7) { Serial.println(F("comp idx must be 0..7")); return; }
    compFault((uint8_t)idx); delayMicroseconds(GLITCH_PULSE_US); compSafe((uint8_t)idx);
    logPush(LOG_ACTION, TAG4('G','L','C','1'), (uint8_t)idx);
    delay(10);
    printObserve(observeLogic());
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
  printObserve(o);
  logPush(LOG_INFO, TAG4('B','E','E','F'), o.porta, o.portc);
}

void loop() {
  String line = readLine();
  if (line.length()) handleCommand(line);

  // Keep loop light for responsiveness.
}
