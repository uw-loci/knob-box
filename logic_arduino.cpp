/*
  Knob Box - Logic Arduino (Mega 2560 Rev 3)

  ========================= SAFETY / TRUTH TABLE =========================

  Comparator lines (D42-D49 on PORTL) are open-drain:
    - SAFE: comparator drives LOW
    - FAULT/INTERLOCK: comparator Hi-Z => Arduino pullup makes input HIGH
    - Disconnected wire => HIGH => treated as FAULT

  States:
    1) BI / INTERLOCK
       - CCS enable  (A0): OFF
       - Beam enable (A1): OFF
       - 3kV enable  (A2): follows 3kV enable switch (D10)
       - NomOp flag  (D25): LOW
       - Enter NomOp only on RESET press edge, with:
           * Arm 80kV switch asserted (D13)
           * 3kV enable switch asserted (D10)
           * ALL 8 comparators SAFE (all LOW)
       - Enter Timer if 3kI trips

    2) NOM_OP
       - CCS enable  (A0): follows CCS allow switch (D12)
       - Beam enable (A1): follows Arm Beams switch (D11)
       - 3kV enable  (A2): follows 3kV enable switch (D10)
       - NomOp flag  (D25): HIGH
       - Exit to BI if any interlock tripped:
           * any comparator goes FAULT (HIGH)  -> if 3kV V/I trip -> TIMER, else BI
           * Arm 80kV switch deasserts
           * 3kV enable switch deasserts

    3) 3KV_TIMER
       - CCS enable  (A0): OFF
       - Beam enable (A1): OFF
       - 3kV enable  (A2): forced OFF for 100 ms
       - After 100 ms: return to BI

  Flags:
    - PORTC (D30-D37): latched comparator FAULTS (HIGH=FAULT) since last ACK toggle
    - PORTA (D22-D29):
        * D22-24: reflect state of outputs A0-A2 respectively
        * D25 NomOp flag: HIGH = In NOM OP  ->  LOW = not NOM OP
        * D26-29 lached debounced switches (D10-13 asserted=1) since last ACK toggle
    - ACK toggle input (D14): any change clears latched interlock faults
*/

#include <Arduino.h>
#include <avr/io.h>

struct Sample; // redundant but arduino does cpp weird, and it freaks out if this forward dec isnt here
struct Output;

// ========================= Constants =========================
static constexpr uint32_t TIMER_3KV_MS   = 100;
static constexpr uint8_t  DEBOUNCE_BITS = 6;    // Can be set from 1 to 31 (do NOT set an illegal number for this, weird stuf could happen)

// ========================= Port mapping =========================
// Switches D10-13 => PB4-PB7
// Comparators D42-49 => PL7-PL0 (D49=PL0, D42=PL7)
// Flags out: D22-29 => PA0-PA7, D30-37 => PC7-PC0
// ACK D14 => PJ1, RESET BUTTON D15 => PJ0
// LED D16 => PH1
// Outputs A0/A1/A2 => PF0/PF1/PF2

// ========================== State Enum ==========================
enum class State : uint8_t {
  STATE_INTERLOCK  = 0,
  STATE_NOM_OP     = 1,
  STATE_3KV_TIMER  = 2
};

static State currentState = State::STATE_INTERLOCK;

// ========================= Masks =========================
static constexpr uint8_t MASK_SWITCHES_PORTB = _BV(PB4) | _BV(PB5) | _BV(PB6) | _BV(PB7);
static constexpr uint8_t MASK_PA_CCS    = _BV(PA0);
static constexpr uint8_t MASK_PA_BEAMS  = _BV(PA1);
static constexpr uint8_t MASK_PA_3KVEN  = _BV(PA2);
static constexpr uint8_t MASK_PA_NOMOP  = _BV(PA3);


// Outputs
static constexpr uint8_t MASK_OUT_CCS   = _BV(PF0); // A0
static constexpr uint8_t MASK_OUT_BEAM  = _BV(PF1); // A1
static constexpr uint8_t MASK_OUT_3KV   = _BV(PF2); // A2
static constexpr uint8_t MASK_INTERLOCK_LED = _BV(PH1); // D16

// ACK/RESET
static constexpr uint8_t MASK_ACK       = _BV(PJ1); // D14
static constexpr uint8_t MASK_RESET_BTN = _BV(PJ0); // D15

// creates a mask based off of debounce bits. Ex. 0...00011111 for 5
static constexpr uint32_t MASK_DEBOUNCE = (uint32_t)((1u << DEBOUNCE_BITS) - 1u);

// Comparator masks
// PL0=D49 3kV I, PL1=D48 3kV V
static constexpr uint8_t MASK_COMP_3KV      = (uint8_t)(_BV(PL0) | _BV(PL1));
static constexpr uint8_t MASK_COMP_3KV_I    = _BV(PL0);

// ===================== Runtime state globals  =====================

// stores the latest read of each port 
static uint8_t prevPORTF = 0;
static uint8_t prevPORTH = 0;

// Used to store the previous states of switches / button for debouncing
static uint32_t switchHist[4] = {0,0,0,0};
static bool    switchStable[4] = {false, false, false, false};
static uint32_t resetButtonHist = 0;
static bool    resetButtonStable = false;
static bool    prevResetButtonDb = false;

// ========================= Helpers =========================

static inline bool debounce_update(uint32_t &hist, bool sample, bool &stable) {
  hist = (uint32_t)((hist << 1) | (sample ? 1u : 0u));         // shift bits left, put most recent sample bit in lsb
  const uint32_t maskedHist = (uint32_t)(hist & MASK_DEBOUNCE);

  if (maskedHist == 0) {                      // fully debounced 0 register
    stable = false;
  } else if (maskedHist == MASK_DEBOUNCE) {   // fully debounced 1 reguster
    stable = true;
  }
  // Hold previous value;
  return stable;
}

static inline uint8_t debounce_switches(uint8_t swAssertPB) {
  const bool switch3kEn     = (swAssertPB & _BV(PB4)) != 0; // D10
  const bool switchArmBeams = (swAssertPB & _BV(PB5)) != 0; // D11
  const bool switchCCSAllow = (swAssertPB & _BV(PB6)) != 0; // D12
  const bool switch80kAllow = (swAssertPB & _BV(PB7)) != 0; // D13

  const bool debounce3kEn     = debounce_update(switchHist[0], switch3kEn,     switchStable[0]);
  const bool debounceArmBeams = debounce_update(switchHist[1], switchArmBeams, switchStable[1]);
  const bool debounceCCSAllow = debounce_update(switchHist[2], switchCCSAllow, switchStable[2]);
  const bool debounce80kAllow = debounce_update(switchHist[3], switch80kAllow, switchStable[3]);

  uint8_t out = 0;
  if (debounce3kEn)     out |= _BV(PB4);
  if (debounceArmBeams) out |= _BV(PB5);
  if (debounceCCSAllow) out |= _BV(PB6);
  if (debounce80kAllow) out |= _BV(PB7);
  return out;
}

static inline bool debounce_reset_button(bool resetButtonAsserted) {
  return debounce_update(resetButtonHist, resetButtonAsserted, resetButtonStable);
}

// ========================= Low-level IO (pullups always ON) =========================
static inline void io_init_registers() {
  // Switch inputs: PB4-PB7 with pullups
  DDRB  &= (uint8_t)~MASK_SWITCHES_PORTB;
  PORTB |= MASK_SWITCHES_PORTB;             // enable pull ups

  // Comparator inputs: Port L pullups ON 
  DDRL  = 0x00;
  PORTL = 0xFF;

  // ACK/RESET inputs: Port J - pullups on reset, ack
  DDRJ &= (uint8_t)~(MASK_ACK | MASK_RESET_BTN);
  PORTJ |= (MASK_ACK | MASK_RESET_BTN);

  // Flags outputs: Port A and C
  DDRA = 0xFF;
  DDRC = 0xFF;

  // Outputs: A0-A2 (PF0-PF2) and LED PH1
  DDRF |= (MASK_OUT_CCS | MASK_OUT_BEAM | MASK_OUT_3KV);
  DDRH |= MASK_INTERLOCK_LED;

  // Initial Outputs:
  PORTF &= (uint8_t)~(MASK_OUT_CCS | MASK_OUT_BEAM | MASK_OUT_3KV); // OFF
  PORTH |= MASK_INTERLOCK_LED;                                      // ON

  // Flags default:
  PORTA = 0x00;
  PORTC = 0x00;
}

struct Sample {
  uint8_t switchesAssertPortB;   // PB4-PB7: 1 means switch asserted (ON)
  uint8_t comparators;           // PL0-PL7: 1 means FAULT (HIGH)
  bool ackLevel;                 // Flag Ack
  bool resetAsserted;            // 1 means button asserted (PUSHED)
};

struct Output {
  bool ccsPowerEnable;
  bool armBeamsEnable;
  bool enable3kV;
  bool nomOp;
};

static inline void sample_inputs(Sample &s) {
  const uint8_t pinB = PINB;
  const uint8_t pinL = PINL;
  const uint8_t pinJ = PINJ;

  s.switchesAssertPortB = (uint8_t)(~pinB) & MASK_SWITCHES_PORTB;
  s.comparators         = pinL;
  s.ackLevel            = (pinJ & MASK_ACK) != 0;
  s.resetAsserted       = (pinJ & MASK_RESET_BTN) == 0;
}

static inline void write_flags(const Sample& sample, const Output& out)
{
  // Latched event flags since last ACK toggle
  static uint8_t prevFlagsComparators = 0;  // PL0-PL7 comparator bits (1 = fault)
  static uint8_t prevFlagsSwitches    = 0;  // PB4-PB7 asserted bits (1 = asserted)

  // Track ACK level to detect toggle
  static bool prevAck = false;

  // Clear flags on ACK toggle
  if (sample.ackLevel != prevAck) {
    prevFlagsComparators = 0;
    prevFlagsSwitches    = 0;
  }

  prevAck = sample.ackLevel;

  // Latch new interlock events
  prevFlagsComparators |= sample.comparators;
  prevFlagsSwitches |= (uint8_t)(sample.switchesAssertPortB & MASK_SWITCHES_PORTB);

  // Build PORTA (D22-D29)
  // PA0-PA2 (D22-D24): reflect outputs A0, A1, A2
  // PA3     (D25): NomOp flag
  // PA4-PA7 (D26-D29): latched switch flags
  uint8_t porta = 0x00;
  porta |= out.ccsPowerEnable ? MASK_PA_CCS   : 0;
  porta |= out.armBeamsEnable ? MASK_PA_BEAMS : 0;
  porta |= out.enable3kV      ? MASK_PA_3KVEN : 0;
  porta |= out.nomOp          ? MASK_PA_NOMOP : 0;
  porta |= (uint8_t)(prevFlagsSwitches & MASK_SWITCHES_PORTB);

  // write flags
  PORTA = porta;
  PORTC = prevFlagsComparators;
}


static inline void write_outputs(const Output& out) {

  uint8_t porth = prevPORTH;
  uint8_t portf = prevPORTF & ~(MASK_OUT_CCS | MASK_OUT_BEAM | MASK_OUT_3KV);   // clears CCS, ARM BEAMS, and 3kV EN bits, keeps all others


  portf |= out.ccsPowerEnable ? MASK_OUT_CCS  : 0;
  portf |= out.armBeamsEnable ? MASK_OUT_BEAM : 0;
  portf |= out.enable3kV      ? MASK_OUT_3KV  : 0;

  // LED active-high
  if (out.nomOp) {
    porth &= (uint8_t)~MASK_INTERLOCK_LED;
  } else {
    porth |= MASK_INTERLOCK_LED;
  }

  // write if there were changes
  if (portf != prevPORTF) { 
    PORTF = portf; 
    prevPORTF = portf; 
  }
  if (porth != prevPORTH) {
     PORTH = porth; 
     prevPORTH = porth; 
  }
}



// ========================= State machine step =========================
static inline void step() {
  

  static uint32_t timerEnterMs = 0;

  // Sample all inputs
  Sample inputSnapshot;
  sample_inputs(inputSnapshot);

  // Debounce swithces and buttons inputs
  const uint8_t switchesDebounced = debounce_switches(inputSnapshot.switchesAssertPortB);

  const bool resetButtonDb   = debounce_reset_button(inputSnapshot.resetAsserted);
  const bool resetButtonEdge = resetButtonDb && !prevResetButtonDb;

  prevResetButtonDb = resetButtonDb;
  

  // Switch states (debounced, asserted=1)
  const bool sw_3kv_enable = (switchesDebounced & _BV(PB4)) != 0; // D10
  const bool sw_arm_beams  = (switchesDebounced & _BV(PB5)) != 0; // D11
  const bool sw_ccs_allow  = (switchesDebounced & _BV(PB6)) != 0; // D12
  const bool sw_arm_80kv   = (switchesDebounced & _BV(PB7)) != 0; // D13

  // Outputs, all off by default
  Output outputSnapshot = {false, false, false, false};

  // ---- State machine ----
  switch (currentState) {
    case State::STATE_INTERLOCK: {

      // Check for 3kv overcurrent right away, if so, change outputs and break
      if (inputSnapshot.comparators & MASK_COMP_3KV_I) {
        currentState = State::STATE_3KV_TIMER;
        outputSnapshot.enable3kV = false;
        timerEnterMs = millis();
        break;
      }

      outputSnapshot.ccsPowerEnable  = false;
      outputSnapshot.armBeamsEnable = false;
      outputSnapshot.enable3kV  = sw_3kv_enable;

      // Enter NomOp only on reset button edge and all comparators SAFE and 80k asserted and 3k asserted
      if (resetButtonEdge && (inputSnapshot.comparators == 0) && sw_arm_80kv && sw_3kv_enable) {
        currentState = State::STATE_NOM_OP;
      }
      
    } break;

    case State::STATE_NOM_OP: {

      // Check for 3kv right away, if so, break and change output & flags
      if (inputSnapshot.comparators & MASK_COMP_3KV) {
        currentState = State::STATE_3KV_TIMER;
        outputSnapshot.ccsPowerEnable  = false;
        outputSnapshot.armBeamsEnable = false;
        outputSnapshot.enable3kV  = false;
        timerEnterMs = millis();
        break;
      }

      // If required interlock switches drop, or any other comparators trip return to BI
      if ((inputSnapshot.comparators != 0) ||!sw_arm_80kv || !sw_3kv_enable) {
        currentState = State::STATE_INTERLOCK;
        outputSnapshot.ccsPowerEnable  = false;
        outputSnapshot.armBeamsEnable = false;
        outputSnapshot.enable3kV  = sw_3kv_enable;
        break;
      }

      // NomOp: normal outputs follow their own switches
      outputSnapshot.enable3kV  = sw_3kv_enable;
      outputSnapshot.ccsPowerEnable  = sw_ccs_allow;
      outputSnapshot.armBeamsEnable = sw_arm_beams;

    } break;

    case State::STATE_3KV_TIMER: {
      // TIMER: CCS/Beam off, 3kV forced off for TIMER_3KV_MS
      outputSnapshot.ccsPowerEnable  = false;
      outputSnapshot.armBeamsEnable = false;
      outputSnapshot.enable3kV  = false;

      if ((uint32_t)(millis() - timerEnterMs) >= TIMER_3KV_MS) {
        currentState = State::STATE_INTERLOCK;
      }
    } break;
  }

  // ---- Flags  ----
  outputSnapshot.nomOp = (currentState == State::STATE_NOM_OP);
  write_flags(inputSnapshot, outputSnapshot);

  // ---- Drive outputs ----
  write_outputs(outputSnapshot);
}

// ========================= Arduino Hook Functions =========================
void setup() {
  io_init_registers();

  // Set prevport variables to the current register value
  prevPORTF = PORTF;
  prevPORTH = PORTH;

  // Reset state machine to interlock
  currentState = State::STATE_INTERLOCK;

  // set debounce histories to 0
  for (uint8_t i = 0; i < 4; i++) switchHist[i] = 0;
  resetButtonHist = 0;
  prevResetButtonDb = false;

  // Produce a initial flag image and set outputs
  Sample raw;
  sample_inputs(raw);
  Output out = {false, false, false, false};
  write_flags(raw, out);

  // Ensure outputs are in safe posture
  write_outputs(out);
}

void loop() {
  step();
}