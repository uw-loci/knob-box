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
        * D26 latched 3kV timer flag: HIGH = timer state occurred since last ACK toggle
        * D27-29 latched debounced switches (D11-13 asserted=1) since last ACK toggle
    - ACK toggle input (D14): any change clears latched interlock faults
    - ACK echo output (D9): toggles on every observed ACK edge so the monitor Arduino can
      prove this firmware is alive and still sampling D14
*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/wdt.h>

struct Sample; // redundant but arduino does cpp weird, and it freaks out if this forward dec isnt here
struct Output;

// ========================= Constants =========================
static constexpr uint32_t TIMER_3KV_MS   = 100;
static constexpr uint8_t  DEBOUNCE_BITS = 6;    // Can be set from 1 to 31 (do NOT set an illegal number for this, weird stuf could happen)

enum class Timer3kVStateMode : uint8_t {
  DISABLE = 0,
  ENABLE  = 1
};

// Set to DISABLE to remove all transitions into STATE_3KV_TIMER, or ENABLE to keep transitions.
static constexpr Timer3kVStateMode TIMER_3KV_STATE_MODE = Timer3kVStateMode::ENABLE;

// ========================= Port mapping =========================
// Switches D10-13 => PB4-PB7
// Comparators D42-49 => PL7-PL0 (D49=PL0, D42=PL7)
// Flags out: D22-29 => PA0-PA7, D30-37 => PC7-PC0
// ACK D14 => PJ1, RESET BUTTON D15 => PJ0
// ACK echo D9 => PH6, LED D16 => PH1
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
static constexpr uint8_t MASK_PA_3KVTMR = _BV(PA4);
static constexpr uint8_t MASK_SWITCH_FLAGS_PORTB = _BV(PB5) | _BV(PB6) | _BV(PB7);


// Outputs
static constexpr uint8_t MASK_OUT_CCS   = _BV(PF0); // A0
static constexpr uint8_t MASK_OUT_BEAM  = _BV(PF1); // A1
static constexpr uint8_t MASK_OUT_3KV   = _BV(PF2); // A2
static constexpr uint8_t MASK_INTERLOCK_LED = _BV(PH1); // D16
static constexpr uint8_t MASK_ACK_ECHO      = _BV(PH6); // D9

// ACK/RESET
static constexpr uint8_t MASK_ACK       = _BV(PJ1); // D14
static constexpr uint8_t MASK_RESET_BTN = _BV(PJ0); // D15

// creates a mask based off of debounce bits. Ex. 0...00011111 for 5
static constexpr uint32_t MASK_DEBOUNCE = (uint32_t)((1u << DEBOUNCE_BITS) - 1u);

// Comparator masks
// PL0=D49 3kV I, PL1=D48 3kV V
static constexpr uint8_t MASK_COMP_3KV      = (uint8_t)(_BV(PL0) | _BV(PL1));
static constexpr uint8_t MASK_COMP_3KV_I    = _BV(PL0);

static inline bool timer_3kv_state_enabled() {
  return TIMER_3KV_STATE_MODE == Timer3kVStateMode::ENABLE;
}

// Capture reset cause and stop any inherited watchdog before normal startup runs.
// This follows the standard avr-libc early-startup watchdog pattern.
uint8_t resetCauseMirror __attribute__((section(".noinit")));
void watchdog_early_init(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));

void watchdog_early_init(void) {
  resetCauseMirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

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
static bool    ackEchoState = false;
static uint8_t latchedComparatorFlags = 0;
static uint8_t latchedSwitchFlags = 0;
static bool    latched3kVTimerFlag = false;
static bool    prevAckLevel = false;

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

  // Outputs: A0-A2 (PF0-PF2), ACK echo D9 (PH6), and LED D16 (PH1)
  DDRF |= (MASK_OUT_CCS | MASK_OUT_BEAM | MASK_OUT_3KV);
  DDRH |= (MASK_INTERLOCK_LED | MASK_ACK_ECHO);

  // Initial Outputs:
  PORTF &= (uint8_t)~(MASK_OUT_CCS | MASK_OUT_BEAM | MASK_OUT_3KV); // OFF
  PORTH &= (uint8_t)~MASK_ACK_ECHO;                                 // ACK echo LOW
  PORTH |= MASK_INTERLOCK_LED;                                      // LED ON

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

static inline void handle_ack_toggle(bool ackLevel) {
  if (ackLevel != prevAckLevel) {
    latchedComparatorFlags = 0;
    latchedSwitchFlags = 0;
    latched3kVTimerFlag = false;
    ackEchoState = !ackEchoState;   // Flip ack Echo State, written in write_outputs with portH
  }

  prevAckLevel = ackLevel;
}

static inline void enter_3kv_timer_state(uint32_t& timerEnterMs) {
  currentState = State::STATE_3KV_TIMER;
  timerEnterMs = millis();
  // D26 is an event latch, so only set it on timer-state entry.
  latched3kVTimerFlag = true;
}

static inline void write_flags(const Sample& sample, const Output& out)
{
  // Latch new interlock events
  latchedComparatorFlags |= sample.comparators;
  latchedSwitchFlags |= (uint8_t)(sample.switchesAssertPortB & MASK_SWITCH_FLAGS_PORTB);

  // Build PORTA (D22-D29)
  // PA0-PA2 (D22-D24): reflect outputs A0, A1, A2
  // PA3     (D25): NomOp flag
  // PA4     (D26): latched 3kV timer-event flag
  // PA5-PA7 (D27-D29): latched switch flags
  uint8_t porta = 0x00;
  porta |= out.ccsPowerEnable ? MASK_PA_CCS   : 0;
  porta |= out.armBeamsEnable ? MASK_PA_BEAMS : 0;
  porta |= out.enable3kV      ? MASK_PA_3KVEN : 0;
  porta |= out.nomOp          ? MASK_PA_NOMOP : 0;
  porta |= latched3kVTimerFlag ? MASK_PA_3KVTMR : 0;
  porta |= (uint8_t)(latchedSwitchFlags & MASK_SWITCH_FLAGS_PORTB);

  // write flags
  PORTA = porta;
  PORTC = latchedComparatorFlags;
}


static inline void write_outputs(const Output& out) {

  uint8_t porth = prevPORTH & (uint8_t)~(MASK_INTERLOCK_LED | MASK_ACK_ECHO);            // clears interlock LED and ack-echo bits, keeps all others
  uint8_t portf = prevPORTF & (uint8_t)~(MASK_OUT_CCS | MASK_OUT_BEAM | MASK_OUT_3KV);   // clears CCS, ARM BEAMS, and 3kV EN bits, keeps all others


  portf |= out.ccsPowerEnable ? MASK_OUT_CCS  : 0;
  portf |= out.armBeamsEnable ? MASK_OUT_BEAM : 0;
  portf |= out.enable3kV      ? MASK_OUT_3KV  : 0;

  if (ackEchoState) {
    porth |= MASK_ACK_ECHO;
  }

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
  handle_ack_toggle(inputSnapshot.ackLevel);

  // Debounce swithces and buttons inputs
  inputSnapshot.switchesAssertPortB = debounce_switches(inputSnapshot.switchesAssertPortB);
  
  const bool resetButtonDb   = debounce_reset_button(inputSnapshot.resetAsserted);
  const bool resetButtonEdge = resetButtonDb && !prevResetButtonDb;

  prevResetButtonDb = resetButtonDb;

  // Switch states (debounced, asserted=1)
  const bool sw_3kv_enable = (inputSnapshot.switchesAssertPortB & _BV(PB4)) != 0; // D10
  const bool sw_arm_beams  = (inputSnapshot.switchesAssertPortB & _BV(PB5)) != 0; // D11
  const bool sw_ccs_allow  = (inputSnapshot.switchesAssertPortB & _BV(PB6)) != 0; // D12
  const bool sw_arm_80kv   = (inputSnapshot.switchesAssertPortB & _BV(PB7)) != 0; // D13

  // Outputs, all off by default
  Output outputSnapshot = {false, false, false, false};

  // ---- State machine ----
  switch (currentState) {
    case State::STATE_INTERLOCK: {

      // If timer is enabled, check for 3kv overcurrent right away 
      if (timer_3kv_state_enabled()) {
        if (inputSnapshot.comparators & MASK_COMP_3KV_I) {
          enter_3kv_timer_state(timerEnterMs);
          break;
        }
      }

      // Enter NomOp only on reset button edge and all comparators SAFE and 80k asserted and 3k asserted
      if (resetButtonEdge && (inputSnapshot.comparators == 0) && sw_arm_80kv && sw_3kv_enable) {
        currentState = State::STATE_NOM_OP;
      }

    } break;

    case State::STATE_NOM_OP: {

      if (timer_3kv_state_enabled()) {
        // Check for 3kv comparator trips right away
        if (inputSnapshot.comparators & MASK_COMP_3KV) {
          enter_3kv_timer_state(timerEnterMs);
          break;
        }
      }

      // If required interlock switches drop, or any other comparators trip return to BI
      if ((inputSnapshot.comparators != 0) || !sw_arm_80kv || !sw_3kv_enable) {
        currentState = State::STATE_INTERLOCK;
        break;
      }

    } break;

    case State::STATE_3KV_TIMER: {
      // Leave timer state if 3kV I comparator is low, and it's been longer than the timer duration since we entered
      if (((inputSnapshot.comparators & MASK_COMP_3KV_I) == 0)  && 
              ((uint32_t)(millis() - timerEnterMs) >= TIMER_3KV_MS)) {
        currentState = State::STATE_INTERLOCK;
      }
    } break;
  }

  // ---- Assign Outputs ----
  switch (currentState) {
    case State::STATE_INTERLOCK:
      outputSnapshot.ccsPowerEnable  = false;
      outputSnapshot.armBeamsEnable  = false;
      outputSnapshot.enable3kV       = sw_3kv_enable;
      outputSnapshot.nomOp           = false;
      break;

    case State::STATE_NOM_OP:
      outputSnapshot.ccsPowerEnable  = sw_ccs_allow;
      outputSnapshot.armBeamsEnable  = sw_arm_beams;
      outputSnapshot.enable3kV       = sw_3kv_enable;
      outputSnapshot.nomOp           = true;
      break;

    case State::STATE_3KV_TIMER:
      outputSnapshot.ccsPowerEnable  = false;
      outputSnapshot.armBeamsEnable  = false;
      outputSnapshot.enable3kV       = false;
      outputSnapshot.nomOp           = false;
      break;
  }

  // ---- Flags  ----
  write_flags(inputSnapshot, outputSnapshot);

  // Flags are updated before outputs so the current-step ACK handling and latch state
  // are reflected together when write_outputs() drives the ack-back bit on PORTH.

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
  ackEchoState = false;
  latchedComparatorFlags = 0;
  latchedSwitchFlags = 0;
  latched3kVTimerFlag = false;
  prevAckLevel = false;

  // Produce a initial flag image and set outputs
  Sample raw;
  sample_inputs(raw);
  handle_ack_toggle(raw.ackLevel);
  Output out = {false, false, false, false};
  write_flags(raw, out);

  // Ensure outputs are in safe posture
  write_outputs(out);

  // Lightweight watchdog polling loop to reset the board in case of a lockup.  
  // Start watchdog supervision only after the board is already driving its safe defaults.
  wdt_enable(WDTO_500MS);
  wdt_reset();
}

void loop() {
  step();
  wdt_reset();
}
