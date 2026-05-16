/**
 * High-Voltage Power Supply Monitoring Firmware.
 * Please set SELECTED_PS_ID before compiling.
 */

#include <arduino-timer.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <ModbusRtu.h>

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

/////////////////////////////////////////////////
//////    EDIT BELOW TO SET POWER SUPPLY   //////
/////////////////////////////////////////////////

#define SELECTED_PS_ID PS_POS1KV

/////////////////////////////////////////////////

#if SELECTED_PS_ID != PS_POS1KV && SELECTED_PS_ID != PS_NEG1KV && \
    SELECTED_PS_ID != PS_20KV && SELECTED_PS_ID != PS_3KV
#error "Invalid SELECTED_PS_ID. Use PS_POS1KV, PS_NEG1KV, PS_20KV, or PS_3KV."
#endif

// Do Not Edit, edit #define SELECTED_PS_ID above instead
const uint8_t ps_id = SELECTED_PS_ID;
const char firmwareVersion[] = "2.0";

// Capture reset cause and stop any inherited watchdog before normal startup runs.
// This follows the standard avr-libc early-startup watchdog pattern.
uint8_t resetCauseMirror __attribute__((section(".noinit")));
void watchdog_early_init(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));

void watchdog_early_init(void) {
    resetCauseMirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

//============= MODBUS MAP ==================================
//===========================================================
/*
Input Registers (Function Code 04)
*/
#define IREG_V_SET_ADDR             0   // integer volts
#define IREG_V_READ_ADDR            1   // integer volts
#define IREG_I_READ_ADDR            2   // integer microamps
#define IREG_3KV_RESET_COUNT_ADDR   3   // count of reset events for 3kV Bertan

/*
"Discrete Inputs" (really also input registers)
The monitor exposes two packed DINPUT registers:
    4 = unlatched signals
    5 = latched flags
*/
#define DINPUT_UNLATCHED_SIGNALS_ADDR   4
#define DINPUT_LATCHED_FLAGS_ADDR       5

// note: when changing this map, update these register counts:
#define IREG_COUNT              4
#define DINPUT_COUNT            2
#define TOTAL_REG_COUNT         (IREG_COUNT + DINPUT_COUNT)
//============================================================
//============================================================

/**
 * System Constants
 */
#define RESET_ENTER_V       2                     // V
#define RESET_ENTER_I       0.5                     // mA
#define RESET_EXIT_V        2.5                     // V
#define RESET_EXIT_I        1.0                     // mA
#define VOLTS_PER_COUNT     0.1875F / 1000.0F       // correct with GAIN_TWO_THIRDS

/**
 * Pin assignments
 */
#define I_THRESH_PIN                    A0      // Current Comparator threshold voltage
#define V_THRESH_PIN                    A1      // Voltage Comparator threshold voltage
#define RESET_LED_PIN                   6       // Yellow Matsusada Reset Indicator Light
#define HV_ENABLE_SWITCH_PIN            7       // Active low;
#define ARM_80KV_SWITCH_PIN             8       // Active low
#define ARM_BEAMS_SWITCH_PIN            11      // Active low
#define CCS_POWER_ALLOW_SWITCH_PIN      12      // Active Low
#define RS485_TX_PIN                    18
#define RS485_DIR_PIN                   17      // low = receive mode
#define FLAGS_ACK_PIN                   14      // ack pin to Logic Arduino
#define LOGIC_ACK_ECHO_PIN              9       // ACK-back from Logic Arduino (toggles when Logic observes ACK edge)

// (logic arduino outputs / live signals)
#define OUTPUT_CCSPOWER_PIN             22
#define OUTPUT_ARMBEAMS_PIN             23
#define OUTPUT_3KV_ENABLE_PIN           24
// (live signal)
#define FLAG_NOMOP_PIN                  25      // Logic Arduino live Nom Op signal
// (latched flags)
#define FLAG_3KV_TIMER_PIN              26      // Logic Arduino latched 3kV timer-event flag
#define FLAG_ARMBEAMS_PIN               27
#define FLAG_CCSPOWER_PIN               28
#define FLAG_ARM80KV_PIN                29
#define FLAG_1K_VCOMP_PIN               30
#define FLAG_1K_ICOMP_PIN               31
#define FLAG_NEG_1K_VCOMP_PIN           32
#define FLAG_NEG_1K_ICOMP_PIN           33
#define FLAG_20K_VCOMP_PIN              34
#define FLAG_20K_ICOMP_PIN              35
#define FLAG_3K_VCOMP_PIN               36
#define FLAG_3K_ICOMP_PIN               37

const uint16_t UNLATCHED_SIGNAL_MASK_HVENABLE         = ((uint16_t)1 << 0);   // D7
const uint16_t UNLATCHED_SIGNAL_MASK_RESET_STATE_1KV  = ((uint16_t)1 << 1);   // reset state
const uint16_t UNLATCHED_SIGNAL_MASK_ARM80KV_ENABLE   = ((uint16_t)1 << 2);   // D8
const uint16_t UNLATCHED_SIGNAL_MASK_CCSPOWER_ENABLE  = ((uint16_t)1 << 3);   // D22
const uint16_t UNLATCHED_SIGNAL_MASK_ARMBEAMS_ENABLE  = ((uint16_t)1 << 4);   // D23
const uint16_t UNLATCHED_SIGNAL_MASK_3KV_ENABLE       = ((uint16_t)1 << 5);   // D24
const uint16_t UNLATCHED_SIGNAL_MASK_NOMOP            = ((uint16_t)1 << 6);   // D25
const uint16_t UNLATCHED_SIGNAL_MASK_LOGIC_ALIVE      = ((uint16_t)1 << 7);   // logic alive edge observed

const uint16_t LATCHED_FLAG_MASK_3KV_TIMER            = ((uint16_t)1 <<  4);  // D26
const uint16_t LATCHED_FLAG_MASK_ARMBEAMS_SWITCH      = ((uint16_t)1 <<  5);  // D27
const uint16_t LATCHED_FLAG_MASK_CCSPOWER_ALLOW       = ((uint16_t)1 <<  6);  // D28
const uint16_t LATCHED_FLAG_MASK_ARM80KV_SWITCH       = ((uint16_t)1 <<  7);  // D29
const uint16_t LATCHED_FLAG_MASK_1K_VCOMP             = ((uint16_t)1 <<  8);  // D30
const uint16_t LATCHED_FLAG_MASK_1K_ICOMP             = ((uint16_t)1 <<  9);  // D31
const uint16_t LATCHED_FLAG_MASK_NEG_1K_VCOMP         = ((uint16_t)1 << 10);  // D32
const uint16_t LATCHED_FLAG_MASK_NEG_1K_ICOMP         = ((uint16_t)1 << 11);  // D33
const uint16_t LATCHED_FLAG_MASK_20K_VCOMP            = ((uint16_t)1 << 12);  // D34
const uint16_t LATCHED_FLAG_MASK_20K_ICOMP            = ((uint16_t)1 << 13);  // D35
const uint16_t LATCHED_FLAG_MASK_3K_VCOMP             = ((uint16_t)1 << 14);  // D36
const uint16_t LATCHED_FLAG_MASK_3K_ICOMP             = ((uint16_t)1 << 15);  // D37

/**
 * Other declarations and initializations
 */
float               ratedHV_V;                      // "rated voltage" -- just the max output of the HVPSU
float               ratedI_mA;                      // same for "rated current"
const char          *powerSupplyName = "";
const char          *ratedOutputText = "";
float               measuredI_mA;                   // calculated values
float               measuredHV_V;                   // ""
float               programmedHV_V;                 // ""
float               iPot_V;                         // potentiometer values
float               vPot_V;                         // ""  
float               thresholdHV_V;                  // thresholds
float               thresholdI_mA;                  // ""           
bool                ack_state = false;              // false = HI-Z, true = LOW
bool                prevLogicAckEcho = false;       // D9 state sampled on previous 150 ms cycle
bool                resetState1kV = false;          // for Matsusadas, true if predicted to be currently in the reset state after an overcurrent event
char                buffer[21];                     // store formatted string to print to LCD
char                programmedHV_buf[10];           // store current/voltage values for printing
char                measuredHV_buf[10];             // ""    
char                thresholdHV_buf[10];            // ""
char                measuredI_buf[10];              // ""
char                thresholdI_buf[10];             // ""
bool                prevNomOpState = false;         // previous D25 state, used to clear the 3kV timer-event count on Nom Op entry
int                 resetState3kV = 0;              // count of latched 3kV timer events since the last Nom Op entry
uint16_t            latchedFlags = 0;               // sticky Modbus copy of D26-D37 until the next successful reply
bool                clearPending = false;           // defer sticky-flag clear until the next 150 ms sampling boundary
Timer<4, millis>    timer;
Adafruit_ADS1115    ads; 
LiquidCrystal_I2C   lcd(0x27, 20, 4);
Modbus slave(ps_id, Serial1, RS485_DIR_PIN);
uint16_t            modbus_regs[IREG_COUNT+DINPUT_COUNT]; // modbus register storage (input registers followed by discrete inputs)

/**
 * External ADC channel assignments
 */

#define CH_VSET 0
#define CH_IMON 1
#define CH_VMON 2

/**
 * Helper to round and clamp values before sending over RS-485.
 */
static inline uint16_t round_clamp_u16(float x)
{
    if (x < 0.0f) return 0;
    if (x > 65535.0f) return 65535;
    return (uint16_t)(x + 0.5f);
}

/**
 * Helper to ensure raw ADS reads are clamped to expected range.
 */
static inline int16_t clamp_i16_positive(float x)
{
    if (x < 0.0f) return 0;
    if (x > 32760.0f) return 32767;
    else return (int16_t)x;
}

static inline bool readHVEnableSwitchSignal()
{
    if (ps_id == PS_20KV) {
        // for +20kV Bertan, signal is active-high
        return digitalRead(HV_ENABLE_SWITCH_PIN) == HIGH;
    }

    return digitalRead(HV_ENABLE_SWITCH_PIN) == LOW;
}

static inline uint16_t readFlagsWord()
{
    uint16_t flags = 0;

    flags |= (digitalRead(FLAG_3KV_TIMER_PIN)    == HIGH) ? LATCHED_FLAG_MASK_3KV_TIMER       : 0;
    flags |= (digitalRead(FLAG_ARMBEAMS_PIN)     == HIGH) ? LATCHED_FLAG_MASK_ARMBEAMS_SWITCH : 0;
    flags |= (digitalRead(FLAG_CCSPOWER_PIN)     == HIGH) ? LATCHED_FLAG_MASK_CCSPOWER_ALLOW  : 0;
    flags |= (digitalRead(FLAG_ARM80KV_PIN)      == HIGH) ? LATCHED_FLAG_MASK_ARM80KV_SWITCH  : 0;
    flags |= (digitalRead(FLAG_1K_VCOMP_PIN)     == HIGH) ? LATCHED_FLAG_MASK_1K_VCOMP        : 0;
    flags |= (digitalRead(FLAG_1K_ICOMP_PIN)     == HIGH) ? LATCHED_FLAG_MASK_1K_ICOMP        : 0;
    flags |= (digitalRead(FLAG_NEG_1K_VCOMP_PIN) == HIGH) ? LATCHED_FLAG_MASK_NEG_1K_VCOMP    : 0;
    flags |= (digitalRead(FLAG_NEG_1K_ICOMP_PIN) == HIGH) ? LATCHED_FLAG_MASK_NEG_1K_ICOMP    : 0;
    flags |= (digitalRead(FLAG_20K_VCOMP_PIN)    == HIGH) ? LATCHED_FLAG_MASK_20K_VCOMP       : 0;
    flags |= (digitalRead(FLAG_20K_ICOMP_PIN)    == HIGH) ? LATCHED_FLAG_MASK_20K_ICOMP       : 0;
    flags |= (digitalRead(FLAG_3K_VCOMP_PIN)     == HIGH) ? LATCHED_FLAG_MASK_3K_VCOMP        : 0;
    flags |= (digitalRead(FLAG_3K_ICOMP_PIN)     == HIGH) ? LATCHED_FLAG_MASK_3K_ICOMP        : 0;

    return flags;
}

static inline bool readLogicAliveSignal()
{
    bool logicAckEcho = digitalRead(LOGIC_ACK_ECHO_PIN);
    bool logicAlive = (logicAckEcho != prevLogicAckEcho);
    prevLogicAckEcho = logicAckEcho;
    return logicAlive;
}

// forward dec for Matsusada reset state helper, which is used in readUnlatchedSignalsWord()
static inline bool checkMatsusadaResetState();

static inline uint16_t readUnlatchedSignalsWord()
{
    uint16_t signals = 0;

    signals |= readHVEnableSwitchSignal() ? UNLATCHED_SIGNAL_MASK_HVENABLE        : 0;

    if (ps_id == PS_POS1KV || ps_id == PS_NEG1KV) {
        // only for Matsusada, check the reset state
        signals |= checkMatsusadaResetState() ? UNLATCHED_SIGNAL_MASK_RESET_STATE_1KV : 0;
    }

    if (ps_id == PS_3KV) {
        signals |= (digitalRead(ARM_80KV_SWITCH_PIN)   == LOW)  ? UNLATCHED_SIGNAL_MASK_ARM80KV_ENABLE  : 0;
        signals |= (digitalRead(OUTPUT_CCSPOWER_PIN)   == HIGH) ? UNLATCHED_SIGNAL_MASK_CCSPOWER_ENABLE : 0;
        signals |= (digitalRead(OUTPUT_ARMBEAMS_PIN)   == HIGH) ? UNLATCHED_SIGNAL_MASK_ARMBEAMS_ENABLE : 0;
        signals |= (digitalRead(OUTPUT_3KV_ENABLE_PIN) == HIGH) ? UNLATCHED_SIGNAL_MASK_3KV_ENABLE      : 0;
        signals |= (digitalRead(FLAG_NOMOP_PIN)        == HIGH) ? UNLATCHED_SIGNAL_MASK_NOMOP           : 0;
        signals |= readLogicAliveSignal()                       ? UNLATCHED_SIGNAL_MASK_LOGIC_ALIVE     : 0;
    }

    return signals;
}

/**
 * Helper to check for Matsusada reset state.
 * 
 * From HW Dev Spec: A potential reset state is determined if the output enable switch is on, 
 * the potentiometer set voltage is greater than a near zero threshold, but both the actual 
 * current and actual voltage are at near zero values.
 * 
 * The matsusada will enter into a reset state after an overcurrent event. This state will 
 * disable the HV enable functionality and can only be cleared by shorting the reset pins on
 * the DB25 connector on the back of the power supply. This logic is intended to detect when
 * the power supply is in this state, so the dashboard can alert the user and guide them to reset.
 * 
 * Toggling HV Enable will NOT clear the reset state -- the user must short the reset pins by
 * hitting the physical matsusada momentary reset switch on the front of the knob box.
 */
static inline bool checkMatsusadaResetState() {
    if (ps_id != PS_POS1KV && ps_id != PS_NEG1KV) {
        return false;
    }

    bool hvEnabled = digitalRead(HV_ENABLE_SWITCH_PIN) == LOW;
    bool highSetV = programmedHV_V > 1.0;

    // enter reset state if both voltage and current below tresholds while HV is set above 1
    if (!resetState1kV && hvEnabled && highSetV &&
        measuredHV_V < RESET_ENTER_V && measuredI_mA < RESET_ENTER_I) {
        resetState1kV = true;
        digitalWrite(RESET_LED_PIN, HIGH);
    }

    // on recover, we only wait for current OR voltage to be above treshold
    else if (resetState1kV && (hvEnabled && (measuredHV_V > RESET_EXIT_V || measuredI_mA > RESET_EXIT_I))) {
        resetState1kV = false;
        digitalWrite(RESET_LED_PIN, LOW);
    }

    return resetState1kV;
}

/**
 * Helper to maintain the +3kV timer/reset-event counter.
 *
 * The Logic Arduino raises D26 when it enters the 3kV timer state and holds that event flag
 * until the next ACK edge. The +3kV monitor therefore counts any sampled-high D26 event flag
 * and clears the count whenever Nom Op rises
 */
void update3KVResetCounter(bool nomop, bool timerEventLatched) {
    if (!prevNomOpState && nomop) {
        // Clear the accumulated timer-event count when Nom Op is re-entered.
        resetState3kV = 0;
    } else if (timerEventLatched) {
        // Count each sampled D26 timer-event flag once per 150 ms monitor read/ACK cycle.
        resetState3kV++;
    }

    modbus_regs[IREG_3KV_RESET_COUNT_ADDR] = resetState3kV;
    prevNomOpState = nomop;
}

/**
 * Read and scale monitored voltage and current, set voltage, and potentiometer thresholds.
 * 
 * Perform Matsusada reset state logic.
 * 
 * Read Logic Arduino interface signals (only +3kV Bertan).
 */
bool read_value()
{
    /*
    Calculate the voltage and current values, then store them in RS-485 input regs.
    */
    int16_t imonRaw = ads.readADC_SingleEnded(CH_IMON);
    int16_t vmonRaw = ads.readADC_SingleEnded(CH_VMON);
    int16_t vsetRaw = ads.readADC_SingleEnded(CH_VSET);
    // Clamp raw readings to be in [0, 32760]
    imonRaw = clamp_i16_positive(imonRaw);
    vmonRaw = clamp_i16_positive(vmonRaw);
    vsetRaw = clamp_i16_positive(vsetRaw);
    // Convert from raw ADC counts to volts using ±6.144 V full-scale.
    float imonVolts = imonRaw * VOLTS_PER_COUNT; 
    float vmonVolts = vmonRaw * VOLTS_PER_COUNT;
    float vsetVolts = vsetRaw * VOLTS_PER_COUNT; 
    // Convert to full scale HV (assumed 0-5V input range to ADC)
    measuredI_mA = (imonVolts / 5.0) * ratedI_mA;
    measuredHV_V = (vmonVolts / 5.0) * ratedHV_V;
    programmedHV_V = (vsetVolts / 5.0) * ratedHV_V;
    // Store in input regs, rounding to the nearest volt and the nearest uA
    modbus_regs[IREG_V_SET_ADDR] = round_clamp_u16(programmedHV_V);
    modbus_regs[IREG_V_READ_ADDR] = round_clamp_u16(measuredHV_V);
    modbus_regs[IREG_I_READ_ADDR] = round_clamp_u16(measuredI_mA * 1000.0f);

    /* 
    Calculate voltage and current thresholds. 
    */
    iPot_V = analogRead(I_THRESH_PIN) * (5.0 / 1023.0);
    vPot_V = analogRead(V_THRESH_PIN) * (5.0 / 1023.0);
    // Convert pot values to fullscale thresholds
    thresholdHV_V = (vPot_V / 5.0) * ratedHV_V;
    thresholdI_mA = (iPot_V / 5.0) * ratedI_mA;

    /**
     *  3kV Bertan specific: 
     * 
     *      read the live unlatched signals, latched logic event flags, logic arduino outputs, and switch states.
     *
     *      update the 3kV timer/reset-event counter from the latched D26 timer flag.
     */
    if (ps_id == PS_3KV) { // only for +3kV Bertan

        uint16_t flags = readFlagsWord();

        if (clearPending) {
            latchedFlags = 0;
            clearPending = false;
        }

        latchedFlags |= flags;
        modbus_regs[DINPUT_LATCHED_FLAGS_ADDR] = latchedFlags;

        uint16_t unlatchedSignals = readUnlatchedSignalsWord();
        modbus_regs[DINPUT_UNLATCHED_SIGNALS_ADDR] = unlatchedSignals;

        // D25 is live in the unlatched register; D26 is a latched timer-event flag.
        bool nomop = (unlatchedSignals & UNLATCHED_SIGNAL_MASK_NOMOP) != 0;
        bool timerEventLatched = (flags & LATCHED_FLAG_MASK_3KV_TIMER) != 0;

        // Update the 3kV timer/reset-event counter from the sampled D26 event flag.
        update3KVResetCounter(nomop, timerEventLatched);

        // ack flag read so logic arduino can reset and continue
        if (ack_state == false) {
            pinMode(FLAGS_ACK_PIN, OUTPUT);
            digitalWrite(FLAGS_ACK_PIN, LOW);
            ack_state = true;
        } else {
            pinMode(FLAGS_ACK_PIN, INPUT); // high impedance
            ack_state = false;
        }

    } else {
        modbus_regs[DINPUT_UNLATCHED_SIGNALS_ADDR] = readUnlatchedSignalsWord();
        modbus_regs[DINPUT_LATCHED_FLAGS_ADDR] = 0;
    }

    return true;
}

/* Display Measured Voltage, Current, Set Voltage, and Thresholds on LCD via I2C bus. */
bool display_value()
{

    // make current values printable -> convert from float to string
    dtostrf(measuredI_mA, 6, 3, measuredI_buf);
    dtostrf(thresholdI_mA, 3, 1, thresholdI_buf);

    switch(ps_id) {
        case PS_POS1KV: // +1kV Matsusada

            // make voltage values printable -> convert from float to string
            dtostrf(programmedHV_V, 4, 0, programmedHV_buf);
            dtostrf(measuredHV_V, 4, 0, measuredHV_buf);
            dtostrf(thresholdHV_V, 4, 0, thresholdHV_buf);

            snprintf(buffer, 21 * sizeof(char), "Set V:   +%sV      ", programmedHV_buf);
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%sV      ", measuredHV_buf);
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %sV  ", thresholdI_buf, thresholdHV_buf);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            break;
        
        case PS_NEG1KV: // -1kV Matsusada

            // make voltage values printable -> convert from float to string
            dtostrf(programmedHV_V, 4, 0, programmedHV_buf);
            dtostrf(measuredHV_V, 4, 0, measuredHV_buf);
            dtostrf(thresholdHV_V, 4, 0, thresholdHV_buf);

            snprintf(buffer, 21 * sizeof(char), "Set V:   -%sV     ", programmedHV_buf);
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  -%sV     ", measuredHV_buf);
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA -%sV  ", thresholdI_buf, thresholdHV_buf);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            break;

        case PS_20KV: // +20kV Bertan

            // make voltage values printable -> convert from float to string
            dtostrf((programmedHV_V / 1000.0), 5, 2, programmedHV_buf);
            dtostrf((measuredHV_V / 1000.0), 5, 2, measuredHV_buf);
            dtostrf((thresholdHV_V / 1000.0), 4, 1, thresholdHV_buf);

            snprintf(buffer, 21 * sizeof(char), "Set V:   +%skV  ", programmedHV_buf);
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%skV  ", measuredHV_buf);
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA    ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %skV ", thresholdI_buf, thresholdHV_buf);
            lcd.setCursor(0,3);
            lcd.print(buffer);
            
            break;

        case PS_3KV: // +3kV Bertan

            // make voltage values printable -> convert from float to string
            dtostrf(programmedHV_V, 4, 0, programmedHV_buf);
            dtostrf(measuredHV_V, 4, 0, measuredHV_buf);
            dtostrf(thresholdHV_V, 4, 0, thresholdHV_buf);

            snprintf(buffer, 21 * sizeof(char), "Set V:   +%sV      ", programmedHV_buf);
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%sV      ", measuredHV_buf);
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %sV  ", thresholdI_buf, thresholdHV_buf);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            break;
    }

    return true;
}

bool clear_display() {
    lcd.clear();
    return true;
}

static void lcdPrintPaddedLine(uint8_t row, const char *text)
{
    char line[21];
    snprintf(line, sizeof(line), "%-20s", text);
    lcd.setCursor(0, row);
    lcd.print(line);
}

static void displayStartupInfo()
{
    char line[21];

    lcd.clear();
    lcdPrintPaddedLine(0, powerSupplyName);
    snprintf(line, sizeof(line), "Firmware v%s", firmwareVersion);
    lcdPrintPaddedLine(1, line);
    snprintf(line, sizeof(line), "%s %s", __DATE__, __TIME__);
    lcdPrintPaddedLine(2, line);
    lcdPrintPaddedLine(3, ratedOutputText);
}

static void failStartupAndTripWatchdog(const char *message)
{
    Serial.println(message);
    Serial.flush();
    wdt_enable(WDTO_1S);

    while (true) {
        // Intentionally stop here so the watchdog forces a reset.
    }
}

void setup()
{

    Serial.begin(9600);
    Serial.println("High Voltage Power Supply Monitoring with ADS1115");

    // Initialize the ADS1115 and I2C bus
    Wire.begin();
    if (!ads.begin()) {
        failStartupAndTripWatchdog("Failed to initialize ADS.");
    }
    ads.setDataRate(RATE_ADS1115_860SPS);
    ads.setGain(GAIN_TWOTHIRDS); // deafult, but want to be sure

    // Initialize the LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);

    pinMode(HV_ENABLE_SWITCH_PIN, INPUT_PULLUP);

    // Configure HVPSU specs
    switch (ps_id) {
        case PS_POS1KV: // +1kV Matsusada

            Serial.println("Configured for +1kV Matsusada");

            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            powerSupplyName = "+1kV Matsusada";
            ratedOutputText = "Rated +1kV 30mA";
            pinMode(RESET_LED_PIN, OUTPUT);
            break;

        case PS_NEG1KV: // -1kV Matsusada

            Serial.println("Configured for -1kV Matsusada");

            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            powerSupplyName = "-1kV Matsusada";
            ratedOutputText = "Rated -1kV 30mA";
            pinMode(RESET_LED_PIN, OUTPUT);
            break;

        case PS_20KV: // +20kV Bertan

            Serial.println("Configured for +20kV Bertan");

            ratedHV_V = 20000.0;
            ratedI_mA = 1.0;
            powerSupplyName = "+20kV Bertan";
            ratedOutputText = "Rated +20kV 1mA";
            break;

        case PS_3KV: // +3kV Bertan

            Serial.println("Configured for +3kV Bertan");

            ratedHV_V = 3000.0;
            ratedI_mA = 10.0;
            powerSupplyName = "+3kV Bertan";
            ratedOutputText = "Rated +3kV 10mA";

            // pins for logic arduino outputs / flags / ack
            pinMode(OUTPUT_CCSPOWER_PIN, INPUT);
            pinMode(OUTPUT_ARMBEAMS_PIN, INPUT);
            pinMode(OUTPUT_3KV_ENABLE_PIN, INPUT);
            pinMode(FLAG_NOMOP_PIN, INPUT);
            pinMode(FLAG_3KV_TIMER_PIN, INPUT);
            pinMode(FLAG_ARMBEAMS_PIN, INPUT);
            pinMode(FLAG_CCSPOWER_PIN, INPUT);
            pinMode(FLAG_ARM80KV_PIN, INPUT);
            pinMode(FLAG_1K_VCOMP_PIN, INPUT);
            pinMode(FLAG_1K_ICOMP_PIN, INPUT);
            pinMode(FLAG_NEG_1K_VCOMP_PIN, INPUT);
            pinMode(FLAG_NEG_1K_ICOMP_PIN, INPUT);
            pinMode(FLAG_20K_VCOMP_PIN, INPUT);
            pinMode(FLAG_20K_ICOMP_PIN, INPUT);
            pinMode(FLAG_3K_VCOMP_PIN, INPUT);
            pinMode(FLAG_3K_ICOMP_PIN, INPUT);
            pinMode(FLAGS_ACK_PIN, INPUT);
            pinMode(LOGIC_ACK_ECHO_PIN, INPUT_PULLUP);
            prevLogicAckEcho = digitalRead(LOGIC_ACK_ECHO_PIN); // initialize D9 edge detection
            prevNomOpState = digitalRead(FLAG_NOMOP_PIN);
            // switches only monitored by +3kV
            pinMode(ARM_BEAMS_SWITCH_PIN, INPUT_PULLUP);
            pinMode(CCS_POWER_ALLOW_SWITCH_PIN, INPUT_PULLUP);
            pinMode(ARM_80KV_SWITCH_PIN, INPUT_PULLUP);

            break;
    }

    displayStartupInfo();
    delay(3000);

    Serial.println("Initializing Modbus RTU Server on Serial1...");
    Serial1.begin(9600);
    slave.start();
    Serial.println("Modbus RTU Server started.");

    timer.every(150, read_value);
    timer.every(200, display_value);
    timer.every(1000UL*60UL*30UL, clear_display); // every 30 minutes

    wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout

}

void loop()
{
  wdt_reset(); //Feed dog

  int8_t pollResult = slave.poll(modbus_regs, TOTAL_REG_COUNT); // poll for requests from dashboard

  // The dashboard currently reads the full 0-5 block in one request.
  // A successful reply schedules a clear, but the clear itself is applied on the
  // next 150 ms read_value() boundary so sampling and second-tier latch rollover
  // stay aligned.
  if (ps_id == PS_3KV && pollResult > 4) {
    clearPending = true;
  }

  timer.tick();
}
