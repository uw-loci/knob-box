/* TODOs:
    - reset logic: does it work, does it need separate entry/exit thresholds

    - RS-485 Discrete Inputs:
        interlock state, "overcurrent" state? 
    */

#include <arduino-timer.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

//============= MODBUS MAP ==================================
//===========================================================
// Input Registers (Function Code 04)
#define IREG_HEALTH_ADDR        0   // TODO track health/error mode
#define IREG_V_SET_ADDR         1   // integer volts
#define IREG_V_READ_ADDR        2   // integer volts
#define IREG_I_READ_ADDR        3   // integer microamps

// Discrete Inputs (Function Code 02)
#define DINPUT_HVENABLE_ADDR            0
// just for matsusadas
#define DINPUT_RESET_STATE_ADDR         1
// just for 3kV Bertan
#define DINPUT_ARMBEAMS_ADDR            2
#define DINPUT_CCSPOWER_ADDR            3
#define DINPUT_ARM80KV_ADDR             4
#define DINPUT_3KV_ENABLE_ADDR          5 
// (flags)
#define DINPUT_NOMOP_FLAG_ADDR          6
#define DINPUT_3K_HVENABLE_FLAG_ADDR    7
#define DINPUT_ARMBEAMS_FLAG_ADDR       8
#define DINPUT_CCSPOWER_FLAG_ADDR       9
#define DINPUT_ARM80KV_FLAG_ADDR        10
#define DINPUT_1K_VCOMP_FLAG_ADDR       11
#define DINPUT_1K_ICOMP_FLAG_ADDR       12
#define DINPUT_NEG_1K_VCOMP_FLAG_ADDR   13
#define DINPUT_NEG_1K_ICOMP_FLAG_ADDR   14
#define DINPUT_20K_VCOMP_FLAG_ADDR      15
#define DINPUT_20K_ICOMP_FLAG_ADDR      16
#define DINPUT_3K_VCOMP_FLAG_ADDR       17
#define DINPUT_3K_ICOMP_FLAG_ADDR       18

// note: when changing this map, update these register counts:
#define IREG_COUNT              4
#define DINPUT_COUNT            19
//============================================================
//============================================================

/**
 * GLOBAL MONITORING ARDUINO ID
 *      - 1: -1kV Matsusada
 *      - 2: +1kV Matsusada
 *      - 3: +20kV Bertan
 *      - 4: +3kV Bertan
 */
const int ps_id = 1;

/**
 * System Constants
 */
#define RESET_THRESHOLD_V       0.3                 // V
#define RESET_THRESHOLD_I       0.3                 // mA
#define VOLTS_PER_COUNT         0.1875F / 1000.0F   // correct with GAIN_TWO_THIRDS

/**
 * Pin assignments
 */
#define I_THRESH_PIN                    A0      // Current Comparator threshold voltage
#define V_THRESH_PIN                    A1      // Voltage Comparator threshold voltage
#define RESET_LED_PIN                   6       // Yellow Matsusada Reset Indicator Light
#define HV_ENABLE_SWITCH_PIN            7       // Active low (shorted to D10)
#define ARM_BEAMS_SWITCH_PIN            11      // Active low
#define CCS_POWER_ALLOW_SWITCH_PIN      12      // Active Low
#define ARM_80KV_SWITCH_PIN             13      // Active low
#define RS485_TX_PIN                    18
#define RS485_DIR_PIN                   17      // low = receive mode
#define FLAGS_ACK_PIN                   14      // ack pin to Logic Arduino
// (logic arduino outputs)
#define OUTPUT_CCSPOWER_PIN             22
#define OUTPUT_ARMBEAMS_PIN             23
#define OUTPUT_3KV_ENABLE_PIN        24
// (flags)
#define FLAG_NOMOP_PIN                  25
#define FLAG_3K_HVENABLE_PIN            26
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

/**
 * Other declarations and initializations
 */
float               ratedHV_V;              // "rated voltage" -- just the max output of the HVPSU
float               ratedI_mA;              // same for "rated current"
float               measuredI_mA;           // calculated values
float               measuredHV_V;           // ""
float               programmedHV_V;         // ""
float               iPot_V;                 // potentiometer values
float               vPot_V;                 // ""  
float               thresholdHV_V;          // thresholds
float               thresholdI_mA;           // ""
bool                resetState = false;    
uint16_t            flags = 0;              
bool                ack_state = false;      // false = HI-Z, true = LOW
char                buffer[21];             // store formatted string to print to LCD
char                programmedHV_buf[10];   // store current/voltage values for printing
char                measuredHV_buf[10];     // ""    
char                thresholdHV_buf[10];    // ""
char                measuredI_buf[10];      // ""
char                thresholdI_buf[10];     // ""
Timer<4, millis>    timer;
Adafruit_ADS1115    ads; 
LiquidCrystal_I2C   lcd(0x27, 20, 4);

/**
 * External ADC channel assignments
 */
#define CH_IMON 0
#define CH_VMON 1
#define CH_VSET 2

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

/**
 * Read and scale monitored voltage and current, set voltage, and potentiometer thresholds.
 * 
 * Perform Matsusada reset state logic.
 * 
 * Read Logic Arduino Flags (only +3kV Bertan).
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
    ModbusRTUServer.inputRegisterWrite(IREG_V_SET_ADDR, round_clamp_u16(programmedHV_V));
    ModbusRTUServer.inputRegisterWrite(IREG_V_READ_ADDR, round_clamp_u16(measuredHV_V));
    ModbusRTUServer.inputRegisterWrite(IREG_I_READ_ADDR, round_clamp_u16(measuredI_mA * 1000.0f));

    /* 
    Calculate voltage and current thresholds. 
    */
    iPot_V = analogRead(I_THRESH_PIN) * (5.0 / 1023.0);
    vPot_V = analogRead(V_THRESH_PIN) * (5.0 / 1023.0);
    // Convert pot values to fullscale thresholds
    thresholdHV_V = (vPot_V / 5.0) * ratedHV_V;
    thresholdI_mA = (iPot_V / 5.0) * ratedI_mA;

    /*
    Read HV Enable switch state and store in RS-485 discrete input.
    */
    ModbusRTUServer.discreteInputWrite(DINPUT_HVENABLE_ADDR, digitalRead(HV_ENABLE_SWITCH_PIN) == LOW);

    /* From HW Dev Spec: A potential reset state is determined if the output enable switch is on, 
    the potentiometer set voltage is greater than a near zero threshold, but both the actual 
    current and actual voltage are at near zero values.*/
    if (ps_id == 1 || ps_id == 2) { // Matsusadas
        if (resetState == false && digitalRead(HV_ENABLE_SWITCH_PIN) == LOW && programmedHV_V > 1 && measuredHV_V < RESET_THRESHOLD_V && measuredI_mA < RESET_THRESHOLD_I) {
            // reset state found
            digitalWrite(RESET_LED_PIN, HIGH);
            resetState = true;
            ModbusRTUServer.discreteInputWrite(DINPUT_RESET_STATE_ADDR, 1);
        } else if (resetState == true && digitalRead(HV_ENABLE_SWITCH_PIN) == LOW && programmedHV_V > 1 && measuredHV_V > RESET_THRESHOLD_V && measuredI_mA > RESET_THRESHOLD_I) {
            // psu has come out of reset state
            digitalWrite(RESET_LED_PIN, LOW);
            resetState = false;
            ModbusRTUServer.discreteInputWrite(DINPUT_RESET_STATE_ADDR, 0);
        }
    }

    /**
     *  3kV Bertan specific: read flags, logic arduino outputs, and switch states.
     */
    if (ps_id == 4) { // only for +3kV Bertan

        // read switch state of arm 80kV
        ModbusRTUServer.discreteInputWrite(DINPUT_ARM80KV_ADDR, digitalRead(ARM_80KV_SWITCH_PIN) == LOW);

        // read logic arduino outputs
        ModbusRTUServer.discreteInputWrite(DINPUT_ARMBEAMS_ADDR, digitalRead(OUTPUT_ARM_BEAMS_PIN) == LOW);
        ModbusRTUServer.discreteInputWrite(DINPUT_CCSPOWER_ADDR, digitalRead(OUTPUT_CCSPOWER_PIN) == LOW);
        ModbusRTUServer.discreteInputWrite(DINPUT_3KV_ENABLE_ADDR, digitalRead(OUTPUT_3KV_ENABLE_PIN) == LOW);

        // read flags
        ModbusRTUServer.discreteInputWrite(DINPUT_NOMOP_FLAG_ADDR, digitalRead(FLAG_NOMOP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_3K_HVENABLE_FLAG_ADDR, digitalRead(FLAG_3K_HVENABLE_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_ARMBEAMS_FLAG_ADDR, digitalRead(FLAG_ARMBEAMS_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_CCSPOWER_FLAG_ADDR, digitalRead(FLAG_CCSPOWER_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_ARM80KV_FLAG_ADDR, digitalRead(FLAG_ARM80KV_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_1K_VCOMP_FLAG_ADDR, digitalRead(FLAG_1K_VCOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_1K_ICOMP_FLAG_ADDR, digitalRead(FLAG_1K_ICOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_NEG_1K_VCOMP_FLAG_ADDR, digitalRead(FLAG_NEG_1K_VCOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_NEG_1K_ICOMP_FLAG_ADDR, digitalRead(FLAG_NEG_1K_ICOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_20K_VCOMP_FLAG_ADDR, digitalRead(FLAG_20K_VCOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_20K_ICOMP_FLAG_ADDR, digitalRead(FLAG_20K_ICOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_3K_VCOMP_FLAG_ADDR, digitalRead(FLAG_3K_VCOMP_PIN));
        ModbusRTUServer.discreteInputWrite(DINPUT_3K_ICOMP_FLAG_ADDR, digitalRead(FLAG_3K_ICOMP_PIN));

        // ack flag read so logic arduino can reset and continue
        if (ack_state == false) {
            pinMode(FLAGS_ACK_PIN, OUTPUT);
            digitalWrite(FLAGS_ACK_PIN, LOW);
            ack_state = true;
        } else {
            pinMode(FLAGS_ACK_PIN, INPUT); // high impedance
            ack_state = false;
        }
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
        case 1: // -1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   -%4dV     ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  -%4dV     ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA -%4dV  ", thresholdI_buf, thresholdHV_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            break;
        
        case 2: // 1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   +%4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %4dV  ", thresholdI_buf, thresholdHV_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            break;

        case 3: // +20kV Bertan

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

        case 4: // +3kV Bertan
            snprintf(buffer, 21 * sizeof(char), "Set V:   +%4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", measuredI_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %4dV  ", thresholdI_buf, thresholdHV_V);
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

void setup()
{

    Serial.begin(9600);
    Serial.println("High Voltage Power Supply Monitoring with ADS1115");

    // Initialize the ADS1115 and I2C bus
    Wire.begin();
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS.");
        // while (1);
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
        case 1: // -1kV Matsusada
            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            pinMode(RESET_LED_PIN, OUTPUT);
            break;

        case 2: // +1kV Matsusada
            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            pinMode(RESET_LED_PIN, OUTPUT);
            break;

        case 3: // +20kV Bertan
            ratedHV_V = 20000.0;
            ratedI_mA = 1.0;
            break;

        case 4: // +3kV Bertan
            ratedHV_V = 3000.0;
            ratedI_mA = 10.0;

            // pins for logic arduino flags
            pinMode(25, INPUT);
            pinMode(26, INPUT);
            pinMode(27, INPUT);
            pinMode(28, INPUT);
            pinMode(29, INPUT);
            pinMode(30, INPUT);
            pinMode(31, INPUT);
            pinMode(32, INPUT);
            pinMode(33, INPUT);
            pinMode(34, INPUT);
            pinMode(35, INPUT);
            pinMode(36, INPUT);
            pinMode(37, INPUT); 
            pinMode(FLAGS_ACK_PIN, INPUT);
            // switches only monitored by +3kV
            pinMode(ARM_BEAMS_SWITCH_PIN, INPUT_PULLUP);
            pinMode(CCS_POWER_ALLOW_SWITCH_PIN, INPUT_PULLUP);
            pinMode(ARM_80KV_SWITCH_PIN, INPUT_PULLUP);

            break;
    }

    // Setup RS-485
    RS485.setPins(RS485_TX_PIN, RS485_DIR_PIN, RS485_DIR_PIN); // DE and RE pins shorted together
    RS485.begin(9600);
    if (!ModbusRTUServer.begin(ps_id, 9600)) { // address of slave lines up with ps_id
        Serial.println("Failed to start Modbus RTU Server");
        // while (1);
    }
    ModbusRTUServer.configureInputRegisters(0, IREG_COUNT);
    ModbusRTUServer.configureDiscreteInputs(0, DINPUT_COUNT);
    ModbusRTUServer.inputRegisterWrite(IREG_HEALTH_ADDR, (uint16_t)ps_id); // TODO

    timer.every(150, read_value);
    timer.every(200, display_value);
    timer.every(1000*60*30, clear_display); // every 30 minutes

    wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout

}

void loop()
{
  wdt_reset(); //Feed dog

  ModbusRTUServer.poll(); // poll for requests from dashboard

  timer.tick();
}