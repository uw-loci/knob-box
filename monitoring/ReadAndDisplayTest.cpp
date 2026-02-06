
/**
 * This file is a modified version of monitor_firmware.cpp.
 * It is intended for testing the read and display functions,
 * and uses dummy ADS readings instead of actual hardware.
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
#define DINPUT_ARMBEAMS_ADDR            1
#define DINPUT_CCSPOWER_ADDR            2
#define DINPUT_ARM80KV_ADDR             3 
// (flags)
#define DINPUT_NOMOP_FLAG_ADDR          4
#define DINPUT_3K_HVENABLE_FLAG_ADDR    5
#define DINPUT_ARMBEAMS_FLAG_ADDR       6
#define DINPUT_CCSPOWER_FLAG_ADDR       7
#define DINPUT_ARM80KV_FLAG_ADDR        8
#define DINPUT_1K_VCOMP_FLAG_ADDR       9
#define DINPUT_1K_ICOMP_FLAG_ADDR       10
#define DINPUT_NEG_1K_VCOMP_FLAG_ADDR   11
#define DINPUT_NEG_1K_ICOMP_FLAG_ADDR   12
#define DINPUT_20K_VCOMP_FLAG_ADDR      13
#define DINPUT_20K_ICOMP_FLAG_ADDR      14
#define DINPUT_3K_VCOMP_FLAG_ADDR       15
#define DINPUT_3K_ICOMP_FLAG_ADDR       16

// note: when changing this map, update these register counts:
#define IREG_COUNT              4
#define DINPUT_COUNT            17
//============================================================
//============================================================

/**
 * GLOBAL MONITORING ARDUINO ID
 *      - 1: -1kV Matsusada
 *      - 2: +1kV Matsusada
 *      - 3: +3kV Bertan
 *      - 4: +20kV Bertan
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
#define I_THRESH                A0      // Current Comparator threshold voltage
#define V_THRESH                A1      // Voltage Comparator threshold voltage
#define RESET_LED               6       // Yellow Matsusada Reset Indicator Light
#define HV_ENABLE               7       // Active low
#define ARM_BEAMS               11      // Active low
#define CCS_POWER_ALLOW         12      // Active Low
#define ARM_80KV                13      // Active low
#define RS485_RX                19
#define RS485_TX                18
#define RS485_DIR               17      // low = receive mode
#define FLAGS_ACK_PIN           14      // ack pin to Logic Arduino
// (flags)
#define FLAG_NOMOP              25
#define FLAG_3K_HVENABLE        26
#define FLAG_ARMBEAMS           27
#define FLAG_CCSPOWER           28
#define FLAG_ARM80KV            29
#define FLAG_1K_VCOMP           30
#define FLAG_1K_ICOMP           31
#define FLAG_NEG_1K_VCOMP       32
#define FLAG_NEG_1K_ICOMP       33
#define FLAG_20K_VCOMP          34
#define FLAG_20K_ICOMP          35
#define FLAG_3K_VCOMP           36
#define FLAG_3K_ICOMP           37

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

static inline int16_t clamp_i16_positive(float x)
{
    if (x < 0.0f) return 0;
    if (x > 32760.0f) return 32767;
    else return (int16_t)x;
}

/* Dummy ADS reads. */
static inline int16_t fake_ads_read()
{
    // Simulate 0–5V input on ADS1115 with GAIN_TWOTHIRDS
    // 5V ≈ 26667 counts
    return random(0, 26668);
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
    int16_t imonRaw = ads.fake_ads_read();
    int16_t vmonRaw = ads.fake_ads_read();
    int16_t vsetRaw = ads.fake_ads_read();
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
    iPot_V = analogRead(I_THRESH) * (5.0 / 1023.0);
    vPot_V = analogRead(V_THRESH) * (5.0 / 1023.0);
    // Convert pot values to fullscale thresholds
    thresholdHV_V = (vPot_V / 5.0) * ratedHV_V;
    thresholdI_mA = (iPot_V / 5.0) * ratedI_mA;

    /*
    Read switch states and store in RS-485 discrete inputs.
    */
    ModbusRTUServer.discreteInputWrite(DINPUT_HVENABLE_ADDR, digitalRead(HV_ENABLE) == LOW);
    ModbusRTUServer.discreteInputWrite(DINPUT_ARMBEAMS_ADDR, digitalRead(ARM_BEAMS) == LOW);
    ModbusRTUServer.discreteInputWrite(DINPUT_CCSPOWER_ADDR, digitalRead(CCS_POWER_ALLOW) == LOW);
    ModbusRTUServer.discreteInputWrite(DINPUT_ARM80KV_ADDR, digitalRead(ARM_80KV) == LOW);

    /* From HW Dev Spec: A potential reset state is determined if the output enable switch is on, 
    the potentiometer set voltage is greater than a near zero threshold, but both the actual 
    current and actual voltage are at near zero values.*/
    if (ps_id == 1 || ps_id == 2) { // Matsusadas
        if (resetState == false && digitalRead(HV_ENABLE) == LOW && programmedHV_V > 1 && measuredHV_V < RESET_THRESHOLD_V && measuredI_mA < RESET_THRESHOLD_I) {
            // reset state found
            digitalWrite(RESET_LED, HIGH);
            resetState = true;
        } else if (resetState == true && digitalRead(HV_ENABLE) == LOW && programmedHV_V > 1 && measuredHV_V > RESET_THRESHOLD_V && measuredI_mA > RESET_THRESHOLD_I) {
            // psu has come out of reset state
            digitalWrite(RESET_LED, LOW);
            resetState = false;
        }
    }

    /* Read Logic Arduino Flags */
    if (ps_id == 3) { // only for +3kV Bertan

        // read flags into RS-485 discrete inputs
        ModbusRTUServer.discreteInputWrite(DINPUT_NOMOP_FLAG_ADDR, digitalRead(FLAG_NOMOP));
        ModbusRTUServer.discreteInputWrite(DINPUT_3K_HVENABLE_FLAG_ADDR, digitalRead(FLAG_3K_HVENABLE));
        ModbusRTUServer.discreteInputWrite(DINPUT_ARMBEAMS_FLAG_ADDR, digitalRead(FLAG_ARMBEAMS));
        ModbusRTUServer.discreteInputWrite(DINPUT_CCSPOWER_FLAG_ADDR, digitalRead(FLAG_CCSPOWER));
        ModbusRTUServer.discreteInputWrite(DINPUT_ARM80KV_FLAG_ADDR, digitalRead(FLAG_ARM80KV));
        ModbusRTUServer.discreteInputWrite(DINPUT_1K_VCOMP_FLAG_ADDR, digitalRead(FLAG_1K_VCOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_1K_ICOMP_FLAG_ADDR, digitalRead(FLAG_1K_ICOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_NEG_1K_VCOMP_FLAG_ADDR, digitalRead(FLAG_NEG_1K_VCOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_NEG_1K_ICOMP_FLAG_ADDR, digitalRead(FLAG_NEG_1K_ICOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_20K_VCOMP_FLAG_ADDR, digitalRead(FLAG_20K_VCOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_20K_ICOMP_FLAG_ADDR, digitalRead(FLAG_20K_ICOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_3K_VCOMP_FLAG_ADDR, digitalRead(FLAG_3K_VCOMP));
        ModbusRTUServer.discreteInputWrite(DINPUT_3K_ICOMP_FLAG_ADDR, digitalRead(FLAG_3K_ICOMP));

        // ack read so logic arduino can reset and continue
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

        case 3: // +3kV Bertan
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

        case 4: // +20kV Bertan

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
    }

    return true;
}

bool clear_display() {
    lcd.clear();
    return true;
}

void setup()
{

    randomSeed(analogRead(A7));  // floating pin on Mega

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

    pinMode(HV_ENABLE, INPUT_PULLUP);

    // Configure HVPSU specs
    switch (ps_id) {
        case 1: // -1kV Matsusada
            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            pinMode(RESET_LED, OUTPUT);
            break;

        case 2: // +1kV Matsusada
            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            pinMode(RESET_LED, OUTPUT);
            break;

        case 3: // +3kV Bertan
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
            pinMode(ARM_BEAMS, INPUT_PULLUP);
            pinMode(CCS_POWER_ALLOW, INPUT_PULLUP);
            pinMode(ARM_80KV, INPUT_PULLUP);

            break;

        case 4: // +20kV Bertan
            ratedHV_V = 20000.0;
            ratedI_mA = 1.0;
            break;
    }

    // Setup RS-485
    RS485.setPins(RS485_TX, RS485_DIR, RS485_DIR); // DE and RE pins shorted together
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