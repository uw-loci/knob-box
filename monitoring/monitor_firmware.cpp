/* TODOs:
    - reset logic: does it work, does it need separate entry/exit thresholds

    - is ADS1115 scaling logic correct? will the ADC voltages read all be 0-5V?
    
    - RS485: implement master-slave behavior

    - do the flag pins need to be INPUT_PULLUP? 
    */

#include <arduino-timer.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>

/**
 * GLOBAL MONITORING ARDUINO ID
 *      - 0: -1kV Matsusada
 *      - 1: +1kV Matsusada
 *      - 2: +3kV Bertan
 *      - 3: +20kV Bertan
 */
const int ps_id = 0;

/**
 * System Constants
 */
#define RESET_THRESHOLD_V       0.3                 // V
#define RESET_THRESHOLD_I       0.3                 // mA
#define TX_BUF_SIZE             128
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
float               tresholdI_mA;           // ""
bool                resetState = false;     
uint16_t            flags = 0;
char                buffer[21];             // store formatted string to print to LCD
char                programmedHV_buf[10];   // store current/voltage values for printing
char                measuredHV_buf[10];     // ""    
char                thresholdHV_buf[10];    // ""
char                measuredI_buf[10];      // ""
char                thresholdI_buf[10];     // ""
static char         txBuf[TX_BUF_SIZE];
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
 * Read and scale monitored voltage and current, set voltage, and potentiometer thresholds.
 * 
 * Perform Matsusada reset state logic.
 * 
 * Read Logic Arduino Flags (only +3kV Bertan).
 */
bool read_value()
{
    
    int16_t imonRaw = ads.readADC_SingleEnded(CH_IMON);
    int16_t vmonRaw = ads.readADC_SingleEnded(CH_VMON);
    int16_t vsetRaw = ads.readADC_SingleEnded(CH_VSET);

    // Convert from raw ADC value to voltage between 0-5V.
    float imonVolts = imonRaw * VOLTS_PER_COUNT; 
    float vmonVolts = vmonRaw * VOLTS_PER_COUNT;
    float vsetVolts = vsetRaw * VOLTS_PER_COUNT; 

    // Now Convert to full scale HV
    measuredI_mA = (imonVolts / 5.0) * ratedI_mA;
    measuredHV_V = (vmonVolts / 5.0) * ratedHV_V;
    programmedHV_V = (vsetVolts / 5.0) * ratedHV_V;

    // Get potentiometer values and convert to 0-5V.
    iPot_V = analogRead(I_THRESH) * (5.0 / 1023.0);
    vPot_V = analogRead(V_THRESH) * (5.0 / 1023.0);

    // Convert potentiometer values to thresholds
    thresholdHV_V = (vPot_V / 5.0) * ratedHV_V;
    thresholdI_mA = (iPot_V / 5.0) * ratedI_mA;

    /* From HW Dev Spec: A potential reset state is determined if the output enable switch is on, 
    the potentiometer set voltage is greater than a near zero threshold, but both the actual 
    current and actual voltage are at near zero values.*/
    if (ps_id == 0 || ps_id == 1) { // Matsusadas
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
    if (ps_id == 2) { // only for +3kV Bertan

        // read flags
        flags = 0;
        flags |= digitalRead(25) << 0;
        flags |= digitalRead(26) << 1;
        flags |= digitalRead(27) << 2;
        flags |= digitalRead(28) << 3;
        flags |= digitalRead(29) << 4;
        flags |= digitalRead(30) << 5;
        flags |= digitalRead(31) << 6;
        flags |= digitalRead(32) << 7;
        flags |= digitalRead(33) << 8;
        flags |= digitalRead(34) << 9;
        flags |= digitalRead(35) << 10;
        flags |= digitalRead(36) << 11;
        flags |= digitalRead(37) << 12;

        // ack read so logic arduino can reset and continue
        bool inv = !digitalRead(FLAGS_ACK_PIN);
        digitalWrite(FLAGS_ACK_PIN, inv);
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
        case 0: // -1kV Matsusada
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
        
        case 1: // 1kV Matsusada
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

        case 2: // +3kV Bertan
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
    }

    return true;
}

/**
 *  Send data to python dashboard.
 */
bool transmit_data()
{

    // TODO implement master-slave behavior:
    //   - this arduino should only TX data when called on by the dashboard

    // Format voltage, current values to be scaled integers
    uint32_t setVoltage_e4 = (uint32_t)(programmedHV_V * 10000.0f);
    uint32_t actualVoltage_e4 = (uint32_t)(measuredHV_V * 10000.0f);
    uint32_t actualCurrent_e4 = (uint32_t)(measuredI_mA * 10000.0f);

    // Create a hex digit for statuses.
    uint8_t switchStates = 
        ((digitalRead(HV_ENABLE) == LOW ? 1 : 0) << 3) |
        ((digitalRead(ARM_BEAMS) == LOW ? 1 : 0) << 2) | 
        ((digitalRead(CCS_POWER_ALLOW) == LOW ? 1 : 0) << 1) |
        ((digitalRead(ARM_80KV) == LOW ? 1 : 0));
    switchStates = switchStates & 0x0F;

    // Print components to buffer to be transmitted
    snprintf(txBuf, TX_BUF_SIZE,
        "%u;%u;%u;%x;%u",
        setVoltage_e4,
        actualVoltage_e4,
        actualCurrent_e4,
        switchStates,
        flags);

    // Send data string to python gui through Serial1
    digitalWrite(RS485_DIR, HIGH);
    delayMicroseconds(5);
    Serial1.println(txBuf);
    //Serial1.flush(); taken out due to watchdog reset concerns
    delayMicroseconds(5);
    digitalWrite(RS485_DIR, LOW);

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
    ads.begin();
    ads.setDataRate(RATE_ADS1115_860SPS);
    ads.setGain(GAIN_TWOTHIRDS); // deafult, but want to be sure

    // Initialize the LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);

    pinMode(HV_ENABLE, INPUT_PULLUP);
    pinMode(ARM_BEAMS, INPUT_PULLUP);
    pinMode(CCS_POWER_ALLOW, INPUT_PULLUP);
    pinMode(ARM_80KV, INPUT_PULLUP);

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

    pinMode(RS485_DIR, OUTPUT);
    digitalWrite(RS485_DIR, LOW); // start in receive mode
    Serial1.begin(9600);

    // Configure HVPSU specs
    switch (ps_id) {
        case 0: // -1kV Matsusada
            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            pinMode(RESET_LED, OUTPUT);
            break;

        case 1: // +1kV Matsusada
            ratedHV_V = 1000.0;
            ratedI_mA = 30.0;
            pinMode(RESET_LED, OUTPUT);
            break;

        case 2: // +3kV Bertan
            ratedHV_V = 3000.0;
            ratedI_mA = 10.0;
            break;

        case 3: // +20kV Bertan
            ratedHV_V = 20000.0;
            ratedI_mA = 1.0;
            break;
    }

    
    timer.every(150, read_value);
    timer.every(200, display_value);
    timer.every(1000*60*30, clear_display); // every 30 minutes

    wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout

}

void loop()
{
  wdt_reset(); //Feed dog

  // TODO: do rs-485 RX task

  timer.tick();
}