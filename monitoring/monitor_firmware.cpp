/* Things to be tested/changed:
    - reset logic: does it work, does it need separate entry/exit thresholds
    - is ADS1115 scaling logic correct (setting gain) 
    - RS485 timing with watchdog: flush() has been removed, does the timeout need to be lengthened? maybe the Serial.println() needs to happen in loop()? 
    - more RS485 timing: is every 500ms good for sending data? 
    
    - LED functionality
    - display functionality
    */

#include <arduino-timer.h>
#include <Wire.h>

//Watch dog timer
#include <avr/wdt.h>

#include <LiquidCrystal_I2C.h>
// Adafruit’s unified ADS1X15 library handles both ADS1015 and ADS1115
#include <Adafruit_ADS1X15.h>

Timer<1> timer;

// Create the ADS1115 object
Adafruit_ADS1115 ads; 

// 20x4 LCD Address is 0x27
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Global ID tracking which arduino this is:
//   0: -1kV
//   1: +1kV
//   2: +3kV
//   3: +20kV
const int ps_id = 0;

// Specs for specific power supply
float hv_rated_V; // "rated voltage" really just the max output of the HVPSU
float i_rated_mA;
float hv_polarity;
float fullscale; // Matsusadas: 0-10V, Bertans: 0-5V

/* External ADC Channel Assignment: 
    A0 -> Current Monitoring 
    A1 -> Voltage Monitoring
    A2 -> Set Voltage */
#define CH_IMON 0
#define CH_VMON 1
#define CH_VSET 2

/* Calculated voltage, current values */
float measuredI_mA;
float measuredHV_V;
float programmedHV_V;

/* Internal ADC Channel Assignment:
    A0 -> Current Comparator threshold voltage 
    A1 -> Voltage Comparator threshold voltage */
#define I_THRESH A0
#define V_THRESH A1

// Reset State Thresholds (change after testing matsusada reset)
const float RESET_THRESHOLD_V = 0.3; // V
const float RESET_THRESHOLD_I = 0.3; // mA

/* Potentiometer Values */
float ipot_V;
float vpot_V;
// calculated thresholds
float icomp_mA;
float vcomp_V;

/* Scaling Parameters */
const float voltsPerCount = 0.1875F / 1000.0F;

/* Yellow Matsusada Reset Indicator Light */
#define RESET_LED 6
bool reset_state = false;

/* HVPSU Enable Switch (active low) */
#define HV_ENABLE 7

/* Arm Beams Switch (active low) */
#define ARM_BEAMS 11

/* CCS Power Allow Switch (active low) */
#define CCS_POWER_ALLOW 12

/* Arm 80kV Switch (active low) */
#define ARM_80KV 13

/* RS-485 Pins */
#define RS485_RX 19
#define RS485_TX 18
#define RS485_DIR 17 // low = receive mode

/* RS-485 buffer */
#define TX_BUF_SIZE 128
static char txBuf[TX_BUF_SIZE];

/* +3kV Flags from Logic Arduino (there are 13 flags) */
uint16_t flags = 0;

/* Read and Scale all values:
    From external ADC (ADS1115):
        - monitored current
        - monitoried voltage
        - set voltage 
    From Internal ADC:
        - current threshold
        - voltage threshold
    Other Signals (for 3kV):
        - Logic arduino flags */
bool read_value()
{
    
    float imon_raw = ads.readADC_SingleEnded(CH_IMON);
    float vmon_raw = ads.readADC_SingleEnded(CH_VMON);
    float vset_raw = ads.readADC_SingleEnded(CH_VSET);

    // Convert from raw ADC value to voltage between 0-5V.
    float imon_volts = imon_raw * voltsPerCount; 
    float vmon_volts = vmon_raw * voltsPerCount;
    float vset_volts = vset_raw * voltsPerCount; 

    // Now Convert to full scale HV
    measuredI_mA = (imon_volts / fullscale) * i_rated_mA;
    measuredHV_V = (vmon_volts / fullscale) * hv_rated_V * hv_polarity;
    programmedHV_V = (vset_volts / fullscale) * hv_rated_V * hv_polarity;

    ipot_V = analogRead(I_THRESH) * (5.0 / 1023.0);
    vpot_V = analogRead(V_THRESH) * (5.0 / 1023.0);

    // TODO convert potentiometer values to thresholds

    // Every time we read the values from the HVPSU, we want to check for the Matsusada Reset State.
    /* From HW Dev Spec: A potential reset state is determined if the output enable switch is on, 
    the potentiometer set voltage is greater than a near zero threshold, but both the actual 
    current and actual voltage are at near zero values.*/
    if (ps_id == 0 || ps_id == 1) { // Matsusadas
        if (reset_state == false && digitalRead(HV_ENABLE) == LOW && programmedHV_V > 1 && measuredHV_V < RESET_THRESHOLD_V && measuredI_mA < RESET_THRESHOLD_I) {
            // reset state found
            digitalWrite(RESET_LED, HIGH);
            reset_state = true;
        } else if (reset_state == true && digitalRead(HV_ENABLE) == LOW && programmedHV_V > 1 && measuredHV_V > RESET_THRESHOLD_V && measuredI_mA > RESET_THRESHOLD_I) {
            // psu has come out of reset state
            digitalWrite(RESET_LED, LOW);
            reset_state = false;
        }
    }

    if (ps_id == 2) { // only for +3kV Bertan
        // Logic Arduino flags
        flags = 0;

        flags |= (digitalRead(25) == HIGH) ? (1 << 0) : (0);
        flags |= (digitalRead(26) == HIGH) ? (1 << 1) : (0);
        flags |= (digitalRead(27) == HIGH) ? (1 << 2) : (0);
        flags |= (digitalRead(28) == HIGH) ? (1 << 3) : (0);
        flags |= (digitalRead(29) == HIGH) ? (1 << 4) : (0);
        flags |= (digitalRead(30) == HIGH) ? (1 << 5) : (0);
        flags |= (digitalRead(31) == HIGH) ? (1 << 6) : (0);
        flags |= (digitalRead(32) == HIGH) ? (1 << 7) : (0);
        flags |= (digitalRead(33) == HIGH) ? (1 << 8) : (0);
        flags |= (digitalRead(34) == HIGH) ? (1 << 9) : (0);
        flags |= (digitalRead(35) == HIGH) ? (1 << 10) : (0);
        flags |= (digitalRead(36) == HIGH) ? (1 << 11) : (0);
        flags |= (digitalRead(37) == HIGH) ? (1 << 12) : (0);
    }

    return true; // repeat? yes
}

char buffer[21]; // buffer to store formatted string to print

/* Display Measured Voltage, Current, Set Voltage, and Thresholds on LCD via I2C bus. */
bool display_value()
{

    // make current values printable -> convert from float to string
    char meas_i_buf[10];
    dtostrf(measuredI_mA, 6, 3, meas_i_buf);
    char comp_i_buf[10];
    dtostrf(icomp_mA, 3, 1, comp_i_buf);

    switch(ps_id) {
        case 0: // -1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   -%4dV     ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  -%4dV     ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", meas_i_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA -%4dV  ", comp_i_buf, vcomp_V);
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

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", meas_i_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %4dV  ", comp_i_buf, vcomp_V);
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

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", meas_i_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %4dV  ", comp_i_buf, vcomp_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            break;

        case 3: // +20kV Bertan

            // make voltage values printable -> convert from float to string
            char set_v_buf[10];
            dtostrf((programmedHV_V / 1000.0), 5, 2, set_v_buf);
            char meas_v_buf[10];
            dtostrf((measuredHV_V / 1000.0), 5, 2, meas_v_buf);
            char comp_v_buf[10];
            dtostrf((vcomp_V / 1000.0), 4, 1, comp_v_buf);

            snprintf(buffer, 21 * sizeof(char), "Set V:   +%skV  ", set_v_buf);
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%skV  ", meas_v_buf);
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA    ", meas_i_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %skV ", comp_i_buf, comp_v_buf);
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
    uint32_t set_voltage_e4 = (uint32_t)(programmedHV_V * 10000.0f);
    uint32_t actual_voltage_e4 = (uint32_t)(measuredHV_V * 10000.0f);
    uint32_t actual_current_e4 = (uint32_t)(measuredI_mA * 10000.0f);

    // Create a hex digit for statuses.
    uint8_t switch_states = 
        ((digitalRead(HV_ENABLE) == LOW ? 1 : 0) << 3) |
        ((digitalRead(ARM_BEAMS) == LOW ? 1 : 0) << 2) | 
        ((digitalRead(CCS_POWER_ALLOW) == LOW ? 1 : 0) << 1) |
        ((digitalRead(ARM_80KV) == LOW ? 1 : 0));
    switch_states = switch_states & 0x0F;

    // Print components to buffer to be transmitted
    snprintf(txBuf, TX_BUF_SIZE,
        "%u;%u;%u;%x;%u",
        set_voltage_e4,
        actual_voltage_e4,
        actual_current_e4,
        switch_states,
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

    // Configure HVPSU specs, do some HVPSU-specific operations
    switch (ps_id) {
        case 0: // -1kV Matsusada
            hv_rated_V = 1000.0;
            i_rated_mA = 30.0;
            hv_polarity = -1.0;
            fullscale = 10.0;

            pinMode(RESET_LED, OUTPUT);

            break;

        case 1: // +1kV Matsusada
            hv_rated_V = 1000.0;
            i_rated_mA = 30.0;
            hv_polarity = 1.0;
            fullscale = 10.0;

            pinMode(RESET_LED, OUTPUT);

            break;

        case 2: // +3kV Bertan
            hv_rated_V = 3000.0;
            i_rated_mA = 10.0;
            hv_polarity = 1.0;
            fullscale = 5.0;

            break;

        case 3: // +20kV Bertan
            hv_rated_V = 20000.0;
            i_rated_mA = 1.0;
            hv_polarity = 1.0;
            fullscale = 5.0;

            break;
    }

    
    timer.every(150, read_value);
    timer.every(200, display_value);
    timer.every(1000*60*30, clear_display); // every 30 minutes
    timer.every(500, transmit_data);

    wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout

}

void loop()
{
  wdt_reset(); //Feed dog
  timer.tick();
}