/* Central Application Logic: (from HW dev spec)
    - read external ADC, done
    - read internal ADC, done
    - apply scaling, done
    - update LCD, done
    - tx over RS-485
    - detect fault conditions -> 3kV arduino recieves logic arduino flags to comm with dashboard
    - detect reset state for matsusadas and update led
    - read EN switch */

#include <arduino-timer.h>
#include <Wire.h>

//Watch dog timer
#include <avr/wdt.h>

#include <LiquidCrystal_I2C.h>
// Adafruit’s unified ADS1X15 library handles both ADS1015 and ADS1115
#include <Adafruit_ADS1X15.h>

// Create the ADS1115 object
Adafruit_ADS1115 ads; 

// 20x4 LCD Address is 0x27
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Global ID tracking which arduino this is:
//   0: -1kV
//   1: +1kV
//   2: +3kV
//   3: +20kV
const int ps_id;

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
#define I_COMP A0
#define V_COMP A1
// TODO is the above right?

// Reset State Thresholds (change after testing matsusada reset)
const float RESET_THRESHOLD_V = 0.3; // V
const float RESET_THRESHOLD_I = 0.3; // mA

/* Values read by internal ADC. */
float icomp_mA;
float vcomp_V;
// TODO reset/en

/* Scaling Parameters */
ads.setGain(GAIN_TWOTHIRDS); // deafult, but want to be sure
const float voltsPerCount = 0.1875F / 1000.0F;

/* Yellow Matsusada Reset Indicator Light */
#define RESET_LED 6
bool reset_state = false;

/* HVPSU Enable Switch (active low) */
#define HV_ENABLE 7

/* Arm Beams Switch (active low) */
#define ARM_BEAMS 11

/* CCS Power Allow Switch (active low) */
#define CSS_POWER_ALLOW 12

/* Arm 80kV Switch (active low) */
#define ARM_80KV 13

/* RS-485 Pins */
#define RS485_RX 19
#define RS485_TX 18
#define RS485_DIR 17 // low = receive mode

/* RS-485 buffer */
#define TX_BUF_SIZE 128
static char txBuf[TX_BUF_SIZE];

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

    icomp_mA = analogRead(I_COMP) * (5.0 / 1023.0);
    vcomp_V = analogRead(V_COMP) * (5.0 / 1023.0);

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

    // TODO Logic Arduino flags (only for +3kV)

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

    // TODO RS485 comm with the dashboard

    return true;
}

/**
 *  Send data to python dashboard.
 */
bool transmit_data()
{

    // Format voltage, current values to be scaled integers
    int32_t set_voltage_e4 = (int32_t)(programmedHV_V * 10000.0f);
    int32_t actual_voltage_e4 = (int32_t)(measuredHV_V * 10000.0f);
    int32_t actual_current_e4 = (int32_t)(measuredI_mA * 10000.0f);

    // Create a hex digit for statuses.
    uint8_t statuses = 
        ((digitalRead(HV_ENABLE) == LOW ? 1 : 0) << 3) |
        ((digitalRead(ARM_BEAMS) == LOW ? 1 : 0) << 2) | 
        ((digitalRead(CCS_POWER_ALLOW) == LOW ? 1 : 0) << 1) |
        ((digitalRead(ARM_80KV) == LOW ? 1 : 0));
    statuses = statuses & 0x0F;

    // TODO Create bit string for logic arduino flags.

    // Send data string to python gui through Serial1
    digitalWrite(RS485_DIR_PIN, HIGH);
    delayMicroseconds(5);
    Serial1.println(dataString);
    Serial1.flush();
    delayMicroseconds(5);
    digitalWrite(RS485_DIR_PIN, LOW);
}

void setup()
{
    Serial.begin(9600);
    Serial.println("High Voltage Power Supply Monitoring with ADS1115");

    // Initialize the ADS1115 and I2C bus
    Wire.begin();
    ads.begin();
    ads.setDataRate(RATE_ADS1115_860SPS);

    // Initialize the LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);

    pinMode(HV_ENABLE, INPUT_PULLUP);
    pinMode(ARM_BEAMS, INPUT_PULLUP);
    pinMode(CCS_POWER_ALLOW, INPUT_PULLUP);
    pinMode(ARM_80KV, INPUT_PULLUP);

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

    wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout

}

void loop()
{
  wdt_reset(); //Feed dog
  timer.tick();
}