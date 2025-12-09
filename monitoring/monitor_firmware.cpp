/* Central Application Logic: (from HW dev spec)
    - read external ADC
    - read internal ADC
    - apply scaling
    - update LCD
    - tx over RS-485
    - detect fault conditions
    - read EN switch */

    
// Include config file for the per-board settings
#include <config.h>

#include <arduino-timer.h>
#include <Wire.h>

//Watch dog timer
#include <avr/wdt.h>

#include <LiquidCrystal_I2C.h>
// Adafruit’s unified ADS1X15 library handles both ADS1015 and ADS1115
#include <Adafruit_ADS1X15.h>

// Create the ADS1115 object
Adafruit_ADS1115 ads; 

// Create the LCD object at address 0x27 or 0x3F, depending on your module
// (Check your LCD’s documentation for its I²C address.)
LiquidCrystal_I2C lcd(0x27, 20, 4);

//Power supply mode pin define
#define SET_BERTAN_3KV 48
#define SET_BERTAN_20KV 50
#define SET_MATSUSADA_1KV 52

// Global ID tracking which arduino this is:
//   0: -1kV
//   1: +1kV
//   2: +3kV
//   3: +20kV
int ps_id;

/* External ADC Channel Assignment: 
    A0 -> Current Monitoring 
    A1 -> Voltage Monitoring
    A2 -> Set Voltage */
#define CH_IMON 0
#define CH_VMON 1
#define CH_VSET 2

/* Values read by External ADC. */
float measuredI_mA;
float measuredHV_V;
float programmedHV_V;

/* Internal ADC Channel Assignment:
    A0 -> Current Comparator threshold voltage 
    A1 -> Voltage Comparator threshold voltage */
#define I_COMP A0
#define V_COMP A1

/* Values read by internal ADC. */
float icomp_mA;
float vcomp_V;
// TODO reset/en

/* Scaling Parameters */
const float voltsPerCount = 0.1875F / 1000.0F;

/* Read and Scale all values:
    From external ADC (ADS1115):
        - monitored current
        - monitoried voltage
        - set voltage 
    From Internal ADC:
        - current threshold
        - voltage threshold
    Other Signals (for 3kV):
        - Logic arduino flags
        - 80kV Armed Switch
        - CCS Power Switch
        - Beams Armed Switch
    Other Signals (for Matsusadas):
        - reset state (control the yellow LEDs */
void read_value()
{
    
    float imon_raw = ads.readADC_SigleEnded(CH_IMON);
    float vmon_raw = ads.readADC_SigleEnded(CH_VMON);
    float vset_raw = ads.readADC_SigleEnded(CH_VSET);

    // Convert from raw ADC value to voltage between 0-5V.
    float imon_volts = imon_raw * voltsPerCount; 
    float vmon_volts = vmon_raw * voltsPerCount;
    float vset_volts = vset_raw * voltsPerCount; 

    // Now Convert to full scale HV
    float measuredI_mA = (imon_volts / 5.0) * voltage_mulitplier;
    float measuredHV_V = (vmon_volts / 5.0) * voltage_mulitplier;
    float programmedHV_V = (vset_volts / 5.0) * voltage_mulitplier;


    icomp_mA = analogRead(I_COMP) * (5.0 / 1023.0);
    vcomp_V = analogRead(V_COMP) * (5.0 / 1023.0);

}

/* Display Measured Voltage, Current, Set Voltage, and Thresholds on LCD via I2C bus. */
void display_value()
{
    switch(ps_id) {
        case 0: // -1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   -%4dV     ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  -%4dV     ", int(measuredHV_V));
            lcd.setCursor(0,1)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA   ", measuredI_mA);
            lcd.setCursor(0,2)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %.1fmA %4dV  ", icomp_mA, vcomp_V);
            lcd.setCursor(0,3)
            lcd.print(buffer);

            return;
        
        case 1: // 1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   %4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  %4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA   ", measuredI_mA);
            lcd.setCursor(0,2)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %.1fmA %4dV  ", icomp_mA, vcomp_V);
            lcd.setCursor(0,3)
            lcd.print(buffer);

            return;

        case 2: // +3kV Bertan
            snprintf(buffer, 21 * sizeof(char), "Set V:   %4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  %4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA   ", measuredI_mA);
            lcd.setCursor(0,2)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %.1fmA %4dV  ", icomp_mA, vcomp_V);
            lcd.setCursor(0,3)
            lcd.print(buffer);

            return;

        case 3: // +20kV Bertan
            snprintf(buffer, 21 * sizeof(char), "Set V:   +%.3fkV  ", (programmedHV_V / 1000.0));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%.3fkV  ", (measuredHV_V / 1000.0));
            lcd.setCursor(0,1)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA    ", measuredI_mA);
            lcd.setCursor(0,2)
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig:  %.1fmA %.1fkV ", icomp_mA, (vcomp_V / 1000.0));
            lcd.setCursor(0,3)
            lcd.print(buffer);
    }
}

void setup()
{
    Serial.begin(9600);
    Serial.println("High Voltage power supply Monitoring with ADS1115");

    // Initialize the ADS1115 and I2C bus
    Wire.begin();
    ads.begin();
    ads.setDataRate(RATE_ADS1115_860SPS);

    // Initialize the LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);

    // Init pin for checking
    pinMode(SET_BERTAN_3KV, INPUT_PULLUP);
    pinMode(SET_BERTAN_20KV, INPUT_PULLUP);
    pinMode(SET_MATSUSADA_1KV, INPUT_PULLUP);
    delay(50); // Make sure pullup stable

    if (!digitalRead(SET_BERTAN_3KV)) {
        voltage_multiplier = 3000.0;
        current_multiplier = 10.0;
    } else if (!digitalRead(SET_BERTAN_20KV)) {
        voltage_multiplier = 20000.0;
        current_multiplier = 1.0;
    } else if (!digitalRead(SET_MATSUSADA_1KV)) {
        voltage_multiplier = 1000.0;
        current_multiplier = 30.0;
    } // TODO -1kV????

    timer.every(150, read_value);
    timer.every(200, display_value);
    timer.every(1000*60*30, clear_display); // every 30 minutes


}