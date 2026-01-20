/* Central Application Logic: (from HW dev spec)
    - read external ADC
    - read internal ADC
    - apply scaling
    - update LCD
    - tx over RS-485
    - detect fault conditions
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

// Create the LCD object at address 0x27 or 0x3F, depending on your module
// (Check your LCD’s documentation for its I²C address.)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Global ID tracking which arduino this is:
//   0: -1kV
//   1: +1kV
//   2: +3kV
//   3: +20kV
int ps_id;

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
bool read_value()
{
    
    float imon_raw = ads.readADC_SigleEnded(CH_IMON);
    float vmon_raw = ads.readADC_SigleEnded(CH_VMON);
    float vset_raw = ads.readADC_SigleEnded(CH_VSET);

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

            return true;
        
        case 1: // 1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:  +%4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V: +%4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", meas_i_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %4dV  ", comp_i_buf, vcomp_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            return true;

        case 2: // +3kV Bertan
            snprintf(buffer, 21 * sizeof(char), "Set V:  +%4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V: +%4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %smA   ", meas_i_buf);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %smA %4dV  ", comp_i_buf, vcomp_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            return true;

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

            snprintf(buffer, 21 * sizeof(char), "Trig:  %smA %skV ", comp_i_buf, comp_v_buf);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            return true;
    }
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

    // Configure power supply specs 
    switch (ps_id) {
        case 0: // -1kV Matsusada
            hv_rated_V = 1000.0;
            i_rated_mA = 30.0;
            hv_polarity = -1.0;
            fullscale = 10.0;
        case 1: // +1kV Matsusada
            hv_rated_V = 1000.0;
            i_rated_mA = 30.0;
            hv_polarity = 1.0;
            fullscale = 10.0;
        case 2: // +3kV Bertan
            hv_rated_V = 3000.0;
            i_rated_mA = 10.0;
            hv_polarity = 1.0;
            fullscale = 5.0;
        case 3: // +20kV Bertan
            hv_rated_V = 20000.0;
            i_rated_mA = 1.0;
            hv_polarity = 1.0;
            fullscale = 5.0;
    }

    timer.every(150, read_value);
    timer.every(200, display_value);
    timer.every(1000*60*30, clear_display); // every 30 minutes


}