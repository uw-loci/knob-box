#include <arduino-timer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
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

/* Values read by External ADC. */
float measuredI_mA;
float measuredHV_V;
float programmedHV_V;

void setup() {
    Serial.begin(9600);
    Serial.println("High Voltage Power Supply Monitoring with ADS1115");
    Wire.begin();

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
}

void loop() {

    imon_raw = random(0, 26001);
    vmon_raw = random(0, 26001);
    vset_raw = random(0, 26001);

    read_value();
    display_value();

    delay(2000);

}

bool display_value()
{
    switch(ps_id) {
        case 0: // -1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   %4dV     ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  %4dV     ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA   ", measuredI_mA);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %.1fmA %4dV  ", icomp_mA, vcomp_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            return;
        
        case 1: // 1kV Matsusada
            snprintf(buffer, 21 * sizeof(char), "Set V:   %4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  %4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA   ", measuredI_mA);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %.1fmA %4dV  ", icomp_mA, vcomp_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            return;

        case 2: // +3kV Bertan
            snprintf(buffer, 21 * sizeof(char), "Set V:   %4dV      ", int(programmedHV_V));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  %4dV      ", int(measuredHV_V));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA   ", measuredI_mA);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig: %.1fmA %4dV  ", icomp_mA, vcomp_V);
            lcd.setCursor(0,3);
            lcd.print(buffer);

            return;

        case 3: // +20kV Bertan
            snprintf(buffer, 21 * sizeof(char), "Set V:   +%.3fkV  ", (programmedHV_V / 1000.0));
            lcd.setCursor(0,0);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Meas V:  +%.3fkV  ", (measuredHV_V / 1000.0));
            lcd.setCursor(0,1);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Current: %.3fmA    ", measuredI_mA);
            lcd.setCursor(0,2);
            lcd.print(buffer);

            snprintf(buffer, 21 * sizeof(char), "Trig:  %.1fmA %.1fkV ", icomp_mA, (vcomp_V / 1000.0));
            lcd.setCursor(0,3);
            lcd.print(buffer);
    }
    return true; // repeat? yes
}

bool read_value()
{
    
    // float imon_raw = ads.readADC_SigleEnded(CH_IMON);
    // float vmon_raw = ads.readADC_SigleEnded(CH_VMON);
    // float vset_raw = ads.readADC_SigleEnded(CH_VSET);

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