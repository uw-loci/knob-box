#include <Wire.h>
// Adafruit’s unified ADS1X15 library handles both ADS1015 and ADS1115
#include <Adafruit_ADS1X15.h>

// For a typical 2x16 or 4x20 I2C-character LCD. 
// Adjust the include and constructor depending on which I²C LCD library you use.
#include <LiquidCrystal_I2C.h>

// Create the ADS1115 object
Adafruit_ADS1115 ads;  

// Create the LCD object at address 0x27 or 0x3F, depending on your module
// (Check your LCD’s documentation for its I²C address.)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Channel assignment:
//   A0 -> Voltage Monitor from Bertan 
//   A1 -> Pot (voltage program setting)
//   A2 -> Current Monitor from Bertan
#define CH_VMON      0
#define CH_POT       1
#define CH_IMON      2

// For convenience, pick an ADS1115 data rate that easily exceeds 2.5 samples/s.
// For instance, the default (128 SPS) is plenty fast for a 400 ms loop.
void setup()
{
  Serial.begin(9600);
  Serial.println("Bertan 205B-03R Monitoring with ADS1115 Test");

  // Initialize the I²C bus, ADS1115, and set sample rate/gain if desired
  Wire.begin();   
  ads.begin();
  // Optionally pick a data rate:
  //   ads.setDataRate(RATE_ADS1115_128SPS); // default
  // or  ads.setDataRate(RATE_ADS1115_250SPS);
  //
  // Optionally pick a gain (depends on your 0–5 V signals):
  //   ads.setGain(GAIN_ONE);    // ±4.096 V range (good if your signals never exceed ~4 V)
  //   ads.setGain(GAIN_TWO);    // ±2.048 V
  //   ads.setGain(GAIN_FOUR);   // ±1.024 V
  // etc. The default is GAIN_TWOTHIRDS (±6.144 V), fine for 0–5 V.
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bertan Setup");
  delay(1000);
  lcd.clear();
}

void loop()
{
  // Read the four ADS1115 channels in single‐ended mode:
  int16_t potRaw       = ads.readADC_SingleEnded(CH_POT);
  int16_t vmonRaw      = ads.readADC_SingleEnded(CH_VMON);
  int16_t imonRaw      = ads.readADC_SingleEnded(CH_IMON);

  // Convert from raw ADC reading to voltage at each ADS input pin.
  // By default, the Adafruit library uses ±6.144 V as the full‐scale range,
  // which yields ~0.1875 mV/LSB. If you used setGain(GAIN_ONE) or others,
  // your conversion factor changes. For the default ±6.144 V:
  float voltsPerCount = 0.1875F / 1000.0F; // = 0.0001875 V per LSB

  float potVolts      = potRaw      * voltsPerCount; 
  float vmonVolts     = vmonRaw     * voltsPerCount;
  float imonVolts     = imonRaw     * voltsPerCount;

  // The pot may be setting 0–5 V for 0–3 kV (on the 205B-03R). For instance:
  //   HV_kV = (potVolts / 5.0) * 3.0;
  // The Bertan’s V-MON pin is typically 0–5 V for 0–full-scale HV. So for a 3 kV unit:
  //   measuredHV_kV = (vmonVolts / 5.0) * 3.0;
  // Similarly, for I-MON (0–5 V) you’d scale it to 0–the supply’s max current (10 mA):
  //   measuredI_mA = (imonVolts / 5.0) * 10.0;
  // The polarity pin is open-collector; if you’ve got a pull-up to 5 V, reading near 0 means
  // positive polarity, near 5 means negative. You can threshold it to figure out text labels.

  float hvProgram_V = (potVolts / 5.0) * 3000;     // example for 3 kV supply
  float measuredHV_V = (vmonVolts / 5.0) * 3000;   // for 3 kV supply
  float measuredI_mA  = (imonVolts / 5.0) * 10.0;  // for 10 mA max rating

  // Print results to Serial
  Serial.print("Set: ");
  Serial.print(hvProgram_V, 0);
  Serial.print(" V,  HV: ");
  Serial.print(measuredHV_V, 0);
  Serial.print(" V,  I: ");
  Serial.print(measuredI_mA, 2);
  Serial.print(" mA");

  // Display two readings on each of the two LCD rows
  // Row 1: HV setpoint (kV) and measured HV
  lcd.setCursor(0, 0);
  lcd.print("Set: ");
  lcd.print(hvProgram_V, 0);
  lcd.print(" V");

  // Row 2: measured current (mA) and polarity
  lcd.setCursor(0, 1);
  lcd.print("V:");
  lcd.print(measuredHV_V, 0);
  lcd.print("V ");
  lcd.print("I:");
  lcd.print(measuredI_mA, 2);
  lcd.print("mA");

  // Wait ~400 ms between reads
  delay(250);
}
