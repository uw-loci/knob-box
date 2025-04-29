#include <arduino-timer.h>
#include <Wire.h>
// Adafruit’s unified ADS1X15 library handles both ADS1015 and ADS1115
#include <Adafruit_ADS1X15.h>

// For a typical 2x16 or 4x20 I2C-character LCD. 
// Adjust the include and constructor depending on which I²C LCD library you use.
#include <LiquidCrystal_I2C.h>

//Watch dog timer
#include <avr/wdt.h>

//Modbus
#include <ModbusSerial.h>
//Modbus ID
#define DEVICE_ID   2
// Create the ADS1115 object
Adafruit_ADS1115 ads;  

// Create the LCD object at address 0x27 or 0x3F, depending on your module
// (Check your LCD’s documentation for its I²C address.)
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Timer for read ADC Value
Timer<3, millis> timer;

//Modbus enum
enum {
  COMMAND,
  MODE,
  V_SET,
  V_READ,
  I_READ,
  OVERCURRENT_TH,
  OVERCURRENT
} MODBUS_REG;

ModbusSerial ModbusObj (Serial, DEVICE_ID, -1);

int cmd_reg = 0;
int mode_reg = 0;
int overcurrent_threshold = 0;

// Channel assignment:
//   A0 -> Voltage Monitor from Bertan 
//   A1 -> Pot (voltage program setting)
//   A2 -> Current Monitor from Bertan
#define CH_VMON      0
#define CH_POT       1
#define CH_IMON      2

//Power supply mode pin define
#define SET_BERTAN_3KV 48
#define SET_BERTAN_20KV 50
#define SET_MATSUSADA_1KV 52

float voltage_multiplier = 0;
float current_multiplier = 0;

// For convenience, pick an ADS1115 data rate that easily exceeds 2.5 samples/s.
// For instance, the default (128 SPS) is plenty fast for a 400 ms loop.

//Timer callback
int16_t potRaw = 0;
int16_t vmonRaw = 0;
int16_t imonRaw = 0;

float potVolts;
float vmonVolts;
float imonVolts;

float hvProgram_V;
float measuredHV_V;
float measuredI_mA;

float voltsPerCount = 0.1875F / 1000.0F;

bool clear_display(void *){
  lcd.clear();
  return true;
}

bool read_value(void *) { //Callback to read ADC
  // Read the four ADS1115 channels in single‐ended mode:
  potRaw       = ads.readADC_SingleEnded(CH_POT);
  vmonRaw      = ads.readADC_SingleEnded(CH_VMON);
  imonRaw      = ads.readADC_SingleEnded(CH_IMON);

  // Convert from raw ADC reading to voltage at each ADS input pin.
  // By default, the Adafruit library uses ±6.144 V as the full‐scale range,
  // which yields ~0.1875 mV/LSB. If you used setGain(GAIN_ONE) or others,
  // your conversion factor changes. For the default ±6.144 V:

  potVolts      = potRaw      * voltsPerCount; 
  vmonVolts     = vmonRaw     * voltsPerCount;
  imonVolts     = imonRaw     * voltsPerCount;

  // The pot may be setting 0–5 V for 0–3 kV (on the 205B-03R). For instance:
  //   HV_kV = (potVolts / 5.0) * 3.0;
  // The Bertan’s V-MON pin is typically 0–5 V for 0–full-scale HV. So for a 3 kV unit:
  //   measuredHV_kV = (vmonVolts / 5.0) * 3.0;
  // Similarly, for I-MON (0–5 V) you’d scale it to 0–the supply’s max current (10 mA):
  //   measuredI_mA = (imonVolts / 5.0) * 10.0;
  // The polarity pin is open-collector; if you’ve got a pull-up to 5 V, reading near 0 means
  // positive polarity, near 5 means negative. You can threshold it to figure out text labels.

  hvProgram_V = (potVolts / 5.0) * voltage_multiplier;     //programmed voltage set * voltage_multiplier
  measuredHV_V = (vmonVolts / 5.0) * voltage_multiplier;   // reading voltage *voltage_multiplier
  measuredI_mA  = (imonVolts / 5.0) * current_multiplier;  // reading current * voltage_multiplier

  #ifndef DEBUG
  ModbusObj.setIreg(I_READ, measuredI_mA);
  ModbusObj.setIreg(V_SET, hvProgram_V);
  ModbusObj.setIreg(V_READ, measuredHV_V);
  if((imonVolts* current_multiplier) > measuredI_mA)
    ModbusObj.setIsts(OVERCURRENT,1);
  #endif

  return true; // repeat? true
}

char buffer[16];
char vbuffer[16];
char current[8];

bool display_value(void *) { //Callback to Display value
  #ifdef DEBUG
  // Print results to Serial
  Serial.print("Set: ");
  Serial.print(hvProgram_V, 0);
  Serial.print(" V,  HV: ");
  Serial.print(measuredHV_V, 0);
  Serial.print(" V,  I: ");
  Serial.print(measuredI_mA, 2);
  Serial.println(" mA");
  #endif

  // Display two readings on each of the two LCD rows
  // Row 1: HV setpoint (kV) and measured HV
  snprintf(buffer, 16 * sizeof(char), "Vs:%5dV ", int(hvProgram_V));

  lcd.setCursor(0, 0);
  lcd.print(buffer);
  lcd.setCursor(10, 0);
  lcd.print("I:");

  snprintf(vbuffer, 16 * sizeof(char), "V:%5dV ", int(measuredHV_V));
  // Row 2: measured current (mA) and polarity
  lcd.setCursor(0, 1);
  lcd.print(vbuffer);
  lcd.setCursor(10, 1);
  lcd.print(measuredI_mA,1);
  lcd.print("mA ");
  
  return true; // repeat? true
}

void ModBuspacketHandler(){
  overcurrent_threshold = ModbusObj.Hreg(OVERCURRENT_TH);
  cmd_reg = ModbusObj.Hreg(COMMAND);
}

void setup()
{
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.println("High voltage power supply Monitoring with ADS1115 Test");
  #else
  Serial.begin(9600,SERIAL_8E1);
  ModbusObj.config(9600);
  #endif
  // Initialize the I²C bus, ADS1115, and set sample rate/gain if desired
  Wire.begin();   
  ads.begin();
  ads.setDataRate(RATE_ADS1115_860SPS); //Set data rate
  // Optionally pick a data rate:
  //   ads.setDataRate(RATE_ADS1115_128SPS); // default
  // or  ads.setDataRate(RATE_ADS1115_250SPS);
  //
  // Optionally pick a gain (depends on your 0–5 V signals):
  //   ads.setGain(GAIN_ONE);    // ±4.096 V range (good if your signals never exceed ~4 V)
  //   ads.setGain(GAIN_TWO);    // ±2.048 V
  //   ads.setGain(GAIN_FOUR);   // ±1.024 V
  // etc. The default is GAIN_TWOTHIRDS (±6.144 V), fine for 0–5 V.

  //Init pin for checking
  pinMode(SET_BERTAN_3KV, INPUT_PULLUP);
  pinMode(SET_BERTAN_20KV, INPUT_PULLUP);
  pinMode(SET_MATSUSADA_1KV, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(50); // Make sure pullup stable

  #ifndef DEBUG
  ModbusObj.addHreg(COMMAND, cmd_reg);
  ModbusObj.addIreg(MODE, mode_reg);
  ModbusObj.addIreg(V_SET, hvProgram_V);
  ModbusObj.addIreg(V_READ, measuredHV_V);
  ModbusObj.addIreg(I_READ, measuredI_mA);
  ModbusObj.addHreg(OVERCURRENT_TH, overcurrent_threshold);
  ModbusObj.addIsts(OVERCURRENT);
  #endif
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  if(!digitalRead(SET_BERTAN_3KV)){         //Pin 48 pull down
    lcd.print("Bertan");
    lcd.setCursor(0, 1);
    lcd.print("3kV Setup");
    voltage_multiplier = 3000.0;
    current_multiplier = 10.0;
    ModbusObj.setIreg(MODE, 0);
  }else if(!digitalRead(SET_BERTAN_20KV)){  //Pin 50 pull down
    lcd.print("Bertan");
    lcd.setCursor(0, 1);
    lcd.print("20kV Setup");
    voltage_multiplier = 20000.0;
    current_multiplier = 1.0;
    ModbusObj.setIreg(MODE, 1);
  }else if(!digitalRead(SET_MATSUSADA_1KV)){//Pin 52 pull down
    lcd.print("Matsusada");
    lcd.setCursor(0, 1);
    lcd.print("1kV Setup");
    voltage_multiplier = 1000.0;
    current_multiplier = 30.0;
    ModbusObj.setIreg(MODE, 2);
  }else{                                    //Nothing connencted
    lcd.print("Error");
    lcd.setCursor(0,1);
    lcd.print("No mode set!");
    ModbusObj.setIreg(MODE, 255);
    while (true){//Halt everything, indicate error through LED_BUILTIN
      #ifndef DEBUG
      ModbusObj.task();
      #endif
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(750);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(750);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    } 
  }
  delay(1000);  //Wait a little bit for everything ready
  lcd.clear();

  timer.every(100, read_value); //Setup timer callback
  timer.every(200, display_value); //Setup timer callback
  timer.every(1000 * 60 * 30, clear_display); //Clear display every 30 minutes

  wdt_enable(WDTO_8S); // Enable watchdog with 8s timeout
}

void loop()
{
  wdt_reset(); //Feed dog
  #ifndef DEBUG
  ModbusObj.task();
  ModBuspacketHandler();
  #endif
  timer.tick();
}
