#include <ArduinoModbus.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println("Starting Master Device...");
  if (!ModbusRTUClient.begin(Serial1, 9600)) {
    Serial.println("Failed to start Modbus RTU Client");
    while (1);
  }
  Serial.println("Master Initialized Successfully");
}

void loop() {

  if (!ModbusRTUClient.requestFrom(1, INPUT_REGISTERS, 1, 1)) {
    Serial.println("Read failed");
  } else {
    while (ModbusRTUClient.available()) {
      Serial.print("Read successful: I_REGISTER[1] = ");
      Serial.println(ModbusRTUClient.read());
    }
  }

  delay(1000);
}
