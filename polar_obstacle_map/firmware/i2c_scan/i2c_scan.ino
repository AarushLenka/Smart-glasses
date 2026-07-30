// I2C bus scanner - diagnostic only. Flash this when a sensor "not found".
// Expect: 0x29 (VL53L0X), 0x68 or 0x69 (MPU6050, 0x69 if AD0 pulled high).
// Nothing listed = wiring/power problem, not an address problem.

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.print("SDA=GPIO"); Serial.print(SDA_PIN);
  Serial.print("  SCL=GPIO"); Serial.println(SCL_PIN);
  Wire.begin(SDA_PIN, SCL_PIN, 100000);   // slow clock: tolerates long breadboard wires
}

void loop() {
  int found = 0;
  Serial.println("scanning 0x08..0x77");
  for (uint8_t addr = 8; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  NOTHING on the bus - check 3V3, GND, SDA/SCL, pull-ups");
  Serial.println();
  delay(3000);
}
