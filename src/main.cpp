#include <Arduino.h>
#include <Wire.h>

#define SDA1 21
#define SCL1 22

 #define CST_DEVICE_ADDR 0x38

TwoWire I2Cone = TwoWire(0);

uint8_t ft6336(uint8_t reg) {
  I2Cone.beginTransmission((uint8_t)CST_DEVICE_ADDR);
  I2Cone.write(reg);
  I2Cone.endTransmission();
  I2Cone.requestFrom((uint8_t)CST_DEVICE_ADDR, uint8_t(1));
  return I2Cone.read();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  // fast I2C spped
  // I2Cone.begin(SDA1,SCL1,400000); // SDA pin 21, SCL pin 22 TTGO TQ
  // slow I2C spped
  I2Cone.begin(SDA1,SCL1,100000); // SDA pin 21, SCL pin 22 TTGO TQ
  Serial.println("setup done");
}

void loop() { 
  Serial.println("scan");
  Serial.println(ft6336(0xA3));
  Serial.println();
  delay(5000);
}