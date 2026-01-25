// Progressive Debug Test for Melty Brain
// Open this sketch in Arduino IDE to test step by step

// ============ STEP 1: Basic includes (uncomment to test) ============
/*
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <Arduino.h>
#include <EEPROM.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 1: Basic Includes ===");
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 2: Add CRSF include ============
/*
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <CRSFforArduino.hpp>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <Arduino.h>
#include <EEPROM.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 2: With CRSF Include ===");
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 3: Add DShotESC include ============
/*
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <CRSFforArduino.hpp>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "DShotESC.h"  // Local file - copy from Main_V5.0 folder

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 3: With DShot Include ===");
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 4: Test NeoPixel object ============
/*
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip(10, A3, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 4: NeoPixel Global Object ===");
  strip.begin();
  strip.clear();
  strip.show();
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 5: Test CRSF object ============
/*
#include <CRSFforArduino.hpp>

CRSFforArduino crsf = CRSFforArduino(&Serial1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 5: CRSF Global Object ===");
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 6: Test DShot ESC object ============
// NOTE: Need to copy DShotESC.h and DShotESC.cpp to this folder first!
/*
#include "DShotESC.h"

DShotESC esc;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 6: DShot ESC Global Object ===");
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 7: Test Accelerometer object (CRASHES) ============
/*
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

Adafruit_H3LIS331 xl = Adafruit_H3LIS331();  // THIS CRASHES!

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 7: Accelerometer Global Object ===");
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 8: Test Accelerometer as POINTER (should work) ============
/*
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

Adafruit_H3LIS331 *xl = nullptr;  // Pointer, not object!

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 8: Accelerometer as Pointer ===");
  
  Wire.begin();
  xl = new Adafruit_H3LIS331();  // Created in setup()
  
  if(xl->begin_I2C(0x18)) {
    Serial.println("Accelerometer found!");
  } else {
    Serial.println("Accelerometer NOT found (but didn't crash!)");
  }
  Serial.println("PASSED!");
}

void loop() { Serial.print("."); delay(1000); }
*/

// ============ STEP 9: Test SparkFun LIS331 (Alternative Library) ============
/*
#include <Wire.h>
#include <SparkFun_LIS331.h>

LIS331 xl;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 9: SparkFun LIS331 Library ===");
  
  Wire.begin();
  Wire.setClock(400000);
  xl.setI2CAddr(0x18);  // 0x18 for Adafruit, 0x19 for SparkFun breakout
  xl.begin(LIS331::USE_I2C);
  xl.setFullScale(LIS331::HIGH_RANGE);  // 400g mode
  
  Serial.println("SparkFun LIS331 initialized!");
  Serial.println("PASSED!");
}

void loop() { 
  int16_t x, y, z;
  xl.readAxes(x, y, z);
  Serial.print("Y: ");
  Serial.println(y);
  delay(100);
}
*/

// ============ STEP 10: Minimal I2C LIS331 (No Library) ============
/*
#include <Wire.h>

#define LIS331_ADDR 0x18
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t readY() {
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(OUT_Y_L | 0x80); // Auto-increment
  Wire.endTransmission(false);
  Wire.requestFrom(LIS331_ADDR, 2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== STEP 10: Minimal I2C LIS331 ===");
  
  Wire.begin();
  Wire.setClock(400000);
  
  // Power on, 1000Hz data rate, all axes enabled
  writeReg(CTRL_REG1, 0x3F);
  // 400g full scale, block data update
  writeReg(CTRL_REG4, 0x30);
  
  delay(100);
  Serial.println("Minimal LIS331 initialized!");
  Serial.println("PASSED!");
}

void loop() {
  int16_t y = readY();
  float g = (y / 32768.0) * 400.0; // Convert to g-force
  Serial.print("Y (g): ");
  Serial.println(g);
  delay(100);
}
*/

// ============ ACTIVE TEST: START HERE ============
// Uncomment one step at a time and upload!

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== DEBUG TEST ===");
  Serial.println("Uncomment one STEP section above and upload!");
  Serial.println("Find which step causes the crash.");
}

void loop() {
  Serial.print(".");
  delay(1000);
}
