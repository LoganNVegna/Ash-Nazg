// Minimal I2C H3LIS331 Accelerometer Test
// For Serial Plotter - ESP32-S3 with SDA=5, SCL=6

#include <Wire.h>

// H3LIS331 I2C address (0x18 for Adafruit, 0x19 for SparkFun)
#define LIS331_ADDR 0x18

// Register addresses
#define WHO_AM_I   0x0F
#define CTRL_REG1  0x20
#define CTRL_REG4  0x23
#define OUT_X_L    0x28
#define OUT_X_H    0x29
#define OUT_Y_L    0x2A
#define OUT_Y_H    0x2B
#define OUT_Z_L    0x2C
#define OUT_Z_H    0x2D

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS331_ADDR, 1);
  return Wire.read();
}

int16_t readAxis(uint8_t lowReg) {
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(lowReg | 0x80);  // Auto-increment bit
  Wire.endTransmission(false);
  Wire.requestFrom(LIS331_ADDR, 2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize I2C with YOUR board's pins
  Wire.begin(5, 6);  // SDA=GPIO5, SCL=GPIO6
  Wire.setClock(400000);  // 400kHz fast mode
  
  Serial.println("H3LIS331 Accelerometer Test");
  
  // Check WHO_AM_I register (should be 0x32 for H3LIS331)
  uint8_t whoami = readReg(WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  
  if (whoami == 0x32) {
    Serial.println("H3LIS331 detected!");
  } else if (whoami == 0xFF || whoami == 0x00) {
    Serial.println("ERROR: No sensor found! Check wiring.");
    Serial.println("SDA -> GPIO 5");
    Serial.println("SCL -> GPIO 6");
    Serial.println("VCC -> 3.3V");
    Serial.println("GND -> GND");
    while(1) delay(1000);  // Stop here
  } else {
    Serial.print("Unknown device ID: 0x");
    Serial.println(whoami, HEX);
  }
  
  // Configure sensor
  // CTRL_REG1: Power on, 1000Hz data rate, all axes enabled
  // Bits: PM2 PM1 PM0 DR1 DR0 Zen Yen Xen
  // 0x3F = 0011 1111 = Normal mode, 1000Hz, XYZ enabled
  writeReg(CTRL_REG1, 0x3F);
  
  // CTRL_REG4: 400g full scale, block data update
  // Bits: BDU BLE FS1 FS0 STsign 0 ST SIM
  // 0x30 = 0011 0000 = BDU enabled, 400g range
  writeReg(CTRL_REG4, 0x30);
  
  delay(100);
  Serial.println("Sensor configured for 400g range!");
  Serial.println("Opening Serial Plotter will show X, Y, Z acceleration.");
  Serial.println("---");
  delay(2000);
}

void loop() {
  // Read all three axes
  int16_t x_raw = readAxis(OUT_X_L);
  int16_t y_raw = readAxis(OUT_Y_L);
  int16_t z_raw = readAxis(OUT_Z_L);
  
  // Convert to g-force (400g full scale, 16-bit signed)
  // For H3LIS331 in 400g mode: 1 LSB = 400g / 32768 = ~0.0122g
  float x_g = (x_raw / 32768.0) * 400.0;
  float y_g = (y_raw / 32768.0) * 400.0;
  float z_g = (z_raw / 32768.0) * 400.0;
  
  // Output for Serial Plotter (comma-separated values)
  Serial.print(x_g);
  Serial.print(",");
  Serial.print(y_g);
  Serial.print(",");
  Serial.println(z_g);
  
  delay(10);  // ~100Hz update rate
}
