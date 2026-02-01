/**
 * Ash-Nazg Melty Brain Robot - V5 Testing
 * 
 * PlatformIO port of the V5_Testing Arduino sketch
 * For Adafruit QT Py ESP32 Pico with PSRAM
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Adafruit_DotStar.h>
#include <string.h>  // For strncpy, strcmp
#include <CRSFforArduino.hpp>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <EEPROM.h>
#include "DShotESC.h"
#include <esp_timer.h>

//------ Task CPU-time profiling (per-iteration, excludes vTaskDelay) ------
enum { PROF_LOOP = 0, PROF_I2C, PROF_CRSF, PROF_LED, PROF_ESC, PROF_N };
struct task_prof_s {
  const char *name;
  volatile uint32_t last_us;
  volatile uint32_t max_us;
  volatile uint64_t sum_us;
  volatile uint32_t count;
};
static struct task_prof_s task_profs[PROF_N] = {
  { "Loop", 0, 0, 0, 0 },
  { "Accel", 0, 0, 0, 0 },
  { "CRSF", 0, 0, 0, 0 },
  { "LED",  0, 0, 0, 0 },
  { "ESC",  0, 0, 0, 0 },
};
static void prof_record(int id, uint32_t dt_us) {
  if (id < 0 || id >= PROF_N) return;
  struct task_prof_s *p = &task_profs[id];
  p->last_us = dt_us;
  if (dt_us > p->max_us) p->max_us = dt_us;
  p->sum_us += dt_us;
  p->count++;
}
#define PROF_REPORT_INTERVAL_US  500000   // 500 ms = 2 Hz
static unsigned long long last_prof_report_us = 0;
static void prof_report(void) {
  unsigned long long now = (unsigned long long)esp_timer_get_time();
  if ((now - last_prof_report_us) < PROF_REPORT_INTERVAL_US) return;
  last_prof_report_us = now;
  Serial.println();
  Serial.println("=== Task CPU-time profile (us, last 500ms window) ===");
  for (int i = 0; i < PROF_N; i++) {
    struct task_prof_s *p = &task_profs[i];
    uint32_t la = p->last_us, mx = p->max_us, n = p->count;
    uint64_t s = p->sum_us;
    unsigned long avg = (n > 0) ? (unsigned long)(s / n) : 0;
    Serial.print("  ");
    Serial.print(p->name);
    Serial.print(": last=");
    Serial.print(la);
    Serial.print(" max=");
    Serial.print(mx);
    Serial.print(" avg=");
    Serial.print(avg);
    Serial.print(" count=");
    Serial.println(n);
    // Reset for next window
    p->max_us = 0;
    p->sum_us = 0;
    p->count = 0;
  }
  Serial.println("=====================================================");
}

//------accelerometer config (SPI - faster than I2C)------------
#define ACCEL_MAX_SCALE 400  // 400g range
const int accradius = 18; //radius where the g force sensor is located in millimeters

// LIS331 SPI pins (keep GPIO 7,8 from I2C; add 9,10 for MISO, CS)
#define ACCEL_SCK_PIN  7   // was I2C SCL - SPI clock
#define ACCEL_MOSI_PIN 8   // was I2C SDA - SPI MOSI (data to accel)
#define ACCEL_MISO_PIN 9   // NEW - SPI MISO (data from accel)
#define ACCEL_CS_PIN   10  // NEW - Chip select
#define ACCEL_SPI_SPEED_HZ  1000000  // 1 MHz

// LIS331 register addresses
#define CTRL_REG1    0x20
#define CTRL_REG4    0x23
#define STATUS_REG   0x27
#define OUT_X_L      0x28
#define OUT_X_H      0x29
#define OUT_Y_L      0x2A
#define OUT_Y_H      0x2B
#define OUT_Z_L      0x2C
#define OUT_Z_H      0x2D

// Non-blocking SPI accel using FreeRTOS task
static TaskHandle_t accel_task_handle = NULL;
static float latest_gforce = 0.0;
static bool accel_initialized = false;
static unsigned long last_accel_update_time = 0;

// LIS331 SPI: write one register
static void accel_spi_write(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(ACCEL_SPI_SPEED_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(ACCEL_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);  // write: bit7=0
  SPI.transfer(val);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  SPI.endTransaction();
}

// LIS331 SPI: read 6 bytes from OUT_X_L with auto-increment
// Per datasheet: chip drives SDO at start of bit 8, so first byte received is dummy
static void accel_spi_read_xyz(int16_t *buf) {
  SPI.beginTransaction(SPISettings(ACCEL_SPI_SPEED_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(ACCEL_CS_PIN, LOW);
  (void)SPI.transfer(0xC0 | (OUT_X_L & 0x3F));  // Send read cmd; first byte received is dummy
  uint8_t xl = SPI.transfer(0xFF);
  uint8_t xh = SPI.transfer(0xFF);
  uint8_t yl = SPI.transfer(0xFF);
  uint8_t yh = SPI.transfer(0xFF);
  uint8_t zl = SPI.transfer(0xFF);
  uint8_t zh = SPI.transfer(0xFF);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  SPI.endTransaction();
  buf[0] = (int16_t)(xl | (xh << 8));
  buf[1] = (int16_t)(yl | (yh << 8));
  buf[2] = (int16_t)(zl | (zh << 8));
}

// FreeRTOS task to read accelerometer via SPI in background
void accel_spi_read_task(void *pvParameters) {
  int16_t xyz[3];
  Serial.println("  [Accel SPI Task] Started");
  
  while (1) {
    int64_t t0 = esp_timer_get_time();
    // Read accelerometer every 20ms - read each register individually (like SparkFun library)
    // This is more reliable than auto-increment on some boards
    accel_spi_read_xyz(xyz);
    int16_t y = xyz[1] >> 4;  // 12-bit left-justified, Y axis
    float gforce = (float)ACCEL_MAX_SCALE * (float)y / 2047.0f;
    
    // Store result (atomic write - float is 32-bit, should be safe on ESP32)
    latest_gforce = gforce;
    last_accel_update_time = esp_timer_get_time();
    
    int64_t t1 = esp_timer_get_time();
    prof_record(PROF_I2C, (uint32_t)(t1 - t0));
    vTaskDelay(pdMS_TO_TICKS(20));  // Read every 20ms
  }
}

//-------ESC config (ESP32-S2 Mini)--------
// DShotESC: install(gpio, rmtChannel, frequency, divider), sendThrottle3D(-999..999)
#define escR_gpio GPIO_NUM_3  // right ESC pin (safe)
#define escL_gpio GPIO_NUM_4  // left ESC pin (safe)
DShotESC escR;  // right ESC object
DShotESC escL;  // left ESC object

//---------Pin assignments (ESP32-S2 Mini)----------
// Available GPIOs: 1-18, 21, 33-40
// DotStar uses SPI: MOSI (data) and SCK (clock) pins
// ESP32-S2 Mini safe pins: 1-8, 17, 18, 21
//
// PIN ASSIGNMENT SUMMARY (verify no conflicts):
// GPIO 1: volt_pin (battery voltage ADC)
// GPIO 2: bottom_led_sck (bottom LED strip clock)
// GPIO 3: escR_gpio (right ESC)
// GPIO 4: escL_gpio (left ESC)
// GPIO 5: CRSF_RX_PIN (receiver RX)
// GPIO 6: CRSF_TX_PIN (receiver TX)
// GPIO 7: ACCEL_SCK (SPI clock - accelerometer)
// GPIO 8: ACCEL_MOSI (SPI data to accel)
// GPIO 9: ACCEL_MISO (SPI data from accel)
// GPIO 10: ACCEL_CS (accel chip select)
// GPIO 17: top_led_sck (top LED strip clock)
// GPIO 21: top_led_mosi (top LED strip data, DI)
// GPIO 18: bottom_led_mosi (bottom LED strip data, DI)
// GPIO 2:  bottom_led_sck (bottom LED strip clock)
//
const int top_led_mosi = 21;     // Top strip DI (data) - GPIO 21
const int top_led_sck = 17;      // Top strip CI (clock) - GPIO 17
const int bottom_led_mosi = 18;  // Bottom strip DI (data) - GPIO 18
const int bottom_led_sck = 2;    // Bottom strip CI (clock) - GPIO 2
const int volt_pin = 1;         //pin for measure battery voltage (GPIO 1 - ADC capable)

//------LED definitions--------
const int NUMPIXELST = 10; //number of leds in the top strip
const int NUMPIXELSB = 8; //number of leds in the bottom strip
// DotStar strips using software SPI (faster than NeoPixel, uses SPI protocol)
Adafruit_DotStar top_strip(NUMPIXELST, top_led_mosi, top_led_sck, DOTSTAR_BGR);
Adafruit_DotStar bottom_strip(NUMPIXELSB, bottom_led_mosi, bottom_led_sck, DOTSTAR_BGR);
const int animSpeed = 100;   // ms between LED updates when in animation mode

//------Reciever config (CRSF)-----------
// ESP32-S2 Mini safe pins
#define CRSF_RX_PIN 5   // ESP32 RX <- Receiver TX (GPIO 5 - safe)
#define CRSF_TX_PIN 6   // ESP32 TX -> Receiver RX (GPIO 6 - safe)
#define CRSF_POLL_HZ 200
#define CRSF_POLL_INTERVAL_MS  (1000 / CRSF_POLL_HZ)  // 5 ms
CRSFforArduino *crsf = nullptr;
const int NUM_CHANNELS = 8; //number of reciever channels to use

// CRSF channel data (callback fills; main loop reads via update_channels)
static volatile uint16_t crsf_channels[NUM_CHANNELS + 1] = {0};  // Shared channel data (indexed 1-NUM_CHANNELS)
static volatile bool crsf_failsafe = true;  // Failsafe status from CRSF library
static volatile unsigned long last_crsf_update = 0;  // Timestamp of last CRSF update (microseconds)

// CRSF callback function prototype
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

// Non-blocking LED using FreeRTOS task
static volatile int led_angle_shared = 0;  // Current angle (0-359)
static volatile int led_rpm_shared = 0;   // Current RPM
static char led_status_shared[32] = "armed";  // LED status string (char array for thread safety)
static volatile bool led_data_changed = false;  // Flag when status changes
static TaskHandle_t led_task_handle = NULL;
static bool led_task_initialized = false;

//------Driving characteristcs------
const int LEDheading = 315; //degree where the LED heading is centered, adjust for tuning heading vs driving direction
const int percentdecel = 15; //percentage of rotation the translation deceleration wave occurs for each motor. Should be <= 50
const float acc_rate = 0.4; //0.2(least aggressive spinup) - 1.0(most aggressive spinup)

//-------Wifi config---------
const char *ssid = "Beyblade"; //wifi ssid
const char *password = "meltybrain"; //wifi password
// Start TelnetStream: Enter telnet {insert ip address} for me: telnet 192.168.4.1 in CMD to view output

//-----other constants-------
const int denom = round((1.0 / sqrt(0.00001118*accradius))*3); //calculates denominator ahead of time to reduce unnecessary calculations;
const int OFFSET_ADDR = 0;  // EEPROM address to store rpm offset
const int WDT_TIMEOUT = 5;  // seconds

//=============GLOBAL VARIABLES==================
//-------LED control-------
String LEDStatus = "armed"; //tells LED control what mode to be in
unsigned long lastUpdate = 0;   // last LED animation update time
long stripstart = 0;   //time of last update
int animIndex = 0;     // position in LED animation
int direction = 1; // 1 = forward, -1 = backward
//-------rpm and heading calc--------
int rpm;
unsigned long long previoustime = 0; //will store last time that was updated
float heading = 1.0; //variable used to adjust heading using left transmitter stick
float offset = 1.0; //creates variable to tune a stable heading, will update from EEPROM in setup
int angle = 0; //creates a variable to track the rotation angle of the bot from its heading
int max_rpm = 0;  // stores the greatest rpm achieved
int max_gforce = 0; //for storing max gforce achieved
//--------translation----------
unsigned long long dtime; //variable for the time it's been since decel start
unsigned long long startime; //variable to store the time in microseconds that the decel started
bool startimeset = false;
bool off_set = false;
bool reversed = false;  //used for reverse spin direction
unsigned long loopstart = 0; //stores last gforce read time
int motorL;
int motorR;
int spinspeed;
unsigned long long motorRsent = 0; //timestamp of last right motor command
unsigned long long motorLsent = 0; //timestamp of last left motor command
bool motorRsend = false; //alternation flag for motor sending
//--------reciever----------
unsigned long chanstart = 0; //stores last RX read time
int pwm[NUM_CHANNELS + 1]; //list values from 1000-2000 (indexed 1-6)
int duty[NUM_CHANNELS + 1]; //list values from 0 - 100 (indexed 1-6)
int last_rec; //used to store update times for detecting signal loss
int rec_gap;
int rec_last;
bool rc_status = false; //whether getting rc signal, used for triggering failsafe
//----------other----------
float volts; //variable to store current battery voltage
int batloop; //variable for tracking battery update loop
long loop_time;
long loop_start;
bool motor_on = false;
int graph[200];
int n = 0;

//=============FUNCTION PROTOTYPES==================
// Heading.ino
void calcrpm();
long long spintimefunct();
float get_accel_force_g();
int rotation_angle();
bool isAngleInRange(float center, float range);
void heading_funct();
void heading_adj();
float readOffsetFromEEPROM();

// LED_Control.ino
void updateLED();
void led_update_task(void *pvParameters);

// Motor_Control.ino
void update_motors();
void spin();
void translate();

// RC_Handler.ino
void update_channels();
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

// Wifi_Handler.ino
void wifi_mode();
void data_export();

// Main functions
void get_battery();
void failsafe();

static float last_gforce_raw = 0.0;

// Debug print: 5Hz only
#define DEBUG_PRINT_INTERVAL_US 200000  // 200ms = 5Hz
static unsigned long long last_debug_print_us = 0;

// Loop timing stats (min/avg/max over each 200ms window) – see explanation below
static unsigned long loop_time_min_us = 0;
static unsigned long loop_time_max_us = 0;
static unsigned long long loop_time_sum_us = 0;
static unsigned int loop_time_count = 0;

// LOOP TIMING NOTES:
// - Measurement: t0 at loop start -> loop_end after update_motors(). Serial print is
//   after loop_end, so it is NOT included in loopTime_us. Measurement is correct.
// - "Suspiciously high" 80–90 kHz: update_motors() often does nothing. We alternate
//   L/R ESCs with 100us between each; most loops skip both motor updates. The loop
//   is just update_channels, heading, calcrpm, conditionals, update_motors (no-op).
// - Occasional 700–1000 us loops: FreeRTOS preemption. I2C, CRSF, LED tasks all run
//   at same priority (1) as the main loop. When preempted (e.g. I2C doing 6× reads
//   ~500–1000 us), t0->loop_end spans that delay. esp_timer_get_time() is wall-clock,
//   so we measure real iteration latency. Slow loops are real, not measurement error.


//=============HEADING FUNCTIONS==================
void calcrpm()  // calculates the rpm based on g-force
{
  float gforce;
  gforce = fabs(get_accel_force_g());
  rpm = round(sqrt(gforce) * denom);

  if(rpm > max_rpm) 
  {
    max_rpm = rpm;
  }
}

long long spintimefunct()
{
  unsigned long long spintime;
  calcrpm();
  if(rpm > 200)
  {
    spintime = (60000000LL / rpm) * offset * heading; //calculates microseconds per revolution
  }
  else
  {
    spintime = 1000; //some random placeholder
  }
  if(rpm > 2000)
  {
    graph[n] = spintime;
    if (n >= 199)
    {
      n = 0;
    }
    else{
      n = n + 1;
    }
  }
  return spintime;
}

// Non-blocking I2C read - just returns latest value from background task
// Always returns immediately (non-blocking)
static bool accel_read_y_nonblocking(float *result) {
  if (accel_initialized) {
    *result = latest_gforce;  // Atomic read (float is 32-bit, safe on ESP32)
    last_gforce_raw = latest_gforce;
    return true;
  }
  *result = 0.0;
  return true;
}

float get_accel_force_g() 
{
  if (!accel_initialized) return 0.0f;
  static const int NUM_SAMPLES = 7;  // Reduced from 10 to 7 for faster response (still smooth)
  static float samples[NUM_SAMPLES];
  static int index = 0;
  static bool filled = false;
  static float total = 0.0;
  static unsigned long last_update = 0;

  // Get latest reading from background task (non-blocking, instant)
  float gforce;
  accel_read_y_nonblocking(&gforce);
  
  // Update rolling average every 20ms (background task reads every 20ms)
  // Use cached time from last_i2c_update_time if available, otherwise get current time
  unsigned long now = (last_accel_update_time > 0) ? last_accel_update_time : esp_timer_get_time();
  if (now - last_update > 20000) {
    last_update = now;
    
    // update rolling average
    total -= samples[index];
    samples[index] = gforce;
    total += gforce;

    index = (index + 1) % NUM_SAMPLES;
    if (index == 0) filled = true;

    if (round(gforce) > max_gforce) {
      max_gforce = round(gforce);
    }
  }

  // return current average
  if (filled) return total / NUM_SAMPLES;
  else return total / (index + 1); // partial average while filling buffer
}

int rotation_angle()
{
  unsigned long long currentime;
  unsigned long long spintime;
  spintime = spintimefunct();
  currentime = esp_timer_get_time();
  //angle = ((currentime - ((irVisibleEnd + irVisibleStart) / 2) ) / spintime) * 360;
  angle = ((double)(currentime - previoustime) / (double)spintime) * 360.0;
  if((currentime - previoustime) >= spintime)  //resets timer every revolution
  {
    previoustime = currentime + (spintime - (currentime - previoustime));
  }
  return angle;
}

bool isAngleInRange(float center, float range)
{
  center = fmod((center + 360), 360);

  // Compute smallest difference, -180..180
  float diff = fabs(fmod((angle - center + 540), 360) - 180);

  return diff <= range; // within ±range degrees
}

void heading_funct()
{
  if(reversed == true)
  {
    heading = (map(duty[1], 0, 100, 50, -50) * 0.001) + 1;
  }
  else
  {
    heading = (map(duty[1], 0, 100, -50, 50) * 0.001) + 1;
  }
}

void heading_adj()
{
  if(offset < 0.8 or offset > 1.2)
  {
    offset = 1.0;
  }
  if(duty[4] < 30 && off_set == false)
  {
    offset = offset - 0.004;
    EEPROM.put(OFFSET_ADDR, offset);
    EEPROM.commit();  // Required on ESP32 to finalize the write
    off_set = true;
  }
  else if(duty[4] > 70 && off_set == false)
  {
    offset = offset + 0.004;
    EEPROM.put(OFFSET_ADDR, offset);
    EEPROM.commit();  // Required on ESP32 to finalize the write
    off_set = true;
  }
  else if(duty[4] > 40 && duty[4] < 60)
  {
    off_set = false;
  }
}

float readOffsetFromEEPROM()
{
  float val;
  EEPROM.get(OFFSET_ADDR, val);
  if (isnan(val) || abs(val) > 360.0) { // sanity check
    return 0.0;
  }
  return val;
}

//=============LED CONTROL FUNCTIONS==================
// FreeRTOS task to update LEDs in background (non-blocking, angle-based with 5° precision)
void led_update_task(void *pvParameters) {
  Serial.println("  [LED Task] Started");
  
  static int last_displayed_angle = -999;  // Track last displayed angle
  static String last_processed_status = "";  // Track last processed status
  static unsigned long last_anim_update = 0;  // For animation timing
  static unsigned long last_armed_update = 0;  // For armed status animation timing
  static int anim_index = 0;  // Animation index (local to task)
  static int anim_direction = 1;  // Animation direction
  
  const int ANGLE_PRECISION = 5;  // 5° precision (72 updates per rotation)
  
  while (1) {
    int64_t t0 = esp_timer_get_time();
    // Read shared variables (atomic reads)
    int current_angle = led_angle_shared;
    int current_rpm = led_rpm_shared;
    char current_status[32];
    strncpy(current_status, led_status_shared, sizeof(current_status) - 1);
    current_status[sizeof(current_status) - 1] = '\0';  // Ensure null termination
    bool status_changed = led_data_changed;
    
    // Calculate angle difference with wrap-around handling
    int angle_diff = abs(current_angle - last_displayed_angle);
    if (angle_diff > 180) {
      angle_diff = 360 - angle_diff;  // Handle wrap-around (e.g., 359° to 1° = 2° difference)
    }
    
    // Determine if we need to update LEDs
    bool need_update = false;
    
    // Update if angle changed by 5° or more
    if (angle_diff >= ANGLE_PRECISION) {
      need_update = true;
    }
    
    // Update if status changed
    if (status_changed || strcmp(current_status, last_processed_status.c_str()) != 0) {
      need_update = true;
      led_data_changed = false;  // Clear flag
      last_processed_status = String(current_status);
    }
    
    // Update animation index for failsafe mode
    if (strcmp(current_status, "failsafe") == 0 && (millis() - last_anim_update) > animSpeed) {
      anim_index += anim_direction;
      if (anim_index >= 9) {
        anim_index = 9;
        anim_direction = -1;
      } else if (anim_index <= 0) {
        anim_index = 0;
        anim_direction = 1;
      }
      last_anim_update = millis();
      need_update = true;
    }
    
    // Update armed status animation (every 2 seconds)
    if (strcmp(current_status, "armed") == 0 && (millis() - last_armed_update) > 2000) {
      last_armed_update = millis();
      need_update = true;
    }
    
    if (need_update) {
      // LED logic depending on status
      // Note: rc_status is a global variable, safe to read (it's set in main loop)
      bool current_rc_status = rc_status;  // Read once for this update
      if (strcmp(current_status, "failsafe") == 0) 
      {
        if (current_rc_status == false) 
        {
          // yellow bounce chase
          top_strip.clear();
          bottom_strip.clear();
          top_strip.setPixelColor(anim_index, 100, 80, 0);
          bottom_strip.setPixelColor(anim_index, 100, 80, 0);
        }
        else
        {
          // breathing blue effect
          float phase = (millis() % 2000) / 2000.0 * 2 * PI;
          int brightness = (sin(phase) * 50) + 50;
          for (int i = 0; i < NUMPIXELST; i++) {
            top_strip.setPixelColor(i, 0, 0, brightness);
          }
          for (int i = 0; i < NUMPIXELSB; i++) {
            bottom_strip.setPixelColor(i, 0, 0, brightness);
          }
        }
      }
      else if (strcmp(current_status, "export") == 0)
      {
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 100, 100, 100);
        }
        for (int i = 0; i < NUMPIXELSB; i++) {
          bottom_strip.setPixelColor(i, 100, 100, 100);
        }
      }
      else if (strcmp(current_status, "armed") == 0)
      {
        top_strip.clear();
        bottom_strip.clear();
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 100, 0, 0);
        }
        for (int i = 0; i < NUMPIXELSB; i++) {
          bottom_strip.setPixelColor(i, 100, 15, 0);
        }
      }
      else if (strcmp(current_status, "heading on") == 0) 
      {
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 0, 255, 0);
        }
        for (int i = 0; i < NUMPIXELSB; i++) {
          bottom_strip.setPixelColor(i, 0, 255, 100);
        }
      }
      else if (strcmp(current_status, "motor on") == 0) 
      {
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 80, 0, 0);
        }
        for (int i = 0; i < NUMPIXELSB; i++) {
          bottom_strip.setPixelColor(i, 80, 0, 0);
        }
      }
      else if (strcmp(current_status, "reading") == 0)
      {
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 0, 0, 255);
        }
        for (int i = 0; i < NUMPIXELSB; i++) {
          bottom_strip.setPixelColor(i, 0, 0, 255);
        }
      }
      else 
      {
        top_strip.clear();
        bottom_strip.clear();
      }
      
      top_strip.show();
      bottom_strip.show();
      
      // Update last displayed angle
      last_displayed_angle = current_angle;
    }
    
    int64_t t1 = esp_timer_get_time();
    prof_record(PROF_LED, (uint32_t)(t1 - t0));
    // Small delay to yield to other tasks (prevents task from consuming all CPU)
    // Using 0ms delay = yield immediately, check again as fast as possible
    vTaskDelay(pdMS_TO_TICKS(0));
  }
}

// Legacy updateLED function - now just updates shared variables (non-blocking)
void updateLED() 
{
  // Update shared variables for background LED task (non-blocking)
  static String last_status = "";
  
  led_angle_shared = angle;
  led_rpm_shared = rpm;
  
  // Copy String to char array (thread-safe)
  strncpy(led_status_shared, LEDStatus.c_str(), sizeof(led_status_shared) - 1);
  led_status_shared[sizeof(led_status_shared) - 1] = '\0';  // Ensure null termination
  
  // Check if status changed
  if (LEDStatus != last_status) {
    led_data_changed = true;
    last_status = LEDStatus;
  }
  
  // LED task handles all the actual LED updates in background
  // This function now returns immediately without blocking
}

//=============MOTOR CONTROL FUNCTIONS==================
void update_motors()
{
  // Safety clamp: prevent values outside valid DShot 3D range (-999 to 999)
  if (motorL > 999) motorL = 999;
  if (motorL < -999) motorL = -999;
  if (motorR > 999) motorR = 999;
  if (motorR < -999) motorR = -999;
  
  unsigned long long current_time = esp_timer_get_time();
  
  // Check both motors independently and send to whichever is ready
  // This prevents long delays when one motor's timing condition isn't met
  // Maintain minimum 100us spacing between sends to avoid RMT conflicts
  // Fixed: Check both motors every iteration instead of strict alternation
  
  bool right_ready = (current_time - motorRsent) >= 100;
  bool left_ready = (current_time - motorLsent) >= 100;
  unsigned long long time_since_last_send = 0;
  
  // If both are ready, send to the one that was sent longest ago first
  // This ensures fair scheduling and prevents one motor from starving
  if (right_ready || left_ready) {
    int64_t t0 = esp_timer_get_time();
    if (right_ready && left_ready) {
      // Send to the motor that was sent longest ago
      if ((current_time - motorRsent) >= (current_time - motorLsent)) {
        escR.sendThrottle3D(motorR);
        motorRsent = current_time;
      } else {
        escL.sendThrottle3D(motorL);
        motorLsent = current_time;
      }
    } else if (right_ready) {
      escR.sendThrottle3D(motorR);
      motorRsent = current_time;
    } else {
      escL.sendThrottle3D(motorL);
      motorLsent = current_time;
    }
    prof_record(PROF_ESC, (uint32_t)(esp_timer_get_time() - t0));
    motorRsend = !motorRsend;
  }
}

void spin()
{
  motor_on = false;
  
  // Read duty[3] and rpm once to prevent race condition with CRSF updates
  // This ensures we use consistent values throughout the calculation
  int ch3_duty = duty[3];
  int current_rpm = rpm;  // Read once for consistency
  
  // Calculate RPM-based limit to prevent rapid acceleration
  // Limit is acc_rate * rpm + 200, allowing higher speeds at higher RPM
  long rpm_limit = long(acc_rate * current_rpm) + 200;
  
  if(reversed == true)
  {
    //pinspeed = map(duty[3], 0, 100, 0, -1000);
    long requested = map(ch3_duty, 0, 100, 0, -1000);
    spinspeed = max(requested, -rpm_limit);
    
    // Enforce limit: never exceed the RPM-based limit (prevent rapid acceleration bypass)
    if (spinspeed < -rpm_limit) spinspeed = -rpm_limit;
  }
  else
  {
    //spinspeed = map(duty[3], 0, 100, 0, 1000);
    long requested = map(ch3_duty, 0, 100, 0, 1000);
    spinspeed = min(requested, rpm_limit);
    
    // Enforce limit: never exceed the RPM-based limit (prevent rapid acceleration bypass)
    if (spinspeed > rpm_limit) spinspeed = rpm_limit;
  }
  
  motorR = spinspeed;
  motorL = spinspeed;
}

void translate()
{
  unsigned long long currentime;
  unsigned long long duration; //defines variable for decel duration in microseconds
  int transpeed; //variable to store movement speed 0-100 from ch2 duty
  currentime = esp_timer_get_time();
  duration = spintimefunct() * float(percentdecel) * 0.01; //duration of total decel pulse
  dtime = currentime - startime; //calculates time it's been since start of decel, will need a way to start the timer when decel initiates
  // Read duty values and rpm once to prevent race condition with CRSF updates
  // This ensures we use consistent values throughout the calculation
  int ch3_duty = duty[3];
  int ch2_duty = duty[2];
  int current_rpm = rpm;  // Read once for consistency
  
  // Calculate RPM-based limit to prevent rapid acceleration
  // Limit is acc_rate * rpm + 200, allowing higher speeds at higher RPM
  long rpm_limit = long(acc_rate * current_rpm) + 200;
  
  if(reversed == true)
  {
    long requested = map(ch3_duty, 0, 100, 0, -1000);
    spinspeed = max(requested, -rpm_limit);
    
    // Enforce limit: never exceed the RPM-based limit (prevent rapid acceleration bypass)
    if (spinspeed < -rpm_limit) spinspeed = -rpm_limit;
    
    transpeed = abs(map(ch2_duty, 0, 100, -100, 100));  //-100 to 100 and due to formula, - will automatically switch motor direction without needing separate if statements
  }
  else
  {
    long requested = map(ch3_duty, 0, 100, 0, 1000);
    spinspeed = min(requested, rpm_limit);
    
    // Enforce limit: never exceed the RPM-based limit (prevent rapid acceleration bypass)
    if (spinspeed > rpm_limit) spinspeed = rpm_limit;
    
    transpeed = abs(map(ch2_duty, 0, 100, 100, -100));  //-100 to 100 and due to formula, - will automatically switch motor direction without needing separate if statements
  }
  if(angle > 180)
  {
    if(angle > 180 && angle < 250 && startimeset == false) //resets start time when 300 degrees is hit
    {
      startime = currentime;
      dtime = 0;
      startimeset = true;
    }
    if(angle > 250)
    {
      startimeset = false;
    }
    if(dtime < duration)
    {
      if(reversed == true)
      {
        motorR = map(transpeed, 0, 100, spinspeed, 0);
        motorL = map(transpeed, 0, 100, spinspeed, -1000);
      }
      else
      {
        motorR = map(transpeed, 0, 100, spinspeed, 0);
        motorL = map(transpeed, 0, 100, spinspeed, 1000);
      }
      motor_on = true;
    }
    else
    {
      spin();
    }
  }
  else
  {
    if(angle < 80 && startimeset == false) //resets start time when 120 degrees is hit
    {
      startime = currentime;
      dtime = 0;
      startimeset = true;
    }
    if(angle > 80)
    {
      startimeset = false;
    }
    if(dtime < duration)
    {
      if(reversed == true)
      {
        motorL = map(transpeed, 0, 100, spinspeed, 0);
        motorR = map(transpeed, 0, 100, spinspeed, -1000);
      }
      else
      {
        motorL = map(transpeed, 0, 100, spinspeed, 0);
        motorR = map(transpeed, 0, 100, spinspeed, 1000);
      }
      motor_on = true;
    }
    else
    {
      spin();
    }
  }
}

// CRSF raw->us scale (same as library rcToUs): (raw * 0.62477120195241) + 881
#define CRSF_RAW_TO_US(raw)  ((uint16_t)(((raw) * 0.62477120195241F) + 881))

//=============RC HANDLER FUNCTIONS==================
// CRSF callback - use rcChannels->value[] directly, convert to us inline (no getChannel/rcToUs per channel)
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
  if (rcChannels->failsafe == false)
  {
    for (int ch = 1; ch <= NUM_CHANNELS; ch++) {
      uint16_t raw = rcChannels->value[ch - 1];
      crsf_channels[ch] = CRSF_RAW_TO_US(raw);
    }
    crsf_failsafe = false;
    last_crsf_update = esp_timer_get_time();
  }
  else
  {
    crsf_failsafe = true;
  }
}

void update_channels()
{
  // Copy channel data from CRSF callback (filled when crsf->update() runs in main loop)
  for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
    uint16_t crsfVal = crsf_channels[channel];  // Atomic read
    pwm[channel] = crsfVal;
    duty[channel] = map(crsfVal, 1000, 2000, 0, 100);
  }
  
  // Use failsafe status from CRSF library callback
  bool current_failsafe = crsf_failsafe;  // Atomic read
  
  if (!current_failsafe && pwm[1] > 900) {
    rc_status = true;
  } else {
    // Failsafe active - set safe channel values
    duty[1] = 50;
    duty[2] = 50;
    duty[3] = 0;
    duty[4] = 50;
    duty[5] = 100;
    duty[6] = 0;
    rc_status = false;
  }
}

//=============WIFI HANDLER FUNCTIONS==================
void wifi_mode()   //turns on wifi mode to connect wirelessly
{
  bool exportdata = false;
  static unsigned long lastWifiPrint = 0;
  
  // Explicitly reset motor values to 0 when entering WiFi mode
  motorL = 0;
  motorR = 0;
  failsafe();
  esp_task_wdt_init(WDT_TIMEOUT, false);  // disable watchdog
  
  // Enable WiFi (was disabled in setup() to prevent background tasks)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("=== WIFI/KILL MODE ACTIVE ===");
  Serial.println("Motors stopped. Flip CH5 OFF to resume.");
  Serial.print("WiFi SSID: ");
  Serial.println(ssid);
  ArduinoOTA.begin();  //starts ota
  
  while(duty[5] > 50)  //stuck in loop so robot cannot run while in wifi mode, channel 5 initiates wifi
  {
    // Keep motor values at 0 and continuously send stop commands
    motorL = 0;
    motorR = 0;
    failsafe();
    update_channels();
    update_motors();  // Continuously send stop commands to ESCs
    ArduinoOTA.handle();
    
    // Print status periodically while in wifi mode
    if (millis() - lastWifiPrint > 500) {
      Serial.print("WIFI MODE | CH5:");
      Serial.print(duty[5]);
      Serial.print(" | CH6:");
      Serial.print(duty[6]);
      Serial.println(" | Flip CH5 OFF to exit");
      lastWifiPrint = millis();
    }
    
    if(duty[6] < 50)  //if export switch is not triggered, will not export
    {
      exportdata = false;
    }
    if((duty[6] > 50) && (exportdata == false)) //if export switch is triggered and it has not exported yet, call export function
    {
      LEDStatus = "export";
      updateLED();
      data_export();
      exportdata = true;
    }
  }
  Serial.println("=== EXITING WIFI MODE ===");
  WiFi.softAPdisconnect(false);  //turn off wifi
  WiFi.mode(WIFI_OFF);  // Explicitly disable WiFi to stop all background tasks
  delay(500);
  for (int i = 0; i < 2500; i++)
  {
    escR.writeData(0, true);
    delayMicroseconds(400);
    escL.writeData(0, true);
    delayMicroseconds(400);
  }
  // Explicitly reset motor values to 0 after re-arming
  motorL = 0;
  motorR = 0;
  update_motors();  // Send stop command to ensure motors are stopped
  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable watchdog
}

void data_export()    //exports data to telnet client for diagnostics, wifi mode must be turned on first
{
  //Serial.println("data export");
  TelnetStream.begin(); //start telnet
  delay(2000);
    // Print data
  TelnetStream.println("Telnet stream started");
  TelnetStream.println("==== Sensor Data Export ====");
  TelnetStream.print("Max RPM: ");
  TelnetStream.println(max_rpm);
  TelnetStream.print("loop time: ");
  TelnetStream.println(loop_time);
  TelnetStream.print("battery: ");
  TelnetStream.println(volts);
  TelnetStream.print("Max G-Force: ");
  TelnetStream.println(max_gforce);
  TelnetStream.print("G-force output: ");
  for(int i = 0; i < 199; i++)
  {
    TelnetStream.print(",");
    TelnetStream.print(graph[i]);
  }
  
  TelnetStream.println("============================");
  TelnetStream.stop(); // Close telnet connection
}

//=============MAIN FUNCTIONS==================
void get_battery()  //calculates battery voltage based on voltage divider input on pin A0
{
  // Use millis() for battery check (only runs every 1s, so overhead is negligible)
  if ((millis() - batloop) > 1000) //samples every second to avoid flickering
  {
    volts = (float(analogRead(volt_pin)) / float(310.0)) * 1.29;
    batloop = millis();
  }
}

void failsafe() //failsafe mode, shuts off all motors
{
      LEDStatus = "failsafe";
      motorR = 0;
      motorL = 0;
      update_motors();  // Send stop command to ESCs
      updateLED();
}

//=============SETUP==================
void setup() 
{
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  Serial.println();
  Serial.println("================================");
  Serial.println("Ash-Nazg Melty Brain - V5");
  Serial.println("Initializing...");
  Serial.println("================================");
  
  Serial.println("Starting EEPROM...");
  EEPROM.begin(512);  // Initialize EEPROM with 512 bytes of storage (adjust size if needed)
  offset = readOffsetFromEEPROM();
  Serial.print("Offset loaded: ");
  Serial.println(offset);
  
  loopstart = esp_timer_get_time();
  
  Serial.println("Starting accelerometer (SPI)...");
  Serial.print("  SCK=GPIO ");
  Serial.print(ACCEL_SCK_PIN);
  Serial.print(", MOSI=GPIO ");
  Serial.print(ACCEL_MOSI_PIN);
  Serial.print(", MISO=GPIO ");
  Serial.print(ACCEL_MISO_PIN);
  Serial.print(", CS=GPIO ");
  Serial.println(ACCEL_CS_PIN);
  
  pinMode(ACCEL_CS_PIN, OUTPUT);
  digitalWrite(ACCEL_CS_PIN, HIGH);
  SPI.begin(ACCEL_SCK_PIN, ACCEL_MISO_PIN, ACCEL_MOSI_PIN, ACCEL_CS_PIN);
  
  accel_spi_write(CTRL_REG1, 0x27);  // Normal mode, 400Hz, XYZ enabled
  delay(10);
  accel_spi_write(CTRL_REG4, 0x90);  // 400g, high res
  delay(100);
  
  int16_t test_xyz[3];
  accel_spi_read_xyz(test_xyz);
  Serial.print("  Test read: X=");
  Serial.print(test_xyz[0]);
  Serial.print(" Y=");
  Serial.print(test_xyz[1]);
  Serial.print(" Z=");
  Serial.println(test_xyz[2]);
  
  Serial.println("  Accelerometer configured (400g range, 400Hz)");
  xTaskCreate(
    accel_spi_read_task,
    "Accel_SPI",
    2048,
    NULL,
    1,
    &accel_task_handle
  );
  accel_initialized = true;
  Serial.println("  Background accel SPI task started");
  
  Serial.println("Starting CRSF receiver...");
  Serial.print("  CRSF RX Pin: GPIO ");
  Serial.println(CRSF_RX_PIN);
  Serial.print("  CRSF TX Pin: GPIO ");
  Serial.println(CRSF_TX_PIN);
  
  // Configure Serial1 with custom pins for ESP32-S2
  Serial1.begin(420000, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  
  // Initialize CRSF for Arduino (pointer-based, like example)
  crsf = new CRSFforArduino();
  if (!crsf->begin()) {
    crsf->end();
    delete crsf;
    crsf = nullptr;
    
    Serial.println("ERROR: CRSF initialization failed!");
    while (1) {
      delay(10);
    }
  } else {
    Serial.println("CRSF initialized successfully.");
    
    // Set callback for RC channels (like example)
    crsf->setRcChannelsCallback(onReceiveRcChannels);
    Serial.println("  CRSF: polling in main loop (no background task)");
  }
  
  Serial.println("Starting LED strips (DotStar)...");
  top_strip.begin(); //initializes LEDs
  bottom_strip.begin();
  top_strip.clear();  // Turn all LEDs off ASAP
  bottom_strip.clear();
  top_strip.show();   // Send clear command
  bottom_strip.show(); // Send clear command
  
  // Initialize shared LED variables
  led_angle_shared = 0;
  led_rpm_shared = 0;
  strncpy(led_status_shared, "export", sizeof(led_status_shared) - 1);
  led_status_shared[sizeof(led_status_shared) - 1] = '\0';
  led_data_changed = true;
  
  // Start background LED update task (non-blocking, angle-based with 5° precision)
  xTaskCreate(
    led_update_task,      // Task function
    "LED_Update",         // Task name
    4096,                 // Stack size (larger for LED logic)
    NULL,                 // Parameters
    1,                    // Priority (low, so main loop has priority)
    &led_task_handle      // Task handle
  );
  led_task_initialized = true;
  Serial.println("  Background LED update task started (non-blocking, 5° precision)");
  
  LEDStatus = "export";
  delay(2000);
  
  // Disable WiFi by default to prevent background tasks from running
  // WiFi will only be enabled when explicitly entering wifi_mode()
  WiFi.mode(WIFI_OFF);
  
  Serial.println("Configuring OTA...");
  ArduinoOTA.setHostname("esp32-ap"); //wifi ota config
  ArduinoOTA.setPassword("admin"); //enter this if window opens in arduino IDE asking for pswrd
  // Note: ArduinoOTA.begin() is only called in wifi_mode(), so OTA is not active by default
  
  // DShotESC: install(gpio, rmtChannel, frequency, divider) - 600kHz = DSHOT600
  Serial.println("Installing ESC drivers (DShotESC 600kHz)...");
  escR.install(escR_gpio, RMT_CHANNEL_3, 600000UL, 3);
  escL.install(escL_gpio, RMT_CHANNEL_2, 600000UL, 3);

  Serial.println("Arming ESCs...");
  for (int i = 0; i < 2500; i++) //arm esc's
  {
    escR.writeData(0, true);
    delayMicroseconds(400);
    escL.writeData(0, true);
    delayMicroseconds(400);
  }
  escR.setReversed(false);
  escR.set3DMode(true);
  delayMicroseconds(200);
  escL.setReversed(false);
  escL.set3DMode(true);

  Serial.println("Starting watchdog...");
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Enable panic so ESP32 restarts, esp_task_wdt_reset(); feeds watchdog
  esp_task_wdt_add(NULL);  // Add current thread (loopTask) to WDT
  
  // Initialize loop timing variables
  loop_start = esp_timer_get_time();
  loopstart = loop_start;
  
  Serial.println("================================");
  Serial.println("Initialization complete!");
  Serial.println("System ready.");
  Serial.println("================================");
}

//=============MAIN LOOP==================
// Optimized loop for maximum ESC update frequency
// Operations ordered from fastest to slowest

void loop() 
{
  unsigned long long t0 = esp_timer_get_time();
  
  // CRSF: poll in main loop (no task) so Serial1 is read every iteration
  if (crsf != nullptr) {
    int64_t t_crsf0 = esp_timer_get_time();
    crsf->update();
    prof_record(PROF_CRSF, (uint32_t)(esp_timer_get_time() - t_crsf0));
  }
  update_channels();
  heading_funct();
  heading_adj();
  calcrpm();
  
  if (!rc_status) {
    failsafe();
    unsigned long long t1 = esp_timer_get_time();
    prof_record(PROF_LOOP, (uint32_t)(t1 - t0));
    // prof_report();  // Disabled
    // Debug output (5Hz) - also when in failsafe
    static unsigned long long last_failsafe_debug_us = 0;
    if ((t1 - last_failsafe_debug_us) >= DEBUG_PRINT_INTERVAL_US) {
      last_failsafe_debug_us = t1;
      Serial.print("L=");
      Serial.print(motorL);
      Serial.print(" R=");
      Serial.print(motorR);
      Serial.print(" CH3=");
      Serial.print(pwm[3]);
      Serial.print(" G=");
      Serial.print(last_gforce_raw, 2);
      // If last_crsf_update is 0, no valid CRSF packet has ever been received (wiring/baud/receiver)
      Serial.print(" crsf_us=");
      Serial.print((unsigned long)last_crsf_update);
      Serial.println(" [failsafe]");
    }
    esp_task_wdt_reset();
    // Yield so CRSF task can run and read Serial1; otherwise main loop starves it at same priority
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
  }
  
  if(duty[5] > 50)
  {
    wifi_mode();
  }
  
  reversed = (duty[6] > 50);
  
  if(rpm > 400)
  {
    angle = rotation_angle();
    if (isAngleInRange(LEDheading, 20)) // LED turns on within ±10° of heading
    {
      LEDStatus = "heading on";
    }
    else if(motor_on == true)
    {
      LEDStatus = "motor on";
    }
    else //turns off led
    {
      LEDStatus = "heading off";
    }
    if(duty[2] > 60 || duty[2] < 40)
    {
      translate();
    }
    else 
    {
      spin();
    }
  }
  else if(duty[3] > 10)  // Spin command
  {
    spin();
  }
  else if(reversed == true && (duty[2] > 55 || duty[2] < 45)) // Unstick using the right stick
  {
    // Use same range as tank mode to prevent exceeding limits
    motorL = map(duty[2], 0, 100, -120, 120);
    motorR = map(duty[2], 0, 100, 120, -120);
  }
  else if(duty[2] > 55 || duty[2] < 45 || duty[1] > 55 || duty[1] < 45) // Tank drive mode
  {
    // Apply dead zone: if stick is within ±5% of center (45-55), treat as 50 (neutral)
    int ch2_value = duty[2];
    int ch1_value = duty[1];
    
    // Apply dead zone to each channel
    if (ch2_value >= 45 && ch2_value <= 55) {
      ch2_value = 50;  // Center position
    }
    if (ch1_value >= 45 && ch1_value <= 55) {
      ch1_value = 50;  // Center position
    }
    
    // If both sticks are centered after dead zone, stop motors
    if (ch2_value == 50 && ch1_value == 50) {
      motorL = 0;
      motorR = 0;
    } else {
      // Calculate motor values with dead zone applied (original -100 to 100 range)
      motorL = map(ch2_value, 0, 100, -100, 100) + map(ch1_value, 0, 100, -20, 20);
      motorR = map(ch2_value, 0, 100, 100, -100) + map(ch1_value, 0, 100, -20, 20);
      
      // Clamp tank mode values to -120 to 120 to prevent exceeding limits
      // This prevents the issue where quick stick movements can cause values to exceed 120
      if (motorL > 120) motorL = 120;
      if (motorL < -120) motorL = -120;
      if (motorR > 120) motorR = 120;
      if (motorR < -120) motorR = -120;
    }
  }
  else
  {
    // Sticks are centered - explicitly stop motors
    motorL = 0;
    motorR = 0;
  }
  
  if(rpm < 400)
  {
    LEDStatus = "armed";
  }
  
  updateLED();
  update_motors();
  
  unsigned long long loop_end_us = esp_timer_get_time();
  unsigned long loopTime_us = (unsigned long)(loop_end_us - t0);
  loop_time = loopTime_us;
  loop_start = loop_end_us;
  
  // Update min/avg/max stats (reset each print window)
  if (loop_time_count == 0) {
    loop_time_min_us = loopTime_us;
    loop_time_max_us = loopTime_us;
    loop_time_sum_us = 0;
  }
  loop_time_count++;
  if (loopTime_us < loop_time_min_us) loop_time_min_us = loopTime_us;
  if (loopTime_us > loop_time_max_us) loop_time_max_us = loopTime_us;
  loop_time_sum_us += loopTime_us;
  
  prof_record(PROF_LOOP, (uint32_t)loopTime_us);
  // prof_report();  // Disabled - was 2 Hz task CPU-time profile dump
  
  // Debug output at 5Hz: motor speeds, channel 3, accelerometer
  static unsigned long long last_debug_print_us = 0;
  unsigned long long now_us = esp_timer_get_time();
  if ((now_us - last_debug_print_us) >= DEBUG_PRINT_INTERVAL_US) {
    last_debug_print_us = now_us;
    Serial.print("L=");
    Serial.print(motorL);
    Serial.print(" R=");
    Serial.print(motorR);
    Serial.print(" CH3=");
    Serial.print(pwm[3]);
    Serial.print(" G=");
    Serial.print(last_gforce_raw, 2);
    Serial.println();
  }
  
  esp_task_wdt_reset();
}
