/**
 * Ash-Nazg Melty Brain Robot - V5
 *
 * Adafruit QT Py ESP32-S3 (dual-core).
 * DShot ESC output runs on Core 1 at fixed 6 kHz for steady timing (4–8 kHz range);
 * main loop, CRSF, accel, and LEDs run on Core 0.
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
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
#define PROF_REPORT_INTERVAL_US  1000000   // 1 s window -> count = Hz
static unsigned long long last_prof_report_us = 0;
static void prof_report(void) {
  unsigned long long now = (unsigned long long)esp_timer_get_time();
  if ((now - last_prof_report_us) < PROF_REPORT_INTERVAL_US) return;
  last_prof_report_us = now;
  Serial.println();
  Serial.println("=== Task CPU-time profile (us, last 1s window, count=Hz) ===");
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

// LIS331 accelerometer in SPI mode: chip pins are SDA, SCL, SDO, CS.
// Wire LIS331 -> Qt Py ESP32-S3:
//   LIS331 SDA (data in)  -> Qt Py GPIO 35  (our MOSI)
//   LIS331 SCL (clock)    -> Qt Py GPIO 36  (our SCK)
//   LIS331 SDO (data out) -> Qt Py GPIO 37  (our MISO)
//   LIS331 CS             -> Qt Py GPIO 8   (our CS)
#define ACCEL_SCK_PIN  36  // Qt Py SCK  -> LIS331 SCL
#define ACCEL_MOSI_PIN 35  // Qt Py MOSI -> LIS331 SDA
#define ACCEL_MISO_PIN 37  // Qt Py MISO -> LIS331 SDO
#define ACCEL_CS_PIN   8   // Qt Py CS   -> LIS331 CS
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
  Serial.println("  [Accel SPI Task] Started (Core 0)");
  
  while (1) {
    int64_t t0 = esp_timer_get_time();
    accel_spi_read_xyz(xyz);
    int16_t y = xyz[1] >> 4;  // 12-bit left-justified, Y axis
    float gforce = (float)ACCEL_MAX_SCALE * (float)y / 2047.0f;
    
    // Store result (atomic write - float is 32-bit, should be safe on ESP32)
    latest_gforce = gforce;
    last_accel_update_time = esp_timer_get_time();
    
    int64_t t1 = esp_timer_get_time();
    prof_record(PROF_I2C, (uint32_t)(t1 - t0));
    vTaskDelay(pdMS_TO_TICKS(1));  // Read every 1ms = 1kHz (SPI is fast enough)
  }
}

//-------ESC config (Qt Py ESP32-S3)--------
// DShot 600 @ 6 kHz send rate on Core 1 only (no main-loop jitter)
#define escR_gpio GPIO_NUM_18   // right ESC (left side pin 1)
#define escL_gpio GPIO_NUM_17   // left ESC (left side pin 2)
DShotESC escR;
DShotESC escL;

//---------Pin assignments (Adafruit QT Py ESP32-S3 - your board)----------
// Left side:  18, 17, 9, 8, 7 (SDA), 6 (SCL), 5 (TX)
// Right side: 35 (MOSI), 37 (MISO), 36 (SCK), 16 (RX)
//
// ASSIGNMENTS:
// GPIO 18:  escR (right ESC)
// GPIO 17:  escL (left ESC)
// GPIO 5:   CRSF_TX (UART TX)
// GPIO 16:  CRSF_RX (UART RX)
// GPIO 36:  ACCEL SCK  -> LIS331 SCL
// GPIO 35:  ACCEL MOSI -> LIS331 SDA
// GPIO 37:  ACCEL MISO -> LIS331 SDO
// GPIO 8:   ACCEL CS   -> LIS331 CS
// GPIO 9:   top_led_mosi (DotStar data) - top strip only
// GPIO 7:   top_led_sck (DotStar clock)
//
const int top_led_mosi = 9;
const int top_led_sck = 7;

//------LED definitions (top strip only)--------
const int NUMPIXELST = 10; // number of LEDs in the top strip
const int animSpeed = 100;   // ms between LED updates when in animation mode
Adafruit_DotStar top_strip(NUMPIXELST, top_led_mosi, top_led_sck, DOTSTAR_BGR);

//------Receiver config (CRSF) - Qt Py ESP32-S3: 5=TX, 16=RX --------
#define CRSF_RX_PIN 16   // ESP32 RX <- Receiver TX (right side)
#define CRSF_TX_PIN 5    // ESP32 TX -> Receiver RX (left side)
CRSFforArduino crsf = CRSFforArduino(&Serial1, CRSF_RX_PIN, CRSF_TX_PIN);
const int NUM_CHANNELS = 8; //number of reciever channels to use
static uint32_t lastCrsfPacket = 0;  // For failsafe detection

// Non-blocking CRSF using FreeRTOS task (declared after NUM_CHANNELS)
static volatile uint16_t crsf_channels[NUM_CHANNELS + 1] = {0};  // Shared channel data (indexed 1-NUM_CHANNELS)
static volatile unsigned long last_crsf_update = 0;  // Timestamp of last CRSF update (microseconds)
static TaskHandle_t crsf_task_handle = NULL;
static bool crsf_task_initialized = false;

// Non-blocking LED using FreeRTOS task
static volatile int led_angle_shared = 0;  // Current angle (0-359)
static volatile int led_rpm_shared = 0;   // Current RPM
static char led_status_shared[32] = "armed";  // LED status string (char array for thread safety)
static volatile bool led_data_changed = false;  // Flag when status changes
// Motor-on LED: show at heading+180°, green (accel) to red (decel), brightness by magnitude
static volatile bool led_motor_on_shared = false;
static volatile float led_pulse_position_shared = 0.0f;  // 0 = accel, 1 = decel
static volatile int led_transpeed_shared = 0;  // 0-100 for brightness
static TaskHandle_t led_task_handle = NULL;
static bool led_task_initialized = false;
static SemaphoreHandle_t led_status_mutex = NULL;  // Protects led_status_shared read/write

//------Driving characteristcs------
// Unified "floor diagram" angles: 0° = forward (direction robot moves). rotation_angle() should be
// calibrated so 0° = forward; then heading LED and motor pulse LED both use this same reference.
const int LEDheading = 0;
const int LED_STRIP_OFFSET_DEG = 30;  // LED strip start is this many degrees clockwise from forward (0°)
const int LED_HEADING_HALF_DEG = 10;  // Heading LED on within ±this of 0°
const int LED_MOTOR_ARC_HALF_DEG = 27;  // Motor-on LED arc half-width (~15% of 360°)
const int percentdecel = 15; //percentage of rotation the translation deceleration wave occurs for each motor. Should be <= 50
const float acc_rate = 0.4; //0.2(least aggressive spinup) - 1.0(most aggressive spinup)

//-------Wifi config---------
const char *ssid = "Beyblade"; //wifi ssid
const char *password = "meltybrain"; //wifi password
// Start TelnetStream: Enter telnet {insert ip address} for me: telnet 192.168.4.1 in CMD to view output

//-----other constants-------
const int denom = round(1.0 / sqrt(0.00001118*accradius)); //calculates denominator ahead of time to reduce unnecessary calculations;
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
// DShot task on Core 1 reads these; main loop (Core 0) writes. 32-bit aligned = atomic.
volatile int motorL;
volatile int motorR;
int spinspeed;
static TaskHandle_t dshot_task_handle = NULL;
#define DSHOT_SEND_HZ       6000   // 6 kHz
#define DSHOT_PERIOD_US      (1000000 / DSHOT_SEND_HZ)
//--------reciever----------
unsigned long chanstart = 0; //stores last RX read time
int pwm[NUM_CHANNELS + 1]; //list values from 1000-2000 (indexed 1-6)
int duty[NUM_CHANNELS + 1]; //list values from 0 - 100 (indexed 1-6)
int last_rec; //used to store update times for detecting signal loss
int rec_gap;
int rec_last;
bool rc_status = false; //whether getting rc signal, used for triggering failsafe
//----------other----------
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
void update_motors();   // no-op; DShot task on Core 1 does all ESC output
void spin();
void translate();
void dshot_task(void *pvParameters);  // runs on Core 1 at DSHOT_SEND_HZ

// RC_Handler.ino
void update_channels();
void crsf_read_task(void *pvParameters);

// Wifi_Handler.ino
void wifi_mode();
void data_export();

// Main functions
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
  unsigned long long currentime = esp_timer_get_time();
  long long spintime = spintimefunct();
  if (spintime <= 0) spintime = 1000;
  // First time or after long pause: avoid huge delta and overflow in previoustime update
  if (previoustime == 0) {
    previoustime = currentime;
    angle = 0;
    return 0;
  }
  unsigned long long delta = currentime - previoustime;
  angle = (int)((double)delta / (double)spintime * 360.0);
  // Reset revolution start when we've passed one full period (no signed overflow)
  if (delta >= (unsigned long long)spintime) {
    previoustime = currentime - (delta % (unsigned long long)spintime);
  }
  return (int)angle;
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
  if(offset < 0.7 or offset > 1.3)
  {
    offset = 1.0;
  }
  if(duty[4] < 30 && off_set == false)
  {
    offset = offset - 0.004;
    EEPROM.put(OFFSET_ADDR, offset);
    EEPROM.commit();  // Required on ESP32 to finalize the write
    off_set = true;
    Serial.print("[cal] offset -0.004 -> ");
    Serial.println(offset, 4);
    yield();  // Let other tasks run after blocking commit
  }
  else if(duty[4] > 70 && off_set == false)
  {
    offset = offset + 0.004;
    EEPROM.put(OFFSET_ADDR, offset);
    EEPROM.commit();
    off_set = true;
    Serial.print("[cal] offset +0.004 -> ");
    Serial.println(offset, 4);
    yield();
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
  // offset should be 0.8-1.2 for spintime tuning
  if (isnan(val) || val < 0.5f || val > 2.0f) {
    return 1.0f;
  }
  return val;
}

//=============LED CONTROL FUNCTIONS==================
// FreeRTOS task to update LEDs in background (non-blocking, angle-based with 5° precision)
void led_update_task(void *pvParameters) {
  Serial.println("  [LED Task] Started");
  
  static int last_displayed_angle = -999;  // Track last displayed angle
  static char last_processed_status[32] = "";  // Fixed buffer, no heap (was String)
  static unsigned long last_anim_update = 0;  // For animation timing
  static unsigned long last_armed_update = 0;  // For armed status animation timing
  static int anim_index = 0;  // Animation index (local to task)
  static int anim_direction = 1;  // Animation direction
  
  const int ANGLE_PRECISION = 5;  // 5° precision (72 updates per rotation)
  static float last_pulse_position = -1.0f;

  while (1) {
    int64_t t0 = esp_timer_get_time();
    // Read shared variables (atomic for ints; mutex for status string to avoid torn read)
    int current_angle = led_angle_shared;
    int current_rpm = led_rpm_shared;
    bool current_motor_on = led_motor_on_shared;
    float current_pulse_pos = led_pulse_position_shared;
    int current_transpeed = led_transpeed_shared;
    char current_status[32];
    if (led_status_mutex != NULL && xSemaphoreTake(led_status_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      strncpy(current_status, led_status_shared, sizeof(current_status) - 1);
      current_status[sizeof(current_status) - 1] = '\0';
      xSemaphoreGive(led_status_mutex);
    } else {
      current_status[0] = '\0';
    }
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
    if (status_changed || strcmp(current_status, last_processed_status) != 0) {
      need_update = true;
      led_data_changed = false;  // Clear flag
      strncpy(last_processed_status, current_status, sizeof(last_processed_status) - 1);
      last_processed_status[sizeof(last_processed_status) - 1] = '\0';
    }
    // Update when motor-on gradient changes (pulse position or transpeed)
    if (current_motor_on && (fabsf(current_pulse_pos - last_pulse_position) > 0.05f)) {
      need_update = true;
      last_pulse_position = current_pulse_pos;
    }
    if (!current_motor_on) last_pulse_position = -1.0f;

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
          // yellow bounce chase (top strip only)
          top_strip.clear();
          top_strip.setPixelColor(anim_index, 100, 80, 0);
        }
        else
        {
          // breathing blue effect
          float phase = (millis() % 2000) / 2000.0 * 2 * PI;
          int brightness = (sin(phase) * 50) + 50;
          for (int i = 0; i < NUMPIXELST; i++) {
            top_strip.setPixelColor(i, 0, 0, brightness);
          }
        }
      }
      else if (strcmp(current_status, "export") == 0)
      {
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 100, 100, 100);
        }
      }
      else if (strcmp(current_status, "armed") == 0)
      {
        top_strip.clear();
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 100, 0, 0);
        }
      }
      else if (strcmp(current_status, "heading on") == 0) 
      {
        // Light only the LED(s) at floor 0° (forward): direction the robot will travel. Same reference as motor.
        top_strip.clear();
        float heading_center = (float)LEDheading;
        for (int i = 0; i < NUMPIXELST; i++) {
          float led_floor_deg = fmodf((float)(i * 36 + LED_STRIP_OFFSET_DEG) + 360.0f, 360.0f);
          float diff = fabsf(fmodf(led_floor_deg - heading_center + 540.0f, 360.0f) - 180.0f);
          if (diff <= (float)LED_HEADING_HALF_DEG) {
            top_strip.setPixelColor(i, 255, 80, 0);  // Fiery orange at forward
          }
        }
      }
      else if (strcmp(current_status, "motor on") == 0) 
      {
        // Light only the LED(s) at current rotation angle (floor diagram): motor pulses here, so indicator here.
        // Green = accelerating, red = decelerating; brightness by transpeed. Same 0° = forward reference.
        top_strip.clear();
        float pulse_center = (float)(current_angle % 360);
        if (pulse_center < 0) pulse_center += 360.0f;
        float pos = current_pulse_pos;
        if (pos < 0.0f) pos = 0.0f;
        if (pos > 1.0f) pos = 1.0f;
        int r = (int)(255.0f * pos);
        int g = (int)(255.0f * (1.0f - pos));
        int brightness = (180 * current_transpeed) / 100;
        if (brightness > 180) brightness = 180;
        r = (r * brightness) / 255;
        g = (g * brightness) / 255;
        for (int i = 0; i < NUMPIXELST; i++) {
          float led_floor_deg = fmodf((float)(i * 36 + LED_STRIP_OFFSET_DEG) + 360.0f, 360.0f);
          float diff = fabsf(fmodf(led_floor_deg - pulse_center + 540.0f, 360.0f) - 180.0f);
          if (diff <= (float)LED_MOTOR_ARC_HALF_DEG) {
            top_strip.setPixelColor(i, r, g, 0);
          }
        }
      }
      else if (strcmp(current_status, "reading") == 0)
      {
        for (int i = 0; i < NUMPIXELST; i++) {
          top_strip.setPixelColor(i, 0, 0, 255);
        }
      }
      else 
      {
        top_strip.clear();
      }
      
      top_strip.show();
      
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
  static char last_status[32] = "";
  
  led_angle_shared = angle;
  led_rpm_shared = rpm;
  
  if (strcmp(LEDStatus.c_str(), last_status) != 0) {
    led_data_changed = true;
    strncpy(last_status, LEDStatus.c_str(), sizeof(last_status) - 1);
    last_status[sizeof(last_status) - 1] = '\0';
  }
  if (led_status_mutex != NULL && xSemaphoreTake(led_status_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    strncpy(led_status_shared, LEDStatus.c_str(), sizeof(led_status_shared) - 1);
    led_status_shared[sizeof(led_status_shared) - 1] = '\0';
    xSemaphoreGive(led_status_mutex);
  }
  
  // LED task handles all the actual LED updates in background
  // This function now returns immediately without blocking
}

//=============MOTOR CONTROL FUNCTIONS==================
// DShot is sent by dshot_task on Core 1 at 6 kHz. Main loop only sets motorL/motorR.
void update_motors()
{
  // Safety clamp in main thread (DShot task also clamps when sending)
  int l = (int)motorL, r = (int)motorR;
  if (l > 999) motorL = 999;
  if (l < -999) motorL = -999;
  if (r > 999) motorR = 999;
  if (r < -999) motorR = -999;
}

// Runs on Core 1 only at DSHOT_SEND_HZ for steady ESC timing (no main-loop jitter).
void dshot_task(void *pvParameters)
{
  Serial.println("  [DShot Task] Started on Core 1");
  // Let setup() on Core 0 finish (watchdog, Serial) before we start sending
  vTaskDelay(pdMS_TO_TICKS(200));
  int64_t next_wake_us = esp_timer_get_time();
  for (;;) {
    int64_t now_us = esp_timer_get_time();
    while (now_us < next_wake_us) {
      int64_t delay_us = next_wake_us - now_us;
      if (delay_us > 1000)
        vTaskDelay(pdMS_TO_TICKS(1));
      else if (delay_us > 0)
        delayMicroseconds((unsigned)delay_us);
      now_us = esp_timer_get_time();
    }
    next_wake_us += DSHOT_PERIOD_US;
    if (next_wake_us <= now_us)
      next_wake_us = now_us + DSHOT_PERIOD_US;

    int64_t t0 = esp_timer_get_time();
    int l = (int)motorL, r = (int)motorR;
    if (l > 999) l = 999;
    if (l < -999) l = -999;
    if (r > 999) r = 999;
    if (r < -999) r = -999;

    escR.sendThrottle3D(r);
    delayMicroseconds(25);
    escL.sendThrottle3D(l);

    int64_t t1 = esp_timer_get_time();
    prof_record(PROF_ESC, (uint32_t)(t1 - t0));

    // Yield every iteration so main loop gets CPU and can feed watchdog (was every 500 iters -> loop starved -> TASK_WDT)
    taskYIELD();
  }
}

void spin()
{
  motor_on = false;
  led_motor_on_shared = false;

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
      led_motor_on_shared = true;
      led_pulse_position_shared = (duration > 0) ? ((float)dtime / (float)duration) : 0.0f;
      led_transpeed_shared = transpeed;
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
      led_motor_on_shared = true;
      led_pulse_position_shared = (duration > 0) ? ((float)dtime / (float)duration) : 0.0f;
      led_transpeed_shared = transpeed;
    }
    else
    {
      spin();
    }
  }
}

//=============RC HANDLER FUNCTIONS==================
// FreeRTOS task to read CRSF channels in background (non-blocking)
void crsf_read_task(void *pvParameters) {
  Serial.println("  [CRSF Task] Started");
  
  while (1) {
    int64_t t0 = esp_timer_get_time();
    // Update CRSF library (reads serial data)
    crsf.update();
    
    // Read all channels and store in shared volatile array
    for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
      uint16_t crsfVal = crsf.rcToUs(crsf.getChannel(channel));
      crsf_channels[channel] = crsfVal;
    }
    
    // Update timestamp (atomic write - unsigned long is 32-bit, safe on ESP32)
    last_crsf_update = esp_timer_get_time();
    
    prof_record(PROF_CRSF, (uint32_t)(esp_timer_get_time() - t0));
    
    // Delay 5ms (200Hz update rate - sufficient for human reaction time)
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void update_channels()
{
  // Fast data copy from shared volatile variables (non-blocking)
  // CRSF reading happens in background task every 5ms
  
  // Copy channel data from background task (atomic reads)
  bool gotNewData = false;
  for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
    uint16_t crsfVal = crsf_channels[channel];  // Atomic read
    if (crsfVal > 0) {
      gotNewData = true;
    }
    pwm[channel] = crsfVal;
    duty[channel] = map(crsfVal, 1000, 2000, 0, 100);
  }
  
  // Failsafe detection - check if we're receiving valid data
  unsigned long now_us = esp_timer_get_time();
  unsigned long last_update_us = last_crsf_update;  // Atomic read
  
  if (gotNewData && pwm[1] > 900) {
    rc_status = true;
    lastCrsfPacket = now_us / 1000;  // Convert to milliseconds for compatibility
  } else {
    // Check timeout (500ms = 500000 microseconds)
    if (last_update_us > 0 && (now_us - last_update_us) > 500000) {
      duty[1] = 50;
      duty[2] = 50;
      duty[3] = 0;
      duty[4] = 50;
      duty[5] = 100;
      duty[6] = 0;
      rc_status = false;
    }
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
    
    // Print basics periodically while in wifi mode (L, R, G, CH3)
    if (millis() - lastWifiPrint > 200) {
      lastWifiPrint = millis();
      Serial.print("L=");
      Serial.print(motorL);
      Serial.print(" R=");
      Serial.print(motorR);
      Serial.print(" G=");
      Serial.print(last_gforce_raw, 2);
      Serial.print(" CH3=");
      Serial.print(duty[3]);
      Serial.println(" [wifi]");
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
  WiFi.softAPdisconnect(false);
  WiFi.mode(WIFI_OFF);
  delay(500);
  // Re-arm ESCs: suspend DShot task so we own RMT, then send 0 repeatedly
  if (dshot_task_handle) vTaskSuspend(dshot_task_handle);
  for (int i = 0; i < 2500; i++) {
    escR.writeData(0, true);
    delayMicroseconds(400);
    escL.writeData(0, true);
    delayMicroseconds(400);
  }
  motorL = 0;
  motorR = 0;
  if (dshot_task_handle) vTaskResume(dshot_task_handle);
  esp_task_wdt_init(WDT_TIMEOUT, true);
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
  xTaskCreatePinnedToCore(
    accel_spi_read_task,
    "Accel_SPI",
    2048,
    NULL,
    1,
    &accel_task_handle,
    0
  );
  accel_initialized = true;
  Serial.println("  Background accel SPI task started");
  
  Serial.println("Starting CRSF receiver...");
  Serial.print("  CRSF RX Pin: GPIO ");
  Serial.println(CRSF_RX_PIN);
  Serial.print("  CRSF TX Pin: GPIO ");
  Serial.println(CRSF_TX_PIN);
  
  // Let CRSF library handle Serial1 configuration
  if (!crsf.begin()) {
    Serial.println("ERROR: CRSF initialization failed!");
  } else {
    Serial.println("CRSF initialized successfully.");
    xTaskCreatePinnedToCore(
      crsf_read_task,
      "CRSF_Read",
      2048,
      NULL,
      1,
      &crsf_task_handle,
      0
    );
    crsf_task_initialized = true;
    Serial.println("  Background CRSF read task started (non-blocking, 200Hz)");
  }
  
  Serial.println("Starting LED strip (DotStar, top only)...");
  top_strip.begin();
  top_strip.setBrightness(85);  // 1/3 brightness for testing (255/3 ≈ 85)
  top_strip.clear();
  top_strip.show();
  
  // Initialize shared LED variables and mutex for thread-safe status string
  led_angle_shared = 0;
  led_rpm_shared = 0;
  strncpy(led_status_shared, "export", sizeof(led_status_shared) - 1);
  led_status_shared[sizeof(led_status_shared) - 1] = '\0';
  led_data_changed = true;
  if (led_status_mutex == NULL)
    led_status_mutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(
    led_update_task,
    "LED_Update",
    4096,
    NULL,
    1,
    &led_task_handle,
    0
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
    if (i % 500 == 0) {
      Serial.print("  arm ");
      Serial.println(i);
    }
    escR.writeData(0, true);
    delayMicroseconds(400);
    escL.writeData(0, true);
    delayMicroseconds(400);
  }
  Serial.println("  Arming loop done.");
  escR.setReversed(false);
  escR.set3DMode(true);
  delayMicroseconds(200);
  escL.setReversed(false);
  escL.set3DMode(true);
  Serial.println("  setReversed/set3D done.");

  // DShot on Core 1 so main loop on Core 0 can run and feed watchdog (fixes TASK_WDT crash)
  xTaskCreatePinnedToCore(
    dshot_task,
    "DShot",
    8192,
    NULL,
    1,
    &dshot_task_handle,
    1     // Core 1
  );
  Serial.println("  DShot task started on Core 1 (6 kHz)");

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
  esp_task_wdt_reset();  // Feed watchdog at start so one slow iteration (e.g. Serial block) doesn't trigger reboot
  unsigned long long t0 = esp_timer_get_time();
  
  update_channels();
  heading_funct();
  heading_adj();
  calcrpm();
  
  if (!rc_status) {
    failsafe();
    unsigned long long t1 = esp_timer_get_time();
    prof_record(PROF_LOOP, (uint32_t)(t1 - t0));
    prof_report();  // Print task CPU-time summary every 1s (throttled inside)
    // Debug (5Hz) in failsafe
    static unsigned long long last_failsafe_debug_us = 0;
    if ((t1 - last_failsafe_debug_us) >= DEBUG_PRINT_INTERVAL_US) {
      last_failsafe_debug_us = t1;
      Serial.print("L=");
      Serial.print(motorL);
      Serial.print(" R=");
      Serial.print(motorR);
      Serial.print(" G=");
      Serial.print(last_gforce_raw, 2);
      Serial.print(" rpm=");
      Serial.print(rpm);
      Serial.print(" ang=");
      Serial.print(angle);
      Serial.print(" off=");
      Serial.print(offset, 3);
      Serial.print(" CH4=");
      Serial.print(duty[4]);
      Serial.print(" CH3=");
      Serial.print(duty[3]);
      Serial.println(" [failsafe]");
    }
    esp_task_wdt_reset();
    // Yield so CRSF task can run
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
    if (isAngleInRange(LEDheading, LED_HEADING_HALF_DEG))  // LED on when pointing at floor 0° (forward)
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
  prof_report();  // Print task CPU-time summary every 1s (throttled inside)

  // Debug at 5Hz: L/R throttle, G force, CH3
  static unsigned long long last_debug_print_us = 0;
  unsigned long long now_us = esp_timer_get_time();
  if ((now_us - last_debug_print_us) >= DEBUG_PRINT_INTERVAL_US) {
    last_debug_print_us = now_us;
    Serial.print("L=");
    Serial.print(motorL);
    Serial.print(" R=");
    Serial.print(motorR);
    Serial.print(" G=");
    Serial.print(last_gforce_raw, 2);
    Serial.print(" rpm=");
    Serial.print(rpm);
    Serial.print(" ang=");
    Serial.print(angle);
    Serial.print(" off=");
    Serial.print(offset, 3);
    Serial.print(" CH4=");
    Serial.print(duty[4]);
    Serial.print(" rst=");
    Serial.print(off_set ? "1" : "0");
    Serial.print(" CH3=");
    Serial.println(duty[3]);
  }
}
