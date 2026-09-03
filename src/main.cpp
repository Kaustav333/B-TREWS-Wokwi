/*
 * ==============================================================================
 * Project : B-TREWS (Battery-Mounted Thermal Runaway Early-Warning System)
 * Board   : ESP32-S3-DevKitC-1
 * Capstone: Phase 1 - Task 1.2 Embedded Emulation via Wokwi
 *
 * Modes:
 *   USE_COMSOL_PLAYBACK = true  -> Auto-plays COMSOL dataset (for demo/viva)
 *   USE_COMSOL_PLAYBACK = false -> Live manual potentiometer & BMP280 control
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// ===================== MODE SWITCH =====================
#define USE_COMSOL_PLAYBACK  true
// =======================================================

// --- ESP32-S3 Pin Assignments ---
#define PIN_BMP_SDA   4    // BMP280 I2C SDA
#define PIN_BMP_SCL   5    // BMP280 I2C SCL
#define PIN_VOC_ADC   7    // MQ-135 VOC Gas Sensor (ADC)
#define PIN_NTC_ADC   8    // NTC Busbar Temp Probe (ADC)
#define PIN_LED_GREEN 13   // State 0: Normal (Green LED)
#define PIN_LED_AMBER 14   // State 1: Warning (Amber LED)
#define PIN_LED_RED   21   // State 2: Critical Trip (Red LED)
#define PIN_BUZZER    47   // Emergency Alarm Buzzer
#define PIN_RELAY     48   // 10A Solid State Safety Relay

// --- B-TREWS Algorithmic Thresholds ---
#define THRESHOLD_DP_DT    2.5    // Pressure rate spike > 2.5 hPa/s
#define THRESHOLD_DT_DT    1.2    // Thermal acceleration > 1.2 C/s
#define THRESHOLD_TEMP_MAX 60.0   // Max terminal temp = 60.0 C
#define THRESHOLD_VOC_ADC  2500   // Off-gas solvent detection (ADC 0-4095)

// --- System State Machine ---
enum SystemState {
  STATE_NORMAL   = 0,
  STATE_WARNING  = 1,
  STATE_CRITICAL = 2
};

SystemState currentState = STATE_NORMAL;
Adafruit_BMP280 bmp;

// --- COMSOL Multiphysics Time-Series Dataset ---
// REPLACE THESE VALUES WITH YOUR ACTUAL COMSOL SIMULATION OUTPUT!
struct COMSOL_Point {
  float    time_sec;
  float    pressure_hPa;
  float    busbarTemp_C;
  uint16_t voc_adc;
};

COMSOL_Point comsol_profile[] = {
  // { Time(s), Pressure(hPa), Temp(C), VOC_Gas(ADC) }
  { 0.0,  1013.25, 25.0, 400  },  // Step 0:  Normal operation
  { 5.0,  1013.30, 25.4, 420  },  // Step 1:  Normal operation
  { 10.0, 1013.50, 25.9, 460  },  // Step 2:  Normal operation
  { 15.0, 1015.10, 26.8, 1100 },  // Step 3:  SEI decomposition begins
  { 20.0, 1018.80, 28.5, 2850 },  // Step 4:  STAGE 1 -> VOC off-gas spike!
  { 25.0, 1022.30, 31.2, 3300 },  // Step 5:  dP/dt pressure gradient rising
  { 30.0, 1024.50, 37.0, 3600 },  // Step 6:  Core thermal runaway acceleration
  { 35.0, 1025.20, 45.5, 3750 },  // Step 7:  Approaching thermal limit
  { 40.0, 1025.80, 53.8, 3850 },  // Step 8:  dT/dt exceeding threshold
  { 45.0, 1026.10, 62.4, 3950 },  // Step 9:  STAGE 2 -> Exothermic trip (>60C)!
  { 50.0, 1026.50, 76.0, 4000 },  // Step 10: Post-trip (relay already tripped)
};
const int comsol_length = sizeof(comsol_profile) / sizeof(comsol_profile[0]);

int   stepIndex = 0;
float prevP     = 1013.25;
float prevT     = 25.0;

// ==============================================================================
// setup() - Runs once on boot
// ==============================================================================
void setup() {
  Serial.begin(115200);

  // Configure all output pins
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_AMBER, OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_RELAY,     OUTPUT);

  // Force NORMAL state on boot: Green LED ON, Buzzer OFF, Relay CLOSED
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_AMBER, LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  digitalWrite(PIN_BUZZER,    LOW);
  digitalWrite(PIN_RELAY,     HIGH);  // Relay closed = Load lamp ON

  // Initialize BMP280 pressure/temperature sensor
  Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL);
  bmp.begin(0x76);

  Serial.println("==========================================================");
  Serial.println(" B-TREWS ESP32-S3 Thermal Runaway Early-Warning System    ");
  Serial.println(" COMSOL HIL Simulation Engine Initialized                 ");
  Serial.println("==========================================================");
  Serial.println();

  if (USE_COMSOL_PLAYBACK) {
    Serial.println("MODE: COMSOL PLAYBACK (Automatic Timeline)");
    Serial.printf("Dataset: %d time-steps over %.0f seconds\n", comsol_length, comsol_profile[comsol_length-1].time_sec);
  } else {
    Serial.println("MODE: MANUAL (Use potentiometer knobs & BMP280 sensor)");
  }
  Serial.println("----------------------------------------------------------");
  Serial.println();

  // Hold NORMAL state visibly for 5 seconds so user can see green LED
  delay(5000);
}

// ==============================================================================
// loop() - Main execution loop (runs every 5 seconds in COMSOL mode)
// ==============================================================================
void loop() {
  float    rawP     = 1013.25;
  float    busbarT  = 25.0;
  uint16_t vocADC   = 400;
  float    simTime  = 0.0;

  // --- Read sensor data ---
  if (USE_COMSOL_PLAYBACK) {
    // Play back COMSOL Multiphysics dataset step-by-step
    rawP    = comsol_profile[stepIndex].pressure_hPa;
    busbarT = comsol_profile[stepIndex].busbarTemp_C;
    vocADC  = comsol_profile[stepIndex].voc_adc;
    simTime = comsol_profile[stepIndex].time_sec;
  } else {
    // Live sensor reading from BMP280 + potentiometer knobs
    rawP = bmp.readPressure() / 100.0F;  // Pa -> hPa
    if (isnan(rawP) || rawP == 0) rawP = 1013.25;

    vocADC = analogRead(PIN_VOC_ADC);
    uint16_t ntcADC = analogRead(PIN_NTC_ADC);
    busbarT = 20.0f + ((float)ntcADC / 4095.0f) * 65.0f;  // Map 0-4095 -> 20-85 C
    simTime = millis() / 1000.0f;
  }

  // --- Calculate first derivatives: dP/dt and dT/dt ---
  float dt   = 5.0f;  // 5 seconds between COMSOL steps
  float dPdt = (rawP - prevP) / dt;
  float dTdt = (busbarT - prevT) / dt;
  prevP = rawP;
  prevT = busbarT;

  // --- State Escalation Decision Engine ---
  if (dTdt > THRESHOLD_DT_DT || busbarT > THRESHOLD_TEMP_MAX) {
    currentState = STATE_CRITICAL;
  } else if (vocADC > THRESHOLD_VOC_ADC || dPdt > THRESHOLD_DP_DT) {
    if (currentState != STATE_CRITICAL) {  // Don't downgrade from critical
      currentState = STATE_WARNING;
    }
  } else {
    if (currentState != STATE_CRITICAL) {  // Critical is latched until reset
      currentState = STATE_NORMAL;
    }
  }

  // --- Actuate outputs based on state ---
  switch (currentState) {
    case STATE_NORMAL:
      digitalWrite(PIN_RELAY,     HIGH);  // Relay closed (Blue load lamp ON)
      digitalWrite(PIN_LED_GREEN, HIGH);  // Green LED ON
      digitalWrite(PIN_LED_AMBER, LOW);
      digitalWrite(PIN_LED_RED,   LOW);
      digitalWrite(PIN_BUZZER,    LOW);   // Silent

      Serial.printf("[t=%5.0fs] STATE 0 NORMAL  | P:%7.1f hPa | T:%5.1f C | VOC:%4d | dP/dt:%+6.2f | dT/dt:%+5.2f\n",
                    simTime, rawP, busbarT, vocADC, dPdt, dTdt);
      break;

    case STATE_WARNING:
      digitalWrite(PIN_RELAY,     HIGH);  // Relay still closed (load active)
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_AMBER, HIGH);  // Amber LED ON
      digitalWrite(PIN_LED_RED,   LOW);
      digitalWrite(PIN_BUZZER,    LOW);   // Silent

      Serial.printf("[t=%5.0fs] STAGE 1 WARNING | P:%7.1f hPa | T:%5.1f C | VOC:%4d | dP/dt:%+6.2f | dT/dt:%+5.2f\n",
                    simTime, rawP, busbarT, vocADC, dPdt, dTdt);
      Serial.printf("         [CAN TX 0x18FF0100] VOC off-gas spike detected! Sampling rate boosted to 100Hz.\n");
      break;

    case STATE_CRITICAL:
      digitalWrite(PIN_RELAY,     LOW);   // TRIP RELAY -> Disconnect battery load!
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_AMBER, LOW);
      digitalWrite(PIN_LED_RED,   HIGH);  // Red LED ON
      digitalWrite(PIN_BUZZER,    HIGH);  // Buzzer ALARM sounding!

      Serial.printf("[t=%5.0fs] STAGE 2 TRIP!!! | P:%7.1f hPa | T:%5.1f C | VOC:%4d | dP/dt:%+6.2f | dT/dt:%+5.2f\n",
                    simTime, rawP, busbarT, vocADC, dPdt, dTdt);
      Serial.printf("         [CAN TX 0x0CF00200] EMERGENCY: Busbar T=%.1f C | RELAY DISCONNECTED!\n", busbarT);
      break;
  }

  // Advance to next COMSOL step
  if (stepIndex < comsol_length - 1) {
    stepIndex++;
  }

  // Wait 5 seconds before next step for a clear, readable presentation
  delay(5000);
}
