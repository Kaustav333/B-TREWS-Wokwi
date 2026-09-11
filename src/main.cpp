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
#define PIN_BMP_SDA    4    // BMP280 I2C SDA
#define PIN_BMP_SCL    5    // BMP280 I2C SCL
#define PIN_VOC_ADC    7    // MQ-135 VOC Gas Sensor (ADC)
#define PIN_NTC_ADC    8    // NTC Busbar Temp Probe (ADC)
#define PIN_LED_GREEN  13   // State 0: Normal
#define PIN_LED_YELLOW 10   // State 1: Caution
#define PIN_LED_AMBER  14   // State 2: Warning
#define PIN_LED_RED    21   // State 3: Critical Trip
#define PIN_BUZZER     47   // Emergency Alarm Buzzer
#define PIN_RELAY      48   // 10A Solid State Safety Relay

// --- B-TREWS Code-ready constants (from COMSOL mapping) ---
const float DTDT_NORMAL_REF = 0.041f;    // C/s, thermal Case 1
const float DTDT_STRONG_REF = 0.408f;    // C/s, thermal Case 2
const float DTDT_CRITICAL   = 1.20f;     // C/s, project candidate trigger

const float DPDT_NORMAL_REF  = 0.041317f; // hPa/s, 0.1 m/s sensitivity case
const float DPDT_CAUTION_REF = 0.19852f;  // hPa/s, 0.25 m/s sensitivity case
const float DPDT_WARNING_REF = 0.69313f;  // hPa/s, final optimized COMSOL case
const float DPDT_CRITICAL    = 2.50f;     // hPa/s, project candidate trigger

const float VOC_ARRIVAL        = 0.05f;   // mol/m^3, 5% normalized CFD criterion
const float VOC_FINAL_PEAK_REF = 1.0015f; // mol/m^3, final COMSOL peak

// --- System State Machine ---
enum SystemState {
  STATE_NORMAL   = 0,
  STATE_CAUTION  = 1,
  STATE_WARNING  = 2,
  STATE_CRITICAL = 3
};

SystemState currentState = STATE_NORMAL;
Adafruit_BMP280 bmp;

// --- COMSOL Multiphysics Time-Series Dataset ---
struct COMSOL_Point {
  float time_sec;
  float pressure_hPa;
  float busbarTemp_C;
  float voc_mol_m3;
};

// Representative Wokwi test vectors using project values (Expanded for 10s per state)
COMSOL_Point comsol_profile[] = {
  // NORMAL (dPdt = ~0.04, dTdt = ~0.04)
  { 0.0,  1013.250f, 20.000f, 0.00f },
  { 5.0,  1013.450f, 20.200f, 0.00f },
  { 10.0, 1013.650f, 20.400f, 0.00f },
  
  // CAUTION (dPdt = 0.20, dTdt = 0.20, VOC = 0.05)
  { 15.0, 1014.650f, 21.400f, 0.05f },
  { 20.0, 1015.650f, 22.400f, 0.05f },
  
  // WARNING (dPdt = 0.70, dTdt = 0.41, VOC = 0.10)
  { 25.0, 1019.150f, 24.450f, 0.10f },
  { 30.0, 1022.650f, 26.500f, 0.10f },
  
  // CRITICAL TRIP (dPdt = 2.60, dTdt = 1.30, VOC = 1.00)
  { 35.0, 1035.650f, 33.000f, 1.00f },
  { 40.0, 1048.650f, 39.500f, 1.00f }
};
const int comsol_length = sizeof(comsol_profile) / sizeof(comsol_profile[0]);

int   stepIndex = 0;
float prevP     = 1013.25;
float prevT     = 20.0;

// ==============================================================================
void setup() {
  Serial.begin(115200);

  // Configure all output pins
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_AMBER,  OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);
  pinMode(PIN_RELAY,      OUTPUT);

  // Force NORMAL state on boot
  digitalWrite(PIN_LED_GREEN,  HIGH);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_AMBER,  LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_BUZZER,     LOW);
  digitalWrite(PIN_RELAY,      HIGH);  // Relay closed = Load lamp ON

  // Initialize BMP280 pressure/temperature sensor
  Wire.begin(PIN_BMP_SDA, PIN_BMP_SCL);
  bmp.begin(0x76);

  Serial.println("==========================================================");
  Serial.println(" B-TREWS ESP32-S3 Thermal Runaway Early-Warning System    ");
  Serial.println(" COMSOL HIL Simulation Engine Initialized                 ");
  Serial.println("==========================================================");
  Serial.println();

  delay(5000);
}

// ==============================================================================
void loop() {
  float rawP    = 1013.25;
  float busbarT = 20.0;
  float voc     = 0.0;
  float simTime = 0.0;

  // --- Read sensor data ---
  if (USE_COMSOL_PLAYBACK) {
    rawP    = comsol_profile[stepIndex].pressure_hPa;
    busbarT = comsol_profile[stepIndex].busbarTemp_C;
    voc     = comsol_profile[stepIndex].voc_mol_m3;
    simTime = comsol_profile[stepIndex].time_sec;
  } else {
    rawP = bmp.readPressure() / 100.0F; 
    if (isnan(rawP) || rawP == 0) rawP = 1013.25;

    // Map ADC to mol/m^3 (0 to 1.5)
    voc = (analogRead(PIN_VOC_ADC) / 4095.0f) * 1.5f;
    uint16_t ntcADC = analogRead(PIN_NTC_ADC);
    busbarT = 20.0f + ((float)ntcADC / 4095.0f) * 65.0f; 
    simTime = millis() / 1000.0f;
  }

  // --- Calculate first derivatives ---
  float dt   = 5.0f;  
  float dPdt = (rawP - prevP) / dt;
  float dTdt = (busbarT - prevT) / dt;
  
  if (stepIndex == 0) {
    dPdt = 0;
    dTdt = 0;
  }
  
  prevP = rawP;
  prevT = busbarT;

  // --- State Escalation Decision Engine (Project-Matched Logic) ---
  if (dTdt > DTDT_CRITICAL || dPdt >= DPDT_CRITICAL) {
    currentState = STATE_CRITICAL;
  } else if (dTdt >= DTDT_STRONG_REF || dPdt >= DPDT_WARNING_REF) {
    if (currentState != STATE_CRITICAL) {
      currentState = STATE_WARNING;
    }
  } else if (dTdt > DTDT_NORMAL_REF || dPdt > DPDT_NORMAL_REF || voc >= VOC_ARRIVAL) {
    if (currentState != STATE_CRITICAL && currentState != STATE_WARNING) {
      currentState = STATE_CAUTION;
    }
  } else {
    if (currentState != STATE_CRITICAL && currentState != STATE_WARNING && currentState != STATE_CAUTION) {
      currentState = STATE_NORMAL;
    }
  }

  // --- Actuate outputs based on state ---
  switch (currentState) {
    case STATE_NORMAL:
      digitalWrite(PIN_RELAY,      HIGH);
      digitalWrite(PIN_LED_GREEN,  HIGH);
      digitalWrite(PIN_LED_YELLOW, LOW);
      digitalWrite(PIN_LED_AMBER,  LOW);
      digitalWrite(PIN_LED_RED,    LOW);
      digitalWrite(PIN_BUZZER,     LOW);
      Serial.printf("[t=%5.0fs] STATE 0 NORMAL   | dP/dt: %6.3f hPa/s | dT/dt: %5.3f C/s | VOC: %.2f mol/m^3\n", simTime, dPdt, dTdt, voc);
      break;

    case STATE_CAUTION:
      digitalWrite(PIN_RELAY,      HIGH);
      digitalWrite(PIN_LED_GREEN,  LOW);
      digitalWrite(PIN_LED_YELLOW, HIGH);
      digitalWrite(PIN_LED_AMBER,  LOW);
      digitalWrite(PIN_LED_RED,    LOW);
      digitalWrite(PIN_BUZZER,     LOW);
      Serial.printf("[t=%5.0fs] STATE 1 CAUTION  | dP/dt: %6.3f hPa/s | dT/dt: %5.3f C/s | VOC: %.2f mol/m^3\n", simTime, dPdt, dTdt, voc);
      break;

    case STATE_WARNING:
      digitalWrite(PIN_RELAY,      HIGH);
      digitalWrite(PIN_LED_GREEN,  LOW);
      digitalWrite(PIN_LED_YELLOW, LOW);
      digitalWrite(PIN_LED_AMBER,  HIGH);
      digitalWrite(PIN_LED_RED,    LOW);
      digitalWrite(PIN_BUZZER,     LOW);
      Serial.printf("[t=%5.0fs] STATE 2 WARNING  | dP/dt: %6.3f hPa/s | dT/dt: %5.3f C/s | VOC: %.2f mol/m^3\n", simTime, dPdt, dTdt, voc);
      break;

    case STATE_CRITICAL:
      digitalWrite(PIN_RELAY,      LOW); // OPEN Relay -> Disconnect Battery Load
      digitalWrite(PIN_LED_GREEN,  LOW);
      digitalWrite(PIN_LED_YELLOW, LOW);
      digitalWrite(PIN_LED_AMBER,  LOW);
      digitalWrite(PIN_LED_RED,    HIGH);
      digitalWrite(PIN_BUZZER,     HIGH);
      Serial.printf("[t=%5.0fs] STATE 3 CRITICAL | dP/dt: %6.3f hPa/s | dT/dt: %5.3f C/s | VOC: %.2f mol/m^3\n", simTime, dPdt, dTdt, voc);
      Serial.println(">>> BATTERY LOAD DISCONNECTED. SYSTEM HALTED FOR SAFETY. <<<");
      while(true) {
        // Halt completely. Disconnect itself.
        delay(1000);
      }
      break;
  }

  // Advance timeline
  if (stepIndex < comsol_length - 1) {
    stepIndex++;
  }

  delay(5000);
}
