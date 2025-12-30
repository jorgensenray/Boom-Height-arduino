// https://github.com/jorgensenray

#include <Arduino.h>
#include <elapsedMillis.h>
#include <NewPing.h>
#include <EEPROM.h>
#include "Types.h"
#include <string.h>  // for memset


// Comms API (implemented in Comms.ino tab)
void Comm_Init();
void Comm_Poll();
void Comm_TelemetryTick(float gps_ips);
void Comm_PrintHelp();
void Comm_PrintParams();

// Tap test
bool g_tapTestActive = false;
bool g_tapValveIssued = false;
unsigned long g_tapStartMs = 0;

float g_tapBaseline = NAN;

// ======================================================
// PINS (RIGHT SIDE ONLY)
// ======================================================
const uint8_t PIN_TRIG_R = 2;
const uint8_t PIN_ECHO_R = 3;

const uint8_t PIN_VALVE_UP_R = 6;    // energize to raise boom
const uint8_t PIN_VALVE_DOWN_R = 7;  // energize to lower boom

// ======================================================
// SONAR
// ======================================================
NewPing sonarR(PIN_TRIG_R, PIN_ECHO_R, 120);  // max distance in inches (tight = faster timeout)

// ======================================================
// PARAMETERS (EEPROM-backed)
// ======================================================
Params P;
bool g_armed = false;  // DISARMED by default
uint8_t g_faultFlags = FAULT_NONE;
unsigned long g_lastCommsMs = 0;

// ======================================================
// TELEMETRY STATE
// ======================================================

RightState R;

elapsedMillis readTimer;

// From GPS parser later (MPH). For now you can set it from PC via SET gps_mph 5.5
float gps_mph = 5.0f;

elapsedMillis sensorInvalidTimer;

// ======================================================
// CRC32 (small, table-less)
// ======================================================
uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320UL;
    else crc = (crc >> 1);
  }
  return crc;
}

uint32_t crc32_compute(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) crc = crc32_update(crc, data[i]);
  return ~crc;
}

// ======================================================
// EEPROM PARAMS: defaults / load / save
// ======================================================
void setDefaults() {
  memset(&P, 0, sizeof(P));
  P.magic = PARAM_MAGIC;
  P.version = PARAM_VERSION;
  P.sizeBytes = sizeof(P);

  P.targetIn = 24.0f;
  P.deadbandIn = 1.0f;
  P.sensorLeadIn = 24.0f;
  P.hydraulicDelaySec = 0.30f;  // good starting guess for "slow-ish" valves

  P.medianSamples = 3;
  P.validMinIn = 6.0f;
  P.validMaxIn = 60.0f;

  P.minValveOnMs = 250;
  P.minValveOffMs = 200;

  P.pulseMinMs = 140;
  P.pulseMaxMs = 400;
  P.errForMaxPulseIn = 8.0f;

  P.commMode = COMM_USB;

  // crc filled on save
}

void saveParamsToEEPROM() {
  P.magic = PARAM_MAGIC;
  P.version = PARAM_VERSION;
  P.sizeBytes = sizeof(P);

  // compute CRC over everything except crc field itself
  P.crc32 = 0;
  uint32_t crc = crc32_compute((uint8_t*)&P, sizeof(P));
  P.crc32 = crc;

  EEPROM.put(0, P);
}

bool loadParamsFromEEPROM() {
  Params tmp;
  EEPROM.get(0, tmp);

  if (tmp.magic != PARAM_MAGIC) return false;
  if (tmp.version != PARAM_VERSION) return false;
  if (tmp.sizeBytes != sizeof(Params)) return false;

  uint32_t storedCrc = tmp.crc32;
  tmp.crc32 = 0;
  uint32_t calc = crc32_compute((uint8_t*)&tmp, sizeof(tmp));
  if (calc != storedCrc) return false;

  // basic sanity clamps
  if (tmp.medianSamples < 1) tmp.medianSamples = 1;
  if (tmp.medianSamples > 9) tmp.medianSamples = 9;
  if (tmp.validMinIn < 1) tmp.validMinIn = 1;
  if (tmp.validMaxIn < tmp.validMinIn + 5) tmp.validMaxIn = tmp.validMinIn + 5;

  P = tmp;
  return true;
}

// ======================================================
// VALVE CONTROL (RIGHT)
// ======================================================
void valvesOffRight() {
  digitalWrite(PIN_VALVE_UP_R, LOW);
  digitalWrite(PIN_VALVE_DOWN_R, LOW);
  R.cmd = 0;
  R.valveTimer = 0;
}

void commandRight(int8_t dir) {
  if (dir == R.cmd) return;

  // enforce minimum ON time before switching away from active
  if (R.cmd != 0 && R.valveTimer < P.minValveOnMs) return;

  // enforce minimum OFF time before turning on from OFF
  if (R.cmd == 0 && R.valveTimer < P.minValveOffMs) return;

  // never energize both
  if (dir > 0) {  // UP
    digitalWrite(PIN_VALVE_DOWN_R, LOW);
    digitalWrite(PIN_VALVE_UP_R, HIGH);
  } else {  // DOWN
    digitalWrite(PIN_VALVE_UP_R, LOW);
    digitalWrite(PIN_VALVE_DOWN_R, HIGH);
  }

  R.cmd = dir;
  R.valveTimer = 0;
}

// ======================================================
// SENSOR (fractional inches from uS)
// ======================================================
static inline float uS_to_inches_float(unsigned int uS) {
  // ~0.0135039 inches/us one-way @ ~20C, round-trip => *0.5
  return (uS * 0.0135039f) * 0.5f;
}

float readRightInchesFiltered() {
  unsigned int uS = sonarR.ping_median(P.medianSamples);
  if (uS == 0) return NAN;

  float inches = uS_to_inches_float(uS);
  if (!(inches >= P.validMinIn && inches <= P.validMaxIn)) return NAN;

  return inches;
}

// ======================================================
// PREDICTION: physical lead + hydraulic delay compensation
// ======================================================
void updateRight(float dtSec, float gps_ips) {
  float x = readRightInchesFiltered();
  if (isnan(x)) {
    R.predicted = NAN;
    return;
  }

  R.filtPrev = R.filtNow;
  R.filtNow = x;

  // effective lead distance includes actuator delay
  // leadDistance = sensorLead + speed * hydraulicDelay
  float effectiveLeadIn = P.sensorLeadIn + (gps_ips * P.hydraulicDelaySec);
  float leadTimeSec = effectiveLeadIn / gps_ips;

  if (!isnan(R.filtPrev) && dtSec > 0.0f) {
    float slope = (R.filtNow - R.filtPrev) / dtSec;  // in/sec
    R.predicted = R.filtNow + slope * leadTimeSec;
  } else {
    R.predicted = R.filtNow;
  }
}

// ======================================================
// SETUP / LOOP
// ======================================================
void setup() {
  pinMode(PIN_VALVE_UP_R, OUTPUT);
  pinMode(PIN_VALVE_DOWN_R, OUTPUT);
  valvesOffRight();

  Serial.begin(115200);

  if (!loadParamsFromEEPROM()) {
    setDefaults();
    saveParamsToEEPROM();
    Serial.println("EEPROM INIT DEFAULTS");
  } else {
    Serial.println("EEPROM LOADED");
  }

  Comm_Init();

  // Make sure off at boot
  g_armed = false;
  g_faultFlags = FAULT_NONE;
  g_lastCommsMs = millis();
  sensorInvalidTimer = 0;
  valvesOffRight();
  Serial.println("DISARMED (boot)");
}


void loop() {
  // ================= TAP TEST LOGIC =================
  if (g_tapTestActive) {
    // Read sensor
    float h = readRightInchesFiltered();
    if (!isnan(h) && isnan(g_tapBaseline)) {
      g_tapBaseline = h;
    }

    // Issue valve pulse once
    if (!g_tapValveIssued && !isnan(g_tapBaseline)) {
      commandRight(-1);  // DOWN (safe direction)
      g_tapStartMs = millis();
      g_tapValveIssued = true;
    }

    // Detect first movement
    if (g_tapValveIssued && !isnan(h)) {
      if (fabs(h - g_tapBaseline) > 0.3f)  // 0.3 in threshold
      {
        unsigned long dtMs = millis() - g_tapStartMs;
        float delaySec = dtMs / 1000.0f;

        valvesOffRight();

        Serial.print("TAPRESULT,");
        Serial.println(delaySec, 3);

        g_tapTestActive = false;
      }
    }

    // Safety timeout
    if (g_tapValveIssued && millis() - g_tapStartMs > 2000) {
      valvesOffRight();
      Serial.println("ERR TAPTEST TIMEOUT");
      g_tapTestActive = false;
    }

    return;  // skip normal control
  }

  Comm_Poll();
 
  float gps_ips = max(gps_mph * 17.6f, 5.0f);

  unsigned long intervalMs =
    (unsigned long)constrain(250.0f / (gps_ips / 12.0f), 80.0f, 180.0f);



  if (readTimer >= intervalMs) {
    float dtSec = (float)readTimer / 1000.0f;
    readTimer = 0;

    updateRight(dtSec, gps_ips);

    // ---- Sensor validity tracking ----
    bool sensorValid = !isnan(R.predicted);

    if (sensorValid) {
      sensorInvalidTimer = 0;
      g_faultFlags &= ~FAULT_SENSOR_BAD;
    } else {
      // Immediate safe action on any bad reading:
      valvesOffRight();
      R.err = NAN;

      // If it stays bad too long, latch fault + disarm
      if (sensorInvalidTimer >= SENSOR_INVALID_DISARM_MS) {
        g_faultFlags |= FAULT_SENSOR_BAD;
        if (g_armed) {
          g_armed = false;
          Serial.println("AUTO DISARM (sensor invalid)");
        }
      }
    }

    // ---- Comms-loss watchdog ----
    // If no command received from PC for too long, disarm.
    unsigned long nowMs = millis();
    if ((nowMs - g_lastCommsMs) >= COMMS_LOSS_DISARM_MS) {
      g_faultFlags |= FAULT_COMMS_LOSS;
      if (g_armed) {
        g_armed = false;
        Serial.println("AUTO DISARM (comms loss)");
      }
    } else {
      g_faultFlags &= ~FAULT_COMMS_LOSS;
    }

    // ---- Hard gate: if not armed, valves OFF and skip control ----
    if (!g_armed) {
      valvesOffRight();
    } else {
      // Only run control when armed AND sensor valid AND no comms-loss fault
      if (sensorValid && !(g_faultFlags & FAULT_COMMS_LOSS)) {
        R.err = R.predicted - P.targetIn;

        if (fabs(R.err) <= P.deadbandIn) {
          if (R.cmd != 0 && R.valveTimer >= P.minValveOnMs) {
            valvesOffRight();
          }
        } else {
          // pulse-by-error
          float mag = fabs(R.err) - P.deadbandIn;
          mag = constrain(mag, 0.0f, P.errForMaxPulseIn);

          unsigned long pulseMs = (unsigned long)(P.pulseMinMs + (P.pulseMaxMs - P.pulseMinMs) * (mag / P.errForMaxPulseIn));

          if (R.cmd == 0) {
            int8_t dir = (R.err > 0) ? -1 : +1;  // err>0 => too high => DOWN
            commandRight(dir);
          }

          if (R.cmd != 0 && R.valveTimer >= pulseMs && R.valveTimer >= P.minValveOnMs) {
            valvesOffRight();
          }
        }
      } else {
        // Armed but blocked by safety condition -> valves off
        valvesOffRight();
      }
    }
  }

  Comm_TelemetryTick(gps_ips);
}
