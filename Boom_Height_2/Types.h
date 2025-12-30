#pragma once

#include <Arduino.h>
#include <elapsedMillis.h>
#include <math.h>

// ======================================================
// EEPROM identity / versioning
// ======================================================
static const uint32_t PARAM_MAGIC   = 0xB00BCAFEUL;
static const uint16_t PARAM_VERSION = 1;

// ======================================================
// Communication mode (USB now, UDP later)
// ======================================================
enum CommMode : uint8_t {
  COMM_USB = 0,
  COMM_UDP = 1
};

// ======================================================
// Parameter block (EEPROM-backed)
// ======================================================
struct Params {
  uint32_t magic;
  uint16_t version;
  uint16_t sizeBytes;

  float targetIn;
  float deadbandIn;
  float sensorLeadIn;
  float hydraulicDelaySec;

  uint8_t medianSamples;
  float validMinIn;
  float validMaxIn;

  uint16_t minValveOnMs;
  uint16_t minValveOffMs;

  uint16_t pulseMinMs;
  uint16_t pulseMaxMs;
  float errForMaxPulseIn;

  uint8_t commMode;

  uint32_t crc32;
};

// ======================================================
// Runtime state (RIGHT SIDE)
// ======================================================
struct RightState {
  float filtPrev = NAN;
  float filtNow  = NAN;
  float predicted = NAN;
  float err = NAN;

  int8_t cmd = 0;            // +1=UP, -1=DOWN, 0=OFF
  elapsedMillis valveTimer;
};

// ======================================================
// Safety globals (defined in Main.ino, used by Comms.ino)
// ======================================================
extern bool g_armed;
extern uint8_t g_faultFlags;       // bitfield, see FaultFlags below
extern unsigned long g_lastCommsMs;

// Fault flags (bitfield)
enum FaultFlags : uint8_t {
  FAULT_NONE         = 0,
  FAULT_SENSOR_BAD   = 1 << 0,
  FAULT_COMMS_LOSS   = 1 << 1,
  FAULT_ESTOP        = 1 << 2,
};

// Safety timeouts (ms)
static const uint16_t SENSOR_INVALID_DISARM_MS = 600;   // invalid sensor -> disarm after this
static const uint16_t COMMS_LOSS_DISARM_MS     = 2000;  // no PC comms -> disarm after this


