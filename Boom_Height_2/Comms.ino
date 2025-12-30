// ===================== Comms.ino =====================
#include <Arduino.h>
#include <elapsedMillis.h>
#include <string.h>
#include <stdlib.h>
#include "Types.h"

// These live in Main.ino
extern Params P;
extern RightState R;
extern float gps_mph;

// Safety globals (defined in Main.ino)
extern bool g_armed;
extern uint8_t g_faultFlags;
extern unsigned long g_lastCommsMs;

// These EEPROM functions live in Main.ino
extern void saveParamsToEEPROM();
extern bool loadParamsFromEEPROM();
extern void setDefaults();

// Tap test live in Main.ino
extern bool g_tapTestActive;
extern bool g_tapValveIssued;
extern unsigned long g_tapStartMs;
extern float g_tapBaseline;


// ------------------------------------------------------
// Comms state
// ------------------------------------------------------
static bool telemEnabled = true;

static const uint16_t RX_LINE_MAX = 96;
static char rxLine[RX_LINE_MAX];
static uint16_t rxLen = 0;

// ------------------------------------------------------
// Send helpers (USB now, UDP later)
// ------------------------------------------------------
static void sendLine(const char* s) {
  Serial.println(s);
}

static void sendKV(const char* key, float value, uint8_t decimals = 3) {
  Serial.print(key);
  Serial.print('=');
  Serial.println(value, decimals);
}

static void sendKVi(const char* key, long value) {
  Serial.print(key);
  Serial.print('=');
  Serial.println(value);
}

// ------------------------------------------------------
// Public: print help / params
// ------------------------------------------------------
void Comm_PrintHelp() {
  sendLine("CMDS: HELP | GET | PARAMS | SAVE | LOAD | DEFAULTS | SET <key> <value> | TELEM <0|1> | ARM <0|1> | FAULTS | STATUS | ESTOP");
  sendLine("Example: SET targetIn 24.5");
  sendLine("Example: SET gps_mph 5.8");
  sendLine("Example: ARM 1");
}

void Comm_PrintParams() {
  sendLine("PARAMS BEGIN");
  sendKV("targetIn", P.targetIn, 3);
  sendKV("deadbandIn", P.deadbandIn, 3);
  sendKV("sensorLeadIn", P.sensorLeadIn, 3);
  sendKV("hydraulicDelaySec", P.hydraulicDelaySec, 3);

  sendKVi("medianSamples", P.medianSamples);
  sendKV("validMinIn", P.validMinIn, 2);
  sendKV("validMaxIn", P.validMaxIn, 2);

  sendKVi("minValveOnMs", P.minValveOnMs);
  sendKVi("minValveOffMs", P.minValveOffMs);

  sendKVi("pulseMinMs", P.pulseMinMs);
  sendKVi("pulseMaxMs", P.pulseMaxMs);
  sendKV("errForMaxPulseIn", P.errForMaxPulseIn, 2);

  sendKVi("commMode", P.commMode);
  sendLine("PARAMS END");
}

// ------------------------------------------------------
// Telemetry (CSV)
// ------------------------------------------------------
static void printTelemetryCSV(float gps_ips) {
  // CSV: T,mph,ips,filt,pred,err,cmd,armed,fault
  Serial.print("T,");
  Serial.print(gps_mph, 2);
  Serial.print(',');
  Serial.print(gps_ips, 1);
  Serial.print(',');
  Serial.print(R.filtNow, 2);
  Serial.print(',');
  Serial.print(R.predicted, 2);
  Serial.print(',');
  Serial.print(R.err, 2);
  Serial.print(',');
  Serial.print((int)R.cmd);
  Serial.print(',');
  Serial.print(g_armed ? 1 : 0);
  Serial.print(',');
  Serial.println((int)g_faultFlags);
}

void Comm_TelemetryTick(float gps_ips) {
  static elapsedMillis telemTimer;
  if (!telemEnabled) return;

  if (telemTimer >= 100) {  // ~10 Hz
    telemTimer = 0;
    printTelemetryCSV(gps_ips);
  }
}

// ------------------------------------------------------
// SET handling
// ------------------------------------------------------
static void applySet(const char* key, const char* val) {
  float f = atof(val);
  long i = atol(val);

  if (!strcmp(key, "gps_mph")) gps_mph = f;

  else if (!strcmp(key, "targetIn")) P.targetIn = f;
  else if (!strcmp(key, "deadbandIn")) P.deadbandIn = f;
  else if (!strcmp(key, "sensorLeadIn")) P.sensorLeadIn = f;
  else if (!strcmp(key, "hydraulicDelaySec")) P.hydraulicDelaySec = f;

  else if (!strcmp(key, "medianSamples")) P.medianSamples = (uint8_t)constrain(i, 1, 9);
  else if (!strcmp(key, "validMinIn")) P.validMinIn = f;
  else if (!strcmp(key, "validMaxIn")) P.validMaxIn = f;

  else if (!strcmp(key, "minValveOnMs")) P.minValveOnMs = (uint16_t)constrain(i, 0, 5000);
  else if (!strcmp(key, "minValveOffMs")) P.minValveOffMs = (uint16_t)constrain(i, 0, 5000);

  else if (!strcmp(key, "pulseMinMs")) P.pulseMinMs = (uint16_t)constrain(i, 0, 5000);
  else if (!strcmp(key, "pulseMaxMs")) P.pulseMaxMs = (uint16_t)constrain(i, 0, 5000);
  else if (!strcmp(key, "errForMaxPulseIn")) P.errForMaxPulseIn = f;

  else if (!strcmp(key, "commMode")) P.commMode = (uint8_t)constrain(i, 0, 1);

  else {
    sendLine("ERR unknown_key");
    return;
  }

  sendLine("OK");
}

// ------------------------------------------------------
// Command parser
// ------------------------------------------------------
static void handleCommand(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == 0) return;

  char* cmd = strtok(line, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "HELP")) {
    Comm_PrintHelp();

  } else if (!strcasecmp(cmd, "GET") || !strcasecmp(cmd, "PARAMS")) {
    Comm_PrintParams();

  } else if (!strcasecmp(cmd, "SAVE")) {
    saveParamsToEEPROM();
    sendLine("OK SAVED");

  } else if (!strcasecmp(cmd, "LOAD")) {
    if (loadParamsFromEEPROM()) sendLine("OK LOADED");
    else sendLine("ERR LOAD_FAILED");

  } else if (!strcasecmp(cmd, "DEFAULTS")) {
    setDefaults();
    sendLine("OK DEFAULTS");

  } else if (!strcasecmp(cmd, "TAPTEST")) {
    if (g_armed) {
      sendLine("ERR TAPTEST requires DISARMED");
      return;
    }

    g_tapTestActive = true;
    g_tapValveIssued = false;
    g_tapBaseline = NAN;

    sendLine("OK TAPTEST START");

  } else if (!strcasecmp(cmd, "TELEM")) {
    char* v = strtok(nullptr, " \t");
    if (!v) {
      sendLine("ERR TELEM needs 0|1");
      return;
    }
    telemEnabled = (atoi(v) != 0);
    sendLine("OK");
  } else if (!strcasecmp(cmd, "SET")) {
    char* key = strtok(nullptr, " \t");
    char* val = strtok(nullptr, " \t");
    if (!key || !val) {
      sendLine("ERR SET needs <key> <value>");
      return;
    }
    applySet(key, val);
  } else if (!strcasecmp(cmd, "ARM")) {
    char* v = strtok(nullptr, " \t");
    if (!v) {
      sendLine("ERR ARM needs 0|1");
      return;
    }

    int a = atoi(v);
    g_armed = (a != 0);

    if (!g_armed) {
      // clear command state; main loop forces OFF
      R.cmd = 0;
      sendLine("OK DISARMED");
    } else {
      // clear faults on arm (they will re-latch if condition persists)
      g_faultFlags &= (uint8_t) ~(FAULT_COMMS_LOSS | FAULT_SENSOR_BAD | FAULT_ESTOP);
      g_lastCommsMs = millis();
      sendLine("OK ARMED");
    }
  } else if (!strcasecmp(cmd, "FAULTS")) {
    sendKVi("faultFlags", (int)g_faultFlags);
  } else if (!strcasecmp(cmd, "STATUS")) {
    // One-line status snapshot (easy for C# to parse)
    // STATUS,armed,fault,mph,filt,pred,err,cmd
    Serial.print("STATUS,");
    Serial.print(g_armed ? 1 : 0);
    Serial.print(',');
    Serial.print((int)g_faultFlags);
    Serial.print(',');
    Serial.print(gps_mph, 2);
    Serial.print(',');
    Serial.print(R.filtNow, 2);
    Serial.print(',');
    Serial.print(R.predicted, 2);
    Serial.print(',');
    Serial.print(R.err, 2);
    Serial.print(',');
    Serial.println((int)R.cmd);
  } else if (!strcasecmp(cmd, "ESTOP")) {
    g_armed = false;
    g_faultFlags |= FAULT_ESTOP;
    g_lastCommsMs = millis();
    R.cmd = 0;
    sendLine("OK ESTOP");
  } else {
    sendLine("ERR unknown_cmd");
  }
}

// ------------------------------------------------------
// Public API
// ------------------------------------------------------
void Comm_Init() {
  Comm_PrintHelp();
}

void Comm_Poll() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxLine[rxLen] = 0;

      // Any received line counts as "comms alive"
      g_lastCommsMs = millis();

      handleCommand(rxLine);
      rxLen = 0;
    } else {
      if (rxLen < RX_LINE_MAX - 1) rxLine[rxLen++] = c;
      // overflow: discard until newline
    }
  }
}

void PollSerial()
{
    while (Serial.available() > 0)
    {
        int b = Serial.peek();

        // AOG binary frame starts with 0x80
        if (b == 0x80)
        {
            AOG_Poll();      // consumes only AOG bytes
        }
        else
        {
            Comm_Poll();     // consumes ASCII lines
        }
    }
}

