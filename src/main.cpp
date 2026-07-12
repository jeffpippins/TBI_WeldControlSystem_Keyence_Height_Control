// ============================================================================
//  TBI SubArc Height Control
//  Arduino Nano (ATmega328P) bang-bang torch-height controller for a
//  Submerged Arc Welding (SAW) slide, using a Keyence IL-1000/IL-300 laser.
//
//  The IL-1000 does the height judging (LOW / HIGH / ALARM) and outputs a 0-5V
//  analog distance; the Nano reacts:
//    * HIGH/LOW digital -> drive the SAW slide UP/DOWN (bang-bang)
//    * analog -> slew-rate + baseline sanity check that rides over fast
//      artifacts (spatter, tacks, gouges, off-plate) instead of chasing them
//
//  Pin map, polarity, and all tunables are in include/pins.h.
//  See README.md for wiring and the bring-up / calibration procedure.
// ============================================================================
#include <Arduino.h>
#include "pins.h"

// ---------------------------------------------------------------------------
//  Low-level I/O helpers (apply configured active levels)
// ---------------------------------------------------------------------------
static inline bool readAsserted(uint8_t pin, uint8_t activeLevel) {
  return digitalRead(pin) == activeLevel;
}

static inline void writeOutput(uint8_t pin, bool on, uint8_t activeLevel) {
  digitalWrite(pin, on ? activeLevel : (activeLevel == HIGH ? LOW : HIGH));
}

static inline void setMode(bool on)     { writeOutput(PIN_MODE,      on, OUT_ACTIVE_LEVEL); }
static inline void setUp(bool on)       { writeOutput(PIN_UP,        on, OUT_ACTIVE_LEVEL); }
static inline void setDown(bool on)     { writeOutput(PIN_DOWN,      on, OUT_ACTIVE_LEVEL); }
static inline void setAlarmLed(bool on) { writeOutput(PIN_ALARM_LED, on, LED_ACTIVE_LEVEL); }

// Drive exactly one direction (or neither). UP and DOWN can never be co-asserted.
static inline void driveRaise() { setDown(false); setUp(true);   }
static inline void driveLower() { setUp(false);   setDown(true); }
static inline void holdMotion() { setUp(false);   setDown(false); }

// ---------------------------------------------------------------------------
//  Debounce: commit a raw reading to a stable state once it holds N scans
// ---------------------------------------------------------------------------
struct Debounced {
  bool    stable;   // committed value
  bool    last;     // last raw value seen
  uint8_t count;    // consecutive scans of `last`
};

static void debounceUpdate(Debounced &d, bool raw) {
  if (raw != d.last) {
    d.last = raw;
    d.count = 1;
  } else if (d.count < 255) {
    d.count++;
  }
  if (d.count >= DEBOUNCE_SCANS) {
    d.stable = d.last;
  }
}

// ---------------------------------------------------------------------------
//  Analog: median-of-N read (rejects single-sample spatter spikes)
// ---------------------------------------------------------------------------
static int analogMedian() {
  int s[MEDIAN_SAMPLES];
  for (uint8_t i = 0; i < MEDIAN_SAMPLES; i++) {
    s[i] = analogRead(PIN_IL_ANALOG);
  }
  for (uint8_t i = 1; i < MEDIAN_SAMPLES; i++) {   // insertion sort
    int v = s[i];
    int8_t j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  return s[MEDIAN_SAMPLES / 2];
}

// ---------------------------------------------------------------------------
//  Control state
// ---------------------------------------------------------------------------
enum SubState { TRACKING, DISTURBANCE };

static Debounced dbEnable  = {false, false, 0};
static Debounced dbLow     = {false, false, 0};
static Debounced dbHigh    = {false, false, 0};
static Debounced dbAlarmOK = {false, false, 0};  // starts "not OK" = fail-safe

static bool     inAuto      = false;
static SubState sub         = DISTURBANCE;
static int      aPrev       = 0;     // previous filtered analog reading
static int      baseline    = 0;     // last-known-good plate reading
static uint16_t settleCount = 0;     // scans back at baseline
static uint16_t stepCount   = 0;     // scans a new level has persisted

static void enterManual() {
  setMode(false);
  holdMotion();
  setAlarmLed(false);
  inAuto = false;
}

static void enterAuto(int aNow) {
  setMode(true);          // enable the SAW slide + Auto-on LED
  holdMotion();
  aPrev       = aNow;     // delta = 0 on first scan (no false slew)
  baseline    = aNow;
  sub         = TRACKING;
  settleCount = 0;
  stepCount   = 0;
  inAuto      = true;
}

// ---------------------------------------------------------------------------
//  Debug telemetry (compiled in only for the *_debug build). One labeled line
//  per ~200 ms: every input, the analog value, and every output.
// ---------------------------------------------------------------------------
#ifdef DEBUG_SERIAL
static void debugStatus(bool enabled, int aNow, int delta,
                        bool lowJ, bool highJ, bool sensorOK) {
  static uint16_t tick = 0;
  if (++tick < (200 / SCAN_MS)) return;   // throttle to ~5 Hz
  tick = 0;

  // Inputs
  Serial.print(F("IN  EN:"));   Serial.print(enabled);
  Serial.print(F(" LOW:"));     Serial.print(lowJ);
  Serial.print(F(" HIGH:"));    Serial.print(highJ);
  Serial.print(F(" ALM:"));     Serial.print(sensorOK ? F("OK") : F("FAULT"));
  // Analog
  Serial.print(F(" | A7:"));    Serial.print(aNow);
  Serial.print(F(" d:"));       Serial.print(delta);
  Serial.print(F(" base:"));    Serial.print(baseline);
  // Outputs (read back the driven pin levels)
  Serial.print(F(" | OUT MODE:")); Serial.print(readAsserted(PIN_MODE,      OUT_ACTIVE_LEVEL));
  Serial.print(F(" UP:"));         Serial.print(readAsserted(PIN_UP,        OUT_ACTIVE_LEVEL));
  Serial.print(F(" DN:"));         Serial.print(readAsserted(PIN_DOWN,      OUT_ACTIVE_LEVEL));
  Serial.print(F(" FLT:"));        Serial.print(readAsserted(PIN_ALARM_LED, LED_ACTIVE_LEVEL));
  // State
  Serial.print(F(" | "));
  if (!enabled)              Serial.println(F("MANUAL"));
  else if (sub == TRACKING)  Serial.println(F("TRACKING"));
  else                       Serial.println(F("DISTURBANCE"));
}
#endif

// ---------------------------------------------------------------------------
void setup() {
  // Inputs
  pinMode(PIN_ENABLE,   INPUT_PULLUP);
  pinMode(PIN_IL_LOW,   INPUT_PULLUP);
  pinMode(PIN_IL_HIGH,  INPUT_PULLUP);
  pinMode(PIN_IL_ALARM, INPUT_PULLUP);
  // PIN_IL_ANALOG (A7) is ADC-only; no pinMode / pull-up.

  // Outputs - drive everything OFF *before* anything else (boot-safe).
  pinMode(PIN_MODE,      OUTPUT);
  pinMode(PIN_UP,        OUTPUT);
  pinMode(PIN_DOWN,      OUTPUT);
  pinMode(PIN_ALARM_LED, OUTPUT);
  setMode(false);
  holdMotion();
  setAlarmLed(false);

#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println(F("TBI SubArc Height Control - DEBUG build"));
#endif
}

// ---------------------------------------------------------------------------
void loop() {
  // Fixed-rate scan gate.
  static uint32_t nextScan = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextScan) < 0) return;
  nextScan = now + SCAN_MS;

  // --- Sample every input each scan (visible even in MANUAL, for bring-up) --
  debounceUpdate(dbEnable,  readAsserted(PIN_ENABLE,  ENABLE_ACTIVE_LEVEL));
  debounceUpdate(dbLow,     readAsserted(PIN_IL_LOW,  JUDGE_ACTIVE_LEVEL));
  debounceUpdate(dbHigh,    readAsserted(PIN_IL_HIGH, JUDGE_ACTIVE_LEVEL));
  debounceUpdate(dbAlarmOK, digitalRead(PIN_IL_ALARM) == ALARM_OK_LEVEL);
  bool lowJ     = dbLow.stable;
  bool highJ    = dbHigh.stable;
  bool sensorOK = dbAlarmOK.stable;
  int  aNow     = analogMedian();
  int  delta    = 0;

  if (!dbEnable.stable) {
    // --- MANUAL: SAW ignores us; keep outputs safe -------------------------
    if (inAuto) enterManual();
    aPrev = aNow;                 // stay fresh so enabling sees no false slew

  } else if (!inAuto) {
    // --- First AUTO scan: initialize ---------------------------------------
    enterAuto(aNow);

  } else {
    // --- AUTO --------------------------------------------------------------
    delta = aNow - aPrev;
    aPrev = aNow;

    bool railed    = (aNow <= ANALOG_MIN_VALID) || (aNow >= ANALOG_MAX_VALID);
    bool slew      = (abs(delta) > MAX_SLEW_COUNTS_PER_SCAN);
    bool hardFault = (!sensorOK) || railed;

    // Any fast/invalid reading freezes the axis (baseline stays frozen).
    if ((hardFault || slew) && sub != DISTURBANCE) {
      sub = DISTURBANCE;
      settleCount = 0;
      stepCount = 0;
    }

    if (sub == DISTURBANCE) {
      holdMotion();
      setAlarmLed(true);
      // Resume only when the signal is valid AND slow again.
      if (!hardFault && !slew) {
        if (abs(aNow - baseline) <= RETURN_TOL) {
          // Returned to the pre-disturbance height (tack/spatter/gouge passed).
          stepCount = 0;
          if (++settleCount >= RECOVER_SETTLE_SCANS) {
            sub = TRACKING;         // baseline unchanged - torch never moved
          }
        } else {
          // Settled at a new level - accept only if it persists.
          settleCount = 0;
          if (++stepCount >= STEP_CONFIRM_SCANS) {
            baseline = aNow;        // genuine fast step -> re-center from here
            sub = TRACKING;
          }
        }
      } else {
        settleCount = 0;
        stepCount = 0;
      }

    } else {
      // --- TRACKING: healthy + slow -> bang-bang on the judgment window -----
      setAlarmLed(false);
      baseline = aNow;              // follow slow, genuine height drift

      if (highJ == lowJ) {
        holdMotion();               // in GO window (neither) or invalid (both)
      } else {
        bool wantRaise = highJ;     // default: HIGH judgment -> RAISE
#if DIRECTION_INVERT
        wantRaise = !wantRaise;
#endif
        if (wantRaise) driveRaise(); else driveLower();
      }
    }
  }

#ifdef DEBUG_SERIAL
  debugStatus(dbEnable.stable, aNow, delta, lowJ, highJ, sensorOK);
#endif
}
