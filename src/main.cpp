// ============================================================================
//  TBI SubArc Height Control  -  Arduino Nano (ATmega328P)
//  Built up one section at a time.
// ============================================================================
#include <Arduino.h>
#include "pins.h"

// --- Loop timing ------------------------------------------------------------
const unsigned long SCAN_MS = 20;        // fixed control-loop period (ms)

// --- Analog rate / baseline guard: state + tunables -------------------------
enum GuardMode    { TRACKING, HOLD };    // TRACKING = following; HOLD = frozen
enum FreezeReason { R_NONE, R_SLEW, R_RAIL, R_ALARM, R_WAIT };
GuardMode    mode = TRACKING;
FreezeReason freezeReason = R_NONE;      // why we're holding (display + LED)
int  baseline = 0;                       // last good plate reading (counts)
int  analogValue = 0;                    // filtered ADC counts (0-1023, ~512 at target)
int  lastDelta = 0;                      // last per-sample analog delta (diag)
static int prevAnalog = 0;               // previous sample (for slew delta)
static uint16_t settleCount = 0;         // samples back near baseline
static uint16_t stepCount   = 0;         // samples a new level has persisted

const int      SLEW_THRESH  = 40;        // counts/sample: above = fast artifact
                                         // Bench-tuned (2026-07). The laser rides the
                                         // JOINT GROOVE; on multipass welds that surface
                                         // is a prior bead, and its ripple tripped
                                         // constant false HLD-S freezes at the original
                                         // value of 8. 40 (~0.8 mm/sample) sits above
                                         // bead ripple and the commanded slide
                                         // (~5.2 c/sample @ 12 in/min) while remaining
                                         // below a sharp tack/step edge (up to ~140
                                         // c/sample). Trade-off: edges slower than
                                         // ~40 c/sample no longer freeze -- root-pass
                                         // tacks are assumed sharp. Re-check if the
                                         // IL-1000 analog window is rescaled.
const int      RETURN_TOL   = 50;        // counts (~1 mm) within baseline = "returned"
const uint8_t  SETTLE       = 3;         // samples near baseline before resume
const uint16_t STEP_CONFIRM = 350;       // ~7 s: accept a permanent new level
                                         // (must exceed the ~6 s longest tack)
const int      RAIL_LO      = 5;         // <= this = railed low  (out of window)
const int      RAIL_HI      = 1018;      // >= this = railed high (out of window)
const unsigned long BLINK_MS = 100;      // alarm-LED fast-blink half-period

// --- Auto-mode button (momentary, active-low) -------------------------------
bool automode = false;                   // latched: true = auto height on
const unsigned long DEBOUNCE_MS = 30;    // button must settle this long
static int btnStable = HIGH;             // debounced level (HIGH = released)
static int btnLast   = HIGH;             // last raw reading
static unsigned long btnChangedAt = 0;   // when the raw reading last changed
//-----------------------------------------------------------------------------


// Debounce PIN_ENABLE and toggle automode on each press.
static void updateAutoButton() {
  int reading = digitalRead(PIN_ENABLE);

  if (reading != btnLast) {              // raw changed -> restart the timer
    btnLast = reading;
    btnChangedAt = millis();
  }

  if (millis() - btnChangedAt >= DEBOUNCE_MS && reading != btnStable) {
    btnStable = reading;                 // commit the new stable level
    if (btnStable == LOW) {              // press edge (active-low) -> toggle
      automode = !automode;
      digitalWrite(PIN_MODE, automode ? HIGH : LOW);
    }
  }
}


// --- Alarm input (Keyence IL-1000 system-OK line) ---------------------------
// IL-1000 ALARM is HIGH=OK / LOW=fault; through the NPN opto the sense may
// invert, so ALARM_ACTIVE_LEVEL is the *pin* level that means "alarm/fault".
// Verify on the bench (disconnect the line: it should read ALARM = fail-safe).
const int ALARM_ACTIVE_LEVEL = LOW;      // pin level that means alarm/fault
bool alarm = false;                      // true = IL-1000 reporting a fault

// Read the alarm input into the `alarm` flag. (The D4 LED is driven by
// updateIndicator(), which also shows guard freezes as a fast blink.)
static void updateAlarm() {
  alarm = !(digitalRead(PIN_IL_ALARM) == ALARM_ACTIVE_LEVEL);
}


// --- Height control: hybrid re-center ---------------------------------------
// The Keyence HIGH/LOW judgments (+/-1.5 mm on the amp) START a correction and
// set its direction; the ANALOG value decides when to STOP -- keep driving until
// the reading is within STOP_BAND of TARGET_COUNTS, so the torch returns to ~0
// instead of parking at the +/-1.5 mm band edge.
// Mapping: HIGH -> UP (raise), LOW -> DOWN (lower). Swap on the bench if reversed
// (a backwards map drives to a rail; the guard then freezes it).
const int JUDGE_ACTIVE_LEVEL = LOW;      // pin level meaning a judgment is asserted
const int TARGET_COUNTS      = 512;      // analog counts at the Keyence zero (~center)
const int STOP_BAND          = 26;       // counts (~0.5 mm): stop within this of target
bool ilHigh = false;                     // HIGH judgment active
bool ilLow  = false;                     // LOW  judgment active

static void updateHeight() {
  ilHigh = (digitalRead(PIN_IL_HIGH) == JUDGE_ACTIVE_LEVEL);
  ilLow  = (digitalRead(PIN_IL_LOW)  == JUDGE_ACTIVE_LEVEL);

  static bool correcting = false;        // latched from a HIGH/LOW trigger
  static bool dirUp      = false;        // latched direction for this correction

  // Motion only in auto, no alarm, and while the guard is TRACKING.
  if (!(automode && !alarm && mode == TRACKING)) {
    correcting = false;
    digitalWrite(PIN_UP,   LOW);
    digitalWrite(PIN_DOWN, LOW);
    return;
  }

  // START: a Keyence judgment kicks off a correction and picks the direction.
  if (!correcting) {
    if      (ilHigh) { correcting = true; dirUp = true;  }
    else if (ilLow)  { correcting = true; dirUp = false; }
  }

  // STOP: once the analog is back within STOP_BAND of center.
  if (correcting && abs(analogValue - TARGET_COUNTS) <= STOP_BAND) {
    correcting = false;
  }

  // DRIVE the latched direction (interlocked: never both), else hold.
  if (correcting) {
    digitalWrite(PIN_UP,   dirUp ? HIGH : LOW);
    digitalWrite(PIN_DOWN, dirUp ? LOW  : HIGH);
  } else {
    digitalWrite(PIN_UP,   LOW);
    digitalWrite(PIN_DOWN, LOW);
  }
}


// --- Analog input (Keyence IL-1000 0-5V distance) ---------------------------
const uint8_t ANALOG_SAMPLES = 100;      // reads per sample, median-filtered
// (analogValue is declared with the guard state near the top of the file.)

// Take ANALOG_SAMPLES reads of A7 and return their median (rejects noise/spikes).
// ~100 x 100us = ~10 ms, well within the loop cadence. Even count -> average the
// two middle samples.
static int readAnalogMedian() {
  int s[ANALOG_SAMPLES];
  for (uint8_t i = 0; i < ANALOG_SAMPLES; i++) {
    s[i] = analogRead(PIN_IL_ANALOG);
  }
  for (uint8_t i = 1; i < ANALOG_SAMPLES; i++) {   // insertion sort
    int v = s[i];
    int8_t j = i - 1;
    while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
    s[j + 1] = v;
  }
  return (s[ANALOG_SAMPLES / 2 - 1] + s[ANALOG_SAMPLES / 2]) / 2;
}

// Read the analog distance signal on A7 (10-bit: 0-1023 over 0-5V).
static void updateAnalog() {
  analogValue = readAnalogMedian();
}


// --- Analog min/max over a sliding ~10 s window -----------------------------
// 10 one-second buckets. Each bucket holds the min/max seen during that second;
// the window min/max is the aggregate across all buckets. Rotating one bucket
// per second gives a ~10 s sliding window cheaply (no per-sample history).
const uint8_t      WIN_BUCKETS = 10;     // 10 buckets x 1 s = ~10 s window
const unsigned long BUCKET_MS  = 1000;
int winMin[WIN_BUCKETS];
int winMax[WIN_BUCKETS];
static uint8_t      winIdx = 0;
static unsigned long bucketStart = 0;
int analogMin = 0;                       // lowest  A7 count in the last ~10 s
int analogMax = 0;                       // highest A7 count in the last ~10 s

// Seed all buckets with the current reading (call once at startup).
static void initAnalogStats() {
  analogValue = readAnalogMedian();
  for (uint8_t i = 0; i < WIN_BUCKETS; i++) {
    winMin[i] = analogValue;
    winMax[i] = analogValue;
  }
  bucketStart = millis();
}

static void updateAnalogStats() {
  if (millis() - bucketStart >= BUCKET_MS) {   // start a fresh bucket each second
    bucketStart = millis();
    winIdx = (winIdx + 1) % WIN_BUCKETS;
    winMin[winIdx] = analogValue;
    winMax[winIdx] = analogValue;
  }
  if (analogValue < winMin[winIdx]) winMin[winIdx] = analogValue;
  if (analogValue > winMax[winIdx]) winMax[winIdx] = analogValue;

  int lo = winMin[0], hi = winMax[0];          // aggregate across all buckets
  for (uint8_t i = 1; i < WIN_BUCKETS; i++) {
    if (winMin[i] < lo) lo = winMin[i];
    if (winMax[i] > hi) hi = winMax[i];
  }
  analogMin = lo;
  analogMax = hi;
}


// --- Analog rate / baseline guard -------------------------------------------
// The torch moves only on a valid, stable, slowly-changing reading. A fast jump
// (tack/step edge), a railed reading (big gap / off-plate), or ALARM freezes
// motion (HOLD). It resumes when the signal returns to the pre-disturbance
// baseline, or when a new level persists long enough to be a genuine step.
static void updateGuard() {
  int delta = analogValue - prevAnalog;
  prevAnalog = analogValue;
  lastDelta = delta;                       // expose for the status line

  bool railed = (analogValue <= RAIL_LO) || (analogValue >= RAIL_HI);
  bool fast   = (abs(delta) > SLEW_THRESH);

  if (mode == TRACKING) {
    if (alarm || railed || fast) {         // disturbance -> freeze
      mode = HOLD;                          // baseline stays at last good value
      settleCount = 0;
      stepCount   = 0;
    } else {
      baseline = analogValue;               // follow slow, genuine drift
    }
  } else {                                  // HOLD
    if (!alarm && !railed && !fast) {       // valid and slow again
      if (abs(analogValue - baseline) <= RETURN_TOL) {
        stepCount = 0;
        if (++settleCount >= SETTLE) mode = TRACKING;   // returned to baseline
      } else {
        settleCount = 0;
        if (++stepCount >= STEP_CONFIRM) {              // real new level
          baseline = analogValue;
          mode = TRACKING;
        }
      }
    } else {
      settleCount = 0;
      stepCount   = 0;
    }
  }

  // Reason for the display / indicator.
  if      (mode == TRACKING) freezeReason = R_NONE;
  else if (alarm)            freezeReason = R_ALARM;
  else if (railed)           freezeReason = R_RAIL;
  else if (fast)             freezeReason = R_SLEW;
  else                       freezeReason = R_WAIT;
}


// --- D4 indicator: solid = ALARM, fast blink = guard freeze -----------------
static void updateIndicator() {
  if (alarm) {
    digitalWrite(PIN_ALARM_LED, HIGH);                 // solid = true alarm
  } else if (automode && mode == HOLD) {
    bool on = (millis() / BLINK_MS) & 1;               // fast blink = freeze
    digitalWrite(PIN_ALARM_LED, on ? HIGH : LOW);
  } else {
    digitalWrite(PIN_ALARM_LED, LOW);
  }
}


// --- Status report over serial ----------------------------------------------
// Updates a single line in place using a carriage return ('\r'): the cursor
// returns to column 0 each time so the text overwrites instead of scrolling.
// No ANSI escape codes, so it works in simple monitors too.
// (Redraw rate is set by the loop's SCAN_MS gate.)

// Print an ADC value zero-padded to a fixed 4 digits (keeps the line aligned).
static void print4(int v) {
  if (v < 1000) Serial.print('0');
  if (v < 100)  Serial.print('0');
  if (v < 10)   Serial.print('0');
  Serial.print(v);
}

// Print the current system state (expanded as more sections are added).
// Keep every field a fixed width so shorter values overwrite longer ones.
static void printStatus() {
  // All fields are fixed width, so the line length is constant and never wraps.
  Serial.print('\r');                    // back to column 0, overwrite the line

  Serial.print(F("M:"));                 // mode + freeze reason (fixed 5 chars)
  if (mode == TRACKING) {
    Serial.print(F("TRK  "));
  } else {
    Serial.print(F("HLD-"));
    char r = 'W';                        // R_WAIT
    if      (freezeReason == R_SLEW)  r = 'S';
    else if (freezeReason == R_RAIL)  r = 'R';
    else if (freezeReason == R_ALARM) r = 'A';
    Serial.print(r);
  }

  Serial.print(F(" AUTO:")); Serial.print(automode ? '1' : '0');
  Serial.print(F(" ALM:"));  Serial.print(alarm ? '1' : '0');
  Serial.print(F(" HI:"));   Serial.print(ilHigh ? '1' : '0');
  Serial.print(F(" LO:"));   Serial.print(ilLow ? '1' : '0');
  Serial.print(F(" UP:"));   Serial.print(digitalRead(PIN_UP) ? '1' : '0');
  Serial.print(F(" DN:"));   Serial.print(digitalRead(PIN_DOWN) ? '1' : '0');
  Serial.print(F(" A7:"));   print4(analogValue);
  Serial.print(F(" base:")); print4(baseline);
  Serial.print(F(" d:"));    Serial.print(lastDelta < 0 ? '-' : '+');
  print4(abs(lastDelta));
  int err = analogValue - TARGET_COUNTS;
  Serial.print(F(" err:"));  Serial.print(err < 0 ? '-' : '+');
  print4(abs(err));
  Serial.print(F("  "));                 // small pad
}


void setup() {
  Serial.begin(115200);

  // --- Inputs (opto boards drive these; use internal pull-ups) -------------
  pinMode(PIN_ENABLE,   INPUT_PULLUP);
  pinMode(PIN_IL_LOW,   INPUT_PULLUP);
  pinMode(PIN_IL_HIGH,  INPUT_PULLUP); 
  pinMode(PIN_IL_ALARM, INPUT_PULLUP);
  // PIN_IL_ANALOG (A7) is ADC-only; no pinMode / pull-up needed.

  // --- Outputs: drive OFF first, then set as outputs (boot-safe) -----------
  digitalWrite(PIN_MODE,      LOW);
  digitalWrite(PIN_UP,        LOW);
  digitalWrite(PIN_DOWN,      LOW);
  digitalWrite(PIN_ALARM_LED, LOW);
  pinMode(PIN_MODE,      OUTPUT);
  pinMode(PIN_UP,        OUTPUT);
  pinMode(PIN_DOWN,      OUTPUT);
  pinMode(PIN_ALARM_LED, OUTPUT);

  initAnalogStats();     // seed the 10 s min/max window
  baseline   = analogValue;   // seed guard state so the first scan sees no jump
  prevAnalog = analogValue;
}

void loop() {
  // Fixed-rate scan gate: run the body once every SCAN_MS.
  static unsigned long lastScan = 0;
  if (millis() - lastScan < SCAN_MS) return;
  lastScan = millis();

  updateAlarm();         // read A2 -> alarm flag
  updateAutoButton();    // debounce D3 -> toggle automode, drive D5
  updateAnalog();        // median-of-100 A7 -> analogValue
  updateAnalogStats();   // 10 s min/max
  updateGuard();         // set mode (TRACKING/HOLD) from analog + alarm
  updateHeight();        // drive UP/DOWN (only while TRACKING)
  updateIndicator();     // D4: solid=alarm, blink=freeze
  printStatus();
}
