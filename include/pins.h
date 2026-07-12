// ============================================================================
//  TBI SubArc Height Control - pin map, signal polarity, and tunables
//  Target: Arduino Nano (ATmega328P)
//
//  All pin assignments, active-levels, and timing constants live here so the
//  behaviour can be re-mapped / re-tuned without touching control logic.
//  Values marked "CALIBRATE" are finalized during bench/on-machine bring-up.
// ============================================================================
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Pin assignments  (see README wiring table)
// ---------------------------------------------------------------------------
//  Inputs (all 5V logic, arriving via external opto-isolator boards)
#define PIN_ENABLE     3    // D3  - active-low "auto height" command (external)
#define PIN_IL_LOW     A0   // A0  - Keyence IL-1000 LOW  judgment (digital)
#define PIN_IL_HIGH    A1   // A1  - Keyence IL-1000 HIGH judgment (digital)
#define PIN_IL_ALARM   A2   // A2  - Keyence IL-1000 ALARM / system-OK (digital)
#define PIN_IL_ANALOG  A7   // A7  - Keyence IL-1000 0-5V analog out (ADC only)

//  Outputs (5V logic; UP/DOWN/MODE feed a 5V->24V opto board to the SAW slide)
#define PIN_MODE       5    // D5  - AutoHeightMode -> SAW enable (+ Auto-on LED)
#define PIN_UP         6    // D6  - RAISE torch
#define PIN_DOWN       7    // D7  - LOWER torch
#define PIN_ALARM_LED  4    // D4  - Alarm/fault indicator LED

// ---------------------------------------------------------------------------
//  Signal polarity  (verify every one at bring-up; opto stages may invert)
// ---------------------------------------------------------------------------
// Enable + judgment lines arrive through NPN opto outputs -> asserted = LOW.
#define ENABLE_ACTIVE_LEVEL   LOW   // ENABLE input asserted when pin reads this
#define JUDGE_ACTIVE_LEVEL    LOW   // HIGH/LOW judgment asserted when pin reads this

// ALARM is a *system-OK* line (HIGH=OK / LOW=fault at the IL-1000).
// LOW here means: with INPUT_PULLUP a disconnected/dead line floats HIGH and
// therefore reads as FAULT -> fail-safe. CONFIRM at bring-up (checklist step 3).
#define ALARM_OK_LEVEL        LOW   // sensor considered OK when pin reads this

// Output drive: pin level that turns an output ON (opto board / LED input).
#define OUT_ACTIVE_LEVEL      HIGH  // UP/DOWN/MODE ON when pin driven to this
#define LED_ACTIVE_LEVEL      HIGH  // Alarm LED ON when pin driven to this

// Which judgment drives which direction. Default: HIGH judgment -> RAISE,
// LOW judgment -> LOWER. Set to 1 to swap if the torch moves the wrong way
// (bring-up checklist step 7).
#define DIRECTION_INVERT      0

// ---------------------------------------------------------------------------
//  Timing / control tunables   (scan-count values assume SCAN_MS = 10)
// ---------------------------------------------------------------------------
#define SCAN_MS               10    // fixed control-loop period (ms)
#define DEBOUNCE_SCANS        3     // digital input must hold N scans (~30 ms)
#define MEDIAN_SAMPLES        5     // analog median filter width (odd)

// Analog sanity check ------------------------------------------------------
#define MAX_SLEW_COUNTS_PER_SCAN 15 // CALIBRATE: max plausible per-scan change;
                                    // above this = fast artifact (tack/spatter)
#define RECOVER_SETTLE_SCANS  3     // scans back at baseline before resuming
#define RETURN_TOL            8     // CALIBRATE: counts within baseline = "returned"
#define STEP_CONFIRM_SCANS    800   // ~8 s: a new level held this long is a real
                                    // step, not a tack (2in tack /20ipm = 6.0 s)

// Analog validity (10-bit ADC, 5V ref: 512 ~= 2.5V target) -----------------
#define ANALOG_MIN_VALID      20    // <= this (~0.10V) = railed low  -> fault
#define ANALOG_MAX_VALID      1003  // >= this (~4.90V) = railed high -> fault

// ---------------------------------------------------------------------------
//  Optional directional cross-check (analog side must agree with digital)
//  OFF by default; needs an accurate center. Hooks left for future use.
// ---------------------------------------------------------------------------
#define ENABLE_DIR_CROSSCHECK 0
#define ANALOG_CENTER         512   // counts at target height (~2.5V)
