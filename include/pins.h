// ============================================================================
//  TBI SubArc Height Control  -  pin assignments
//  Arduino Nano (ATmega328P)
// ============================================================================
#pragma once
#include <Arduino.h>

// --- Inputs -----------------------------------------------------------------
#define PIN_ENABLE     3    // D3  - auto-mode momentary button (active-low)
#define PIN_IL_HIGH    A0   // A0  - Keyence IL-1000 HIGH judgment
#define PIN_IL_LOW     A1   // A1  - Keyence IL-1000 LOW  judgment
#define PIN_IL_ALARM   A2   // A2  - Keyence IL-1000 ALARM / system-OK
#define PIN_IL_ANALOG  A7   // A7  - Keyence IL-1000 0-5V analog out (ADC only)

// --- Outputs ----------------------------------------------------------------
#define PIN_MODE       5    // D5  - AutoHeightMode -> SAW enable (HIGH = auto on)
#define PIN_UP         6    // D6  - RAISE torch
#define PIN_DOWN       7    // D7  - LOWER torch
#define PIN_ALARM_LED  4    // D4  - Alarm/fault indicator LED
