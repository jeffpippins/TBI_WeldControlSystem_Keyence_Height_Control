# TBI SubArc Height Control — Plan / current-state (rewritten from code)

## Context
Arduino Nano (ATmega328P) firmware that holds a Submerged Arc Welding (SAW) torch
at constant height using a **Keyence IL-1000 / IL-300** laser. Built section by
section, flashed and bench-verified as we go. The **digital HIGH/LOW judgments**
(from the IL-1000, set on the amplifier) do the fine height control; the **analog
A7** signal (0–5 V, median-filtered) is a sanity/rate guard that **freezes torch
motion** during physically-implausible events (tack, gap, off-plate) so the torch
never dives or climbs when the reading isn't trustworthy.

Committed on branch `feature/height-control-firmware`. This document reflects
`src/main.cpp`, `include/pins.h`, `platformio.ini` as they are now.

## Hardware / wiring (external opto boards; Nano only ever sees/drives 5 V)
- Keyence digital outs (LOW/HIGH/ALARM) → 24 V→5 V **NPN** opto → Nano (asserted = LOW).
- Keyence **analog** 0–5 V → Nano **A7**. Rescaled on the amp to **±10 mm window**
  → **~51 counts/mm** (0.02 mm/count), target ≈ 512 counts (2.5 V).
- External active-low **auto** command → **D3** (momentary button).
- Nano **D5** (Mode) → 5 V→24 V opto → SAW auto-enable; **D6/D7** → RAISE/LOWER.
- **D4** → alarm/fault LED.

## Pin map — `include/pins.h`
| Signal | Macro | Pin |
|--------|-------|-----|
| Auto button (active-low) | `PIN_ENABLE` | D3 |
| IL HIGH judgment | `PIN_IL_HIGH` | A0 |
| IL LOW judgment | `PIN_IL_LOW` | A1 |
| IL ALARM | `PIN_IL_ALARM` | A2 |
| IL analog | `PIN_IL_ANALOG` | A7 (ADC-only) |
| Mode → SAW enable | `PIN_MODE` | D5 |
| RAISE / UP | `PIN_UP` | D6 |
| LOWER / DOWN | `PIN_DOWN` | D7 |
| Alarm LED | `PIN_ALARM_LED` | D4 |

## Runtime architecture — `src/main.cpp`
Fixed **50 Hz / 20 ms** loop (`SCAN_MS`) via a `millis()` scan gate. `loop()` order:
`updateAlarm → updateAutoButton → updateAnalog → updateAnalogStats → updateGuard
→ updateHeight → updateIndicator → printStatus`.

- **`updateAutoButton()`** — 30 ms (`DEBOUNCE_MS`) debounce on D3; each *press edge*
  toggles latched `automode` and drives D5 (HIGH = auto on). Powers up OFF (safe).
- **`updateAlarm()`** — `alarm = !(digitalRead(PIN_IL_ALARM) == ALARM_ACTIVE_LEVEL)`
  with `ALARM_ACTIVE_LEVEL = LOW` → `alarm` true when A2 reads HIGH. (Verify sense
  + fail-safe direction at bench.) Does NOT drive the LED.
- **`readAnalogMedian()` / `updateAnalog()`** — median of `ANALOG_SAMPLES = 100`
  reads of A7 (~10 ms) → `analogValue`.
- **`updateAnalogStats()`** — 10 one-second buckets → sliding ~10 s
  `analogMin`/`analogMax` (computed but not currently on the status line).
- **`updateGuard()`** — the freeze logic (see below); sets `mode` + `freezeReason`.
- **`updateHeight()`** — reads HIGH/LOW; drives UP/DOWN **only when
  `automode && !alarm && mode == TRACKING`**, else both OFF. Map: HIGH→UP, LOW→DOWN
  (no invert constant; to swap, swap the two `digitalWrite`s or the pins).
- **`updateIndicator()`** — D4: **solid = `alarm`**; else **fast blink** (`BLINK_MS`)
  when `automode && mode == HOLD`; else OFF.
- **`printStatus()` / `print4()`** — one `\r` line, fixed-width (never wraps).
- **`setup()`** — Serial 115200; INPUT_PULLUP inputs; boot-safe outputs (LOW before
  OUTPUT); `initAnalogStats()`; seed `baseline`/`prevAnalog`.

## Guard behavior (`updateGuard`)
Each scan: `delta = analogValue - prevAnalog`.
- **Freeze (→HOLD)** if `alarm` OR railed (`≤RAIL_LO` / `≥RAIL_HI`) OR `|delta| > SLEW_THRESH`.
- **In TRACKING** (no disturbance): `baseline` follows `analogValue` (slow drift).
- **In HOLD** (UP/DOWN forced off): resume to TRACKING when valid + slow + within
  `RETURN_TOL` of `baseline` for `SETTLE` samples; OR, if a new level persists
  `STEP_CONFIRM` samples without returning, adopt it as the new baseline and resume.
- `freezeReason` (R_SLEW/R_RAIL/R_ALARM/R_WAIT) drives the status letter.

## Constants / tunables (current values)
| Name | Value | Meaning |
|------|-------|---------|
| `SCAN_MS` | 20 | loop period (50 Hz) |
| `ANALOG_SAMPLES` | 100 | median filter width |
| `DEBOUNCE_MS` | 30 | button debounce |
| `SLEW_THRESH` | 8 | counts/sample (~0.16 mm/samp, ~7.8 mm/s) freeze trip |
| `RETURN_TOL` | 50 | counts (~1 mm) to count as "returned to baseline" |
| `SETTLE` | 3 | samples near baseline before resume (60 ms) |
| `STEP_CONFIRM` | 350 | samples (~7 s) to accept a permanent new level (> ~6 s max tack) |
| `RAIL_LO/HI` | 5 / 1018 | out-of-window rail thresholds |
| `BLINK_MS` | 100 | alarm-LED blink half-period (~5 Hz) |
| `ALARM_ACTIVE_LEVEL` | LOW | pin sense for alarm (bench-verify) |
| `JUDGE_ACTIVE_LEVEL` | LOW | HIGH/LOW asserted level |
| `WIN_BUCKETS`/`BUCKET_MS` | 10 / 1000 | 10 s min/max window |

## Status line (115200 baud, single line, in-place)
`M:` `TRK ` or `HLD-S/R/A/W` · `AUTO:` · `ALM:` · `HI:` · `LO:` · `UP:` · `DN:` ·
`A7:####` · `base:####` · `d:±####` (per-sample delta) · `sc:####` (step-confirm counter).

## Reference (for tuning the guard)
- 51 counts/mm, 0.02 mm/count; ±10 mm analog window.
- Sampling geometry: `samples/inch = 3000 / travel-in-min`; at 50 ipm ≈ 0.42 mm/sample.
- Tacks 3/16"–1/4" (4.76–6.35 mm) → ~240–325 counts off baseline; edge ~14–140 c/sample.
- **Vertical slide (self-commanded motion):** 12 ipm = 5.08 mm/s → ~5.2 c/sample at
  20 ms. The guard can't distinguish this from a disturbance, so `SLEW_THRESH` must
  bracket real warp (~0.4) **and** slide motion (~5.2) *below* it and the tack edge
  (14–140) *above* it. At the current 8-count trip that's only a ~1.5× margin over the
  slide — **re-check if slide speed exceeds ~18 ipm or the analog window is rescaled to
  more counts/mm** (either raises the slide's c/sample and can trip the controller's own
  corrections → stutter, D4 blinking during normal moves).
- Real warp ≈ 0.4 c/sample. `SLEW_THRESH` sits between warp/slide-motion and the tack-edge.
- Set `SLEW_THRESH` from the `d:` field: max `d:` while stationary (noise floor) vs
  the `d:` spike on a real tack; pick a value between.

## Open items
- **Alarm logic/polarity** (`updateAlarm`) — verify at bench, incl. the fail-safe
  direction (a disconnected ALARM line should read as fault).
- **Direction map** HIGH→UP / LOW→DOWN — verify torch moves correctly; swap if not.
- Re-add the 10 s min/max to the status line if wanted (dropped to keep the line
  from wrapping; diagnostics `d:`/`sc:` currently occupy that space).

## Verification
1. Build: `pio run` → SUCCESS (no warnings; ~260 B RAM, ~4.1 KB flash).
2. Flash: `pio run -t upload` (COM3); `pio device monitor` @ 115200.
3. Bench: press D3 → `AUTO:1` + D5 high; trigger HIGH/LOW → UP/DN follow (in auto);
   wave a target fast → `M:HLD-S`, D4 blinks, UP/DN drop; remove target →
   `M:HLD-A/R`, D4 solid/blink; confirm `d:` noise floor vs tack spike to tune
   `SLEW_THRESH`; confirm W holds are brief now that `RETURN_TOL`≈1 mm.
