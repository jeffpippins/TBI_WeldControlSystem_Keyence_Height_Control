# TBI SubArc Height Control

Arduino Nano (ATmega328P) firmware that holds a **Submerged Arc Welding (SAW)**
torch at a constant height above the plate as it travels, so plate warp and
unevenness don't change the contact-tip-to-work distance.

A **Keyence IL-1000 / IL-300** laser sensor measures the height and does the
threshold judging. The Nano runs a **hybrid re-center controller** — the IL-1000
HIGH/LOW judgments *start* a correction and pick its direction, and the analog
distance signal decides when to *stop* — with an **analog slew-rate + baseline
guard** that rides over fast artifacts (spatter, tacks, gouges, running off the
plate) instead of chasing them.

The operator configures everything (target height, judgment window, hysteresis)
**on the IL-1000 amplifier**. There is no UI on the Nano — only an Auto-mode
button, two LEDs, and a live serial status line.

## How it works
- The IL-1000 outputs three digital judgments — **LOW**, **HIGH**, **ALARM** —
  plus a **0–5 V analog** distance (zero-shifted so the target height ≈ 2.5 V,
  ~512 ADC counts).
- The **Auto-mode button** (D3, momentary) latches auto height on/off. While on,
  the Nano asserts its **Mode** output (D5), which enables the SAW slide to obey
  the Nano's UP/DOWN and lights the Auto-on LED.
- Each **20 ms** scan the loop reads the alarm, debounces the button, takes a
  median-of-100 analog reading, updates a 10 s min/max window, runs the guard,
  drives the axis, updates the indicator LED, and prints the status line.
- **Hybrid re-center:** a HIGH or LOW judgment kicks off a correction and latches
  a direction (**HIGH → UP / raise**, **LOW → DOWN / lower**). The torch keeps
  driving that direction until the *analog* reading is back within `STOP_BAND` of
  `TARGET_COUNTS`, so it returns to ~center instead of parking at the ±band edge.
- The **guard** oversees all of this: a fast jump (faster than a real height
  change could be), a railed/invalid reading, or an ALARM freezes the axis
  (**HOLD**). Motion resumes only when the reading returns to its pre-disturbance
  **baseline**, or a new level persists long enough to be a genuine step. See
  *Disturbance handling*.

## Wiring / pinout
All isolation and level-shifting is external — the Nano only ever sees/drives
**5 V logic**. Keyence digital outputs reach the Nano through 24V→5V opto boards
(NPN, so **asserted = LOW**); the Nano's outputs drive a 5V→24V opto board into
the SAW slide.

| Dir | Signal | Nano pin | Notes |
|-----|--------|----------|-------|
| IN  | Auto-mode button | **D3** | momentary, active-LOW, `INPUT_PULLUP` |
| IN  | IL-1000 HIGH judgment | **A0** | digital, active-LOW |
| IN  | IL-1000 LOW judgment | **A1** | digital, active-LOW |
| IN  | IL-1000 ALARM (system-OK) | **A2** | digital, **HIGH=OK / LOW=fault** |
| IN  | IL-1000 analog out | **A7** | 0–5 V, ADC-only pin, ~512 at target |
| OUT | Mode → SAW enable (+ Auto LED) | **D5** | via 5V→24V opto |
| OUT | RAISE / UP → slide | **D6** | via 5V→24V opto |
| OUT | LOWER / DOWN → slide | **D7** | via 5V→24V opto |
| OUT | Alarm / freeze indicator LED | **D4** | see *LED meanings* |

Pins are defined in [`include/pins.h`](include/pins.h). Free for expansion:
D0/D1 (USB serial), D2, D8–D13, A3–A6.

### LED meanings
- **Auto-on LED (D5, the Mode output):** lit whenever auto mode is on.
- **Indicator LED (D4):**
  - **Solid** — true IL-1000 ALARM (sensor not OK / out of range).
  - **Fast blink** — the guard is *holding* on a disturbance (slew, rail, or
    settling) while auto mode is on.
  - **Off** — normal tracking, or auto mode off.

## Disturbance handling (tacks, spatter, gouges, off-plate)
A tack/gouge/spatter/off-plate event is a **fast excursion the plate returns
from** — the true height didn't change — so the firmware freezes the axis, rides
across at constant height, and resumes only on **return to baseline**:

- **Spatter** (sub-100 ms spike): rejected by the median analog filter; longer
  spikes trip the slew guard, hold, and return to baseline.
- **Tack** (bump, up to a few seconds under the beam): held throughout — because
  mid-tack the reading is steady but *far from baseline*, recovery waits for the
  trailing edge to bring it back. The torch never chases the bump.
- **Real fast step** (rare): if a new level persists past `STEP_CONFIRM`
  samples, it's accepted as the new baseline and the torch re-centers calmly.
- **Off-plate / no target:** caught three ways (ALARM, analog railed, slew jump);
  holds last-good height until the plate returns.

The guard has two states — **TRACKING** (following) and **HOLD** (frozen). In
TRACKING, a slow, valid reading updates the baseline; any disturbance drops it to
HOLD. In HOLD, once the reading is valid and slow again it either settles back to
baseline (within `RETURN_TOL` for `SETTLE` samples → resume) or, if it stays put
somewhere new for `STEP_CONFIRM` samples, adopts that as the new baseline.

## Configuration & tuning
Pins and signal assignments live in [`include/pins.h`](include/pins.h). All
timing/threshold tunables and signal polarities are `const`s near the top of
[`src/main.cpp`](src/main.cpp). Scan-count values assume `SCAN_MS = 20`.

| Constant | Default | Meaning |
|----------|---------|---------|
| `SCAN_MS` | 20 | control-loop period (ms) |
| `DEBOUNCE_MS` | 30 | button must settle this long before a toggle |
| `ANALOG_SAMPLES` | 100 | reads per sample, median-filtered |
| `SLEW_THRESH` | 8 | **CALIBRATE** — counts/sample above which a change is a fast artifact (see note below) |
| `RETURN_TOL` | 50 | **CALIBRATE** — counts within baseline (~1 mm) that count as "returned" |
| `SETTLE` | 3 | samples back near baseline before resuming |
| `STEP_CONFIRM` | 350 | ~7 s; a new level held this long is a real step |
| `RAIL_LO` / `RAIL_HI` | 5 / 1018 | analog rail (out-of-window) thresholds |
| `TARGET_COUNTS` | 512 | analog counts at the Keyence zero (~center) |
| `STOP_BAND` | 26 | counts (~0.5 mm): stop a correction within this of target |
| `BLINK_MS` | 100 | indicator-LED fast-blink half-period |
| `JUDGE_ACTIVE_LEVEL` | `LOW` | pin level meaning a HIGH/LOW judgment is asserted |
| `ALARM_ACTIVE_LEVEL` | `LOW` | pin level that means alarm/fault (fail-safe: a dead line reads FAULT) |

`STEP_CONFIRM` must exceed the longest time a tack sits under the beam
(tack length ÷ slowest travel speed). For 2 in tacks at 20 in/min that's 6.0 s,
so the 350-sample (~7 s at 20 ms) default has margin. Retune if travel drops
below 20 in/min.

`SLEW_THRESH` has an upper *and* lower bound. The guard can't tell the controller's
own commanded slide motion from an external disturbance, so the threshold must sit
**above** the per-sample analog change the slide produces while correcting — at
12 in/min with the ~51 counts/mm analog scaling that's ~5.2 counts/sample, so the
8-count default has only ~1.5× margin — while staying **below** the tack-edge delta
(~14–140 counts/sample). Re-check it if the slide speed exceeds ~18 in/min or the
IL-1000 analog window is rescaled to more counts/mm; otherwise the controller can
start freezing its own corrections (stutter, with the D4 LED blinking during normal
moves).

If UP/DOWN drive the torch the wrong way, swap the `PIN_UP`/`PIN_DOWN` mapping in
`updateHeight()` (a backwards map drives to a rail; the guard then freezes it).

## Build & upload
The firmware is a single build; serial telemetry (115200 baud) always prints.

```
pio run -t upload
pio device monitor
```

The board is `nanoatmega328new` (new bootloader, 115200 upload). CH340-clone
Nanos usually ship this bootloader; if uploads fail with
`stk500_getsync(): not in sync`, switch `board` in
[`platformio.ini`](platformio.ini) between `nanoatmega328new` (new, 115200) and
`nanoatmega328` (old, 57600).

### Serial status line
`printStatus()` overwrites a single line each scan (carriage-return, no ANSI), e.g.:

```
M:TRK  AUTO:1 ALM:0 HI:0 LO:0 UP:0 DN:0 A7:0512 base:0512 d:+0000 err:+0000
```

- **M** — mode: `TRK` (tracking) or `HLD-x` where `x` is the freeze reason
  (`S` slew, `R` rail, `A` alarm, `W` waiting to settle).
- **AUTO** — auto mode on/off. **ALM** — IL-1000 alarm. **HI/LO** — judgments.
- **UP/DN** — slide outputs. **A7** — filtered analog counts. **base** — guard
  baseline. **d** — last per-sample analog delta. **err** — `A7 − TARGET_COUNTS`.

## Bring-up / calibration checklist
Do this on the bench first, then on the machine, watching the serial status line.

1. **Boot check** — confirm all outputs come up OFF (Mode de-asserted, no motion).
2. **Input polarity** — press the button to enter auto; trigger LOW/HIGH and
   confirm `HI`/`LO` track. Fix the `*_ACTIVE_LEVEL` consts if inverted.
3. **ALARM sense + fail-safe** — confirm `ALM:0` when OK; force a Keyence fault
   (pin flips → `ALM:1`, solid LED, hold). **Disconnect the ALARM line and
   confirm it reads FAULT** — set `ALARM_ACTIVE_LEVEL` so a dead line is fail-safe.
4. **Analog scaling** — confirm `A7` ≈ 512 at target and swings with distance;
   set `RAIL_LO` / `RAIL_HI` from the observed railed values.
5. **Slew calibration** — watch `d:` during *normal* travel; set `SLEW_THRESH`
   comfortably above the max normal value. Simulate a fast event and confirm it
   holds and auto-recovers (LED blinks during hold).
6. **Mode + outputs** (slide disconnected) — button → Mode + Auto LED on; force
   LOW/HIGH → correct single output; verify `UP` and `DN` are never both `1`;
   button off → Mode and UP/DOWN drop.
7. **Direction** — connect the slide at low travel; confirm LOW/HIGH move the
   torch the correct way; swap `PIN_UP`/`PIN_DOWN` in `updateHeight()` if reversed.
8. **Live weld trial** (low stakes) — confirm it holds height, tracks slow plate
   variation, ignores spatter/tack spikes without chatter, and returns to manual
   when auto mode is off. Fine-tune the Keyence window and `SLEW_THRESH`.

## Project layout
- [`src/main.cpp`](src/main.cpp) — control loop, state machine, and all tunables
- [`include/pins.h`](include/pins.h) — pin assignments
- [`platformio.ini`](platformio.ini) — build configuration
- [`HOW_IT_WORKS.md`](HOW_IT_WORKS.md) — plain-language operator/theory overview
- [`PLAN.md`](PLAN.md) — firmware design notes
