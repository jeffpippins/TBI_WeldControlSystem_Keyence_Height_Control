# TBI SubArc Height Control

Arduino Nano (ATmega328P) firmware that holds a **Submerged Arc Welding (SAW)**
torch at a constant height above the plate as it travels, so plate warp and
unevenness don't change the contact-tip-to-work distance.

A **Keyence IL-1000 / IL-300** laser sensor measures the height and does the
threshold judging. The Nano is a **bang-bang controller** that drives the SAW's
existing motorized vertical slide up/down, with an **analog slew-rate + baseline
sanity check** that rides over fast artifacts (spatter, tacks, gouges, running
off the plate) instead of chasing them.

The operator configures everything (target height, judgment window, hysteresis)
**on the IL-1000 amplifier**. There is no UI on the Nano.

## How it works
- The IL-1000 outputs three digital judgments — **LOW**, **HIGH**, **ALARM** —
  plus a **0–5V analog** distance (zero-shifted so the target height ≈ 2.5 V).
- An external, active-low **ENABLE** ("auto height") command tells the Nano to
  start. While enabled, the Nano asserts its **Mode** output, which enables the
  SAW slide to obey the Nano's UP/DOWN and lights the Auto-on LED.
- Each 10 ms scan: **LOW → drive one way, HIGH → the other, in-window → hold.**
- The analog signal guards the loop: a **fast jump** (faster than a real height
  change could be) or an **invalid reading** freezes the axis; motion resumes
  only when the reading returns to its pre-disturbance **baseline** (or a new
  level persists long enough to be a genuine step). See *Disturbance handling*.

## Wiring / pinout
All isolation and level-shifting is external — the Nano only ever sees/drives
**5V logic**. Keyence digital outputs reach the Nano through 24V→5V opto boards
(NPN, so **asserted = LOW**); the Nano's outputs drive a 5V→24V opto board into
the SAW slide.

| Dir | Signal | Nano pin | Notes |
|-----|--------|----------|-------|
| IN  | ENABLE (auto height cmd) | **D3** | active-LOW, `INPUT_PULLUP` |
| IN  | IL-1000 LOW judgment | **A0** | digital, active-LOW |
| IN  | IL-1000 HIGH judgment | **A1** | digital, active-LOW |
| IN  | IL-1000 ALARM (system-OK) | **A2** | digital, **HIGH=OK / LOW=fault** |
| IN  | IL-1000 analog out | **A7** | 0–5V, ADC-only pin, ~512 at target |
| OUT | Mode → SAW enable (+ Auto LED) | **D5** | via 5V→24V opto |
| OUT | RAISE / UP → slide | **D6** | via 5V→24V opto |
| OUT | LOWER / DOWN → slide | **D7** | via 5V→24V opto |
| OUT | Alarm/fault LED | **D4** | lit on any fault-hold |

Free for expansion: D0/D1 (USB serial), D2, D8–D13, A3–A6.

### LED meanings
- **Auto-on LED (on D5):** lit whenever the controller is enabled and in auto mode.
- **Alarm LED (D4):** lit whenever the loop is *holding on a fault* — IL-1000 not
  OK, analog railed, or a fast/implausible reading. Off during normal tracking
  and during a normal in-window hold.

## Disturbance handling (tacks, spatter, gouges, off-plate)
A tack/gouge/spatter/off-plate event is a **fast excursion the plate returns
from** — the true height didn't change — so the firmware freezes the axis, rides
across at constant height, and resumes only on **return to baseline**:

- **Spatter** (sub-100 ms spike): rejected by the median analog filter; longer
  spikes trip the slew guard, hold, and return to baseline.
- **Tack** (bump, up to a few seconds under the beam): held throughout — because
  mid-tack the reading is steady but *far from baseline*, recovery waits for the
  trailing edge to bring it back. The torch never chases the bump.
- **Real fast step** (rare): if a new level persists past `STEP_CONFIRM_SCANS`,
  it's accepted as the new baseline and the torch re-centers calmly.
- **Off-plate / no target:** caught three ways (ALARM, analog railed, slew jump);
  holds last-good height until the plate returns.

## Configuration & tuning
All pins, signal polarity, and tunables live in [`include/pins.h`](include/pins.h).
Key tunables (scan-count values assume `SCAN_MS = 10`):

| Define | Default | Meaning |
|--------|---------|---------|
| `SCAN_MS` | 10 | control-loop period (ms) |
| `DEBOUNCE_SCANS` | 3 | digital input stable time (~30 ms) |
| `MEDIAN_SAMPLES` | 5 | analog median filter width |
| `MAX_SLEW_COUNTS_PER_SCAN` | 15 | **CALIBRATE** — max plausible per-scan change |
| `RECOVER_SETTLE_SCANS` | 3 | scans back at baseline before resuming |
| `RETURN_TOL` | 8 | **CALIBRATE** — counts within baseline = "returned" |
| `STEP_CONFIRM_SCANS` | 800 | ~8 s; a new level held this long is a real step |
| `ANALOG_MIN_VALID` / `MAX_VALID` | 20 / 1003 | analog rail (out-of-range) thresholds |
| `DIRECTION_INVERT` | 0 | swap which judgment raises vs lowers |
| `ALARM_OK_LEVEL` | LOW | pin level meaning "sensor OK" (see fail-safe note) |

`STEP_CONFIRM_SCANS` must exceed the longest time a tack sits under the beam
(tack length ÷ slowest travel speed). For 2 in tacks at 20 in/min that's 6.0 s,
so the 800-scan (8 s) default has margin. Retune if travel drops below 20 in/min.

## Build & upload
Production build:
```
pio run -t upload
```
Bring-up / tuning build (adds serial telemetry at 115200 baud):
```
pio run -e nanoatmega328_debug -t upload
pio device monitor
```

## Bring-up / calibration checklist
Do this on the bench first, then on the machine. Use the debug build.

1. **Boot check** — confirm all outputs come up OFF (Mode de-asserted).
2. **Input polarity** — assert ENABLE, trigger LOW/HIGH; confirm A0/A1 read as
   expected. Fix the `*_ACTIVE_LEVEL` defines if inverted.
3. **ALARM sense + fail-safe** — confirm the OK level on A2, force a Keyence
   fault (pin flips → Alarm LED + hold). **Disconnect the ALARM line and confirm
   it reads FAULT** — set `ALARM_OK_LEVEL` so a dead line is fail-safe.
4. **Analog scaling** — confirm A7 ≈ 512 at target and swings with distance; set
   `ANALOG_MIN_VALID` / `MAX_VALID` from the observed railed values.
5. **Slew calibration** — log `abs(delta)` during *normal* travel; set
   `MAX_SLEW_COUNTS_PER_SCAN` comfortably above the max normal value. Simulate a
   fast event and confirm it holds and auto-recovers (Alarm LED on during hold).
6. **Mode + outputs** (slide disconnected) — assert ENABLE → Mode + Auto LED on;
   force LOW/HIGH → correct single output; verify UP and DOWN are never both on;
   de-assert ENABLE → Mode and UP/DOWN drop.
7. **Direction** — connect the slide at low travel; confirm LOW/HIGH move the
   torch the correct way; flip `DIRECTION_INVERT` if reversed.
8. **Live weld trial** (low stakes) — confirm it holds height, tracks slow plate
   variation, ignores spatter/tack spikes without chatter, and returns to manual
   when ENABLE de-asserts. Fine-tune the Keyence window and `MAX_SLEW_*`.

## Project layout
- [`src/main.cpp`](src/main.cpp) — control loop / state machine
- [`include/pins.h`](include/pins.h) — pins, polarity, tunables
- [`platformio.ini`](platformio.ini) — build environments (prod + `_debug`)
