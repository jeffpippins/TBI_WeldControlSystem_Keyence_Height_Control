# TBI SubArc Laser Height Control — How It Works

*A plain-language description of what the system does and how it behaves.*

---

## 1. What it does

During submerged-arc welding (SAW), the torch must stay a constant height above
the plate. As the torch travels, plate warp and unevenness change that height,
which changes the weld. This system **automatically keeps the torch at the set
height** by measuring the distance to the plate with a laser and raising or
lowering the welding machine's existing vertical slide to match.

The goal: consistent contact-tip-to-work distance across the whole weld, with no
operator babysitting — and, just as important, **it does not get fooled** by tack
welds, gaps, or moments when the laser can't see the plate.

---

## 2. The parts

- **Keyence IL-300 laser sensor** (mounted near the torch) + **IL-1000 amplifier** —
  measures the distance from the sensor to the plate surface.
- **Controller** (Arduino Nano) — reads the laser and decides when to raise, lower,
  or hold the slide.
- **The welding machine's existing vertical slide** — the controller does not add a
  motor; it simply tells the machine's own slide to go up or down.
- **Operator controls** — an on/off button and a status light.

All electrical connections between the controller and the machine are isolated, so
the controller and the welding circuit are electrically separated.

---

## 3. Using it (operator)

**Turning it on/off** — press the **Auto Height** button once to turn it **ON**,
press again to turn it **OFF**. While ON, the controller is allowed to move the
machine's slide; while OFF, the slide is under normal manual control.

**The status light:**

| Light | Meaning |
|-------|---------|
| **Off** | Normal — the system is tracking the plate. |
| **Fast blinking** | The system is **holding position** on purpose — it's riding over a tack, gap, or other disturbance and will resume on its own. |
| **Solid on** | **Sensor alarm** — the laser can't read the plate (out of range / no target). Motion is stopped and held. |

**Where the target height is set** — the desired height and the acceptable
tolerance band are configured **on the Keyence amplifier**, not on the controller.
The controller simply reacts to what the amplifier reports.

**What to expect** — once you press ON and start welding, the torch will gently
follow the plate. When it crosses a tack or a gap you'll see the light blink for a
moment while it holds steady, then it goes back to tracking. You don't need to do
anything during those.

---

## 4. How it decides to move (theory of operation)

The Keyence amplifier does two useful things at once:
1. It **judges** whether the torch is **too HIGH**, **too LOW**, or **in the
   acceptable band**.
2. It outputs a **live distance signal** the controller can watch.

**Normal tracking** is simple: too low → raise; too high → lower; in the band →
hold. This alone would keep height — but it would also blindly chase anything the
laser sees, which is a problem in a welding environment.

### The important part: the disturbance guard

Real plate-height changes are **slow and smooth**. Tacks, gaps, spatter, and the
laser running off the plate are **sudden or invalid**. The controller uses that
difference to protect the torch:

- It constantly watches **how fast** the reading is changing and whether it's
  **valid** (in range, no sensor alarm).
- A **sudden jump**, an **out-of-range** reading, or a **sensor alarm** means "this
  is not a real height change." The controller **freezes** — it holds the torch
  exactly where it is (status light blinks) and stops obeying the sensor.
- It **resumes only** when the reading is stable again **and back at the height it
  had before** the disturbance — i.e., the tack or gap has passed and the torch is
  over real plate again.

Because the torch physically holds still through the disturbance, and the plate on
both sides of a tack or gap is at the same height, when it resumes it's **already at
the right height** — no dive, no climb.

---

## 5. What it handles, and how

| Event | What happens |
|-------|--------------|
| **Tack weld (raised bump)** | Freezes and coasts over it at constant height; resumes once past it. It never climbs the tack. |
| **Gap or gouge** | Same — holds through, resumes on the far side. |
| **Laser runs off the plate / no target** | Holds the last good height (never dives into the void); resumes when the plate is back under the beam. |
| **Weld spatter / momentary flash** | Filtered out before it can affect anything. |
| **Genuine new plate height** | If the height really does change and *stays* changed (longer than any tack could last), the controller accepts the new height and tracks it. |

---

## 6. Built-in safety behaviors

- **Starts OFF.** On power-up the system is off; the operator must press to engage.
- **Safe on reset.** After any power cycle or reset, all outputs are off first, so
  the slide cannot lurch.
- **Fails to a stop.** If the sensor faults or loses the plate, motion stops and the
  torch holds — it does not guess.
- **No conflicting commands.** The controller can never call for "up" and "down" at
  the same time.
- **Operator in control.** The slide only responds to the controller while the
  operator has Auto Height switched on.

---

## 7. What is set where

| Setting | Where |
|---------|-------|
| Target height | Keyence IL-1000 amplifier |
| Acceptable height band (tolerance) | Keyence IL-1000 amplifier |
| Laser measuring window / analog scaling | Keyence IL-1000 amplifier |
| Disturbance sensitivity, hold/resume timing | Controller firmware (set once at commissioning) |

---

## 8. Commissioning notes (for the setup technician)

The controller has a live status readout over its USB serial port (115200 baud)
that shows every input, both outputs, the current mode, and the live distance value.
During first setup, use it to:

1. **Confirm signal polarity** — make sure each laser signal (HIGH, LOW, ALARM) and
   the on/off button read correctly.
2. **Confirm the alarm is fail-safe** — a disconnected sensor line should read as a
   fault (motion held), not as "OK."
3. **Confirm direction** — check that the torch raises and lowers the correct way;
   the mapping can be swapped if reversed.
4. **Tune disturbance sensitivity** — watch the live change-rate value while the
   plate is still (the noise floor) versus when a real tack passes (a large spike),
   and set the threshold between them.

Once verified and tuned, these settings are fixed and the operator only ever uses
the button and reads the status light.

---

*For the firmware internals (pin assignments, functions, constants, and tuning
values), see `PLAN.md`.*
