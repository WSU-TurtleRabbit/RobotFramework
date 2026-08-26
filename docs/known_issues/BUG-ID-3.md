# BUG-ID-3 — motor 1 (front-right) stalls at high current / "seized wheel"

**Last updated:** 19/08/2026

---

## Symptom

The front-right wheel (CAN motor 1) will not drive under normal load, while
motors 2/3/4 are fine:

1. A bench velocity command of ~1.0 rev/s produces almost no rotation
   (~0.04–0.16 rev/s) while the q-current ramps to the trip limit
   (10–14 A) within ~100 ms and aborts.
2. On the field this shows up indirectly: motor 1 exceeds the supervisor's
   sustained over-current limit (10 A / 300 ms, `config/Safety.yaml`), which
   **latches the onboard e-stop**. The MOVE feature bit drops mid-move, the
   robot freezes/holds and auto-recovers after ~3 s of idle. Tools that keep
   commanding it see the robot "stop short", "freeze", or (during a ball
   approach) end up in the wrong place — often misread as a software bug.

Seen on both the original Robot 5 chassis and a second chassis (2026-08-19),
always at the **same front-right position** — treat a motor-1 stall as
expected-and-diagnosable, not a mystery.

---

## Cause

Two distinct causes present with the same symptom — distinguish them with a
**gentle low-torque probe** (command ~0.5 rev/s at a 0.4 Nm cap):

- **Commutation miscalibration (software, common).** At low torque the motor
  turns *smoothly* but draws ~2–3× the current of a healthy wheel. The
  moteus commutation calibration is wrong, so the controller drives the
  phases inefficiently and the motor fights itself. This is fully recoverable
  in software.
- **Mechanical bind (hardware).** At low torque the motor *still* barely moves
  and current pins at the cap — a seized bearing, debris in an omni sub-roller,
  or the wheel rubbing the chassis. No software fix helps; needs physical
  repair.

---

## Recovery / Fix

**First, try a commutation recalibration** (this fixed the second chassis on
2026-08-19: motor 1 went from a 13 A stall to 0.28 A at 1.0–1.5 rev/s, matching
the other three wheels). Stop `robotframework.service` first for exclusive CAN
access, then, from the moteus venv:

```bash
# back up the current config FIRST (restore via dump-config restore mode)
python -m moteus.moteus_tool --force-transport pi3hat --pi3hat-cfg '1=1' \
    --target 1 --dump-config > motor1-config-backup.cfg

# recalibrate (requires the wheel to have freedom of motion)
python -m moteus.moteus_tool --force-transport pi3hat --pi3hat-cfg '1=1' \
    --target 1 --calibrate
```

Then re-verify with a bench spin (`~/wheelspin.py 1 1.0`): a healthy wheel
tracks the commanded velocity at ~0.3–2 A, no trip.

**If recal does not help** (motor still won't turn at low torque, or the drag
returns after a short run), the wheel is mechanically bound — inspect the
bearing, omni sub-rollers, and chassis clearance and repair physically. Do not
keep driving it: sustained high current overheats motor 1.

---

## Related

- `RobotFramework/config/Safety.yaml` — the 10 A / 300 ms trip that latches on
  the stall (protective; do not raise it to "fix" the symptom).
- `Telemetry/match_feedback.cpp` — the MOVE feature bit that drops when the
  supervisor latches.
