# BUG-ID-4 — kicker never fires (commanded kicks are silent)

**Last updated:** 19/08/2026

---

## Symptom

The robot never physically kicks, even though the server/tool reports a kick
and MatchFeedback's `kick_counter` toggles and `kicker_level_v` reads ~full.

---

## Cause

Three independent faults were found stacked on one robot (2026-08-19); any one
of them alone keeps the kicker silent. Work through all three:

1. **Legacy Arduino firmware.** An old flash of `Arduino/Arduino.ino`
   understands only the one-byte legacy `'K'` command and **silently ignores**
   the parameterized `'k'+<ms>` (and `'d'` dribbler) commands that
   RobotFramework sends for every MatchCtrl kick. Result: every match kick is
   a no-op. The current sketch handles both and prints a boot banner
   (`RF-KD v2 ready, kicker active=…`).
2. **Wrong kicker polarity.** The old perf board fires on an **active-LOW**
   pulse; the newer board on **active-HIGH**. With the wrong `kickerActiveLevel`
   the idle level holds the discharge path closed, the capacitor never charges,
   and kicks are silent. This is **per-board** — verify for each chassis.
3. **Pulse too short.** The old board dumps the capacitor through the solenoid
   and needs a long drive (**≥150 ms**); the framework's computed 10–25 ms
   pulses do nothing. The sketch clamps to `kickerMinPulseMs`.

Also important, not bugs but easy to misread:

- **`kick_counter` is not proof of a physical kick** — it toggles when the
  firmware *decides* to fire (`Motion/actuators.cpp`), not when the solenoid
  moves. `kicker_level_v` is a recharge-time model, not a measurement.
- **Recharge takes ~90 s**, and the firmware rejects kicks within 5 s of each
  other or of boot ("Kicking frequnecy is too fast").
- **The Arduino glitch-fires on every reset** — a serial reconnect (service
  restart, any pyserial open toggling DTR) can fire a charged capacitor. Keep
  hands clear of the kicker whenever the cap may be charged.

---

## Fix

1. Reflash with the current `Arduino/Arduino.ino` (arduino-cli, FQBN
   `arduino:megaavr:nona4809`, port `/dev/ttyACM0`). Confirm the boot banner.
2. Set `kickerActiveLevel` to match this chassis's board (LOW = old perf
   board, HIGH = new kicker) and reflash. Verify a real kick after a full
   recharge — there is no onboard sensor, so a human must watch it fire once.
3. Leave `kickerMinPulseMs` in place so short requested pulses still fire.

To fire manually for testing (service stopped), open `/dev/ttyACM0` at 115200,
wait out the recharge, and send `b'k' + bytes([200])` (or legacy `b'K'`).
