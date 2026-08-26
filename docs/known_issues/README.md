# Robot Bug Index

**Last updated:** 19/08/2026

This folder contains an index of all known bugs associated with the robots.

Use the symptom descriptions below to identify which `BUG-ID` file to consult for details.

---

## Bug Index

### BUG-ID-1 — pi3hat: code fails to run despite responsive controller

**Symptom:** When using a pi3hat, our code does not run the motor, even though the controller was verified as responsive in `tview` before launch.

### BUG-ID-2 — pi3hat: get "processor "aux" has incorrect CAN SPI version 255 != [2,3,4]"

**Symptom:** When using pi3hat_tools I get an error and cannot run the code.

### BUG-ID-3 — motor 1 (front-right) stalls at high current / "seized wheel"

**Symptom:** The front-right wheel won't drive under load (near-zero rev/s while current trips at 10–14 A) while motors 2/3/4 are fine; on the field the robot freezes/"stops short" mid-move as the onboard supervisor latches. Often a moteus commutation miscalibration (software-fixable with a recal), sometimes a mechanical bind.

### BUG-ID-4 — kicker never fires (commanded kicks are silent)

**Symptom:** The robot never physically kicks even though `kick_counter` toggles and the kicker voltage reads full. Caused by stacked faults: legacy Arduino firmware ignoring parameterized kicks, wrong per-board kicker polarity, and/or too-short pulses.
