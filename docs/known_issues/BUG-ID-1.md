# BUG-ID-1 — pi3hat: code fails to run despite responsive controller

**Last updated:** 26/05/2026

---

## Symptom

You have completed the following sequence with the pi3hat and it does not work:

1. You use `tview` to confirm the motor controller is working — and it responds correctly.
2. Once the controller is verified as responsive, you run the `motor_test` code (as of 26/05/2026).
3. As soon as `motor_test` runs, the motors stop running and the controller becomes unresponsive.

---

## Cause

This issue has two contributing reasons:

- **Older pi3hat firmware fault behaviour.** On older versions of the pi3hat firmware, if you send packets to buses that are not connected to anything, the pi3hat will stop working and enter a **soft fault state**.
- **Bus becomes fully unusable in this state.** Once in the soft fault state, you cannot use any of the buses at all. The only way to reset it is to **power-cycle the pi3hat** (turn it off and back on).

---

## Recovery

If you power-cycle the pi3hat, you will be able to use `tview` to communicate with the motor controller again.

---

## Fix

You have two options to prevent this from happening again:

1. **Ensure all buses specified in your code are populated.** If any specified bus has nothing connected to it, you will re-enter the same fault state.
2. **Verify the cause using `SingleMotorTest` (as of 26/05/2026).** Running this executable maps out the buses and IDs for the pi3hat. If the pi3hat works correctly with `SingleMotorTest`, you can safely conclude that the unpopulated-bus fault is the issue.