# BUG-ID-2 — pi3hat: "aux" processor CAN SPI version error

**Last updated:** 26/05/2026

---

## Symptom

When you try to run any code that uses the pi3hat, you get the following error:

> processor "aux" has incorrect CAN SPI version 255 != [2,3,4]

---

## Cause

Within this repo, we have **disabled the auxiliary port JC5** because of this error. Please verify that this is still the case.

- The relevant variable is `enable_aux` in `mjbots/pi3hat/pi3hat.h`.
- If `enable_aux` is set to `true`, you will get this aux error on any of the newer boards.
- If you are using a virtual environment and running `tview` on a known affected pi3hat, the error will continue to persist until `enable_aux` is disabled.

---

## Fix

The easiest fix is to set `enable_aux = false` in `mjbots/pi3hat/pi3hat.h` to get the main code running.

**Trade-off:** This will make **JC bus 5 unusable**.

---