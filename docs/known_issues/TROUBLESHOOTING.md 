# Robot Troubleshooting Guide — Wheel & Motor Diagnostics

**Last updated:** 26/05/2026

This document outlines the steps you can take to troubleshoot and isolate faults on the robot.

---

## When to use this guide

If the wheels are unresponsive, there are three possible causes:

1. The **motor controller**
2. The **motor** itself
3. The **pi3hat**

The tools below help you isolate which of these is at fault.

---

## Diagnostic tools

We have three known diagnostic tools available on the robot:

### `pi3hat_tools`

Built directly as part of our repo build (it lives in the `build/` folder after a normal build, no separate setup required). Use this to test the state of the CAN buses on the pi3hat itself — i.e. to confirm the pi3hat is healthy *before* you start blaming the motor or controller.

Typical uses:

- **Scan a bus for connected devices.** Confirms which IDs are alive on which JC bus. If a motor controller doesn't show up here, the problem is upstream of the controller (wiring, power, or pi3hat fault) — not the motor.
- **Send raw CAN frames** to a specific bus and ID. Useful for sanity-checking that the pi3hat can transmit and receive on a given bus without involving the higher-level moteus stack.
- **Read raw CAN frames** coming back from the bus. Lets you see whether a controller is replying at all, even if `tview` or `moteus_tool` are failing to talk to it.
- **Verify bus mapping.** If you're unsure which physical JC port a controller is wired to, `pi3hat_tools` lets you probe each bus in turn to find it.

**Rule of thumb:** if `pi3hat_tools` can see the controller on the bus, the pi3hat and wiring are fine — move on to `moteus_tool` / `tview`. If `pi3hat_tools` *cannot* see it, the fault is in the pi3hat, the CAN wiring, or the controller's power — do not waste time on motor calibration yet.

### `moteus_tool`

Used to connect directly to a motor controller on a specified bus and ID, and to configure or troubleshoot the motor and motor controller.

### `pi3hat-cfg`

A flag/config helper used alongside `moteus_tool` and `tview` to tell them which pi3hat bus to talk on (see usage below).

---

## Setup — getting `moteus_tool` working on the robot

To use `moteus_tool` on our robot:

1. **Run the setup script:**
```bash