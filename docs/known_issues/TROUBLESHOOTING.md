# Robot Troubleshooting Guide — Wheel & Motor Diagnostics

**Last updated:** 27/05/2026

This document outlines how to troubleshoot and isolate faults in the wheel/motor stack on the robot. The stack is built on the mjbots ecosystem: a Raspberry Pi running our software, a pi3hat daughterboard providing the CAN-FD interfaces, and moteus brushless motor controllers driving each wheel motor.

---

## When to use this guide

If a wheel is unresponsive, there are three possible causes:

1. The **motor controller** (moteus)
2. The **motor** itself (or its wiring/encoder)
3. The **pi3hat** (or the CAN wiring between it and the controller)

The tools below let you isolate which of these is at fault. The golden rule: **work outward from the pi3hat.** Confirm the bus is healthy first, then talk to the controller, then worry about the motor.

---

## Diagnostic tools

Three tools are available on the robot.

### `pi3hat_tools`

This is our local build of mjbots' reference `pi3hat_tool` (from `pi3hat/lib/cpp/mjbots/`), bundled into our repo build process. After a normal build it lives in the `build/` folder — no separate install needed.

It talks directly to the pi3hat's CAN-FD hardware, bypassing the moteus stack entirely. Use it to confirm the pi3hat itself is healthy *before* you start blaming the motor or controller.

Like all pi3hat tooling it must be run as root (it bypasses the Linux SPI drivers), so prefix with `sudo` or run from a `sudo bash` shell.

Typical uses:

- **Scan a bus for connected devices.** Confirms which IDs are alive on which JC bus. If a controller doesn't show up here, the problem is upstream of the controller — wiring, power, or pi3hat — not the motor.
- **Send raw CAN frames** to a specific bus and ID. Useful for sanity-checking that the pi3hat can transmit on a given bus without involving the higher-level moteus stack.
- **Read raw CAN frames** coming back from the bus. Lets you see whether a controller is replying at all, even if `tview` or `moteus_tool` are failing to talk to it.
- **Read the IMU/attitude.** Confirms processor 3 (IMU + low-speed CAN + nrf24l01) is alive: `./pi3hat_tool --read-att`.
- **Verify bus mapping.** If you're unsure which physical JC port a controller is wired to, probe each bus in turn to find it.

Common invocations (your exact flag names may differ slightly depending on the build — run `./pi3hat_tool --help` to confirm):

```bash
sudo ./build/pi3hat_tools/pi3hat_tool --help
sudo ./build/pi3hat_tools/pi3hat_tool --read-att          # IMU attitude — confirms processor 3 alive
sudo ./build/pi3hat_tools/pi3hat_tool --can-send 1,8001,<hex>   # send a raw frame on bus 1
sudo ./build/pi3hat_tools/pi3hat_tool --read-can 1        # read incoming frames on bus 1
```

**Rule of thumb:** if `pi3hat_tools` can see the controller on a bus, the pi3hat and CAN wiring are fine — move on to `moteus_tool` / `tview`. If `pi3hat_tools` *cannot* see it, the fault is in the pi3hat, the CAN wiring, the controller's power supply, or the controller's CAN ID. Do not waste time on motor calibration yet.

### `moteus_tool`

Used to connect to a moteus motor controller on a specific bus and ID, and to configure or troubleshoot the controller and its attached motor. This is the Python-based mjbots command-line utility, installed into our `moteus-venv` virtual environment (see Setup below).

Common invocations:

```bash

# Run motor calibration (motor must be able to spin freely)
moteus_tool --target <id> --calibrate

# Open an interactive diagnostic console
moteus_tool --target <id> --console

```

`--target` accepts a comma-separated list when you want to operate on several controllers at once (e.g. `--target 1,2,3`).

If the controller is on a pi3hat bus other than JC1, you must combine these with `--pi3hat-cfg` (see next section).

### `--pi3hat-cfg`

This is a command-line **flag** (not a separate program) accepted by both `moteus_tool` and `tview`. It tells those tools which pi3hat bus each CAN ID lives on. Without it, the tools default to **port JC1**, which is fine if all controllers are on JC1 but silently broken otherwise.

Syntax:

```
--pi3hat-cfg 'BUS1=ID1,ID2,...;BUS2=ID3,ID4,...'
```

The bus number refers to the pi3hat's JC port (1 = JC1, 2 = JC2, etc.).

Example — controllers 1 and 6 on JC1, controllers 3 and 7 on JC2:

```bash
sudo bash
source moteus-venv/bin/activate

tview --pi3hat-cfg '1=1,6;2=3,7' -t 1,3,6,7
moteus_tool --pi3hat-cfg '1=1,6;2=3,7' --target 3 --info
```

> **Note:** Recent versions of the moteus Python library (≥ 0.3.x with UUID-based addressing) auto-probe all pi3hat channels if `--pi3hat-cfg` is omitted, so this flag is increasingly optional. It is still useful when you want to constrain probing to specific buses, or when working with older library versions. If you see auto-discovery behaving oddly, fall back to specifying `--pi3hat-cfg` explicitly.

### `tview`

Worth mentioning alongside `moteus_tool` because it ships in the same package. `tview` is the interactive GUI: a live telemetry tree, real-time plots, editable configuration parameters, and a console that speaks the moteus diagnostic protocol. Run with the same `--pi3hat-cfg` and `-t <id>` (or `--devices`) flags as `moteus_tool`. Use it when you want to *watch* a controller behave rather than send one-shot commands.

---

## Setup — getting `moteus_tool` and `tview` working on the robot

The pi3hat C++ library bypasses the kernel SPI driver, which means **everything that talks to the pi3hat must run as root**. The standard pattern is:

1. **First-time setup only** — run our setup script. This creates `moteus-venv` (a Python virtual environment using system site packages so the Raspberry Pi headers are visible) and installs the `moteus` and `moteus-pi3hat` packages into it:

   ```bash
   bash Moteus_setup.sh
   ```

   Roughly equivalent to the upstream mjbots instructions:

   ```bash
   sudo apt install libraspberrypi-dev
   python -m venv --system-site-packages moteus-venv
   source moteus-venv/bin/activate
   pip3 install moteus moteus-pi3hat
   ```

2. **Every time you want to use the tools** — get a root shell, then activate the venv inside that shell:

   ```bash
   sudo bash
   source moteus-venv/bin/activate
   ```

   Your prompt should change to indicate `(moteus-venv)`. You can now run `moteus_tool`, `tview`, and any of our Python scripts that use the pi3hat transport.

> **Why root?** The pi3hat C++ library bypasses the Linux SPI driver to hit tight real-time deadlines (busy-waits, `isolcpus`, `chrt 99`, `mlockall`). That requires root. There is no way around this — don't try to `chmod` your way out.

---

## Fault isolation flow

When a wheel won't respond, walk this in order. Don't skip steps.

Can `pi3hat_tools` see the controller's bus?**

```bash
sudo ./build/pi3hat_tools/pi3hat_tool --read-att
```

- If this fails or hangs → the pi3hat itself, the ribbon header, or the Pi's SPI is broken. Stop and fix that first.
- If it succeeds → the pi3hat is alive. Continue.


Send a raw CAN frame to the controller's ID on the bus you think it's on, and listen for a reply. If you see nothing on any bus, it is one of:

- Controller is unpowered (check XT30, fuses, battery).
- CAN wiring fault (JST PH-3 connector unseated, CANH/CANL swapped, broken solder joint).
- Controller is on a different bus than you assumed → probe each JC port.
- Controller's CAN ID is not what you expect.

Does `moteus_tool --console` work?**

```bash
moteus_tool --pi3hat-cfg '<bus>=<id>' --target <id> --console
```

- If this fails but step 2 worked → check `--pi3hat-cfg` syntax, check the venv is active, check you're running as root.
- If this works → the comms path is fine, the problem is in the motor or the controller's configuration.

What does the fault say?**

If the controller is reporting a fault, read the relevant diagnostic channel

If everything checks out but the motor still won't spin:**

- Confirm calibration: a fresh controller has no commutation calibration. Run `moteus_tool --target <id> --calibrate` with the motor free to spin.
- Check `servopos.position_min` and `position_max` — a too-narrow range will trap commanded positions.
- Check `servo.max_current_A` — a too-low limit will silently cap torque.

---


## References

- mjbots moteus reference: <https://github.com/mjbots/moteus/blob/main/docs/reference.md>
- mjbots pi3hat reference: <https://github.com/mjbots/pi3hat/blob/master/docs/reference.md>
- moteus getting started: <https://github.com/mjbots/moteus/blob/main/docs/getting_started.md>
- mjbots blog (release notes, new flags): <https://blog.mjbots.com>
