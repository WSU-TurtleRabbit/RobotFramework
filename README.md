# RobotFramework
Robot Client in C++

## Start here — the visual tour

New to the repo? **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) explains the
whole system in seven diagrams**: the big picture, the code map, the 250 Hz
tick, the delayed-vision trick, the control cascade, the wire format, and the
safety net.

![System overview](docs/images/system-overview.svg)

## Onboard motion, protocol, and safety

The robot runs the **TIGERs Mannheim MatchCtrl architecture**: the server
sends one binary MatchCtrl frame per robot per camera tick (freshest vision
pose + its measured age + a skill), and the robot owns all the fast motion
itself — delayed-vision fusion (time-slot ring + replay), per-tick
bang-bang trajectory regeneration, and a Panthera-style
feedforward+P control cascade ending in per-wheel velocity setpoints on the
moteus controllers, at 250 Hz. The kicker fires on the robot's own
ball-contact signal; MatchFeedback reports the matched-slot pose, traction,
kicker charge model, battery, and health. Safety tiers: 1 s command loss →
onboard EMERGENCY ramp, vision timeout → dead reckoning, motion gated until
the first vision fix, plus the recoverable per-motor supervisor.

* [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — the visual tour (start here)
* [docs/MOTION.md](docs/MOTION.md) — the cascade, config knobs, commissioning
* [docs/PROTOCOL.md](docs/PROTOCOL.md) — MatchCtrl/MatchFeedback wire formats
* [docs/ADAPTIVE_MOTION_AUDIT.md](docs/ADAPTIVE_MOTION_AUDIT.md) — verified
  repository/control-path audit and baseline state
* [docs/ADAPTIVE_MOTION.md](docs/ADAPTIVE_MOTION.md) — RLS adaptation,
  residual-policy ABI, telemetry, constraints and promotion design
* `tests/host/` — Pi-free unit tests for all pure logic (wire, skills,
  estimator, trajectory, controller, bridge, feedback, kinematics, safety):
  `cmake -S tests/host -B tests/host/build && cmake --build tests/host/build && ./tests/host/build/host_tests`

Legacy 7-field velocity commands and `STOP`/`PING` still work as bench
fallbacks; a v1 command takes the robot back to direct control until the
next MatchCtrl frame.

### Robot A RLearn calibration

Robot A uses the field-validated RLearn seed-20260805 motion profile in
`config/Motion.yaml`. The validated fast envelope is a 2.5 m/s velocity
ceiling with at most 2.0 m/s^2 translation acceleration; pure lateral moves
remain limited to 1.5 m/s and 1.5 m/s^2. Arbitrary simultaneous pose moves
use 1.5 m/s, 1.0 m/s^2 translation acceleration, and 3.0 rad/s^2 yaw
acceleration; use 1.25 m/s for turns near or above 90 degrees. Its complete
provenance, simulation, held-out and field metrics, and rollback profile are stored in
`config/motion_calibrations.json`. Use
`tools/select_motion_calibration.py` to verify, apply, or roll back the active
motion values without editing YAML manually. See `docs/MOTION.md` for the
required staged field procedure.

## Building on the Pi

To use this repository, you will need several dependencies.  

First, install the required packages for building and OpenCV:

```bash
sudo apt install build-essential pkg-config libopencv-dev cmake libserial-dev libyaml-cpp-dev
```

This will install the required dependencies to run the ball camera detector.  

Next, install the Raspberry Pi libraries needed for running moteus code:

```bash
sudo apt-get install libraspberrypi-dev raspberrypi-kernel-headers
```

Finally, you will need **CMake** to compile the repository.  
Build the project with the following commands:

```bash
mkdir build
cd build
cmake <repo-destination>
make
```

On a Raspberry Pi running Linux, the repo now has one main setup entrypoint:

```bash
bash ./SETUP.sh
```

That script covers system dependencies, the shared Python virtual environment, moteus package installation, Motor.yaml-based calibration, and project builds. The older helper scripts still exist as wrappers and forward into `SETUP.sh`.

## Driving and diagnosing a robot by hand

These are operator tools, not part of the build, so they are documented here
rather than wired into `SETUP.sh`. They talk to the moteus controllers
directly, so **stop the autonomous control process first** — it holds the CAN
bus and an exclusive `/dev/mem` mmap:

```bash
sudo bash ./STARTUP.sh --stop      # or: sudo systemctl stop robotframework.service
```

Both binaries below are built with the tree and live in `build/`. Run them from
`build/` so the relative `../config/Motor.yaml` resolves, exactly like the main
binary. When you are done, hand control back:

```bash
sudo systemctl start robotframework.service
```

### RemoteControl — hand-drive, straight to the motors

Bypasses vision and the MOVE gate, so it works when SSL-Vision is down or the
supervisor is latched. It has **no field awareness and no envelope** — the
robot goes exactly where you point it and will happily drive off the carpet.
Put it on blocks or keep the area clear. A hard speed cap, `SetStop()` on every
exit path, and the moteus watchdog are the only guards.

```bash
cd ~/RobotFramework/build && sudo ./RemoteControl
#   W/S fwd-back   A/D strafe   Q/E rotate   SPACE stop
#   [ ] speed      K kick       F dribbler   X quit
sudo ./RemoteControl --listen 50600     # headless: drive over UDP, deadman stop
```

### debugger — drivetrain diagnostics (motors stay dead)

```bash
cd ~/RobotFramework/build && ./debugger            # live per-wheel dashboard
./debugger scan          # which CAN id answers on which bus
./debugger selftest      # pre-drive gate (exits non-zero on any fault)
./debugger help
```

Full reference: [`docs/DEBUGGER.md`](docs/DEBUGGER.md).

### debugger commission — supervised remote drive with logging

Drive by keyboard under a safety ceiling derived from the controllers' own
backstops, logging every control cycle and optionally streaming to your laptop:

```bash
sudo bash ./STARTUP.sh --stop
cd ~/RobotFramework
sudo ./build/debugger commission --config-dir config --stream-to <laptop ip>:50600
sudo systemctl start robotframework.service
```

On the laptop (standard-library Python 3, Windows is fine):

```bash
python tools/telemetry_receiver.py --port 50600 --out session.ndjson
```
