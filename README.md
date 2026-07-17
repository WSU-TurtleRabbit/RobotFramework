# RobotFramework
Robot Client in C++

## Onboard motion, protocol, and safety

The robot now runs its own trajectory executor: the server streams **MV2
pose-target frames** and the robot generates and follows the motion locally
at the motor rate, with wheel-odometry + gyro pose estimation, S-curve
shaping, and a brake/coast command watchdog. A recoverable safety supervisor
(speed envelope, per-motor trips, auto-recovery) and telemetry protocol v2
(`mv=2` capability flag) complete the loop. The executor and constants are
ported from the team's hardware-proven phoenix-rf / phoenix-core stack.

* [docs/MOTION.md](docs/MOTION.md) — architecture, modes, tuning table
* [docs/PROTOCOL.md](docs/PROTOCOL.md) — MV2 + telemetry v2 wire formats
* `tests/host/` — Pi-free unit tests for all pure logic (decoder,
  kinematics, executor, safety, telemetry): `cmake -S tests/host -B
  tests/host/build && cmake --build tests/host/build && ./tests/host/build/host_tests`

Legacy 7-field velocity commands still work unchanged; a v1 command takes
the robot back to direct control. Operator `STOP` now safe-stops and clears
safety latches while the daemon keeps running.

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