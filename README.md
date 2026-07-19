# RobotFramework
Robot Client in C++

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

* [docs/MOTION.md](docs/MOTION.md) — the cascade, config knobs, commissioning
* [docs/PROTOCOL.md](docs/PROTOCOL.md) — MatchCtrl/MatchFeedback wire formats
* `tests/host/` — Pi-free unit tests for all pure logic (wire, skills,
  estimator, trajectory, controller, bridge, feedback, kinematics, safety):
  `cmake -S tests/host -B tests/host/build && cmake --build tests/host/build && ./tests/host/build/host_tests`

Legacy 7-field velocity commands and `STOP`/`PING` still work as bench
fallbacks; a v1 command takes the robot back to direct control until the
next MatchCtrl frame.

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