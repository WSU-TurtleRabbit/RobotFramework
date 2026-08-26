# Adaptive motion audit and staged integration plan

Date: 2026-08-20. Scope: software motion only. No persistent Moteus
configuration or physical component is changed by this work.

## Existing deployed control path

Phoenix Server runs a deterministic 100 Hz loop. Strategy and operator skills
produce `MoveIntent` objects; `SafetyGovernor` is the final server authority;
`RobotLink` then encodes a complete little-endian MatchCtrl datagram and sends
it over UDP to the configured robot address on port 50514. Each fresh SSL-Vision
capture is attached once with its measured receive/pipeline age. MatchFeedback
returns on port 50513 at 50 Hz.

RobotFramework drains UDP with latest-wins semantics. `MatchBridge` validates
the robot id and rolling sequence, applies a pending delayed vision pose to a
128-slot estimator history, replays wheel odometry and gyro data to the present,
regenerates a bang-bang pose trajectory, and runs the Panthera-style
feed-forward plus proportional controller. `Kinematics::inverse` maps the body
twist to motor output rev/s in CAN order `[FR, RR, RL, FL]`. `Telemetry::cycle`
sends one Pi3Hat CAN-FD transaction to four Moteus controllers in local velocity
mode and receives the next feedback sample. The normal loop is 250 Hz (4 ms).

Frames and units are fixed and tested: global SSL coordinates are metres with
X/Y in the field frame and heading CCW-positive; the robot body frame is
forward X, left Y, CCW yaw. MatchCtrl uses mm, mrad and Q6.2 ms on the wire.
Motor commands and feedback are output rev/s. All new logs use SI units and
monotonic timestamps.

## Feedback and timing actually available

* SSL-Vision supplies pose, camera/capture timing and source identity to
  Phoenix. The current MatchCtrl protocol supplies pose, camera id and age to
  the robot, but not detector confidence.
* The Pi3Hat transaction supplies quaternion attitude, angular rate and linear
  acceleration. Existing RobotFramework exposes angular rate and attitude;
  this integration also surfaces the already-available acceleration values.
* Each Moteus default query already requests wheel position and velocity; the
  deployed code previously surfaced velocity, q-current, bus voltage,
  temperature, mode and fault. This integration also surfaces that existing
  position field without changing persistent controller configuration.
* Commands, estimator/reference state and 20 Hz motor samples were previously
  written to unkeyed text logs. The map iteration order made them parseable but
  not a versioned contract. There was no controller deadline, residual,
  intervention, estimator-confidence, or policy-version record.
* RobotFramework is one superloop plus a 10 Hz camera thread. CAN, estimation,
  planning and actuation are synchronous in the superloop. The logger buffers
  but flushes from the caller. Phoenix vision/referee/API drains and command
  sending are one deterministic process; UI I/O is isolated.

## Existing failure behaviour retained

Phoenix sends EMERGENCY for lost vision, HALT, unsafe stopping envelopes, or
shutdown. RobotFramework requires a first vision fix; one second without a
fresh MatchCtrl enters a controlled emergency brake and then coasts. A 100 ms
per-command Moteus watchdog is independent of both processes. The onboard
supervisor latches sustained current, temperature, voltage and controller
fault trips, and explicit STOP clears the latch. Non-finite estimator or wheel
commands already fail closed. These paths remain authoritative and learned
logic cannot bypass them.

## Baseline on this workstation

Before adaptive-control edits: RobotFramework's prebuilt host binary passed
130 tests / 2,754 checks; `replaytest` passed 8 tests; `testing/rlearn` passed
18 tests. Phoenix passed 211 of 213 tests. Its two pre-existing failures are
configuration assertions that still expect robot 5 at `.129` rather than the
current `.201`, and a stopping-inset bound smaller than the currently configured
4 m/s / 0.18 s / 5.5 m/s^2 envelope. Ruff is not installed. The supplied MSVC
batch file references a BuildTools path absent from this host, so the existing
prebuilt test executable was used for the C++ baseline.

## Design selection

The runtime learner is robust, diagonal recursive least squares with a
forgetting factor. It identifies first-order longitudinal, lateral and yaw
response, separate braking effectiveness, drag, direction-dependent response,
slip, battery scale and bounded actuation delay. Huber innovation weighting,
excitation/saturation/disturbance gates, covariance confidence, per-sample
change limits and hard physical bounds prevent one event from rewriting the
model. Low confidence decays parameters toward conservative defaults. Geometry
and calibrated wheel scale remain separate and frozen.

The deterministic controller remains the baseline. Identified response only
adds a small confidence-gated body-acceleration/velocity correction. A compact
residual policy may propose a still smaller correction in `shadow` or `bounded`
mode. A final deterministic safety filter applies residual slew, velocity,
acceleration, jerk and wheel-speed limits, rejects non-finite data, watches
deadlines/telemetry/policy health, and can disable RL after repeated
interventions. `off` is the shipped default.

Training is offboard. The existing RLearn plant ensemble and real-data
chronological holdouts are extended with a randomized residual environment.
TD3 is selected for bounded continuous residual actions and replay efficiency;
deployment uses only its deterministic actor. Training never runs in the
real-time process and physical random exploration is prohibited. Simulator
and log replay, then shadow benchmarks, are mandatory before promotion.

## Staged integration

1. Add a versioned asynchronous JSONL telemetry contract and surface all
   already-available IMU/Moteus data without changing controller behaviour.
2. Add deterministic straight/rectangle scenario generation, metrics,
   reporting and replay; real execution remains dual-armed and interactive.
3. Add the bounded RLS surface estimator, independent per-robot profiles and
   atomic best-confidence persistence.
4. Split stable body control from wheel mapping, insert adaptive correction,
   residual inference and the final constraint layer, with zero-correction
   compatibility tests.
5. Extend the existing learned simulator with domain randomization and TD3
   residual training/evaluation, champion/challenger atomic promotion and
   rollback.
6. Verify offline baseline/adaptive/shadow/bounded comparisons, timing and all
   repository suites. Physical validation starts later at low speed only after
   explicit operator arming and a live emergency stop.

The Raspberry Pi 4 is appropriate for the fixed-size RLS and small actor
inference at 250 Hz. Model fitting, replay, TD3 updates and report generation
remain offboard so they can never block CAN or motion control.
