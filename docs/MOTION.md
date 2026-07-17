# Onboard Motion (MV2 pose-target executor)

The server no longer streams raw wheel velocities. It streams **MV2
pose-target frames** — "you are here (vision), be there (target), in this
mode, under these caps" — and the robot generates and follows the trajectory
itself. Wheel motion is produced at the motor rate (100 Hz today, 250 Hz
target), so it never depends on Wi-Fi or vision cadence. The design and every
tuned constant were proven on hardware by the team's Rust robot (phoenix-rf)
and its C++ server (phoenix-core); the executor here is vendored 1:1 from
that stack (`Motion/phx/`, same author, relicensed GPLv3).

```
UDP 50514            Networks/decode          Motion/MotionBridge
 packets  ──────────►  classify  ── MV2 raw ──►  strict parse, id check,
                        │                        seq dedup, kick edges
                        │ v1 velocity                   │
                        ▼                               ▼ (per motor tick)
              Supervisor::shape_twist          phx::MoveExecutor
                        │                        pose estimate ◄─ FK odometry
                        │                        control law      + pi3hat gyro
                        │                        S-curve shaping
                        ▼                               │ body twist + energize
                  Math/kinematics  ◄────────────────────┘  (shape_twist too)
                        │  inverse()  → motor rev/s
                        ▼
              Telemetry::cycle  → moteus CAN (+ watchdog, vel/accel limits)
                        │  replies: velocity, current, fault, voltage, IMU
                        └────► forward() odometry, Safety supervisor, telemetry v2
```

## Executor pipeline (per motor tick)

1. **Pose estimate** — dead-reckon the last fused pose with wheel-odometry
   translation and gyro heading (`imu.yaw_rate_sign` applies the mounting
   polarity; NaN/absent IMU falls back to odometry yaw). Each accepted frame
   *blends* the server pose in (complementary filter, `pose_gain` /
   `heading_gain`); an error beyond `snap_dist_m` snaps instead (teleport,
   long occlusion).
2. **Control law** — braking-envelope P law toward the target pose with
   settle latches (no twitching at the target), optional `arrive_speed` to
   carry velocity through waypoints.
3. **Shaping** — acceleration- AND jerk-limited (S-curve) twist shaper; the
   per-mode profile caps speed/accel/jerk, and per-frame wire caps can only
   *tighten* it. The operator envelope (`Safety/supervisor.h`) applies on
   top, direction-preserving.
4. **Watchdog** — no fresh frame for `brake_after_ms` → BRAKE tier (active,
   shaped stop); still nothing at `coast_after_ms` → COAST (motor output
   cut). This replaces the legacy 3-empty-poll stop while MV2 is active; the
   v1 velocity path keeps its historical behavior, and a v1 command stands
   the executor down.

## Frame kinds and modes

| Kind      | Meaning                                                        |
|-----------|----------------------------------------------------------------|
| `MOVE`    | Drive to the target pose under the mode profile                |
| `HOLD`    | Actively hold the pose the robot had when HOLD first arrived   |
| `BRAKE`   | Decelerate to zero smoothly-but-quickly, then coast            |
| `DISABLE` | Cut motor output immediately (daemon stays alive)              |

| Mode    | Profile          | Use                                    |
|---------|------------------|----------------------------------------|
| `FAST`  | fast_travel      | Open-field repositioning               |
| `BALL`  | ball_approach    | Closing on the ball                    |
| `ALIGN` | precision_align  | Final alignment for a kick             |
| `HOLD`  | hold_position    | Position hold (also used by HOLD kind) |
| `BRAKE` | hold_position    | Selector for BRAKE frames              |

## Tuning table (config/Motion.yaml — hardware-proven defaults)

| Profile         | max_speed m/s | max_accel m/s² | max_jerk m/s³ |
|-----------------|---------------|----------------|---------------|
| fast_travel     | 2.5           | 2.5            | 15.0          |
| ball_approach   | 0.8           | 1.2            | 8.0           |
| precision_align | 0.35          | 1.0            | 6.0           |
| hold_position   | 0.3           | 1.5            | 10.0          |

Top-level: `brake_after_ms: 300`, `coast_after_ms: 1200`,
`brake_decel_mps2: 3.5`, `pose_gain: 0.35`, `heading_gain: 0.5`,
`snap_dist_m: 0.35`. Every knob has a built-in default in
`Motion/phx/executor.h`; the YAML only overrides.

`imu.yaw_rate_sign`: +1 when the pi3hat gyro reads positive for a CCW (+w)
body rotation. Verified +1 on Robot B (its pi3hat is mounted ~180° flipped).
Re-verify per robot: command `+w` and check the reported gyro sign matches.

## Drive math

`Math/kinematics.h` holds the calibrated geometry: wheel table (mount angles
30/−45/−135/150°; the legacy −130 was a typo that curved straight drives),
true cross-product moment arms, and ONE measured scale constant
`metersPerMotorRev` (Motor.yaml, default 2π·0.0335) that converts wheel
surface speed to moteus **rev/s** (the legacy chain divided by the radius —
rad/s — a hidden 2π error). `inverse()` produces motor setpoints;
`forward()` (least-squares) turns measured motor velocities back into the
body twist used for odometry and telemetry.

## Safety

`Safety/supervisor.h` (port of phoenix-rf `safety.rs`):

* **Envelope** — SAFE/CAPPED/UNSAFE (`-s`/`-c`/`-unsafe` flags) scale the
  commanded twist direction-preservingly; SAFE no longer hard-stops on an
  over-limit command.
* **Trips** — moteus fault codes, over-temperature (70 °C), sustained
  over-current (`currentLimit`, `currentGraceMs` — accel transients don't
  trip, stalls do), bus under-voltage. All latch a **recoverable** estop
  (motors coast, `state=estop` in telemetry) instead of killing the process.
* **Recovery** — automatic after `recoveryS` continuously-healthy seconds
  while the operator commands zero, or explicitly via the `STOP` opcode.
  **STOP no longer exits the daemon** — it safe-stops, clears latches, and
  keeps serving.
* **Hardware failsafe** — every motor command carries a moteus
  `watchdog_timeout` (0.1 s): wheels stop themselves if the Pi hangs.

## Control rate

`Motor_interval` (Main.yaml) is 10 ms (100 Hz). The executor, odometry, and
safety all run in that tick. 4 ms (250 Hz — the rate proven on the Rust
robot) is the target once validated on this hardware; everything in the
chain is rate-independent (real `dt` is measured each tick).

## Host tests

All pure logic is tested off-robot: `tests/host/` builds `host_tests` with
any desktop toolchain (no Pi dependencies) — decoder, kinematics IK/FK,
moteus formats, MV2 wire golden strings, the full executor plant suite,
safety supervisor, and the telemetry v2 encoder.

```
cmake -S tests/host -B tests/host/build
cmake --build tests/host/build
./tests/host/build/host_tests            # optional arg = name filter
```
