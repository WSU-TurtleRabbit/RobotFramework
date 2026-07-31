# Onboard Motion (MatchCtrl / TIGERs architecture)

The robot speaks the Phoenix **MatchCtrl** protocol and owns all fast,
precise motion — the same split that makes TIGERs Mannheim's robots move
the way they do. The server sends one MatchCtrl frame per robot per camera
tick (freshest vision pose + its measured age + a **skill**); the robot
fuses the delayed vision with its own encoders + gyro, regenerates its own
trajectory every control tick, and runs the kicker/dribbler itself. Wheel
motion is produced at the control rate (**250 Hz**), so it never waits on
Wi-Fi.

Everything between the wire and the wheels lives in pure, host-tested
modules (`Networks/matchctrl`, `Motion/skills`, `Motion/estimator`,
`Motion/trajectory`, `Motion/controller`, `Motion/actuators`), orchestrated
per tick by `Motion/match_bridge`. The superloop (`RobotFramework.cpp`) is
thin glue: sockets, CAN, serial, and the safety supervisor.

```
UDP 50514          Networks/matchctrl          Motion/match_bridge
 MatchCtrl ───────►  strict binary decode ───►  seq dedup, robot id,
 + legacy v1 text    (Networks/decode: STOP,    watchdog tiers, kind switch
                      PING, v1 fallback)              │
                              ┌───────────────────────┘
                              ▼ per control tick (250 Hz)
   Motion/skills      skill -> setpoint + limits (+ KD field)
   Motion/estimator   delayed-vision fusion  ◄── FK odometry + pi3hat gyro
   Motion/trajectory  per-tick bang-bang regeneration (pose skills)
   Motion/controller  Panthera cascade -> wheel rev/s (+ FF torque, opt.)
   Motion/actuators   KD field -> kicker ARM/FORCE + dribbler ESC
                              │
                              ▼
   Telemetry::cycle  moteus CAN (velocity mode, watchdog, vel/accel limits,
                     feedforward torque) + pi3hat IMU sample
                              │
                              ▼
   MatchFeedback     matched-slot pose, traction, kicker model, battery,
                     barrier, features — 50 Hz to the server (UDP 50513)
```

The same flow as a picture:

![The 250 Hz per-tick pipeline](images/tick-pipeline.svg)

## The per-tick sequence (TIGERs `robot.c` order)

1. **Read sensors** — moteus encoder velocities (previous CAN cycle) → body
   twist via forward kinematics; pi3hat gyro yaw rate (`imu.yaw_rate_sign`
   applies the mounting polarity; NaN gyro falls back to odometry yaw).
2. **Parse the latest MatchCtrl** — skill → a normalized setpoint + limits
   (`Motion/skills`), vision pose + `posDelay` queued for the estimator.
3. **State estimation** (`Motion/estimator`) — the delayed-vision trick:
   - A ring of 128 time slots (one per control tick) holds each tick's gyro
     rate, wheel odometry, and the dead-reckoned INS state `[x, y, θ, vgx, vgy]`.
   - A vision pose is inserted into the slot `posDelay + capture_delay`
     (20 ms) **in the past**: a steady-state Kalman correction (position
     innovation also corrects velocity — that pays back wheel slip) runs at
     that slot, then the slots up to the present are **replayed** through
     their stored inputs and dragged toward the corrected history with a
     tracking gain, so the output moves smoothly instead of snapping.
   - Outliers are gated (a jump implying > 3 m/s of correction speed); a run
     of rejections means *we* were lost → snap. Vision timeout (1 s)
     dead-reckons on encoders + gyro; the first fix back snaps.
   - Heading integrates the gyro with a fixed-gain vision correction.
4. **Trajectory regeneration** (`Motion/trajectory`, pose skills) — a fresh
   synchronized bang-bang is planned **every tick from the reference state**
   (the last trajectory sample — smooth by construction), never from the
   noisy estimate. 2D alpha-bisection sync (both axes arrive together; the
   same planner the server mirrors in `phoenix/core/trajectory.py`), a 1D
   orientation profile, the **centrifugal ω limit** (`|ω| ≤ centAccMax /
   |v_xy|`), and a light low-pass on the orientation target. FAST_POS slaves
   the heading to the drive direction and raises accel once aligned;
   `primaryDirection` keeps the robot's preferred axis along travel. At
   launch the target displacement defines travel before velocity exists, and
   the pose-alignment gate shares traction with yaw before releasing full
   translation. This prevents the real chassis's large sideways launch arc
   without lowering the skill's velocity ceiling.
5. **Control law** (`Motion/controller`, TIGERs "Panthera") — trajectory
   velocity FF (+ accel lead) **+ P** on the reference-vs-estimate position
   error (global frame, clamped) → world velocity; **P** heading + **P**
   yaw-rate vs the gyro → yaw command. Rotated into the body frame by the
   robot's own heading estimate, then through the inverse kinematics. Output
   slew (120 rev/s²) + saturation (45 rev/s) + NaN guard. Velocity skills
   shape through an accel+jerk S-curve; EMERGENCY ramps to zero at 8 m/s²
   then coasts. Model feedforward (coulomb + viscous + mass → per-wheel
   torque) is implemented but **off until identified** (see *Commissioning*).
6. **Actuators** (`Motion/actuators`) — ARM is continuous and fires
   edge-triggered on the robot's own ball-contact signal; FORCE fires on its
   activation edge; DISARM clears; ARM_TIME uses the raw discharge duration.
   Kick speed maps to solenoid pulse ms (config). Dribbler speed maps to ESC
   microseconds.
7. **MatchFeedback** at 50 Hz — pose/vel from the estimator's
   measurement-matched slot (TIGERs semantics), kicker recharge model,
   dribbler speed + traction, battery, barrier bit, feature bits.
8. **Watchdogs** — 1 s without an accepted MatchCtrl → onboard EMERGENCY
   (ramp, then coast). First vision fix gates all motion. The moteus
   per-command watchdog (0.1 s) and the recoverable supervisor trips
   (over-current, over-temp, under-voltage, fault) stay armed below
   everything.

### The estimator, the cascade, and the safety net — visually

![Delayed-vision fusion: fix the past, replay to the present](images/estimator-fusion.svg)

![The Panthera control cascade](images/controller-cascade.svg)

![The six safety tiers](images/safety-tiers.svg)

## Control rate

**250 Hz (4 ms)** — the rate phoenix-rf proved on this exact hardware
(~0.7 ms CAN cycle, ~0.014 ms loop jitter). Set by `Motor_interval` in
`config/Main.yaml`. The estimator's slots are one control tick each (128
slots = 512 ms of fusion history, far deeper than the ≤ 83.5 ms insertion).

## Ball contact (the break-beam stand-in)

These robots have no IR break-beam. The onboard ball camera is the contact
sensor: a big, centered, confident, fresh detection = ball at the mouth
(`actuators.barrier_*` thresholds). It triggers ARM kicks and reports as the
MatchFeedback barrier bit; with the dribbler-traction ladder
(OFF/IDLE/STRONG) the server's contact fallback chain keeps working. A real
break-beam wired to a spare Arduino pin would be better — the policy module
already isolates the decision, so only `ball_contact()` changes.

## Configuration

`config/Motion.yaml` — every knob with comments: `imu.yaw_rate_sign`
(**verify per robot**: +1 on Robot B), `match.*` (timeout, vision gate),
`estimator.*` (fusion), `trajectory.*` (regeneration), `controller.*`
(cascade gains + model FF), `actuators.*` (kicker/dribbler mappings,
contact thresholds), `feedback.*` (battery window, hardware id).
`config/Main.yaml` — intervals + `Robot_id`. `config/Safety.yaml` —
envelope, trips, moteus watchdog. `config/Motor.yaml` — motor map,
`metersPerMotorRev` (the ONE drive-scale calibration), moteus vel/accel
limits.

### Active Robot A calibration

`config/Motion.yaml` carries the **field-validated RLearn seed-20260805
champion**. Robot A completed its guarded campaign on 2026-07-30. Longitudinal
and diagonal motion may use a 2.5 m/s velocity ceiling with at most 2.0 m/s^2
translation acceleration. Pure lateral motion is limited to 1.5 m/s and
1.5 m/s^2 because the 2.0 m/s lateral stage exceeded the 0.12 m corridor.
Arbitrary simultaneous pose moves use at most 1.5 m/s, 1.0 m/s^2 translation
acceleration, and 3.0 rad/s^2 yaw acceleration; heading changes near or above
90 degrees use a 1.25 m/s velocity ceiling. Pre-aligned approaches retain the
2.5 m/s velocity ceiling.

The complete machine-readable record is
`config/motion_calibrations.json`. It includes:

- the active RLearn values and the field-proven 2026-07-23 rollback values;
- hashes of the champion, dataset manifest, held-out model report, and seed
  report;
- robust simulated limits and held-out wheel-model error;
- the required `0.5 → 1.0 → 1.5 → 2.0 → 2.5 m/s` field-test stages.

Verify or switch the repository configuration without hand-editing YAML:

```bash
python tools/select_motion_calibration.py rlearn-champion-20260805 --check
python tools/select_motion_calibration.py field-proven-20260723
python tools/select_motion_calibration.py rlearn-champion-20260805
```

Applying a profile writes `config/Motion.yaml.before-calibration` before a
change. That local snapshot is an extra convenience; the authoritative
rollback remains the field-proven profile in the registry. The registry
records the accepted field envelope and retained rollback.

## Commissioning (on hardware, in order)

1. `estop.py` within reach. `monitor.py` to watch MatchFeedback.
2. `phoenix-server/phoenix/tools/commission.py` — `wheel` (WHEEL_VEL) to
   verify motor order + signs, `vel` (LOCAL_VEL) for body axes, `sine`
   (SINE skill) for system ID: the response fits the model-FF constants
   (`robot_mass_kg`, `friction_*`, `drivetrain_efficiency`) and the gyro
   sign. Only then flip `controller.model_ff_enabled: true`.
3. Calibrate `metersPerMotorRev` (command known revs, measure travel) and
   the actuator mappings (`kick_ms_per_mps`, `dribble_us_per_mps`,
   `barrier_radius_px` per robot/camera).
4. GLOBAL_POS precision moves, then go-to-ball-and-kick.

## Host tests

Pure logic is tested on any desktop (`tests/host/`, MSVC or gcc):

```
cmake -S tests/host -B tests/host/build -G "NMake Makefiles"
cmake --build tests/host/build && tests\host\build\host_tests.exe
```

Golden byte vectors come from `phoenix-server`'s own encoder
(`tests/host/golden_matchctrl.py`); the planner is pinned to the server's
`trajectory.py` numbers; and a simulated-plant test drives the whole
cascade (wire → skills → fusion → trajectory → control → wheels) to a
GLOBAL_POS target within 3 cm / 0.03 rad under 60 Hz / 40 ms-delayed noisy
vision.
