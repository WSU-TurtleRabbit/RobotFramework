# Radio Protocol

Plain-text UDP. Commands arrive on port **50514**; telemetry leaves on port
**50513**, addressed to the last commander's IP. Both formats are **pinned**
across the team's repos (this framework, phoenix-rf `wire.rs`, phoenix-core
`robot/wire.h` hold the same golden strings) — change all of them together
or none.

## Commands (server → robot)

### Bare opcodes

| Opcode      | Effect                                                            |
|-------------|-------------------------------------------------------------------|
| `PING`      | Link discovery. Adopts the sender for telemetry. **Not** a drive command — a robot fed only PINGs still times out and stops. |
| `STOP`      | Safe-stop: zero motion, motors stopped, safety latches cleared. The daemon keeps running. |
| `CALIBRATE` | Reserved for the motor commissioner (not implemented here; logged and ignored). |

### v1 velocity frame (7 fields)

```
<id> <vx> <vy> <w> <kick> <dribble> <time>
1 0.26 0 -0.45 0 0 1782911401.487
```

Body-frame velocities: `vx` forward m/s, `vy` strafe-left m/s, `w` yaw rad/s
CCW+. `kick`/`dribble` are bare `0`/`1`. `time` is the sender clock
(informational). Decoding is **strict**: exactly 7 fields, all floats finite,
`id` must match `Robot_id` (config; −1 accepts any) — otherwise the whole
packet is rejected and nothing moves. A v1 frame takes the robot out of MV2
mode.

### MV2 pose-target frame (23 fields)

```
MV2 <id> <seq> <KIND> <MODE> <px> <py> <ptheta> <vx> <vy> <vw>
    <tx> <ty> <ttheta> <max_speed> <max_w> <max_accel> <max_jerk>
    <arrive_speed> <kick> <dribble> <pose_age_ms> <time_set>
```

Golden string (pinned byte-for-byte in every repo's tests):

```
MV2 5 1234 MOVE ALIGN -1.2345 0.5 1.5708 0.25 -0.1 0.05 -1.2845 0.55 1.5708 0.35 1.5 1.2 8 0 0 1 18 1782911401.487
```

* Units: metres, radians, seconds; world = SSL-Vision frame.
* `px py ptheta` / `vx vy vw` — current pose and velocity from vision
  (`pose_age_ms` is its capture→send age; the robot extrapolates and blends).
* `tx ty ttheta` — target world pose.
* `KIND` ∈ `MOVE HOLD BRAKE DISABLE`; `MODE` ∈ `FAST BALL ALIGN HOLD BRAKE`
  (see docs/MOTION.md).
* Caps `max_*`: 0 = "use the mode profile"; a nonzero value can only
  tighten.
* `arrive_speed` — m/s to carry through the target (0 = stop there).
* `seq` — per-robot sequence number; duplicates and reordered (older) frames
  are dropped (wrapping compare). Echoed in telemetry as `mv_seq`.
* Parsing is strict: exact field count, known words, finite floats — a
  malformed frame is rejected whole and never moves the robot.

The server must only send MV2 to robots that advertise `mv=2` in telemetry.

## Telemetry (robot → server, protocol v2)

`key=value` pairs joined by commas, one datagram per `Telemetry_interval`
(default 200 ms). The **v1 core comes first** so pre-v2 dashboards still
parse; unknown keys must be ignored (the protocol is additive).

| Block | Keys |
|-------|------|
| v1 core | `state` (`active`\|`estop`\|`shutdown`), `voltage`, `ball`, `px`, `py`, `r`, `bearing`, `conf`, `ts_ms` |
| identity & link | `proto=2`, `rid` (physical robot letter), `seq`, `up_ms`, `ifip`, `vmin`, `m_ok`, `m_exp`, `cmd_age_ms`, `cmd_rx`, `cmd_last_id`, `ard`, `cam`, `estop`, `tx_err`, `cycle_ms` |
| per motor (×4) | `m{id}_ok`, `m{id}_mode`, `m{id}_fault`, `m{id}_temp`, `m{id}_volt`, `m{id}_vel`, `m{id}_cur`, `m{id}_cal` |
| IMU / odometry / loop | `imu_yaw_dps` (CCW+, `nan` = IMU down), `heading_deg`, `odo_vx`, `odo_vy`, `odo_w` (forward-kinematics body twist), `loop_ms`, `loop_jitter_ms`, `imu_ok` |
| calibration | `needs_cal`, `cal_state`, `cfg_fixed` (commissioner placeholders) |
| MV2 status | `mv=2` (capability flag — gates server MV2 dispatch), `mv_seq` (last accepted seq, −1 = never), `wd` (0 fresh / 1 braking / 2 coasted), `mv_kind` (`MOVE`/`HOLD`/`BRAKE`/`DISABLE`/`-`), `tgt_mm` (distance to MOVE target, −1 = none) |

Floats render with up to 4 decimals, trailing zeros trimmed; unavailable
optionals render as `nan` (parseable, never garbage). The encoder and its
required-key set are locked by host tests
(`tests/host/test_telemetry_wire.cpp`).
