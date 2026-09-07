# `debugger` — field diagnostic console

One binary for diagnosing the drivetrain from a terminal. No GUI, no ncurses,
no alternate screen: it is built to be run over `ssh` into the robot's Pi, and
it degrades cleanly when piped to a file or a CI log.

```
ssh pi@robot
cd ~/RobotFramework/build
./debugger              # live dashboard, motors stay dead
./debugger scan         # which CAN id answers on which bus
./debugger selftest     # pre-drive gate
./debugger help
```

## Why this exists

Diagnosing a wheel fault previously meant running `moteus_tool`, `tview`,
`pi3hat_tool` and the main binary in turn, then converting between wheel and
body frames by hand. Worse, several of those conversions are easy to get wrong
in ways that produce a plausible-looking number:

- A missing CAN reply rendered as `0.0` reads as *a stopped wheel* rather than
  *a wheel that did not answer*. That confusion is finding **B-04**.
- The generic motor-catalog torque constant (`9.5493/KV`) overstates torque by
  ~15% against the convention moteus actually uses (`8.2699/Kv`), which makes a
  healthy wheel look like it is under-delivering.
- Register `0x00f` carries **both** latched faults **and** live "my output is
  being limited right now" codes (≥96). Reading a flux-braking notice as a
  fault sends you chasing a failure that never happened.

This tool encodes those distinctions so you cannot make them by accident.

## Safety model

| Behaviour | Guarantee |
|---|---|
| `monitor`, `scan`, `info`, `imu`, `record`, `selftest`, `config dump` | Send **STOP** or query frames only. Wheels never energize. |
| `spin` | Energizes **one** wheel. Requires a typed `yes`, refuses when stdin is not a terminal unless `--yes` is passed, arms the moteus watchdog, and stops on every exit path including Ctrl-C. |
| `config apply` | **Refuses.** Prints the correct `moteus_tool --restore-config` command instead. |
| `calibrate` | **Does not calibrate.** Prints the exact `moteus_tool` commands for this robot's CAN map. |

`config apply` and `calibrate` deliberately do not act. Both write persistent
controller state and must not share the CAN bus with a running control loop —
and `--write-config` (the flag people reach for) sends each line verbatim with
no `conf set` prefix, so it would fail every line of a dump-shaped file.
`--restore-config` is the one that works.

## Commands

### `monitor` (default)

Live per-wheel telemetry: mode, fault/limit, position, velocity, q- and
d-current, torque, temperature, bus voltage. Below the table it derives the
**body twist from wheel odometry** via forward kinematics, and shows the pi3hat
IMU. Wheels that go silent get a consecutive-miss counter.

```
./debugger monitor --hz 10
./debugger monitor --once --no-color >> capture.txt
```

Motors are **not** energized: the cycle sends STOP frames, so telemetry flows
while the wheels stay dead.

### `scan`

Probes CAN ids 1..16 across buses 1..5 and reports what answered where. This is
the tool for "which motor ID is on what bus". It flags three distinct problems:

- **MISSING** — a controller in `Motor.yaml` did not answer on its declared bus.
- **MISMATCH** — a controller answered on a *different* bus than declared.
- **DUPLICATE** — the same id answered on more than one bus. This one matters
  more than it looks: the running control loop **drops** duplicate replies as
  ambiguous, so that wheel silently reads as *absent* rather than as
  misconfigured.

### `info`

Model number (`0x100`), firmware ABI (`0x101`), register-map version (`0x102`),
and hardware revision per controller. It cross-checks consistency across the
fleet, because a mixed ABI or register map changes what every other number
means.

ABI and register-map version are **different quantities** and this tool never
conflates them. For an r4.11 the expected hardware revision is **8** — hwrev 7
is r4.5b–r4.8 and takes a reduced gate-drive path meant for that revision, so a
7 here is highlighted.

### `selftest`

Pre-drive gate. Checks the CAN map loaded, the drive-scale calibration is
present, the moteus limits and watchdog are configured, every controller
replies fault-free with finite voltage and current, and the IMU produced a
sample. Exits non-zero on any failure, so it works in a startup script.

### `faults`

Offline reference: every fault code (1–7, 32–50) and limit reason (96–105) with
its meaning and the next diagnostic step, plus the controller mode table. Needs
no hardware.

### `kinematics`

Turns a commanded body twist into per-wheel demand using the robot's real drive
matrix, and runs the result back through forward kinematics as a round-trip
check. Shows each wheel's mount angle, true moment arm, surface speed, and how
close it sits to `velocityLimit`.

```
./debugger kinematics --vx 1.0 --vy 0.0 --w 0.0
```

Needs no hardware, so it is also the quickest way to sanity-check a config
change to `metersPerMotorRev`, `bodyLateralScale` or the wheel table.

### `record`

Captures telemetry to CSV at a fixed rate. Missing fields are written as
**empty**, never as `0`, so a gap stays a gap in whatever reads the file.

```
./debugger record --out /tmp/wheels.csv --hz 50 --duration 30
```

### `spin` — moves a wheel

Single-wheel jog with live commanded-vs-measured-vs-error. The robot must be on
blocks. Requires a typed confirmation.

```
./debugger spin --id 1 --vel 2.0 --duration 3
```

## `commission` — moves the robot

Supervised per-robot limit commissioning. You drive the robot by keyboard
through the calibrated kinematics while raising or lowering the moteus
per-command `velocity_limit` / `accel_limit` and the body-speed envelope, under a
**ceiling derived from the controllers' own backstops**. Every control cycle is
logged locally (authoritative) and optionally streamed to your laptop; a
sign-off writes a signed profile under `config/robot_profiles/`.

```
sudo systemctl stop robotframework.service
cd ~/RobotFramework
sudo ./build-debugger/debugger commission --config-dir config \
     --stream-to <laptop ip>:50600           # optional
sudo systemctl start robotframework.service
```

On the laptop (standard-library Python 3, Windows is fine):

```
python tools/telemetry_receiver.py --port 50600 --out session.ndjson
```

`--dry-run` prints identity, the ceiling derivation and the banner, then exits
without entering the drive loop (STOP and query frames only). `--yes` skips
the typed confirmation — pass it only if you have physically checked the robot.

### The ceiling

Per quantity, **80% of the smallest readable backstop**. Candidates, per
controller, read live over the diagnostic channel: `servo.default_velocity_limit`
/ `servo.default_accel_limit` (if a controller reports `nan` — no limit of its
own — `Motor.yaml`'s `velocityLimit` / `accelLimit` stands in for that
controller, since that is what the runtime sends it every cycle) and
`servo.max_velocity`. The derivation is printed line by line before the first
keypress and written into the log header and the profile, naming the candidate
that bound. If nothing readable bounds a quantity the tool refuses to run.

The wheel ceiling also implies a body-speed cap through the calibrated
kinematics (worst direction), min'd with `Safety.yaml`'s
`envelope.cappedLinear` / `cappedAngular`.

While driving, wheel demand is additionally capped at the *current*
`velocity_limit` setting (direction-preserving scale), so a controller never
clips one wheel and not another; the screen says `CLIPPED by velocity_limit`
and you raise the limit to drive faster.

### Keys

| Key | Action |
|---|---|
| `W` / `S`, `A` / `D`, `Q` / `E` | forward/back, strafe left/right, rotate CCW/CW (latched until changed) |
| `SPACE` | stop; also clears a HOLD once its cause is gone |
| `[` / `]` | body speed −/+ 0.10 m/s |
| `{` / `}` | body yaw rate −/+ 0.25 rad/s |
| `-` / `=` | moteus `velocity_limit` −/+ 0.5 rev/s |
| `_` / `+` | moteus `accel_limit` −/+ 1.0 rev/s² |
| `N` | record a typed note into the log |
| `P` | sign off: operator name, then observation note |
| `X` | quit (motors stopped, log closed) |
| `ESC` | **ABORT**: zero twist, `SetStop()` on every controller, exit |

No key can pass a cap; hitting one prints `CEILING: … cannot exceed …` and logs
a `limit_refused` event. Every accepted change logs `limit_change` with the
before/after values, so a reader can tie a limit to the motion it produced.

### Safety behaviour

- `ESC`, Ctrl-C, `SIGTERM` and `SIGHUP` (dropped ssh) all leave through the
  same path: zero twist, `SetStop()` twice on every controller. An exception
  does the same via an RAII guard.
- The moteus per-command watchdog (`Safety.yaml watchdogTimeout`) is armed on
  every position frame; the tool refuses to start if it is unset or > 0.5 s.
- **HOLD**: the `rf::Supervisor` trips (hard fault, over-temperature, sustained
  over-current, under-voltage, thresholds from `Safety.yaml`), a controller
  silent for 25 cycles, `position_timeout` on any controller, or a failing log
  file all zero the twist and send STOP frames until you press `SPACE`.
  Auto-recovery is off here: a commissioning trip must be looked at.
- It refuses to run without root, while `robotframework.service` is active
  (an mmap of `/dev/mem` is not exclusive), with a controller missing, or
  with an unreadable controller serial.

### Log and stream

NDJSON, one record per control cycle at `Motor_interval` (4 ms), under
`<repo>/logs/commission/commission_<pi serial>_<utc>.ndjson`. A header event
carries the Pi serial, every controller's serial / firmware hash / dirty flag,
the ceiling and its derivation, the initial settings and the trip thresholds.
Cycle records carry monotonic and wall time, requested and applied twist,
per-wheel commanded and measured velocity, position, q-current, temperature,
voltage, mode and fault, IMU rates / heading / acceleration, the current
settings and caps, and the hold / clip state. **Non-finite values and missing
replies serialise as `null`, never `0`.** Operator events (`limit_change`,
`limit_refused`, `note`, `hold`, `hold_cleared`, `signoff`, `signoff_refused`,
`abort`, `quit`) share the same sequence numbers.

Writing goes through a bounded queue and a worker thread; the control loop
never blocks on disk or network. Queue drops and UDP send failures are counted
on screen. The stream is best effort; the local file is the record.

### Sign-off and profiles

`P` asks for an operator name and an observation note, both required, and
refuses if a HOLD is active or the robot has not been driven since the last
setting change. It writes
`config/robot_profiles/profile_<pi serial>_<utc>.json` with: Pi CPU serial,
the four controller serials and firmware hashes, the approved settings, the
ceiling in force and its derivation, operator, note, UTC time, the log path,
and the window (sequence and time range, drive cycles, peak commanded and
measured wheel speed, peak current, faults and limit codes seen) that
justifies it. That file is the only write under `config/`; `Motor.yaml` is not
touched. Applying a profile is a separate step.

At startup the newest profile for this Pi is compared with the live
controllers by CAN id — serial first, then firmware hash. Any difference prints
a loud **DRIVETRAIN CHANGED** warning, is repeated on every screen frame and
recorded in the log header.

Note: the repository `.gitignore` ignores `*.json`, so profiles are not picked
up by `git add` unless an exception is added for `config/robot_profiles/`.

### Source layout

| File | Contents |
|---|---|
| `tools/debugger/commission_core.{h,cpp}` | Pure: ceiling derivation, setting/wheel clamps, NDJSON encoders, profile encode/decode, identity encodings, a small JSON reader. Covered by `tests/host/test_commission.cpp`. |
| `tools/debugger/commission.{h,cpp}` | Hardware: terminal, drive loop, supervisor HOLD, diagnostic-channel identity and backstop reads, log sink and UDP stream, sign-off. |
| `tools/telemetry_receiver.py` | Operator-side UDP sink with gap / out-of-order / duplicate accounting. |

## Reading the output

- **`--` means missing, always.** Never a zero. If a column shows `--`, that
  field was not in the reply.
- **`q-cur(A)` is phase current, not pack current.** Four wheels at 8 A phase is
  not 32 A out of the battery.
- **The fault column shows two different things.** A code below 96 is a latched
  hard fault and the controller has stopped. A code ≥96 is a live limit reason
  and the controller is still running — it is telling you what is capping it.
- **`position_timeout` (mode 11)** means the per-command watchdog expired: the
  host loop stalled or CAN dropped. It is not a controller fault.

## Options

| Flag | Meaning |
|---|---|
| `--config-dir <path>` | Where `Motor.yaml` lives. Default: search `../config`, `config`, `../../config`. |
| `--hz <n>` | Refresh rate for live views (default 5). |
| `--once` | Render one frame and exit — for scripts and pipes. |
| `--id <n>` | Target a single CAN id. |
| `--vel <rev/s>`, `--duration <s>` | For `spin` and `record`. |
| `--vx --vy --w` | Body twist for `kinematics`. |
| `--out <file>` | Output path for `record`. |
| `--no-color` | Disable ANSI colour. `NO_COLOR` and `TERM=dumb` are honoured too. |
| `--yes` | Skip the confirmation prompt on guarded actions. |

Colour and in-place redraw switch off automatically when stdout is not a
terminal, so `./debugger monitor | tee log.txt` produces a readable file rather
than a screenful of escape codes.

## Refusals, and why

The tool refuses rather than guessing in two places, both deliberate:

1. **`Motor.yaml` will not load** → it exits. The main runtime falls back to a
   guessed `{1:1, 2:2, 3:3, 4:4}` map, which is finding **B-03**: a wrong CAN
   map silently attributes one wheel's telemetry to another. A diagnostic tool
   that did the same would be actively misleading.
2. **`config apply`** → it prints the correct command instead of running it.

## Source layout

| File | Contents |
|---|---|
| `tools/debugger/decode.{h,cpp}` | Fault, limit and mode tables. Pure — no hardware headers, covered by `tests/host`. |
| `tools/debugger/term.{h,cpp}` | ANSI/tty handling, table rendering, NaN-safe number formatting, signal handling. |
| `tools/debugger/main.cpp` | CLI dispatch and the subcommands. |

The decode tables are sourced from the firmware (`moteus/fw/error.h`, `enum
class Mode`), not from documentation, and are verified by
`tests/host/test_debugger_decode.cpp`.

## Building

Built with the rest of the tree:

```
cmake -S . -B build && cmake --build build
```

The pure decode layer is also compiled into the host test suite, which needs no
Pi:

```
cmake -S tests/host -B tests/host/build && cmake --build tests/host/build
./tests/host/build/host_tests debugger
```
