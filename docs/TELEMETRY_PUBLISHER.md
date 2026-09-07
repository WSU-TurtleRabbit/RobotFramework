# Independent match telemetry publisher

This implementation is observational. RobotFramework remains the only CAN owner
and keeps the existing command/feedback protocol. A fixed-size, nonblocking local
Unix datagram carries snapshots to a separate Python process. The Python process
owns multicast serialization and networking; it never reads CAN, sends commands,
imports the commissioner, writes robot configuration, or gates motion.

No live robot deployment is performed by building or testing these files.

## Files and interface

- `Telemetry/diagnostics.h` and `.cpp`: fixed-size stack snapshot and Unix socket
  sender. No disk, JSON, logging, waits or dynamic allocation in `offer()`.
- `RobotFramework.cpp`: snapshot from existing CAN results and control variables,
  sampled independently of disk logger health at at most 100 Hz.
- `diagnostics/ipc.py`: private binary IPC decoder.
- `diagnostics/wire.py`: public telemetry v1 encoder and sampled transition events.
- `diagnostics/publisher.py`: separate process, bounded receipt, latest state,
  event queue, multicast rate budget and counters.
- `tests/diagnostics_fixture.cpp`: hardware-free encoder and nonblocking failure tests.
- `tests/test_diagnostics_publisher.py`: protocol, restart, queue, failure tests.

IPC is disabled unless `PHOENIX_TELEMETRY_SOCKET` is set before RobotFramework
starts. Socket/randomness initialization failure disables only diagnostics.
Absent, stopped or saturated receivers increment a drop counter; no receiver
ACK, retry, ownership update or command behavior is involved. The local sender
socket remains usable when the publisher restarts. Linux Unix datagram sockets
bound kernel buffering; they are not a hard real-time scheduling guarantee.

Private IPC v1 is exactly 668 bytes, explicitly little-endian, not compiler struct
memory. Header `<4sI16siIQQQQ`: `PHI1`, version 1, random producer process ID,
onboard robot ID, reserved zero, cycle, post-CAN acquisition microseconds,
offered count, previous dropped count. Four `<7diiIIQ` motor records follow:
controller request, submitted velocity, measured velocity/position/current/
voltage/temperature, mode, fault, replied, energized, age microseconds.
Tail `<33dIQQ` contains the motion values documented in `ipc.py`/`wire.py`, flags,
kick request and successful Arduino-byte-send counters. This is local IPC only;
the network is always the independently versioned JSON format.

## Public telemetry v1

Default destination `239.255.50.60:50610`, IPv4 multicast TTL 1. An explicit
outbound IPv4 interface is required. One compact UTF-8 JSON object per datagram,
at most 1200 bytes. Oversize packets are dropped and counted, never fragmented
by the application or silently stripped of fields.

Every packet has `schema="phoenix.telemetry"`, `version=1`, `type`, `device_id`,
32-hex `session_id`, onboard `robot_id`, uint32 `seq`, decimal-string `cycle`,
decimal-string `t_mono_us`, integer `snapshot_age_us`, and `data`. Sequence is
independent per type. Session changes on publisher restart or new producer ID.
`t_mono_us` is Linux steady/CLOCK_MONOTONIC acquisition time, not UTC. The receiver
must preserve its own time and cannot directly subtract cross-machine clocks.

Motor `data.wheels` rows use these exact columns:

```text
id, requested_rev_s, sent_rev_s, vel_rev_s, pos_rev, q_current_a,
voltage_v, temp_c, mode, fault, replied, age_us, energized
```

CAN 1=FR, 2=RR, 3=RL, 4=FL. Position/velocity are controller output revolutions,
not automatically rotor or physical wheel units. Current is q-axis phase current;
temperature is controller temperature. Missing replies null measurements, mode,
fault and age. STOP/de-energized frames have `sent_rev_s=null`; this is distinct
from an energized zero-velocity command. `sent_rev_s` means submitted CAN demand,
not an acknowledgement that the motor applied it.

Motor `age_us` is zero at the recorded CAN-cycle completion: it is a lower bound
on measurement age, not a controller-internal sample timestamp. The publisher's
`snapshot_age_us` adds time since that completion; the receiver adds local age.

Identity includes columns, wheel mapping, capabilities and nominal rates. Firmware
revision is explicitly unknown in this increment; the process cannot certify the
running RobotFramework build from the Python checkout's revision. No moteus ABI,
CPU/network metric, battery current or actuator sensor measurement is fabricated.

Motion reports existing requested/reference/estimated states, timing, IMU and
vision. `controller_output_body` is the bridge controller output before the outer
supervisor gate; final motor demand and energized state are authoritative about
what was submitted after that gate. Command receive/apply sequence and
`control_session_id` are null until their ownership/correlation is instrumented.
The estimator consumes previous-cycle wheels, so motion and new motor replies
are not simultaneous observations. `estimator_precedes_motor_sample_s` describes
the tick-time offset; it is not an independently measured sensor latency.
`elapsed_before_telemetry_s` excludes the snapshot offer and later logging;
`tick_dt_s` measures successive tick starts. Neither alone proves full control
deadline behavior or total telemetry overhead.

Health reports producer offers/drops, local receive/coalescing/invalid/late/stale
counts, packet send/errors/budget/oversize counts, supervisor/Arduino state and
kick request/send counts. Successful Arduino writes do not prove discharge or
ball contact; `physical_kick_confirmation` is null.
Actuator counters cover MatchCtrl only and are null when the bridge is inactive;
legacy actuator commands are not instrumented by these counters.

Events describe sampled supervisor/motor-state changes. Each has an event ID,
code, severity, subsystem and source `sampled_transition`. Transitions wholly
between snapshots are unobservable and are explicitly identified in health.
There is no reliable event delivery guarantee. Raw limiting codes 96..105 are
distinct from latched faults 1..7 and 32..50. Unknown firmware-specific meanings need the raw
numeric code retained by the receiver.

Rates are capped at motors 50 Hz, motion 20 Hz, health 1 Hz, identity every 5 s.
One token bucket covers all network packets: at most 65536 bytes/s replenishment,
with a bounded 4800-byte burst capacity. Full queues and exhausted budgets drop
diagnostics. Events have a 32-entry queue, one emission per loop; UDP receives
drain at most 256 datagrams per pass. Stale/future snapshots are rejected, and
no ordinary stream continues after producer data is older than 250 ms.

## Build and verify without hardware access

From the checkout root on Linux:

```sh
mkdir -p build-telemetry-test tests/fixtures
g++ -std=c++20 -O2 -Wall -Wextra -I. tests/diagnostics_fixture.cpp \
    Telemetry/diagnostics.cpp -o build-telemetry-test/diagnostics_fixture
build-telemetry-test/diagnostics_fixture tests/fixtures/diagnostics-v1.bin
python3 -m unittest discover -s tests -p test_diagnostics_publisher.py -v
python3 -m tests.write_diagnostics_fixtures
```

If the local `tests` directory is shadowed by an installed Python package, run the
fixture writer with the repository root on `PYTHONPATH` instead:
`PYTHONPATH=. python3 tests/write_diagnostics_fixtures.py`.

Fixtures are intentionally synthetic: `tests/fixtures/diagnostics-v1.bin` is made
by the actual C++ encoder; `telemetry-v1.ndjson` carries its decoded fields through
the actual network encoder. They are protocol compatibility evidence, not recorded
robot movement. Tests do not instantiate moteus, pi3hat, Arduino or RobotFramework.

## Planned deployment, not an automatic action

1. Review/test the isolated RobotFramework change, confirm the robot's physical,
   wire and camera identities, and schedule a supervised HALTED restart. Back up
   the current binary/configuration and preserve current controller settings.
2. Build the changed RobotFramework binary in an isolated build directory with
   the existing Pi toolchain; do not replace the running binary during tests.
3. Start the Python publisher as an independent service/user. Give it a private
   owned directory, for example `/run/phoenix-telemetry`, via service manager
   `RuntimeDirectory`. The directory must not be group/world writable; socket
   permissions are 0600. Run RobotFramework as the same user or root.
4. Use the robot's actual outbound address, for example:

   ```sh
   python3 -m diagnostics.publisher \
       --socket /run/phoenix-telemetry/snapshot.sock \
       --interface 192.168.210.126
   ```

5. Configure `PHOENIX_TELEMETRY_SOCKET=/run/phoenix-telemetry/snapshot.sock` in the
   RobotFramework service environment and deploy/restart only at the agreed
   HALTED window. No script here performs that restart or enables robot motion.
6. Verify fresh idle measurements on at least two independent multicast
   receivers, compare control timing with telemetry off/on, then test publisher
   loss/restart while keeping existing robot watchdog/STOP behavior unchanged.
   Do not launch the commissioner beside RobotFramework.
7. A rollback removes the environment setting and restores the prior binary in
   a planned restart. Stopping just the Python publisher is safe for control;
   the producer continues bounded unsuccessful snapshot offers until disabled.

Legacy synchronous logs and shared CPU/radio resources still exist. Passing the
hardware-free tests proves bounded code paths and wire interoperability, not
match-load timing, radio capacity or physical actuator performance.

### Inspected deployment paths for robot 192.168.210.126

The current service runs as root:

```text
WorkingDirectory=/home/pi/RobotFramework
ExecStart=/usr/bin/bash /home/pi/RobotFramework/STARTUP.sh --run-service
Binary=/home/pi/RobotFramework/build/RobotFramework
```

`STARTUP.sh` manages Wi-Fi, and `ExecStopPost` calls its `--restore` mode.
A restart can disrupt SSH. Execute an approved activation/rollback through a
persistent supervised service-manager job; do not depend on an interactive SSH
connection surviving between stop and start. The following are concrete paths
and commands for review, not a deployment performed by this task.

Build artifact: `/tmp/phoenix-telemetry-20260908-a/build/RobotFramework`.
Source checkout: `/tmp/phoenix-telemetry-20260908-a/source`.
Compiler flags match the inspected default build (`-std=gnu++20`, empty CMake
build type), rather than changing control optimization during integration.

Before activation, preserve the existing binary/configuration/service definition
and record their hashes. Stage a durable publisher package independently of the
live source checkout; `/tmp` is a test location, not an installation directory.
An example independent publisher unit is:

```ini
[Unit]
Description=Phoenix observational telemetry publisher
After=network-online.target

[Service]
User=pi
WorkingDirectory=/home/pi/phoenix-telemetry
RuntimeDirectory=phoenix-telemetry
RuntimeDirectoryMode=0700
ExecStart=/usr/bin/python3 -m diagnostics.publisher --socket /run/phoenix-telemetry/snapshot.sock --interface 192.168.210.126
Restart=on-failure
RestartSec=2
Nice=10
NoNewPrivileges=true
MemoryMax=64M

[Install]
WantedBy=multi-user.target
```

The robot service requires only an explicitly reviewed environment drop-in:

```ini
[Service]
Environment=PHOENIX_TELEMETRY_SOCKET=/run/phoenix-telemetry/snapshot.sock
```

Do not add `Requires=phoenix-telemetry.service` or otherwise couple robot liveness
to publisher liveness. The sender works if the publisher starts later.

After HALT and backup verification, the activation job can stage and swap the
binary using the exact reviewed paths:

```sh
sudo cp -a /home/pi/RobotFramework/build/RobotFramework \
  /home/pi/RobotFramework/build/RobotFramework.before-telemetry-20260908
sudo install -m 0755 /tmp/phoenix-telemetry-20260908-a/build/RobotFramework \
  /home/pi/RobotFramework/build/RobotFramework.telemetry-candidate
sudo systemctl stop robotframework.service
sudo mv /home/pi/RobotFramework/build/RobotFramework.telemetry-candidate \
  /home/pi/RobotFramework/build/RobotFramework
sudo systemctl start robotframework.service
```

Require the backup destination to be absent before the first copy; do not replace
an earlier rollback artifact. A rollback stages that verified backup under a new
temporary filename, stops the robot service while HALTED, swaps the binary back,
and starts the service. Restore/remove only the telemetry drop-in created during
this activation, retaining all other service configuration. The previous binary
ignores the telemetry environment variable, so binary rollback does not depend
on the diagnostics service being reachable.

Neither activation nor rollback should enable RUN, execute a wheel test, modify
motor limits, rebuild the live tree, or launch a second CAN process.
