// commission — `debugger commission`: supervised per-robot limit commissioning.
//
// The hardware half. Drives the robot by keyboard through the calibrated
// kinematics while the operator raises or lowers the moteus per-command
// velocity_limit / accel_limit and the body-speed envelope, all under a
// ceiling DERIVED from the controllers' own backstops (commission_core.h),
// logging every cycle locally (authoritative) and streaming best-effort.
//
// Safety properties, each of which has a single owner in commission.cpp:
//   * ESC / Ctrl-C / SIGTERM / SIGHUP -> zero twist, SetStop() on every
//     controller, exit. StopGuard's destructor also runs on exceptions.
//   * The moteus per-command watchdog is armed on every position frame
//     (Telemetry does this); the tool refuses to start if it is not.
//   * No setting can pass its cap: step_setting() is the only mutator.
//   * The rf::Supervisor trips (fault / over-temp / over-current /
//     under-voltage) plus a missing-reply and position_timeout watch put the
//     tool in HOLD: zero twist, STOP frames, SPACE to clear.
//   * Sign-off requires a name, a note, a drive window at the current
//     settings, and no active hold. It writes ONE file under
//     config/robot_profiles/ and touches nothing else.
#pragma once

#include <map>
#include <string>

namespace rf::dbg::commission {

struct RunOptions {
    std::string config_dir;  // absolute; main() has already chdir'd into it
    std::map<int, int> motor_map;
    double yaml_velocity_limit_rev_s = 0.0;  // Motor.yaml velocityLimit (NaN if absent)
    double yaml_accel_limit_rev_s2 = 0.0;    // Motor.yaml accelLimit (NaN if absent)
    double meters_per_motor_rev = 0.0;       // strict read, cross-checked against Wheel_math
    int motor_interval_ms = 4;
    int robot_id = -1;
    std::string stream_to;  // "ip:port" or ""
    std::string log_dir;    // absolute, or "" for <config_dir>/../logs/commission
    bool dry_run = false;
    bool assume_yes = false;
    // Test-only, hidden from help, refused unless dry_run: pre-set ceilings
    // ("velocity=25,accel=40,body=5,yaw=12") so the startup ceiling block can
    // be verified on hardware in every state. In a live session the C menu
    // is the only way to change a ceiling.
    std::string test_ceiling;
};

// Returns the process exit code.
int run(const RunOptions& opt);

// The help text for `debugger commission --help` / `debugger help commission`.
void print_help();

}  // namespace rf::dbg::commission
