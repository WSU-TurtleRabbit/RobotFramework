// Copyright 2023 mjbots Robotic Systems, LLC.
// Licensed under the Apache License, Version 2.0
//
// OpenLoopTest — open-loop straight/diagonal drive diagnostic.
//
// PURPOSE
//   The robot veers when commanded pure translation. This harness isolates
//   WHETHER that is a kinematic/wheel-speed problem or a heading (yaw) problem.
//   Only the latter is fixable with the gyro / HeadingController.
//
//   It sends a FIXED, HELD body-frame command (w == 0 exactly) for a set
//   duration, hands-off, with NO closed-loop correction of any kind:
//     - no vision
//     - no IMU feedback in the command path (HeadingController is NOT used)
//   The IMU is only *read and logged* to record whether the robot actually
//   rotated during a "drive straight" command. It never touches the command.
//
//   Per-wheel commanded vs. actual velocity (moteus telemetry, rev/s) and the
//   raw pi3hat yaw rate (dps, z) are logged to CSV at the loop rate. A summary
//   prints after each run.
//
// USAGE (run from the build dir, same as the other tests — the Telemetry
// class loads ../config/Motor.yaml with a relative path):
//
//   sudo ./OpenLoopTest straight              # vx=1, vy=0, w=0, 3 s
//   sudo ./OpenLoopTest diagonal              # vx=1, vy=1, w=0, 3 s
//   sudo ./OpenLoopTest straight 5            # 5 s run
//   sudo ./OpenLoopTest diagonal 3 --unsafe   # bypass Wheel_math speed limits
//
//   --safe (default: capped) / --unsafe select the Wheel_math limit mode.
//   Ctrl+C stops and zeros all motors at any time.
//
// SAFE TO RUN ON THE FLOOR: give the robot ~2 m of clear space. The command is
// held only for `duration`, then motors are zeroed and stopped automatically.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "Telemetry.h"
#include "wheel_math.h"

// --- Atomic flag set by Ctrl+C ---
std::atomic<bool> stop_flag{false};
void signalHandler(int /*signum*/) { stop_flag.store(true, std::memory_order_relaxed); }

// Zero, then stop, all motors. Belt-and-braces (same pattern as MultiMotor.cpp).
static void stop_all(Telemetry& telemetry) {
    using namespace std::chrono_literals;
    std::map<int, double> zero_map = {{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}};
    for (int i = 0; i < 3; ++i) {
        telemetry.cycle(zero_map);
        std::this_thread::sleep_for(20ms);
    }
    for (int i = 0; i < 3; ++i) {
        for (const auto& pair : telemetry.controllers) pair.second->SetStop();
        std::this_thread::sleep_for(20ms);
    }
}

int main(int argc, char** argv) {
    using namespace std::chrono;
    using namespace std::chrono_literals;

    // ----- Config / arg parsing -----
    std::string test_case = "straight";
    double duration_s = 3.0;
    int    mode = 1;                       // Wheel_math: 0=safe 1=capped 2=unsafe
    const auto cycle_period = 20ms;        // 50 Hz, matches the real control loop

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "straight" || a == "diagonal") test_case = a;
        else if (a == "--safe")   mode = 0;
        else if (a == "--capped") mode = 1;
        else if (a == "--unsafe") mode = 2;
        else if (a == "-h" || a == "--help") {
            std::cerr << "Usage: sudo " << argv[0]
                      << " [straight|diagonal] [duration_s] [--safe|--capped|--unsafe]\n";
            return 0;
        } else {
            // A bare number is the duration.
            try { duration_s = std::stod(a); }
            catch (...) { std::cerr << "Unknown arg: " << a << "\n"; return 1; }
        }
    }

    // Body-frame command for this run. w is ZERO by construction — never set.
    const double cmd_vx = 1.0;
    const double cmd_vy = (test_case == "diagonal") ? 1.0 : 0.0;
    const double cmd_w  = 0.0;

    std::cout << "\n=== OpenLoopTest: " << test_case << " ===\n"
              << "  vx=" << cmd_vx << "  vy=" << cmd_vy << "  w=" << cmd_w
              << "  duration=" << duration_s << "s  mode="
              << (mode == 0 ? "safe" : mode == 1 ? "capped" : "unsafe") << "\n"
              << "  NO IMU/vision correction in the command path (open loop).\n";

    // ----- Init -----
    Telemetry telemetry;
    Wheel_math m;
    m.setMode(mode);

    // Compute the held per-wheel command ONCE (open loop => constant).
    // Wheel_math::calculate() is the exact kinematics the real robot uses, so
    // this test exposes that pipeline, axis-swap and all.
    std::vector<double> wheel_cmd = m.calculate(cmd_vx, cmd_vy, cmd_w);
    if (wheel_cmd.size() < 4) { std::cerr << "kinematics returned <4 wheels\n"; return 1; }
    std::map<int, double> velocity_map = {
        {1, wheel_cmd[0]}, {2, wheel_cmd[1]}, {3, wheel_cmd[2]}, {4, wheel_cmd[3]}};

    std::cout << "  commanded wheel vel (rev/s as sent to moteus): "
              << wheel_cmd[0] << ", " << wheel_cmd[1] << ", "
              << wheel_cmd[2] << ", " << wheel_cmd[3] << "\n";

    // ----- CSV -----
    std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::string fname = "openloop_" + test_case + "_" + stamp + ".csv";
    std::ofstream csv(fname);
    csv << "t_s,cmd1,act1,cmd2,act2,cmd3,act3,cmd4,act4,yaw_rate_dps,yaw_int_deg\n";

    // ----- Ctrl+C + clear faults -----
    std::signal(SIGINT, signalHandler);
    for (const auto& pair : telemetry.controllers) pair.second->SetStop();
    std::this_thread::sleep_for(100ms);

    // Countdown so nobody is holding the robot when it lurches.
    std::cout << "  Starting in 3s — clear the area..." << std::flush;
    for (int i = 3; i > 0 && !stop_flag.load(); --i) {
        std::cout << " " << i << std::flush;
        std::this_thread::sleep_for(1s);
    }
    std::cout << "  GO\n";

    // ----- Run: hold the command for `duration`, log every cycle -----
    const int ids[4] = {1, 2, 3, 4};
    double sum_cmd[4] = {0, 0, 0, 0};
    double sum_act[4] = {0, 0, 0, 0};
    int    n_act[4]   = {0, 0, 0, 0};
    double yaw_int_deg = 0.0;              // integrated raw gyro yaw

    auto t0 = steady_clock::now();
    auto t_prev = t0;
    while (!stop_flag.load(std::memory_order_relaxed)) {
        auto t_now = steady_clock::now();
        double t_s  = duration<double>(t_now - t0).count();
        double dt_s = duration<double>(t_now - t_prev).count();
        t_prev = t_now;
        if (t_s >= duration_s) break;

        // Send held command + read IMU in the same CAN cycle.
        auto servo_status = telemetry.cycle(velocity_map);

        // Raw gyro yaw rate (deg/s about vertical) — logged, NOT fed back.
        double yaw_rate_dps = telemetry.attitude.rate_dps.z;
        yaw_int_deg += yaw_rate_dps * dt_s;

        double act[4] = {NAN, NAN, NAN, NAN};
        for (int k = 0; k < 4; ++k) {
            auto it = servo_status.find(ids[k]);
            if (it != servo_status.end()) {
                act[k] = it->second.velocity;
                sum_cmd[k] += velocity_map[ids[k]];
                sum_act[k] += act[k];
                n_act[k]++;
            }
        }

        csv << t_s;
        for (int k = 0; k < 4; ++k) csv << "," << velocity_map[ids[k]] << "," << act[k];
        csv << "," << yaw_rate_dps << "," << yaw_int_deg << "\n";

        std::this_thread::sleep_for(cycle_period);
    }
    double run_s = duration<double>(steady_clock::now() - t0).count();

    // ----- Stop -----
    std::cout << "\nRun complete (" << run_s << "s). Stopping motors...\n";
    stop_all(telemetry);
    csv.close();

    // ----- Summary -----
    std::cout << "\n----- SUMMARY (" << test_case << ") -----\n";
    std::cout << "commanded: vx=" << cmd_vx << " vy=" << cmd_vy << " w=" << cmd_w << "\n";
    std::cout << "duration:  " << run_s << " s\n";
    std::cout << "integrated IMU yaw: " << yaw_int_deg << " deg\n";
    std::cout << "per-wheel mean actual/commanded ratio (moteus rev/s):\n";
    for (int k = 0; k < 4; ++k) {
        double mc = n_act[k] ? sum_cmd[k] / n_act[k] : 0.0;
        double ma = n_act[k] ? sum_act[k] / n_act[k] : 0.0;
        std::cout << "  wheel " << ids[k] << ": cmd=" << mc << " act=" << ma
                  << " ratio=" << (std::fabs(mc) > 1e-6 ? ma / mc : NAN)
                  << (n_act[k] ? "" : "  [no telemetry]") << "\n";
    }
    std::cout << "CSV: " << fname << "\n";
    std::cout << "\nHow to read: |yaw| small (say <~5 deg) => heading held, path is\n"
                 "a straight line (maybe wrong direction) => KINEMATIC error, the\n"
                 "gyro will NOT fix it. |yaw| large (tens of deg) => robot rotated\n"
                 "=> HEADING drift, HeadingController will fix it. Ratios far from 1\n"
                 "or negative on a single wheel => that motor isn't tracking / is\n"
                 "reversed (a per-wheel calibration or wiring problem).\n";
    return 0;
}
