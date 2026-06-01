// Copyright 2023 mjbots Robotic Systems, LLC.
// Licensed under the Apache License, Version 2.0
//
// Simple multi-motor test using the Telemetry library:
//   - Sends a velocity command to all motors via Telemetry::cycle()
//   - Prints telemetry returned each cycle
//   - Ctrl+C stops all motors cleanly before exit

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <map>
#include <thread>

#include "Telemetry.h"

// --- Atomic flag set by Ctrl+C ---
std::atomic<bool> stop_flag{false};

void signalHandler(int /*signum*/) {
    stop_flag.store(true, std::memory_order_relaxed);
}

int main() {
    using namespace std::chrono_literals;

    // ----- Config -----
    const double test_velocity = 1.0;   // rev/s — same value to all 4 motors
    const auto   cycle_period  = 20ms;  // 50 Hz control loop

    // Motor ID → commanded velocity. Telemetry::cycle() expects this map.
    std::map<int, double> velocity_map = {
        {1, test_velocity},
        {2, test_velocity},
        {3, test_velocity},
        {4, test_velocity},
    };

    // ----- Init telemetry (constructs the controllers internally) -----
    Telemetry telemetry;

    // ----- Install Ctrl+C handler -----
    std::signal(SIGINT, signalHandler);

    // ----- Clear any latched faults before commanding motion -----
    for (const auto& pair : telemetry.controllers) {
        pair.second->SetStop();
    }
    std::this_thread::sleep_for(100ms);

    std::cout << "Spinning all motors at " << test_velocity
              << " rev/s. Ctrl+C to stop.\n";

    // ----- Main loop -----
    while (!stop_flag.load(std::memory_order_relaxed)) {
        auto servo_status = telemetry.cycle(velocity_map);

        for (const auto& pair : servo_status) {
            const int   id = pair.first;
            const auto& r  = pair.second;
            std::cout << "ID " << id
                      << "  mode=" << static_cast<int>(r.mode)
                      << "  vel="  << r.velocity
                      << "  cur="  << r.current
                      << "  V="    << r.voltage
                      << "  T="    << r.temperature
                      << "\n";
        }

        std::this_thread::sleep_for(cycle_period);
    }

    // ----- Clean shutdown -----
    std::cout << "\nCtrl+C received. Stopping motors...\n";

    // Two paths in parallel, belt-and-braces:
    //   1) Command zero velocity through Telemetry so the loop closes properly
    //   2) Then SetStop() directly to disable the bridges
    std::map<int, double> zero_map = {{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}};
    for (int i = 0; i < 3; ++i) {
        telemetry.cycle(zero_map);
        std::this_thread::sleep_for(20ms);
    }
    for (int i = 0; i < 3; ++i) {
        for (const auto& pair : telemetry.controllers) {
            pair.second->SetStop();
        }
        std::this_thread::sleep_for(20ms);
    }

    std::cout << "All motors stopped. Exiting.\n";
    return 0;
}