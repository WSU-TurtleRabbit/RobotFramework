// Copyright 2023 mjbots Robotic Systems, LLC.
// Licensed under the Apache License, Version 2.0
//
// Simple single-motor test using moteus + pi3hat directly:
//   - Configures the pi3hat router with ONLY the specified bus
//   - Sends a constant velocity command at 50 Hz
//   - Prints telemetry returned each cycle
//   - Ctrl+C stops the motor cleanly before exit
//
// Usage: sudo ./motor_test --bus 1 --id 1 --vel 1

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

using namespace mjbots;

// --- Atomic flag set by Ctrl+C ---
std::atomic<bool> stop_flag{false};

void signalHandler(int /*signum*/) {
    stop_flag.store(true, std::memory_order_relaxed);
}

int main(int argc, char** argv) {
    using namespace std::chrono_literals;

    // ----- Config (defaults, overridable by flags) -----
    int    bus      = 1;
    int    id       = 1;
    double velocity = 1.0;       // rev/s
    const auto cycle_period = 20ms;  // 50 Hz control loop

    // ----- Argument parsing -----
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--bus" && i + 1 < argc) bus = std::stoi(argv[++i]);
        else if (a == "--id" && i + 1 < argc) id = std::stoi(argv[++i]);
        else if (a == "--vel" && i + 1 < argc) velocity = std::stod(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::cerr << "Usage: " << argv[0]
                      << " --bus N --id N --vel V\n"
                      << "Ctrl+C to stop.\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 1;
        }
    }

    // ----- Init transport (configures pi3hat with ONLY this bus) -----
    pi3hat::Pi3HatMoteusTransport::Options topts;
    topts.servo_map[id] = bus;
    auto transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(topts);

    moteus::Controller::Options copts;
    copts.transport = transport;
    copts.id = id;
    moteus::Controller controller(copts);

    // ----- Install Ctrl+C handler -----
    std::signal(SIGINT, signalHandler);

    // ----- Clear any latched faults before commanding motion -----
    controller.SetStop();
    std::this_thread::sleep_for(100ms);

    std::cout << "Spinning motor (bus " << bus << ", id " << id
              << ") at " << velocity << " rev/s. Ctrl+C to stop.\n";

    // ----- Main loop -----
    while (!stop_flag.load(std::memory_order_relaxed)) {
        moteus::PositionMode::Command cmd;
        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = velocity;

        auto result = controller.SetPosition(cmd);

        if (result) {
            const auto& v = result->values;
            std::cout << "ID " << id
                      << "  mode=" << static_cast<int>(v.mode)
                      << "  pos="  << v.position
                      << "  vel="  << v.velocity
                      << "  trq="  << v.torque
                      << "  V="    << v.voltage
                      << "  T="    << v.temperature
                      << "  flt="  << static_cast<int>(v.fault)
                      << "\n";
        } else {
            std::cout << "ID " << id << "  NO RESPONSE\n";
        }

        std::this_thread::sleep_for(cycle_period);
    }

    // ----- Clean shutdown -----
    std::cout << "\nCtrl+C received. Stopping motor...\n";

    // Two paths in parallel, belt-and-braces:
    //   1) Command zero velocity so the loop closes properly
    //   2) Then SetStop() to disable the bridges
    moteus::PositionMode::Command zero_cmd;
    zero_cmd.position = std::numeric_limits<double>::quiet_NaN();
    zero_cmd.velocity = 0.0;
    for (int i = 0; i < 3; ++i) {
        controller.SetPosition(zero_cmd);
        std::this_thread::sleep_for(20ms);
    }
    for (int i = 0; i < 3; ++i) {
        controller.SetStop();
        std::this_thread::sleep_for(20ms);
    }

    std::cout << "Motor stopped. Exiting.\n";
    return 0;
}