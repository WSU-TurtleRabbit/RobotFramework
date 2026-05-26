// motor_test.cc
// Minimal moteus single-motor test via pi3hat.
// Ctrl+C cleanly stops the motor before exit.

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

using namespace mjbots;

// Signal flag - must be sig_atomic_t for signal-handler safety.
// std::atomic<bool> happens to be lock-free for bool on every platform
// you'd run this on, so it's fine here too.
std::atomic<bool> g_stop{false};

void HandleSignal(int /*signum*/) {
    g_stop = true;
}

int main(int argc, char** argv) {
    int bus = 1;
    int id = 1;
    double velocity = 1.0;
    int duration = 3;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--bus" && i + 1 < argc) bus = std::stoi(argv[++i]);
        else if (a == "--id" && i + 1 < argc) id = std::stoi(argv[++i]);
        else if (a == "--vel" && i + 1 < argc) velocity = std::stod(argv[++i]);
        else if (a == "--duration" && i + 1 < argc) duration = std::stoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::cerr << "Usage: " << argv[0]
                      << " --bus N --id N --vel V [--duration S]\n"
                      << "Ctrl+C to stop cleanly.\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 1;
        }
    }

    // Install signal handlers BEFORE constructing transport, so even
    // if init hangs we can still bail out somewhat cleanly.
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << "Bus " << bus << ", ID " << id
              << ", velocity " << velocity << " rev/s, "
              << duration << "s (Ctrl+C to stop early)\n";

    pi3hat::Pi3HatMoteusTransport::Options topts;
    topts.servo_map[id] = bus;
    auto transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(topts);

    moteus::Controller::Options copts;
    copts.transport = transport;
    copts.id = id;
    moteus::Controller controller(copts);

    controller.SetStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "\nCycle | Pos     | Vel     | Trq     | Volt   | Temp | Fault | Mode\n"
              << "------+---------+---------+---------+--------+------+-------+-----\n";

    const auto period = std::chrono::milliseconds(50);  // 20 Hz
    auto start = std::chrono::steady_clock::now();
    int cycle = 0;
    bool clean_exit = true;

    while (!g_stop &&
           std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start).count() < duration) {
        auto next = std::chrono::steady_clock::now() + period;

        moteus::PositionMode::Command cmd;
        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = velocity;

        auto result = controller.SetPosition(cmd);

        if (result) {
            const auto& v = result->values;
            std::printf("%5d | %+7.3f | %+7.3f | %+7.3f | %6.2f | %4.1f | %5d | %4d\n",
                        cycle, v.position, v.velocity, v.torque,
                        v.voltage, v.temperature,
                        static_cast<int>(v.fault),
                        static_cast<int>(v.mode));
        } else {
            std::printf("%5d | NO RESPONSE\n", cycle);
        }

        cycle++;

        // Sleep in short chunks so Ctrl+C is responsive (won't wait
        // the full 50ms before noticing the signal).
        while (!g_stop && std::chrono::steady_clock::now() < next) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // ---- Clean shutdown ----
    // Reached whether by timeout, Ctrl+C, or fall-through.
    // Try multiple times in case CAN is briefly unresponsive.
    if (g_stop) {
        std::cout << "\n[Ctrl+C received - stopping motor]\n";
    } else {
        std::cout << "\nDuration elapsed - stopping motor\n";
    }

    for (int i = 0; i < 3; ++i) {
        try {
            controller.SetStop();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } catch (const std::exception& e) {
            std::cerr << "SetStop attempt " << (i + 1)
                      << " failed: " << e.what() << "\n";
            clean_exit = false;
        }
    }

    std::cout << "Completed " << cycle << " cycles.\n";
    return clean_exit ? 0 : 1;
}