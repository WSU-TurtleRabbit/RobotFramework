// motor_test.cc
// Minimal moteus single-motor test via pi3hat.
// Only the specified bus is registered in the router, so this is a
// clean baseline for diagnosing the empty-bus wedge bug: if it works
// here but fails in your real code, your real code is hitting empty buses.

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

using namespace mjbots;

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
                      << " --bus N --id N --vel V [--duration S]\n";
            return 0;
        } else {
            std::cerr << "Unknown arg: " << a << "\n";
            return 1;
        }
    }

    std::cout << "Bus " << bus << ", ID " << id
              << ", velocity " << velocity << " rev/s, "
              << duration << "s\n";

    // Configure transport with ONLY this one bus
    pi3hat::Pi3HatMoteusTransport::Options topts;
    topts.servo_map[id] = bus;
    auto transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(topts);

    moteus::Controller::Options copts;
    copts.transport = transport;
    copts.id = id;
    moteus::Controller controller(copts);

    // Clear any latched fault
    controller.SetStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "\nCycle | Pos     | Vel     | Trq     | Volt   | Temp | Fault | Mode\n"
              << "------+---------+---------+---------+--------+------+-------+-----\n";

    const auto period = std::chrono::milliseconds(50);  // 20 Hz
    auto start = std::chrono::steady_clock::now();
    int cycle = 0;

    while (std::chrono::duration_cast<std::chrono::seconds>(
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
        std::this_thread::sleep_until(next);
    }

    std::cout << "\nStopping...\n";
    controller.SetStop();
    return 0;
}