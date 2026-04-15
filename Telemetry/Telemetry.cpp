#include "Telemetry.h"
#include <yaml-cpp/yaml.h>
#include <limits>

Telemetry::Telemetry()
{
    // Transport configuration for Pi3Hat
    mjbots::pi3hat::Pi3HatMoteusTransport::Options toptions;
    std::map<int, int> servo_map = YAML_Load_MotorMap("../config/Motor.yaml");
    toptions.servo_map = servo_map;

    // A shared transport instance used for the Cycle method
    transport = std::make_shared<mjbots::pi3hat::Pi3HatMoteusTransport>(toptions);

    // Create controllers for each motor ID / bus pair, using the shared transport
    for (const auto &p : servo_map)
    {
        mjbots::moteus::Controller::Options opts;
        opts.id = p.first;
        opts.bus = p.second;
        opts.transport = transport;
        controllers[opts.id] = std::make_shared<mjbots::moteus::Controller>(opts);
    }

    // Issue a stop command to all controllers (clear faults before starting)
    for (const auto &pair : controllers)
    {
        pair.second->SetStop();
    }
}

std::map<int, MotorTelemetry> Telemetry::cycle(const std::map<int, double> &velocity_map)
{
    // Build command frames
    std::vector<mjbots::moteus::CanFdFrame> command_frames;
    command_frames.reserve(controllers.size());

    for (const auto &pair : controllers)
    {
        mjbots::moteus::PositionMode::Command position_command;
        position_command.position = std::numeric_limits<double>::quiet_NaN();
        auto it = velocity_map.find(pair.first);
        position_command.velocity = (it != velocity_map.end()) ? it->second : 0.0;
        command_frames.push_back(pair.second->MakePosition(position_command));
    }

    // Send all commands in one BlockingCycle and collect replies
    std::vector<mjbots::moteus::CanFdFrame> replies;

    if (!command_frames.empty())
    {
        transport->BlockingCycle(command_frames.data(), command_frames.size(), &replies);
    }

    // Parse replies into a map keyed by responding CAN ID (frame.source)
    std::map<int, MotorTelemetry> servo_data;
    for (const auto &frame : replies)
    {
        auto parsed = mjbots::moteus::Query::Parse(frame.data, frame.size);
        MotorTelemetry mt;
        mt.temperature = parsed.temperature;
        mt.voltage = parsed.voltage;
        mt.velocity = parsed.velocity;
        mt.current = parsed.q_current;
        mt.mode = static_cast<int>(parsed.mode);
        servo_data[frame.source] = mt;
    }
    // std::cout << servo_data << "\n";

    return servo_data;
}

std::map<int,int> Telemetry::YAML_Load_MotorMap(const std::string& path) {
    std::map<int,int> motor_map;

    try {
        YAML::Node config = YAML::LoadFile(path);
        YAML::Node motors = config["motorMap"];

        for (YAML::const_iterator it = motors.begin(); it != motors.end(); ++it) {
            int motor_id = it->first.as<int>();   // key in YAML
            int bus_num  = it->second.as<int>();  // value in YAML
            motor_map[motor_id] = bus_num;       // store in C++ map
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading motor config: " << e.what() << std::endl;
        // optional: fallback default mapping
        motor_map = {{1,1},{2,2},{3,2},{4,1}};
    }

    return motor_map;
}