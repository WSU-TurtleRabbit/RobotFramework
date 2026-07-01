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
        // Actually transmit the velocity/accel limits: without setting the
        // format resolution these command fields default to kIgnore and are
        // never put on the wire.
        opts.position_format.velocity_limit = mjbots::moteus::kFloat;
        opts.position_format.accel_limit    = mjbots::moteus::kFloat;
        controllers[opts.id] = std::make_shared<mjbots::moteus::Controller>(opts);
    }

    // Issue a stop command to all controllers (clear faults before starting)
    for (const auto &pair : controllers)
    {
        pair.second->SetStop();
    }

    // Push the current limit to every controller. moteus has no per-command
    // current-limit register, so this goes over the diagnostic channel and
    // sets the running config (not persisted to flash — see Telemetry.h).
    for (const auto &pair : controllers)
    {
        try
        {
            pair.second->DiagnosticCommand(
                "conf set servo.max_current_A " + std::to_string(current_limit_A));
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to set current limit on id " << pair.first
                      << ": " << e.what() << std::endl;
        }
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
        // Default motion limits sent with every command (rev/s, rev/s^2).
        position_command.velocity_limit = velocity_limit;
        position_command.accel_limit    = accel_limit;
        command_frames.push_back(pair.second->MakePosition(position_command));
    }

    // Send all commands AND read the pi3hat IMU attitude in one CAN cycle.
    // Passing &attitude sets request_attitude in the transport, so the gyro/AHRS
    // data comes back "for free" alongside the moteus replies.
    std::vector<mjbots::moteus::CanFdFrame> replies;

    if (!command_frames.empty())
    {
        mjbots::moteus::BlockingCallback cbk;
        transport->Cycle(command_frames.data(), command_frames.size(),
                         &replies, &attitude, nullptr, nullptr, cbk.callback());
        cbk.Wait();
        attitude_ok = true;
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
        // mt.position = parsed.position;
        mt.mode = static_cast<int>(parsed.mode);
        servo_data[frame.source] = mt;

        // std::cout<< "Current is: " << parsed.q_current<< "\n"; 
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
        motor_map = {{1,1},{2,2},{3,3},{4,4}};
    }

    return motor_map;
}

// Heading (yaw) about the vertical axis, from the fused attitude quaternion.
double Telemetry::yaw_rad() const
{
    const auto& q = attitude.attitude;   // quaternion (w, x, y, z)
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// Yaw rate (rad/s) from the gyro (rate_dps.z is deg/s about the vertical axis).
double Telemetry::yaw_rate_rps() const
{
    return attitude.rate_dps.z * (M_PI / 180.0);
}