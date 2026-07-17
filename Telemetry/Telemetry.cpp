#include "Telemetry.h"
#include "motor_formats.h"
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

    // moteus hardware watchdog (seconds); fallback keeps the header default.
    try
    {
        YAML::Node s_config = YAML::LoadFile("../config/Safety.yaml");
        if (s_config["watchdogTimeout"])
            watchdog_timeout_s = s_config["watchdogTimeout"].as<double>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading Safety config for watchdog: " << e.what() << std::endl;
    }

    // Optional moteus-level velocity/accel shaping (rev/s, rev/s^2).
    try
    {
        YAML::Node m_config = YAML::LoadFile("../config/Motor.yaml");
        if (m_config["velocityLimit"])
            velocity_limit_rev_s = m_config["velocityLimit"].as<double>();
        if (m_config["accelLimit"])
            accel_limit_rev_s2 = m_config["accelLimit"].as<double>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading Motor config for limits: " << e.what() << std::endl;
    }

    // Create controllers for each motor ID / bus pair, using the shared transport
    for (const auto &p : servo_map)
    {
        mjbots::moteus::Controller::Options opts;
        opts.id = p.first;
        opts.bus = p.second;
        opts.transport = transport;
        // Ask for q-phase current (the stock format ignores it, which made
        // every current reading NaN) and arm the per-command watchdog.
        opts.query_format = rf::robot_query_format();
        opts.position_format = rf::robot_position_format();
        controllers[opts.id] = std::make_shared<mjbots::moteus::Controller>(opts);
    }

    // Issue a stop command to all controllers (clear faults before starting)
    for (const auto &pair : controllers)
    {
        pair.second->SetStop();
    }
}

std::map<int, MotorTelemetry> Telemetry::cycle(const std::map<int, double> &velocity_map,
                                               bool energize)
{
    // Build command frames
    std::vector<mjbots::moteus::CanFdFrame> command_frames;
    command_frames.reserve(controllers.size());

    for (const auto &pair : controllers)
    {
        if (!energize)
        {
            // Coast: cut motor output but keep querying telemetry.
            command_frames.push_back(pair.second->MakeStop());
            continue;
        }
        mjbots::moteus::PositionMode::Command position_command;
        position_command.position = std::numeric_limits<double>::quiet_NaN();
        auto it = velocity_map.find(pair.first);
        position_command.velocity = (it != velocity_map.end()) ? it->second : 0.0;
        // Hardware failsafe: the motor stops itself if no further command
        // arrives within this window (loop hang, process kill, CAN drop).
        position_command.watchdog_timeout = watchdog_timeout_s;
        // Controller-level smoothing between 100 Hz updates (NaN = default).
        position_command.velocity_limit = velocity_limit_rev_s;
        position_command.accel_limit = accel_limit_rev_s2;
        command_frames.push_back(pair.second->MakePosition(position_command));
    }

    // Send all commands in one transaction and collect replies, sampling the
    // pi3hat IMU in the same pass (the transport requests attitude whenever
    // a destination struct is supplied).
    std::vector<mjbots::moteus::CanFdFrame> replies;

    if (!command_frames.empty())
    {
        mjbots::pi3hat::Attitude attitude;
        mjbots::pi3hat::Pi3Hat::Output pi3hat_output;
        mjbots::moteus::BlockingCallback cbk;
        transport->Cycle(command_frames.data(), command_frames.size(), &replies,
                         &attitude, &pi3hat_output, nullptr, cbk.callback());
        cbk.Wait();

        attitude_present = pi3hat_output.attitude_present;
        if (attitude_present)
        {
            imu_yaw_dps = attitude.rate_dps.z;
            // Yaw Euler angle from the attitude quaternion, degrees.
            const auto &q = attitude.attitude;
            imu_heading_deg = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z)) *
                              (180.0 / M_PI);
        }
        else
        {
            imu_yaw_dps = std::numeric_limits<double>::quiet_NaN();
            imu_heading_deg = std::numeric_limits<double>::quiet_NaN();
        }
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
        mt.current = parsed.q_current; // real amps now that the query requests it
        // mt.position = parsed.position;
        mt.mode = static_cast<int>(parsed.mode);
        mt.fault = static_cast<int>(parsed.fault);
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