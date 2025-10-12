#include "MotorControl.h"
#include <limits>

MotorControl::MotorControl()
{
    // Transport configuration for Pi3Hat
    mjbots::pi3hat::Pi3HatMoteusTransport::Options toptions;
    std::map<int, int> servo_map = {
        // Motor ID → Bus pair mapping
        {1, 1}, // Motor ID 1 mapped to BUS 1
        {4, 4}, // Motor ID 4 mapped to BUS 1
        {2, 2}, // Motor ID 2 mapped to BUS 2
        {3, 3}, // Motor ID 3 mapped to BUS 2
    };
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
    stopAll();
}

std::map<int, MotorStatus> MotorControl::cycle(const std::map<int, double> &velocity_map)
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
    std::map<int, MotorStatus> motor_status_data;
    for (const auto &frame : replies)
    {
        auto parsed = mjbots::moteus::Query::Parse(frame.data, frame.size);
        MotorStatus ms;
        ms.temperature = parsed.temperature;
        ms.voltage = parsed.voltage;
        ms.velocity = parsed.velocity;
        ms.current = parsed.q_current;
        ms.mode = static_cast<int>(parsed.mode);
        motor_status_data[frame.source] = ms;
    }

    return motor_status_data;
}

void MotorControl::stopAll()
{
    for (const auto &pair : controllers)
    {
        pair.second->SetStop();
    }
}
