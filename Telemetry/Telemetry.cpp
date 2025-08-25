#include "Telemetry.h"

Telemetry::Telemetry()
{
    transport = std::make_shared<mjbots::pi3hat::Pi3HatMoteusTransport>();

    for (int id = 1; id <= 4; ++id) {
        motorControllers[id] = std::make_shared<mjbots::moteus::Controller>(id, transport.get());
    }
}

/**
* @return Vector of doubles containing the temperature of each motor.
*/
std::vector<double> Telemetry::queryTemperature()
{
    std::vector<double> result;
    std::vector<mjbots::moteus::CanFdFrame> queryFrames, replyFrames;

    for (const auto& pair : motorControllers)
    {
        queryFrames.push_back(pair.second->MakeQuery(mjbots::moteus::Query::Format::temperature));
    }

    transport->BlockingCycle(queryFrames.data(), queryFrames.size(), &replyFrames);

    for (const auto& frame : replyFrames)
    {
        auto parsed = mjbots::moteus::Query::Parse(frame.data, frame.size);
        result.push_back(parsed.temperature);
    }

    return result;
}

/**
* @return Vector of doubles containing the voltage of each motor.
*/
std::vector<double> Telemetry::queryVoltage()
{
    std::vector<double> result;
    std::vector<mjbots::moteus::CanFdFrame> queryFrames, replyFrames;

    for (const auto& pair : motorControllers)
    {
        queryFrames.push_back(pair.second->MakeQuery(mjbots::moteus::Query::Format::voltage));
    }

    transport->BlockingCycle(queryFrames.data(), queryFrames.size(), &replyFrames);

    for (const auto& frame : replyFrames)
    {
        auto parsed = mjbots::moteus::Query::Parse(frame.data, frame.size);
        result.push_back(parsed.voltage);
    }

    return result;
}

/**
* @param motorID Integer of motor's CAN ID.
* @return Double of motor's velocity.
*/
double Telemetry::queryVelocity(int motorID)
{
    std::vector<mjbots::moteus::CanFdFrame> queryFrames, replyFrames;

    auto it = motorControllers.find(motorID);
    if (it == motorControllers.end())
    {
        return NaN;
    }

    queryFrames.push_back(it->second->MakeQuery(mjbots::moteus::Query::Format::velocity));
    transport->BlockingCycle(queryFrames.data(), queryFrames.size(), &replyFrames);

    if (!replyFrames.empty()) {
        auto parsed = mjbots::moteus::Query::Parse(replyFrames[0].data, replyFrames[0].size);
        return parsed.velocity;
    } 
    else 
    {
        return NaN;
    }
}

