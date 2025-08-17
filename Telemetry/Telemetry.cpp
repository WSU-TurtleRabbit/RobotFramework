#include "Telemetry.h"

namespace moteus {

Telemetry()
{
    this->motorControllers = {1, 2, 3, 4};
}

/**
* @return Vector of doubles containing the temperature of each motor.
*/
std::vector<double> Telemetry::queryTemperature()
{
    std::vector<double> result;
    std::vector<CanFdFrame> queryFrames, replyFrames;

    for (const auto& pair : motorControllers)
    {
        queryFrames.push_back(pair.second->
            MakeQuery(Query::Format.temperature));
    }

    transport->BlockingCycle(&queryFrames[0], queryFrames.size(), &replyFrames);

    for (const auto& frame : replyFrames)
    {
        auto parsed = Query::Parse(frame.data, frame.size);
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
    std::vector<CanFdFrame> queryFrames, replyFrames;

    for (const auto& pair : motorControllers)
    {
        queryFrames.push_back(pair.second->
            MakeQuery(Query::Format.voltage));
    }

    transport->BlockingCycle(&queryFrames[0], queryFrames.size(), &replyFrames);

    for (const auto& frame : replyFrames)
    {
        auto parsed = Query::Parse(frame.data, frame.size);
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
    std::vector<CanFdFrame> queryFrames, replyFrames;
    auto parsedResult;

    auto it = motorControllers.find(motorID);
    if (it != motorControllers.end())
    {
        queryFrames.push_back(it->second->
            MakeQuery(Query::Format.voltage));
    }
    else
    {
        return NaN;
    }

    transport->BlockingCycle(&queryFrames[0], queryFrames.size(), &replyFrames);

    for (const auto& frame : replyFrames)
    {
        parsedResult = Query::Parse(frame.data, frame.size);
    }

    return parsedResult;
}

}