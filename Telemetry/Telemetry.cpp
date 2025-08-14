#include "Telemetry.h"

Telemetry::Telemetry() {
}
namespace moteus {

float Telemetry::queryTemperature() {
    auto result;
    std::vector<CanFdFrame> queryFrames, replyFrames;
    for (const auto& pair : controller)
    {
        queryFrames.push_back(pair.second->
            MakeQuery(Query::Format.temperature));
    }

    transport->BlockingCycle(&queryFrames[0], queryFrames.size(), &replyFrames);

    for (const auto& frame : reply)
    {
        result = Query::Parse(frame.data, frame.size);
        return result.temperature;
    }

    return NaN;
}
}