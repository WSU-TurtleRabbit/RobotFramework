#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <limits>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

struct MotorTelemetry
{
    double temperature;
    double voltage;
    double velocity;
    double current;
    int mode;
};

class Telemetry
{
public:
    Telemetry();

    // Query all telemetry in one cycle (like the example)
    std::map<int, MotorTelemetry> cycle(const std::map<int, double>& velocity_map);

    // controllers keyed by CAN ID
    std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers;

    // shared transport instance used for BlockingCycle
    std::shared_ptr<mjbots::pi3hat::Pi3HatMoteusTransport> transport;

    std::map<int, int> Yamel_Load_MotorMap(const std::string& path);
};

#endif // TELEMETRY_H