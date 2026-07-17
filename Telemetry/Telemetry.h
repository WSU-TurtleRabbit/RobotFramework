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
    double current;   // q-phase current, A (requires q_current in the query format)
    // double position;
    int mode;
    int fault;        // moteus fault code; 0 = none
};

class Telemetry
{
public:
    Telemetry();

    // Query all telemetry in one cycle (like the example)
    std::map<int, MotorTelemetry> cycle(const std::map<int, double>& velocity_map);

    // moteus per-command watchdog, seconds (config/Safety.yaml
    // `watchdogTimeout`): motors self-stop this long after the last command
    // frame if the control loop stalls.
    double watchdog_timeout_s = 0.1;

    // controllers keyed by CAN ID
    std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers;

    // shared transport instance used for BlockingCycle
    std::shared_ptr<mjbots::pi3hat::Pi3HatMoteusTransport> transport;

    std::map<int,int> YAML_Load_MotorMap(const std::string& path);
};

#endif // TELEMETRY_H