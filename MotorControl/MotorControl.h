#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

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

struct MotorStatus
{
    double temperature;
    double voltage;
    double velocity;
    double current;
    int mode;
};

class MotorControl
{
public:
    MotorControl();

    // Execute motor commands and query status in one cycle
    std::map<int, MotorStatus> cycle(const std::map<int, double>& velocity_map);

    // Stop all motors
    void stopAll();

    // controllers keyed by CAN ID
    std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers;

    // shared transport instance used for BlockingCycle
    std::shared_ptr<mjbots::pi3hat::Pi3HatMoteusTransport> transport;
};

#endif // MOTORCONTROL_H
