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
    // double position;
    int mode;
};

class Telemetry
{
public:
    Telemetry();

    // Query all telemetry in one cycle (like the example). Also reads the
    // pi3hat IMU attitude in the same CAN cycle (see `attitude` / yaw helpers).
    std::map<int, MotorTelemetry> cycle(const std::map<int, double>& velocity_map);

    // controllers keyed by CAN ID
    std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers;

    // shared transport instance used for BlockingCycle
    std::shared_ptr<mjbots::pi3hat::Pi3HatMoteusTransport> transport;

    std::map<int,int> YAML_Load_MotorMap(const std::string& path);

    // --- IMU / attitude (filled by cycle(), used for gyro heading hold) ---
    mjbots::pi3hat::Attitude attitude;   // latest attitude read from the pi3hat
    bool attitude_ok = false;            // true once at least one attitude read succeeded

    double yaw_rad() const;              // heading about the vertical axis (rad)
    double yaw_rate_rps() const;         // yaw rate (rad/s) from the gyro
};

#endif // TELEMETRY_H