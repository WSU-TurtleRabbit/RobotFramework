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

    // Send one command to every motor and collect telemetry replies, all in
    // a single CAN transaction that also samples the pi3hat IMU.
    // energize=false sends STOP frames instead of velocity commands (coast:
    // motor output cut, telemetry keeps flowing) — the EMERGENCY landed
    // state and the first-vision gate use this.
    // ff_torque_nm: optional per-motor feedforward torque (the MatchCtrl
    // cascade's model FF; nullptr = 0 N-m, neutral).
    std::map<int, MotorTelemetry> cycle(const std::map<int, double>& velocity_map,
                                        bool energize = true,
                                        const std::map<int, double>* ff_torque_nm = nullptr);

    // moteus per-command watchdog, seconds (config/Safety.yaml
    // `watchdogTimeout`): motors self-stop this long after the last command
    // frame if the control loop stalls.
    double watchdog_timeout_s = 0.1;

    // moteus-level per-command shaping (config/Motor.yaml `velocityLimit` /
    // `accelLimit`, output rev/s and rev/s^2). NaN = controller default.
    double velocity_limit_rev_s = std::numeric_limits<double>::quiet_NaN();
    double accel_limit_rev_s2 = std::numeric_limits<double>::quiet_NaN();

    // pi3hat IMU sample from the most recent cycle(). Yaw rate is the RAW
    // gyro z-axis in deg/s (mounting polarity is applied downstream via
    // config imu.yaw_rate_sign). Heading is the attitude quaternion's yaw
    // Euler angle in degrees.
    bool attitude_present = false;
    double imu_yaw_dps = std::numeric_limits<double>::quiet_NaN();
    double imu_heading_deg = std::numeric_limits<double>::quiet_NaN();

    // controllers keyed by CAN ID
    std::map<int, std::shared_ptr<mjbots::moteus::Controller>> controllers;

    // shared transport instance used for BlockingCycle
    std::shared_ptr<mjbots::pi3hat::Pi3HatMoteusTransport> transport;

    std::map<int,int> YAML_Load_MotorMap(const std::string& path);
};

#endif // TELEMETRY_H