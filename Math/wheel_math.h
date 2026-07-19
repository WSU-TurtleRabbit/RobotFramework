#ifndef WHEEL_MATH_H
#define WHEEL_MATH_H

#include <vector>

#include "kinematics.h"

// Thin config-loading wrapper around the pure Kinematics model: loads the
// drive-scale calibration (config/Motor.yaml `metersPerMotorRev`) and turns
// body twists into motor velocity setpoints in output REV/S (what moteus
// expects).
//
// Policy (speed envelopes, fault trips) lives in Safety/supervisor.h — the
// superloop shapes every commanded twist through the supervisor BEFORE it
// reaches this class. All geometry lives in Math/kinematics.h — including
// the corrected rear wheel angle (-135 deg; the old table's -130 was a typo)
// and the rev/s unit chain (the old code divided by the wheel radius,
// producing rad/s, a hidden 2*pi scale error).
class Wheel_math {
private:
    // Pure geometry + scale (host-tested; see tests/host/test_kinematics.cpp).
    Kinematics kin;

    void initalize_math();

public:
    Wheel_math();
    ~Wheel_math() = default;

    // Access to the calibrated geometry (odometry, telemetry).
    const Kinematics& kinematics() const { return kin; }

    // Calculate motor velocity setpoints (rev/s, motor ids 1..4 in order)
    // from the desired body twist. Pure inverse kinematics: the caller is
    // responsible for envelope shaping (Supervisor::shape_twist).
    std::vector<double> calculate(double velocity_x, double velocity_y, double velocity_w);
};

#endif
