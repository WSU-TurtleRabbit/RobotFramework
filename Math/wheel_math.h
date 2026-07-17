#ifndef WHEEL_MATH_H
#define WHEEL_MATH_H

#include <vector>

#include "kinematics.h"

// Thin config-loading wrapper around the pure Kinematics model: loads the
// velocity limits (config/Safety.yaml) and the drive-scale calibration
// (config/Motor.yaml `metersPerMotorRev`), applies the operator mode policy,
// and returns motor velocity setpoints in output REV/S (what moteus expects).
//
// All geometry lives in Math/kinematics.h — including the corrected rear
// wheel angle (-135 deg; the old table's -130 was a typo) and the rev/s unit
// chain (the old code divided by the wheel radius, producing rad/s, a hidden
// 2*pi scale error).
class Wheel_math {
private:
    // Pure geometry + scale (host-tested; see tests/host/test_kinematics.cpp).
    Kinematics kin;

    // Max velocity limits (from config/Safety.yaml)
    double X_LIMIT;   // m/s
    double Y_LIMIT;   // m/s
    double W_LIMIT;   // rad/s

    // Running mode for safety: 0 = SAFE (stop on over-limit),
    // 1 = CAPPED (scale down), 2 = UNSAFE (no limits).
    int mode = 0;

    void initalize_math();

public:
    Wheel_math();
    ~Wheel_math() = default;
    void setMode(int base_mode);

    // Access to the calibrated geometry (odometry, telemetry).
    const Kinematics& kinematics() const { return kin; }

    // Calculate motor velocity setpoints (rev/s, motor ids 1..4 in order)
    // from the desired body twist, applying the mode's limit policy.
    std::vector<double> calculate(double velocity_x, double velocity_y, double velocity_w);
};

#endif
