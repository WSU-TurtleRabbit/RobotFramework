#include <iostream>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "wheel_math.h"

Wheel_math::Wheel_math() {
    initalize_math();
}

void Wheel_math::initalize_math() {
    // Drive-scale calibration: meters of body travel per motor output
    // revolution. Optional override of the physical default (2*pi*R for the
    // direct-drive 33.5 mm wheel).
    try {
        YAML::Node motor = YAML::LoadFile("../config/Motor.yaml");
        if (motor["metersPerMotorRev"]) {
            const double v = motor["metersPerMotorRev"].as<double>();
            if (v > 0.0) kin.meters_per_motor_rev = v;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading Motor config for kinematics: " << e.what() << std::endl;
        // Keep the physical default from Kinematics.
    }
}

std::vector<double> Wheel_math::calculate(double velocity_x, double velocity_y, double velocity_w) {
    // ---- Omni wheel inverse kinematics (motor rev/s) ----
    const std::array<double, 4> rev_s =
        kin.inverse(BodyTwist{velocity_x, velocity_y, velocity_w});
    return {rev_s[0], rev_s[1], rev_s[2], rev_s[3]};
}
