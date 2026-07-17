#include <iostream>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "wheel_math.h"

Wheel_math::Wheel_math() {
    initalize_math();
}

void Wheel_math::initalize_math() {
    try {
        YAML::Node config = YAML::LoadFile("../config/Safety.yaml");
        YAML::Node vLimits = config["velocityLimit"];

        X_LIMIT = vLimits["xLimit"].as<double>();   // m/s
        Y_LIMIT = vLimits["yLimit"].as<double>();   // m/s
        W_LIMIT = vLimits["wLimit"].as<double>();   // rad/s
    } catch (const std::exception& e) {
        std::cerr << "Error loading Velocity Limit config: " << e.what() << std::endl;
        X_LIMIT = 0.5;
        Y_LIMIT = 0.5;
        W_LIMIT = 0.1;
    }

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
    // ---- Limit Checking ----
    if (mode == 0)
    {
        // Safe mode, stops movement
        if (std::abs(velocity_x) > X_LIMIT) {
            std::cout << "error: incoming x is too large\n";
            return {0.0, 0.0, 0.0, 0.0};
        }
        if (std::abs(velocity_y) > Y_LIMIT) {
            std::cout << "error: incoming y is too large\n";
            return {0.0, 0.0, 0.0, 0.0};
        }
        if (std::abs(velocity_w) > W_LIMIT) {
            std::cout << "error: incoming w is too large\n";
            return {0.0, 0.0, 0.0, 0.0};
        }
    }
    else if (mode == 1)
    {
        // Capped mode, scales down movement if values exceed
        double scale = 1;
        if (std::abs(velocity_x) > X_LIMIT) {
            scale = std::min(scale, X_LIMIT / std::abs(velocity_x));
        }
        if (std::abs(velocity_y) > Y_LIMIT) {
            scale = std::min(scale, Y_LIMIT / std::abs(velocity_y));
        }
        if (std::abs(velocity_w) > W_LIMIT) {
            scale = std::min(scale, W_LIMIT / std::abs(velocity_w));
        }

        velocity_x *= scale;
        velocity_y *= scale;
        velocity_w *= scale;
    }

    // ---- Omni wheel inverse kinematics (motor rev/s) ----
    const std::array<double, 4> rev_s =
        kin.inverse(BodyTwist{velocity_x, velocity_y, velocity_w});
    return {rev_s[0], rev_s[1], rev_s[2], rev_s[3]};
}

std::vector<double> Wheel_math::calculateCapped(double velocity_x, double velocity_y, double velocity_w) {
    // Uniform scale into the configured limits: direction preserved.
    double scale = 1;
    if (std::abs(velocity_x) > X_LIMIT) {
        scale = std::min(scale, X_LIMIT / std::abs(velocity_x));
    }
    if (std::abs(velocity_y) > Y_LIMIT) {
        scale = std::min(scale, Y_LIMIT / std::abs(velocity_y));
    }
    if (std::abs(velocity_w) > W_LIMIT) {
        scale = std::min(scale, W_LIMIT / std::abs(velocity_w));
    }

    const std::array<double, 4> rev_s = kin.inverse(
        BodyTwist{velocity_x * scale, velocity_y * scale, velocity_w * scale});
    return {rev_s[0], rev_s[1], rev_s[2], rev_s[3]};
}

void Wheel_math::setMode(int base_mode)
{
    mode = base_mode;
    std::cout << "Mode set to: " << mode;
}
