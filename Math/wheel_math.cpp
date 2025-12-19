#include <iostream>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "wheel_math.h"

Wheel_math::Wheel_math() {
    initalize_math();
}

void Wheel_math::initalize_math() {
    try{
    YAML::Node config = YAML::LoadFile("../config/Safety.yaml");
    YAML::Node vLimits = config["velocityLimit"];

    X_LIMIT = vLimits["xLimit"].as<double>();   // m/s
    Y_LIMIT = vLimits["yLimit"].as<double>();   // m/s
    W_LIMIT = vLimits["wLimit"].as<double>();   // rad/s
    }catch (const std::exception& e) {
    std::cerr << "Error loading Velocity Limit config: " << e.what() << std::endl;
    X_LIMIT = 0.5;
    Y_LIMIT = 0.5;
    W_LIMIT = 0.1;
    }
}

std::vector<double> Wheel_math::calculate(double velocity_x, double velocity_y, double velocity_w) {

    // ---- Limit Checking ----
    if (mode == 0)
    {
        // Safe mode, stops movement
        if (std::abs(velocity_x) > X_LIMIT) {
            std::cout << "error: incoming x is too large\n";
            return stop_vel;
        }
        if (std::abs(velocity_y) > Y_LIMIT) {
            std::cout << "error: incoming y is too large\n";
            return stop_vel;
        }
        if (std::abs(velocity_w) > W_LIMIT) {
            std::cout << "error: incoming w is too large\n";
            return stop_vel;
        }
    }
    else if (mode == 1)
    {
        // Capped mode, stops scales down movement if values exceed
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

    // ---- Omni Wheel Kinematics ----
    wheel_vel[0] = ( (-velocity_x * std::sin(W1_RAD)) +
                     (velocity_y * std::cos(W1_RAD)) +
                     (velocity_w * wheel_dist_1) ) / WHEEL_RADIUS;

    wheel_vel[1] = ( (-velocity_x * std::sin(W2_RAD)) +
                     (velocity_y * std::cos(W2_RAD)) +
                     (velocity_w * wheel_dist_2) ) / WHEEL_RADIUS;

    wheel_vel[2] = ( (-velocity_x * std::sin(W3_RAD)) +
                     (velocity_y * std::cos(W3_RAD)) +
                     (velocity_w * wheel_dist_3) ) / WHEEL_RADIUS;

    wheel_vel[3] = ( (-velocity_x * std::sin(W4_RAD)) +
                     (velocity_y * std::cos(W4_RAD)) +
                     (velocity_w * wheel_dist_4) ) / WHEEL_RADIUS;

    return wheel_vel;
}

void Wheel_math::setMode(int base_mode)
{
    mode = base_mode;
    std::cout << "Mode set to: " << mode;
}