#include <iostream>
#include <vector>
#include <cmath>
#include "wheel_math.h"

Wheel_math::Wheel_math() {

}

std::vector<double> Wheel_math::calculate(double velocity_x, double velocity_y, double velocity_w) {

    // ---- Limit Checking ----
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

    // ---- Omni Wheel Kinematics ----
    wheel_vel[0] = ( (velocity_x * std::sin(WHEEL_1_RAD)) +
                     (velocity_y * std::cos(WHEEL_1_RAD)) +
                     (velocity_w * wheel_dist_1) ) / WHEEL_RADIUS;

    wheel_vel[1] = ( (velocity_x * std::sin(WHEEL_2_RAD)) +
                     (velocity_y * std::cos(WHEEL_2_RAD)) +
                     (velocity_w * wheel_dist_2) ) / WHEEL_RADIUS;

    wheel_vel[2] = ( (velocity_x * std::sin(WHEEL_3_RAD)) +
                     (velocity_y * std::cos(WHEEL_3_RAD)) +
                     (velocity_w * wheel_dist_3) ) / WHEEL_RADIUS;

    wheel_vel[3] = ( (velocity_x * std::sin(WHEEL_4_RAD)) +
                     (velocity_y * std::cos(WHEEL_4_RAD)) +
                     (velocity_w * wheel_dist_4) ) / WHEEL_RADIUS;

    return wheel_vel;
}


