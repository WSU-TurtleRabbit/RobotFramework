#ifndef WHEEL_MATH_H
#define WHEEL_MATH_H

#include <cmath>
#include <vector>

class Wheel_math {
private:
    // Wheel positions in meters (x, y)
    const std::vector<double> WHEEL_1 = {63.6/1000.0, 36.87/1000.0};
    const std::vector<double> WHEEL_2 = {52.14/1000.0, -52.14/1000.0};
    const std::vector<double> WHEEL_3 = {-52.14/1000.0, -52.14/1000.0};
    const std::vector<double> WHEEL_4 = {-63.6/1000.0, 36.87/1000.0};

    // Wheel orientation (degrees)
    const double W1_ANG = 45.0;
    const double W2_ANG = -45.0;
    const double W3_ANG = -120.0;
    const double W4_ANG = 120.0;

    // Stop state
    std::vector<double> stop_vel = {0.0, 0.0, 0.0, 0.0};

    // Max velocity limits
    const float X_LIMIT = 0.5;   // m/s
    const float Y_LIMIT = 0.5;   // m/s
    const float W_LIMIT = 0.1;   // rad/s

    // Wheel computed velocities
    std::vector<double> wheel_vel = {0.0, 0.0, 0.0, 0.0};

    // Angles converted to radians
    const double W1_RAD = W1_ANG * (M_PI / 180.0);
    const double W2_RAD = W2_ANG * (M_PI / 180.0);
    const double W3_RAD = W3_ANG * (M_PI / 180.0);
    const double W4_RAD = W4_ANG * (M_PI / 180.0);

    // Wheel radius (meters)
    const double WHEEL_RADIUS = 33.5 / 1000.0;

    // Distances from robot center
    float wheel_dist_1 = std::hypot(WHEEL_1[0], WHEEL_1[1]);
    float wheel_dist_2 = std::hypot(WHEEL_2[0], WHEEL_2[1]);
    float wheel_dist_3 = std::hypot(WHEEL_3[0], WHEEL_3[1]);
    float wheel_dist_4 = std::hypot(WHEEL_4[0], WHEEL_4[1]);

public:
    Wheel_math() = default;
    ~Wheel_math() = default;

    // Calculate wheel velocities from robot desired motion
    std::vector<double> calculate(double velocity_x, double velocity_y, double velocity_w);
};

#endif
