#ifndef WHEEL_MATH_H
#define WHEEL_MATH_H
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

class Wheel_math{
    private:

    const std::vector<double> wheel_1 = {63.6, 36.87};
    const std::vector<double> wheel_2 = {52.14, -52.14};
    const std::vector<double> wheel_3 = {-52.14, -52.14};
    const std::vector<double> wheel_4 = {-63.86, 36.87};

    const int wheel_1_ang = -120;
    const int wheel_2_ang = -45;
    const int wheel_3_ang = 45;
    const int wheel_4_ang = 120;

    const double radius = 33.5;

    

    float wheel_dist_1 = sqrt(pow(wheel_1[0],2) + pow(wheel_1[1],2));
    float wheel_dist_2 = sqrt(pow(wheel_2[0],2) + pow(wheel_2[1],2));
    float wheel_dist_3 = sqrt(pow(wheel_3[0],2) + pow(wheel_3[1],2));
    float wheel_dist_4 = sqrt(pow(wheel_4[0],2) + pow(wheel_4[1],2));


    public:
    // SetWheelDist();
    std::vector<double> calculate(double velocity_x, double velocity_y, double velocity_w);
    double deg2rad(int angle);

};


#endif