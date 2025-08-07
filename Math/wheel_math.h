#ifndef WHEEL_MATH_H
#define WHEEL_MATH_H
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

class Wheel_math{
    private:

    const std::vector<double> WHEEL_1 = {63.6, 36.87};
    const std::vector<double> WHEEL_2 = {52.14, -52.14};
    const std::vector<double> WHEEL_3 = {-52.14, -52.14};
    const std::vector<double> WHEEL_4 = {-63.86, 36.87};

    const int WHEEL_1_ANG = 45;
    const int WHEEL_2_ANG = -45;
    const int WHEEL_3_ANG = -120;
    const int WHEEL_4_ANG = 120;



    //todo - convert to radians

    const double WHEEL_1_RAD = WHEEL_1_ANG*(M_PI/180);
    const double WHEEL_2_RAD = WHEEL_2_ANG*(M_PI/180);
    const double WHEEL_3_RAD = WHEEL_3_ANG*(M_PI/180);
    const double WHEEL_4_RAD = WHEEL_4_ANG*(M_PI/180);

    const double WHEEL_RADIUS = 33.5;

    float wheel_dist_1 = sqrt(pow(WHEEL_1[0],2) + pow(WHEEL_1[1],2));
    float wheel_dist_2 = sqrt(pow(WHEEL_2[0],2) + pow(WHEEL_2[1],2));
    float wheel_dist_3 = sqrt(pow(WHEEL_3[0],2) + pow(WHEEL_3[1],2));
    float wheel_dist_4 = sqrt(pow(WHEEL_4[0],2) + pow(WHEEL_4[1],2));


    public:

    Wheel_math() = default;
    ~Wheel_math() = default;
    // SetWheelDist();
    std::vector<double> calculate(double velocity_x, double velocity_y, double velocity_w);
    // double deg2rad(int angle);

};


#endif