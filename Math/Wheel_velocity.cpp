#include <iostream>
#include <vector>
#include <cmath>
#include "wheel_math.h"

double Wheel_math::deg2rad(int angle){
    return angle * M_PI/180;
}

std::vector<double> Wheel_math::calculate(double velocity_x, double velocity_y, double velocity_w){
    std::vector<double> wheel_vel(4);
    wheel_vel[0] = {(1/radius)*(wheel_dist_1*velocity_w) - (velocity_x * sin(deg2rad(wheel_1_ang)))+ (velocity_y * cos(deg2rad(wheel_1_ang)))};
    wheel_vel[1] = {(1/radius)*(wheel_dist_2*velocity_w) - (velocity_x * sin(deg2rad(wheel_2_ang)))+ (velocity_y * cos(deg2rad(wheel_2_ang)))};
    wheel_vel[2] = {(1/radius)*(wheel_dist_3*velocity_w) - (velocity_x * sin(deg2rad(wheel_3_ang)))+ (velocity_y * cos(deg2rad(wheel_3_ang)))};
    wheel_vel[3] = {(1/radius)*(wheel_dist_4*velocity_w) - (velocity_x * sin(deg2rad(wheel_4_ang)))+ (velocity_y * cos(deg2rad(wheel_4_ang)))}; 
    return wheel_vel;
}
