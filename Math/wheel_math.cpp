#include <iostream>
#include <vector>
#include <cmath>
#include "wheel_math.h"

// double Wheel_math::deg2rad(int angle){
//     return angle * M_PI/180;
// }

std::vector<double> Wheel_math::calculate(double velocity_x, double velocity_y, double velocity_w){
      std::vector<double> stop_vel(4);
      for(int i = 0; i< 3; i++)
        stop_vel[i] = 0.0;
    if(std::abs(velocity_x) > 0.5){
        std::cout << "error:incomming x is too large" << "\n";
        return stop_vel;
    }
    if(std::abs(velocity_y) > 0.5){
        std::cout << "error:incomming y is too large" << "\n";
        return stop_vel;
    }
    if(std::abs(velocity_w) > 0.1){
        std::cout << "error:incomming w is too large" << "\n";
        return stop_vel;
    }
    std::vector<double> wheel_vel(4);
    wheel_vel[0] = {(1/WHEEL_RADIUS) * ((wheel_dist_1*velocity_w) - (velocity_x * sin(WHEEL_1_RAD)) + (velocity_y * cos(WHEEL_1_RAD)))};
    wheel_vel[1] = {(1/WHEEL_RADIUS) * ((wheel_dist_2*velocity_w) - (velocity_x * sin(WHEEL_2_RAD)) + (velocity_y * cos(WHEEL_2_RAD)))};
    wheel_vel[2] = {(1/WHEEL_RADIUS) * ((wheel_dist_3*velocity_w) - (velocity_x * sin(WHEEL_3_RAD)) + (velocity_y * cos(WHEEL_3_RAD)))};
    wheel_vel[3] = {(1/WHEEL_RADIUS) * ((wheel_dist_4*velocity_w) - (velocity_x * sin(WHEEL_4_RAD)) + (velocity_y * cos(WHEEL_4_RAD)))}; 
    return wheel_vel;
}
