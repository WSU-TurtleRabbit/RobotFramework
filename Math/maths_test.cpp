#include <iostream>
#include <vector>
#include <cmath>
#include "wheel_math.h"


int main(){
    Wheel_math wheel;

    std::vector<double> vel = {wheel.calculate(5, 0, 0)};

    std::cout << "Wheel one is " << vel[0] << "Wheel 2 is " << vel[1] << "Wheel 3 is " << vel[2] << "Wheel 4 is " << vel[3] << std::endl; 

return 0;
}