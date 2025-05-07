#include <iostream>
#include <vector>
#include <cmath>

double deg2rad(int angle){
    return angle * M_PI/180;

}


int vx = 1;
int vw = 0;
int vy = 5;

int main(){
std::vector<double> wheel_1 = {63.6, 36.87};
std::vector<double> wheel_2 = {52.14, -52.14};
std::vector<double> wheel_3 = {-52.14, -52.14};
std::vector<double> wheel_4 = {-63.86, 36.87};

int wheel_1_ang = -120;
int wheel_2_ang = -45;
int wheel_3_ang = 45;
int wheel_4_ang = 120;

double radius = {33.5};

float wheel_dist_1 = sqrt(pow(wheel_1[0],2) + pow(wheel_1[1],2));
float wheel_dist_2 = sqrt(pow(wheel_2[0],2) + pow(wheel_2[1],2));
float wheel_dist_3 = sqrt(pow(wheel_3[0],2) + pow(wheel_3[1],2));
float wheel_dist_4 = sqrt(pow(wheel_4[0],2) + pow(wheel_4[1],2));


std::vector<double> wheel_vel(4);


wheel_vel[0] = {(1/radius)*(wheel_dist_1*vw) - (vx * sin(deg2rad(wheel_1_ang)))+ (vy * cos(deg2rad(wheel_1_ang)))};
wheel_vel[1] = {(1/radius)*(wheel_dist_2*vw) - (vx * sin(deg2rad(wheel_2_ang)))+ (vy * cos(deg2rad(wheel_2_ang)))};
wheel_vel[2] = {(1/radius)*(wheel_dist_3*vw) - (vx * sin(deg2rad(wheel_3_ang)))+ (vy * cos(deg2rad(wheel_3_ang)))};
wheel_vel[3] = {(1/radius)*(wheel_dist_4*vw) - (vx * sin(deg2rad(wheel_4_ang)))+ (vy * cos(deg2rad(wheel_4_ang)))}; 

std::cout << wheel_vel[0] << ":"<< wheel_vel[1] << ":"<< wheel_vel[2]<< ":"<< wheel_vel[3] << std::endl;

return 0;
}