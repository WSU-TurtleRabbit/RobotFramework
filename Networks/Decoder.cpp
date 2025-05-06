#include <iostream>
#include <sstream>

class cmdDecoder{
    public:
    double velocity_x;
    double velocity_y;
    double velocity_w;
    bool kick;
    bool dribble;
    double time;
    
    public:

    void decode_cmd(std::string message){
    std::stringstream ss(message);
    ss >> velocity_x >> velocity_y >> velocity_w >> kick>> dribble>> time;
    }
};

// int main(){

//     std::string message = "1 4 2 1 0";

//     cmdDecoder cmd;
//     cmd.decode_cmd(message);

//     std::cout<< "Vx:"<< cmd.velocity_x << " Vy:" << cmd.velocity_y<< " W"<<cmd.velocity_w<< std::endl;

//     return 0;
// };