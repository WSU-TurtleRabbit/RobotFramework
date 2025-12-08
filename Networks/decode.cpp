#include <iostream>
#include "decode.h"

void cmdDecoder::decode_cmd(std::string message){
    std::stringstream iss(message);
    iss >> id >> velocity_x >> velocity_y >> velocity_w >> kick>> dribble>> time;
};