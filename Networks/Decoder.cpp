#include <iostream>
#include <sstream>
#include <string>
#include "decode.h"

    void cmdDecoder::decode_cmd(std::string message){
    std::stringstream iss(message);
    iss >> velocity_x >> velocity_y >> velocity_w >> kick>> dribble>> time;
    };