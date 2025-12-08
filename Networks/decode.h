
#pragma once

#include <sstream>
#include <string>


class cmdDecoder{
    public:
        int id;
        double velocity_x;
        double velocity_y;
        double velocity_w;
        bool kick;
        bool dribble;
        double time;
    
    public:
        void decode_cmd(std::string message);
};

