#ifndef DECODE_H
#define DECODE_H
#include <sstream>
#include <string>


class cmdDecoder{
    public:
    double velocity_x;
    double velocity_y;
    double velocity_w;
    bool kick;
    bool dribble;
    double time;
    
    public:
    void decode_cmd(std::string message);
};

#endif