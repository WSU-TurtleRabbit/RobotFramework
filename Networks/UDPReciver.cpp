#include <iostream>
#include <cstring>
#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <sstream>
#include "decode.h"
#include "wheel_math.h"
#include "UDP.h"

int main(){
  Reciver r;
  Wheel_math m;
  std::string msg;
  cmdDecoder cmd;
  std::vector <double> vel;
  while(true){
    r.clear_buffer();
    msg = r.recive();
    // ::printf(msg);
    cmd.decode_cmd(msg);
    vel = {m.calculate(cmd.velocity_x, cmd.velocity_y, cmd.velocity_w)};

    std::cout << "Wheel input velocity is " << " x: "<< cmd.velocity_x << " y: "<< cmd.velocity_y << " w :" << cmd.velocity_w << std::endl;
    // std::cout << "Wheel one is :" << vel[0] << " Wheel 2 is :" << vel[1] << " Wheel 3 is :" << vel[2] << " Wheel 4 is :" << vel[3] << std::endl; 
  };
};