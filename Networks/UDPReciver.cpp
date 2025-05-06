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

class Reciver{
    private:
    static const int buffer_size = 1024;
    static const int r_port = 50514;
    int sockfd;
    char buffer[buffer_size];
    socklen_t len;
    // sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in server_addr, client_addr;




    public:
    Reciver(){
        // int sockfd;
        // char buffer[BUFFER_SIZE];
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(r_port);

        bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        len = sizeof(client_addr);
    }

    std::string recive(){

        int latest_msg  = recvfrom(sockfd, buffer, buffer_size, 0, (struct sockaddr*)&client_addr, &len);
        buffer[latest_msg] = '\0';
        return buffer;
    }

    void clear_buffer(){
        int discard_msg;
        do{
        discard_msg = recvfrom(sockfd, buffer, buffer_size, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);
            
        }while(discard_msg > 0); 
    }
};
int main(){
  Reciver r;
  std::string msg;
  cmdDecoder cmd;
  while(true){
    r.clear_buffer();
    msg = r.recive();
    cmd.decode_cmd(msg);
    std::cout<< "Velocity_x" << cmd.velocity_w<< "Velocity_y" << cmd.velocity_y << "Velocity_w" << cmd.velocity_w << std::endl;
  }
}