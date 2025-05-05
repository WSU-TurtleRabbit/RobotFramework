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


class Reciver{
    private:
    static const int buffer_size = 1024;
    static const int r_port = 5014;
    struct sockaddr_in server_addr, client_addr;
    char buffer[buffer_size];
    socklen_t len; 
    int sockfd;

    public:
    Reciver(){
        char buffer[buffer_size];
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(r_port);

        bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        len = sizeof(client_addr);
    }

    std::string recive(){
        int n = recvfrom(sockfd, buffer, buffer_size, 0, (struct sockaddr*)&client_addr, &len);
        buffer[n] = '\0';
        return buffer;
    }
};
int main(){
  Reciver r;
  message
  r.recive()
}