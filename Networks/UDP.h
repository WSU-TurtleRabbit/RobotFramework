
#pragma once


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


class Reciver{
    private:
    static const int buffer_size = 1024;
    static const int r_port = 50514;
    int sockfd;
    char buffer[buffer_size];
    socklen_t len;
    struct sockaddr_in server_addr, client_addr;
    

    public:
    Reciver();
    std::string recive();
    void clear_buffer();
};



