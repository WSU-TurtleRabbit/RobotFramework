#ifndef UDP_H
#define UDP_H

#pragma once

#include <iostream>
#include <cstring>
#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sstream>

#if defined(__unix__) || defined(__APPLE__)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#endif

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
        ~Reciver()=default;
        std::string recive();
        void clear_buffer();
};

#endif // UDP_H


