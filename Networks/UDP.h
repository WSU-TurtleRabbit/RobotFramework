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

class UDP
{
private:

    int buffer_size;
    int reciver_port; 
    int sender_port; 
  
    int sockfd;
    socklen_t len;
    struct sockaddr_in server_addr, client_addr, target_addr;
    struct timeval tv;
    std::vector<char> buffer;

public:
    UDP();
    ~UDP() = default;
    std::string recive();
    void clear_buffer();
    void send(const std::string& message);
    void close_socket();
};

#endif // UDP_H
