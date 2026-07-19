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
    int receiver_port; 
    int sender_port; 
  
    int sockfd;
    socklen_t len;
    struct sockaddr_in server_addr, client_addr, target_addr;
    struct timeval tv;
    std::vector<char> buffer;

    bool Msg_found;
    // True once at least one packet has been received: only then does
    // client_addr hold a real peer we can send telemetry back to.
    bool has_peer;
    // Failed sendto() calls (telemetry v2 `tx_err`).
    unsigned long long tx_err_count;

public:
    UDP();
    ~UDP() = default;
    std::string receive();
    void clear_buffer();
    void send(const std::string& message);
    void close_socket();
    int getBufferSize();
    int getRecieverPort();
    int getSenderPort();

    // Telemetry v2 link stats.
    unsigned long long tx_errors() const { return tx_err_count; }
    // Best-effort local IP of the interface that routes to the current
    // peer ("0.0.0.0" until a peer is known).
    std::string local_ip();
};

#endif // UDP_H
