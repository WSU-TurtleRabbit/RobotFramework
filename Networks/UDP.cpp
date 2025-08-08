#include "UDP.h"

Reciver::Reciver() {
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(r_port);

    tv.tv_usec = 20000; 
    tv.tv_sec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); 

    bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
    
    len = sizeof(client_addr);
};

std::string Reciver::recive() {

    int latest_msg = recvfrom(sockfd, buffer, buffer_size , 0, (struct sockaddr*)&client_addr, &len);
    if (latest_msg < 0) {
    return "TIMEOUT";  // Or handle timeout/error
    }
    buffer[latest_msg] = '\0';
    return buffer;

};

void Reciver::clear_buffer() {
    int discard_msg;
    while(discard_msg != 0)
        discard_msg = recvfrom(sockfd, buffer, buffer_size, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);
    };
};

void Reciver::close_socket() {
    close(this->sockfd);
}

// Sender junk
Sender::Sender(const std::string& ip_address, const int& port) {

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip_address.c_str(), &server_addr.sin_addr);
};

void Sender::send(const std::string& message) {
    sendto(sockfd, message.c_str(), message.size(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
};

void Sender::clear_buffer() {
    memset(buffer, 0, BUFFER_SIZE);
}

void Sender::close_socket() {
    close();
}