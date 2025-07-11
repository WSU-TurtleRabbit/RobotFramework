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
    do{
        discard_msg = recvfrom(sockfd, buffer, buffer_size, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);
        
    } while(discard_msg > 0); 
};

