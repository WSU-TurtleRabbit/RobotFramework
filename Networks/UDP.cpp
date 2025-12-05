#include "UDP.h"


UDP::UDP() {
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(r_port);

    tv.tv_usec = 50000; 
    tv.tv_sec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); 

    bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));

    
    Msg_found;
    len = sizeof(client_addr);
};

std::string UDP::recive() {

    int latest_msg = recvfrom(sockfd, buffer, buffer_size , 0, (struct sockaddr*)&client_addr, &len);
    if (latest_msg < 0) {
        Msg_found = false;
    return "TIMEOUT";  // Or handle timeout/error

    }
    buffer[latest_msg] = '\0';
    Msg_found = true;
    return buffer;

};

void UDP::clear_buffer() {
    int discard_msg = 1;
    while(discard_msg != 0)
    {
        discard_msg = recvfrom(sockfd, buffer, buffer_size, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);
    };
};


void UDP::send(const std::string& message){
    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(50513);  // your chosen port

    // Copy the IP from the sender
    target_addr.sin_addr = client_addr.sin_addr;

    sendto(sockfd, message.c_str(), message.size(), 0,
           (struct sockaddr*)&target_addr, sizeof(target_addr));
}


void UDP::close_socket() {
    close(this->sockfd);
}
