#include "UDP.h"
#include <yaml-cpp/yaml.h>
#include <vector>


UDP::UDP() {

    try {
    YAML::Node config = YAML::LoadFile("../config/Network.yaml");
    YAML::Node network = config["network"];

    buffer_size   = network["bufferSize"].as<int>();
    receiver_port  = network["receiver_port"].as<int>();
    sender_port   = network["sender_port"].as<int>();


    } catch (const std::exception& e) {
    std::cerr << "Error loading network config: " << e.what() << std::endl;
    // Provide fallback defaults
    buffer_size = 1024;
    receiver_port = 50514;
    sender_port = 50513;
    }

    buffer.resize(buffer_size);

    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(receiver_port);

    tv.tv_usec = 50000; 
    tv.tv_sec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); 

    bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));

    
    Msg_found;
    len = sizeof(client_addr);
};

std::string UDP::receive() {

    int latest_msg = recvfrom(sockfd, buffer.data(), buffer_size , 0, (struct sockaddr*)&client_addr, &len);
    if (latest_msg < 0) {
        Msg_found = false;
    return "TIMEOUT";  // Or handle timeout/error

    }
    buffer[latest_msg] = '\0';
    Msg_found = true;
    return std::string(buffer.data());

};

void UDP::clear_buffer() {
    int discard_msg = 1;
    while(discard_msg != 0)
    {
        discard_msg = recvfrom(sockfd, buffer.data(), buffer_size, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);
    };
};


void UDP::send(const std::string& message){
    sendto(sockfd, message.c_str(), message.size(), 0,
           (struct sockaddr*)&target_addr, sizeof(target_addr));

    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(sender_port);  

    // Copy the IP from the sender
    target_addr.sin_addr = client_addr.sin_addr;
}


void UDP::close_socket() {
    close(this->sockfd);
}
