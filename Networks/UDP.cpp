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

    // The address structs start zeroed: client_addr/target_addr hold garbage
    // otherwise, and telemetry sent before the first command would go to a
    // garbage destination.
    std::memset(&server_addr, 0, sizeof(server_addr));
    std::memset(&client_addr, 0, sizeof(client_addr));
    std::memset(&target_addr, 0, sizeof(target_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(receiver_port);

    tv.tv_usec = 50000; // 50 ms receive timeout
    tv.tv_sec = 0;

    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));

    Msg_found = false;
    has_peer = false;
    len = sizeof(client_addr);
};

std::string UDP::receive() {

    ssize_t bytes;
    ssize_t last_bytes = -1;

    // Drain all queued packets, keep only the newest
    while (true) {
        bytes = recvfrom(sockfd, buffer.data(), buffer_size - 1, MSG_DONTWAIT,
                         (struct sockaddr*)&client_addr, &len);
        if (bytes < 0) break;
        last_bytes = bytes;
    }

    if (last_bytes < 0) {
        Msg_found = false;
        return "TIMEOUT";
    }

    buffer[last_bytes] = '\0';
    Msg_found = true;
    has_peer = true; // client_addr now holds a real sender to reply to
    return std::string(buffer.data(), last_bytes);

};

void UDP::clear_buffer() {
    int discard_msg = 1;
    while(discard_msg != 0)
    {
        discard_msg = recvfrom(sockfd, buffer.data(), buffer_size, MSG_DONTWAIT, (struct sockaddr*)&client_addr, &len);
    };
};


void UDP::send(const std::string& message) {
    // No command received yet: we do not know who to talk to. The old code
    // called sendto() BEFORE filling target_addr, so the first telemetry
    // packet (and every packet until a command arrived) went to a garbage
    // address.
    if (!has_peer) {
        return;
    }

    // Reply to the last commander's IP on the telemetry port, THEN send.
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(sender_port);
    target_addr.sin_addr = client_addr.sin_addr; // copy the IP from the sender

    sendto(sockfd, message.c_str(), message.size(), 0,
           (struct sockaddr*)&target_addr, sizeof(target_addr));
}

int UDP::getBufferSize() {
    return buffer_size;
}

int UDP::getRecieverPort() {
    return receiver_port;
}

int UDP::getSenderPort() {
    return sender_port;
}

void UDP::close_socket() {
    close(this->sockfd);
}
