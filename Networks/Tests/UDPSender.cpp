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
#include "UDP.h"

int main(){
  Sender s("127.0.0.1", 50514);
  srand(time(0));

  while(true){
    s.clear_buffer();

    std::string message = "1";
    for (int i = 0; i < 6; i++)
    {
        message += " " + std::to_string(rand() % 100 + 1);
    }

    s.send(message);

    std::cout << "Sending: " << message << " to 127.0.0.1:50514" << std::endl;
    msleep();
  };
};