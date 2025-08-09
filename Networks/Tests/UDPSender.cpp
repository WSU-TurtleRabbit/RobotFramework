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

  while(true){
    s.clear_buffer();
    s.send("1 2 3 4 5 6 7");
  };
};