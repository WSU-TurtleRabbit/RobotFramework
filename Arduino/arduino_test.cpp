#include "arduino.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Create Arduino instance
    Arduino arduino;
    
    // Auto-detect and connect to Arduino
    std::cout << "Searching for Arduino..." << std::endl;
    if (!arduino.findArduino()) {
        std::cerr << "Failed to find Arduino. Exiting." << std::endl;
        return 1;
    }
    
    // Example: Send some commands
    std::cout << "\nSending test commands..." << std::endl;
    
    // Send 'K' command (Kick)
    std::cout << "Sending 'K' (Kick)" << std::endl;
    arduino.sendCommand('K');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Send 'D' command (Dribble)
    std::cout << "Sending 'D' (Dribble)" << std::endl;
    arduino.sendCommand('D');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Send 'S' command (Stop)
    std::cout << "Sending 'S' (Stop)" << std::endl;
    arduino.sendCommand('S');
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Interactive mode
    std::cout << "\nEntering interactive mode..." << std::endl;
    std::cout << "Commands: K=Kick, D=Dribble, S=Stop, Q=Quit" << std::endl;
    
    char command;
    while (true) {
        std::cout << "\nEnter command: ";
        std::cin >> command;
        
        if (command == 'Q' || command == 'q') {
            std::cout << "Quitting..." << std::endl;
            break;
        }
        
        if (arduino.sendCommand(command)) {
            std::cout << "Sent: " << command << std::endl;
        } else {
            std::cerr << "Failed to send command" << std::endl;
        }
    }
    
    return 0;
}
