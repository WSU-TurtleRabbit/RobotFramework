#include <iostream>
#include <chrono>
#include <thread>
#include "../Arduino/arduino.h"

int main() {
    std::cout << "Starting Arduino Nano detection and LED blink test...\n" << std::endl;
    
    Arduino ard;
    
    // Try to find Arduino Nano
    if (!ard.findArduino()) {
        std::cerr << "\nFailed to find Arduino Nano on any port" << std::endl;
        return 1;
    }
    
    std::cout << "\nArduino connected on port: " << ard.getPort() << std::endl;
    std::cout << "Is connected: " << (ard.isConnected() ? "YES" : "NO") << "\n" << std::endl;
    
    // Send LED blink commands
    std::cout << "Sending LED blink commands to Arduino...\n" << std::endl;
    
    // Send 'B' command to make LED blink (10 times)
    for (int i = 0; i < 10; i++) {
        if (ard.sendCommand('B')) {
            std::cout << "Blink command " << (i + 1) << " sent - Success" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            std::cout << "Blink command " << (i + 1) << " sent - Failed" << std::endl;
            break;
        }
    }
    
    std::cout << "\nTest complete. Disconnecting..." << std::endl;
    ard.disconnect();
    
    return 0;
}
