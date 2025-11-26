#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <thread>
#include <chrono>

class Arduino {
private:
    LibSerial::SerialPort serial_port;
    std::string connected_port;
    bool is_connected;
    
    // Helper function to test if a port has an Arduino
    bool testPort(const std::string& port);
    
public:
    Arduino();
    ~Arduino();
    
    // Auto-detect Arduino Nano on USB ports
    bool findArduino();
    
    // Connect to a specific port
    bool connect(const std::string& port);
    
    // Disconnect from Arduino
    void disconnect();
    
    // Send a single character command
    bool sendCommand(char command);
    
    // Check if connected
    bool isConnected() const;
    
    // Get the connected port name
    std::string getPort() const;
};