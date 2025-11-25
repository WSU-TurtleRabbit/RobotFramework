#include "Arduino.h"
#include <filesystem>
#include <fcntl.h>
#include <unistd.h>

Arduino::Arduino() : is_connected(false), connected_port("") {}

Arduino::~Arduino() {
  disconnect();
}

bool Arduino::testPort(const std::string& port) {
  try {
    // Try to open the port
    LibSerial::SerialPort test_port;
    test_port.Open(port);
    
    // Configure serial settings for Arduino
    test_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    test_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    test_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    test_port.SetParity(LibSerial::Parity::PARITY_NONE);
    test_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    
    // Wait a moment for Arduino to reset (it resets on serial connection)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Flush any initial data
    test_port.FlushIOBuffers();
    
    // Send a ping command and wait for response (optional)    
    test_port.Close();
    return true;
        
  } catch (const std::exception& e) {
    return false;
  }
}

bool Arduino::findArduino() {
  // List of common USB serial device patterns on Linux
  std::vector<std::string> port_patterns = {
    "/dev/ttyUSB",  // USB to serial adapters
    "/dev/ttyACM"   // Arduino Nano, Uno, etc.
  };
    
  // Try each pattern with indices 0-9
  for (const auto& pattern : port_patterns) {
    for (int i = 0; i < 10; i++) {
      std::string port = pattern + std::to_string(i);
            
      std::cout << "Checking port: " << port << std::endl;
            
      // Check if port exists
      if (access(port.c_str(), F_OK) != 0) {
        continue;  // Port doesn't exist, skip
      }
            
      // Test if we can connect to this port
      if (testPort(port)) {
        std::cout << "Arduino found on: " << port << std::endl;
        return connect(port);
      }
    }
  }

  std::cerr << "No Arduino found on any USB port" << std::endl;
  return false;
}

bool Arduino::connect(const std::string& port) {
  try {
    if (is_connected) {
      disconnect();
    }
        
    serial_port.Open(port);
        
    // Configure serial settings
    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
    // Wait for Arduino to reset
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
    // Flush any initial data
    serial_port.FlushIOBuffers();
        
    connected_port = port;
    is_connected = true;
        
    std::cout << "Successfully connected to Arduino on " << port << std::endl;
    return true;
        
  } catch (const std::exception& e) {
    std::cerr << "Failed to connect to " << port << ": " << e.what() << std::endl;
    is_connected = false;
    return false;
  }
}

void Arduino::disconnect() {
  if (is_connected && serial_port.IsOpen()) {
    try {
      serial_port.Close();
        std::cout << "Disconnected from " << connected_port << std::endl;
      } 
      catch (const std::exception& e) {
      std::cerr << "Error disconnecting: " << e.what() << std::endl;
      }
    }
  is_connected = false;
  connected_port = "";
}

bool Arduino::sendCommand(char command) {
  if (!is_connected || !serial_port.IsOpen()) {
    std::cerr << "Cannot send command: Arduino not connected" << std::endl;
    return false;
  }
    
  try {
    serial_port.WriteByte(command);
    serial_port.DrainWriteBuffer();  // Ensure data is sent
    return true;
        
  }
  catch (const std::exception& e) {
    std::cerr << "Failed to send command: " << e.what() << std::endl;
    return false;
  }
}

bool Arduino::isConnected() const {
  return is_connected;
}

std::string Arduino::getPort() const {
  return connected_port;
}