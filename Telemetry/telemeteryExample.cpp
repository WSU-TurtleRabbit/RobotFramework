/// This example shows how multiple controllers can be commanded using
/// the Cycle method.  This approach can result in lower overall
/// latency and improved performance with some transports, such as the
/// fdcanusb and pi3hat.

#include <unistd.h>
#include "unistd.h"
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <chrono>
#include "moteus.h"
#include "pi3hat_moteus_transport.h"
#include "thread"

// A simple way to get the current time accurately as a double.
static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

int main(int argc, char** argv) {

  using namespace mjbots;

  static auto MotorInterval = std::chrono::milliseconds(20); 
  
  std::vector <double> wheel_velocity;
  std::map<int, double> velocity_map;
  

  // Storage for controllers and their query results (telemetry)
  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  std::map<int, moteus::Query::Result> servo_data;

  // Transport configuration for Pi3Hat
  pi3hat::Pi3HatMoteusTransport::Options toptions; 
  std::map <int, int> servo_map = { 
    // Motor ID → Bus pair mapping
    {1, 1},  // Motor ID 1 mapped to BUS 1
    {4, 1},  // Motor ID 4 mapped to BUS 1 
    {2, 2},  // Motor ID 2 mapped to BUS 2
    {3, 2},  // Motor ID 3 mapped to BUS 2 
  };
  toptions.servo_map = servo_map; 
  
  int count = 1;

  // Create controllers for each motor ID / bus pair
  for(auto& pairs : servo_map){
      moteus::Controller::Options opts;
      opts.id = pairs.first;
      opts.bus = pairs.second;
      opts.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
      controllers[count] = std::make_shared<moteus::Controller>(opts);
      count++;
  }

  // Issue a stop command to all controllers (clear faults before starting)
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }

  // A shared transport instance used for the Cycle method
  auto transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);

  
  while(true){

    auto current_time = std::chrono::steady_clock::now();
    static auto last_motor_time = current_time;

    // Set default velocity commands to 0 for all motors
    velocity_map = {
      {1, 0.0}, 
      {2, 0.0}, 
      {3, 0.0}, 
      {4, 0.0}, 
    };

    // Run loop at ~20ms intervals
    if (current_time - last_motor_time >= MotorInterval){

      const auto now = GetNow();
      std::vector<moteus::CanFdFrame> command_frames;
    
      // Build position/velocity commands for all controllers
      for (const auto& pair : controllers) {
          moteus::PositionMode::Command position_command;
          position_command.position = NaN;  // Don't command absolute position
          position_command.velocity = velocity_map[pair.first]; // Command velocity
          
          // Convert command into CAN frame
          command_frames.push_back(pair.second->MakePosition(position_command));
      };

      // Send all commands in a single transport cycle
      std::vector<moteus::CanFdFrame> replies;
      const auto start = GetNow();
      transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
      const auto end = GetNow();
      const auto cycle_time = end - start;

      // -------- TELEMETRY EXTRACTION --------
      // Each reply from the Moteus controllers contains telemetry data.
      // Parse the reply frames into structured Query::Result objects.
      for (const auto& frame : replies) {
        servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
      }

      // Now we can access telemetry fields from the parsed results.
      for (const auto& pair : servo_data) {
        const auto r = pair.second;

        // r.voltage   → Bus voltage at the motor
        // r.current   → Motor current
        // r.temperature → Motor temperature
        std::cout << "Motor " << pair.first << " -> "
                  << "Voltage: " << r.voltage << " V, "
                  << "Current: " << r.current << " A, "
                  << "Temperature: " << r.temperature << " °C\n";
      }

      last_motor_time = current_time;
    };
  };
};
