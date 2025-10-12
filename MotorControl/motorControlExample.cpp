/// This example shows how to control multiple motors using
/// the MotorControl class. This approach can result in lower overall
/// latency and improved performance with some transports, such as the
/// fdcanusb and pi3hat.

#include <unistd.h>
#include "unistd.h"
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <chrono>
#include "MotorControl.h"
#include "thread"

// A simple way to get the current time accurately as a double.
static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

int main(int argc, char** argv) {
  
  static auto MotorInterval = std::chrono::milliseconds(20); 
  
  std::vector<double> wheel_velocity;
  std::map<int, double> velocity_map;
  
  // Create motor control instance
  MotorControl motorControl;

  int count = 1;

  // Main control loop
  while (count <= 200) {
    const auto cycle_start = GetNow();
    
    // Set desired velocities for each motor
    velocity_map[1] = sin(count * 0.1) * 2.0;  // Motor 1: sinusoidal velocity
    velocity_map[2] = cos(count * 0.1) * 2.0;  // Motor 2: cosine velocity  
    velocity_map[3] = sin(count * 0.05) * 1.5; // Motor 3: slower sine
    velocity_map[4] = cos(count * 0.05) * 1.5; // Motor 4: slower cosine

    // Execute motor commands and get status
    auto motor_status = motorControl.cycle(velocity_map);

    // Print status information
    std::cout << "Cycle " << count << " - Motor Status:" << std::endl;
    for (const auto& [motor_id, status] : motor_status) {
      std::cout << "  Motor " << motor_id 
                << ": vel=" << status.velocity 
                << ", temp=" << status.temperature 
                << ", voltage=" << status.voltage 
                << ", current=" << status.current << std::endl;
    }

    // Sleep to maintain control frequency
    const auto cycle_end = GetNow();
    const auto elapsed = cycle_end - cycle_start;
    const auto desired_period = std::chrono::duration<double>(MotorInterval).count();
    
    if (elapsed < desired_period) {
      std::this_thread::sleep_for(
        std::chrono::duration<double>(desired_period - elapsed));
    }

    count++;
  }

  // Stop all motors before exiting
  motorControl.stopAll();
  std::cout << "All motors stopped. Example completed." << std::endl;

  return 0;
}
