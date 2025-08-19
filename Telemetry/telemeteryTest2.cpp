
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
  

  // This shows how you could construct a runtime number of controller
  // instances.
  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  std::map<int, moteus::Query::Result> servo_data;

  pi3hat::Pi3HatMoteusTransport::Options toptions; // Mapping out the servo maps 
  std::map <int, int> servo_map = { 
  //Made a map of motor controller and
  //and matching bus pair 

    {1, 1},  // Motor ID 1 mapped to BUS 1
    {4, 1},  // Motor ID 4 mapped to BUS 1 
    {2, 2},  // Motor ID 2 mapped to BUS 2
    {3, 2},  // Motor ID 3 mapped to BUS 2 
  };
  toptions.servo_map = servo_map; 
  int count = 1;

  //Simple for loop to go though the map and create the matching ID and BUS pair
    for(auto& pairs : servo_map){
      moteus::Controller::Options opts;
      opts.id = pairs.first;
      opts.bus = pairs.second;
      opts.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
      controllers[count] = std::make_shared<moteus::Controller>(opts);
      count++;
    }

  // Stop everything to clear faults.
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }

  // Getting the Pi3hat Transpot insatnces, to use for the Cycle
  auto transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);


  
  while(true){

  auto current_time = std::chrono::steady_clock::now();

  static auto last_motor_time = current_time;
// set the velocity as 0 to send a command 
   velocity_map = {
    {1, 0.0}, 
    {2, 0.0}, 
    {3, 0.0}, 
    {4, 0.0}, 
    };

  if (current_time - last_motor_time >= MotorInterval){

    const auto now = GetNow();
    std::vector<moteus::CanFdFrame> command_frames;
    
      // Accumulate all of our command CAN frames.
    for (const auto& pair : controllers) {
        moteus::PositionMode::Command position_command;
          position_command.position = NaN;
          position_command.velocity = velocity_map[pair.first];
      command_frames.push_back(pair.second->MakePosition(position_command));
        };

      // Now send them in a single call to Transport::Cycle.
      std::vector<moteus::CanFdFrame> replies;
      const auto start = GetNow();
      transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
      const auto end = GetNow();
      const auto cycle_time = end - start;

      // Finally, print out our current query results.

      char buf[4096] = {};
      std::string status_line;

      ::snprintf(buf, sizeof(buf) - 1, "%10.2f dt=%7.4f) ", now, cycle_time);

      status_line += buf;

      // We parse these into a map to both sort and de-duplicate them,
      // and persist data in the event that any are missing.
      for (const auto& frame : replies) {
        servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
      }

      for (const auto& pair : servo_data) {
        const auto r = pair.second;
        ::snprintf(buf, sizeof(buf) - 1,
                  "%2d %3d temp/velocity/volatge=(%7.3f,%7.3f,%7.3f)  ",
                  pair.first,
                  static_cast<int>(r.mode),
                  r.temperature,
                  r.velocity,
                  r.voltage);
        status_line += buf;
      }
      ::printf("%s  \r", status_line.c_str());
      ::fflush(::stdout);

      // Sleep 20ms between iterations.  By default, when commanded over
      // CAN, there is a watchdog which requires commands to be sent at
      // least every 100ms or the controller will enter a latched fault
      // state.

      last_motor_time = current_time;
    };

  };
  
};