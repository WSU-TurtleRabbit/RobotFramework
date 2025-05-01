// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// This example shows how multiple controllers can be commanded using
/// the Cycle method.  This approach can result in lower overall
/// latency and improved performance with some transports, such as the
/// fdcanusb and pi3hat.

#include <unistd.h>

#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

// A simple way to get the current time accurately as a double.
static double GetNow() {
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
      static_cast<double>(ts.tv_nsec) / 1e9;
}

int main(int argc, char** argv) {
  using namespace mjbots;

  // This shows how you could construct a runtime number of controller
  // instances.
  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  std::map<int, moteus::Query::Result> servo_data;

  pi3hat::Pi3HatMoteusTransport::Options toptions; // Mapping out the servo maps 
  std::vector<std::vector<int>> servo_map = { 
  //Made a map of motor controller and
  //and matching bus pair 

    {1, 1},  // bus 1, servo ID 1
    {1, 4},  // bus 2, servo ID 2
    {2, 2},  // bus 3, servo ID 3
    {2, 3},  // bus 4, servo ID 4
  };
  toptions.servo_map = servo_map; 

  //Simple for loop to go though the map and create the matching ID and BUS pair
    for(int i= 1, i < servo_map.size()+1, i++){
      std::vector<int> controller_pairs = servo_map[i-1];
      moteus::Controller::Options opts;
      opts.id = controller_pairs[0];
      opts.bus = controller_pairs[1];
      opts.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
      controllers[i] = std::make_shared<moteus::Controller>(opts);
    }

  // Stop everything to clear faults.
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }

  // Getting the Pi3hat Transpot insatnces, to use for the Cycle
  auto transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);

  while (true) {
    const auto now = GetNow();
    std::vector<moteus::CanFdFrame> command_frames;
   
    // Accumulate all of our command CAN frames.
    for (const auto& pair : controllers) {
      moteus::PositionMode::Command position_command;
      position_command.position = NaN;
      position_command.velocity = 3;
      command_frames.push_back(pair.second->MakePosition(position_command));
    }

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
                 "%2d %3d p/v/t=(%7.3f,%7.3f,%7.3f)  ",
                 pair.first,
                 static_cast<int>(r.mode),
                 r.position,
                 r.velocity,
                 r.torque);
      status_line += buf;
    }
    ::printf("%s  \r", status_line.c_str());
    ::fflush(::stdout);

    // Sleep 20ms between iterations.  By default, when commanded over
    // CAN, there is a watchdog which requires commands to be sent at
    // least every 100ms or the controller will enter a latched fault
    // state.
    ::usleep(20000);
  }
}