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
int main(int argc, char** argv) {
  using namespace mjbots;

  // This shows how you could construct a runtime number of controller
  // instances.
  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  std::map<int, moteus::Query::Result> servo_data;

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
      
  ::printf("Stopping Motor Now......\n");
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }
  return 0;
}