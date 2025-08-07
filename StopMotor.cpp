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
/// this code is to properly shut down all the motors 

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

  pi3hat::Pi3HatMoteusTransport::Options toptions; // Mapping out the servo maps 
  std::map <int, int> servo_map = { 
  //Made a map of motor controller and
  //and matching bus pair 

    {1, 1},  // Motor ID 1 mapped to BUS 1
    {2, 2},  // Motor ID 4 mapped to BUS 1 
    {3, 3},  // Motor ID 2 mapped to BUS 2
    {4, 4},  // Motor ID 3 mapped to BUS 2 
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
      
  ::printf("Stopping Motor Now......\n");
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }
  return 0;
}