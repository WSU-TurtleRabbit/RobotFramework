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

  pi3hat::Pi3HatMoteusTransport::Options toptions; // Mapping out the servo maps 
  toptions.servo_map = {
    {1, 1},  // bus 1, servo ID 1
    {1, 4},  // bus 1, servo ID 4
    {2, 2},  // bus 2, servo ID 2
    {2, 3},  // bus 2, servo ID 3
  };

    moteus::Controller::Options opt1;
    opt1.id = 1;
    opt1.bus = 1;
    opt1.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
    controllers[1] = std::make_shared<moteus::Controller>(opt1);

    moteus::Controller::Options opt2;
    opt2.id = 2;
    opt2.bus = 2;
    opt2.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
    controllers[2] = std::make_shared<moteus::Controller>(opt2);

    moteus::Controller::Options opt3;
    opt3.id = 3;
    opt3.bus = 2;
    opt3.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
    controllers[3] = std::make_shared<moteus::Controller>(opt3);

    moteus::Controller::Options opt4;
    opt4.id = 4;
    opt4.bus = 1;
    opt4.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
    controllers[4] = std::make_shared<moteus::Controller>(opt4);

  // Stop everything to clear faults.

  ::printf("Stopping Motor Now......\n");
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }
  return 0;
}