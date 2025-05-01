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

  int servo_count = 4;

  // This shows how you could construct a runtime number of controller
  // instances.
  std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  std::map<int, moteus::Query::Result> servo_data;

  pi3hat::Pi3HatMoteusTransport::Options toptions; // Mapping out the servo maps 
  toptions.servo_map[1] = 1;
  toptions.servo_map[2] = 2;
  toptions.servo_map[3] = 3;
  toptions.servo_map[4] = 4;


  for (int i = 1; i <= servo_count; i++) {
    moteus::Controller::Options options;
    options.id = i;
    options.bus = i;
    options.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);

    // If the intended transport supported multiple busses, you would
    // configure them here as well.
    //
    // options.bus = foo;

    controllers[i] = std::make_shared<moteus::Controller>(options);
  }

  // Stop everything to clear faults.

  ::printf("Stopping Motor Now......");
  for (const auto& pair : controllers) {
    pair.second->SetStop();
  }
  return 0;
}