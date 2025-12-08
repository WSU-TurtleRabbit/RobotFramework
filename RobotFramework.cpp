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
#include <chrono>
#include "moteus.h"
#include "pi3hat_moteus_transport.h"
#include "wheel_math.h"
#include "decode.h"
#include "UDP.h"
#include "detect_ball.h"
#include "MotorControl.h"
#include "arduino.h"
#include <thread>
#include <atomic>

std::atomic<bool> ball_detected{false};

// void CameraThread(BallDetection &detector)
// {
//   while (true)
//   {
//     bool result = detector.find_ball();
//     ball_detected.store(result, std::memory_order_relaxed);
//     std::this_thread::sleep_for(std::chrono::milliseconds(300));
//   }
// }

// A simple way to get the current time accurately as a double.
static double GetNow()
{
  struct timespec ts = {};
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) / 1e9;
}

int main(int argc, char **argv)
{

  using namespace mjbots;

  static auto UDP_interval = std::chrono::milliseconds(20);
  static auto CameraInterval = std::chrono::milliseconds(100);
  static auto MotorInterval = std::chrono::milliseconds(20);

  // Initialize and auto-connect to Arduino
  Arduino arduino;
  if (!arduino.findArduino()) {
    std::cerr << "Warning: Could not find Arduino. Continuing without Arduino support." << std::endl;
  }

  BallDetection detect;
  Reciver r;
  Wheel_math m;
  std::string msg;
  cmdDecoder cmd;
  std::vector<double> wheel_velocity;
  std::map<int, double> velocity_map;

  // if (detect.open_cam() > 0)
  // {
  //   std::thread camera_thread(CameraThread, std::ref(detect));
  // }

  // // This shows how you could construct a runtime number of controller
  // // instances.
  // std::map<int, std::shared_ptr<moteus::Controller>> controllers;
  // std::map<int, moteus::Query::Result> servo_data;

  // pi3hat::Pi3HatMoteusTransport::Options toptions; // Mapping out the servo maps
  // std::map <int, int> servo_map = {
  // //Made a map of motor controller and
  // //and matching bus pair

  //   {1, 1},  // Motor ID 1 mapped to BUS 1
  //   {4, 1},  // Motor ID 4 mapped to BUS 1
  //   {2, 2},  // Motor ID 2 mapped to BUS 2
  //   {3, 2},  // Motor ID 3 mapped to BUS 2
  // };
  // toptions.servo_map = servo_map;
  // int count = 1;

  // //Simple for loop to go though the map and create the matching ID and BUS pair
  //   for(auto& pairs : servo_map){
  //     moteus::Controller::Options opts;
  //     opts.id = pairs.first;
  //     opts.bus = pairs.second;
  //     opts.transport = std::make_shared<pi3hat::Pi3HatMoteusTransport>(toptions);
  //     controllers[count] = std::make_shared<moteus::Controller>(opts);
  //     count++;
  //   }

  // Use the motor control class
  MotorControl motorControl;

  // Stop everything to clear faults.
  motorControl.stopAll();

  while (true)
  {
    if (arduino.isConnected()) 
    {
      arduino.sendCommand('K');
    }

    auto current_time = std::chrono::steady_clock::now();
    static auto last_UDP_time = current_time;
    static auto last_motor_time = current_time;
    static auto last_camera_time = current_time;

    // if (current_time - last_UDP_time >= UDP_interval)
    // {
    //   r.clear_buffer();
    //   msg = r.recive();
    //   if (msg == "TIMEOUT")
    //   {
    //     std::cout << msg << "\n";
    //     velocity_map = {
    //         {1, 0.0},
    //         {2, 0.0},
    //         {3, 0.0},
    //         {4, 0.0},
    //     };
    //   }
    //   else
    //   {
    //     std::cout << msg << "\n";
    //     cmd.decode_cmd(msg);
    //     wheel_velocity = m.calculate(cmd.velocity_x, cmd.velocity_y, cmd.velocity_w);
    //     velocity_map = {
    //         {1, wheel_velocity[0]},
    //         {2, wheel_velocity[1]},
    //         {3, wheel_velocity[2]},
    //         {4, wheel_velocity[3]}};
    //   };
    //   last_UDP_time = current_time;
    // }

    // if (current_time - last_camera_time >= CameraInterval)
    // {
    //   bool camera_ball_dected = ball_detected.load(std::memory_order_relaxed);
    //   std::cout << camera_ball_dected << "\n";
    //   last_camera_time = current_time;
    // };

    // if (current_time - last_motor_time >= MotorInterval){

    //   const auto now = GetNow();
    //   std::vector<moteus::CanFdFrame> command_frames;

    //     // std::cout << "Doing" << std::endl;

    //     // Accumulate all of our command CAN frames.
    //   for (const auto& pair : controllers) {
    //       moteus::PositionMode::Command position_command;
    //         position_command.position = NaN;
    //         position_command.velocity = velocity_map[pair.first];

    //     command_frames.push_back(pair.second->MakePosition(position_command));
    //       };

    //     // Now send them in a single call to Transport::Cycle.
    //     std::vector<moteus::CanFdFrame> replies;
    //     const auto start = GetNow();
    //     transport->BlockingCycle(&command_frames[0], command_frames.size(), &replies);
    //     const auto end = GetNow();
    //     const auto cycle_time = end - start;

    //     // Finally, print out our current query results.

    //     char buf[4096] = {};
    //     std::string status_line;

    //     ::snprintf(buf, sizeof(buf) - 1, "%10.2f dt=%7.4f) ", now, cycle_time);

    //     status_line += buf;

    //     // We parse these into a map to both sort and de-duplicate them,
    //     // and persist data in the event that any are missing.
    //     for (const auto& frame : replies) {
    //       servo_data[frame.source] = moteus::Query::Parse(frame.data, frame.size);
    //     }

    //     for (const auto& pair : servo_data) {
    //       const auto r = pair.second;
    //       ::snprintf(buf, sizeof(buf) - 1,
    //                 "%2d %3d temp/velocity/volatge=(%7.3f,%7.3f,%7.3f)  ",
    //                 pair.first,
    //                 static_cast<int>(r.mode),
    //                 r.temperature,
    //                 r.velocity,
    //                 r.voltage);
    //       status_line += buf;
    //     }
    //     ::printf("%s  \r", status_line.c_str());
    //     ::fflush(::stdout);

    //     // Sleep 20ms between iterations.  By default, when commanded over
    //     // CAN, there is a watchdog which requires commands to be sent at
    //     // least every 100ms or the controller will enter a latched fault
    //     // state.

    //     last_motor_time = current_time;
    //   };

    // };
    if (current_time - last_motor_time >= MotorInterval)
    {

      const auto now = GetNow();
      // Use MotorControl to send position/velocity commands and collect motor status in one synchronous cycle.
      const auto start = GetNow();
      velocity_map = {
            {1, 0.0},
            {2, 0.0},
            {3, 0.0},
            {4, 0.0},
        };
      auto motor_status = motorControl.cycle(velocity_map);
      const auto end = GetNow();
      const auto cycle_time = end - start;

      // Print out motor status like before.
      char buf[4096] = {};
      std::string status_line;
      ::snprintf(buf, sizeof(buf) - 1, "%10.2f dt=%7.4f) ", now, cycle_time);
      status_line += buf;

      for (const auto &pair : motor_status)
      {
        const auto &r = pair.second;
        ::snprintf(buf, sizeof(buf) - 1,
                   "%2d %3d temp/velocity/voltage=(%7.3f,%7.3f,%7.3f)  ",
                   pair.first,
                   r.mode,
                   r.temperature,
                   r.velocity,
                   r.voltage);
        status_line += buf;
      }
      ::printf("%s  \r", status_line.c_str());
      ::fflush(::stdout);

      last_motor_time = current_time;
    };
  };
}