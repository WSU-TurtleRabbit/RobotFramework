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
#include "arduino.h"
#include "Telemetry.h"
#include <thread>
#include <atomic>

std::atomic<bool> ball_detected{false};

void CameraThread(BallDetection &detector)
{
  while (true)
  {
    bool result = detector.find_ball();
    ball_detected.store(result, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

struct Telemetry_msg{
  bool ball_present;
  float voltage;
};


int main(int argc, char **argv)
{
  using namespace mjbots;

  static auto Reciver_interval = std::chrono::milliseconds(20);
  static auto CameraInterval = std::chrono::milliseconds(200);
  static auto MotorInterval = std::chrono::milliseconds(20);
  static auto Sender_interval = std::chrono::milliseconds(1000);

  BallDetection detect;
  UDP UDP;
  Wheel_math m;
  std::string msg;
  cmdDecoder cmd;
  std::vector<double> wheel_velocity;
  std::map<int, double> velocity_map;
  Telemetry_msg sender_msg;

  bool emergency_stop = false; 

  if (detect.open_cam() > 0)
  {
    std::thread camera_thread(CameraThread, std::ref(detect));
  }

  // Use the telemetry class
  Telemetry telemetry;

  // Stop everything to clear faults.
  for (const auto &pair : telemetry.controllers)
  {
    pair.second->SetStop();
  }

  while (!emergency_stop)
  {

    auto current_time = std::chrono::steady_clock::now();
    static auto last_reciver_time = current_time;
    static auto last_motor_time = current_time;
    static auto last_camera_time = current_time;
    static auto last_sender_time = current_time;

    if (current_time - last_reciver_time >= Reciver_interval)
    {
      // UDP.clear_buffer();
      msg = UDP.recive();
      if (msg == "TIMEOUT")
      {
        std::cout << msg << "\n";
        velocity_map = {
            {1, 0.0},
            {2, 0.0},
            {3, 0.0},
            {4, 0.0},
        };
      }
      else
      {
        std::cout << msg << "\n";
        cmd.decode_cmd(msg);
        wheel_velocity = m.calculate(cmd.velocity_x, cmd.velocity_y, cmd.velocity_w);
        velocity_map = {
            {1, wheel_velocity[0]},
            {2, wheel_velocity[1]},
            {3, wheel_velocity[2]},
            {4, wheel_velocity[3]}};
      };
      last_reciver_time = current_time;
    }




    if (current_time - last_camera_time >= CameraInterval)
    {
      bool camera_ball_dected = ball_detected.load(std::memory_order_relaxed);
      std::cout << camera_ball_dected << "\n";
      sender_msg.ball_present = camera_ball_dected;

      last_camera_time = current_time;
    };




    if (current_time - last_motor_time >= MotorInterval)
    {
 
      auto servo_status = telemetry.cycle(velocity_map);

      // Print out telemetry like before.
      float voltage[4];
      int i = 0 ;
      float current_limit = 5;

    

      for (const auto &pair : servo_status)
      {
        const auto &r = pair.second;
        voltage[i] = r.voltage;
        if (current_limit < r.current){
          emergency_stop = true;
        }
        i++;
      }

      float sum = 0;
      float avg_voltage;
      
      for (int i = 0 ; i < 4; i++){
        sum = sum + voltage[i];
      }
      avg_voltage = sum / 4;

      sender_msg.voltage = avg_voltage;
      std::cout << avg_voltage << "\n";
      last_motor_time = current_time;

    };

    if (current_time - last_sender_time >= Sender_interval){
      std::string msg = "Robot State : Active, Battery Voltage:" + std::to_string(sender_msg.voltage) + ", Ball Detection:" + std::to_string(sender_msg.ball_present); 
      UDP.send(msg);
      last_sender_time = current_time;
    }


  };

  
}