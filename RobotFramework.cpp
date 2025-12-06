
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
#include "Arduino.h"
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>


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

int interval_reciver, interval_sender, interval_arduino, interval_camera, interval_motor;

try {
    YAML::Node config = YAML::LoadFile("config/Main.yaml");
    YAML::Node interval_values = config["intervals"];

    interval_reciver = interval_values["Reciver_interval"].as<int>();
    interval_sender = interval_values["Sender_interval"].as<int>(); 
    interval_arduino = interval_values["Arduino_interval"].as<int>(); 
    interval_camera = interval_values["Camera_interval"].as<int>(); 
    interval_motor = interval_values["Motor_interval"].as<int>();
} catch (const std::exception &e) {
    std::cerr << "Error loading Interval config: " << e.what() << std::endl;
    // fallback defaults
    interval_reciver = 20;
    interval_sender = 1000;
    interval_arduino = 100;
    interval_camera = 200;
    interval_motor = 20;
}


  static auto Reciver_interval = std::chrono::milliseconds(interval_reciver);
  static auto CameraInterval = std::chrono::milliseconds(interval_camera);
  static auto MotorInterval = std::chrono::milliseconds(interval_motor);
  static auto Sender_interval = std::chrono::milliseconds(interval_sender);
  static auto Arduino_interval = std::chrono::milliseconds(interval_arduino);

  BallDetection detect;
  UDP UDP;
  Wheel_math m;
  std::string msg;
  cmdDecoder cmd;
  std::vector<double> wheel_velocity;
  std::map<int, double> velocity_map;
  Telemetry_msg sender_msg;
  // Use the telemetry class
  Telemetry telemetry;

  Arduino a;
  a.findArduino();
  a.connect();




  bool emergency_stop = false; 

  std::thread camera_thread;
if (detect.open_cam() > 0) {
    camera_thread = std::thread(CameraThread, std::ref(detect));
    camera_thread.detach(); // if you don’t need to join
}

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
    static auto last_arduino_time = current_time;

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
      bool camera_ball_detected = ball_detected.load(std::memory_order_relaxed);
      std::cout << camera_ball_detected << "\n";
      sender_msg.ball_present = camera_ball_detected;

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

    if (current_time - last_arduino_time >= Arduino_interval) {
    if (a.isConnected()) {
        if (cmd.kick) {
            a.sendCommand("K");
        } else if (cmd.dribble) {
            a.sendCommand("D");
        } else {
            a.sendCommand("S");
        }
    }
    last_arduino_time = current_time;
    }


  };

  for (const auto &pair : telemetry.controllers)
  {
    pair.second->SetStop();
  }

  std::cout << "Emergency Stop has been activated" << "\n";
  
}