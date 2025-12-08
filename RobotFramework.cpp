#include <unistd.h>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include "moteus.h"
#include "pi3hat_moteus_transport.h"
#include "wheel_math.h"
#include "decode.h"
#include "UDP.h"
#include "detect_ball.h"
#include "arduino.h"
#include "Telemetry.h"
#include "arduino.h"
#include <yaml-cpp/yaml.h>

// --- Atomic flags for inter-thread communication ---
std::atomic<bool> ball_detected{false};       // Stores ball detection result from camera thread
std::atomic<bool> stop_camera_thread{false}; // Signals the camera thread to stop
std::atomic<bool> manual_stop_flag{false};   // Signals main loop to stop on Ctrl+C

// --- Forward declaration for signal handler ---
void signalHandler(int signum);

// --- Thread function for camera detection ---
void CameraThread(BallDetection &detector)
{
    while (!stop_camera_thread.load(std::memory_order_relaxed))
    {
        bool result = detector.find_ball(); // Check if ball is present
        ball_detected.store(result, std::memory_order_relaxed); // Store result atomically
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Reduce CPU usage
    }
}

// --- Struct for telemetry message to send ---
struct Telemetry_msg{
    bool ball_present; // True if ball detected
    float voltage;     // Average motor voltage
};

int main(int argc, char **argv)
{
    using namespace mjbots;
    
    char kick = 'k';
    char dribble = 'd';
    char stop_dribble = 's';
    
    double current_limit;
    // double temperture_limit;

    // --- Interval times (ms) for periodic tasks ---
    int interval_reciver, interval_sender, interval_arduino, interval_camera, interval_motor;

    // --- Load YAML config ---
    try {
        YAML::Node config = YAML::LoadFile("config/Main.yaml"); //Main control Config file 
        YAML::Node s_config = YAML::LoadFile("config/Safety.yaml"); // Safety Config file 
        YAML::Node interval_values = config["intervals"];

        current_limit = s_config["currentLimit"].as<double>();

        interval_reciver = interval_values["Reciver_interval"].as<int>();
        interval_sender = interval_values["Sender_interval"].as<int>(); 
        interval_arduino = interval_values["Arduino_interval"].as<int>(); 
        interval_camera = interval_values["Camera_interval"].as<int>(); 
        interval_motor = interval_values["Motor_interval"].as<int>();

        
    } catch (const std::exception &e) {
        std::cerr << "Error loading Interval config: " << e.what() << std::endl;
        // Fallback defaults
        interval_reciver = 20;
        interval_sender = 1000;
        interval_arduino = 100;
        interval_camera = 200;
        interval_motor = 20;

        current_limit = 5.0;
    }

    // --- Convert intervals to chrono durations ---
    static auto Reciver_interval = std::chrono::milliseconds(interval_reciver);
    static auto CameraInterval = std::chrono::milliseconds(interval_camera);
    static auto MotorInterval = std::chrono::milliseconds(interval_motor);
    static auto Sender_interval = std::chrono::milliseconds(interval_sender);
    static auto Arduino_interval = std::chrono::milliseconds(interval_arduino);

    // --- Initialize modules ---
    BallDetection detect;      // Camera detection
    UDP UDP;                   // UDP communication
    Wheel_math m;              // Wheel velocity calculations
    cmdDecoder cmd;            // Decode incoming commands
    Telemetry telemetry;       // Motor telemetry
    Arduino a;                 // Arduino controller

    std::string msg;                   // Incoming UDP message
    std::vector<double> wheel_velocity; // Calculated wheel velocities
    std::map<int, double> velocity_map; // Motor ID → velocity map
    Telemetry_msg sender_msg;          // Telemetry message to send

    // --- Initialize Arduino ---
    a.findArduino();
    a.connect(a.getPort());

    bool emergency_stop = false; // Flag to stop robot on emergency

    // --- Start camera detection thread ---
    std::thread camera_thread;
    if (detect.open_cam() > 0) {
        // I didnt detach this beacsue it wanted to close it later on. 
        camera_thread = std::thread(CameraThread, std::ref(detect));
    }

    // --- Setup signal handler to catch Ctrl+C ---
    std::signal(SIGINT, signalHandler);

    // --- Stop all motors initially to clear faults ---
    for (const auto &pair : telemetry.controllers) {
        pair.second->SetStop();
    }

    // --- Main control loop ---
 
    while (!emergency_stop && !manual_stop_flag.load(std::memory_order_relaxed))
    {
        auto current_time = std::chrono::steady_clock::now();

        // Static timers for periodic tasks
        static auto last_reciver_time = current_time;
        static auto last_motor_time = current_time;
        static auto last_camera_time = current_time;
        static auto last_sender_time = current_time;
        static auto last_arduino_time = current_time;

        // --- UDP Receiver ---
        if (current_time - last_reciver_time >= Reciver_interval)
        {
            msg = UDP.recive(); // Receive new message
            if (msg == "TIMEOUT")
            {
                std::cout << msg << "\n";
                velocity_map = {{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}}; // Stop wheels
            }
            else
            {
                std::cout << msg << "\n";
                cmd.decode_cmd(msg); // Decode velocity commands
                wheel_velocity = m.calculate(cmd.velocity_x, cmd.velocity_y, cmd.velocity_w);
                // Map velocities to motors
                velocity_map = {
                    {1, wheel_velocity[0]},
                    {2, wheel_velocity[1]},
                    {3, wheel_velocity[2]},
                    {4, wheel_velocity[3]}
                };
            }
            last_reciver_time = current_time;
        }

        // --- Camera Ball Detection ---
        if (current_time - last_camera_time >= CameraInterval)
        {
            bool camera_ball_detected = ball_detected.load(std::memory_order_relaxed);
            std::cout << camera_ball_detected << "\n";
            sender_msg.ball_present = camera_ball_detected;
            last_camera_time = current_time;
        }

        // --- Motor Telemetry and Safety Check ---
        if (current_time - last_motor_time >= MotorInterval)
        {
            auto servo_status = telemetry.cycle(velocity_map); // Send commands & receive telemetry

            float voltage[4];
            int i = 0;

            for (const auto &pair : servo_status)
            {
                const auto &r = pair.second;
                voltage[i] = r.voltage;
                if (r.current > current_limit) {
                    emergency_stop = true; // Stop on overcurrent
                }
                i++;
            }

            // Compute average voltage
            float sum = 0;
            for (int i = 0; i < 4; i++) sum += voltage[i];
            sender_msg.voltage = sum / 4;

            std::cout << sender_msg.voltage << "\n";

            last_motor_time = current_time;
        }

        // --- UDP Telemetry Sender ---
        if (current_time - last_sender_time >= Sender_interval) {
            std::string msg = "Robot State: Active, Battery Voltage:" + std::to_string(sender_msg.voltage) +
                              ", Ball Detection:" + std::to_string(sender_msg.ball_present); 
            UDP.send(msg);
            last_sender_time = current_time;
        }

        // --- Arduino Commands ---
        if (current_time - last_arduino_time >= Arduino_interval) {
            if (a.isConnected()) {
                if (cmd.kick) {
                    a.sendCommand(kick); // Kick
                } else if (cmd.dribble) {
                    a.sendCommand(dribble); // Dribble
                } else {
                    a.sendCommand(stop_dribble); // Stop
                }
            }
            last_arduino_time = current_time;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Reduce CPU load
    }

    // --- Emergency Stop ---
    for (const auto &pair : telemetry.controllers) {
        pair.second->SetStop();
    }
    a.disconnect();

    // Stop camera thread and join
    stop_camera_thread.store(true, std::memory_order_relaxed);
    if (camera_thread.joinable()) {
        camera_thread.join();
    }

    std::cout << "Emergency Stop has been activated\n";
}

// --- Signal handler for Ctrl+C ---
void signalHandler(int signum) {
    std::cout << "\nSIGINT received. Stopping safely...\n";
    manual_stop_flag.store(true);
}
