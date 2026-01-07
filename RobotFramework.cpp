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
#include "Logger/Logger.h"

// --- Atomic flags for inter-thread communication ---
std::atomic<bool> ball_detected{false};      // Stores ball detection result from camera thread
std::atomic<bool> stop_camera_thread{false}; // Signals the camera thread to stop
std::atomic<bool> manual_stop_flag{false};   // Signals main loop to stop on Ctrl+C

// --- Forward declaration for signal handler ---
void signalHandler(int signum);

// --- Thread function for camera detection ---
void CameraThread(BallDetection &detector)
{
    while (!stop_camera_thread.load(std::memory_order_relaxed))
    {
        bool result = detector.find_ball();                          // Check if ball is present
        ball_detected.store(result, std::memory_order_relaxed);      // Store result atomically
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Reduce CPU usage
    }
}

// --- Struct for telemetry message to send ---
struct Telemetry_msg
{
    bool ball_present; // True if ball detected
    float voltage;     // Average motor voltage
};

enum class State
{
    STARTUP,
    IDLE,
    RUNNING,
    FAULT,
    STOPPING
};

// --- Forward declaration of states ---
void startup(int argc, char **argv);
void idle();
void running();
void fault();
void stopping();

// --- Initialize Modules ---
Logger logger("logs"); // Logger
BallDetection detect;  // Camera detection
UDP UDP;               // UDP communication
Wheel_math m;          // Wheel velocity calculations
cmdDecoder cmd;        // Decode incoming commands
Telemetry telemetry;   // Motor telemetry
Arduino a;             // Arduino controller

std::string msg;                    // Incoming UDP message
std::vector<double> wheel_velocity; // Calculated wheel velocities
std::map<int, double> velocity_map; // Motor ID → velocity map
Telemetry_msg sender_msg;           // Telemetry message to send

// --- Faulty motors tracking ---
std::set<int> faulty_motors;                                              // Set of motor IDs considered faulty
std::map<int, std::chrono::steady_clock::time_point> last_motor_response; // Last response time for each motor
std::chrono::milliseconds FaultGrace{1500};                               // Time in milliseconds a part can go without responding before being considered faulty

// --- Initialize Dribbler Variables ---
char kick = 'k';         // Kick command
char dribble = 'd';      // Dribble command
char stop_dribble = 's'; // Stop Dribble command

// --- Cam Detection Thread ---
std::thread camera_thread;

// --- Interval times (ms) for periodic tasks and YAML content ---

int interval_reciver, interval_sender, interval_arduino, interval_camera, interval_motor, time_until_idle, fault_grace;
double current_limit;
std::map<std::string, double> configData;

// --- Interval durations for periodic tasks ---
std::chrono::milliseconds Reciver_interval{20};
std::chrono::milliseconds CameraInterval{200};
std::chrono::milliseconds MotorInterval{20};
std::chrono::milliseconds Sender_interval{1000};
std::chrono::milliseconds Arduino_interval{100};

// --- Robot mode and state ---
State state = State::STARTUP;
bool emergency_stop = false;
int mode = 0;

// --- Current time and time from last command and how long until IDLE mode ---
auto current_time = std::chrono::steady_clock::now();
auto last_command_time = current_time;
std::chrono::milliseconds TimeUntilIdle{3000};

int main(int argc, char **argv)
{
    using namespace mjbots;

    state = State::STARTUP;

    while (!emergency_stop && !manual_stop_flag.load(std::memory_order_relaxed))
    {
        switch (state)
        {
        case State::STARTUP:
            startup(argc, argv);
            break;

        case State::IDLE:
            idle();
            break;

        case State::RUNNING:
            running();
            break;

        case State::FAULT:
            fault();
            break;
        }
    }

    state = State::STOPPING;
    stopping();
}

void startup(int argc, char **argv)
{
    // --- Logger ---
    logger.initialize({"rframework"});
    logger.log("rframework", "--- ROBOTFRAMEWORK STARTING ---", LogLevel::LOVE);

    // --- Initializing mode (SAFE, CAPPED, UNSAFE) ---
    if (argc > 1)
    {
        std::string arg = argv[1];
        if (arg == "-s" || arg == "-safe")
        {
            // Robot will stop and shutdown at set safety speed limits
            mode = 0;
            logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
        }
        else if (arg == "-c" || arg == "-capped")
        {
            // Robot will have speed capped to speed limits
            mode = 1;
            logger.log("rframework", "Starting in CAPPED mode", LogLevel::INFO);
        }
        else if (arg == "-unsafe")
        {
            // Robot will not follow speed limits
            mode = 2;
            logger.log("rframework", "Starting in UNSAFE mode", LogLevel::WARN);
        }
        else
        {
            // Fallback on SAFE mode
            mode = 0;
            std::cerr << "Unknown flag or argument: " << argv[1] << std::endl;
            logger.log("rframework", std::string("Unknown flag or argument: ") + argv[1], LogLevel::WARN);
            logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
        }
    }
    else
    {
        // SAFE mode is default
        mode = 0;
        logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
    }

    // --- Load YAML config ---
    logger.log("rframework", "Loading configs...", LogLevel::INFO);
    try
    {
        YAML::Node config = YAML::LoadFile("../config/Main.yaml");     // Main control Config file
        YAML::Node s_config = YAML::LoadFile("../config/Safety.yaml"); // Safety Config file
        YAML::Node interval_values = config["intervals"];

        current_limit = s_config["currentLimit"].as<double>();
        fault_grace = s_config["faultyGracePeriod"].as<int>();

        interval_reciver = interval_values["Reciver_interval"].as<int>();
        interval_sender = interval_values["Sender_interval"].as<int>();
        interval_arduino = interval_values["Arduino_interval"].as<int>();
        interval_camera = interval_values["Camera_interval"].as<int>();
        interval_motor = interval_values["Motor_interval"].as<int>();
        time_until_idle = interval_values["Idle_interval"].as<int>();

        logger.log("rframework", "Successfully loaded configs!", LogLevel::INFO);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading Interval config: " << e.what() << std::endl;

        logger.log("rframework", std::string("Failed to load configs: ") + (e.what()), LogLevel::WARN);

        // Fallback defaults
        interval_reciver = 20;
        interval_sender = 1000;
        interval_arduino = 100;
        interval_camera = 200;
        interval_motor = 20;

        current_limit = 5.0;
        fault_grace = 1500;

        time_until_idle = 3000;

        logger.log("rframework", std::string("Failed to load configs: ") + (e.what()), LogLevel::WARN);
        logger.log("rframework", std::string("Using fallback values:") + (e.what()), LogLevel::WARN);
    }

    configData = {
        {"Reciever Interval", interval_reciver},
        {"Sender Interval", interval_sender},
        {"Arduino Interval", interval_arduino},
        {"Camera Interval", interval_camera},
        {"Motor Interval", interval_motor},
        {"Current Limit", current_limit},
        {"Fault Grace", fault_grace},
        {"Time until Idle", time_until_idle}};
    logger.log("rframework", configData, LogLevel::INFO);

    // --- Convert intervals to chrono durations ---
    Reciver_interval = std::chrono::milliseconds(interval_reciver);
    CameraInterval = std::chrono::milliseconds(interval_camera);
    MotorInterval = std::chrono::milliseconds(interval_motor);
    Sender_interval = std::chrono::milliseconds(interval_sender);
    Arduino_interval = std::chrono::milliseconds(interval_arduino);
    FaultGrace = std::chrono::milliseconds(fault_grace);
    TimeUntilIdle = std::chrono::milliseconds(time_until_idle);

    // Set mode of Wheel_math based on flags
    m.setMode(mode);

    // --- Initialize Arduino ---
    logger.log("rframework", "arduino", "Searching for Arduino...", LogLevel::INFO);
    a.findArduino();
    logger.log("rframework", "arduino", "Connecting to Arduino port...", LogLevel::INFO);
    a.connect(a.getPort());

    if (a.isConnected())
    {
        logger.log("rframework", "arduino", std::string("Port found at ") + (a.getPort()), LogLevel::DONE);
    }
    else
    {
        logger.log("rframework", "arduino", "No arduino found", LogLevel::WARN);
    }

    emergency_stop = false;

    // --- Start camera detection thread ---
    if (detect.open_cam() > 0)
    {
        // I didnt detach this beacsue it wanted to close it later on.
        camera_thread = std::thread(CameraThread, std::ref(detect));
        logger.log("rframework", "camball", "Camera thread started", LogLevel::INFO);
    }

    // --- Setup signal handler to catch Ctrl+C ---
    std::signal(SIGINT, signalHandler);

    // --- Stop all motors initially to clear faults ---
    for (const auto &pair : telemetry.controllers)
    {
        pair.second->SetStop();
    }
    logger.log("rframework", "Sent stop to all controllers", LogLevel::DONE);

    // --- Log UDP ports ---
    logger.log("rframework", std::string("Sending port at: ") + std::to_string(UDP.getSenderPort()), LogLevel::LOVE);
    logger.log("rframework", std::string("Recieving port at: ") + std::to_string(UDP.getRecieverPort()), LogLevel::LOVE);

    // --- Set state to IDLE immediate after STARTUP ---
    logger.log("rframework", "Entering IDLE state", LogLevel::INFO);
    state = State::IDLE;
}

void idle()
{
    current_time = std::chrono::steady_clock::now();

    static auto last_reciver_time = current_time;
    static auto last_motor_time = current_time;
    static auto last_sender_time = current_time;

    // --- UDP Receiver ---
    if (current_time - last_reciver_time >= Reciver_interval)
    {
        msg = UDP.receive(); // Receive new message
        if (msg == "TIMEOUT" || msg == "IDLE")
        {
            velocity_map = {{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}}; // Stop wheels
        }
        else
        {
            // std::cout << msg << "\n";
            logger.log("rframework", "reciever", std::string("Message Recieved: ") + msg, LogLevel::INFO);
            last_command_time = current_time;

            logger.log("rframework", "Entering RUNNING state", LogLevel::INFO);
            state = State::RUNNING;
            return;
        }
        last_reciver_time = current_time;

        if (current_time - last_command_time > TimeUntilIdle || msg == "IDLE")
        {
            state = State::IDLE;
            return;
        }
    }

    // --- Motor Telemetry and Safety Check ---
    if (current_time - last_motor_time >= MotorInterval)
    {
        process_motor_telemetry();
        last_motor_time = current_time;
    }

    // --- UDP Telemetry Sender ---
    // In IDLE state, robot sends state 3 times less often
    if (current_time - last_sender_time >= (Sender_interval * 3))
    {
        std::string msg = "Robot State: IDLE, Battery Voltage:" + std::to_string(sender_msg.voltage) +
                          ", Ball Detection:" + std::to_string(sender_msg.ball_present);
        logger.log("rframework", "sender", msg, LogLevel::INFO);
        UDP.send(msg);
        last_sender_time = current_time;
    }
}

void running()
{
    // --- Running control loop ---

    current_time = std::chrono::steady_clock::now();

    // Static timers for periodic tasks
    static auto last_reciver_time = current_time;
    static auto last_motor_time = current_time;
    static auto last_camera_time = current_time;
    static auto last_sender_time = current_time;
    static auto last_arduino_time = current_time;

    // --- UDP Receiver ---
    if (current_time - last_reciver_time >= Reciver_interval)
    {
        msg = UDP.receive(); // Receive new message
        if (msg == "TIMEOUT" || msg == "IDLE")
        {
            logger.log("rframework", "reciever", "UDP TIMEOUT", LogLevel::WARN);
            velocity_map = {{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}}; // Stop wheels
        }
        else
        {
            // std::cout << msg << "\n";
            logger.log("rframework", "reciever", std::string("Message Recieved: ") + msg, LogLevel::INFO);
            cmd.decode_cmd(msg); // Decode velocity commands
            wheel_velocity = m.calculate(cmd.velocity_x, cmd.velocity_y, cmd.velocity_w);
            // Map velocities to motors
            velocity_map = {
                {1, wheel_velocity[0]},
                {2, wheel_velocity[1]},
                {3, wheel_velocity[2]},
                {4, wheel_velocity[3]}};
            // Update last command time
            last_command_time = current_time;
        }
        last_reciver_time = current_time;

        if (current_time - last_command_time > TimeUntilIdle || msg == "IDLE")
        {
            logger.log("rframework", "Entering IDLE state", LogLevel::INFO);
            state = State::IDLE;
            return;
        }
    }

    // --- Camera Ball Detection ---
    if (current_time - last_camera_time >= CameraInterval)
    {
        bool camera_ball_detected = ball_detected.load(std::memory_order_relaxed);
        // std::cout << camera_ball_detected << "\n";
        logger.log("rframework", "camball", std::string("ball_detected=") + (camera_ball_detected ? "true" : "false"), LogLevel::INFO);
        sender_msg.ball_present = camera_ball_detected;
        last_camera_time = current_time;
    }

    // --- Motor Telemetry and Safety Check ---
    if (current_time - last_motor_time >= MotorInterval)
    {
        process_motor_telemetry();
        last_motor_time = current_time;
    }

    // --- UDP Telemetry Sender ---
    if (current_time - last_sender_time >= Sender_interval)
    {
        std::string msg = "Robot State: RUNNING, Battery Voltage:" + std::to_string(sender_msg.voltage) +
                          ", Ball Detection:" + std::to_string(sender_msg.ball_present);
        logger.log("rframework", "sender", msg, LogLevel::INFO);
        UDP.send(msg);
        last_sender_time = current_time;
    }

    // --- Arduino Commands ---
    if (current_time - last_arduino_time >= Arduino_interval)
    {
        if (a.isConnected())
        {
            if (cmd.kick)
            {
                a.sendCommand(kick); // Kick
                logger.log("rframework", "arduino", "Sent kick", LogLevel::HATE);
            }
            else if (cmd.dribble)
            {
                a.sendCommand(dribble); // Dribble
                logger.log("rframework", "arduino", "Sent dribble", LogLevel::LOVE);
            }
            else
            {
                a.sendCommand(stop_dribble); // Stop
                logger.log("rframework", "arduino", "Sent stop dribble", LogLevel::INFO);
            }
        }
        last_arduino_time = current_time;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Reduce CPU load
}

void fault()
{
    // In FAULT state, keep running but do not send commands to faulty motors
    current_time = std::chrono::steady_clock::now();

    static auto last_reciver_time = current_time;
    static auto last_motor_time = current_time;
    static auto last_camera_time = current_time;
    static auto last_sender_time = current_time;
    static auto last_arduino_time = current_time;

    // --- UDP Receiver ---
    if (current_time - last_reciver_time >= Reciver_interval)
    {
        msg = UDP.receive();
        // In FAULT state, always stop all motors, and keep track of which are faulty
        for (const auto &pair : telemetry.controllers)
        {
            pair.second->SetStop();
        }
        logger.log("rframework", "Sent stop to all controllers", LogLevel::DONE);

        // Log FAULT state for each faulty motor
        if (!faulty_motors.empty())
        {
            for (int id : faulty_motors)
            {
                logger.log("rframework", "motor-" + std::to_string(id), "FAULT state: all motors shut off. This motor is FAULTY.", LogLevel::CRIT);
            }
        }

        last_reciver_time = current_time;
    }

    // --- Camera Ball Detection ---
    if (current_time - last_camera_time >= CameraInterval)
    {
        bool camera_ball_detected = ball_detected.load(std::memory_order_relaxed);
        logger.log("rframework", "camball", std::string("ball_detected=") + (camera_ball_detected ? "true" : "false"), LogLevel::INFO);
        sender_msg.ball_present = camera_ball_detected;
        last_camera_time = current_time;
    }

    // --- Motor Telemetry and Safety Check ---
    if (current_time - last_motor_time >= MotorInterval)
    {
        process_motor_telemetry();
        last_motor_time = current_time;
    }

    // --- UDP Telemetry Sender ---
    // In FAULT state, robot sends state 2 times less often
    if (current_time - last_sender_time >= (Sender_interval * 2))
    {
        std::string fault_list;
        if (!faulty_motors.empty())
        {
            for (int id : faulty_motors)
            {
                fault_list += std::to_string(id) + ",";
            }
            if (!fault_list.empty())
                fault_list.pop_back(); // Remove trailing comma
        }
        else
        {
            fault_list = "none";
        }
        std::string msg = "Robot State: FAULT, Battery Voltage:" + std::to_string(sender_msg.voltage) +
                          ", Ball Detection:" + std::to_string(sender_msg.ball_present) +
                          ", Faulty Motors: [" + fault_list + "]";
        logger.log("rframework", "sender", msg, LogLevel::INFO);
        UDP.send(msg);
        last_sender_time = current_time;
    }

    // --- Arduino Commands ---
    if (current_time - last_arduino_time >= Arduino_interval)
    {
        if (a.isConnected())
        {
            if (cmd.kick)
            {
                a.sendCommand(kick);
                logger.log("rframework", "arduino", "Sent kick", LogLevel::HATE);
            }
            else if (cmd.dribble)
            {
                a.sendCommand(dribble);
                logger.log("rframework", "arduino", "Sent dribble", LogLevel::LOVE);
            }
            else
            {
                a.sendCommand(stop_dribble);
                logger.log("rframework", "arduino", "Sent stop dribble", LogLevel::INFO);
            }
        }
        last_arduino_time = current_time;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void stopping()
{
    // --- Stopping ---
    for (const auto &pair : telemetry.controllers)
    {
        pair.second->SetStop();
    }
    a.disconnect();
    logger.log("rframework", "Emergency stop activated, shutting down", LogLevel::HATE);

    // Stop camera thread and join
    stop_camera_thread.store(true, std::memory_order_relaxed);
    if (camera_thread.joinable())
    {
        camera_thread.join();
    }

    std::cout << "Emergency Stop has been activated\n";
    logger.closeAll();
}

inline void process_motor_telemetry()
{
    auto servo_status = telemetry.cycle(velocity_map);
    float voltage[4] = {0, 0, 0, 0};
    int i = 0;
    auto now = std::chrono::steady_clock::now();
    std::set<int> responding_motors;
    for (const auto &pair : servo_status)
    {
        const auto &r = pair.second;
        int motor_id = pair.first;
        responding_motors.insert(motor_id);
        voltage[i] = r.voltage;
        std::string sub = "motor-" + std::to_string(motor_id);
        std::map<std::string, double> data = {
            {"temperature", r.temperature},
            {"voltage", r.voltage},
            {"velocity", r.velocity},
            {"current", r.current},
            {"mode", static_cast<double>(r.mode)}};
        logger.log("rframework", sub, data, "", LogLevel::INFO);
        if (r.current > current_limit)
        {
            logger.log("rframework", sub, "Overcurrent detected", LogLevel::CRIT);
            emergency_stop = true;
        }
        // Update last response time for this motor
        last_motor_response[motor_id] = now;
        i++;
    }
    // Mark motors as faulty if not responding for >1s
    for (int id = 1; id <= 4; ++id)
    {
        auto it = last_motor_response.find(id);
        auto ms_since_response = it == last_motor_response.end() ? FaultGrace.count() + 1 : std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
        if (ms_since_response > FaultGrace.count())
        {
            if (faulty_motors.count(id) == 0)
            {
                logger.log("rframework", "motor-" + std::to_string(id), "No telemetry for >" + std::to_string(FaultGrace.count()) + "ms, marking as FAULTY", LogLevel::CRIT);
            }
            faulty_motors.insert(id);
        }
    }
    // If any new faulty motors detected, enter FAULT state
    if (!faulty_motors.empty() && state != State::FAULT)
    {
        logger.log("rframework", "FAULT detected", LogLevel::CRIT);
        logger.log("rframework", "Entering FAULT state", LogLevel::INFO);
        state = State::FAULT;
    }
    float sum = 0;
    int count = 0;
    for (int j = 0; j < 4; j++)
    {
        if (voltage[j] > 0)
        {
            sum += voltage[j];
            count++;
        }
    }
    sender_msg.voltage = (count > 0) ? (sum / count) : 0;
}

// --- Signal handler for Ctrl+C ---
void signalHandler(int signum)
{
    std::cout << "\nSIGINT received. Stopping safely...\n";
    manual_stop_flag.store(true);
}