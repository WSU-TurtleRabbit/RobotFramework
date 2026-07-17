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
#include <algorithm>
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
#include "telemetry_wire.h"
#include "arduino.h"
#include "MotionBridge.h"
#include "motion_config_yaml.h"
#include "supervisor.h"
#include <array>
#include <cstdint>
#include <optional>
#include <yaml-cpp/yaml.h>
#include "Logger/Logger.h"

// --- Atomic flags for inter-thread communication ---
std::atomic<bool> ball_detected{false};      // Stores ball detection result from camera thread
std::atomic<bool> stop_camera_thread{false}; // Signals the camera thread to stop
std::atomic<bool> manual_stop_flag{false};   // Signals main loop to stop on Ctrl+C

// Full onboard ball observation, published by the camera thread.
// Read without a lock: trivially copyable POD stored atomically.
static_assert(std::is_trivially_copyable<BallObservation>::value,
              "BallObservation must be trivially copyable for std::atomic");
std::atomic<BallObservation> ball_observation{
    BallObservation{false, 0.f, 0.f, 0.f, 0.f, 0.f}};

// --- Forward declaration for signal handler ---
void signalHandler(int signum);

// --- Thread function for camera detection ---
void CameraThread(BallDetection &detector)
{
    while (!stop_camera_thread.load(std::memory_order_relaxed))
    {
        BallObservation obs = detector.observe();
        ball_observation.store(obs, std::memory_order_relaxed);
        ball_detected.store(obs.found, std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(300)); // Reduce CPU usage
    }
}

// --- Struct for telemetry message to send ---
struct Telemetry_msg
{
    bool  ball_present; // True if ball detected
    float voltage;      // Average motor voltage
    BallObservation obs; // Onboard detection details
};

int main(int argc, char **argv)
{
    using namespace mjbots;

    char kick = 'K';
    char dribble = 'D';
    char stop_dribble = 'S';

    int mode = 0;

    double zero = 0.0;

    // Safety policy (Safety/supervisor.h); defaults ported from phoenix-rf,
    // overridden by config/Safety.yaml below.
    rf::SafetyConfig safety_cfg;
    double current_grace_ms = 300.0; // sustained over-current window
    double fault_grace_ms = 50.0;    // window for fault/temp/voltage trips

    // --- Interval times (ms) for periodic tasks ---
    int interval_reciver, interval_sender, interval_arduino, interval_camera, interval_motor;
    // Telemetry v2 cadence, ms.
    int interval_telemetry = 200;

    // This robot's command-channel id (config Robot_id; -1 = accept any).
    int robot_id = -1;
    // Physical identity letter reported as rid= in telemetry.
    std::string robot_rid = "?";

    // --- Logger ---
    Logger logger("logs");
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
            logger.log("rframework", std::string("Unknown flag or argument: ") + 
                argv[1], LogLevel::WARN);
            logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
        }
    }
    else
    {
        // SAFE mode is default
        mode = 0;
        logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
    }

    // CLI flag -> supervisor drive mode.
    safety_cfg.mode = (mode == 2)   ? rf::DriveMode::Unsafe
                      : (mode == 1) ? rf::DriveMode::Capped
                                    : rf::DriveMode::Safe;

    // --- Load YAML config ---
    logger.log("rframework", "Loading configs...", LogLevel::INFO);
    std::map<std::string, double> configData;
    try
    {
        YAML::Node config = YAML::LoadFile("../config/Main.yaml");     // Main control Config file
        YAML::Node s_config = YAML::LoadFile("../config/Safety.yaml"); // Safety Config file
        YAML::Node interval_values = config["intervals"];

        // Safety supervisor: envelope + trip thresholds (defaults kept when
        // a key is absent).
        if (s_config["envelope"])
        {
            YAML::Node env = s_config["envelope"];
            if (env["safeLinear"]) safety_cfg.safe_linear_mps = env["safeLinear"].as<double>();
            if (env["safeAngular"]) safety_cfg.safe_angular_rps = env["safeAngular"].as<double>();
            if (env["cappedLinear"]) safety_cfg.capped_linear_mps = env["cappedLinear"].as<double>();
            if (env["cappedAngular"]) safety_cfg.capped_angular_rps = env["cappedAngular"].as<double>();
        }
        if (s_config["currentLimit"]) safety_cfg.trip_current_a = s_config["currentLimit"].as<double>();
        if (s_config["currentGraceMs"]) current_grace_ms = s_config["currentGraceMs"].as<double>();
        if (s_config["faultGraceMs"]) fault_grace_ms = s_config["faultGraceMs"].as<double>();
        if (s_config["commandTimeoutMs"]) safety_cfg.command_timeout_ms = s_config["commandTimeoutMs"].as<uint64_t>();
        if (s_config["tripTempC"]) safety_cfg.trip_temp_c = s_config["tripTempC"].as<double>();
        if (s_config["minBusVoltage"]) safety_cfg.min_bus_voltage = s_config["minBusVoltage"].as<double>();
        if (s_config["recoveryS"]) safety_cfg.recovery_s = s_config["recoveryS"].as<double>();
        if (s_config["watchdogTimeout"]) safety_cfg.watchdog_s = s_config["watchdogTimeout"].as<double>();

        interval_reciver = interval_values["Reciver_interval"].as<int>();
        interval_sender = interval_values["Sender_interval"].as<int>();
        interval_arduino = interval_values["Arduino_interval"].as<int>();
        interval_camera = interval_values["Camera_interval"].as<int>();
        interval_motor = interval_values["Motor_interval"].as<int>();
        // v2 telemetry cadence; older configs fall back to Sender_interval.
        interval_telemetry = interval_values["Telemetry_interval"]
                                 ? interval_values["Telemetry_interval"].as<int>()
                                 : interval_sender;

        // Optional: which command-channel id this robot answers to.
        if (config["Robot_id"])
            robot_id = config["Robot_id"].as<int>();
        if (config["Robot_rid"])
            robot_rid = config["Robot_rid"].as<std::string>();

        logger.log("rframework", "Successfully loaded configs!", LogLevel::INFO);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading Interval config: " << e.what() << std::endl;

        logger.log("rframework", std::string("Failed to load configs: ") + (e.what()), LogLevel::WARN);

        // Fallback defaults
        interval_reciver = 5;
        interval_sender = 1000;
        interval_arduino = 100;
        interval_camera = 200;
        // 100 Hz control tick; 4 ms (250 Hz) is the target once validated
        // on hardware.
        interval_motor = 10;
        interval_telemetry = 200;

        // safety_cfg keeps its built-in defaults.

        logger.log("rframework", std::string("Failed to load configs: ") + (e.what()), LogLevel::WARN);
        logger.log("rframework", std::string("Using fallback values:") + (e.what()), LogLevel::WARN);
    }

    // Grace windows are configured in milliseconds (robust to control-rate
    // changes) but the supervisor counts consecutive TICKS, like safety.rs.
    safety_cfg.current_grace_ticks = static_cast<uint32_t>(
        std::max(1.0, current_grace_ms / std::max(1, interval_motor)));
    safety_cfg.fault_grace_ticks = static_cast<uint32_t>(
        std::max(1.0, fault_grace_ms / std::max(1, interval_motor)));

    configData = {
        {"Reciever Interval", interval_reciver},
        {"Sender Interval", interval_sender},
        {"Arduino Interval", interval_arduino},
        {"Camera Interval", interval_camera},
        {"Motor Interval", interval_motor},
        {"Current Limit", safety_cfg.trip_current_a},
        {"Current Grace Ticks", static_cast<double>(safety_cfg.current_grace_ticks)},
        {"Fault Grace Ticks", static_cast<double>(safety_cfg.fault_grace_ticks)},
        {"Robot Id", robot_id}};
    logger.log("rframework", configData, LogLevel::INFO);

    // --- Convert intervals to chrono durations ---
    static auto Reciver_interval = std::chrono::milliseconds(interval_reciver);
    static auto CameraInterval = std::chrono::milliseconds(interval_camera);
    static auto MotorInterval = std::chrono::milliseconds(interval_motor);
    static auto Telemetry_interval = std::chrono::milliseconds(interval_telemetry);
    static auto Arduino_interval = std::chrono::milliseconds(interval_arduino);

    static auto Motor_Command_interval = std::chrono::milliseconds(500);

    // --- Onboard motion (MV2 pose-target executor) config ---
    rf::MotionSettings motion_settings = rf::loadMotionSettings("../config/Motion.yaml");

    // --- Initialize modules ---
    BallDetection detect; // Camera detection
    UDP UDP;              // UDP communication
    Wheel_math m;         // Wheel velocity calculations
    cmdDecoder cmd;       // Decode incoming commands
    Telemetry telemetry;  // Motor telemetry
    Arduino a;            // Arduino controller
    // Onboard trajectory following: MV2 frames in, shaped body twists out.
    rf::MotionBridge bridge(motion_settings.config, motion_settings.imu_yaw_rate_sign, robot_id);
    // Safety supervisor: envelope shaping, protective trips, auto-recovery.
    rf::Supervisor supervisor(safety_cfg);

    std::string msg;                    // Incoming UDP message
    std::vector<double> wheel_velocity; // Calculated wheel velocities
    std::map<int, double> velocity_map; // Motor ID → velocity map
    Telemetry_msg sender_msg;           // Telemetry message to send

    // Last commanded body twist (post-envelope) — the supervisor's
    // "operator idle" input for auto-recovery.
    BodyTwist last_cmd_twist;
    // Monotonic ms of the last VALID drive command (v1 or MV2) for the
    // staleness gate; nullopt until one arrives.
    std::optional<uint64_t> last_valid_cmd_ms;

    // --- Telemetry v2 bookkeeping ---
    const auto process_start = std::chrono::steady_clock::now();
    uint32_t telem_seq = 0;      // telemetry packet counter
    uint64_t cmd_rx_count = 0;   // valid drive commands received
    int last_cmd_id = -1;        // robot id in the last valid command
    BodyTwist odo_twist;         // latest FK wheel-odometry body twist
    double vmin_last = 0.0;      // lowest single-motor voltage, last cycle
    double last_cycle_ms = 0.0;  // CAN cycle duration, ms
    std::map<int, MotorTelemetry> last_servo_status;  // latest motor replies
    // Motor-tick period stats over the current telemetry window.
    double loop_ms_sum = 0.0, loop_ms_min = 1e9, loop_ms_max = 0.0;
    int loop_ms_n = 0;

    // --- Initialize Arduino ---
    logger.log("rframework", "arduino", "Searching for Arduino...", LogLevel::INFO);
    a.findArduino();
    logger.log("rframework", "arduino", "Connecting to Arduino port...", LogLevel::INFO);
    a.connect(a.getPort());

    // Only accept commands addressed to this robot (-1 = accept any).
    cmd.expected_id = robot_id;

    // if (a.isConnected())
    // {
    //     logger.log("rframework", "arduino", std::string("Port found at ") + (a.getPort()), LogLevel::DONE);
    // }
    // else
    // {
    //     logger.log("rframework", "arduino", "No arduino found", LogLevel::WARN);
    // }

    // --- Start camera detection thread ---
    std::thread camera_thread;
    if (detect.open_cam() > 0)
    {
        // I didnt detach this beacsue it wanted to close it later on.
        camera_thread = std::thread(CameraThread, std::ref(detect));
        logger.log("rframework", "camball", "Camera thread started", LogLevel::INFO);
    }

    // --- Setup signal handler to catch Ctrl+C and systemd stop ---
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // --- Stop all motors initially to clear faults ---
    for (const auto &pair : telemetry.controllers)
    {
        pair.second->SetStop();
    }
    logger.log("rframework", "Sent stop to all controllers", LogLevel::DONE);

    // --- Log UDP ports ---
    logger.log("rframework", std::string("Sending port at: ") + 
        std::to_string(UDP.getSenderPort()), LogLevel::LOVE);
    logger.log("rframework", std::string("Recieving port at: ") + 
        std::to_string(UDP.getRecieverPort()), LogLevel::LOVE);

    // --- Main control loop ---
    // Protective trips no longer kill the process: the supervisor latches a
    // recoverable estop (coast + auto-recovery / STOP clears). Only SIGINT/
    // SIGTERM end the loop.
    logger.log("rframework", "Entering main control loop", LogLevel::LOVE);
    while (!manual_stop_flag.load(std::memory_order_relaxed))
    {
        auto current_time = std::chrono::steady_clock::now();

        // Static timers for periodic tasks
        static auto last_reciver_time = current_time;
        static auto last_motor_time = current_time;
        static auto last_camera_time = current_time;
        static auto last_sender_time = current_time;
        static auto last_arduino_time = current_time;


        static auto last_known_message = current_time;
        static int timeout_count = 0;
        static const int TIMEOUT_LIMIT = 3; // 3 x 20ms = 60ms grace before stopping

        // --- UDP Receiver ---
        if (current_time - last_reciver_time >= Reciver_interval)
        {
            msg = UDP.receive(); // Receive new message
            if (msg == "TIMEOUT")
            {
                // Legacy v1 path: N empty polls -> stop. While the MV2
                // executor is active its own watchdog tiers (Fresh -> BRAKE
                // -> COAST) replace this: braking is shaped, not a cliff.
                if (!bridge.active())
                {
                    timeout_count++;
                    if (timeout_count >= TIMEOUT_LIMIT)
                    {
                        logger.log("rframework", "reciever", "UDP TIMEOUT - stopping motors", LogLevel::WARN);
                        velocity_map = {{1, zero}, {2, zero}, {3, zero}, {4, zero}}; // Stop wheels
                    }
                }
            }
            else
            {
                // Strict decode: a packet either classifies cleanly or the
                // robot's motion state is left untouched.
                switch (cmd.decode_cmd(msg))
                {
                case CmdType::Velocity:
                {
                    timeout_count = 0;
                    // A v1 velocity command takes the robot back to direct
                    // (legacy) control; the executor stands down.
                    if (bridge.active())
                    {
                        bridge.clear();
                        logger.log("rframework", "reciever", "v1 command - leaving MV2 mode", LogLevel::INFO);
                    }
                    logger.log("rframework", "reciever", std::string("Message Recieved: ") + msg, LogLevel::INFO);
                    // Drive-mode envelope: direction-preserving scale-to-fit
                    // (SAFE no longer hard-stops on an over-limit command).
                    const BodyTwist shaped = supervisor.shape_twist(
                        BodyTwist{cmd.velocity_x, cmd.velocity_y, cmd.velocity_w});
                    wheel_velocity = m.calculate(shaped.vx, shaped.vy, shaped.w);
                    // Map velocities to motors
                    velocity_map = {
                        {1, wheel_velocity[0]},
                        {2, wheel_velocity[1]},
                        {3, wheel_velocity[2]},
                        {4, wheel_velocity[3]}};

                    last_known_message = current_time;
                    last_cmd_twist = shaped;
                    last_valid_cmd_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        current_time.time_since_epoch()).count();
                    cmd_rx_count++;
                    last_cmd_id = cmd.id;
                    break;
                }

                case CmdType::Move:
                {
                    if (!motion_settings.enabled)
                    {
                        logger.log("rframework", "reciever", "MV2 frame ignored (motion disabled in config)", LogLevel::WARN);
                        break;
                    }
                    const uint64_t rx_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        current_time.time_since_epoch()).count();
                    switch (bridge.accept_frame(msg, rx_ms))
                    {
                    case rf::Mv2Accept::Accepted:
                        timeout_count = 0;
                        last_known_message = current_time;
                        last_valid_cmd_ms = rx_ms;
                        cmd_rx_count++;
                        last_cmd_id = bridge.last_robot_id();
                        break;
                    case rf::Mv2Accept::WrongId:
                        logger.log("rframework", "reciever", "MV2 frame for another robot ignored", LogLevel::WARN);
                        break;
                    case rf::Mv2Accept::Malformed:
                    default:
                        logger.log("rframework", "reciever",
                            std::string("Malformed MV2 frame rejected: ") + msg, LogLevel::WARN);
                        break;
                    }
                    break;
                }

                case CmdType::Stop:
                    // Operator STOP: safe-stop the motors and clear any
                    // latched safety trip — the daemon KEEPS RUNNING so the
                    // operator can resume without an SSH round-trip (ported
                    // semantics from phoenix-rf; this used to exit(0)).
                    logger.log("rframework", "reciever", "UDP STOP - safe stop, latches cleared", LogLevel::HATE);
                    velocity_map = {{1, zero}, {2, zero}, {3, zero}, {4, zero}}; // Stop wheels
                    wheel_velocity = {zero, zero, zero, zero};
                    last_cmd_twist = BodyTwist{};
                    last_valid_cmd_ms.reset();
                    bridge.clear();     // executor stands down (coast)
                    supervisor.clear(); // operator recovery of latched trips

                    for (const auto &pair : telemetry.controllers)
                    {
                        pair.second->SetStop();
                    }
                    break;

                case CmdType::Ping:
                    // Link discovery only — deliberately NOT a drive command,
                    // so a robot fed only PINGs still times out and stops.
                    logger.log("rframework", "reciever", "PING received", LogLevel::INFO);
                    break;

                case CmdType::Calibrate:
                    // No onboard commissioner in this framework (yet).
                    logger.log("rframework", "reciever", "CALIBRATE received (not supported, ignored)", LogLevel::WARN);
                    break;

                case CmdType::WrongId:
                    logger.log("rframework", "reciever",
                        std::string("Command for robot ") + std::to_string(cmd.id) +
                        " ignored (this is robot " + std::to_string(robot_id) + ")",
                        LogLevel::WARN);
                    break;

                case CmdType::Malformed:
                default:
                    logger.log("rframework", "reciever",
                        std::string("Malformed packet rejected: ") + msg, LogLevel::WARN);
                    break;
                }
            }
            last_reciver_time = current_time;
        }

        // --- Camera Ball Detection ---
        if (current_time - last_camera_time >= CameraInterval)
        {
            BallObservation obs = ball_observation.load(std::memory_order_relaxed);
            sender_msg.obs = obs;
            sender_msg.ball_present = obs.found;
            logger.log("rframework", "camball",
                std::string("ball_detected=") + (obs.found ? "true" : "false") +
                " px=" + std::to_string(obs.px) +
                " py=" + std::to_string(obs.py) +
                " r="  + std::to_string(obs.radius) +
                " b="  + std::to_string(obs.bearing) +
                " c="  + std::to_string(obs.confidence),
                LogLevel::INFO);
            last_camera_time = current_time;
        }

        if (current_time - last_known_message >= Motor_Command_interval)
        {
            // velocity_map = {{1, 0.0}, {2, 0.0}, {3, 0.0}, {4, 0.0}}; // Stop wheels
        }

        // --- Motor Telemetry and Safety Check ---
        if (current_time - last_motor_time >= MotorInterval)
        {
            // Measured wheel velocities (rev/s, motor id i -> index i-1) from
            // the PREVIOUS cycle's replies — the executor's odometry input.
            static std::array<double, 4> measured_rev_s = {0.0, 0.0, 0.0, 0.0};
            // Actual elapsed time between motor ticks (the loop timer is
            // approximate; the executor integrates with the real dt).
            static auto last_motor_tick_wall = current_time;
            const double motor_dt =
                std::chrono::duration<double>(current_time - last_motor_tick_wall).count();
            last_motor_tick_wall = current_time;

            const uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time.time_since_epoch()).count();

            // Loop-health stats for telemetry (period achieved + jitter).
            if (loop_ms_n > 0 || motor_dt > 0.0)
            {
                const double dt_ms = motor_dt * 1000.0;
                loop_ms_sum += dt_ms;
                loop_ms_min = std::min(loop_ms_min, dt_ms);
                loop_ms_max = std::max(loop_ms_max, dt_ms);
                loop_ms_n++;
            }

            // Wheel odometry (FK) from the previous cycle's measured
            // velocities — executor input and telemetry odo_vx/vy/w.
            odo_twist = m.kinematics().forward(measured_rev_s);

            bool energize = true;

            if (bridge.active())
            {
                // Onboard trajectory following: wheel odometry + gyro in,
                // shaped body twist out, at the motor rate — wheel motion no
                // longer depends on Wi-Fi cadence.
                const double imu_yaw_radps = telemetry.attitude_present
                    ? telemetry.imu_yaw_dps * (M_PI / 180.0)
                    : std::nan("");
                const phx::ExecOutput out = bridge.tick(
                    now_ms, motor_dt, phx::Twist{odo_twist.vx, odo_twist.vy, odo_twist.w}, imu_yaw_radps);

                // Operator envelope on top of the executor's mode caps —
                // direction-preserving, so the trajectory bends nowhere.
                const BodyTwist shaped = supervisor.shape_twist(
                    BodyTwist{out.twist.lin.x, out.twist.lin.y, out.twist.ang});
                wheel_velocity = m.calculate(shaped.vx, shaped.vy, shaped.w);
                velocity_map = {
                    {1, wheel_velocity[0]},
                    {2, wheel_velocity[1]},
                    {3, wheel_velocity[2]},
                    {4, wheel_velocity[3]}};
                energize = out.energize;
                last_cmd_twist = BodyTwist{out.desired.lin.x, out.desired.lin.y, out.desired.ang};
            }

            // Safety gate: a latched trip coasts the motors; a stale v1
            // command holds zero (the MV2 path has its own richer brake/
            // coast tiers, so only the estop latch applies there).
            if (supervisor.estop())
            {
                velocity_map = {{1, zero}, {2, zero}, {3, zero}, {4, zero}};
                energize = false;
            }
            else if (!bridge.active())
            {
                if (supervisor.motion_gate(now_ms, last_valid_cmd_ms).has_value())
                {
                    velocity_map = {{1, zero}, {2, zero}, {3, zero}, {4, zero}};
                }
            }

            const auto cycle_begin = std::chrono::steady_clock::now();
            auto servo_status = telemetry.cycle(velocity_map, energize); // Send commands & receive telemetry
            last_cycle_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - cycle_begin).count();
            last_servo_status = servo_status;

            // Stash measured velocities for the next tick's odometry.
            measured_rev_s = {0.0, 0.0, 0.0, 0.0};
            for (const auto &pair : servo_status)
            {
                const int id = pair.first;
                if (id >= 1 && id <= 4 && std::isfinite(pair.second.velocity))
                    measured_rev_s[id - 1] = pair.second.velocity;
            }

            float voltage[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            int i = 0;
            vmin_last = 0.0;

            for (const auto &pair : servo_status)
            {
                const auto &r = pair.second;
                if (i < 4)
                    voltage[i] = r.voltage;
                if (std::isfinite(r.voltage) && r.voltage > 0.0 &&
                    (vmin_last <= 0.0 || r.voltage < vmin_last))
                    vmin_last = r.voltage;
                int motor_id = pair.first;

                std::string sub = std::string("motor-") + std::to_string(motor_id);
                std::map<std::string, double> data = {
                    {"temperature", r.temperature},
                    {"voltage", r.voltage},
                    {"velocity", r.velocity},
                    {"current", r.current},
                    {"mode", static_cast<double>(r.mode)},
                    {"fault", static_cast<double>(r.fault)}};
                logger.log("rframework", sub, data, "", LogLevel::INFO);

                // std::cout << "Motor ID: " << motor_id << " Position is: " << r.position << " Mode is: "<< r.mode<< " Velocity is: " << r.velocity<< " Current is: "<< r.current<<"\n";

                i++;
            }

            // Compute average voltage
            float sum = 0;
            for (int j = 0; j < 4; j++)
                sum += voltage[j];
            sender_msg.voltage = sum / 4;

            // --- Safety supervisor: per-motor trips with grace counts ---
            // (The old inline gate compared NaN > limit — see the motor:
            // commit — and hard-exited the process; trips now latch a
            // RECOVERABLE estop.)
            std::vector<rf::MotorObs> motor_obs;
            for (const auto &pair : telemetry.controllers)
            {
                rf::MotorObs o;
                o.id = pair.first;
                auto it = servo_status.find(pair.first);
                if (it != servo_status.end())
                {
                    o.replied = true;
                    o.fault = it->second.fault;
                    o.temperature = it->second.temperature;
                    o.current = std::isfinite(it->second.current) ? it->second.current : 0.0;
                }
                motor_obs.push_back(o);
            }
            for (const rf::StopReason &r : supervisor.observe(motor_obs, sender_msg.voltage))
            {
                logger.log("rframework",
                    std::string("SAFETY TRIP: ") + r.word() +
                    (r.motor ? (std::string(" motor ") + std::to_string(r.motor)) : std::string()),
                    LogLevel::CRIT);
            }

            // Auto-recovery: after recoveryS healthy seconds with the
            // operator commanding zero (or gone stale), the latch releases.
            const bool cmd_stale = !last_valid_cmd_ms ||
                (now_ms - *last_valid_cmd_ms > supervisor.config().command_timeout_ms);
            const bool operator_idle = cmd_stale ||
                (std::abs(last_cmd_twist.vx) + std::abs(last_cmd_twist.vy) +
                     std::abs(last_cmd_twist.w) < 0.02 && !cmd.kick);
            if (auto released = supervisor.try_recover(now_ms, operator_idle))
            {
                for (const rf::StopReason &r : *released)
                {
                    logger.log("rframework",
                        std::string("SAFETY RECOVERED: ") + r.word(), LogLevel::DONE);
                }
            }

            // std::cout << sender_msg.voltage << "\n";

            last_motor_time = current_time;
        }

        // --- UDP Telemetry Sender (protocol v2, key=value CSV) ---
        // v1 keys come first so pre-v2 dashboards still parse; mv=2 is the
        // capability flag the server gates MV2 pose-target frames on.
        if (current_time - last_sender_time >= Telemetry_interval)
        {
            const uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time.time_since_epoch()).count();

            rf::TelemetrySnapshot snap;
            // v1 core.
            snap.state = supervisor.estop() ? "estop" : "active";
            snap.voltage = sender_msg.voltage;
            snap.ball_found = sender_msg.obs.found;
            snap.ball_px = sender_msg.obs.px;
            snap.ball_py = sender_msg.obs.py;
            snap.ball_radius = sender_msg.obs.radius;
            snap.ball_bearing = sender_msg.obs.bearing;
            snap.ball_confidence = sender_msg.obs.confidence;
            snap.robot_ts_ms = now_ms;
            // v2 identity & link.
            snap.rid = robot_rid;
            snap.seq = telem_seq++;
            snap.up_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - process_start).count();
            snap.ifip = UDP.local_ip();
            snap.vmin = vmin_last;
            snap.m_exp = static_cast<uint32_t>(telemetry.controllers.size());
            snap.cmd_age_ms = last_valid_cmd_ms
                ? static_cast<int64_t>(now_ms - *last_valid_cmd_ms) : -1;
            snap.cmd_rx = cmd_rx_count;
            snap.cmd_last_id = last_cmd_id;
            snap.arduino_connected = a.isConnected();
            snap.camera_running = camera_thread.joinable();
            snap.estop = supervisor.estop();
            snap.tx_err = UDP.tx_errors();
            snap.cycle_ms = last_cycle_ms;
            // Per-motor block: every expected motor appears; the ones that
            // replied carry live numbers.
            for (const auto &pair : telemetry.controllers)
            {
                rf::MotorTelem mt;
                mt.id = pair.first;
                auto it = last_servo_status.find(pair.first);
                if (it != last_servo_status.end())
                {
                    mt.ok = true;
                    mt.mode = it->second.mode;
                    mt.fault = it->second.fault;
                    mt.temperature = it->second.temperature;
                    mt.voltage = it->second.voltage;
                    mt.velocity = it->second.velocity;
                    mt.current = std::isfinite(it->second.current) ? it->second.current : 0.0;
                    snap.m_ok++;
                }
                snap.motors.push_back(mt);
            }
            // v2+ additive: IMU (our CCW+ convention), odometry, loop health.
            snap.imu_yaw_dps = telemetry.attitude_present
                ? telemetry.imu_yaw_dps * motion_settings.imu_yaw_rate_sign
                : std::nan("");
            snap.heading_deg = telemetry.imu_heading_deg;
            snap.odo_vx = odo_twist.vx;
            snap.odo_vy = odo_twist.vy;
            snap.odo_w = odo_twist.w;
            if (loop_ms_n > 0)
            {
                snap.loop_ms = loop_ms_sum / loop_ms_n;
                snap.loop_jitter_ms = loop_ms_max - loop_ms_min;
            }
            loop_ms_sum = 0.0;
            loop_ms_min = 1e9;
            loop_ms_max = 0.0;
            loop_ms_n = 0;
            snap.imu_ok = telemetry.attitude_present;
            // MV2 executor status.
            snap.mv_seq = bridge.mv_seq();
            snap.wd_state = bridge.wd_state();
            snap.mv_kind = bridge.kind_word();
            snap.tgt_dist_mm = bridge.tgt_dist_mm();

            const std::string wire = snap.encode();
            logger.log("rframework", "sender", wire, LogLevel::INFO);
            UDP.send(wire);
            last_sender_time = current_time;
        }

        // --- Arduino Commands ---
        if (current_time - last_arduino_time >= Arduino_interval)
        {
            if (a.isConnected())
            {
                // In MV2 mode the executor's frames carry kick/dribble:
                // kick is edge-triggered (once per server pulse), dribble is
                // level-held. The v1 path keeps its historical behavior.
                const bool mv2 = bridge.active();
                const bool want_kick = mv2 ? bridge.take_kick() : cmd.kick;
                const bool want_dribble = mv2 ? bridge.dribble() : cmd.dribble;

                if (want_kick)
                {
                    a.sendCommand(kick); // Kick
                    logger.log("rframework", "arduino", "Sent kick", LogLevel::HATE);
                    cmd.kick = false;
                }
                else if (want_dribble)
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

    // --- Shutdown (SIGINT/SIGTERM) ---
    for (const auto &pair : telemetry.controllers)
    {
        pair.second->SetStop();
    }
    a.disconnect();
    logger.log("rframework", "Shutting down, motors stopped", LogLevel::HATE);

    // Stop camera thread and join
    stop_camera_thread.store(true, std::memory_order_relaxed);
    if (camera_thread.joinable())
    {
        camera_thread.join();
    }

    std::cout << "RobotFramework stopped safely\n";
    logger.closeAll();
}

// --- Signal handler for Ctrl+C ---
void signalHandler(int signum)
{
    if (signum == SIGTERM)
        std::cout << "\nSIGTERM received. Stopping safely...\n";
    else
        std::cout << "\nSIGINT received. Stopping safely...\n";
    manual_stop_flag.store(true);
}
