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

// RobotFramework superloop — TIGERs Mannheim MatchCtrl architecture.
//
// The server sends ONE MatchCtrl binary frame per robot per camera tick
// (freshest vision pose + its measured age + a skill). This loop:
//   1. accepts those frames (strict decode, seq discipline, robot id),
//   2. runs the onboard cascade at the motor control rate: delayed-vision
//      fusion, per-tick trajectory regeneration, Panthera-style control,
//      kicker/dribbler policy — Motion/match_bridge.h owns it,
//   3. reports MatchFeedback (pose/vel, kicker charge, dribbler traction,
//      barrier, battery, health) to the server at 50 Hz,
//   4. keeps the bench fallbacks: legacy v1 text velocity commands and the
//      STOP/PING opcodes, and the recoverable safety supervisor.
//
// Control rate: 250 Hz (4 ms motor tick) — the rate proven on the same
// hardware by phoenix-rf (250 Hz loop, ~0.7 ms CAN cycle, ~0.014 ms
// jitter). The estimator's time slots are one control tick each.

#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <chrono>
#include <thread>
#include <future>
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
#include "match_bridge.h"
#include "match_config_yaml.h"
#include "match_feedback.h"
#include "beacon.h"
#include "motion_logger.h"
#include "diagnostics.h"
#include <cstdlib>
#include "supervisor.h"
#include <array>
#include <cstdint>
#include <optional>
#include <limits>
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
// Monotonic ms when the camera last PRODUCED an observation (the actuator
// policy's ball-contact age). Stamped every camera iteration, so it also
// tells a dead camera thread apart from a quiet ball.
std::atomic<uint64_t> ball_obs_time_ms{0};

// --- Forward declaration for signal handler ---
void signalHandler(int signum);
// Camera snapshot logging (defined below).
void sender_log(Logger &logger, const BallObservation &obs);

// --- Thread function for camera detection ---
void CameraThread(BallDetection &detector)
{
    while (!stop_camera_thread.load(std::memory_order_relaxed))
    {
        BallObservation obs = detector.observe();
        ball_observation.store(obs, std::memory_order_relaxed);
        ball_detected.store(obs.found, std::memory_order_relaxed);
        ball_obs_time_ms.store(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count(),
            std::memory_order_relaxed);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // contact latency vs CPU
    }
}

int main(int argc, char **argv)
{
    using namespace mjbots;

    int mode = 0;

    double zero = 0.0;

    // Safety policy (Safety/supervisor.h); defaults ported from phoenix-rf,
    // overridden by config/Safety.yaml below.
    rf::SafetyConfig safety_cfg;
    double current_grace_ms = 300.0; // sustained over-current window
    double fault_grace_ms = 50.0;    // window for fault/temp/voltage trips

    // --- Interval times (ms) for periodic tasks ---
    int interval_arduino, interval_camera, interval_motor;
    // MatchFeedback cadence, ms (default 20 = 50 Hz, TIGERs-rate reporting).
    int interval_feedback = 20;

    // This robot's command-channel id (config Robot_id; -1 = accept any).
    int robot_id = -1;
    std::string robot_rid;  // asset letter (Main.yaml Robot_rid), beacon only

    // --- Logger ---
    Logger logger("logs");
    logger.initialize({"rframework"});
    logger.log("rframework", "--- ROBOTFRAMEWORK STARTING (MatchCtrl) ---", LogLevel::LOVE);

    // --- Initializing mode (SAFE, CAPPED, UNSAFE) ---
    if (argc > 1)
    {
        std::string arg = argv[1];
        if (arg == "-s" || arg == "-safe")
        {
            mode = 0;
            logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
        }
        else if (arg == "-c" || arg == "-capped")
        {
            mode = 1;
            logger.log("rframework", "Starting in CAPPED mode", LogLevel::INFO);
        }
        else if (arg == "-unsafe")
        {
            mode = 2;
            logger.log("rframework", "Starting in UNSAFE mode", LogLevel::WARN);
        }
        else
        {
            mode = 0;
            std::cerr << "Unknown flag or argument: " << argv[1] << std::endl;
            logger.log("rframework", std::string("Unknown flag or argument: ") +
                argv[1], LogLevel::WARN);
            logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
        }
    }
    else
    {
        mode = 0;
        logger.log("rframework", "Starting in SAFE mode", LogLevel::INFO);
    }

    safety_cfg.mode = (mode == 2)   ? rf::DriveMode::Unsafe
                      : (mode == 1) ? rf::DriveMode::Capped
                                    : rf::DriveMode::Safe;

    // --- Load YAML config ---
    logger.log("rframework", "Loading configs...", LogLevel::INFO);
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

        interval_arduino = interval_values["Arduino_interval"].as<int>();
        interval_camera = interval_values["Camera_interval"].as<int>();
        // The control rate: 4 ms (250 Hz) is the MatchCtrl target — proven
        // on this hardware by phoenix-rf (0.7 ms CAN cycle). 10 ms (100 Hz)
        // works but wastes the cascade.
        interval_motor = interval_values["Motor_interval"].as<int>();

        // Optional: which command-channel id this robot answers to.
        if (config["Robot_id"])
            robot_id = config["Robot_id"].as<int>();
        // Physical asset letter, announced in the discovery beacon.
        if (config["Robot_rid"])
            robot_rid = config["Robot_rid"].as<std::string>();

        logger.log("rframework", "Successfully loaded configs!", LogLevel::INFO);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading Interval config: " << e.what() << std::endl;
        logger.log("rframework", std::string("Failed to load configs: ") + (e.what()), LogLevel::WARN);
        interval_arduino = 100;
        interval_camera = 200;
        interval_motor = 4;  // 250 Hz MatchCtrl default
        logger.log("rframework", "Using fallback intervals", LogLevel::WARN);
    }

    // Grace windows are configured in milliseconds (robust to control-rate
    // changes) but the supervisor counts consecutive TICKS, like safety.rs.
    safety_cfg.current_grace_ticks = static_cast<uint32_t>(
        std::max(1.0, current_grace_ms / std::max(1, interval_motor)));
    safety_cfg.fault_grace_ticks = static_cast<uint32_t>(
        std::max(1.0, fault_grace_ms / std::max(1, interval_motor)));

    static auto CameraInterval = std::chrono::milliseconds(interval_camera);
    static auto MotorInterval = std::chrono::milliseconds(interval_motor);
    static auto Arduino_interval = std::chrono::milliseconds(interval_arduino);

    // --- MatchCtrl motion stack config (config/Motion.yaml) ---
    rf::MatchSettings motion = rf::loadMatchSettings("../config/Motion.yaml");
    interval_feedback = motion.feedback_interval_ms;
    static auto Feedback_interval = std::chrono::milliseconds(interval_feedback);
    if (motion.bridge.expected_robot_id < 0)
    {
        // Match.yaml doesn't override: the command id comes from Main.yaml.
        motion.bridge.expected_robot_id = robot_id;
    }

    // --- Initialize modules ---
    BallDetection detect; // Camera detection
    UDP UDP;              // UDP communication
    Wheel_math m;         // Wheel velocity calculations
    cmdDecoder cmd;       // Decode incoming v1 text commands
    Telemetry telemetry;  // Motor telemetry
    Arduino a;            // Arduino controller
    // The MatchCtrl cascade: skills -> estimator -> trajectory -> controller
    // -> wheels, with the safety tiers (Motion/match_bridge.h).
    rf::MatchBridge bridge(motion.bridge, m.kinematics(), motion.estimator,
                           motion.trajectory, motion.controller, motion.actuators,
                           motion.augmentation);
    rf::MatchFeedbackBuilder feedback;
    // Safety supervisor: envelope shaping, protective trips, auto-recovery.
    rf::Supervisor supervisor(safety_cfg);
    rf::AsyncMotionLogger motion_logger;
    rf::diagnostics::SnapshotSender diagnostics_sender(std::getenv("PHOENIX_TELEMETRY_SOCKET"));
    uint64_t diagnostics_cycle = 0;
    uint64_t diagnostics_kick_requested = 0, diagnostics_kick_sent = 0;
    rf::MotionLoggerLimits motion_log_limits;
    motion_log_limits.max_file_bytes = static_cast<uint64_t>(
        std::max(0.0, motion.motion_log_max_file_mb) * 1024.0 * 1024.0);
    motion_log_limits.max_total_bytes = static_cast<uint64_t>(
        std::max(0.0, motion.motion_log_max_total_mb) * 1024.0 * 1024.0);
    if (motion.motion_log_enabled &&
        !motion_logger.start(motion.motion_log_directory, robot_id,
                             motion.motion_log_rate_hz,
                             rf::ResidualPolicy::kControllerAbi,
                             motion_log_limits))
    {
        logger.log("rframework", "Versioned motion logger failed to start",
                   LogLevel::WARN);
    }
    else if (motion.motion_log_enabled)
    {
        logger.log("rframework", std::string("Versioned motion log: ") +
            motion_logger.path(), LogLevel::INFO);
    }
    if (motion.surface_profile_load && motion.augmentation.estimator.enabled)
    {
        const bool loaded = bridge.augmentor().estimator().load_profile(
            motion.surface_profile_path, robot_id, motion.surface_id);
        logger.log("rframework",
            loaded ? "Loaded compatible surface profile"
                   : "No compatible surface profile; conservative defaults active",
            loaded ? LogLevel::INFO : LogLevel::WARN);
    }
    std::future<bool> profile_save;
    double last_profile_save_s = -1e9;
    uint64_t last_profile_saved_samples = 0;
    uint64_t motion_log_sequence = 0;

    std::string msg;                    // Incoming UDP datagram
    std::vector<double> wheel_velocity; // Calculated wheel velocities
    std::map<int, double> velocity_map; // Motor ID → velocity map

    // The freshest bridge tick result (feedback + actuators consume it).
    rf::BridgeTick bt;
    // Latest measured motor velocities, shared by the odometry path and the
    // lower-rate motion-identification record.
    std::array<double, 4> measured_rev_s = {0.0, 0.0, 0.0, 0.0};
    // Legacy v1 bookkeeping.
    BodyTwist last_cmd_twist;
    // Monotonic ms of the last valid drive command (MatchCtrl or v1) for
    // the v1 staleness gate and the supervisor's auto-recovery.
    std::optional<uint64_t> last_valid_cmd_ms;
    rf::RuntimeFeedback runtime_feedback;

    // Feedback health snapshot (filled per cycle).
    rf::FeedbackHealth health;
    health.robot_id = robot_id;
    health.hardware_id = motion.hardware_id;
    health.battery_empty_v = motion.battery_empty_v;
    health.battery_full_v = motion.battery_full_v;
    health.kicker_max_v = motion.kicker_max_v;
    health.kicker_recharge_s = motion.kicker_recharge_s;
    health.motion_profile_id = motion.profile_id;
    health.adaptive_enabled = motion.augmentation.estimator.enabled;
    logger.log("rframework", std::string("Motion profile: ") +
        (motion.profile_name.empty() ? "(unnamed)" : motion.profile_name) +
        " id " + std::to_string(motion.profile_id) +
        ", rl " + rf::rl_mode_name(motion.augmentation.rl.mode), LogLevel::INFO);

    // --- Discovery beacon (config/Network.yaml beacon.*) ---
    bool beacon_enabled = true;
    int beacon_port = rf::kBeaconPort;
    double beacon_interval_s = rf::kBeaconIntervalS;
    try
    {
        YAML::Node net = YAML::LoadFile("../config/Network.yaml");
        if (net["beacon"])
        {
            if (net["beacon"]["enabled"]) beacon_enabled = net["beacon"]["enabled"].as<bool>();
            if (net["beacon"]["port"]) beacon_port = net["beacon"]["port"].as<int>();
            if (net["beacon"]["interval_s"]) beacon_interval_s = net["beacon"]["interval_s"].as<double>();
        }
    }
    catch (const std::exception &)
    {
        // Network.yaml absent/unreadable: UDP already fell back to defaults.
    }
    rf::BeaconSender beacon(beacon_port);
    rf::BeaconInfo beacon_info;
    beacon_info.robot_id = robot_id;
    beacon_info.hardware_id = motion.hardware_id;
    beacon_info.rid = robot_rid;
    {
        char host[256] = {0};
        if (gethostname(host, sizeof(host) - 1) == 0) beacon_info.hostname = host;
    }
    beacon_info.command_port = UDP.getRecieverPort();
    beacon_info.adaptive = motion.augmentation.estimator.enabled;
    const auto beacon_interval = std::chrono::milliseconds(
        static_cast<int>(std::max(0.2, beacon_interval_s) * 1000.0));
    const auto process_start = std::chrono::steady_clock::now();
    uint16_t last_feedback_features = 0;
    if (beacon_enabled)
    {
        logger.log("rframework", std::string("Discovery beacon on UDP ") +
            std::to_string(beacon_port) + (beacon.ok() ? "" : " (socket unavailable)"),
            beacon.ok() ? LogLevel::INFO : LogLevel::WARN);
    }

    // --- Initialize Arduino ---
    logger.log("rframework", "arduino", "Searching for Arduino...", LogLevel::INFO);
    a.findArduino();
    logger.log("rframework", "arduino", "Connecting to Arduino port...", LogLevel::INFO);
    a.connect(a.getPort());

    // Only accept v1 commands addressed to this robot (-1 = accept any).
    cmd.expected_id = robot_id;

    // --- Start camera detection thread ---
    std::thread camera_thread;
    if (detect.open_cam() > 0)
    {
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

    logger.log("rframework", std::string("Sending port at: ") +
        std::to_string(UDP.getSenderPort()), LogLevel::LOVE);
    logger.log("rframework", std::string("Recieving port at: ") +
        std::to_string(UDP.getRecieverPort()), LogLevel::LOVE);

    // --- Main control loop ---
    logger.log("rframework", "Entering main control loop (MatchCtrl)", LogLevel::LOVE);
    while (!manual_stop_flag.load(std::memory_order_relaxed))
    {
        auto current_time = std::chrono::steady_clock::now();

        static auto last_motor_time = current_time;
        static auto last_camera_time = current_time;
        static auto last_feedback_time = current_time;
        static auto last_motor_log_time = current_time;
        static auto last_arduino_time = current_time;
        static auto last_beacon_time = current_time - beacon_interval;

        // --- UDP receive (every superloop iteration): MatchCtrl binary
        // first, legacy v1 text as the bench fallback. The socket drain
        // keeps only the newest datagram (TIGERs' 1-deep latest-wins queue
        // semantics; seq discipline is inside the bridge).
        msg = UDP.receive();
        if (msg != "TIMEOUT")
        {
            const double now_s = std::chrono::duration<double>(
                current_time.time_since_epoch()).count();
            rf::MatchAccept acc = rf::MatchAccept::Malformed;
            // Phoenix MotionParams (0x07): the server pushing the onboard
            // half of the fleet motion profile. Applied live through the
            // bridge; acknowledged by the profile id in the next feedback.
            if (const auto mp = rf::decode_motion_params(msg))
            {
                if (motion.bridge.expected_robot_id >= 0 &&
                    mp->robot_id != motion.bridge.expected_robot_id)
                {
                    logger.log("rframework", "reciever", "MotionParams for another robot ignored", LogLevel::WARN);
                }
                else
                {
                    motion.profile_id = bridge.apply_motion_params(*mp, motion.profile_id);
                    health.motion_profile_id = motion.profile_id;
                    logger.log("rframework", "reciever",
                        std::string("MotionParams applied: profile ") +
                        std::to_string(motion.profile_id) + ", rl " +
                        rf::rl_mode_name(bridge.augmentor().rl_mode()) +
                        (mp->reload_policy ? ", policy reloaded" : "") +
                        ", " + std::to_string(mp->count) + " params",
                        LogLevel::INFO);
                }
                // Handled here; StaleSeq is the switch's silent no-op so the
                // rest of the superloop iteration (motor tick, feedback) runs.
                acc = rf::MatchAccept::StaleSeq;
            }
            else if (motion.enabled)
            {
                acc = bridge.accept(msg, now_s);
            }
            switch (acc)
            {
            case rf::MatchAccept::Accepted:
                last_valid_cmd_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time.time_since_epoch()).count();
                break;
            case rf::MatchAccept::WrongId:
                logger.log("rframework", "reciever", "MatchCtrl for another robot ignored", LogLevel::WARN);
                break;
            case rf::MatchAccept::StaleSeq:
                break;  // duplicate/reordered: dropped silently by design
            case rf::MatchAccept::Malformed:
            default:
                // Not a binary MatchCtrl frame: the legacy text channel.
                switch (cmd.decode_cmd(msg))
                {
                case CmdType::Velocity:
                {
                    // A v1 velocity command takes the robot back to direct
                    // (legacy) control; the MatchCtrl bridge stands down.
                    if (bridge.active())
                    {
                        bridge.clear();
                        logger.log("rframework", "reciever", "v1 command - leaving MatchCtrl mode", LogLevel::INFO);
                    }
                    const BodyTwist shaped = supervisor.shape_twist(
                        BodyTwist{cmd.velocity_x, cmd.velocity_y, cmd.velocity_w});
                    wheel_velocity = m.calculate(shaped.vx, shaped.vy, shaped.w);
                    velocity_map = {
                        {1, wheel_velocity[0]},
                        {2, wheel_velocity[1]},
                        {3, wheel_velocity[2]},
                        {4, wheel_velocity[3]}};
                    last_cmd_twist = shaped;
                    last_valid_cmd_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        current_time.time_since_epoch()).count();
                    break;
                }
                case CmdType::Stop:
                    // Operator STOP: safe-stop the motors and clear every
                    // latch — the daemon KEEPS RUNNING.
                    logger.log("rframework", "reciever", "UDP STOP - safe stop, latches cleared", LogLevel::HATE);
                    velocity_map = {{1, zero}, {2, zero}, {3, zero}, {4, zero}};
                    last_cmd_twist = BodyTwist{};
                    last_valid_cmd_ms.reset();
                    bridge.clear();
                    supervisor.clear();
                    for (const auto &pair : telemetry.controllers)
                    {
                        pair.second->SetStop();
                    }
                    break;
                case CmdType::Ping:
                    logger.log("rframework", "reciever", "PING received", LogLevel::INFO);
                    break;
                case CmdType::Calibrate:
                    logger.log("rframework", "reciever", "CALIBRATE received (not supported, ignored)", LogLevel::WARN);
                    break;
                case CmdType::WrongId:
                case CmdType::Malformed:
                default:
                    logger.log("rframework", "reciever",
                        std::string("Unrecognized datagram rejected"), LogLevel::WARN);
                    break;
                }
                break;
            }
        }

        // --- Camera ball observation snapshot ---
        if (current_time - last_camera_time >= CameraInterval)
        {
            BallObservation obs = ball_observation.load(std::memory_order_relaxed);
            sender_log(logger, obs);
            last_camera_time = current_time;
        }

        // --- Motor control tick: the MatchCtrl cascade ---
        if (current_time - last_motor_time >= MotorInterval)
        {
            const auto motor_tick_started = std::chrono::steady_clock::now();
            ++diagnostics_cycle;
            // Measured wheel velocities (rev/s, motor id i -> index i-1) from
            // the PREVIOUS cycle's replies — the estimator's odometry input.
            static auto last_motor_tick_wall = current_time;
            const double motor_dt =
                std::chrono::duration<double>(current_time - last_motor_tick_wall).count();
            last_motor_tick_wall = current_time;

            const double now_s = std::chrono::duration<double>(
                current_time.time_since_epoch()).count();
            const uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time.time_since_epoch()).count();

            // Wheel odometry (FK) from the previous cycle's measured
            // velocities.
            const BodyTwist odo = m.kinematics().forward(measured_rev_s);
            runtime_feedback.wheel_odo_body = phx::Twist{odo.vx, odo.vy, odo.w};
            const std::array<double, 3> raw_accel = {
                telemetry.imu_accel_x_mps2,
                telemetry.imu_accel_y_mps2,
                telemetry.imu_accel_z_mps2,
            };
            const bool accel_axes_valid =
                motion.imu_accel_forward_axis >= 0 &&
                motion.imu_accel_forward_axis < 3 &&
                motion.imu_accel_lateral_axis >= 0 &&
                motion.imu_accel_lateral_axis < 3 &&
                telemetry.attitude_present;
            runtime_feedback.imu_accel_available = accel_axes_valid;
            if (accel_axes_valid)
            {
                runtime_feedback.imu_accel_body_mps2 = {
                    raw_accel[motion.imu_accel_forward_axis] *
                        motion.imu_accel_forward_sign,
                    raw_accel[motion.imu_accel_lateral_axis] *
                        motion.imu_accel_lateral_sign,
                };
                runtime_feedback.imu_accel_z_mps2 = telemetry.imu_accel_z_mps2;
            }
            runtime_feedback.collision = accel_axes_valid &&
                runtime_feedback.imu_accel_body_mps2.norm() >
                    motion.augmentation.estimator.disturbance_accel_mps2;

            bool energize = true;

            if (bridge.active())
            {
                // The MatchCtrl cascade: delayed fusion + trajectory +
                // control + actuator policy, one call per tick.
                const std::array<double, 3> imu_rate_dps = {
                    telemetry.imu_roll_dps,
                    telemetry.imu_pitch_dps,
                    telemetry.imu_yaw_dps,
                };
                const int yaw_axis = std::clamp(motion.imu_yaw_rate_axis, 0, 2);
                const double gyro_radps = telemetry.attitude_present
                    ? imu_rate_dps[yaw_axis] * (M_PI / 180.0)
                          * motion.imu_yaw_rate_sign
                    : std::nan("");
                const uint64_t obs_ms = ball_obs_time_ms.load(std::memory_order_relaxed);
                const BallObservation obs = ball_observation.load(std::memory_order_relaxed);
                rf::BallContactObs ball;
                ball.found = obs.found;
                ball.bearing = obs.bearing;
                ball.radius = obs.radius;
                ball.confidence = obs.confidence;
                ball.age_s = obs_ms > 0
                    ? std::max(0.0, (static_cast<double>(now_ms) - static_cast<double>(obs_ms)) / 1000.0)
                    : 1e9;

                bt = bridge.tick(now_s, motor_dt, phx::Twist{odo.vx, odo.vy, odo.w},
                                 gyro_radps, ball, runtime_feedback);
                velocity_map = {
                    {1, bt.ctrl.wheel_rev_s[0]},
                    {2, bt.ctrl.wheel_rev_s[1]},
                    {3, bt.ctrl.wheel_rev_s[2]},
                    {4, bt.ctrl.wheel_rev_s[3]}};
                energize = bt.ctrl.energize;

                // Kicker fire edge: send IMMEDIATELY (never on the slow
                // Arduino timer), and consume it so it fires exactly once.
                if (bt.act.fire_pulse_ms.has_value()) ++diagnostics_kick_requested;
                if (bt.act.fire_pulse_ms.has_value() && a.isConnected())
                {
                    const int pulse = static_cast<int>(
                        std::clamp(*bt.act.fire_pulse_ms, 1.0, 255.0));
                    const char cmd_bytes[2] = {'k', static_cast<char>(pulse)};
                    if (a.sendBytes(cmd_bytes, 2))
                    {
                        ++diagnostics_kick_sent;
                        logger.log("rframework", "arduino",
                            std::string("KICK fired, pulse ") + std::to_string(pulse) + " ms",
                            LogLevel::HATE);
                    }
                    bt.act.fire_pulse_ms.reset();
                }
            }

            // Safety gate: a latched trip coasts the motors; on the legacy
            // v1 path a stale command holds zero (the MatchCtrl bridge has
            // its own richer emergency tiers).
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

            // Optional model feedforward torque (disabled until identified).
            std::map<int, double> ff_torque;
            const std::map<int, double>* ff_ptr = nullptr;
            if (bridge.active() && motion.controller.model_ff_enabled)
            {
                for (int id = 1; id <= 4; ++id)
                    ff_torque[id] = bt.ctrl.wheel_ff_torque_nm[id - 1];
                ff_ptr = &ff_torque;
            }

            auto servo_status = telemetry.cycle(velocity_map, energize, ff_ptr);
            const uint64_t diagnostics_acquired_us = diagnostics_sender.enabled()
                ? rf::diagnostics::monotonic_us() : 0;

            // Stash measured velocities for the next tick's odometry.
            measured_rev_s = {0.0, 0.0, 0.0, 0.0};
            runtime_feedback.wheel_measured_rev_s.fill(0.0);
            runtime_feedback.wheel_current_a.fill(0.0);
            runtime_feedback.wheel_temperature_c.fill(0.0);
            runtime_feedback.wheel_fault.fill(0);
            runtime_feedback.wheel_replied.fill(false);
            for (const auto &pair : servo_status)
            {
                const int id = pair.first;
                if (id >= 1 && id <= 4 && std::isfinite(pair.second.velocity))
                {
                    measured_rev_s[id - 1] = pair.second.velocity;
                    runtime_feedback.wheel_measured_rev_s[id - 1] =
                        pair.second.velocity;
                }
                if (id >= 1 && id <= 4)
                {
                    runtime_feedback.wheel_replied[id - 1] = true;
                    runtime_feedback.wheel_current_a[id - 1] = pair.second.current;
                    runtime_feedback.wheel_temperature_c[id - 1] =
                        pair.second.temperature;
                    runtime_feedback.wheel_fault[id - 1] = pair.second.fault;
                }
            }

            // Bus voltage (average of replying motors) + sampled motor
            // logging. Formatting four map records at the 250 Hz control
            // rate created avoidable disk/CPU jitter; 20 Hz retains health
            // diagnostics while the control and safety reads remain 250 Hz.
            float voltage_sum = 0.0f;
            int voltage_n = 0;
            const bool log_motor_sample =
                current_time - last_motor_log_time >= std::chrono::milliseconds(50);
            for (const auto &pair : servo_status)
            {
                const auto &r = pair.second;
                if (std::isfinite(r.voltage) && r.voltage > 0.0)
                {
                    voltage_sum += r.voltage;
                    voltage_n++;
                }
                if (log_motor_sample)
                {
                    std::string sub =
                        std::string("motor-") + std::to_string(pair.first);
                    std::map<std::string, double> data = {
                        {"temperature", r.temperature},
                        {"voltage", r.voltage},
                        {"velocity", r.velocity},
                        {"current", r.current},
                        {"mode", static_cast<double>(r.mode)},
                        {"fault", static_cast<double>(r.fault)}};
                    logger.log("rframework", sub, data, "", LogLevel::INFO);
                }
            }
            if (log_motor_sample)
            {
                if (telemetry.attitude_present)
                {
                    const std::array<double, 3> imu_rate_dps = {
                        telemetry.imu_roll_dps,
                        telemetry.imu_pitch_dps,
                        telemetry.imu_yaw_dps,
                    };
                    const int yaw_axis =
                        std::clamp(motion.imu_yaw_rate_axis, 0, 2);
                    const double selected_radps =
                        imu_rate_dps[yaw_axis] * (M_PI / 180.0)
                        * motion.imu_yaw_rate_sign;
                    logger.log("rframework", "imu",
                        {
                            {"pitch_dps", telemetry.imu_pitch_dps},
                            {"roll_dps", telemetry.imu_roll_dps},
                            {"selected_radps", selected_radps},
                            {"yaw_dps", telemetry.imu_yaw_dps},
                        },
                        "", LogLevel::INFO);
                }
                last_motor_log_time = current_time;
            }
            const double avg_voltage =
                voltage_n > 0 ? static_cast<double>(voltage_sum) / voltage_n : 0.0;
            runtime_feedback.bus_voltage_v = avg_voltage;
            runtime_feedback.motor_saturated = false;
            for (int i = 0; i < 4; ++i)
            {
                runtime_feedback.motor_saturated =
                    runtime_feedback.motor_saturated ||
                    std::fabs(bt.ctrl.wheel_rev_s[i]) >=
                        0.98 * motion.controller.wheel_max_rev_s ||
                    (runtime_feedback.wheel_fault[i] >= 96);
            }
            runtime_feedback.kick_active = bt.act.fire_pulse_ms.has_value();
            runtime_feedback.dribbler_active = bt.act.dribbler_speed > 0.01;

            // --- Safety supervisor: per-motor trips with grace counts ---
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
            for (const rf::StopReason &r : supervisor.observe(motor_obs, avg_voltage))
            {
                logger.log("rframework",
                    std::string("SAFETY TRIP: ") + r.word() +
                    (r.motor ? (std::string(" motor ") + std::to_string(r.motor)) : std::string()),
                    LogLevel::CRIT);
            }

            // Auto-recovery: healthy for recoveryS seconds with the operator
            // commanding zero (or gone silent) releases the latch.
            const bool cmd_stale = !last_valid_cmd_ms ||
                (now_ms - *last_valid_cmd_ms > supervisor.config().command_timeout_ms);
            const bool motion_idle = bridge.active()
                ? bt.emergency
                : (cmd_stale || (std::abs(last_cmd_twist.vx) + std::abs(last_cmd_twist.vy) +
                                     std::abs(last_cmd_twist.w) < 0.02 && !cmd.kick));
            if (auto released = supervisor.try_recover(now_ms, motion_idle))
            {
                for (const rf::StopReason &r : *released)
                {
                    logger.log("rframework",
                        std::string("SAFETY RECOVERED: ") + r.word(), LogLevel::DONE);
                }
            }

            const double control_elapsed_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - motor_tick_started).count();
            runtime_feedback.control_elapsed_s = control_elapsed_s;
            runtime_feedback.control_deadline_missed =
                control_elapsed_s > std::chrono::duration<double>(MotorInterval).count();

            // Independent of disk logger health. Fixed-size stack snapshot, one
            // nonblocking local datagram; receiver absence only drops diagnostics.
            if (diagnostics_sender.due(diagnostics_acquired_us))
            {
                rf::diagnostics::Snapshot snapshot;
                snapshot.robot_id = robot_id;
                snapshot.cycle = diagnostics_cycle;
                snapshot.acquired_us = diagnostics_acquired_us;
                for (int index = 0; index < 4; ++index)
                {
                    auto& motor = snapshot.motors[index];
                    motor.requested = bridge.active() ? bt.ctrl.wheel_rev_s[index]
                                                     : rf::diagnostics::kMissing;
                    const auto command = velocity_map.find(index + 1);
                    motor.energized = energize;
                    if (energize && command != velocity_map.end()) motor.sent = command->second;
                    const auto reply = servo_status.find(index + 1);
                    if (reply == servo_status.end()) continue;
                    motor.replied = true;
                    motor.velocity = reply->second.velocity;
                    motor.position = reply->second.position;
                    motor.current = reply->second.current;
                    motor.voltage = reply->second.voltage;
                    motor.temperature = reply->second.temperature;
                    motor.mode = reply->second.mode;
                    motor.fault = reply->second.fault;
                }
                snapshot.motion = {
                    bridge.setpoint().target.pos.x, bridge.setpoint().target.pos.y,
                    bridge.setpoint().target.heading,
                    bt.est.pose.pos.x, bt.est.pose.pos.y, bt.est.pose.heading,
                    bt.est.vel_global.x, bt.est.vel_global.y, bt.est.omega,
                    bt.ref.pose.pos.x, bt.ref.pose.pos.y, bt.ref.pose.heading,
                    bt.ref.vel.x, bt.ref.vel.y, bt.ref.omega,
                    bt.ctrl.cmd_body.lin.x, bt.ctrl.cmd_body.lin.y, bt.ctrl.cmd_body.ang,
                    telemetry.imu_roll_dps, telemetry.imu_pitch_dps, telemetry.imu_yaw_dps,
                    telemetry.imu_accel_x_mps2, telemetry.imu_accel_y_mps2,
                    telemetry.imu_accel_z_mps2,
                    bt.last_cmd_age_s, control_elapsed_s, motor_dt,
                    bt.est.vision_age_s, bt.est.vision_delay_s, bt.est.vision_innovation_m,
                    bt.est.vision_heading_innovation_rad, bt.est.vision_confidence,
                    static_cast<double>(diagnostics_acquired_us) / 1e6 - now_s,
                };
                snapshot.flags = (telemetry.attitude_present ? 1u : 0u)
                    | (bt.est.vision_alive ? 2u : 0u)
                    | (bt.est.vision_confidence_available ? 4u : 0u)
                    | (supervisor.estop() ? 8u : 0u)
                    | (runtime_feedback.control_deadline_missed ? 16u : 0u)
                    | (a.isConnected() ? 32u : 0u)
                    | (runtime_feedback.dribbler_active ? 64u : 0u)
                    | (bridge.active() ? 128u : 0u);
                snapshot.kick_requested = diagnostics_kick_requested;
                snapshot.kick_sent = diagnostics_kick_sent;
                diagnostics_sender.offer(snapshot);
            }

            if (motion_logger.due(now_s))
            {
                rf::MotionTelemetrySample sample;
                sample.sequence = motion_log_sequence++;
                sample.robot_id = robot_id;
                sample.skill_id = bt.skill_id;
                sample.mono_time_s = now_s;
                sample.dt_s = motor_dt;
                sample.control_elapsed_s = control_elapsed_s;
                sample.command_age_s = bt.last_cmd_age_s;
                sample.target_pose = bridge.setpoint().target;
                sample.estimate_pose = bt.est.pose;
                sample.estimate_velocity_global =
                    phx::Twist{bt.est.vel_global, bt.est.omega};
                sample.reference = bt.ref;
                sample.model_body = bt.ctrl.model_body;
                sample.adaptive_delta = bt.ctrl.adaptive_delta;
                sample.rl_proposed = bt.ctrl.rl_proposed;
                sample.rl_applied = bt.ctrl.rl_applied;
                sample.safety_body = bt.ctrl.cmd_body;
                sample.wheel_target_rev_s = bt.ctrl.wheel_rev_s;
                sample.wheel_measured_rev_s = measured_rev_s;
                const double unavailable = std::numeric_limits<double>::quiet_NaN();
                sample.wheel_position_rev.fill(unavailable);
                sample.motor_current_a.fill(unavailable);
                sample.motor_voltage_v.fill(unavailable);
                sample.motor_temperature_c.fill(unavailable);
                for (const auto &pair : servo_status)
                {
                    const int index = pair.first - 1;
                    if (index < 0 || index >= 4) continue;
                    sample.motor_replied[index] = true;
                    sample.wheel_position_rev[index] = pair.second.position;
                    sample.motor_current_a[index] = pair.second.current;
                    sample.motor_voltage_v[index] = pair.second.voltage;
                    sample.motor_temperature_c[index] = pair.second.temperature;
                    sample.motor_fault[index] = pair.second.fault;
                    sample.motor_mode[index] = pair.second.mode;
                }
                sample.imu_rate_dps = {telemetry.imu_roll_dps,
                                       telemetry.imu_pitch_dps,
                                       telemetry.imu_yaw_dps};
                sample.imu_accel_raw_mps2 = {telemetry.imu_accel_x_mps2,
                                              telemetry.imu_accel_y_mps2,
                                              telemetry.imu_accel_z_mps2};
                sample.imu_accel_body_mps2 = runtime_feedback.imu_accel_body_mps2;
                sample.imu_available = runtime_feedback.imu_accel_available;
                sample.vision_age_s = bt.est.vision_age_s;
                sample.vision_delay_s = bt.est.vision_delay_s;
                sample.vision_innovation_m = bt.est.vision_innovation_m;
                sample.vision_heading_innovation_rad =
                    bt.est.vision_heading_innovation_rad;
                sample.vision_alive = bt.est.vision_alive;
                sample.vision_confidence_available =
                    bt.est.vision_confidence_available;
                sample.vision_confidence = bt.est.vision_confidence;
                sample.surface = bt.augmentation.surface;
                sample.rl_mode = bridge.augmentor().rl_mode();
                sample.policy_version = bridge.augmentor().policy().version();
                sample.safety_interventions = bt.ctrl.safety_interventions;
                sample.estimator_applied = bt.augmentation.adaptive_applied;
                sample.policy_evaluated = bt.augmentation.policy_evaluated;
                sample.rl_healthy = bt.augmentation.rl_healthy;
                sample.rl_auto_disabled = bt.augmentation.rl_auto_disabled;
                sample.motor_saturated = runtime_feedback.motor_saturated;
                sample.deadline_missed = runtime_feedback.control_deadline_missed;
                sample.kick_active = runtime_feedback.kick_active;
                sample.dribbler_active = runtime_feedback.dribbler_active;
                sample.collision = runtime_feedback.collision;
                motion_logger.push(sample);
            }

            // Save only a better-covered healthy surface profile, and do the
            // disk operation asynchronously so the 250 Hz loop never waits.
            const rf::SurfaceContext surface =
                bridge.augmentor().estimator().context(now_s);
            const bool save_due = motion.surface_profile_save &&
                motion.augmentation.estimator.enabled &&
                now_s - last_profile_save_s >=
                    std::max(1.0, motion.surface_profile_save_interval_s) &&
                surface.confidence >=
                    motion.augmentation.estimator.confidence_threshold &&
                surface.accepted_samples > last_profile_saved_samples;
            const bool saver_ready = !profile_save.valid() ||
                profile_save.wait_for(std::chrono::seconds(0)) ==
                    std::future_status::ready;
            if (save_due && saver_ready)
            {
                if (profile_save.valid()) profile_save.get();
                const rf::SurfaceEstimator snapshot =
                    bridge.augmentor().estimator();
                const std::string profile_path = motion.surface_profile_path;
                const std::string surface_id = motion.surface_id;
                profile_save = std::async(std::launch::async,
                    [snapshot, profile_path, robot_id, surface_id]() mutable {
                        return snapshot.save_profile(profile_path, robot_id,
                                                     surface_id);
                    });
                last_profile_save_s = now_s;
                last_profile_saved_samples = surface.accepted_samples;
            }

            // Feedback health snapshot for the next send.
            health.battery_v = avg_voltage;
            health.estop = supervisor.estop();
            health.arduino_connected = a.isConnected();
            health.camera_running = camera_thread.joinable();
            if (health.robot_id < 0)
                health.robot_id = bridge.last_robot_id();

            last_motor_time = current_time;
        }

        // --- Discovery beacon (1 Hz default): who we are, for the server's
        // "Discovered" list. Broadcast, no peer needed.
        if (beacon_enabled && current_time - last_beacon_time >= beacon_interval)
        {
            last_beacon_time = current_time;
            beacon_info.features = last_feedback_features;
            beacon_info.battery_v = health.battery_v;
            beacon_info.profile_id = health.motion_profile_id;
            beacon_info.rl_mode = static_cast<int>(bridge.augmentor().rl_mode()) & 0x3;
            beacon_info.uptime_s = std::chrono::duration<double>(current_time - process_start).count();
            beacon.send(rf::beacon_json(beacon_info));
        }

        // --- MatchFeedback to the server (50 Hz default) ---
        if (current_time - last_feedback_time >= Feedback_interval)
        {
            const double now_s = std::chrono::duration<double>(
                current_time.time_since_epoch()).count();
            const BallObservation obs = ball_observation.load(std::memory_order_relaxed);
            rf::BallContactObs ball;
            ball.found = obs.found;
            ball.bearing = obs.bearing;
            ball.radius = obs.radius;
            ball.confidence = obs.confidence;
            const uint64_t obs_ms = ball_obs_time_ms.load(std::memory_order_relaxed);
            const uint64_t now_ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time.time_since_epoch()).count();
            ball.age_s = obs_ms > 0
                ? std::max(0.0, (static_cast<double>(now_ms2) - static_cast<double>(obs_ms)) / 1000.0)
                : 1e9;

            const rf::MatchFeedback fb = feedback.build(bridge, bt, health, ball, now_s);
            last_feedback_features = fb.features;
            const auto bytes = rf::encode_match_feedback(fb);
            UDP.send(std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()));
            // Compact 50 Hz motion-identification record: commanded and
            // measured wheels plus the estimator/reference states. This is
            // sufficient to identify wheel imbalance without console output
            // or control-rate logging.
            logger.log("motion", std::map<std::string, double>{
                {"cmd_body_vx", bt.ctrl.cmd_body.lin.x},
                {"cmd_body_vy", bt.ctrl.cmd_body.lin.y},
                {"cmd_body_w", bt.ctrl.cmd_body.ang},
                {"cmd_w1", bt.ctrl.wheel_rev_s[0]},
                {"cmd_w2", bt.ctrl.wheel_rev_s[1]},
                {"cmd_w3", bt.ctrl.wheel_rev_s[2]},
                {"cmd_w4", bt.ctrl.wheel_rev_s[3]},
                {"meas_w1", measured_rev_s[0]},
                {"meas_w2", measured_rev_s[1]},
                {"meas_w3", measured_rev_s[2]},
                {"meas_w4", measured_rev_s[3]},
                {"est_x", bt.est.pose.pos.x},
                {"est_y", bt.est.pose.pos.y},
                {"est_theta", bt.est.pose.heading},
                {"est_vx", bt.est.vel_global.x},
                {"est_vy", bt.est.vel_global.y},
                {"imu_heading_deg", telemetry.imu_heading_deg},
                {"imu_pitch_dps", telemetry.imu_pitch_dps},
                {"imu_roll_dps", telemetry.imu_roll_dps},
                {"imu_yaw_dps", telemetry.imu_yaw_dps},
                {"ref_x", bt.ref.pose.pos.x},
                {"ref_y", bt.ref.pose.pos.y},
                {"ref_vx", bt.ref.vel.x},
                {"ref_vy", bt.ref.vel.y},
            });
            last_feedback_time = current_time;
        }

        // --- Arduino commands ---
        if (current_time - last_arduino_time >= Arduino_interval)
        {
            if (a.isConnected())
            {
                if (bridge.active())
                {
                    // Dribbler level from the actuator policy (speed -> ESC
                    // microseconds). Sent on change; the fire edge goes
                    // IMMEDIATELY (see below), not on this slow timer.
                    static int last_dribble_us = -1;
                    const int us = bt.act.dribbler_us;
                    if (us != last_dribble_us)
                    {
                        if (us > 0)
                        {
                            const char cmd_bytes[2] = {'d', static_cast<char>(us)};
                            a.sendBytes(cmd_bytes, 2);
                        }
                        else
                        {
                            a.sendCommand('S');
                        }
                        last_dribble_us = us;
                    }
                }
                else
                {
                    // Legacy v1 actuators.
                    if (cmd.kick)
                    {
                        a.sendCommand('K');
                        logger.log("rframework", "arduino", "Sent kick", LogLevel::HATE);
                        cmd.kick = false;
                    }
                    else if (cmd.dribble)
                    {
                        a.sendCommand('D');
                    }
                    else
                    {
                        a.sendCommand('S');
                    }
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
    motion_logger.stop();
    if (profile_save.valid()) profile_save.get();
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

// Camera snapshot logging (kept out of the hot loop).
void sender_log(Logger &logger, const BallObservation &obs)
{
    logger.log("rframework", "camball",
        std::string("ball_detected=") + (obs.found ? "true" : "false") +
        " px=" + std::to_string(obs.px) +
        " py=" + std::to_string(obs.py) +
        " r="  + std::to_string(obs.radius) +
        " b="  + std::to_string(obs.bearing) +
        " c="  + std::to_string(obs.confidence),
        LogLevel::INFO);
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
