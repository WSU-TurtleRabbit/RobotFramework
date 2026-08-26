// Versioned, asynchronous SI-unit motion telemetry.
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

#include "../Motion/adaptive.h"

namespace rf {

inline constexpr int kMotionTelemetrySchema = 1;

struct MotionTelemetrySample {
    uint64_t sequence = 0;
    int robot_id = -1;
    int skill_id = -1;
    double mono_time_s = 0.0;
    double dt_s = 0.0;
    double control_elapsed_s = 0.0;
    double command_age_s = -1.0;
    phx::Pose target_pose{};
    phx::Pose estimate_pose{};
    phx::Twist estimate_velocity_global{};
    TrajSample reference{};
    phx::Twist model_body{};
    phx::Twist adaptive_delta{};
    phx::Twist rl_proposed{};
    phx::Twist rl_applied{};
    phx::Twist safety_body{};
    std::array<double, 4> wheel_target_rev_s{};
    std::array<double, 4> wheel_measured_rev_s{};
    std::array<double, 4> wheel_position_rev{};
    std::array<double, 4> motor_current_a{};
    std::array<double, 4> motor_voltage_v{};
    std::array<double, 4> motor_temperature_c{};
    std::array<int, 4> motor_fault{};
    std::array<int, 4> motor_mode{};
    std::array<bool, 4> motor_replied{};
    std::array<double, 3> imu_rate_dps{};
    std::array<double, 3> imu_accel_raw_mps2{};
    phx::Vec2 imu_accel_body_mps2{};
    bool imu_available = false;
    double vision_age_s = 1e9;
    double vision_delay_s = 0.0;
    double vision_innovation_m = 0.0;
    double vision_heading_innovation_rad = 0.0;
    bool vision_alive = false;
    bool vision_confidence_available = false;
    double vision_confidence = 0.0;
    SurfaceContext surface{};
    RlMode rl_mode = RlMode::Off;
    std::string policy_version = "none";
    uint32_t safety_interventions = 0;
    bool estimator_applied = false;
    bool policy_evaluated = false;
    bool rl_healthy = false;
    bool rl_auto_disabled = false;
    bool motor_saturated = false;
    bool deadline_missed = false;
    bool kick_active = false;
    bool dribbler_active = false;
    bool collision = false;
};

std::string encode_motion_telemetry_header(int robot_id,
                                           const std::string& controller_abi);
std::string encode_motion_telemetry_sample(const MotionTelemetrySample& sample);

// Disk bounds for the motion log directory. All file work (rotation and
// retention deletes) happens on the logger's worker thread, never in the
// control loop. A value of 0 disables that bound.
struct MotionLoggerLimits {
    uint64_t max_file_bytes = 64ull * 1024 * 1024;    // rotate active file
    uint64_t max_total_bytes = 512ull * 1024 * 1024;  // delete oldest logs
};

class AsyncMotionLogger {
public:
    AsyncMotionLogger() = default;
    ~AsyncMotionLogger();
    AsyncMotionLogger(const AsyncMotionLogger&) = delete;
    AsyncMotionLogger& operator=(const AsyncMotionLogger&) = delete;

    bool start(const std::string& directory, int robot_id, double rate_hz,
               const std::string& controller_abi,
               const MotionLoggerLimits& limits = MotionLoggerLimits{});
    void stop();
    bool due(double mono_time_s);
    bool push(const MotionTelemetrySample& sample);

    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
    bool healthy() const { return healthy_.load(std::memory_order_relaxed); }
    uint64_t rotations() const { return rotations_.load(std::memory_order_relaxed); }
    uint64_t retention_deleted() const {
        return retention_deleted_.load(std::memory_order_relaxed);
    }
    std::string path() const;

private:
    static constexpr std::size_t kCapacity = 512;
    void worker();
    bool open_segment();
    void enforce_retention();

    std::array<MotionTelemetrySample, kCapacity> queue_{};
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t count_ = 0;
    mutable std::mutex mutex_;
    mutable std::mutex path_mutex_;
    std::condition_variable ready_;
    std::thread thread_;
    std::ofstream file_;
    std::string path_;
    std::string directory_;
    std::string session_stamp_;
    std::string controller_abi_;
    int robot_id_ = -1;
    MotionLoggerLimits limits_{};
    uint64_t segment_bytes_ = 0;
    uint64_t segment_index_ = 0;
    std::atomic<bool> stopping_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<uint64_t> rotations_{0};
    std::atomic<uint64_t> retention_deleted_{0};
    double period_s_ = 0.01;
    double next_sample_s_ = 0.0;
};

}  // namespace rf
