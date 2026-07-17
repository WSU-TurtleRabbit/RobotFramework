#pragma once

#include <cstdint>
#include <string>

#include "phx/executor.h"

namespace rf {

// Verdict for one datagram handed to accept_frame().
enum class Mv2Accept {
    Accepted,   // well-formed, addressed to us; executor updated
    Malformed,  // not a valid MV2 frame — must not touch motion state
    WrongId,    // valid frame for a different robot
};

// Pure integration layer between the superloop and the vendored MV2
// executor. No sockets, no clocks, no YAML — the caller feeds payloads,
// timestamps, odometry, and the raw gyro rate; this class owns the executor
// state, the configured gyro sign, and the kick edge detector.
//
// While an MV2 command is active, the executor's watchdog tiers
// (Fresh -> Brake -> Coast) replace the legacy 3-empty-poll UDP stop: motion
// keeps being shaped between frames and degrades gracefully when they stop.
class MotionBridge {
public:
    MotionBridge() = default;
    MotionBridge(const phx::MotionConfig& cfg, double yaw_rate_sign, int expected_id)
        : exec_(cfg), yaw_rate_sign_(yaw_rate_sign), expected_id_(expected_id) {}

    // Strict parse + robot-id check + executor accept. now_ms must share the
    // clock base later passed to tick().
    Mv2Accept accept_frame(const std::string& payload, uint64_t now_ms);

    // One control tick. odo_body = wheel-odometry body twist (from forward
    // kinematics); imu_yaw_radps = RAW gyro yaw rate in rad/s (NaN = IMU
    // unavailable). The configured yaw_rate_sign is applied here, so the
    // executor always sees CCW-positive.
    phx::ExecOutput tick(uint64_t now_ms, double dt, const phx::Twist& odo_body,
                         double imu_yaw_radps);

    // An MV2 command is live: the executor owns motion until clear().
    bool active() const { return exec_.active(); }

    // Operator STOP or fallback to the legacy v1 velocity path.
    void clear() { exec_.clear(); }

    // Kick fires once per RISING EDGE of the frame kick flag across accepted
    // frames (the server pulses it; re-sent frames with kick=1 must not
    // re-fire). Consumes the pending edge.
    bool take_kick();

    // Dribble is level-held from the active command.
    bool dribble() const;

    // Telemetry accessors (echoed in protocol v2).
    int64_t mv_seq() const { return exec_.last_seq(); }
    const char* kind_word() const { return exec_.kind_word(); }
    int wd_state() const { return static_cast<int>(last_wd_); }  // 0/1/2
    int64_t tgt_dist_mm() const;                                  // -1 = no MOVE

private:
    phx::MoveExecutor exec_;
    double yaw_rate_sign_ = 1.0;
    int expected_id_ = -1;  // -1 = accept any

    bool kick_pending_ = false;
    bool prev_frame_kick_ = false;

    phx::WatchdogTier last_wd_ = phx::WatchdogTier::Coast;
    double last_dist_m_ = -1.0;
};

}  // namespace rf
