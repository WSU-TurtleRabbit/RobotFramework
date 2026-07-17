// Telemetry protocol v2 encoder — pure (no sockets, no hardware), ported
// from the field-proven Rust implementation in phoenix-rf
// (crates/protocol/src/wire.rs TelemetrySnapshot::encode). The key set and
// order are PINNED across repos: phoenix-core's parser requires exactly
// these keys, and the `mv=2` flag is what gates the server's MV2 dispatch —
// never remove it while the executor exists.
//
// The v1 keys (state..ts_ms) come FIRST so pre-v2 dashboards still parse;
// everything after is additive (unknown keys are ignored by design).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rf {

// One motor's health for telemetry.
struct MotorTelem {
    int id = 0;
    bool ok = false;
    int mode = -1;
    int fault = -1;
    double temperature = 0.0;
    double voltage = 0.0;
    double velocity = 0.0;
    double current = 0.0;
    // Calibration status: "ok", "uncalibrated", "suspect", "cal_failed", or
    // "unknown" (this framework has no commissioner yet, so "unknown").
    std::string cal = "unknown";
};

// Everything the robot reports each telemetry tick. encode() renders the
// exact v2 wire string.
struct TelemetrySnapshot {
    // --- v1 core ---
    std::string state = "active";  // "active" | "estop" | "shutdown"
    double voltage = 0.0;          // average bus voltage over replying motors
    bool ball_found = false;
    double ball_px = 0.0;
    double ball_py = 0.0;
    double ball_radius = 0.0;
    double ball_bearing = 0.0;
    double ball_confidence = 0.0;
    uint64_t robot_ts_ms = 0;
    // --- v2 identity & link ---
    std::string rid = "?";  // physical robot identity letter, e.g. "B"
    uint32_t seq = 0;
    uint64_t up_ms = 0;
    std::string ifip = "0.0.0.0";
    double vmin = 0.0;  // lowest single-motor bus voltage this tick
    uint32_t m_ok = 0;
    uint32_t m_exp = 0;
    int64_t cmd_age_ms = -1;
    uint64_t cmd_rx = 0;
    int32_t cmd_last_id = -1;
    bool arduino_connected = false;
    bool camera_running = false;
    bool estop = false;
    uint64_t tx_err = 0;
    double cycle_ms = 0.0;  // duration of the CAN command/telemetry cycle
    std::vector<MotorTelem> motors;
    // --- v2+ additive ---
    double imu_yaw_dps = 0.0;   // gyro yaw rate, deg/s, CCW+; NaN = IMU down
    double heading_deg = 0.0;   // attitude yaw, deg; NaN = unavailable
    double odo_vx = 0.0;        // wheel-odometry body twist (FK)
    double odo_vy = 0.0;
    double odo_w = 0.0;
    double loop_ms = 0.0;         // achieved control-loop period
    double loop_jitter_ms = 0.0;  // its jitter (max-min) over the window
    bool imu_ok = false;
    bool needs_cal = false;
    std::string cal_state = "idle";
    uint32_t cfg_fixed = 0;
    // --- MV2 move-executor status (`mv=2` advertises MV2 support) ---
    int64_t mv_seq = -1;        // last accepted MV2 seq; -1 = never
    int wd_state = 0;           // 0 fresh, 1 braking, 2 coasted
    std::string mv_kind = "-";  // "MOVE"/"HOLD"/"BRAKE"/"DISABLE", "-" = none
    int64_t tgt_dist_mm = -1;   // distance to the MOVE target; -1 = none

    std::string encode() const;
};

}  // namespace rf
