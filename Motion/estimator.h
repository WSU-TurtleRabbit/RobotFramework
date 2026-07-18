// Delayed-vision state estimator — the TIGERs fusion_ekf architecture
// (Firmware src/robot/fusion_ekf.c), adapted to our sensors: vision pose
// arrives LATE (posDelay, measured by the server per frame) and must be
// fused as if it had arrived on time.
//
// How it works (the delayed-vision trick that makes vision usable despite
// latency):
//   * A ring of ~128 time slots, one per control tick, each holding that
//     tick's gyro rate, wheel odometry, and the dead-reckoned (INS) state
//     propagated at full rate: [x, y, theta, vgx, vgy] (global-frame
//     velocities; theta integrates the gyro directly).
//   * When a MatchCtrl carries a fresh vision pose, the measurement is
//     inserted into the slot `posDelay + capture_delay` in the PAST: a
//     steady-state Kalman correction runs at that slot (the measurement is
//     compared against the state at ITS OWN time — this is what removes the
//     latency error a naive complementary filter bakes in), then the slots
//     up to the present are re-propagated (replayed) through their stored
//     gyro/odometry inputs.
//   * Newer INS slots are dragged toward the corrected history with a
//     tracking gain (TIGERs' smoothing — the present-time output moves
//     smoothly instead of snapping on every camera frame).
//   * Outliers are gated: a vision jump implying more than ~3 m/s of
//     correction speed is rejected (repeated rejections mean WE were lost,
//     so the next fix snaps). When vision times out (~1 s) the robot
//     dead-reckons on encoders + gyro and snaps back on the next fix.
//
// The Kalman gains are steady-state (alpha-beta), computed at construction
// by iterating the Riccati equation for a constant-velocity model — real
// Kalman filtering with fixed gains, deterministic and allocation-free at
// runtime. Heading is simpler: theta integrates the gyro and vision theta
// corrects with a fixed gain (the pi3hat gyro is excellent).
//
// Pure and deterministic: the caller feeds time, dt, odometry and gyro in.
// Ported from the TIGERs architecture; the odometry+gyro fusion itself is
// the chain proven on Robot B (phoenix-rf control.rs / motion.rs).
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "../Networks/matchctrl.h"  // VisionPose
#include "phx/pose.h"

namespace rf {

struct EstimatorConfig {
    int slots = 128;  // time-slot ring size (TIGERs: 128 slots)
    // Fixed extra delay of the vision pipeline on top of posDelay
    // (camera exposure + transfer; TIGERs use ~20 ms).
    double capture_delay_s = 0.020;
    // Vision older than this => dead-reckoning; the next fix snaps back.
    double vision_timeout_s = 1.0;
    // Outlier gate: reject a vision fix whose distance from the estimate at
    // its own timestamp exceeds max(gate_min_dist_m, gate_speed_mps * delay).
    double gate_speed_mps = 3.0;
    double gate_min_dist_m = 0.15;
    int gate_snap_after = 5;  // consecutive rejections -> we were lost: snap
    // Drag of INS slots toward the corrected history per vision fix (0..1].
    double tracking_gain = 0.5;
    // Body-velocity complementary gain toward wheel odometry per tick (0..1].
    double odo_vel_gain = 0.5;
    // Vision heading correction gain per fix (0..1].
    double theta_gain = 0.4;
    // Noise model for the steady-state Kalman gains: vision position
    // std-dev (m) and motion process accel std-dev (m/s^2).
    double meas_std_m = 0.005;
    double process_accel_std = 2.0;
    // Nominal tick used for the gain computation (the gains are constant;
    // runtime dt may jitter around this).
    double nominal_dt_s = 0.004;
};

// Present-time best estimate (the newest slot, smoothly consistent with the
// delayed fixes) plus health flags.
struct EstimatorOutput {
    phx::Pose pose{};          // global x, y (m), heading (rad)
    phx::Vec2 vel_global{};    // m/s, global frame
    phx::Twist vel_body{};     // m/s, body frame (derived) + yaw rate
    double omega = 0.0;        // rad/s, gyro rate used this tick
    bool has_fix = false;      // a vision pose has been accepted at least once
    bool vision_alive = false; // !dead-reckoning (vision within timeout)
    uint32_t rejected = 0;     // outlier-rejected vision fixes (stats)
};

class FusionEstimator {
public:
    explicit FusionEstimator(const EstimatorConfig& cfg = {});

    // Advance one control tick: propagate the INS state with the measured
    // body twist (wheel odometry FK) and gyro yaw rate (NaN gyro -> wheel
    // odometry yaw). now_s is any monotonic clock, seconds; dt is clamped.
    void tick(double now_s, double dt, const phx::Twist& odo_body, double gyro_yaw_radps);

    // Fuse a vision pose whose capture time was `pos_delay_s` before the
    // current tick (the server measures this per frame). The measurement is
    // inserted pos_delay_s + capture_delay into the past. Safe to call at
    // most once per camera frame; call after tick().
    void on_vision(const VisionPose& pose, double pos_delay_s);

    EstimatorOutput output() const;
    bool has_fix() const { return has_fix_; }

    // Corrected state at the LAST vision measurement's own timepoint — what
    // TIGERs report in MatchFeedback (pos + vel of the matched time slot).
    struct TimedState {
        phx::Pose pose{};
        phx::Vec2 vel_global{};
        double t_s = 0.0;
    };
    std::optional<TimedState> last_meas_state() const { return last_meas_; }

    // Steady-state Kalman gains (position, velocity) — exposed for tests.
    double k_pos() const { return k_pos_; }
    double k_vel() const { return k_vel_; }

private:
    struct Slot {
        double t_s = 0.0;
        double dt = 0.004;
        // inputs that produced this slot from the previous one
        double odo_vx = 0.0, odo_vy = 0.0, gyro_w = 0.0;
        // INS state
        double x = 0.0, y = 0.0, theta = 0.0, vgx = 0.0, vgy = 0.0;
    };

    struct State {
        double x = 0.0, y = 0.0, theta = 0.0, vgx = 0.0, vgy = 0.0;
    };

    State propagate(const State& s, double odo_vx, double odo_vy, double gyro_w,
                    double dt) const;
    void compute_gains();

    EstimatorConfig cfg_;
    std::vector<Slot> ring_;
    int head_ = 0;          // ring index of the newest slot
    int ticks_ = 0;         // ticks run; ring valid min(ticks_, size) deep
    bool started_ = false;  // first tick done
    bool has_fix_ = false;
    double last_vision_t_s = -1e9;
    int consecutive_rejects_ = 0;
    uint32_t rejected_total_ = 0;
    double k_pos_ = 0.0, k_vel_ = 0.0;  // steady-state Kalman gains
    std::optional<TimedState> last_meas_;
};

}  // namespace rf
