// Per-tick onboard trajectory regeneration — the TIGERs smoothness trick
// (Firmware src/robot/traj_bang_bang.c + src/shared/math/traj_2order.c):
// instead of following a plan made when the command arrived, REGENERATE the
// bang-bang trajectory every control tick, starting from the current
// REFERENCE state (the last trajectory sample — smooth by construction),
// not from the noisy estimator state. Server re-targets, vision snaps and
// disturbances then bend the path continuously instead of jerking it.
//
// The 2D planner is the vendored phx::BangBang2D (alpha-bisection sync —
// both axes arrive together; the SAME algorithm the server mirrors in
// phoenix/core/trajectory.py for ETAs). Orientation gets its own 1D
// profile with two TIGERs refinements:
//   * the centrifugal omega limit — yaw rate is capped by
//     centAccMax / |v_xy| so fast translation doesn't fling the robot
//     sideways past traction;
//   * a light low-pass on the orientation target, smoothing server
//     re-targeting of the final heading.
//
// FAST_POS and primaryDirection semantics are handled here (they are
// trajectory-shaping, not control): FAST_POS slaves the heading to the
// drive direction and raises accel once aligned; a primary direction keeps
// the robot's preferred axis along the travel direction until the final
// approach.
//
// Pure and deterministic: no clocks, no sockets.
#pragma once

#include "phx/bang_bang_1d.h"
#include "phx/bang_bang_2d.h"
#include "phx/pose.h"
#include "skills.h"

namespace rf {

struct TrajectoryConfig {
    // Lateral accel budget for the centrifugal omega cap, m/s^2: yaw rate is
    // limited to cent_acc_max / |v_xy| (divisor floored at 0.3 m/s).
    double cent_acc_max = 2.5;
    // Braking acceleration as a fraction of the skill's acceleration limit.
    // Below one starts braking earlier without weakening launch.
    double brake_scale = 1.0;
    // First-order low-pass on the orientation target, seconds (0 = off).
    double orient_lag_tau_s = 0.05;
    // FAST_POS: heading counts as aligned with the drive direction within
    // this, and the raised accel applies, rad.
    double fast_pos_align_rad = 0.25;
    // FAST_POS / primaryDirection: inside this distance to the target the
    // FINAL skill orientation takes over from the drive-direction slave, m.
    double final_orient_dist_m = 0.35;
    // Below this reference speed the drive direction is undefined — hold the
    // current heading instead of slaving, m/s.
    double drive_dir_min_speed = 0.3;
};

// One regenerated trajectory sample — the reference the controller tracks.
struct TrajSample {
    phx::Pose pose{};     // reference position, global frame
    phx::Vec2 vel{};      // reference velocity, global frame, m/s
    phx::Vec2 acc{};      // reference acceleration, global frame, m/s^2
    double omega = 0;     // reference yaw rate, rad/s
    double alpha = 0;     // reference yaw acceleration, rad/s^2
    double dist_remaining = 0;  // planar distance to the skill target, m
    bool done = false;    // translation AND orientation profiles finished
};

class TrajectoryFollower {
public:
    explicit TrajectoryFollower(const TrajectoryConfig& cfg = {}) : cfg_(cfg) {}

    // Re-anchor the reference state (estimator snap, skill-kind switch):
    // regeneration then continues from here next tick.
    void reset(const phx::Pose& pose, const phx::Vec2& vel_global, double omega);

    // Regenerate from the current reference and advance one control tick.
    // `sp` must be a Pose-kind setpoint (GLOBAL_POS / FAST_POS).
    TrajSample tick(double dt, const MotionSetpoint& sp);

    const TrajSample& current() const { return cur_; }

private:
    TrajectoryConfig cfg_;
    TrajSample cur_{};
    double orient_target_filt_ = 0;  // low-passed orientation target
    bool started_ = false;
};

}  // namespace rf
