#include "trajectory.h"

#include <algorithm>
#include <cmath>

#include "phx/angle.h"

namespace rf {

void TrajectoryFollower::reset(const phx::Pose& pose, const phx::Vec2& vel_global,
                               double omega) {
    cur_ = TrajSample{};
    cur_.pose = pose;
    cur_.vel = vel_global;
    cur_.omega = omega;
    orient_target_filt_ = pose.heading;
    started_ = true;
}

TrajSample TrajectoryFollower::tick(double dt_in, const MotionSetpoint& sp) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    if (!started_) {
        // First use without reset: start at the target at rest (harmless —
        // the bridge always resets from the estimator on activation).
        reset(sp.target, phx::Vec2{}, 0.0);
    }

    const phx::Vec2 to_target = sp.target.pos - cur_.pose.pos;
    const double dist = to_target.norm();
    const double speed = cur_.vel.norm();
    // At launch there is no velocity direction yet.  Use the actual
    // displacement instead of the current chassis heading so FAST_POS and
    // primaryDirection start aligning before translation builds.  Once
    // moving, the reference velocity remains the smoother direction signal
    // across ordinary re-targets.
    const double drive_dir =
        speed > cfg_.drive_dir_min_speed
            ? std::atan2(cur_.vel.y, cur_.vel.x)
            : (dist > 1e-9 ? std::atan2(to_target.y, to_target.x)
                           : cur_.pose.heading);

    // --- orientation target: final heading, drive-direction slave (FAST_POS),
    // or primary-direction offset ---
    double orient_target = sp.target.heading;
    const bool slave_to_drive =
        dist > cfg_.final_orient_dist_m &&
        (sp.fast_pos || sp.primary_direction.has_value());
    if (slave_to_drive) {
        orient_target =
            sp.fast_pos
                ? drive_dir
                : phx::wrap_angle(drive_dir - *sp.primary_direction);
    }
    // Light low-pass on the orientation target (TIGERs smooths re-targets).
    if (cfg_.orient_lag_tau_s > 0.0) {
        const double a = 1.0 - std::exp(-dt / cfg_.orient_lag_tau_s);
        orient_target_filt_ = phx::wrap_angle(
            orient_target_filt_ + a * phx::angle_diff(orient_target, orient_target_filt_));
    } else {
        orient_target_filt_ = orient_target;
    }

    // --- effective limits ---
    double acc_max_xy = sp.acc_max_xy;
    const double orient_error =
        std::fabs(phx::angle_diff(orient_target, cur_.pose.heading));
    if (sp.fast_pos && slave_to_drive &&
        orient_error < cfg_.fast_pos_align_rad) {
        acc_max_xy = std::max(acc_max_xy, sp.acc_max_xy_fast);
    }
    double vel_max_xy = sp.vel_max_xy;
    // Share traction between translation and rotation for every pose mode.
    // Previously FAST_POS/primaryDirection bypassed this gate, so they could
    // launch at full translation while still facing across the requested
    // path.  That produced a large sideways arc on real omni wheels.
    const bool gate_translation =
        slave_to_drive || (!sp.fast_pos && !sp.primary_direction.has_value());
    if (gate_translation) {
        const double heading_error =
            slave_to_drive
                ? orient_error
                : std::fabs(
                      phx::angle_diff(sp.target.heading, cur_.pose.heading));
        const double full = std::max(0.0, cfg_.pose_align_full_speed_rad);
        const double slow = std::max(full + 1e-6, cfg_.pose_align_slow_rad);
        const double min_scale =
            std::clamp(cfg_.pose_align_min_scale, 0.0, 1.0);
        const double blend =
            std::clamp((heading_error - full) / (slow - full), 0.0, 1.0);
        const double scale = 1.0 - blend * (1.0 - min_scale);
        vel_max_xy *= scale;
        acc_max_xy *= scale;
    }
    // Centrifugal omega cap: |w| <= centAccMax / |v_xy| (divisor floored so
    // the cap is inert near standstill).
    const double w_cap_cent = cfg_.cent_acc_max / std::max(speed, 0.3);
    const double vel_max_w_eff = std::min(sp.vel_max_w, w_cap_cent);

    // --- regenerate the translation profile from the reference state ---
    const phx::BangBang2D tr =
        phx::BangBang2D::plan(cur_.pose.pos, cur_.vel, sp.target.pos,
                              vel_max_xy, acc_max_xy,
                              acc_max_xy * std::clamp(cfg_.brake_scale, 0.1, 1.0));
    // --- regenerate the orientation profile (wrapped short way) ---
    const double heading_target =
        cur_.pose.heading + phx::angle_diff(orient_target_filt_, cur_.pose.heading);
    const double orient_brake =
        sp.acc_max_w * std::clamp(cfg_.orient_brake_scale, 0.1, 1.0);
    const phx::BangBang1D rot =
        phx::BangBang1D::plan(cur_.pose.heading, cur_.omega, heading_target, 0.0,
                              vel_max_w_eff, sp.acc_max_w, orient_brake);

    const bool done = tr.total_time() <= dt && rot.total_time() <= dt;
    if (done) {
        // Land exactly: no limit-cycle dither at the target.
        cur_.pose.pos = sp.target.pos;
        cur_.pose.heading = phx::wrap_angle(heading_target);
        cur_.vel = phx::Vec2{};
        cur_.acc = phx::Vec2{};
        cur_.omega = 0.0;
        cur_.alpha = 0.0;
    } else {
        cur_.pose.pos = tr.pos(dt);
        cur_.pose.heading = phx::wrap_angle(rot.pos(dt));
        cur_.vel = tr.vel(dt);
        cur_.acc = tr.acc(dt);
        cur_.omega = rot.vel(dt);
        cur_.alpha = rot.acc(dt);
    }
    cur_.dist_remaining = dist;
    cur_.done = done;
    return cur_;
}

}  // namespace rf
