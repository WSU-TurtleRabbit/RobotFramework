// Trajectory regeneration tests: vendored bang-bang planners pinned to the
// SERVER's trajectory.py golden numbers (the same algorithm must plan on
// both sides — the server computes ETAs with it), plus the per-tick
// regeneration behavior that makes motion smooth.
#include <cmath>

#include "Motion/phx/testing.h"
#include "Motion/trajectory.h"

using namespace rf;

namespace {
constexpr double kDt = 0.004;

MotionSetpoint make_pos(double tx, double ty, double th, double vmax, double amax,
                        double wmax = 10.0, double awmax = 40.0) {
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::Pose;
    sp.target.pos = {tx, ty};
    sp.target.heading = th;
    sp.vel_max_xy = vmax;
    sp.vel_max_w = wmax;
    sp.acc_max_xy = amax;
    sp.acc_max_w = awmax;
    return sp;
}
}  // namespace

// --- vendored planner: golden numbers from phoenix-server trajectory.py ---

PHX_TEST(bangbang2d_matches_server_trajectory_py) {
    // Python: BangBang2D.plan(Vec2(0,0), Vec2(0,0), Vec2(1.0, 0.5), 2.0, 2.5)
    const phx::BangBang2D a =
        phx::BangBang2D::plan({0, 0}, {0, 0}, {1.0, 0.5}, 2.0, 2.5);
    CHECK_NEAR(a.total_time(), 1.337480609953, 1e-9);
    CHECK_NEAR(a.alpha(), 0.463647609001, 1e-9);
    CHECK_NEAR(a.pos(0.10).x, 0.011180339887, 1e-9);
    CHECK_NEAR(a.pos(0.10).y, 0.005590169944, 1e-9);
    CHECK_NEAR(a.vel(0.10).x, 0.223606797750, 1e-9);
    CHECK_NEAR(a.acc(0.30).x, 2.236067977500, 1e-9);
    CHECK_NEAR(a.pos(0.55).x, 0.338205281597, 1e-9);
    CHECK_NEAR(a.vel(0.55).y, 0.614918693812, 1e-9);

    // Moving start: plan(Vec2(0.2,-0.1), Vec2(0.5,0.4), Vec2(-0.5,0.3), 3, 4)
    const phx::BangBang2D b =
        phx::BangBang2D::plan({0.2, -0.1}, {0.5, 0.4}, {-0.5, 0.3}, 3.0, 4.0);
    CHECK_NEAR(b.total_time(), 0.997647949746, 1e-9);
    CHECK_NEAR(b.alpha(), 0.245248868908, 1e-9);
    CHECK_NEAR(b.pos(0.10).x, 0.230598461396, 1e-9);
    CHECK_NEAR(b.vel(0.10).x, 0.111969227919, 1e-9);
    CHECK_NEAR(b.acc(0.10).x, -3.880307720810, 1e-9);
    CHECK_NEAR(b.pos(0.25).y, 0.030349719286, 1e-9);
    CHECK_NEAR(b.vel(0.25).y, 0.642797754292, 1e-9);
}

PHX_TEST(bangbang1d_matches_server_trajectory_py) {
    // Python: BangBang1D.plan(0, 0, 1.2, 0, 6.0, 20.0)
    const phx::BangBang1D c = phx::BangBang1D::plan(0.0, 0.0, 1.2, 0.0, 6.0, 20.0);
    CHECK_NEAR(c.total_time(), 0.489897948557, 1e-9);
    CHECK_NEAR(c.pos(0.10), 0.100000000000, 1e-9);
    CHECK_NEAR(c.vel(0.10), 2.000000000000, 1e-9);
    CHECK_NEAR(c.pos(0.30), 0.839387691340, 1e-9);
    CHECK_NEAR(c.vel(0.30), 3.797958971133, 1e-9);
    CHECK_NEAR(c.acc(0.30), -20.0, 1e-9);
}

PHX_TEST(bangbang1d_asymmetric_braking_starts_earlier_and_lands_exactly) {
    const phx::BangBang1D symmetric =
        phx::BangBang1D::plan(0.0, 0.0, 1.3, 0.0, 1.0, 1.0);
    const phx::BangBang1D early_brake =
        phx::BangBang1D::plan(0.0, 0.0, 1.3, 0.0, 1.0, 1.0, 0.6);
    REQUIRE(early_brake.segment_count() >= 2);
    CHECK_NEAR(early_brake.segment(0).a, 1.0, 1e-12);
    CHECK_NEAR(
        early_brake.segment(early_brake.segment_count() - 1).a, -0.6, 1e-12);
    CHECK(early_brake.segment(0).dt < symmetric.segment(0).dt);
    CHECK_NEAR(early_brake.end_pos(), 1.3, 1e-9);
    CHECK_NEAR(early_brake.end_vel(), 0.0, 1e-9);
    CHECK(early_brake.vel(early_brake.segment(0).dt) <= 1.0 + 1e-9);
}

// --- per-tick regeneration behavior ---

PHX_TEST(trajectory_reaches_target_and_stops) {
    TrajectoryFollower f;
    f.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    const MotionSetpoint sp = make_pos(1.0, 0.5, 0.8, 2.0, 2.5);
    TrajSample s;
    int ticks = 0;
    double max_speed = 0.0;
    do {
        s = f.tick(kDt, sp);
        max_speed = std::max(max_speed, s.vel.norm());
        ++ticks;
    } while (!s.done && ticks < 2000);
    CHECK(s.done);
    CHECK(ticks < 2000);
    CHECK_NEAR(s.pose.pos.x, 1.0, 1e-12);
    CHECK_NEAR(s.pose.pos.y, 0.5, 1e-12);
    CHECK_NEAR(s.pose.heading, 0.8, 1e-9);
    CHECK_NEAR(s.vel.norm(), 0.0, 1e-12);
    // The velocity limit is a hard invariant of the profile.
    CHECK(max_speed <= 2.0 + 1e-9);
}

PHX_TEST(trajectory_retarget_stays_continuous) {
    // THE smoothness test: regenerating every tick from the REFERENCE state
    // means a mid-path re-target bends the velocity profile continuously —
    // never a step. Assert per-tick velocity change never exceeds amax*dt.
    TrajectoryFollower f;
    f.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    const MotionSetpoint a = make_pos(2.0, 0.0, 0.0, 3.0, 4.0);
    const MotionSetpoint b = make_pos(-1.0, 1.0, 1.0, 3.0, 4.0);
    phx::Vec2 prev_v;
    double prev_w = 0.0;
    for (int i = 0; i < 400; ++i) {
        const TrajSample s = f.tick(kDt, i < 100 ? a : b);
        const double dv = (s.vel - prev_v).norm();
        const double dw = std::fabs(s.omega - prev_w);
        CHECK(dv <= 4.0 * kDt + 1e-9);
        CHECK(dw <= 40.0 * kDt + 1e-9);
        prev_v = s.vel;
        prev_w = s.omega;
    }
}

PHX_TEST(trajectory_centrifugal_omega_cap_applies) {
    TrajectoryFollower f;  // default cent_acc_max 2.5
    f.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    // Far target + a heading flip: at 2.5 m/s the cap is 2.5/2.5 = 1 rad/s,
    // far below vel_max_w = 30.
    const MotionSetpoint sp = make_pos(5.0, 0.0, 3.0, 3.5, 5.0, 30.0, 80.0);
    double max_speed = 0.0, max_omega_at_speed = 0.0;
    for (int i = 0; i < 1000; ++i) {
        const TrajSample s = f.tick(kDt, sp);
        const double v = s.vel.norm();
        max_speed = std::max(max_speed, v);
        if (v > 1.0) {
            max_omega_at_speed = std::max(max_omega_at_speed, std::fabs(s.omega));
            CHECK(std::fabs(s.omega) <= 2.5 / v + 0.05 /* tick-rate slack */);
        }
        if (s.done) break;
    }
    CHECK(max_speed > 2.0);  // it really got fast (the cap had to bite)
}

PHX_TEST(trajectory_uses_onboard_heading_for_anisotropic_chassis_limits) {
    TrajectoryConfig cfg;
    cfg.body_longitudinal_vel_max = 3.5;
    cfg.body_lateral_vel_max = 2.2;
    cfg.body_longitudinal_acc_max = 4.0;
    cfg.body_lateral_acc_max = 2.0;
    cfg.pose_align_min_scale = 1.0;

    TrajectoryFollower forward(cfg);
    forward.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    TrajectoryFollower lateral(cfg);
    lateral.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    const MotionSetpoint forward_sp = make_pos(20.0, 0.0, 0.0, 5.0, 8.0);
    const MotionSetpoint lateral_sp = make_pos(0.0, 20.0, 0.0, 5.0, 8.0);

    TrajSample fwd;
    TrajSample side;
    for (int i = 0; i < 1000; ++i) {
        fwd = forward.tick(kDt, forward_sp);
        side = lateral.tick(kDt, lateral_sp);
    }
    CHECK(fwd.vel.norm() <= 3.5 + 1e-9);
    CHECK(fwd.vel.norm() > 3.4);
    CHECK(side.vel.norm() <= 2.2 + 1e-9);
    CHECK(side.vel.norm() > 2.1);
}

PHX_TEST(trajectory_directional_limit_rotates_with_held_chassis_heading) {
    TrajectoryConfig cfg;
    cfg.body_longitudinal_vel_max = 3.5;
    cfg.body_lateral_vel_max = 2.0;
    cfg.body_longitudinal_acc_max = 4.0;
    cfg.body_lateral_acc_max = 2.0;
    cfg.pose_align_min_scale = 1.0;
    TrajectoryFollower f(cfg);
    // Facing +Y makes a global +Y route longitudinal, not lateral.
    f.reset(phx::Pose{phx::Vec2{0, 0}, phx::kPi / 2.0}, phx::Vec2{}, 0.0);
    const MotionSetpoint sp = make_pos(0.0, 20.0, phx::kPi / 2.0, 5.0, 8.0);
    TrajSample sample;
    for (int i = 0; i < 1000; ++i) sample = f.tick(kDt, sp);
    CHECK(sample.vel.norm() > 3.4);
    CHECK(sample.vel.norm() <= 3.5 + 1e-9);
}

PHX_TEST(trajectory_orientation_brake_scale_reduces_peak_yaw_rate) {
    TrajectoryConfig symmetric_cfg;
    symmetric_cfg.orient_lag_tau_s = 0.0;
    TrajectoryFollower symmetric(symmetric_cfg);
    symmetric.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);

    TrajectoryConfig early_cfg = symmetric_cfg;
    early_cfg.orient_brake_scale = 0.5;
    TrajectoryFollower early(early_cfg);
    early.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);

    const MotionSetpoint sp =
        make_pos(0.0, 0.0, phx::kPi / 2.0, 1.0, 1.0, 3.0, 4.0);
    double symmetric_peak = 0.0;
    double early_peak = 0.0;
    TrajSample early_done;
    for (int i = 0; i < 2000; ++i) {
        const TrajSample a = symmetric.tick(kDt, sp);
        early_done = early.tick(kDt, sp);
        symmetric_peak = std::max(symmetric_peak, std::fabs(a.omega));
        early_peak = std::max(early_peak, std::fabs(early_done.omega));
        if (a.done && early_done.done) break;
    }
    CHECK(early_peak < symmetric_peak);
    CHECK(early_done.done);
    CHECK_NEAR(early_done.pose.heading, phx::kPi / 2.0, 1e-9);
}

PHX_TEST(trajectory_orientation_lag_smooths_heading_retarget) {
    TrajectoryConfig cfg;
    cfg.orient_lag_tau_s = 0.1;
    TrajectoryFollower f(cfg);
    f.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    // Already at the target position; only the heading jumps by 2 rad.
    const MotionSetpoint sp = make_pos(0.0, 0.0, 2.0, 1.0, 1.0, 30.0, 100.0);
    const TrajSample first = f.tick(kDt, sp);
    // A light low-pass: the first tick's yaw rate must be well below the
    // instant-replan value (no lag would jump toward 30 rad/s territory).
    CHECK(std::fabs(first.omega) < 5.0);
}

PHX_TEST(trajectory_pose_alignment_moves_gently_then_releases_full_speed) {
    TrajectoryConfig gated_cfg;
    gated_cfg.orient_lag_tau_s = 0.0;
    TrajectoryFollower gated(gated_cfg);
    gated.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);

    TrajectoryConfig open_cfg = gated_cfg;
    open_cfg.pose_align_min_scale = 1.0;
    TrajectoryFollower open(open_cfg);
    open.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);

    const MotionSetpoint sp =
        make_pos(2.0, 0.0, phx::kPi / 2.0, 2.0, 2.0, 3.0, 6.0);
    TrajSample gated_mid;
    TrajSample open_mid;
    for (int i = 0; i < 50; ++i) {
        gated_mid = gated.tick(kDt, sp);
        open_mid = open.tick(kDt, sp);
    }
    CHECK(gated_mid.pose.pos.x > 0.0);  // coupled move, not rotate-only
    CHECK(gated_mid.pose.heading > 0.0);
    CHECK(gated_mid.vel.norm() < open_mid.vel.norm());

    TrajSample done = gated_mid;
    for (int i = 0; i < 3000 && !done.done; ++i) done = gated.tick(kDt, sp);
    CHECK(done.done);
    CHECK_NEAR(done.pose.pos.x, 2.0, 1e-9);
    CHECK_NEAR(done.pose.heading, phx::kPi / 2.0, 1e-9);
}

PHX_TEST(trajectory_fast_pos_slaves_heading_to_drive_direction) {
    TrajectoryConfig cfg;
    cfg.orient_lag_tau_s = 0.0;
    TrajectoryFollower f(cfg);
    f.reset(phx::Pose{phx::Vec2{0, 0}, 1.2 /* looking away */}, phx::Vec2{}, 0.0);
    MotionSetpoint sp = make_pos(2.0, 1.0, -2.0, 3.0, 3.0, 8.0, 40.0);
    sp.fast_pos = true;
    sp.acc_max_xy_fast = 6.0;
    // While far, the heading reference must pull toward the drive direction
    // (atan2(1,2) = 0.4636), not the final -2.0.
    for (int i = 0; i < 50; ++i) f.tick(kDt, sp);
    const TrajSample mid = f.tick(kDt, sp);
    const double drive = std::atan2(1.0, 2.0);
    CHECK(std::fabs(phx::angle_diff(mid.pose.heading, drive)) < 0.35);
    // Run to completion: the final orientation applies near the target.
    TrajSample s = mid;
    for (int i = 0; i < 2000 && !s.done; ++i) s = f.tick(kDt, sp);
    CHECK(s.done);
    CHECK_NEAR(s.pose.heading, -2.0, 1e-6);
}

PHX_TEST(trajectory_fast_pos_aligns_before_full_translation_launch) {
    TrajectoryConfig cfg;
    cfg.orient_lag_tau_s = 0.0;
    cfg.pose_align_min_scale = 0.10;
    TrajectoryFollower gated(cfg);
    gated.reset(phx::Pose{phx::Vec2{0, 0}, phx::kPi / 2.0},
                phx::Vec2{}, 0.0);

    TrajectoryConfig open_cfg = cfg;
    open_cfg.pose_align_min_scale = 1.0;
    TrajectoryFollower open(open_cfg);
    open.reset(phx::Pose{phx::Vec2{0, 0}, phx::kPi / 2.0},
               phx::Vec2{}, 0.0);

    MotionSetpoint sp = make_pos(2.0, 0.0, 0.0, 2.5, 1.2, 3.0, 3.0);
    sp.fast_pos = true;
    sp.acc_max_xy_fast = 1.2;

    const TrajSample first = gated.tick(kDt, sp);
    const TrajSample open_first = open.tick(kDt, sp);
    CHECK(first.pose.heading < phx::kPi / 2.0);
    CHECK(first.vel.norm() < open_first.vel.norm() * 0.2);

    TrajSample aligned = first;
    for (int i = 0; i < 1000 &&
                    std::fabs(phx::angle_diff(aligned.pose.heading, 0.0)) >
                        cfg.pose_align_full_speed_rad;
         ++i) {
        aligned = gated.tick(kDt, sp);
    }
    CHECK(std::fabs(phx::angle_diff(aligned.pose.heading, 0.0)) <=
          cfg.pose_align_full_speed_rad);
    const double before = aligned.vel.norm();
    for (int i = 0; i < 50; ++i) aligned = gated.tick(kDt, sp);
    CHECK(aligned.vel.norm() > before);
}

PHX_TEST(trajectory_primary_direction_uses_target_direction_at_rest) {
    TrajectoryConfig cfg;
    cfg.orient_lag_tau_s = 0.0;
    TrajectoryFollower f(cfg);
    f.reset(phx::Pose{phx::Vec2{0, 0}, phx::kPi / 2.0},
            phx::Vec2{}, 0.0);
    MotionSetpoint sp = make_pos(2.0, 0.0, 1.0, 2.5, 1.2, 3.0, 3.0);
    sp.primary_direction = 0.0;

    const TrajSample first = f.tick(kDt, sp);
    CHECK(first.pose.heading < phx::kPi / 2.0);
    CHECK(first.vel.norm() <=
          sp.acc_max_xy * cfg.pose_align_min_scale * kDt + 1e-9);
}

PHX_TEST(trajectory_reset_reanchors_reference) {
    TrajectoryFollower f;
    f.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    const MotionSetpoint sp = make_pos(1.0, 0.0, 0.0, 2.0, 2.0);
    for (int i = 0; i < 50; ++i) f.tick(kDt, sp);
    // Estimator snapped us to (0.5, 0.1): the reference re-anchors there
    // instead of the controller seeing a fake half-metre jump.
    f.reset(phx::Pose{phx::Vec2{0.5, 0.1}, 0.3}, phx::Vec2{0.2, 0.0}, 0.0);
    const TrajSample s = f.current();
    CHECK_NEAR(s.pose.pos.x, 0.5, 1e-12);
    CHECK_NEAR(s.pose.pos.y, 0.1, 1e-12);
    CHECK_NEAR(s.pose.heading, 0.3, 1e-12);
    CHECK_NEAR(s.vel.x, 0.2, 1e-12);
}
