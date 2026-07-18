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
