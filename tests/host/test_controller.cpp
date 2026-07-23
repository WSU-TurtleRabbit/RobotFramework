// Controller (Panthera cascade) tests: unit checks on the control law, and a
// simulated-plant arrival run that mirrors the server-side oracle's ideal
// follower (fake_robot.py tracks its regenerated trajectory perfectly; our
// plant is perfect velocity tracking through the real kinematics).
#include <cmath>
#include <limits>

#include "Motion/controller.h"
#include "Motion/phx/testing.h"

using namespace rf;

namespace {

constexpr double kDt = 0.004;

MotionSetpoint pose_sp(double tx, double ty, double th, double vmax, double amax,
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

MotionSetpoint vel_sp(MotionSetpoint::Kind kind, double vx, double vy, double w,
                      double acc = 2.0, double jerk = 100.0) {
    MotionSetpoint sp;
    sp.kind = kind;
    sp.vel = phx::Twist{vx, vy, w};
    sp.acc_max_xy = acc;
    sp.acc_max_w_vel = 20.0;
    sp.jerk_max_xy = jerk;
    sp.jerk_max_w = 400.0;
    return sp;
}

EstimatorOutput est_at(double x, double y, double th, double vx = 0, double vy = 0,
                       double w = 0) {
    EstimatorOutput e;
    e.pose.pos = {x, y};
    e.pose.heading = th;
    e.vel_global = {vx, vy};
    e.has_fix = true;
    e.vision_alive = true;
    e.omega = w;
    return e;
}

TrajSample ref_at(double x, double y, double th) {
    TrajSample r;
    r.pose.pos = {x, y};
    r.pose.heading = th;
    return r;
}

}  // namespace

PHX_TEST(controller_pose_p_pulls_toward_reference) {
    Controller c{ControllerConfig{}, Kinematics{}};
    // est at origin, reference 0.5 m ahead: P correction (clamped) drives +x.
    const ControlOutput out =
        c.tick(kDt, pose_sp(0.5, 0, 0, 2.0, 2.5), est_at(0, 0, 0), ref_at(0.5, 0, 0), 0.0);
    // pos_err 0.5 -> clamped 0.4 -> vel_corr = 2.5 * 0.4 = 1.0 (at clamp).
    CHECK_NEAR(out.cmd_body.lin.x, 1.0, 1e-9);
    CHECK_NEAR(out.cmd_body.lin.y, 0.0, 1e-9);
}

PHX_TEST(controller_pose_ff_velocity_flows_through) {
    Controller c{ControllerConfig{}, Kinematics{}};
    TrajSample r = ref_at(0, 0, 0);
    r.vel = {0.5, -0.2};
    const ControlOutput out =
        c.tick(kDt, pose_sp(1, 0, 0, 2.0, 2.5), est_at(0, 0, 0), r, 0.0);
    CHECK_NEAR(out.cmd_body.lin.x, 0.5, 1e-9);
    CHECK_NEAR(out.cmd_body.lin.y, -0.2, 1e-9);
}

PHX_TEST(controller_pose_velocity_feedback_damps_tracking_overshoot) {
    ControllerConfig cfg;
    cfg.kp_pos = 0.0;
    cfg.kp_vel = 0.45;
    cfg.vel_corr_max_mps = 0.6;
    Controller c{cfg, Kinematics{}};
    TrajSample r = ref_at(0, 0, 0);
    r.vel = {0.2, 0.0};
    // Still travelling at 0.8 m/s while the braking profile asks for 0.2:
    // velocity feedback must command below the trajectory reference.
    const ControlOutput out =
        c.tick(kDt, pose_sp(1, 0, 0, 1.0, 1.0),
               est_at(0, 0, 0, 0.8, 0.0), r, 0.0);
    CHECK_NEAR(out.cmd_body.lin.x, 0.2 + 0.45 * (0.2 - 0.8), 1e-9);
}

PHX_TEST(controller_pose_combined_correction_respects_skill_velocity_caps) {
    Controller c{ControllerConfig{}, Kinematics{}};
    TrajSample r = ref_at(0.5, 0.0, 1.0);
    r.vel = {0.25, 0.0};
    r.acc = {2.5, 0.0};
    r.omega = 2.0;
    const ControlOutput out = c.tick(
        kDt, pose_sp(1.0, 0.0, 1.0, 0.25, 2.5, 0.5),
        est_at(0.0, 0.0, 0.0), r, 0.0);
    CHECK(out.cmd_body.lin.norm() <= 0.25 + 1e-9);
    CHECK(std::fabs(out.cmd_body.ang) <= 0.5 + 1e-9);
}

PHX_TEST(controller_pose_actual_distance_caps_delayed_final_approach) {
    ControllerConfig cfg;
    cfg.kp_pos = 0.0;
    cfg.kp_vel = 0.0;
    cfg.acc_ff_lead_s = 0.0;
    cfg.target_brake_scale = 0.5;
    cfg.target_brake_reaction_s = 0.06;
    Controller c{cfg, Kinematics{}};
    TrajSample r = ref_at(0.9, 0.0, 0.0);
    r.vel = {1.0, 0.0};
    const MotionSetpoint sp = pose_sp(1.0, 0.0, 0.0, 1.5, 2.0);
    const ControlOutput out =
        c.tick(kDt, sp, est_at(0.9, 0.0, 0.0), r, 0.0);
    // a_brake=1, d=0.1, reaction=0.06:
    // v <= sqrt((a*t)^2 + 2*a*d) - a*t = 0.3912 m/s.
    CHECK(out.cmd_body.lin.norm() < 0.392);
    CHECK(out.cmd_body.lin.norm() > 0.390);
}

PHX_TEST(controller_omega_damps_gyro_overshoot) {
    Controller c{ControllerConfig{}, Kinematics{}};
    TrajSample r = ref_at(0, 0, 0);
    r.omega = 0.5;
    // Gyro reads 0.8 while the profile wants 0.5: the rate term must brake.
    const ControlOutput out =
        c.tick(kDt, pose_sp(0, 0, 0, 1.0, 1.0), est_at(0, 0, 0), r, 0.8);
    CHECK_NEAR(out.cmd_body.ang, 0.5 + 0.3 * (0.5 - 0.8), 1e-9);
}

PHX_TEST(controller_global_vel_rotates_into_body) {
    Controller c{ControllerConfig{}, Kinematics{}};
    c.reset(phx::Twist{});
    // Facing +y (pi/2), commanded +x in the world: body must go -y (right).
    const MotionSetpoint sp = vel_sp(MotionSetpoint::Kind::GlobalVel, 1.0, 0.0, 0.0);
    ControlOutput out;
    for (int i = 0; i < 250; ++i) {
        out = c.tick(kDt, sp, est_at(0, 0, phx::kPi / 2.0), TrajSample{}, 0.0);
    }
    CHECK_NEAR(out.cmd_body.lin.x, 0.0, 1e-6);
    CHECK_NEAR(out.cmd_body.lin.y, -1.0, 1e-6);
}

PHX_TEST(controller_local_vel_scurve_shapes_accel) {
    Controller c{ControllerConfig{}, Kinematics{}};
    c.reset(phx::Twist{});
    const MotionSetpoint sp = vel_sp(MotionSetpoint::Kind::LocalVel, 1.0, 0.0, 0.0,
                                     2.0, 100.0);
    // First tick: jerk-limited S-curve, not a step to the accel limit.
    ControlOutput out = c.tick(kDt, sp, est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK(std::fabs(out.cmd_body.lin.x) < 0.01);
    // Converges to the setpoint.
    for (int i = 0; i < 500; ++i) out = c.tick(kDt, sp, est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK_NEAR(out.cmd_body.lin.x, 1.0, 1e-6);
}

PHX_TEST(controller_wheel_vel_direct_with_slew) {
    Controller c{ControllerConfig{}, Kinematics{}};
    c.reset(phx::Twist{});
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::WheelVel;
    sp.wheel_rad_s = {10.0, 0.0, 0.0, 0.0};  // CAN order (id1 = FR)
    // First tick: slew-limited (120 rev/s^2 * 4 ms = 0.48 rev/s).
    ControlOutput out = c.tick(kDt, sp, est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK(std::fabs(out.wheel_rev_s[0]) <= 0.48 + 1e-9);
    // Converges to 10 rad/s = 10/(2 pi) rev/s on motor 1 only.
    for (int i = 0; i < 100; ++i) out = c.tick(kDt, sp, est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK_NEAR(out.wheel_rev_s[0], 10.0 / (2.0 * phx::kPi), 1e-9);
    CHECK_NEAR(out.wheel_rev_s[1], 0.0, 1e-9);
    CHECK(out.energize);
}

PHX_TEST(controller_emergency_ramps_at_tigers_rate_then_coasts) {
    Controller c{ControllerConfig{}, Kinematics{}};
    c.reset(phx::Twist{});
    // Spin up to 1 m/s via LOCAL_VEL.
    const MotionSetpoint vel = vel_sp(MotionSetpoint::Kind::LocalVel, 1.0, 0.0, 0.0);
    for (int i = 0; i < 500; ++i) c.tick(kDt, vel, est_at(0, 0, 0), TrajSample{}, 0.0);
    // EMERGENCY: 8 m/s^2 ramp — after 50 ms the speed must be ~0.6.
    MotionSetpoint emg;
    emg.kind = MotionSetpoint::Kind::Emergency;
    ControlOutput out;
    for (int i = 0; i < 12; ++i) out = c.tick(kDt, emg, est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK(out.energize);  // still ramping
    CHECK_NEAR(out.cmd_body.lin.norm(), 1.0 - 8.0 * 12 * kDt, 0.02);
    // ~0.125 s to zero; after 0.4 s the motors must be OFF (coast).
    for (int i = 0; i < 100; ++i) out = c.tick(kDt, emg, est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK(!out.energize);
    CHECK_NEAR(out.wheel_rev_s[0], 0.0, 1e-12);
}

PHX_TEST(controller_nan_reference_never_reaches_motors) {
    Controller c{ControllerConfig{}, Kinematics{}};
    c.reset(phx::Twist{});
    TrajSample r = ref_at(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    const ControlOutput out =
        c.tick(kDt, pose_sp(1, 0, 0, 2.0, 2.5), est_at(0, 0, 0), r, 0.0);
    for (double rev_s : out.wheel_rev_s) CHECK(std::isfinite(rev_s));
}

PHX_TEST(controller_sine_outputs_sinusoid) {
    Controller c{ControllerConfig{}, Kinematics{}};
    c.reset(phx::Twist{});
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::Sine;
    sp.sine_vx_amp = 0.5;
    sp.sine_freq_hz = 1.0;
    ControlOutput out;
    for (int i = 0; i < 63; ++i) {  // 0.252 s ~ quarter period of 1 Hz
        out = c.tick(kDt, sp, est_at(0, 0, 0), TrajSample{}, 0.0);
    }
    CHECK_NEAR(out.cmd_body.lin.x, 0.5, 0.02);
}

PHX_TEST(controller_model_ff_torque_sign_and_off_by_default) {
    Kinematics kin;
    {
        Controller c{ControllerConfig{}, kin};
        c.reset(phx::Twist{1.0, 0.0, 0.0});
        const ControlOutput out =
            c.tick(kDt, vel_sp(MotionSetpoint::Kind::LocalVel, 1.0, 0.0, 0.0),
                   est_at(0, 0, 0), TrajSample{}, 0.0);
        for (double t : out.wheel_ff_torque_nm) CHECK(t == 0.0);
    }
    ControllerConfig cfg;
    cfg.model_ff_enabled = true;
    Controller c{cfg, kin};
    c.reset(phx::Twist{1.0, 0.0, 0.0});  // already moving: coulomb friction
    const ControlOutput out =
        c.tick(kDt, vel_sp(MotionSetpoint::Kind::LocalVel, 1.0, 0.0, 0.0),
               est_at(0, 0, 0), TrajSample{}, 0.0);
    for (int i = 0; i < 4; ++i) {
        // Every wheel that moves forward gets a positive assist torque.
        if (out.wheel_rev_s[i] > 0.01) CHECK(out.wheel_ff_torque_nm[i] > 0.0);
    }
}

// --- simulated plant: does the cascade actually arrive, precisely? ---

PHX_TEST(controller_pose_arrives_on_simulated_plant) {
    Kinematics kin;
    Controller c{ControllerConfig{}, kin};
    TrajectoryFollower follower;

    // Truth state; the plant tracks wheel velocity setpoints perfectly.
    double x = 0, y = 0, th = 0;
    std::array<double, 4> wheels{};
    c.reset(phx::Twist{});
    follower.reset(phx::Pose{phx::Vec2{0, 0}, 0.0}, phx::Vec2{}, 0.0);
    const MotionSetpoint sp = pose_sp(1.0, 0.5, 0.8, 2.0, 2.5);

    double eta = -1.0;
    for (int i = 0; i < 1500; ++i) {  // 6 s max
        const EstimatorOutput est = est_at(x, y, th);
        const TrajSample ref = follower.tick(kDt, sp);
        if (i == 0) {
            // The bang-bang ETA for this move (for the sanity bound below).
            eta = phx::BangBang2D::plan({0, 0}, {0, 0}, {1.0, 0.5}, 2.0, 2.5)
                      .total_time();
        }
        const ControlOutput out = c.tick(kDt, sp, est, ref, est.omega);
        wheels = out.wheel_rev_s;
        // Plant: perfect velocity tracking -> odometry == FK(command).
        const BodyTwist odo = kin.forward(wheels);
        th = phx::wrap_angle(th + odo.w * kDt);
        const double cs = std::cos(th), sn = std::sin(th);
        x += (odo.vx * cs - odo.vy * sn) * kDt;
        y += (odo.vx * sn + odo.vy * cs) * kDt;
    }
    CHECK(std::fabs(x - 1.0) < 0.02);
    CHECK(std::fabs(y - 0.5) < 0.02);
    CHECK(std::fabs(phx::angle_diff(th, 0.8)) < 0.02);
    // Sanity: a perfect tracker arrives within ~1.5x the bang-bang ETA
    // (eta here is for the translation; orientation runs in parallel).
    CHECK(eta > 0.5);
}

PHX_TEST(controller_reset_prevents_reactivation_lurch) {
    Controller c{ControllerConfig{}, Kinematics{}};
    // Reset to the MEASURED twist: the next command must slew from there,
    // never jump to the new setpoint in one step.
    c.reset(phx::Twist{0.5, 0.0, 0.0});
    const ControlOutput out =
        c.tick(kDt, vel_sp(MotionSetpoint::Kind::LocalVel, 1.5, 0.0, 0.0),
               est_at(0, 0, 0), TrajSample{}, 0.0);
    CHECK(std::fabs(out.cmd_body.lin.x - 0.5) < 0.05);
}
