#include "controller.h"

#include <algorithm>
#include <cmath>

#include "phx/angle.h"

namespace rf {

namespace {

double directional_ellipse_limit(double direction, double longitudinal,
                                 double lateral) {
    const double a = std::max(1e-6, longitudinal);
    const double b = std::max(1e-6, lateral);
    const double c = std::cos(direction);
    const double s = std::sin(direction);
    return 1.0 / std::sqrt((c * c) / (a * a) + (s * s) / (b * b));
}

}  // namespace

void Controller::reset(const phx::Twist& measured_body) {
    shaper_.reset(measured_body);
    pose_shaper_initialized_ = false;
    cmd_body_ = measured_body;
    filtered_yaw_rate_ = measured_body.ang;
    emergency_elapsed_s_ = 0.0;
    sine_phase_ = 0.0;
    prev_wheel_ = kin_.inverse(BodyTwist{measured_body.lin.x, measured_body.lin.y,
                                         measured_body.ang});
}

ControlOutput Controller::tick(double dt_in, const MotionSetpoint& sp,
                               const EstimatorOutput& est, const TrajSample& ref,
                               double gyro_yaw_radps) {
    const StableControl stable = compute(dt_in, sp, est, ref, gyro_yaw_radps);
    return finalize(dt_in, stable, stable.body);
}

StableControl Controller::compute(double dt_in, const MotionSetpoint& sp,
                                  const EstimatorOutput& est, const TrajSample& ref,
                                  double gyro_yaw_radps) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    if (sp.kind != MotionSetpoint::Kind::Emergency) emergency_elapsed_s_ = 0.0;

    switch (sp.kind) {
    case MotionSetpoint::Kind::Pose:
        return pose_tick(dt, sp, est, ref, gyro_yaw_radps);

    case MotionSetpoint::Kind::LocalVel: {
        // Body-frame setpoint, accel/jerk shaped (S-curve).
        cmd_body_ = shaper_.step(sp.vel, dt,
                                 phx::ShapeLimits{sp.acc_max_xy, sp.jerk_max_xy,
                                                  sp.acc_max_w_vel, sp.jerk_max_w});
        return StableControl{cmd_body_, true};
    }
    case MotionSetpoint::Kind::GlobalVel: {
        // World-frame setpoint rotated by the robot's own heading estimate.
        const phx::Twist body = phx::global_to_body(sp.vel, est.pose.heading);
        cmd_body_ = shaper_.step(body, dt,
                                 phx::ShapeLimits{sp.acc_max_xy, sp.jerk_max_xy,
                                                  sp.acc_max_w_vel, sp.jerk_max_w});
        return StableControl{cmd_body_, true};
    }
    case MotionSetpoint::Kind::WheelVel: {
        // Commissioning: direct wheel velocities (rad/s at the wheel ->
        // motor rev/s; direct drive, so a wheel revolution is a motor
        // revolution). CAN order was fixed in the skill decode.
        StableControl out;
        out.energize = true;
        out.direct_wheels = true;
        for (int i = 0; i < 4; ++i) {
            out.wheel_rev_s[i] = sp.wheel_rad_s[i] / (2.0 * phx::kPi);
        }
        cmd_body_ = phx::Twist{};
        return out;
    }
    case MotionSetpoint::Kind::Sine: {
        // Open-loop sinusoid for system ID (TIGERs BotSkillSine semantics):
        // vx(t) = amp_x sin(2 pi f t) etc., body frame.
        sine_phase_ += dt;
        const double arg = 2.0 * phx::kPi * sp.sine_freq_hz * sine_phase_;
        cmd_body_ = phx::Twist{sp.sine_vx_amp * std::sin(arg),
                               sp.sine_vy_amp * std::sin(arg),
                               sp.sine_w_amp * std::sin(arg)};
        return StableControl{cmd_body_, true};
    }
    case MotionSetpoint::Kind::Emergency:
    default:
        return emergency_tick(dt, est, gyro_yaw_radps);
    }
}

StableControl Controller::pose_tick(double dt, const MotionSetpoint& sp,
                                    const EstimatorOutput& est, const TrajSample& ref,
                                    double gyro_yaw_radps) {
    // Position error of the REFERENCE vs the estimate, global frame. The P
    // term corrects tracking error only — the trajectory supplies the
    // motion; clamped so a big disturbance can't command a lurch.
    const phx::Vec2 pos_err =
        (ref.pose.pos - est.pose.pos).clamped(cfg_.pos_err_clamp_m);
    const phx::Vec2 vel_err = ref.vel - est.vel_global;
    const phx::Vec2 vel_corr =
        (pos_err * cfg_.kp_pos + vel_err * cfg_.kp_vel)
            .clamped(cfg_.vel_corr_max_mps);

    // Velocity setpoint: trajectory FF (+ accel lead) + P, global frame.
    // The trajectory limit is a HARD motion envelope, not merely a planner
    // hint.  Feedforward lead and tracking correction may sharpen response,
    // but their sum must never exceed the skill's granted velocity ceiling.
    phx::Vec2 vel_cmd_global =
        (ref.vel + ref.acc * cfg_.acc_ff_lead_s + vel_corr)
            .clamped(std::max(0.0, sp.vel_max_xy));

    // The regenerated reference is intentionally smooth, so during a hard
    // launch it can run several centimetres ahead of the physical chassis.
    // Braking solely from that reference then starts late. Bound the final
    // approach speed with the fused robot-to-target distance and an
    // identified reaction allowance. The wheel-output slew remains the last
    // actuator-rate limiter.
    const double target_dist = (sp.target.pos - est.pose.pos).norm();
    const double brake_acc =
        std::max(0.0, sp.acc_max_xy * cfg_.target_brake_scale);
    if (brake_acc > 1e-6 && std::isfinite(target_dist)) {
        const double reaction = std::max(0.0, cfg_.target_brake_reaction_s);
        const double ar = brake_acc * reaction;
        const double approach_max =
            std::max(0.0, std::sqrt(ar * ar + 2.0 * brake_acc * target_dist) - ar);
        vel_cmd_global = vel_cmd_global.clamped(
            std::min(std::max(0.0, sp.vel_max_xy), approach_max));
    }

    // Yaw: FF profile rate + P heading + P yaw-rate vs the measured gyro.
    const double herr = phx::angle_diff(ref.pose.heading, est.pose.heading);
    if (std::isfinite(gyro_yaw_radps)) {
        const double tau = std::max(0.0, cfg_.yaw_rate_filter_tau_s);
        const double alpha = tau > 1e-9 ? 1.0 - std::exp(-dt / tau) : 1.0;
        filtered_yaw_rate_ += alpha * (gyro_yaw_radps - filtered_yaw_rate_);
    }
    const double rate_err = std::isfinite(gyro_yaw_radps)
                                ? ref.omega - filtered_yaw_rate_
                                : 0.0;
    const double yaw_speed_start = std::max(0.0, cfg_.high_speed_yaw_start_mps);
    const double yaw_speed_full =
        std::max(yaw_speed_start + 1e-6, cfg_.high_speed_yaw_full_mps);
    const double yaw_speed_blend = std::clamp(
        (est.vel_global.norm() - yaw_speed_start) /
            (yaw_speed_full - yaw_speed_start),
        0.0, 1.0);
    const double heading_gain = cfg_.kp_heading *
        (1.0 + yaw_speed_blend *
            (std::clamp(cfg_.high_speed_heading_gain_scale, 0.1, 1.0) - 1.0));
    const double rate_gain = cfg_.kp_yaw_rate *
        (1.0 + yaw_speed_blend *
            (std::clamp(cfg_.high_speed_rate_gain_scale, 1.0, 3.0) - 1.0));
    const double omega_corr = std::clamp(heading_gain * herr +
                                             rate_gain * rate_err,
                                         -cfg_.omega_corr_max, cfg_.omega_corr_max);
    const double omega_cmd = std::clamp(
        ref.omega + omega_corr, -std::max(0.0, sp.vel_max_w),
        std::max(0.0, sp.vel_max_w));

    // Shape pose translation in the WORLD frame. Shaping after rotation into
    // the body frame makes a yaw disturbance rotate the shaper's velocity
    // state and bend an otherwise straight field-line command. Only rotate
    // the final smooth command into the body frame for wheel kinematics.
    phx::Twist shaped_global{vel_cmd_global.x, vel_cmd_global.y, omega_cmd};
    if (cfg_.pose_jerk_max_xy > 0.0 && cfg_.pose_jerk_max_w > 0.0) {
        if (!pose_shaper_initialized_) {
            pose_shaper_.reset(phx::Twist{
                est.vel_global.x, est.vel_global.y, est.omega});
            pose_shaper_initialized_ = true;
        }
        const phx::Twist shaped_current = pose_shaper_.current();
        const phx::Vec2 velocity_delta =
            shaped_global.lin - shaped_current.lin;
        const bool braking = shaped_current.lin.norm() > 0.05 &&
            shaped_current.lin.dot(velocity_delta) < -1e-6;
        phx::Vec2 jerk_body = vel_cmd_global.rotated(-est.pose.heading);
        if (jerk_body.norm() <= 0.05) {
            jerk_body = cmd_body_.lin;
        }
        double jerk_max_xy = cfg_.pose_jerk_max_xy;
        const double longitudinal_jerk =
            braking && cfg_.pose_brake_longitudinal_jerk_max_xy > 0.0
                ? cfg_.pose_brake_longitudinal_jerk_max_xy
                : cfg_.pose_longitudinal_jerk_max_xy;
        const double lateral_jerk =
            braking && cfg_.pose_brake_lateral_jerk_max_xy > 0.0
                ? cfg_.pose_brake_lateral_jerk_max_xy
                : cfg_.pose_lateral_jerk_max_xy;
        if (jerk_body.norm() > 1e-9 &&
            longitudinal_jerk > 0.0 && lateral_jerk > 0.0) {
            jerk_max_xy = directional_ellipse_limit(
                std::atan2(jerk_body.y, jerk_body.x),
                longitudinal_jerk, lateral_jerk);
        }
        const double accel_max_xy =
            braking && cfg_.pose_brake_acc_max_xy > 0.0
                ? std::max(sp.acc_max_xy, cfg_.pose_brake_acc_max_xy)
                : std::max(0.0, sp.acc_max_xy);
        shaped_global = pose_shaper_.step(
            shaped_global, dt,
            phx::ShapeLimits{accel_max_xy, jerk_max_xy,
                             std::max(0.0, sp.acc_max_w),
                             cfg_.pose_jerk_max_w});
        // Translation benefits from comfort jerk limiting; held-heading yaw
        // is a disturbance-rejection loop and must not queue corrections
        // behind that shaper. Preserve immediate gyro-damped yaw authority.
        shaped_global.ang = omega_cmd;
    }
    phx::Vec2 vel_body = shaped_global.lin.rotated(-est.pose.heading);
    // A wheel can track its encoder velocity while an omni roller unloads or
    // binds against the carpet. Heading-error-only traction sharing reacts
    // too late to that event: on hardware the chassis could already be
    // 0.20 m off line before the fused angle crossed its gate. Use the
    // filtered gyro rate error as the early signal, preserving full speed
    // during normal motion and continuously restoring it after recovery.
    const double yaw_rate_error = std::fabs(rate_err);
    const double traction_full = std::max(0.0, cfg_.yaw_traction_full_radps);
    const double traction_slow =
        std::max(traction_full + 1e-6, cfg_.yaw_traction_slow_radps);
    const double traction_min =
        std::clamp(cfg_.yaw_traction_min_scale, 0.0, 1.0);
    const double traction_blend =
        std::clamp((yaw_rate_error - traction_full) /
                       (traction_slow - traction_full),
                   0.0, 1.0);
    const double traction_scale =
        1.0 - traction_blend * (1.0 - traction_min);
    cmd_body_ = phx::Twist{vel_body.x, vel_body.y, shaped_global.ang};
    cmd_body_.lin *= traction_scale;
    return StableControl{cmd_body_, true};
}

StableControl Controller::emergency_tick(double dt, const EstimatorOutput& est,
                                         double gyro_yaw_radps) {
    emergency_elapsed_s_ += dt;
    // Controlled ramp to zero (TIGERs onboard EMERGENCY semantics), then
    // coast. Never a cliff: wheels decelerate at the pinned rates.
    const double speed = cmd_body_.lin.norm();
    if (speed > 1e-9) {
        const double ns = std::max(0.0, speed - cfg_.emergency_decel_mps2 * dt);
        cmd_body_.lin *= (ns / speed);
    }
    const double w = cmd_body_.ang;
    if (std::fabs(w) > 1e-9) {
        const double nw =
            std::max(0.0, std::fabs(w) - cfg_.emergency_w_decel_radps2 * dt);
        cmd_body_.ang = std::copysign(nw, w);
    }
    const bool command_stopped =
        cmd_body_.lin.norm() < 0.02 && std::fabs(cmd_body_.ang) < 0.05;
    const double measured_yaw = std::isfinite(gyro_yaw_radps)
                                    ? std::fabs(gyro_yaw_radps)
                                    : std::fabs(est.omega);
    const bool measured_stopped =
        est.vel_global.norm() < cfg_.emergency_settle_speed_mps &&
        measured_yaw < cfg_.emergency_settle_yaw_radps;
    const bool brake_timed_out =
        emergency_elapsed_s_ >= cfg_.emergency_active_brake_timeout_s;
    // Keep the velocity loops energized at zero until the chassis actually
    // lands. Previously we coasted as soon as the COMMAND reached zero; a
    // high-load omni event then free-spun for ~0.9 s and rotated 120 degrees.
    // A latched hardware safety trip still overrides this output and cuts
    // energization in RobotFramework.cpp.
    const bool coast = command_stopped && (measured_stopped || brake_timed_out);
    StableControl out;
    out.body = cmd_body_;
    out.energize = !coast;
    if (coast) {
        cmd_body_ = phx::Twist{};
        out.energize = false;  // motors off: command and chassis have landed
        out.body = phx::Twist{};
    }
    return out;
}

ControlOutput Controller::finalize(double dt_in, const StableControl& stable,
                                   const phx::Twist& applied_body) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    if (stable.direct_wheels) {
        ControlOutput out;
        out.energize = stable.energize;
        out.model_body = stable.body;
        for (int i = 0; i < 4; ++i) {
            const double want = std::isfinite(stable.wheel_rev_s[i])
                                    ? stable.wheel_rev_s[i]
                                    : 0.0;
            const double dv = std::clamp(want - prev_wheel_[i],
                                         -cfg_.out_slew_rev_s2 * dt,
                                         cfg_.out_slew_rev_s2 * dt);
            out.wheel_rev_s[i] =
                std::clamp(prev_wheel_[i] + dv, -cfg_.wheel_max_rev_s,
                           cfg_.wheel_max_rev_s);
        }
        prev_wheel_ = out.wheel_rev_s;
        return out;
    }
    ControlOutput out = map_wheels(dt, applied_body, stable.energize);
    out.model_body = stable.body;
    return out;
}

ControlOutput Controller::map_wheels(double dt, const phx::Twist& body, bool energize) {
    ControlOutput out;
    out.energize = energize;
    out.cmd_body = body;
    const std::array<double, 4> prev = prev_wheel_;
    const std::array<double, 4> want =
        kin_.inverse(BodyTwist{body.lin.x, body.lin.y, body.ang});
    std::array<double, 4> delta{};
    double peak_delta = 0.0;
    for (int i = 0; i < 4; ++i) {
        // NaN guard: a non-finite setpoint must never reach the motor.
        const double w = std::isfinite(want[i]) ? want[i] : 0.0;
        delta[i] = w - prev[i];
        peak_delta = std::max(peak_delta, std::abs(delta[i]));
    }
    // Scale the complete wheel-delta vector as one unit. Independent wheel
    // clipping bends a simultaneous translation+yaw request because the
    // smaller deltas arrive early while the largest wheel lags. A shared
    // scale preserves the inverse-kinematic ratio and therefore the intended
    // body direction, with the same per-wheel slew ceiling.
    const double allowed_delta = std::max(0.0, cfg_.out_slew_rev_s2 * dt);
    const double delta_scale =
        peak_delta > allowed_delta && peak_delta > 1e-12
            ? allowed_delta / peak_delta
            : 1.0;
    for (int i = 0; i < 4; ++i) {
        const double dv = delta[i] * delta_scale;
        out.wheel_rev_s[i] =
            std::clamp(prev[i] + dv, -cfg_.wheel_max_rev_s, cfg_.wheel_max_rev_s);
    }
    prev_wheel_ = out.wheel_rev_s;

    // Optional model feedforward torque (OFF until identified on hardware):
    // per-wheel coulomb + viscous friction + a share of the body accel.
    if (cfg_.model_ff_enabled) {
        const double mpr = kin_.meters_per_motor_rev;
        const double wheel_r = mpr / (2.0 * phx::kPi);
        for (int i = 0; i < 4; ++i) {
            const double surface_v = out.wheel_rev_s[i] * mpr;
            const double surface_a =
                dt > 1e-9 ? (out.wheel_rev_s[i] - prev[i]) * mpr / dt : 0.0;
            const double force =
                (surface_v != 0.0 ? std::copysign(cfg_.friction_coulomb_n, surface_v)
                                  : 0.0) +
                cfg_.friction_viscous_ns_m * surface_v +
                (cfg_.robot_mass_kg / 4.0) * surface_a;
            out.wheel_ff_torque_nm[i] =
                force * wheel_r / std::max(0.1, cfg_.drivetrain_efficiency);
        }
    }
    return out;
}

}  // namespace rf
