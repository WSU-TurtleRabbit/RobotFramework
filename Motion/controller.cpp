#include "controller.h"

#include <algorithm>
#include <cmath>

#include "phx/angle.h"

namespace rf {

void Controller::reset(const phx::Twist& measured_body) {
    shaper_.reset(measured_body);
    cmd_body_ = measured_body;
    sine_phase_ = 0.0;
    prev_wheel_ = kin_.inverse(BodyTwist{measured_body.lin.x, measured_body.lin.y,
                                         measured_body.ang});
}

ControlOutput Controller::tick(double dt_in, const MotionSetpoint& sp,
                               const EstimatorOutput& est, const TrajSample& ref,
                               double gyro_yaw_radps) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    ControlOutput out;

    switch (sp.kind) {
    case MotionSetpoint::Kind::Pose:
        return pose_tick(dt, sp, est, ref, gyro_yaw_radps);

    case MotionSetpoint::Kind::LocalVel: {
        // Body-frame setpoint, accel/jerk shaped (S-curve).
        cmd_body_ = shaper_.step(sp.vel, dt,
                                 phx::ShapeLimits{sp.acc_max_xy, sp.jerk_max_xy,
                                                  sp.acc_max_w_vel, sp.jerk_max_w});
        return map_wheels(dt, cmd_body_, true);
    }
    case MotionSetpoint::Kind::GlobalVel: {
        // World-frame setpoint rotated by the robot's own heading estimate.
        const phx::Twist body = phx::global_to_body(sp.vel, est.pose.heading);
        cmd_body_ = shaper_.step(body, dt,
                                 phx::ShapeLimits{sp.acc_max_xy, sp.jerk_max_xy,
                                                  sp.acc_max_w_vel, sp.jerk_max_w});
        return map_wheels(dt, cmd_body_, true);
    }
    case MotionSetpoint::Kind::WheelVel: {
        // Commissioning: direct wheel velocities (rad/s at the wheel ->
        // motor rev/s; direct drive, so a wheel revolution is a motor
        // revolution). CAN order was fixed in the skill decode.
        out.energize = true;
        for (int i = 0; i < 4; ++i) {
            const double want = sp.wheel_rad_s[i] / (2.0 * phx::kPi);
            const double dv = std::clamp(want - prev_wheel_[i],
                                         -cfg_.out_slew_rev_s2 * dt,
                                         cfg_.out_slew_rev_s2 * dt);
            out.wheel_rev_s[i] =
                std::clamp(prev_wheel_[i] + dv, -cfg_.wheel_max_rev_s,
                           cfg_.wheel_max_rev_s);
        }
        prev_wheel_ = out.wheel_rev_s;
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
        return map_wheels(dt, cmd_body_, true);
    }
    case MotionSetpoint::Kind::Emergency:
    default:
        return emergency_tick(dt);
    }
}

ControlOutput Controller::pose_tick(double dt, const MotionSetpoint& sp,
                                    const EstimatorOutput& est, const TrajSample& ref,
                                    double gyro_yaw_radps) {
    // Position error of the REFERENCE vs the estimate, global frame. The P
    // term corrects tracking error only — the trajectory supplies the
    // motion; clamped so a big disturbance can't command a lurch.
    phx::Vec2 pos_err = (ref.pose.pos - est.pose.pos).clamped(cfg_.pos_err_clamp_m);
    phx::Vec2 vel_corr = (pos_err * cfg_.kp_pos).clamped(cfg_.vel_corr_max_mps);

    // Velocity setpoint: trajectory FF (+ accel lead) + P, global frame.
    const phx::Vec2 vel_cmd_global =
        ref.vel + ref.acc * cfg_.acc_ff_lead_s + vel_corr;

    // Yaw: FF profile rate + P heading + P yaw-rate vs the measured gyro.
    const double herr = phx::angle_diff(ref.pose.heading, est.pose.heading);
    const double rate_err =
        std::isfinite(gyro_yaw_radps) ? ref.omega - gyro_yaw_radps : 0.0;
    const double omega_corr = std::clamp(cfg_.kp_heading * herr +
                                             cfg_.kp_yaw_rate * rate_err,
                                         -cfg_.omega_corr_max, cfg_.omega_corr_max);
    const double omega_cmd = ref.omega + omega_corr;

    // Global -> body by the robot's own heading estimate, then to wheels.
    const phx::Vec2 vel_body = vel_cmd_global.rotated(-est.pose.heading);
    cmd_body_ = phx::Twist{vel_body.x, vel_body.y, omega_cmd};
    return map_wheels(dt, cmd_body_, true);
}

ControlOutput Controller::emergency_tick(double dt) {
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
    const bool stopped = cmd_body_.lin.norm() < 0.02 && std::fabs(cmd_body_.ang) < 0.05;
    ControlOutput out = map_wheels(dt, cmd_body_, !stopped);
    if (stopped) {
        cmd_body_ = phx::Twist{};
        prev_wheel_.fill(0.0);
        out.energize = false;  // motors off: the ramp has landed
        out.wheel_rev_s.fill(0.0);
    }
    return out;
}

ControlOutput Controller::map_wheels(double dt, const phx::Twist& body, bool energize) {
    ControlOutput out;
    out.energize = energize;
    out.cmd_body = body;
    const std::array<double, 4> prev = prev_wheel_;
    const std::array<double, 4> want =
        kin_.inverse(BodyTwist{body.lin.x, body.lin.y, body.ang});
    for (int i = 0; i < 4; ++i) {
        // NaN guard: a non-finite setpoint must never reach the motor.
        const double w = std::isfinite(want[i]) ? want[i] : 0.0;
        const double dv = std::clamp(w - prev[i], -cfg_.out_slew_rev_s2 * dt,
                                     cfg_.out_slew_rev_s2 * dt);
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
