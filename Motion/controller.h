// The motion controller — TIGERs' "Panthera" cascade (Firmware
// src/robot/ctrl_panthera.c) adapted to our actuator layer: TIGERs end their
// cascade at per-wheel CURRENT on custom FOC boards; we end at per-wheel
// VELOCITY on moteus controllers (they are our FOC layer — the inner loop is
// theirs). Everything above that difference is their structure:
//
//   pose skills:   velocity FF (trajectory) + P position + P velocity
//                  tracking (global, jointly clamped)
//                  + P heading + P yaw-rate vs the gyro
//   vel skills:    accel/jerk S-curve shaping toward the velocity setpoint
//                  (the vendored phx::TwistShaper — the S-curve shaper
//                  proven on Robot B via phoenix-rf)
//   wheel skill:   direct per-wheel setpoints (commissioning)
//   sine skill:    open-loop sinusoid (system ID)
//   emergency:     controlled ramp to zero at the TIGERs rate, then coast
//
// Model feedforward (coulomb + viscous friction + mass accel -> per-wheel
// feedforward torque on the moteus) is implemented but OFF by default: its
// constants must be identified on hardware (the SINE skill +
// phoenix-server phoenix/tools/commission.py exist for that). A wrong
// friction model fights the moteus velocity loop; a right one sharpens
// response. Accel feedforward as a velocity lead is likewise configurable.
//
// Pure and deterministic: no clocks, no sockets, no heap.
#pragma once

#include <array>
#include <cstdint>

#include "../Math/kinematics.h"
#include "estimator.h"
#include "phx/pose.h"
#include "phx/twist_shaper.h"  // S-curve, proven on Robot B
#include "skills.h"
#include "trajectory.h"

namespace rf {

struct ControllerConfig {
    // --- Pose cascade gains ---
    double kp_pos = 2.5;            // 1/s: velocity correction per m of error
    double kp_vel = 0.0;            // velocity-error damping (0 = disabled)
    double pos_err_clamp_m = 0.4;   // clamp on the position error vector
    double vel_corr_max_mps = 1.0;  // clamp on combined tracking correction
    // Heading: P on heading error + P on yaw-rate error vs the gyro. Gains
    // proven on Robot B (phoenix-rf HeadingController: kp 6, kd 0.3, clamp 3).
    double kp_heading = 6.0;
    double kp_yaw_rate = 0.3;
    // At high translation speed, delayed pose heading needs less stiffness
    // and more rate damping than precise low-speed endpoint capture.
    double high_speed_yaw_start_mps = 1.0;
    double high_speed_yaw_full_mps = 2.0;
    double high_speed_heading_gain_scale = 1.0;
    double high_speed_rate_gain_scale = 1.0;
    // Low-pass only the gyro signal used by derivative-like yaw damping.
    // Heading fusion/integration remains unfiltered. Zero disables filtering.
    double yaw_rate_filter_tau_s = 0.05;
    double omega_corr_max = 3.0;    // rad/s clamp on the total omega correction
    // Dynamic traction sharing for unexpected yaw during pose translation.
    // Translation remains full below `full`, blends down to `min_scale` at
    // `slow`, and returns continuously as yaw-rate tracking recovers.
    double yaw_traction_full_radps = 0.35;
    double yaw_traction_slow_radps = 1.50;
    double yaw_traction_min_scale = 0.20;
    // Trajectory acceleration feedforward as a velocity lead, seconds (the
    // moteus velocity loop's lag compensation; SINE commissioning tunes it).
    double acc_ff_lead_s = 0.04;
    // Final-approach guard based on the ACTUAL fused pose rather than the
    // ahead-of-plant trajectory reference. The allowed translational speed
    // solves d = v*t_reaction + v^2/(2*a_brake), where
    // a_brake = skill_accel * target_brake_scale.
    double target_brake_scale = 0.5;
    double target_brake_reaction_s = 0.0;
    // Pose-command S-curve. Zero preserves the raw regenerated bang-bang
    // reference; positive values jerk-limit the final body-velocity command
    // so launch and braking acceleration join continuously.
    double pose_jerk_max_xy = 0.0;  // m/s^3
    double pose_jerk_max_w = 0.0;   // rad/s^3
    // Direction-aware jerk limits in the chassis frame. These use the same
    // ellipse as the onboard speed/acceleration envelope; zero falls back to
    // pose_jerk_max_xy for backward-compatible configs.
    double pose_longitudinal_jerk_max_xy = 0.0;
    double pose_lateral_jerk_max_xy = 0.0;
    double pose_brake_acc_max_xy = 0.0;
    double pose_brake_longitudinal_jerk_max_xy = 0.0;
    double pose_brake_lateral_jerk_max_xy = 0.0;

    // --- EMERGENCY ramp (TIGERs onboard rate) ---
    double emergency_decel_mps2 = 8.0;
    double emergency_w_decel_radps2 = 30.0;
    double emergency_settle_speed_mps = 0.05;
    double emergency_settle_yaw_radps = 0.15;
    double emergency_active_brake_timeout_s = 1.50;

    // --- Output stage ---
    double out_slew_rev_s2 = 120.0;  // per-wheel setpoint slew (rev/s^2)
    double wheel_max_rev_s = 45.0;   // per-wheel saturation (rev/s)

    // --- Model feedforward (per-wheel torque), OFF until identified ---
    bool model_ff_enabled = false;
    double robot_mass_kg = 2.6;
    double friction_coulomb_n = 1.5;   // per wheel
    double friction_viscous_ns_m = 0.5;  // per wheel
    double drivetrain_efficiency = 0.8;
};

struct ControlOutput {
    std::array<double, 4> wheel_rev_s{};       // CAN order (id i+1 = index i)
    std::array<double, 4> wheel_ff_torque_nm{};  // 0 unless model FF enabled
    bool energize = false;                     // false = coast (motors off)
    phx::Twist cmd_body{};                     // shaped body twist (telemetry)
    phx::Twist model_body{};                   // stable controller before augmentation
    phx::Twist adaptive_delta{};                // deterministic identified correction
    phx::Twist rl_proposed{};                   // policy proposal (also in shadow)
    phx::Twist rl_applied{};                    // zero unless bounded + healthy
    uint32_t safety_interventions = 0;          // augmentation constraint activations
};

// Stable controller result before body-to-wheel kinematics. Keeping this
// boundary explicit lets the adaptive and residual layers remain separate
// from the conventional controller. `direct_wheels` is used only by the
// commissioning WHEEL_VEL skill, which bypasses all adaptive logic.
struct StableControl {
    phx::Twist body{};
    bool energize = false;
    bool direct_wheels = false;
    std::array<double, 4> wheel_rev_s{};
};

class Controller {
public:
    Controller(const ControllerConfig& cfg, const Kinematics& kin)
        : cfg_(cfg), kin_(kin) {}

    // One control tick. `sp` is the active skill setpoint; `est` the
    // estimator output; `ref` the trajectory sample (Pose skills only —
    // ignored otherwise); `gyro_yaw_radps` the measured yaw rate (NaN =
    // unknown: yaw-rate damping stands down).
    ControlOutput tick(double dt, const MotionSetpoint& sp, const EstimatorOutput& est,
                       const TrajSample& ref, double gyro_yaw_radps);

    // Split form used by MatchBridge:
    //   compute stable model-based body command -> external bounded augmentor
    //   -> finalize through the existing wheel slew/saturation/FF stage.
    // Calling tick() is exactly compute()+finalize() with no augmentation.
    StableControl compute(double dt, const MotionSetpoint& sp,
                          const EstimatorOutput& est, const TrajSample& ref,
                          double gyro_yaw_radps);
    ControlOutput finalize(double dt, const StableControl& stable,
                           const phx::Twist& applied_body);

    // Reset all internal rate state (skill switch, re-enable): the next
    // output slews from the CURRENT measured body twist, never from a stale
    // command (prevents re-activation lurches).
    void reset(const phx::Twist& measured_body);

    const phx::Twist& last_cmd_body() const { return cmd_body_; }

private:
    StableControl pose_tick(double dt, const MotionSetpoint& sp,
                            const EstimatorOutput& est, const TrajSample& ref,
                            double gyro_yaw_radps);
    StableControl emergency_tick(double dt, const EstimatorOutput& est,
                                 double gyro_yaw_radps);
    ControlOutput map_wheels(double dt, const phx::Twist& body, bool energize);

    ControllerConfig cfg_;
    Kinematics kin_;
    phx::TwistShaper shaper_;          // vel skills
    phx::TwistShaper pose_shaper_;     // pose skills, world-frame state
    bool pose_shaper_initialized_ = false;
    phx::Twist cmd_body_{};            // last commanded body twist
    double filtered_yaw_rate_ = 0.0;   // yaw damping input, rad/s
    double emergency_elapsed_s_ = 0.0; // bounded active-braking window
    double sine_phase_ = 0.0;          // SINE accumulator
    std::array<double, 4> prev_wheel_{};  // slew state
};

}  // namespace rf
