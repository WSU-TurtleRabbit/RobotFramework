// The motion controller — TIGERs' "Panthera" cascade (Firmware
// src/robot/ctrl_panthera.c) adapted to our actuator layer: TIGERs end their
// cascade at per-wheel CURRENT on custom FOC boards; we end at per-wheel
// VELOCITY on moteus controllers (they are our FOC layer — the inner loop is
// theirs). Everything above that difference is their structure:
//
//   pose skills:   velocity FF (trajectory) + P position (global, clamped)
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

#include "../Math/kinematics.h"
#include "estimator.h"
#include "phx/executor.h"  // phx::TwistShaper (S-curve, proven on Robot B)
#include "phx/pose.h"
#include "skills.h"
#include "trajectory.h"

namespace rf {

struct ControllerConfig {
    // --- Pose cascade gains ---
    double kp_pos = 2.5;            // 1/s: velocity correction per m of error
    double pos_err_clamp_m = 0.4;   // clamp on the position error vector
    double vel_corr_max_mps = 1.0;  // clamp on the added correction velocity
    // Heading: P on heading error + P on yaw-rate error vs the gyro. Gains
    // proven on Robot B (phoenix-rf HeadingController: kp 6, kd 0.3, clamp 3).
    double kp_heading = 6.0;
    double kp_yaw_rate = 0.3;
    double omega_corr_max = 3.0;    // rad/s clamp on the total omega correction
    // Trajectory acceleration feedforward as a velocity lead, seconds (the
    // moteus velocity loop's lag compensation; SINE commissioning tunes it).
    double acc_ff_lead_s = 0.04;

    // --- EMERGENCY ramp (TIGERs onboard rate) ---
    double emergency_decel_mps2 = 8.0;
    double emergency_w_decel_radps2 = 30.0;

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

    // Reset all internal rate state (skill switch, re-enable): the next
    // output slews from the CURRENT measured body twist, never from a stale
    // command (prevents re-activation lurches).
    void reset(const phx::Twist& measured_body);

    const phx::Twist& last_cmd_body() const { return cmd_body_; }

private:
    ControlOutput pose_tick(double dt, const MotionSetpoint& sp,
                            const EstimatorOutput& est, const TrajSample& ref,
                            double gyro_yaw_radps);
    ControlOutput emergency_tick(double dt);
    ControlOutput map_wheels(double dt, const phx::Twist& body, bool energize);

    ControllerConfig cfg_;
    const Kinematics& kin_;
    phx::TwistShaper shaper_;          // vel skills
    phx::Twist cmd_body_{};            // last commanded body twist
    double sine_phase_ = 0.0;          // SINE accumulator
    std::array<double, 4> prev_wheel_{};  // slew state
};

}  // namespace rf
