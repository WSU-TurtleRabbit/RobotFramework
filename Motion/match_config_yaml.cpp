#include "match_config_yaml.h"

#include <yaml-cpp/yaml.h>

namespace rf {
namespace {

template <typename T>
void opt(const YAML::Node& n, const char* key, T& out) {
    if (n[key]) out = n[key].as<T>();
}

}  // namespace

MatchSettings loadMatchSettings(const std::string& path) {
    MatchSettings s;
    try {
        const YAML::Node root = YAML::LoadFile(path);

        if (root["imu"]) {
            opt(root["imu"], "yaw_rate_axis", s.imu_yaw_rate_axis);
            opt(root["imu"], "yaw_rate_sign", s.imu_yaw_rate_sign);
        }
        if (root["match"]) {
            const YAML::Node m = root["match"];
            opt(m, "enabled", s.enabled);
            opt(m, "command_timeout_s", s.bridge.command_timeout_s);
            opt(m, "require_vision_fix", s.bridge.require_vision_fix);
            opt(m, "expected_robot_id", s.bridge.expected_robot_id);
        }
        if (root["estimator"]) {
            const YAML::Node e = root["estimator"];
            opt(e, "slots", s.estimator.slots);
            opt(e, "capture_delay_s", s.estimator.capture_delay_s);
            opt(e, "vision_timeout_s", s.estimator.vision_timeout_s);
            opt(e, "gate_speed_mps", s.estimator.gate_speed_mps);
            opt(e, "gate_min_dist_m", s.estimator.gate_min_dist_m);
            opt(e, "gate_snap_after", s.estimator.gate_snap_after);
            opt(e, "tracking_gain", s.estimator.tracking_gain);
            opt(e, "odo_vel_gain", s.estimator.odo_vel_gain);
            opt(e, "theta_gain", s.estimator.theta_gain);
            opt(e, "theta_gate_rad", s.estimator.theta_gate_rad);
            opt(e, "vision_heading_rate_max_rad_s",
                s.estimator.vision_heading_rate_max_rad_s);
            opt(e, "vision_heading_jump_tolerance_rad",
                s.estimator.vision_heading_jump_tolerance_rad);
            opt(e, "vision_heading_estimator_tolerance_rad",
                s.estimator.vision_heading_estimator_tolerance_rad);
            opt(e, "meas_std_m", s.estimator.meas_std_m);
            opt(e, "process_accel_std", s.estimator.process_accel_std);
            opt(e, "measurement_dt_s", s.estimator.measurement_dt_s);
            opt(e, "vision_pos_gain", s.estimator.vision_pos_gain);
            opt(e, "vision_vel_gain", s.estimator.vision_vel_gain);
        }
        if (root["trajectory"]) {
            const YAML::Node t = root["trajectory"];
            opt(t, "cent_acc_max", s.trajectory.cent_acc_max);
            opt(t, "brake_scale", s.trajectory.brake_scale);
            opt(t, "orient_brake_scale", s.trajectory.orient_brake_scale);
            opt(t, "orient_lag_tau_s", s.trajectory.orient_lag_tau_s);
            opt(t, "fast_pos_align_rad", s.trajectory.fast_pos_align_rad);
            opt(t, "final_orient_dist_m", s.trajectory.final_orient_dist_m);
            opt(t, "drive_dir_min_speed", s.trajectory.drive_dir_min_speed);
            opt(t, "pose_align_full_speed_rad",
                s.trajectory.pose_align_full_speed_rad);
            opt(t, "pose_align_slow_rad", s.trajectory.pose_align_slow_rad);
            opt(t, "pose_align_min_scale", s.trajectory.pose_align_min_scale);
        }
        if (root["controller"]) {
            const YAML::Node c = root["controller"];
            opt(c, "kp_pos", s.controller.kp_pos);
            opt(c, "kp_vel", s.controller.kp_vel);
            opt(c, "pos_err_clamp_m", s.controller.pos_err_clamp_m);
            opt(c, "vel_corr_max_mps", s.controller.vel_corr_max_mps);
            opt(c, "kp_heading", s.controller.kp_heading);
            opt(c, "kp_yaw_rate", s.controller.kp_yaw_rate);
            opt(c, "omega_corr_max", s.controller.omega_corr_max);
            opt(c, "acc_ff_lead_s", s.controller.acc_ff_lead_s);
            opt(c, "target_brake_scale", s.controller.target_brake_scale);
            opt(c, "target_brake_reaction_s",
                s.controller.target_brake_reaction_s);
            opt(c, "emergency_decel_mps2", s.controller.emergency_decel_mps2);
            opt(c, "emergency_w_decel_radps2", s.controller.emergency_w_decel_radps2);
            opt(c, "out_slew_rev_s2", s.controller.out_slew_rev_s2);
            opt(c, "wheel_max_rev_s", s.controller.wheel_max_rev_s);
            opt(c, "model_ff_enabled", s.controller.model_ff_enabled);
            opt(c, "robot_mass_kg", s.controller.robot_mass_kg);
            opt(c, "friction_coulomb_n", s.controller.friction_coulomb_n);
            opt(c, "friction_viscous_ns_m", s.controller.friction_viscous_ns_m);
            opt(c, "drivetrain_efficiency", s.controller.drivetrain_efficiency);
        }
        if (root["actuators"]) {
            const YAML::Node a = root["actuators"];
            opt(a, "kick_ms_per_mps", s.actuators.kick_ms_per_mps);
            opt(a, "kick_ms_bias", s.actuators.kick_ms_bias);
            opt(a, "kick_pulse_min_ms", s.actuators.kick_pulse_min_ms);
            opt(a, "kick_pulse_max_ms", s.actuators.kick_pulse_max_ms);
            opt(a, "kicker_recharge_s", s.actuators.kicker_recharge_s);
            opt(a, "dribble_us_per_mps", s.actuators.dribble_us_per_mps);
            opt(a, "barrier_radius_px", s.actuators.barrier_radius_px);
            opt(a, "barrier_bearing_rad", s.actuators.barrier_bearing_rad);
            opt(a, "barrier_min_confidence", s.actuators.barrier_min_confidence);
            opt(a, "barrier_max_age_s", s.actuators.barrier_max_age_s);
        }
        if (root["feedback"]) {
            const YAML::Node f = root["feedback"];
            opt(f, "hardware_id", s.hardware_id);
            opt(f, "battery_empty_v", s.battery_empty_v);
            opt(f, "battery_full_v", s.battery_full_v);
            opt(f, "kicker_max_v", s.kicker_max_v);
            opt(f, "kicker_recharge_s", s.kicker_recharge_s);
            opt(f, "interval_ms", s.feedback_interval_ms);
        }
    } catch (const std::exception&) {
        // Missing/invalid file: keep every built-in default (they are the
        // tuned values, not placeholders).
    }
    return s;
}

}  // namespace rf
