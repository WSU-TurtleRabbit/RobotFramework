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

        if (root["profile"]) {
            opt(root["profile"], "id", s.profile_id);
            opt(root["profile"], "name", s.profile_name);
        }
        if (root["imu"]) {
            opt(root["imu"], "yaw_rate_axis", s.imu_yaw_rate_axis);
            opt(root["imu"], "yaw_rate_sign", s.imu_yaw_rate_sign);
            opt(root["imu"], "accel_forward_axis", s.imu_accel_forward_axis);
            opt(root["imu"], "accel_lateral_axis", s.imu_accel_lateral_axis);
            opt(root["imu"], "accel_forward_sign", s.imu_accel_forward_sign);
            opt(root["imu"], "accel_lateral_sign", s.imu_accel_lateral_sign);
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
            opt(e, "gyro_rate_filter_tau_s",
                s.estimator.gyro_rate_filter_tau_s);
            opt(e, "theta_gain", s.estimator.theta_gain);
            opt(e, "theta_gate_rad", s.estimator.theta_gate_rad);
            opt(e, "vision_heading_rate_max_rad_s",
                s.estimator.vision_heading_rate_max_rad_s);
            opt(e, "vision_heading_jump_tolerance_rad",
                s.estimator.vision_heading_jump_tolerance_rad);
            opt(e, "vision_heading_estimator_tolerance_rad",
                s.estimator.vision_heading_estimator_tolerance_rad);
            opt(e, "vision_heading_relock_tolerance_rad",
                s.estimator.vision_heading_relock_tolerance_rad);
            opt(e, "vision_heading_relock_stability_rad",
                s.estimator.vision_heading_relock_stability_rad);
            opt(e, "meas_std_m", s.estimator.meas_std_m);
            opt(e, "process_accel_std", s.estimator.process_accel_std);
            opt(e, "measurement_dt_s", s.estimator.measurement_dt_s);
            opt(e, "vision_pos_gain", s.estimator.vision_pos_gain);
            opt(e, "vision_vel_gain", s.estimator.vision_vel_gain);
        }
        if (root["trajectory"]) {
            const YAML::Node t = root["trajectory"];
            opt(t, "body_longitudinal_vel_max", s.trajectory.body_longitudinal_vel_max);
            opt(t, "body_lateral_vel_max", s.trajectory.body_lateral_vel_max);
            opt(t, "body_longitudinal_acc_max", s.trajectory.body_longitudinal_acc_max);
            opt(t, "body_lateral_acc_max", s.trajectory.body_lateral_acc_max);
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
            opt(c, "high_speed_yaw_start_mps",
                s.controller.high_speed_yaw_start_mps);
            opt(c, "high_speed_yaw_full_mps",
                s.controller.high_speed_yaw_full_mps);
            opt(c, "high_speed_heading_gain_scale",
                s.controller.high_speed_heading_gain_scale);
            opt(c, "high_speed_rate_gain_scale",
                s.controller.high_speed_rate_gain_scale);
            opt(c, "yaw_rate_filter_tau_s",
                s.controller.yaw_rate_filter_tau_s);
            opt(c, "yaw_traction_full_radps",
                s.controller.yaw_traction_full_radps);
            opt(c, "yaw_traction_slow_radps",
                s.controller.yaw_traction_slow_radps);
            opt(c, "yaw_traction_min_scale",
                s.controller.yaw_traction_min_scale);
            opt(c, "omega_corr_max", s.controller.omega_corr_max);
            opt(c, "acc_ff_lead_s", s.controller.acc_ff_lead_s);
            opt(c, "target_brake_scale", s.controller.target_brake_scale);
            opt(c, "target_brake_reaction_s",
                s.controller.target_brake_reaction_s);
            opt(c, "pose_jerk_max_xy", s.controller.pose_jerk_max_xy);
            opt(c, "pose_jerk_max_w", s.controller.pose_jerk_max_w);
            opt(c, "pose_longitudinal_jerk_max_xy",
                s.controller.pose_longitudinal_jerk_max_xy);
            opt(c, "pose_lateral_jerk_max_xy",
                s.controller.pose_lateral_jerk_max_xy);
            opt(c, "pose_brake_acc_max_xy",
                s.controller.pose_brake_acc_max_xy);
            opt(c, "pose_brake_longitudinal_jerk_max_xy",
                s.controller.pose_brake_longitudinal_jerk_max_xy);
            opt(c, "pose_brake_lateral_jerk_max_xy",
                s.controller.pose_brake_lateral_jerk_max_xy);
            opt(c, "emergency_decel_mps2", s.controller.emergency_decel_mps2);
            opt(c, "emergency_w_decel_radps2", s.controller.emergency_w_decel_radps2);
            opt(c, "emergency_settle_speed_mps",
                s.controller.emergency_settle_speed_mps);
            opt(c, "emergency_settle_yaw_radps",
                s.controller.emergency_settle_yaw_radps);
            opt(c, "emergency_active_brake_timeout_s",
                s.controller.emergency_active_brake_timeout_s);
            opt(c, "out_slew_rev_s2", s.controller.out_slew_rev_s2);
            opt(c, "wheel_max_rev_s", s.controller.wheel_max_rev_s);
            opt(c, "model_ff_enabled", s.controller.model_ff_enabled);
            opt(c, "robot_mass_kg", s.controller.robot_mass_kg);
            opt(c, "friction_coulomb_n", s.controller.friction_coulomb_n);
            opt(c, "friction_viscous_ns_m", s.controller.friction_viscous_ns_m);
            opt(c, "drivetrain_efficiency", s.controller.drivetrain_efficiency);
        }
        if (root["adaptive"]) {
            const YAML::Node a = root["adaptive"];
            opt(a, "enabled", s.augmentation.estimator.enabled);
            opt(a, "forgetting_factor", s.augmentation.estimator.forgetting_factor);
            opt(a, "huber_sigma", s.augmentation.estimator.huber_sigma);
            opt(a, "min_velocity_error_mps",
                s.augmentation.estimator.min_velocity_error_mps);
            opt(a, "max_linear_accel_mps2",
                s.augmentation.estimator.max_linear_accel_mps2);
            opt(a, "max_angular_accel_radps2",
                s.augmentation.estimator.max_angular_accel_radps2);
            opt(a, "max_learning_current_a",
                s.augmentation.estimator.max_learning_current_a);
            opt(a, "max_learning_temperature_c",
                s.augmentation.estimator.max_learning_temperature_c);
            opt(a, "max_vision_age_s",
                s.augmentation.estimator.max_vision_age_s);
            opt(a, "disturbance_accel_mps2",
                s.augmentation.estimator.disturbance_accel_mps2);
            opt(a, "confidence_threshold",
                s.augmentation.estimator.confidence_threshold);
            opt(a, "confidence_decay_s",
                s.augmentation.estimator.confidence_decay_s);
            opt(a, "parameter_decay_s",
                s.augmentation.estimator.parameter_decay_s);
            opt(a, "nominal_voltage_v",
                s.augmentation.estimator.nominal_voltage_v);
            opt(a, "default_drag_per_s",
                s.augmentation.estimator.default_drag_per_s);
            opt(a, "max_drag_per_s",
                s.augmentation.estimator.max_drag_per_s);
            opt(a, "correction_limit_fraction",
                s.augmentation.estimator.correction_limit_fraction);
            opt(a, "correction_slew_linear_mps2",
                s.augmentation.estimator.correction_slew_linear_mps2);
            opt(a, "correction_slew_angular_radps2",
                s.augmentation.estimator.correction_slew_angular_radps2);
            opt(a, "slip_compensation_gain",
                s.augmentation.estimator.slip_compensation_gain);
            opt(a, "max_delay_samples",
                s.augmentation.estimator.max_delay_samples);
            if (a["default_response_per_s"] &&
                a["default_response_per_s"].IsSequence() &&
                a["default_response_per_s"].size() == 3) {
                for (std::size_t i = 0; i < 3; ++i)
                    s.augmentation.estimator.default_response_per_s[i] =
                        a["default_response_per_s"][i].as<double>();
            }
            opt(a, "surface_id", s.surface_id);
            opt(a, "profile_path", s.surface_profile_path);
            opt(a, "profile_load", s.surface_profile_load);
            opt(a, "profile_save", s.surface_profile_save);
            opt(a, "profile_save_interval_s", s.surface_profile_save_interval_s);
        }
        if (root["rl"]) {
            const YAML::Node r = root["rl"];
            std::string mode = "off";
            opt(r, "mode", mode);
            s.augmentation.rl.mode = parse_rl_mode(mode);
            opt(r, "policy_path", s.augmentation.rl.policy_path);
            opt(r, "residual_limit_fraction",
                s.augmentation.rl.residual_limit_fraction);
            opt(r, "residual_slew_linear_mps2",
                s.augmentation.rl.residual_slew_linear_mps2);
            opt(r, "residual_slew_angular_radps2",
                s.augmentation.rl.residual_slew_angular_radps2);
            opt(r, "confidence_threshold",
                s.augmentation.rl.confidence_threshold);
            opt(r, "max_consecutive_interventions",
                s.augmentation.rl.max_consecutive_interventions);
            opt(r, "max_loop_elapsed_s",
                s.augmentation.rl.max_loop_elapsed_s);
        }
        if (root["augmentation_safety"]) {
            const YAML::Node a = root["augmentation_safety"];
            opt(a, "velocity_max_mps", s.augmentation.safety.velocity_max_mps);
            opt(a, "angular_velocity_max_radps",
                s.augmentation.safety.angular_velocity_max_radps);
            opt(a, "wheel_max_rev_s", s.augmentation.safety.wheel_max_rev_s);
            opt(a, "correction_limit_fraction",
                s.augmentation.safety.correction_limit_fraction);
            opt(a, "correction_jerk_linear_mps3",
                s.augmentation.safety.correction_jerk_linear_mps3);
            opt(a, "correction_jerk_angular_radps3",
                s.augmentation.safety.correction_jerk_angular_radps3);
            opt(a, "min_bus_voltage_v", s.augmentation.safety.min_bus_voltage_v);
            opt(a, "max_temperature_c", s.augmentation.safety.max_temperature_c);
            opt(a, "max_current_a", s.augmentation.safety.max_current_a);
        }
        if (root["motion_logging"]) {
            const YAML::Node l = root["motion_logging"];
            opt(l, "enabled", s.motion_log_enabled);
            opt(l, "directory", s.motion_log_directory);
            opt(l, "rate_hz", s.motion_log_rate_hz);
            opt(l, "max_file_mb", s.motion_log_max_file_mb);
            opt(l, "max_total_mb", s.motion_log_max_total_mb);
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
