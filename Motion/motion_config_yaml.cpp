#include "motion_config_yaml.h"

#include <iostream>

#include <yaml-cpp/yaml.h>

namespace rf {

namespace {

void load_mode(const YAML::Node& n, phx::ModeLimits& l) {
    if (!n) return;
    if (n["max_speed_mps"]) l.max_speed_mps = n["max_speed_mps"].as<double>();
    if (n["max_w_radps"]) l.max_w_radps = n["max_w_radps"].as<double>();
    if (n["max_accel_mps2"]) l.max_accel_mps2 = n["max_accel_mps2"].as<double>();
    if (n["max_w_accel_radps2"]) l.max_w_accel_radps2 = n["max_w_accel_radps2"].as<double>();
    if (n["max_jerk_mps3"]) l.max_jerk_mps3 = n["max_jerk_mps3"].as<double>();
    if (n["max_w_jerk_radps3"]) l.max_w_jerk_radps3 = n["max_w_jerk_radps3"].as<double>();
    if (n["kp_pos"]) l.kp_pos = n["kp_pos"].as<double>();
    if (n["kp_ang"]) l.kp_ang = n["kp_ang"].as<double>();
    if (n["arrive_radius_m"]) l.arrive_radius_m = n["arrive_radius_m"].as<double>();
    if (n["arrive_heading_rad"]) l.arrive_heading_rad = n["arrive_heading_rad"].as<double>();
}

}  // namespace

MotionSettings loadMotionSettings(const std::string& path) {
    MotionSettings s;
    try {
        YAML::Node root = YAML::LoadFile(path);

        if (YAML::Node imu = root["imu"]) {
            if (imu["yaw_rate_sign"]) s.imu_yaw_rate_sign = imu["yaw_rate_sign"].as<double>();
        }

        YAML::Node m = root["motion"];
        if (!m) return s;
        if (m["enabled"]) s.enabled = m["enabled"].as<bool>();
        phx::MotionConfig& c = s.config;
        if (m["brake_after_ms"]) c.brake_after_ms = m["brake_after_ms"].as<uint64_t>();
        if (m["coast_after_ms"]) c.coast_after_ms = m["coast_after_ms"].as<uint64_t>();
        if (m["brake_decel_mps2"]) c.brake_decel_mps2 = m["brake_decel_mps2"].as<double>();
        if (m["brake_w_decel_radps2"]) c.brake_w_decel_radps2 = m["brake_w_decel_radps2"].as<double>();
        if (m["brake_jerk_mps3"]) c.brake_jerk_mps3 = m["brake_jerk_mps3"].as<double>();
        if (m["pose_gain"]) c.pose_gain = m["pose_gain"].as<double>();
        if (m["heading_gain"]) c.heading_gain = m["heading_gain"].as<double>();
        if (m["snap_dist_m"]) c.snap_dist_m = m["snap_dist_m"].as<double>();
        load_mode(m["fast_travel"], c.fast_travel);
        load_mode(m["ball_approach"], c.ball_approach);
        load_mode(m["precision_align"], c.precision_align);
        load_mode(m["hold_position"], c.hold_position);
    } catch (const std::exception& e) {
        std::cerr << "Error loading Motion config: " << e.what()
                  << " (using built-in defaults)" << std::endl;
    }
    return s;
}

}  // namespace rf
