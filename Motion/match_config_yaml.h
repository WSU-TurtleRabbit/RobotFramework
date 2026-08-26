// YAML shim for the MatchCtrl motion stack (Pi build only — host tests
// construct their configs directly). Defaults come from the modules
// themselves (TIGERs-derived, and the values proven on Robot B via
// phoenix-rf); config/Motion.yaml only overrides what it names.
#pragma once

#include <string>

#include "actuators.h"
#include "controller.h"
#include "estimator.h"
#include "match_bridge.h"
#include "trajectory.h"

namespace rf {

// Everything config/Motion.yaml controls for the MatchCtrl cascade.
struct MatchSettings {
    bool enabled = true;            // false = ignore MatchCtrl frames
    int imu_yaw_rate_axis = 2;      // 0=x/roll, 1=y/pitch, 2=z/yaw
    double imu_yaw_rate_sign = 1.0; // gyro polarity: +1 when the pi3hat gyro
                                    // reads positive for a CCW (+w) rotation
                                    // (VERIFIED +1 on Robot B — re-verify per robot)

    int imu_accel_forward_axis = -1; // -1 = unavailable until mounting verified
    int imu_accel_lateral_axis = -1;
    double imu_accel_forward_sign = 1.0;
    double imu_accel_lateral_sign = 1.0;

    MatchBridgeConfig bridge;
    EstimatorConfig estimator;
    TrajectoryConfig trajectory;
    ControllerConfig controller;
    ActuatorConfig actuators;
    AugmentationConfig augmentation;
    std::string surface_id = "default";
    std::string surface_profile_path = "../config/surface_profiles/robot.json";
    bool surface_profile_load = true;
    bool surface_profile_save = true;
    double surface_profile_save_interval_s = 5.0;

    bool motion_log_enabled = true;
    std::string motion_log_directory = "logs/motion_v1";
    double motion_log_rate_hz = 100.0;
    // Disk bounds (0 disables): rotate the active JSONL at max_file_mb and
    // delete the oldest motion_v1 logs beyond max_total_mb in the directory.
    double motion_log_max_file_mb = 64.0;
    double motion_log_max_total_mb = 512.0;

    // MatchFeedback knobs (flat; the superloop fills FeedbackHealth).
    int hardware_id = 0;
    double battery_empty_v = 10.0;
    double battery_full_v = 16.8;
    double kicker_max_v = 200.0;
    double kicker_recharge_s = 5.0;
    int feedback_interval_ms = 20;  // 50 Hz
};

MatchSettings loadMatchSettings(const std::string& path);

}  // namespace rf
