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
    double imu_yaw_rate_sign = 1.0; // gyro polarity: +1 when the pi3hat gyro
                                    // reads positive for a CCW (+w) rotation
                                    // (VERIFIED +1 on Robot B — re-verify per robot)

    MatchBridgeConfig bridge;
    EstimatorConfig estimator;
    TrajectoryConfig trajectory;
    ControllerConfig controller;
    ActuatorConfig actuators;

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
