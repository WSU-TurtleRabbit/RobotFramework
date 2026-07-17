#pragma once

#include <string>

#include "phx/executor.h"

namespace rf {

// Everything config/Motion.yaml controls. Defaults (here and inside
// phx::MotionConfig) are the values proven on hardware in phoenix-rf's
// robot.yaml; the file only overrides what it names.
struct MotionSettings {
    bool enabled = true;            // false = ignore MV2 frames entirely
    double imu_yaw_rate_sign = 1.0; // gyro polarity: +1 when the pi3hat gyro
                                    // reads positive for a CCW (+w) rotation
    phx::MotionConfig config;
};

// Thin YAML shim (Pi build only — host tests construct MotionSettings
// directly). Missing file or keys fall back to the built-in defaults.
MotionSettings loadMotionSettings(const std::string& path);

}  // namespace rf
