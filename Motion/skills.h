// Onboard skill decode + dispatch — turns a MatchCtrl frame's skillId +
// skillData into ONE normalized motion setpoint + limits, the way TIGERs'
// skills.c produces a MotionSetpoint from the same layouts (Firmware
// src/robot/skills.c, skill_basics.c; layouts mirrored in phoenix-server
// phoenix/core/robot/wire.py).
//
// Pure and deterministic: no clocks, no sockets. The motion pipeline
// (estimator -> trajectory -> controller) consumes MotionSetpoint per tick;
// this module is the only place that knows the skill wire layouts.
//
// Semantics copied from TIGERs and the server's behavioral oracle
// (phoenix-server tests/fake_robot.py):
//   * limit FLOORS: a u8 raw of 0 must not freeze the robot — the oracle
//     floors velXY at 0.05 m/s, accXY at 0.05 m/s^2, velW at 0.1 rad/s;
//   * unknown / unsupported skill ids decode as Emergency (controlled stop)
//     — an undecodable command must never produce arbitrary motion;
//   * WHEEL_VEL arrives in TIGERs wire order FR,FL,RL,RR and is permuted
//     here into OUR CAN order (index i = motor id i+1: FR, RR, RL, FL);
//   * the KickerDribbler field rides out unchanged — actuation is the
//     Arduino layer's business.
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "../Networks/matchctrl.h"
#include "phx/pose.h"

namespace rf {

// Limit floors matching the behavioral oracle (fake_robot.py::_step): a
// raw 0 limit would otherwise mean "command received, never move".
inline constexpr double kSkillMinVelXY = 0.05;   // m/s
inline constexpr double kSkillMinAccXY = 0.05;   // m/s^2
inline constexpr double kSkillMinVelW = 0.1;     // rad/s
// Vel-skill floors (the oracle integrates vel skills instantly and ignores
// the limits; our shaper needs a nonzero bound or it latches).
inline constexpr double kSkillMinAccW = 0.1;     // rad/s^2
inline constexpr double kSkillMinJerkXY = 0.1;   // m/s^3
inline constexpr double kSkillMinJerkW = 1.0;    // rad/s^3

// The normalized per-tick motion command. SI units throughout
// (metres, radians, seconds). Exactly one alternative is active per kind.
struct MotionSetpoint {
    enum class Kind : uint8_t {
        Emergency,  // EMERGENCY (or undecodable skill): controlled ramp to
                    // zero, then motors off (TIGERs onboard semantics)
        Pose,       // GLOBAL_POS / FAST_POS: trajectory to a target pose
        LocalVel,   // LOCAL_VEL: body-frame velocity setpoint
        GlobalVel,  // GLOBAL_VEL: world-frame velocity (rotated per tick)
        WheelVel,   // WHEEL_VEL: raw per-wheel velocities (commissioning)
        Sine,       // SINE: open-loop sinusoidal local vels (system ID)
    };

    Kind kind = Kind::Emergency;
    int skill_id = 0;  // wire id this setpoint was decoded from

    // --- Pose (GLOBAL_POS / FAST_POS) ---
    phx::Pose target{};
    double vel_max_xy = 0.0;  // m/s
    double vel_max_w = 0.0;   // rad/s
    double acc_max_xy = 0.0;  // m/s^2
    double acc_max_w = 0.0;   // rad/s^2
    // FAST_POS: heading slaved to the drive direction; this raised accel
    // applies once aligned (more traction when wheels push straight).
    bool fast_pos = false;
    double acc_max_xy_fast = 0.0;
    // Preferred driving direction (rad, body frame); nullopt = synchronized
    // 2D (TIGERs PRIMARY_DIRECTION_NONE).
    std::optional<double> primary_direction;

    // --- LocalVel / GlobalVel ---
    phx::Twist vel{};          // m/s, m/s, rad/s (frame per kind)
    double acc_max_w_vel = 0.0;
    double jerk_max_xy = 0.0;  // m/s^3
    double jerk_max_w = 0.0;   // rad/s^3

    // --- WheelVel: CAN order (index i = motor id i+1: FR, RR, RL, FL) ---
    std::array<double, 4> wheel_rad_s{};  // rad/s at the wheel

    // --- Sine ---
    double sine_vx_amp = 0.0;  // m/s
    double sine_vy_amp = 0.0;  // m/s
    double sine_w_amp = 0.0;   // rad/s
    double sine_freq_hz = 0.0;

    // --- Kicker + dribbler (every motion skill; defaults = disarmed) ---
    KickerDribbler kd{};
};

// Decode one MatchCtrl frame into its normalized setpoint. Never fails:
// EMERGENCY carries no payload, and an unknown skillId maps to Emergency.
MotionSetpoint decode_skill(const MatchCtrl& mc);

// Permute TIGERs wire wheel order (FR, FL, RL, RR) into our CAN order
// (index i = motor id i+1: FR, RR, RL, FL — Math/kinematics.h). Exposed
// for the host tests; decode_skill applies it internally.
std::array<double, 4> wheel_order_wire_to_can(const std::array<double, 4>& wire);

}  // namespace rf
