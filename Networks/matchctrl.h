// Phoenix MatchCtrl / MatchFeedback binary protocol — the robot side of
// phoenix-server/phoenix/core/robot/wire.py (the AUTHORITATIVE source; a
// faithful adaptation of TIGERs Mannheim's TigerSystemMatchCtrl /
// SystemMatchFeedback, Firmware src/shared/commands.h, 2026 release).
//
// Every datagram = 6-byte header (magic "PX", command, robot id, seq LE) +
// body. MatchCtrl (0x05, server -> robot, 26-byte body) carries the freshest
// vision pose + its measured age, a skillId, and 16 bytes of skill data.
// MatchFeedback (0x06, robot -> server, 29-byte body) reports the robot's own
// pose/velocity estimate, kicker/dribbler state, battery, barrier, and
// health features.
//
// All multi-byte integers are little-endian. Internally this module speaks
// SI units (metres, radians, seconds); the wire is mm/mrad/Q6.2-ms — the
// conversions live EXACTLY at these decode/encode boundaries.
//
// Interoperability is a hard constraint: tests/host/test_matchctrl.cpp pins
// golden byte vectors produced by wire.py (regenerate with
// tests/host/golden_matchctrl.py). Change wire.py and this module together,
// or not at all.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace rf {

// --------------------------------------------------------------------------
// Framing (wire.py: _HEADER = "<2sBBH")
// --------------------------------------------------------------------------
inline constexpr uint8_t kWireMagic0 = 'P';
inline constexpr uint8_t kWireMagic1 = 'X';
inline constexpr uint8_t kCmdMatchCtrl = 0x05;      // TIGERs CMD_SYSTEM_MATCH_CTRL
inline constexpr uint8_t kCmdMatchFeedback = 0x06;  // TIGERs CMD_SYSTEM_MATCH_FEEDBACK

inline constexpr std::size_t kHeaderSize = 6;
inline constexpr std::size_t kSkillDataSize = 16;
inline constexpr std::size_t kMatchCtrlBodySize = 10 + kSkillDataSize;  // 26
inline constexpr std::size_t kMatchCtrlSize = kHeaderSize + kMatchCtrlBodySize;  // 32
inline constexpr std::size_t kMatchFeedbackBodySize = 29;
inline constexpr std::size_t kMatchFeedbackSize = kHeaderSize + kMatchFeedbackBodySize;  // 35

// Sentinels (wire.py: UNUSED_FIELD / POS_DELAY_NONE).
inline constexpr int16_t kUnusedField = 0x7FFF;   // pose axis: "no fresh vision"
inline constexpr uint8_t kPosDelayNone = 255;     // posDelay: none / saturated
inline constexpr double kPosDelayLsbS = 0.00025;  // Q6.2 ms -> seconds (250 us)

// --------------------------------------------------------------------------
// Skill ids + kicker/dribbler (TIGERs skills.c / BasicKDInput, verbatim)
// --------------------------------------------------------------------------
enum class SkillId : uint8_t {
    Emergency = 0,
    WheelVel = 1,
    LocalVel = 2,
    GlobalVel = 3,
    GlobalPos = 4,
    GlobalVelAndOrient = 5,  // reserved by TIGERs; no layout in wire.py
    Sine = 6,
    FastPos = 7,
};

enum class KickerMode : uint8_t {
    Disarm = 0,  // clears
    Arm = 1,     // fires on the robot's own ball-contact (break-beam) signal
    Force = 2,   // fires immediately
    ArmTime = 3, // arm with a raw discharge duration
};

enum class KickerDevice : uint8_t { Straight = 0, Chip = 1 };

// Lower 2 bits of the MatchFeedback dribblerState byte
// (Sumatra EDribbleTractionState; numeric assignment pinned with the server).
enum class DribbleTraction : uint8_t { Off = 0, Idle = 1, Light = 2, Strong = 3 };

// Limit scaling (wire.py "TIGERs' exact values"): a u8 raw means
// raw * (MAX / 255). SI decoded values below already carry these.
inline constexpr double kLocalVelMaxAccXY = 10.0;    // m/s^2
inline constexpr double kLocalVelMaxAccW = 100.0;    // rad/s^2
inline constexpr double kGlobalPosMaxVelXY = 5.0;    // m/s
inline constexpr double kGlobalPosMaxVelW = 30.0;    // rad/s
inline constexpr double kGlobalPosMaxAccXY = 10.0;   // m/s^2
inline constexpr double kGlobalPosMaxAccW = 100.0;   // rad/s^2
inline constexpr double kMaxJerkXY = 100.0;          // m/s^3
inline constexpr double kMaxJerkW = 1000.0;          // rad/s^3
inline constexpr double kWheelVelScale = 0.005;      // raw int16 -> rad/s at wheel

inline constexpr double kKickSpeedScale = 0.02;      // m/s per bit (9 bits)
inline constexpr double kKickTimeScaleUs = 25.0;     // us per bit (ARM_TIME)
inline constexpr double kDribblerSpeedScale = 0.125; // m/s per bit (6 bits)
inline constexpr double kDribblerForceScale = 0.25;  // N per bit (6 bits)

// The 24-bit kicker/dribbler field riding in every motion skill
// (wire.py KickerDribbler). Bits 0-8 kick speed/time, bit 9 device,
// bits 10-11 mode, bits 12-17 dribbler speed, bits 18-23 dribbler force.
struct KickerDribbler {
    KickerMode kick_mode = KickerMode::Disarm;
    KickerDevice kick_device = KickerDevice::Straight;
    double kick_speed = 0.0;    // m/s (0 in ArmTime mode)
    double kick_time_us = 0.0;  // only in ArmTime mode
    double dribbler_speed = 0.0;  // bar surface m/s
    double dribbler_force = 0.0;  // N

    // Decode the 3 wire bytes at `data` (little-endian 24-bit word).
    static KickerDribbler decode(const uint8_t* data);
};

// --------------------------------------------------------------------------
// MatchCtrl (server -> robot)
// --------------------------------------------------------------------------

// Vision pose in SI units (wire mm/mrad converted at the boundary).
struct VisionPose {
    double x = 0.0;        // m, global frame
    double y = 0.0;        // m
    double heading = 0.0;  // rad
};

struct MatchCtrl {
    int robot_id = 0;
    uint16_t seq = 0;
    // Fresh vision pose; nullopt when the frame carries the UNUSED_FIELD
    // sentinel ("no fresh vision — dead-reckon").
    std::optional<VisionPose> vision_pose;
    double pos_delay_s = 0.0;  // measured age of the pose, seconds (Q6.2 ms wire)
    int cam_id = 0;
    int flags = 0;
    int skill_id = 0;
    std::array<uint8_t, kSkillDataSize> skill_data{};
};

// Strict decode of one MatchCtrl datagram. Returns nullopt unless the frame
// is EXACTLY kMatchCtrlSize bytes with the right magic and command — a
// malformed packet must never move the robot.
std::optional<MatchCtrl> decode_match_ctrl(const uint8_t* data, std::size_t len);
std::optional<MatchCtrl> decode_match_ctrl(const std::string& datagram);

// --------------------------------------------------------------------------
// Skill data payloads (the 16-byte MatchCtrl.skill_data field)
// --------------------------------------------------------------------------

// GLOBAL_POS (4), wire "<3h4B3sb" (14 B): target pose + limits + KD.
struct GlobalPosSkill {
    double tx = 0.0, ty = 0.0, ttheta = 0.0;  // m, m, rad
    double vel_max_xy = 0.0;                  // m/s
    double vel_max_w = 0.0;                   // rad/s
    double acc_max_xy = 0.0;                  // m/s^2
    double acc_max_w = 0.0;                   // rad/s^2
    KickerDribbler kd;
    // Preferred driving direction, rad (-128 wire = synchronized 2D).
    std::optional<double> primary_direction;
};

// FAST_POS (7), wire "<3h5B3s" (14 B): GLOBAL_POS + raised accel once the
// heading is slaved to the drive direction.
struct FastPosSkill {
    double tx = 0.0, ty = 0.0, ttheta = 0.0;
    double vel_max_xy = 0.0;
    double vel_max_w = 0.0;
    double acc_max_xy = 0.0;
    double acc_max_w = 0.0;
    double acc_max_xy_fast = 0.0;  // m/s^2, applies once aligned
    KickerDribbler kd;
};

// LOCAL_VEL (2) / GLOBAL_VEL (3), wire "<3h4B3s" (13 B): velocity setpoint +
// accel/jerk limits. LOCAL is body-frame; GLOBAL is world-frame (the robot
// rotates it by its own heading estimate each tick).
struct VelSkill {
    double vx = 0.0, vy = 0.0;  // m/s
    double w = 0.0;             // rad/s
    double acc_max_xy = 0.0;    // m/s^2
    double acc_max_w = 0.0;     // rad/s^2
    double jerk_max_xy = 0.0;   // m/s^3
    double jerk_max_w = 0.0;    // rad/s^3
    KickerDribbler kd;
};

// WHEEL_VEL (1), wire "<4h3s" (11 B): raw per-wheel velocities, rad/s at the
// wheel, in TIGERs wire order FR, FL, RL, RR. NOTE: this is NOT our CAN
// order (ours is FR, RR, RL, FL for ids 1..4) — the motion layer permutes.
struct WheelVelSkill {
    std::array<double, 4> wheel_rad_s{};  // [FR, FL, RL, RR]
    KickerDribbler kd;
};

// SINE (6), wire "<3hH" (8 B): open-loop sinusoidal local velocities for
// system identification. No KD field.
struct SineSkill {
    double vx_amp = 0.0, vy_amp = 0.0;  // m/s amplitudes
    double w_amp = 0.0;                 // rad/s amplitude
    double freq_hz = 0.0;
};

GlobalPosSkill decode_global_pos(const std::array<uint8_t, kSkillDataSize>& d);
FastPosSkill decode_fast_pos(const std::array<uint8_t, kSkillDataSize>& d);
VelSkill decode_vel(const std::array<uint8_t, kSkillDataSize>& d);
WheelVelSkill decode_wheel_vel(const std::array<uint8_t, kSkillDataSize>& d);
SineSkill decode_sine(const std::array<uint8_t, kSkillDataSize>& d);

// --------------------------------------------------------------------------
// MatchFeedback (robot -> server)
// --------------------------------------------------------------------------

// features health bits (OUR assignment, pinned with the server).
inline constexpr uint16_t kFeatureMove = 0x0001;
inline constexpr uint16_t kFeatureDribbler = 0x0002;
inline constexpr uint16_t kFeatureBarrier = 0x0004;
inline constexpr uint16_t kFeatureKickStraight = 0x0008;
inline constexpr uint16_t kFeatureKickChip = 0x0010;

// flags byte layout (wire.py / TIGERs robot.c::sendMatchFeedback).
inline constexpr uint8_t kFlagBarrierInterrupted = 0x80;
inline constexpr int kDribblerTempShift = 5;  // bits 6-5
inline constexpr uint8_t kFlagKickCounter = 0x10;
inline constexpr uint8_t kBallStateMask = 0x07;

struct MatchFeedback {
    int robot_id = 0;
    uint16_t seq = 0;
    // The robot's own estimate — ideally the estimator's delayed slot matched
    // to the vision timepoint (TIGERs semantics). SI units.
    double pos_x = 0.0, pos_y = 0.0, heading = 0.0;  // m, m, rad
    double vel_x = 0.0, vel_y = 0.0;                 // m/s, global frame
    double ang_vel = 0.0;                            // rad/s
    double kicker_level_v = 0.0;
    double kicker_max_v = 0.0;
    double dribbler_speed = 0.0;  // bar surface m/s
    DribbleTraction dribble_traction = DribbleTraction::Off;
    double battery_v = 0.0;
    double battery_percent = 0.0;  // 0..100
    bool barrier_interrupted = false;
    int dribbler_temp_class = 0;  // 0 low .. 3 overheated
    bool kick_counter = false;    // toggles per kick: edge = a kick happened
    int ball_state = 0;           // 0..7, ball-observation state
    uint16_t features = 0;
    int hardware_id = 0;
    int ball_pos_age_ms = 255;  // 255 = no onboard ball estimate
    std::optional<std::pair<double, double>> ball_pos;  // m, nullopt if unused
};

// Encode one MatchFeedback datagram (kMatchFeedbackSize bytes). Wire
// quantization: mm/mrad int16 (clamped to +-32766, mirroring wire.py's _mm),
// kicker volts u8, dribbler speed 0.25 m/s units, battery dV, percent 0..255.
std::array<uint8_t, kMatchFeedbackSize> encode_match_feedback(const MatchFeedback& fb);

}  // namespace rf
