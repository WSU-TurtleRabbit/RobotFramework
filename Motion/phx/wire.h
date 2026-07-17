// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3. This is the ROBOT side
// of that repo's phoenix/robot/wire.h: the Mv2Command struct and enums are
// copied verbatim; the strict MV2 parser mirrors the field-proven Rust
// decoder in phoenix-rf (crates/protocol/src/wire.rs).
//
// Robot radio wire protocol:
//   commands  -> robot port 50514: legacy v1 velocity frames, MV2 pose-target
//                frames, and the bare opcodes PING / STOP / CALIBRATE.
//   telemetry <- robot port 50513: key=value CSV, protocol v2 (v1 accepted).
//
// The MV2 line format and the telemetry key set are PINNED across repos:
// phoenix-rf crates/protocol/src/wire.rs and phoenix-core
// core/include/phoenix/robot/wire.h hold the same golden strings. Change all
// of them together or none.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace phx {

enum class MoveKind { Move, Hold, Brake, Disable };

// Limit-profile selector understood by the onboard executor.
enum class MoveMode { FastTravel, BallApproach, PrecisionAlign, HoldPosition, Brake };

// One MV2 pose-target frame (server -> robot). Units: metres, radians,
// seconds; world = SSL-Vision frame. Zero caps mean "use the mode profile".
//
// Wire form (23 whitespace-separated fields):
//   MV2 <id> <seq> <KIND> <MODE> <px> <py> <ptheta> <vx> <vy> <vw>
//       <tx> <ty> <ttheta> <max_speed> <max_w> <max_accel> <max_jerk>
//       <arrive_speed> <kick> <dribble> <pose_age_ms> <time_set>
struct Mv2Command {
    uint32_t robot_id = 0;
    uint32_t seq = 0;
    MoveKind kind = MoveKind::Brake;
    MoveMode mode = MoveMode::FastTravel;
    // Current world pose + velocity estimate from vision (robot fuses these).
    double px = 0, py = 0, ptheta = 0;
    double vx = 0, vy = 0, vw = 0;
    // Target world pose.
    double tx = 0, ty = 0, ttheta = 0;
    // Per-command caps (0 = profile default).
    double max_speed_mps = 0;
    double max_w_radps = 0;
    double max_accel_mps2 = 0;
    double max_jerk_mps3 = 0;
    // Speed to carry through the target (0 = stop there).
    double arrive_speed_mps = 0;
    bool kick = false;
    bool dribble = false;
    // Vision capture -> send age of the pose fields, milliseconds.
    uint32_t pose_age_ms = 0;
    // Sender wall clock, informational.
    double time_set = 0;
};

// Strict decode of one MV2 line: exact token count, known words, finite
// floats — or nullopt. A malformed packet must never move the robot; this
// parser never partially applies one.
std::optional<Mv2Command> parse_mv2(std::string_view payload);

// Wire words for the enums (used by logging and telemetry).
const char* kind_word(MoveKind k);
const char* mode_word(MoveMode m);

// Compact float rendering shared with the server's encoders: up to 4
// decimals, trimmed, "nan"/"inf" for non-finite. Matches the Rust side's
// push_kv_f exactly (telemetry v2 uses it).
std::string fmt_wire_double(double v);

}  // namespace phx
