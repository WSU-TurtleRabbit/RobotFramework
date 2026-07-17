#pragma once

#include <string>

// Classification of one received command datagram. Anything that is not a
// well-formed frame addressed to this robot must never move the robot.
enum class CmdType {
    Malformed,  // wrong field count, unparseable / non-finite values, unknown text
    WrongId,    // well-formed frame addressed to a different robot
    Velocity,   // v1 body-velocity command; the decoded fields below are valid
    Move,       // MV2 pose-target frame — hand the RAW payload to MotionBridge
    Stop,       // bare opcode "STOP": safe-stop
    Ping,       // bare opcode "PING": link discovery, NOT a drive command
    Calibrate,  // bare opcode "CALIBRATE": operator-triggered self-heal
};

// Strict decoder for the v1 command channel (UDP text):
//   "<id> <vx> <vy> <w> <kick> <dribble> <time>"   (exactly 7 fields)
// plus the bare opcodes STOP / PING / CALIBRATE.
//
// Contract: a packet either decodes completely or is rejected as a whole.
// The previous implementation streamed fields with istringstream>> which left
// STALE values from earlier packets on short/malformed input and the robot
// drove on them; this decoder never partially applies a packet, and zeroes
// the motion fields on any rejection so stale state cannot leak.
class cmdDecoder {
public:
    // Decoded fields — only meaningful when decode_cmd() returned Velocity.
    int id = -1;
    double velocity_x = 0.0;  // body forward, m/s
    double velocity_y = 0.0;  // body strafe-left, m/s
    double velocity_w = 0.0;  // yaw rate, rad/s, CCW+
    bool kick = false;
    bool dribble = false;
    double time = 0.0;        // sender timestamp, informational only

    // Robot id this decoder accepts (config/Main.yaml `Robot_id`).
    // -1 = accept any id (default, matches the historical behavior).
    int expected_id = -1;

    CmdType decode_cmd(const std::string& message);
};
