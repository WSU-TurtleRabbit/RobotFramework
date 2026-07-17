#include "decode.h"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace {

// Parse a full token as a finite double. Returns false on trailing garbage,
// empty input, or non-finite values ("nan"/"inf" must not drive motors).
bool parse_finite(const std::string& token, double& out) {
    if (token.empty()) return false;
    const char* begin = token.c_str();
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end != begin + token.size()) return false;
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
}

// Parse a full token as a non-negative integer robot id.
bool parse_id(const std::string& token, int& out) {
    if (token.empty()) return false;
    const char* begin = token.c_str();
    char* end = nullptr;
    const long v = std::strtol(begin, &end, 10);
    if (end != begin + token.size()) return false;
    if (v < 0 || v > 255) return false;
    out = static_cast<int>(v);
    return true;
}

}  // namespace

CmdType cmdDecoder::decode_cmd(const std::string& message) {
    // Fail safe: rejected packets must leave no motion behind. Zero the
    // motion fields up front and only fill them once the whole packet has
    // validated.
    velocity_x = 0.0;
    velocity_y = 0.0;
    velocity_w = 0.0;
    kick = false;
    dribble = false;

    // Whitespace tokenizer (splitting only; every field is then parsed
    // strictly from its own token).
    std::istringstream iss(message);
    std::vector<std::string> f;
    std::string token;
    while (iss >> token) f.push_back(token);

    // Bare opcodes.
    if (f.size() == 1) {
        if (f[0] == "STOP") return CmdType::Stop;
        if (f[0] == "PING") return CmdType::Ping;
        if (f[0] == "CALIBRATE") return CmdType::Calibrate;
        return CmdType::Malformed;
    }

    // v1 velocity frame: exactly 7 fields, all well-formed, or reject the
    // WHOLE packet.
    if (f.size() != 7) return CmdType::Malformed;

    int frame_id = -1;
    double vx = 0.0, vy = 0.0, w = 0.0, t = 0.0;
    if (!parse_id(f[0], frame_id)) return CmdType::Malformed;
    if (!parse_finite(f[1], vx)) return CmdType::Malformed;
    if (!parse_finite(f[2], vy)) return CmdType::Malformed;
    if (!parse_finite(f[3], w)) return CmdType::Malformed;
    // kick/dribble are serialized as bare 0/1; anything else than "1" is
    // false (matches the server encoder and the Rust robot's decoder).
    const bool frame_kick = (f[4] == "1");
    const bool frame_dribble = (f[5] == "1");
    if (!parse_finite(f[6], t)) return CmdType::Malformed;

    id = frame_id;
    time = t;

    // A frame for another robot is valid but must not move THIS robot.
    if (expected_id >= 0 && frame_id != expected_id) return CmdType::WrongId;

    velocity_x = vx;
    velocity_y = vy;
    velocity_w = w;
    kick = frame_kick;
    dribble = frame_dribble;
    return CmdType::Velocity;
}
