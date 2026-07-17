// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3. Parse semantics mirror
// phoenix-rf crates/protocol/src/wire.rs `MoveCommand::from_wire` — both
// repos pin the same golden wire string in their tests. Number parsing uses
// strtod/strtoul (not std::from_chars<double>) so the Pi's GCC 10 builds it.
#include "wire.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace phx {

namespace {

constexpr size_t kMoveFieldCount = 23;

// Split on any whitespace (the wire uses single spaces; be tolerant).
std::vector<std::string> tokenize(std::string_view s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        const size_t start = i;
        while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        if (i > start) out.emplace_back(s.substr(start, i - start));
    }
    return out;
}

// Full-token double; must consume every character.
bool parse_double(const std::string& t, double& out) {
    if (t.empty()) return false;
    const char* begin = t.c_str();
    char* end = nullptr;
    out = std::strtod(begin, &end);
    return end == begin + t.size();
}

// Full-token FINITE double (motion fields; "nan"/"inf" must not drive).
bool parse_finite(const std::string& t, double& out) {
    return parse_double(t, out) && std::isfinite(out);
}

// Full-token unsigned 32-bit integer (rejects sign characters, like Rust's
// u32 parse; strtoul would silently wrap "-1").
bool parse_u32(const std::string& t, uint32_t& out) {
    if (t.empty() || !std::isdigit(static_cast<unsigned char>(t[0]))) return false;
    const char* begin = t.c_str();
    char* end = nullptr;
    const unsigned long v = std::strtoul(begin, &end, 10);
    if (end != begin + t.size()) return false;
    if (v > 0xFFFFFFFFul) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

}  // namespace

std::optional<Mv2Command> parse_mv2(std::string_view payload) {
    const std::vector<std::string> f = tokenize(payload);
    if (f.size() != kMoveFieldCount || f[0] != "MV2") return std::nullopt;

    Mv2Command c;

    if (!parse_u32(f[1], c.robot_id) || c.robot_id > 255) return std::nullopt;
    if (!parse_u32(f[2], c.seq)) return std::nullopt;

    if (f[3] == "MOVE") c.kind = MoveKind::Move;
    else if (f[3] == "HOLD") c.kind = MoveKind::Hold;
    else if (f[3] == "BRAKE") c.kind = MoveKind::Brake;
    else if (f[3] == "DISABLE") c.kind = MoveKind::Disable;
    else return std::nullopt;

    if (f[4] == "FAST") c.mode = MoveMode::FastTravel;
    else if (f[4] == "BALL") c.mode = MoveMode::BallApproach;
    else if (f[4] == "ALIGN") c.mode = MoveMode::PrecisionAlign;
    else if (f[4] == "HOLD") c.mode = MoveMode::HoldPosition;
    else if (f[4] == "BRAKE") c.mode = MoveMode::Brake;
    else return std::nullopt;

    if (!parse_finite(f[5], c.px)) return std::nullopt;
    if (!parse_finite(f[6], c.py)) return std::nullopt;
    if (!parse_finite(f[7], c.ptheta)) return std::nullopt;
    if (!parse_finite(f[8], c.vx)) return std::nullopt;
    if (!parse_finite(f[9], c.vy)) return std::nullopt;
    if (!parse_finite(f[10], c.vw)) return std::nullopt;
    if (!parse_finite(f[11], c.tx)) return std::nullopt;
    if (!parse_finite(f[12], c.ty)) return std::nullopt;
    if (!parse_finite(f[13], c.ttheta)) return std::nullopt;
    if (!parse_finite(f[14], c.max_speed_mps)) return std::nullopt;
    if (!parse_finite(f[15], c.max_w_radps)) return std::nullopt;
    if (!parse_finite(f[16], c.max_accel_mps2)) return std::nullopt;
    if (!parse_finite(f[17], c.max_jerk_mps3)) return std::nullopt;
    if (!parse_finite(f[18], c.arrive_speed_mps)) return std::nullopt;

    // kick/dribble are serialized as bare 0/1 (anything else than "1" is
    // false — matches the server encoder and the Rust decoder).
    c.kick = (f[19] == "1");
    c.dribble = (f[20] == "1");

    if (!parse_u32(f[21], c.pose_age_ms)) return std::nullopt;
    // time_set is informational; it still must parse (finiteness not
    // required — Rust accepts "nan" here).
    if (!parse_double(f[22], c.time_set)) return std::nullopt;

    return c;
}

const char* kind_word(MoveKind k) {
    switch (k) {
        case MoveKind::Move: return "MOVE";
        case MoveKind::Hold: return "HOLD";
        case MoveKind::Brake: return "BRAKE";
        case MoveKind::Disable: return "DISABLE";
    }
    return "BRAKE";
}

const char* mode_word(MoveMode m) {
    switch (m) {
        case MoveMode::FastTravel: return "FAST";
        case MoveMode::BallApproach: return "BALL";
        case MoveMode::PrecisionAlign: return "ALIGN";
        case MoveMode::HoldPosition: return "HOLD";
        case MoveMode::Brake: return "BRAKE";
    }
    return "FAST";
}

std::string fmt_wire_double(double v) {
    if (!std::isfinite(v)) {
        if (std::isnan(v)) return "nan";
        return v > 0 ? "inf" : "-inf";
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.4f", v);
    std::string s = buf;
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
    if (s.empty() || s == "-" || s == "-0") s = "0";
    return s;
}

}  // namespace phx
