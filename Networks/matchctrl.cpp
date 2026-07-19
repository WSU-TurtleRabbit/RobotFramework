// MatchCtrl / MatchFeedback codec — see matchctrl.h for the contract. Every
// offset, size, scale, and sentinel mirrors phoenix-server
// phoenix/core/robot/wire.py exactly; the golden vectors in
// tests/host/test_matchctrl.cpp pin the byte-level agreement.
#include "matchctrl.h"

#include <algorithm>
#include <cmath>

namespace rf {
namespace {

// Explicit little-endian accessors — never memcpy into packed structs
// (padding/endianness assumptions are how wire protocols rot).
int16_t rd_i16(const uint8_t* p) {
    const uint32_t v = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8);
    return static_cast<int16_t>(v >= 0x8000 ? static_cast<int32_t>(v) - 0x10000
                                            : static_cast<int32_t>(v));
}

uint16_t rd_u16(const uint8_t* p) {
    return static_cast<uint16_t>(static_cast<uint32_t>(p[0]) |
                                 (static_cast<uint32_t>(p[1]) << 8));
}

void wr_i16(uint8_t* p, int16_t v) {
    const uint16_t u = static_cast<uint16_t>(v);
    p[0] = static_cast<uint8_t>(u & 0xFF);
    p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
}

void wr_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// wire.py _unscale_u8: raw * (MAX / 255).
double unscale_u8(uint8_t raw, double max_value) {
    return static_cast<double>(raw) * (max_value / 255.0);
}

// wire.py _mm: round(clamp(v * 1000, -32767, 32766)).
int16_t wire_mm(double v) {
    return static_cast<int16_t>(std::lround(std::clamp(v * 1000.0, -32767.0, 32766.0)));
}

double i16_to_m(int16_t v) { return static_cast<double>(v) / 1000.0; }

}  // namespace

KickerDribbler KickerDribbler::decode(const uint8_t* data) {
    const uint32_t word = static_cast<uint32_t>(data[0]) |
                          (static_cast<uint32_t>(data[1]) << 8) |
                          (static_cast<uint32_t>(data[2]) << 16);
    const uint32_t speed_bits = word & 0x1FF;
    KickerDribbler kd;
    kd.kick_device = static_cast<KickerDevice>((word >> 9) & 0x1);
    kd.kick_mode = static_cast<KickerMode>((word >> 10) & 0x3);
    if (kd.kick_mode == KickerMode::ArmTime) {
        kd.kick_time_us = static_cast<double>(speed_bits) * kKickTimeScaleUs;
        kd.kick_speed = 0.0;
    } else {
        kd.kick_speed = static_cast<double>(speed_bits) * kKickSpeedScale;
        kd.kick_time_us = 0.0;
    }
    kd.dribbler_speed = static_cast<double>((word >> 12) & 0x3F) * kDribblerSpeedScale;
    kd.dribbler_force = static_cast<double>((word >> 18) & 0x3F) * kDribblerForceScale;
    return kd;
}

std::optional<MatchCtrl> decode_match_ctrl(const uint8_t* data, std::size_t len) {
    if (len != kMatchCtrlSize) return std::nullopt;
    if (data[0] != kWireMagic0 || data[1] != kWireMagic1) return std::nullopt;
    if (data[2] != kCmdMatchCtrl) return std::nullopt;

    MatchCtrl mc;
    mc.robot_id = data[3];
    mc.seq = rd_u16(data + 4);
    const uint8_t* body = data + kHeaderSize;
    const int16_t px = rd_i16(body + 0);
    const int16_t py = rd_i16(body + 2);
    const int16_t pw = rd_i16(body + 4);
    // wire.py: pose present iff BOTH planar axes are not the sentinel.
    if (px != kUnusedField && py != kUnusedField) {
        mc.vision_pose = VisionPose{i16_to_m(px), i16_to_m(py), i16_to_m(pw)};
    }
    mc.pos_delay_s = static_cast<double>(body[6]) * kPosDelayLsbS;
    mc.cam_id = body[7];
    mc.flags = body[8];
    mc.skill_id = body[9];
    std::copy_n(body + 10, kSkillDataSize, mc.skill_data.begin());
    return mc;
}

std::optional<MatchCtrl> decode_match_ctrl(const std::string& datagram) {
    return decode_match_ctrl(reinterpret_cast<const uint8_t*>(datagram.data()),
                             datagram.size());
}

GlobalPosSkill decode_global_pos(const std::array<uint8_t, kSkillDataSize>& d) {
    // "<3h4B3sb": 3 int16 (mm, mm, mrad), 4 u8 limits, 3 B KD, int8 primdir.
    GlobalPosSkill s;
    s.tx = i16_to_m(rd_i16(d.data() + 0));
    s.ty = i16_to_m(rd_i16(d.data() + 2));
    s.ttheta = i16_to_m(rd_i16(d.data() + 4));
    s.vel_max_xy = unscale_u8(d[6], kGlobalPosMaxVelXY);
    s.vel_max_w = unscale_u8(d[7], kGlobalPosMaxVelW);
    s.acc_max_xy = unscale_u8(d[8], kGlobalPosMaxAccXY);
    s.acc_max_w = unscale_u8(d[9], kGlobalPosMaxAccW);
    s.kd = KickerDribbler::decode(d.data() + 10);
    const int8_t pd = static_cast<int8_t>(d[13]);
    if (pd != -128) {
        // raw/127 * pi (TIGERs PRIMARY_DIRECTION_NONE = -128).
        s.primary_direction = static_cast<double>(pd) / 127.0 * 3.14159265358979323846;
    }
    return s;
}

FastPosSkill decode_fast_pos(const std::array<uint8_t, kSkillDataSize>& d) {
    // "<3h5B3s": GLOBAL_POS layout + a fifth u8 (fast accel), then KD.
    FastPosSkill s;
    s.tx = i16_to_m(rd_i16(d.data() + 0));
    s.ty = i16_to_m(rd_i16(d.data() + 2));
    s.ttheta = i16_to_m(rd_i16(d.data() + 4));
    s.vel_max_xy = unscale_u8(d[6], kGlobalPosMaxVelXY);
    s.vel_max_w = unscale_u8(d[7], kGlobalPosMaxVelW);
    s.acc_max_xy = unscale_u8(d[8], kGlobalPosMaxAccXY);
    s.acc_max_w = unscale_u8(d[9], kGlobalPosMaxAccW);
    s.acc_max_xy_fast = unscale_u8(d[10], kGlobalPosMaxAccXY);
    s.kd = KickerDribbler::decode(d.data() + 11);
    return s;
}

VelSkill decode_vel(const std::array<uint8_t, kSkillDataSize>& d) {
    // "<3h4B3s": int16 vx,vy (mm/s), w (mrad/s); u8 acc/jerk limits; 3 B KD.
    VelSkill s;
    s.vx = i16_to_m(rd_i16(d.data() + 0));
    s.vy = i16_to_m(rd_i16(d.data() + 2));
    s.w = i16_to_m(rd_i16(d.data() + 4));
    s.acc_max_xy = unscale_u8(d[6], kLocalVelMaxAccXY);
    s.acc_max_w = unscale_u8(d[7], kLocalVelMaxAccW);
    s.jerk_max_xy = unscale_u8(d[8], kMaxJerkXY);
    s.jerk_max_w = unscale_u8(d[9], kMaxJerkW);
    s.kd = KickerDribbler::decode(d.data() + 10);
    return s;
}

WheelVelSkill decode_wheel_vel(const std::array<uint8_t, kSkillDataSize>& d) {
    // "<4h3s": four int16 at 0.005 rad/s, wire order FR, FL, RL, RR, then KD.
    WheelVelSkill s;
    for (int i = 0; i < 4; ++i) {
        s.wheel_rad_s[i] = static_cast<double>(rd_i16(d.data() + 2 * i)) * kWheelVelScale;
    }
    s.kd = KickerDribbler::decode(d.data() + 8);
    return s;
}

SineSkill decode_sine(const std::array<uint8_t, kSkillDataSize>& d) {
    // "<3hH": int16 vx,vy (mm/s), w (mrad/s) amplitudes; u16 freq (mHz).
    SineSkill s;
    s.vx_amp = i16_to_m(rd_i16(d.data() + 0));
    s.vy_amp = i16_to_m(rd_i16(d.data() + 2));
    s.w_amp = i16_to_m(rd_i16(d.data() + 4));
    s.freq_hz = static_cast<double>(rd_u16(d.data() + 6)) / 1000.0;
    return s;
}

std::array<uint8_t, kMatchFeedbackSize> encode_match_feedback(const MatchFeedback& fb) {
    std::array<uint8_t, kMatchFeedbackSize> out{};
    out[0] = kWireMagic0;
    out[1] = kWireMagic1;
    out[2] = kCmdMatchFeedback;
    out[3] = static_cast<uint8_t>(fb.robot_id & 0xFF);
    wr_u16(out.data() + 4, fb.seq);

    uint8_t* b = out.data() + kHeaderSize;
    // "<3h3hBBBBBBHBB2hBBB" — 29 bytes.
    wr_i16(b + 0, wire_mm(fb.pos_x));
    wr_i16(b + 2, wire_mm(fb.pos_y));
    wr_i16(b + 4, wire_mm(fb.heading));
    wr_i16(b + 6, wire_mm(fb.vel_x));
    wr_i16(b + 8, wire_mm(fb.vel_y));
    wr_i16(b + 10, wire_mm(fb.ang_vel));
    // wire.py truncates (int(...)) the kicker voltages, rounds the rest.
    b[12] = static_cast<uint8_t>(std::clamp(fb.kicker_level_v, 0.0, 255.0));
    b[13] = static_cast<uint8_t>(std::clamp(fb.kicker_max_v, 0.0, 255.0));
    const uint8_t drib_speed = static_cast<uint8_t>(
        std::lround(std::clamp(fb.dribbler_speed / 0.25, 0.0, 63.0)));
    b[14] = static_cast<uint8_t>((drib_speed << 2) | static_cast<uint8_t>(fb.dribble_traction));
    b[15] = static_cast<uint8_t>(
        std::lround(std::clamp(fb.battery_v * 10.0, 0.0, 255.0)));
    b[16] = static_cast<uint8_t>(
        std::lround(std::clamp(fb.battery_percent / 100.0 * 255.0, 0.0, 255.0)));
    const uint8_t flags = static_cast<uint8_t>(
        (fb.barrier_interrupted ? kFlagBarrierInterrupted : 0) |
        ((fb.dribbler_temp_class & 0x3) << kDribblerTempShift) |
        (fb.kick_counter ? kFlagKickCounter : 0) |
        (fb.ball_state & kBallStateMask));
    b[17] = flags;
    wr_u16(b + 18, fb.features);
    b[20] = static_cast<uint8_t>(fb.hardware_id & 0xFF);
    b[21] = static_cast<uint8_t>(std::clamp(fb.ball_pos_age_ms, 0, 255));
    const double bx = fb.ball_pos ? fb.ball_pos->first : 0.0;
    const double by = fb.ball_pos ? fb.ball_pos->second : 0.0;
    wr_i16(b + 22, wire_mm(bx));
    wr_i16(b + 24, wire_mm(by));
    b[26] = 0;  // lastKickDuration — not measured yet
    b[27] = 0;  // lastKickDribbleVelDev
    b[28] = 0;  // lastKickDribbleForce
    return out;
}

}  // namespace rf
