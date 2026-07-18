// Shared MatchCtrl frame packers for host tests — build wire-format frames
// by hand (the encoder side lives in phoenix-server; these mirror its
// layouts exactly, as pinned by the golden vectors in test_matchctrl.cpp).
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace pack {

inline void put16(std::string& f, int off, int16_t v) {
    f[off] = static_cast<char>(v & 0xFF);
    f[off + 1] = static_cast<char>((v >> 8) & 0xFF);
}

inline std::string header(int robot, uint16_t seq, int skill_id) {
    std::string f(32, '\0');
    f[0] = 'P';
    f[1] = 'X';
    f[2] = 0x05;
    f[3] = static_cast<char>(robot);
    put16(f, 4, static_cast<int16_t>(seq));
    f[15] = static_cast<char>(skill_id);
    return f;
}

inline void vision(std::string& f, double x, double y, double th, int delay_q62) {
    put16(f, 6, static_cast<int16_t>(std::lround(x * 1000)));
    put16(f, 8, static_cast<int16_t>(std::lround(y * 1000)));
    put16(f, 10, static_cast<int16_t>(std::lround(th * 1000)));
    f[12] = static_cast<char>(delay_q62);
}

inline void no_vision(std::string& f) {
    put16(f, 6, 0x7FFF);
    put16(f, 8, 0x7FFF);
    put16(f, 10, 0x7FFF);
    f[12] = static_cast<char>(255);
}

// GLOBAL_POS with limits in raw u8 (vel/5, velW/30, acc/10, accW/100 of 255).
inline std::string global_pos(int robot, uint16_t seq, double vx, double vy, double vth,
                              double tx, double ty, double tth, uint8_t vel = 102,
                              uint8_t velw = 68, uint8_t acc = 64, uint8_t accw = 102,
                              int delay_q62 = 160) {
    std::string f = header(robot, seq, 4);
    vision(f, vx, vy, vth, delay_q62);
    put16(f, 16, static_cast<int16_t>(std::lround(tx * 1000)));
    put16(f, 18, static_cast<int16_t>(std::lround(ty * 1000)));
    put16(f, 20, static_cast<int16_t>(std::lround(tth * 1000)));
    f[22] = static_cast<char>(vel);
    f[23] = static_cast<char>(velw);
    f[24] = static_cast<char>(acc);
    f[25] = static_cast<char>(accw);
    f[29] = static_cast<char>(0x80);  // primary direction: none
    return f;
}

inline std::string global_pos_no_vision(int robot, uint16_t seq, double tx, double ty,
                                        double tth) {
    std::string f = header(robot, seq, 4);
    no_vision(f);
    put16(f, 16, static_cast<int16_t>(std::lround(tx * 1000)));
    put16(f, 18, static_cast<int16_t>(std::lround(ty * 1000)));
    put16(f, 20, static_cast<int16_t>(std::lround(tth * 1000)));
    f[22] = static_cast<char>(102);
    f[23] = static_cast<char>(68);
    f[24] = static_cast<char>(64);
    f[25] = static_cast<char>(102);
    f[29] = static_cast<char>(0x80);
    return f;
}

inline std::string local_vel(int robot, uint16_t seq, double vx, double vy, double w,
                             double px = 0, double py = 0, double pth = 0,
                             uint8_t acc = 102, uint8_t accw = 51) {
    std::string f = header(robot, seq, 2);
    vision(f, px, py, pth, 160);
    put16(f, 16, static_cast<int16_t>(std::lround(vx * 1000)));
    put16(f, 18, static_cast<int16_t>(std::lround(vy * 1000)));
    put16(f, 20, static_cast<int16_t>(std::lround(w * 1000)));
    f[22] = static_cast<char>(acc);
    f[23] = static_cast<char>(accw);
    f[24] = static_cast<char>(255);  // jerk maxed
    f[25] = static_cast<char>(255);
    return f;
}

inline std::string emergency(int robot, uint16_t seq, double px, double py, double pth) {
    std::string f = header(robot, seq, 0);
    vision(f, px, py, pth, 160);
    return f;
}

// The 3-byte KickerDribbler field at a skill-data offset.
inline void kd(std::string& f, int off, uint32_t word) {
    f[off] = static_cast<char>(word & 0xFF);
    f[off + 1] = static_cast<char>((word >> 8) & 0xFF);
    f[off + 2] = static_cast<char>((word >> 16) & 0xFF);
}

// KD word builder matching wire.py's bit layout.
inline uint32_t kd_word(unsigned speed_bits, unsigned device, unsigned mode,
                        unsigned drib_speed, unsigned drib_force) {
    return (speed_bits & 0x1FF) | ((device & 1) << 9) | ((mode & 3) << 10) |
           ((drib_speed & 0x3F) << 12) | ((drib_force & 0x3F) << 18);
}

}  // namespace pack
