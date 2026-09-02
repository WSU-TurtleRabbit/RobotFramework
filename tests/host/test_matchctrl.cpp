// MatchCtrl/MatchFeedback golden-vector interop tests. Every hex string here
// was produced by the AUTHORITATIVE encoder in phoenix-server
// (phoenix/core/robot/wire.py) via tests/host/golden_matchctrl.py — the C++
// decoder must agree on every byte, and the C++ feedback encoder must
// reproduce the Python bytes exactly. Change both sides together or not at
// all.
#include <array>
#include <cstdint>
#include <cstdio>
#include <string>

#include "Motion/phx/testing.h"
#include "Networks/matchctrl.h"

using namespace rf;

namespace {

std::string from_hex(const char* hex) {
    std::string out;
    for (const char* p = hex; p[0] && p[1]; p += 2) {
        unsigned int b = 0;
        std::sscanf(p, "%2x", &b);
        out.push_back(static_cast<char>(b));
    }
    return out;
}

template <std::size_t N>
void check_hex(const std::array<uint8_t, N>& data, const char* want_hex) {
    std::string got;
    char buf[3];
    for (std::size_t i = 0; i < N; ++i) {
        std::snprintf(buf, sizeof(buf), "%02x", data[i]);
        got += buf;
    }
    if (got != want_hex) {
        std::printf("      got:  %s\n      want: %s\n", got.c_str(), want_hex);
        CHECK(false /* byte mismatch above */);
    }
}

constexpr double kPi = 3.14159265358979323846;

}  // namespace

// --- MatchCtrl decode -------------------------------------------------------

PHX_TEST(matchctrl_golden_global_pos_decodes) {
    // Python: robot 7, seq 42, pose (1.234, -0.567, 0.789), delay 12500 us,
    // cam 2, GLOBAL_POS target (1.24, -0.76, 1.57), limits 2.5/10/3/30,
    // KD = ARM CHIP 6.5 m/s + dribbler 3.0 m/s / 5.0 N, primary dir NONE.
    const auto mc = decode_match_ctrl(from_hex(
        "505805072a00d204c9fd150332020004d80408fd220680554c4c458751800000"));
    REQUIRE(mc.has_value());
    CHECK(mc->robot_id == 7);
    CHECK(mc->seq == 42);
    REQUIRE(mc->vision_pose.has_value());
    CHECK_NEAR(mc->vision_pose->x, 1.234, 1e-12);
    CHECK_NEAR(mc->vision_pose->y, -0.567, 1e-12);
    CHECK_NEAR(mc->vision_pose->heading, 0.789, 1e-12);
    CHECK_NEAR(mc->pos_delay_s, 0.0125, 1e-12);  // 50 * 250 us
    CHECK(mc->cam_id == 2);
    CHECK(mc->flags == 0);
    CHECK(mc->skill_id == static_cast<int>(SkillId::GlobalPos));

    const GlobalPosSkill s = decode_global_pos(mc->skill_data);
    CHECK_NEAR(s.tx, 1.240, 1e-12);
    CHECK_NEAR(s.ty, -0.760, 1e-12);
    CHECK_NEAR(s.ttheta, 1.570, 1e-12);
    CHECK_NEAR(s.vel_max_xy, 128.0 * (5.0 / 255.0), 1e-12);
    CHECK_NEAR(s.vel_max_w, 85.0 * (30.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_xy, 76.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_w, 76.0 * (100.0 / 255.0), 1e-12);
    CHECK(!s.primary_direction.has_value());  // wire -128 = synchronized 2D

    CHECK(s.kd.kick_mode == KickerMode::Arm);
    CHECK(s.kd.kick_device == KickerDevice::Chip);
    CHECK_NEAR(s.kd.kick_speed, 6.5, 1e-12);        // 325 * 0.02
    CHECK_NEAR(s.kd.dribbler_speed, 3.0, 1e-12);    // 24 * 0.125
    CHECK_NEAR(s.kd.dribbler_force, 5.0, 1e-12);    // 20 * 0.25
    CHECK_NEAR(s.kd.kick_time_us, 0.0, 1e-12);
}

PHX_TEST(matchctrl_golden_primdir_decodes) {
    // GLOBAL_POS with primary_direction = pi/2 (wire raw 64).
    const auto mc = decode_match_ctrl(from_hex(
        "50580501070000000000000014000004f401f40170fe66223333000000400000"));
    REQUIRE(mc.has_value());
    CHECK(mc->skill_id == static_cast<int>(SkillId::GlobalPos));
    const GlobalPosSkill s = decode_global_pos(mc->skill_data);
    CHECK_NEAR(s.tx, 0.5, 1e-12);
    CHECK_NEAR(s.ty, 0.5, 1e-12);
    CHECK_NEAR(s.ttheta, -0.4, 1e-12);
    CHECK_NEAR(s.vel_max_xy, 102.0 * (5.0 / 255.0), 1e-12);
    CHECK_NEAR(s.vel_max_w, 34.0 * (30.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_xy, 51.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_w, 51.0 * (100.0 / 255.0), 1e-12);
    REQUIRE(s.primary_direction.has_value());
    CHECK_NEAR(*s.primary_direction, kPi / 2.0, 0.02);
}

PHX_TEST(matchctrl_golden_fast_pos_decodes) {
    const auto mc = decode_match_ctrl(from_hex(
        "505805020800c80038ffe80320010007e803e803000099554c4c990000000000"));
    REQUIRE(mc.has_value());
    CHECK(mc->skill_id == static_cast<int>(SkillId::FastPos));
    REQUIRE(mc->vision_pose.has_value());
    CHECK_NEAR(mc->vision_pose->x, 0.2, 1e-12);
    CHECK_NEAR(mc->vision_pose->y, -0.2, 1e-12);
    CHECK_NEAR(mc->vision_pose->heading, 1.0, 1e-12);
    CHECK_NEAR(mc->pos_delay_s, 0.008, 1e-12);
    const FastPosSkill s = decode_fast_pos(mc->skill_data);
    CHECK_NEAR(s.tx, 1.0, 1e-12);
    CHECK_NEAR(s.ty, 1.0, 1e-12);
    CHECK_NEAR(s.ttheta, 0.0, 1e-12);
    CHECK_NEAR(s.vel_max_xy, 153.0 * (5.0 / 255.0), 1e-12);
    CHECK_NEAR(s.vel_max_w, 85.0 * (30.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_xy, 76.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_w, 76.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(s.acc_max_xy_fast, 153.0 * (10.0 / 255.0), 1e-12);
}

PHX_TEST(matchctrl_golden_vel_skills_decode) {
    // LOCAL_VEL (1.5, -0.25, 2.0), acc 4.0 / 20.0, jerk maxed (Sumatra-style).
    const auto loc = decode_match_ctrl(from_hex(
        "5058050309000000000000000a000002dc0506ffd0076633ffff000000000000"));
    REQUIRE(loc.has_value());
    CHECK(loc->skill_id == static_cast<int>(SkillId::LocalVel));
    CHECK_NEAR(loc->pos_delay_s, 0.0025, 1e-12);
    const VelSkill lv = decode_vel(loc->skill_data);
    CHECK_NEAR(lv.vx, 1.5, 1e-12);
    CHECK_NEAR(lv.vy, -0.25, 1e-12);
    CHECK_NEAR(lv.w, 2.0, 1e-12);
    CHECK_NEAR(lv.acc_max_xy, 102.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(lv.acc_max_w, 51.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(lv.jerk_max_xy, 255.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(lv.jerk_max_w, 255.0 * (1000.0 / 255.0), 1e-12);

    // GLOBAL_VEL (-0.5, 0.75, -1.5), acc 2.0 / 10.0, jerk 50.0 / 500.0.
    const auto glo = decode_match_ctrl(from_hex(
        "505805030a0064006400c8000a0000030cfeee0224fa331a8080000000000000"));
    REQUIRE(glo.has_value());
    CHECK(glo->skill_id == static_cast<int>(SkillId::GlobalVel));
    const VelSkill gv = decode_vel(glo->skill_data);
    CHECK_NEAR(gv.vx, -0.5, 1e-12);
    CHECK_NEAR(gv.vy, 0.75, 1e-12);
    CHECK_NEAR(gv.w, -1.5, 1e-12);
    CHECK_NEAR(gv.acc_max_xy, 51.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(gv.acc_max_w, 26.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(gv.jerk_max_xy, 128.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(gv.jerk_max_w, 128.0 * (1000.0 / 255.0), 1e-12);
}

PHX_TEST(matchctrl_golden_wheel_vel_decodes_in_tigers_order) {
    // (10.0, -10.0, 5.0, -5.0) rad/s in TIGERs wire order FR, FL, RL, RR —
    // NOT our CAN order; the motion layer owns the permutation.
    const auto mc = decode_match_ctrl(from_hex(
        "505805040b00ff7fff7fff7fff000001d00730f8e80318fc0000000000000000"));
    REQUIRE(mc.has_value());
    CHECK(mc->skill_id == static_cast<int>(SkillId::WheelVel));
    CHECK(!mc->vision_pose.has_value());  // sentinels -> dead-reckon
    const WheelVelSkill s = decode_wheel_vel(mc->skill_data);
    CHECK_NEAR(s.wheel_rad_s[0], 10.0, 1e-12);   // FR
    CHECK_NEAR(s.wheel_rad_s[1], -10.0, 1e-12);  // FL
    CHECK_NEAR(s.wheel_rad_s[2], 5.0, 1e-12);    // RL
    CHECK_NEAR(s.wheel_rad_s[3], -5.0, 1e-12);   // RR
}

PHX_TEST(matchctrl_golden_sine_decodes) {
    const auto mc = decode_match_ctrl(from_hex(
        "505805050c00ff7fff7fff7fff000006f4010000e803c4090000000000000000"));
    REQUIRE(mc.has_value());
    CHECK(mc->skill_id == static_cast<int>(SkillId::Sine));
    const SineSkill s = decode_sine(mc->skill_data);
    CHECK_NEAR(s.vx_amp, 0.5, 1e-12);
    CHECK_NEAR(s.vy_amp, 0.0, 1e-12);
    CHECK_NEAR(s.w_amp, 1.0, 1e-12);
    CHECK_NEAR(s.freq_hz, 2.5, 1e-12);  // 2500 mHz
}

PHX_TEST(matchctrl_golden_emergency_and_delay_sentinels) {
    // No vision: all three axes UNUSED_FIELD and posDelay = 255.
    const auto novis = decode_match_ctrl(from_hex(
        "505805060d00ff7fff7fff7fff00000000000000000000000000000000000000"));
    REQUIRE(novis.has_value());
    CHECK(novis->robot_id == 6);
    CHECK(novis->seq == 13);
    CHECK(novis->skill_id == static_cast<int>(SkillId::Emergency));
    CHECK(!novis->vision_pose.has_value());
    CHECK_NEAR(novis->pos_delay_s, 255.0 * 0.00025, 1e-12);

    // Vision present but delay saturated below the none-marker (254).
    const auto sat = decode_match_ctrl(from_hex(
        "505805060e00000000000000fe03000000000000000000000000000000000000"));
    REQUIRE(sat.has_value());
    REQUIRE(sat->vision_pose.has_value());
    CHECK_NEAR(sat->pos_delay_s, 254.0 * 0.00025, 1e-12);
    CHECK(sat->cam_id == 3);
}

PHX_TEST(matchctrl_rejects_garbage_never_partial) {
    // Empty / short / truncated.
    CHECK(!decode_match_ctrl("").has_value());
    CHECK(!decode_match_ctrl(from_hex("505805072a00d204c9fd1503320200")).has_value());
    // 33 bytes (one too many).
    CHECK(!decode_match_ctrl(from_hex(
        "505805072a00d204c9fd150332020004d80408fd220680554c4c45875180000000")).has_value());
    // Wrong magic.
    CHECK(!decode_match_ctrl(from_hex(
        "585005072a00d204c9fd150332020004d80408fd220680554c4c458751800000")).has_value());
    // A MatchFeedback frame fed to the ctrl decoder (cmd 0x06).
    CHECK(!decode_match_ctrl(from_hex(
        "50580604630024fafa001103f40183ffdc05b4c843e2ccb209000b285a000000000000")).has_value());
    // Bare v1 text frame: not binary MatchCtrl either.
    CHECK(!decode_match_ctrl("1 0.5 0.0 0.0 0 0 1782911401.4").has_value());
    // A legacy "TIMEOUT" receive placeholder must never decode.
    CHECK(!decode_match_ctrl("TIMEOUT").has_value());
}

// --- MatchFeedback encode ---------------------------------------------------

PHX_TEST(matchfeedback_golden_encodes_byte_exact) {
    // Mirrors test_robot_wire.py::_feedback() in phoenix-server.
    MatchFeedback fb;
    fb.robot_id = 4;
    fb.seq = 99;
    fb.pos_x = -1.5;
    fb.pos_y = 0.25;
    fb.heading = kPi / 4.0;
    fb.vel_x = 0.5;
    fb.vel_y = -0.125;
    fb.ang_vel = 1.5;
    fb.kicker_level_v = 180.0;
    fb.kicker_max_v = 200.0;
    fb.dribbler_speed = 4.0;
    fb.dribble_traction = DribbleTraction::Strong;
    fb.battery_v = 22.6;
    fb.battery_percent = 80.0;
    fb.barrier_interrupted = true;
    fb.dribbler_temp_class = 1;
    fb.kick_counter = true;
    fb.ball_state = 2;
    fb.features = kFeatureMove | kFeatureKickStraight;
    fb.hardware_id = 11;
    fb.ball_pos_age_ms = 40;
    fb.ball_pos = std::make_pair(0.09, 0.0);
    check_hex(encode_match_feedback(fb),
              "50580604630024fafa001103f40183ffdc05b4c843e2ccb209000b285a000000000000");
}

PHX_TEST(matchfeedback_noball_golden_encodes_byte_exact) {
    MatchFeedback fb;
    fb.robot_id = 4;
    fb.seq = 100;
    fb.kicker_max_v = 200.0;
    fb.battery_v = 24.0;
    fb.battery_percent = 100.0;
    fb.features = kFeatureMove;
    fb.hardware_id = 11;
    fb.ball_pos_age_ms = 255;  // no onboard ball -> ballPos zeroed on the wire
    check_hex(encode_match_feedback(fb),
              "50580604640000000000000000000000000000c800f0ff0001000bff00000000000000");
}

PHX_TEST(matchfeedback_wire_fields_clamp_like_wirepy) {
    MatchFeedback fb;
    fb.robot_id = 4;
    fb.seq = 1;
    fb.pos_x = 99.0;    // -> 32766 (wire.py _mm clamp)
    fb.pos_y = -99.0;   // -> -32767
    fb.heading = 40.0;  // -> 32766
    fb.battery_v = 99.0;        // -> 255 dV
    fb.battery_percent = 140.0; // -> 255
    fb.kicker_level_v = 300.0;  // -> 255
    fb.dribbler_speed = 99.0;   // -> 63 speed bits
    fb.dribbler_temp_class = 7; // -> masked to 3
    fb.ball_state = 15;         // -> masked to 7
    fb.ball_pos_age_ms = 9999;  // -> 255
    const auto bytes = encode_match_feedback(fb);
    const uint8_t* b = bytes.data() + kHeaderSize;
    CHECK(b[0] == 0xFE && b[1] == 0x7F);   // 32766
    CHECK(b[2] == 0x01 && b[3] == 0x80);   // -32767
    CHECK(b[4] == 0xFE && b[5] == 0x7F);   // 32766
    CHECK(b[12] == 255);
    CHECK(b[14] == (63 << 2));             // traction Off
    CHECK(b[15] == 255);
    CHECK(b[16] == 255);
    CHECK(b[17] == ((3 << 5) | 7));        // temp class + ball state masked
    CHECK(b[21] == 255);
}

PHX_TEST(kd_field_layout_matches_wirepy_bit_positions) {
    // The KD word from the GLOBAL_POS golden: ARM CHIP, 6.5 m/s, 3.0 m/s, 5.0 N.
    const uint8_t kd_bytes[3] = {0x45, 0x87, 0x51};
    const KickerDribbler kd = KickerDribbler::decode(kd_bytes);
    CHECK(kd.kick_mode == KickerMode::Arm);
    CHECK(kd.kick_device == KickerDevice::Chip);
    CHECK_NEAR(kd.kick_speed, 325 * kKickSpeedScale, 1e-12);
    CHECK_NEAR(kd.dribbler_speed, 24 * kDribblerSpeedScale, 1e-12);
    CHECK_NEAR(kd.dribbler_force, 20 * kDribblerForceScale, 1e-12);

    // ARM_TIME: speed bits carry discharge time (25 us/bit) and speed is 0.
    // word = 200 bits | (3 << 10) = 0xC80 + 200 = 0xCC8 -> bytes C8 0C 00.
    const uint8_t at_bytes[3] = {0xC8, 0x0C, 0x00};
    const KickerDribbler at = KickerDribbler::decode(at_bytes);
    CHECK(at.kick_mode == KickerMode::ArmTime);
    CHECK_NEAR(at.kick_time_us, 5000.0, 1e-12);
    CHECK_NEAR(at.kick_speed, 0.0, 1e-12);

    // Saturation maxima (FORCE mode: mode bits 2 << 10): 511 * 0.02 = 10.22
    // m/s kick, 63 * 0.125 = 7.875 m/s dribbler, 63 * 0.25 = 15.75 N.
    const uint8_t max_bytes[3] = {0xFF, 0xF9, 0xFF};
    const KickerDribbler mx = KickerDribbler::decode(max_bytes);
    CHECK(mx.kick_mode == KickerMode::Force);
    CHECK_NEAR(mx.kick_speed, 511 * kKickSpeedScale, 1e-12);
    CHECK_NEAR(mx.dribbler_speed, 63 * kDribblerSpeedScale, 1e-12);
    CHECK_NEAR(mx.dribbler_force, 63 * kDribblerForceScale, 1e-12);
}


// --- MotionParams (0x07) + feedback profile bytes ------------------------
// Goldens from phoenix-server tests/test_motion_params.py (wire.py encoder).

PHX_TEST(motion_params_golden_full_decodes) {
    // robot 7, seq 42, profile 1011, rl bounded, reload policy,
    // slots: BodyLongitudinalVelMax 4.0, RlConfidenceThreshold 0.3.
    const auto mp = decode_motion_params(from_hex(
        "505807072a00f30303010100008040069a99993e00000000000000000000"));
    REQUIRE(mp.has_value());
    CHECK(mp->robot_id == 7);
    CHECK(mp->seq == 42);
    CHECK(mp->profile_id == 1011);
    CHECK(mp->rl_mode == 3);
    CHECK(mp->reload_policy);
    REQUIRE(mp->count == 2);
    CHECK(mp->params[0].key == MotionParamKey::BodyLongitudinalVelMax);
    CHECK_NEAR(mp->params[0].value, 4.0, 1e-6);
    CHECK(mp->params[1].key == MotionParamKey::RlConfidenceThreshold);
    CHECK_NEAR(mp->params[1].value, 0.3, 1e-6);
}

PHX_TEST(motion_params_golden_minimal_and_rejections) {
    const auto mp = decode_motion_params(from_hex(
        "505807050100e903ff000000000000000000000000000000000000000000"));
    REQUIRE(mp.has_value());
    CHECK(mp->robot_id == 5 && mp->seq == 1 && mp->profile_id == 1001);
    CHECK(mp->rl_mode == kRlModeKeep);
    CHECK(!mp->reload_policy);
    CHECK(mp->count == 0);
    // wrong length / wrong command / unknown key / MatchCtrl decoder refuses it
    CHECK(!decode_motion_params(from_hex("505807050100e903ff00")).has_value());
    CHECK(!decode_motion_params(from_hex(
        "505805050100e903ff000000000000000000000000000000000000000000")).has_value());
    CHECK(!decode_motion_params(from_hex(
        "505807050100e903ff00c8000000000000000000000000000000000000")).has_value());
    CHECK(!decode_match_ctrl(from_hex(
        "505807050100e903ff000000000000000000000000000000000000000000")).has_value());
}

PHX_TEST(matchfeedback_trailing_bytes_carry_profile_and_rl_mode) {
    MatchFeedback fb;
    fb.robot_id = 3;
    fb.seq = 9;
    fb.pos_x = 0.5;
    fb.pos_y = -0.25;
    fb.heading = 1.0;
    fb.vel_x = 0.1;
    fb.kicker_level_v = 0.0;
    fb.kicker_max_v = 200.0;
    fb.battery_v = 15.2;
    fb.battery_percent = 76.5;
    fb.features = 0x0B;
    fb.hardware_id = 11;
    fb.motion_profile_id = 1011;
    fb.rl_mode = 3;
    fb.adaptive_enabled = true;
    check_hex(encode_match_feedback(fb),
              "505806030900f40106ffe80364000000000000c80098c3000b000bff00000000f30307");
    // defaults keep the historical zero tail
    MatchFeedback plain;
    const auto bytes = encode_match_feedback(plain);
    CHECK(bytes[6 + 26] == 0 && bytes[6 + 27] == 0 && bytes[6 + 28] == 0);
}
