// Skill decode + dispatch tests: MatchCtrl -> normalized MotionSetpoint.
// Golden frames come from phoenix-server's wire.py encoder (see
// golden_matchctrl.py); synthetic frames cover floors, permutation, and
// the unknown-skill failsafe.
#include <array>
#include <cstdint>
#include <string>

#include "Motion/phx/testing.h"
#include "Motion/skills.h"

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

// Build a MatchCtrl by hand (decode_skill only needs these fields).
MatchCtrl make_ctrl(int skill_id, const std::array<uint8_t, kSkillDataSize>& data) {
    MatchCtrl mc;
    mc.skill_id = skill_id;
    mc.skill_data = data;
    return mc;
}

constexpr double kPi = 3.14159265358979323846;

}  // namespace

PHX_TEST(skills_golden_global_pos_normalizes) {
    const auto mc = decode_match_ctrl(from_hex(
        "505805072a00d204c9fd150332020004d80408fd220680554c4c458751800000"));
    REQUIRE(mc.has_value());
    const MotionSetpoint sp = decode_skill(*mc);
    CHECK(sp.kind == MotionSetpoint::Kind::Pose);
    CHECK(!sp.fast_pos);
    CHECK_NEAR(sp.target.pos.x, 1.240, 1e-12);
    CHECK_NEAR(sp.target.pos.y, -0.760, 1e-12);
    CHECK_NEAR(sp.target.heading, 1.570, 1e-12);
    CHECK_NEAR(sp.vel_max_xy, 128.0 * (5.0 / 255.0), 1e-12);
    CHECK_NEAR(sp.vel_max_w, 85.0 * (30.0 / 255.0), 1e-12);
    CHECK_NEAR(sp.acc_max_xy, 76.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(sp.acc_max_w, 76.0 * (100.0 / 255.0), 1e-12);
    CHECK(!sp.primary_direction.has_value());
    CHECK(sp.kd.kick_mode == KickerMode::Arm);
    CHECK(sp.kd.kick_device == KickerDevice::Chip);
    CHECK_NEAR(sp.kd.kick_speed, 6.5, 1e-12);
}

PHX_TEST(skills_fast_pos_sets_flag_and_fast_accel) {
    const auto mc = decode_match_ctrl(from_hex(
        "505805020800c80038ffe80320010007e803e803000099554c4c990000000000"));
    REQUIRE(mc.has_value());
    const MotionSetpoint sp = decode_skill(*mc);
    CHECK(sp.kind == MotionSetpoint::Kind::Pose);
    CHECK(sp.fast_pos);
    CHECK_NEAR(sp.acc_max_xy_fast, 153.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(sp.vel_max_xy, 153.0 * (5.0 / 255.0), 1e-12);
}

PHX_TEST(skills_vel_kinds_and_limits) {
    const auto loc = decode_match_ctrl(from_hex(
        "5058050309000000000000000a000002dc0506ffd0076633ffff000000000000"));
    REQUIRE(loc.has_value());
    const MotionSetpoint lv = decode_skill(*loc);
    CHECK(lv.kind == MotionSetpoint::Kind::LocalVel);
    CHECK_NEAR(lv.vel.lin.x, 1.5, 1e-12);
    CHECK_NEAR(lv.vel.lin.y, -0.25, 1e-12);
    CHECK_NEAR(lv.vel.ang, 2.0, 1e-12);
    CHECK_NEAR(lv.acc_max_xy, 102.0 * (10.0 / 255.0), 1e-12);
    CHECK_NEAR(lv.acc_max_w_vel, 51.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(lv.jerk_max_xy, 255.0 * (100.0 / 255.0), 1e-12);
    CHECK_NEAR(lv.jerk_max_w, 255.0 * (1000.0 / 255.0), 1e-12);

    const auto glo = decode_match_ctrl(from_hex(
        "505805030a0064006400c8000a0000030cfeee0224fa331a8080000000000000"));
    REQUIRE(glo.has_value());
    const MotionSetpoint gv = decode_skill(*glo);
    CHECK(gv.kind == MotionSetpoint::Kind::GlobalVel);
    CHECK_NEAR(gv.vel.lin.x, -0.5, 1e-12);
    CHECK_NEAR(gv.vel.lin.y, 0.75, 1e-12);
    CHECK_NEAR(gv.vel.ang, -1.5, 1e-12);
}

PHX_TEST(skills_wheel_vel_permutates_wire_order_to_can) {
    // Wire (TIGERs FR,FL,RL,RR): (10, -10, 5, -5).
    // CAN (ours: FR,RR,RL,FL):      (10,  -5, 5, -10).
    const auto mc = decode_match_ctrl(from_hex(
        "505805040b00ff7fff7fff7fff000001d00730f8e80318fc0000000000000000"));
    REQUIRE(mc.has_value());
    const MotionSetpoint sp = decode_skill(*mc);
    CHECK(sp.kind == MotionSetpoint::Kind::WheelVel);
    CHECK_NEAR(sp.wheel_rad_s[0], 10.0, 1e-12);   // FR (id 1)
    CHECK_NEAR(sp.wheel_rad_s[1], -5.0, 1e-12);   // RR (id 2)
    CHECK_NEAR(sp.wheel_rad_s[2], 5.0, 1e-12);    // RL (id 3)
    CHECK_NEAR(sp.wheel_rad_s[3], -10.0, 1e-12);  // FL (id 4)

    const std::array<double, 4> w{1.0, 2.0, 3.0, 4.0};
    const auto c = wheel_order_wire_to_can(w);
    CHECK(c[0] == 1.0 && c[1] == 4.0 && c[2] == 3.0 && c[3] == 2.0);
}

PHX_TEST(skills_sine_decodes_and_stays_disarmed) {
    const auto mc = decode_match_ctrl(from_hex(
        "505805050c00ff7fff7fff7fff000006f4010000e803c4090000000000000000"));
    REQUIRE(mc.has_value());
    const MotionSetpoint sp = decode_skill(*mc);
    CHECK(sp.kind == MotionSetpoint::Kind::Sine);
    CHECK_NEAR(sp.sine_vx_amp, 0.5, 1e-12);
    CHECK_NEAR(sp.sine_vy_amp, 0.0, 1e-12);
    CHECK_NEAR(sp.sine_w_amp, 1.0, 1e-12);
    CHECK_NEAR(sp.sine_freq_hz, 2.5, 1e-12);
    CHECK(sp.kd.kick_mode == KickerMode::Disarm);
    CHECK_NEAR(sp.kd.dribbler_speed, 0.0, 1e-12);
}

PHX_TEST(skills_emergency_and_unknown_map_to_emergency) {
    const auto emg = decode_match_ctrl(from_hex(
        "505805060d00ff7fff7fff7fff00000000000000000000000000000000000000"));
    REQUIRE(emg.has_value());
    CHECK(decode_skill(*emg).kind == MotionSetpoint::Kind::Emergency);

    // GlobalVelAndOrient (5) has no wire layout in wire.py.
    const std::array<uint8_t, kSkillDataSize> zeros{};
    CHECK(decode_skill(make_ctrl(5, zeros)).kind == MotionSetpoint::Kind::Emergency);
    // Fully unknown ids must never produce arbitrary motion.
    CHECK(decode_skill(make_ctrl(99, zeros)).kind == MotionSetpoint::Kind::Emergency);
    CHECK(decode_skill(make_ctrl(255, zeros)).kind == MotionSetpoint::Kind::Emergency);
}

PHX_TEST(skills_zero_limits_hit_the_oracle_floors) {
    // A GLOBAL_POS with every limit byte 0 must not freeze the robot:
    // fake_robot.py floors velXY 0.05 m/s, accXY 0.05 m/s^2, velW 0.1 rad/s.
    const std::array<uint8_t, kSkillDataSize> zeros{};
    const MotionSetpoint sp = decode_skill(make_ctrl(4, zeros));
    CHECK(sp.kind == MotionSetpoint::Kind::Pose);
    CHECK_NEAR(sp.vel_max_xy, kSkillMinVelXY, 1e-12);
    CHECK_NEAR(sp.acc_max_xy, kSkillMinAccXY, 1e-12);
    CHECK_NEAR(sp.vel_max_w, kSkillMinVelW, 1e-12);

    const MotionSetpoint lv = decode_skill(make_ctrl(2, zeros));
    CHECK_NEAR(lv.acc_max_xy, kSkillMinAccXY, 1e-12);
    CHECK_NEAR(lv.acc_max_w_vel, kSkillMinAccW, 1e-12);
    CHECK_NEAR(lv.jerk_max_xy, kSkillMinJerkXY, 1e-12);
    CHECK_NEAR(lv.jerk_max_w, kSkillMinJerkW, 1e-12);
}

PHX_TEST(skills_target_heading_is_wrapped) {
    // ttheta raw 30000 mrad = 30 rad must wrap into (-pi, pi].
    std::array<uint8_t, kSkillDataSize> d{};
    d[0] = 0x10; d[1] = 0x27;  // tx = 10000 mm
    d[2] = 0x00; d[3] = 0x00;
    d[4] = 0x30; d[5] = 0x75;  // ttheta = 30000 mrad
    const MotionSetpoint sp = decode_skill(make_ctrl(4, d));
    CHECK_NEAR(sp.target.heading, 30.0 - 5.0 * 2.0 * kPi + 0.0, 1e-9);
    CHECK(sp.target.heading > -kPi && sp.target.heading <= kPi);
}
