// MatchBridge tests: wire -> skills -> estimator -> trajectory -> controller
// -> wheels, end to end on a simulated plant, plus the onboard safety tiers
// (seq discipline, command-loss EMERGENCY, first-vision gate). The plant is
// perfect velocity tracking; vision arrives at 60 Hz with 40 ms delay, like
// the server's fake_robot.py e2e — this is the host-side proof that the
// cascade drives the robot where the server asked, precisely.
#include <cmath>
#include <cstdint>
#include <string>

#include "Motion/match_bridge.h"
#include "Motion/phx/testing.h"

using namespace rf;

namespace {

constexpr double kDt = 0.004;

void put16(std::string& f, int off, int16_t v) {
    f[off] = static_cast<char>(v & 0xFF);
    f[off + 1] = static_cast<char>((v >> 8) & 0xFF);
}

std::string pack_header(int robot, uint16_t seq, int skill_id) {
    std::string f(32, '\0');
    f[0] = 'P';
    f[1] = 'X';
    f[2] = 0x05;
    f[3] = static_cast<char>(robot);
    put16(f, 4, static_cast<int16_t>(seq));
    f[15] = static_cast<char>(skill_id);
    return f;
}

void pack_vision(std::string& f, double x, double y, double th, int delay_q62) {
    put16(f, 6, static_cast<int16_t>(std::lround(x * 1000)));
    put16(f, 8, static_cast<int16_t>(std::lround(y * 1000)));
    put16(f, 10, static_cast<int16_t>(std::lround(th * 1000)));
    f[12] = static_cast<char>(delay_q62);
}

void pack_no_vision(std::string& f) {
    put16(f, 6, 0x7FFF);
    put16(f, 8, 0x7FFF);
    put16(f, 10, 0x7FFF);
    f[12] = static_cast<char>(255);
}

// GLOBAL_POS with limits in raw u8 (vel/5, velW/30, acc/10, accW/100 of 255).
std::string frame_global_pos(int robot, uint16_t seq, double vx, double vy, double vth,
                             double tx, double ty, double tth, uint8_t vel = 102,
                             uint8_t velw = 68, uint8_t acc = 64, uint8_t accw = 102,
                             int delay_q62 = 160) {
    std::string f = pack_header(robot, seq, 4);
    pack_vision(f, vx, vy, vth, delay_q62);
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

std::string frame_local_vel(int robot, uint16_t seq, double vx, double vy, double w,
                            double px = 0, double py = 0, double pth = 0,
                            uint8_t acc = 102, uint8_t accw = 51) {
    std::string f = pack_header(robot, seq, 2);
    pack_vision(f, px, py, pth, 160);
    put16(f, 16, static_cast<int16_t>(std::lround(vx * 1000)));
    put16(f, 18, static_cast<int16_t>(std::lround(vy * 1000)));
    put16(f, 20, static_cast<int16_t>(std::lround(w * 1000)));
    f[22] = static_cast<char>(acc);
    f[23] = static_cast<char>(accw);
    f[24] = static_cast<char>(255);  // jerk maxed
    f[25] = static_cast<char>(255);
    return f;
}

std::string frame_emergency(int robot, uint16_t seq, double px, double py, double pth) {
    std::string f = pack_header(robot, seq, 0);
    pack_vision(f, px, py, pth, 160);
    return f;
}

std::string frame_no_vision_pos(int robot, uint16_t seq, double tx, double ty, double tth) {
    std::string f = pack_header(robot, seq, 4);
    pack_no_vision(f);
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

struct Lcg {
    uint32_t s = 0xABCDEF01u;
    double uniform() {
        s = s * 1664525u + 1013904223u;
        return static_cast<double>(s >> 8) / 16777216.0;
    }
    double gauss() { return (uniform() + uniform() + uniform() + uniform() - 2.0) * 1.732; }
};

// Perfect-velocity-tracking plant with a wheel FK for odometry.
struct Plant {
    double x = 0, y = 0, th = 0;
    Kinematics kin;
    std::array<double, 4> wheels{};

    phx::Twist odo() const { const BodyTwist b = kin.forward(wheels); return phx::Twist{b.vx, b.vy, b.w}; }

    void step(double dt) {
        const BodyTwist o = kin.forward(wheels);
        th = phx::wrap_angle(th + o.w * dt);
        const double c = std::cos(th), s = std::sin(th);
        x += (o.vx * c - o.vy * s) * dt;
        y += (o.vx * s + o.vy * c) * dt;
    }
};

}  // namespace

PHX_TEST(bridge_rejects_garbage_wrong_id_and_stale_seq) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    CHECK(b.accept("garbage in", 0.0) == MatchAccept::Malformed);
    CHECK(b.accept(frame_global_pos(4, 1, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::WrongId);
    CHECK(b.accept(frame_global_pos(3, 10, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::Accepted);
    CHECK(b.accept(frame_global_pos(3, 10, 0, 0, 0, 1, 0, 0), 0.004) == MatchAccept::StaleSeq);
    CHECK(b.accept(frame_global_pos(3, 9, 0, 0, 0, 1, 0, 0), 0.008) == MatchAccept::StaleSeq);
    CHECK(b.accept(frame_global_pos(3, 11, 0, 0, 0, 1, 0, 0), 0.012) == MatchAccept::Accepted);
    // Seq wrap: crossing 65535 -> 0 is AHEAD, not stale.
    MatchBridge w{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    CHECK(w.accept(frame_global_pos(3, 65534, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::Accepted);
    CHECK(w.accept(frame_global_pos(3, 65535, 0, 0, 0, 1, 0, 0), 0.004) == MatchAccept::Accepted);
    CHECK(w.accept(frame_global_pos(3, 0, 0, 0, 0, 1, 0, 0), 0.008) == MatchAccept::Accepted);
    CHECK(w.accept(frame_global_pos(3, 1, 0, 0, 0, 1, 0, 0), 0.012) == MatchAccept::Accepted);
}

PHX_TEST(bridge_first_vision_gate_blocks_motion_until_fix) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    Plant p;
    // Frames arrive WITHOUT vision (sentinels): valid commands, no fix.
    for (int i = 0; i < 100; ++i) {
        b.accept(frame_no_vision_pos(3, static_cast<uint16_t>(i + 1), 1.0, 0.0, 0.0),
                 i * kDt);
        const BridgeTick t = b.tick(i * kDt, kDt, p.odo(), 0.0, BallContactObs{});
        CHECK(!t.motion_enabled);
        CHECK(!t.ctrl.energize);
        for (double v : t.ctrl.wheel_rev_s) CHECK(v == 0.0);
    }
    // The first frame WITH vision: fix snaps, motion enables.
    b.accept(frame_global_pos(3, 500, p.x, p.y, p.th, 1.0, 0.0, 0.0), 100 * kDt);
    const BridgeTick t = b.tick(101 * kDt, kDt, p.odo(), 0.0, BallContactObs{});
    CHECK(t.motion_enabled);
    CHECK(t.ctrl.energize);
}

PHX_TEST(bridge_command_timeout_ramps_down_then_coasts) {
    Kinematics kin;
    MatchBridgeConfig cfg;
    cfg.expected_robot_id = 3;
    MatchBridge b{cfg, kin};
    Plant p;
    double now = 0.0;
    uint16_t seq = 0;
    // Drive forward at 0.5 m/s for a second.
    for (int i = 0; i < 250; ++i) {
        if (i % 4 == 0) {
            b.accept(frame_local_vel(3, seq++, 0.5, 0.0, 0.0, p.x, p.y, p.th), now);
        }
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        p.wheels = t.ctrl.wheel_rev_s;
        p.step(kDt);
        now += kDt;
    }
    const double x_before_silence = p.x;
    CHECK(x_before_silence > 0.1);  // it really moved
    // Silence. The robot must ramp down (not cliff) and then coast.
    bool saw_energized_after_timeout = false;
    double ramp_measured = -1.0;
    for (int i = 0; i < 500 && ramp_measured < 0; ++i) {  // up to 2 s
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        if (t.emergency && t.ctrl.energize) saw_energized_after_timeout = true;
        if (!t.ctrl.energize && t.emergency) ramp_measured = now - (1.0 + 1.0);
        p.step(kDt);
        now += kDt;
    }
    CHECK(saw_energized_after_timeout);  // a RAMP, not a cliff
    CHECK(ramp_measured > 0.0);          // ...that then coasted
    // It stopped well before 2 extra metres of coasting.
    CHECK(p.x - x_before_silence < 1.0);
}

PHX_TEST(bridge_server_emergency_skill_stops_the_robot) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    Plant p;
    double now = 0.0;
    uint16_t seq = 0;
    for (int i = 0; i < 250; ++i) {
        if (i % 4 == 0) b.accept(frame_local_vel(3, seq++, 0.6, 0.0, 0.0, p.x, p.y, p.th), now);
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        p.wheels = t.ctrl.wheel_rev_s;
        p.step(kDt);
        now += kDt;
    }
    CHECK(p.odo().lin.x > 0.3);
    // Server sends EMERGENCY: controlled stop while frames keep coming.
    for (int i = 0; i < 250; ++i) {
        if (i % 4 == 0) b.accept(frame_emergency(3, seq++, p.x, p.y, p.th), now);
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        p.wheels = t.ctrl.wheel_rev_s;
        p.step(kDt);
        now += kDt;
        if (i == 249) CHECK(!t.ctrl.energize);
    }
    CHECK(std::fabs(p.odo().lin.x) < 0.05);
}

// The money test: drive to a pose through the whole cascade with delayed,
// noisy vision — the host-side equivalent of the server's e2e.
PHX_TEST(bridge_drives_to_target_through_full_cascade) {
    Kinematics kin;
    EstimatorConfig est_cfg;
    est_cfg.capture_delay_s = 0.020;  // the real default, on top of 40 ms wire delay
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin, est_cfg};
    Plant p;
    Lcg rng;
    double now = 0.0;
    uint16_t seq = 0;
    double next_frame = 0.0;
    for (int i = 0; i < 2000; ++i) {  // 8 s
        if (now + 1e-12 >= next_frame) {
            next_frame += 1.0 / 60.0;
            // Vision pose as it was 40 ms ago + 3 mm / 3 mrad noise.
            const double dx = p.x - 0.0 /* constant-velocity approx. below */;
            (void)dx;
            const double noise_x = 0.003 * rng.gauss();
            const double noise_y = 0.003 * rng.gauss();
            const double noise_th = 0.003 * rng.gauss();
            // The plant accelerates, so approximate the delayed pose with the
            // current odometry velocity (like the real camera would see it).
            const BodyTwist o = p.kin.forward(p.wheels);
            const double c = std::cos(p.th), s = std::sin(p.th);
            const double gvx = o.vx * c - o.vy * s;
            const double gvy = o.vx * s + o.vy * c;
            b.accept(frame_global_pos(3, seq++, p.x - gvx * 0.040 + noise_x,
                                      p.y - gvy * 0.040 + noise_y,
                                      phx::wrap_angle(p.th - o.w * 0.040 + noise_th),
                                      1.2, 0.6, 0.8),
                     now);
        }
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        p.wheels = t.ctrl.wheel_rev_s;
        p.step(kDt);
        now += kDt;
    }
    CHECK(std::fabs(p.x - 1.2) < 0.03);
    CHECK(std::fabs(p.y - 0.6) < 0.03);
    CHECK(std::fabs(phx::angle_diff(p.th, 0.8)) < 0.03);
}

PHX_TEST(bridge_wheel_vel_maps_to_the_right_motors) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    Plant p;
    // Spin motor id 1 (FR) only: raw 2000 = 10 rad/s on wire wheel FR.
    std::string f = pack_header(3, 1, 1);
    pack_vision(f, 0, 0, 0, 160);
    put16(f, 16, 2000);
    put16(f, 18, 0);
    put16(f, 20, 0);
    put16(f, 22, 0);
    b.accept(f, 0.0);
    b.tick(0.0, kDt, phx::Twist{}, 0.0, BallContactObs{});
    BridgeTick t;
    for (int i = 0; i < 100; ++i) t = b.tick(i * kDt, kDt, p.odo(), 0.0, BallContactObs{});
    CHECK_NEAR(t.ctrl.wheel_rev_s[0], 10.0 / (2.0 * phx::kPi), 1e-9);
    CHECK_NEAR(t.ctrl.wheel_rev_s[1], 0.0, 1e-9);
    CHECK_NEAR(t.ctrl.wheel_rev_s[2], 0.0, 1e-9);
    CHECK_NEAR(t.ctrl.wheel_rev_s[3], 0.0, 1e-9);
}
