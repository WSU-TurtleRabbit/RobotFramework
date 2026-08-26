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
#include "pack_matchctrl.h"

using namespace rf;
using pack::emergency;
using pack::global_pos;
using pack::global_pos_no_vision;
using pack::header;
using pack::local_vel;
using pack::vision;

namespace {

constexpr double kDt = 0.004;

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
    CHECK(b.accept(global_pos(4, 1, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::WrongId);
    CHECK(b.accept(global_pos(3, 10, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::Accepted);
    CHECK(b.accept(global_pos(3, 10, 0, 0, 0, 1, 0, 0), 0.004) == MatchAccept::StaleSeq);
    CHECK(b.accept(global_pos(3, 9, 0, 0, 0, 1, 0, 0), 0.008) == MatchAccept::StaleSeq);
    CHECK(b.accept(global_pos(3, 11, 0, 0, 0, 1, 0, 0), 0.012) == MatchAccept::Accepted);
    // Seq wrap: crossing 65535 -> 0 is AHEAD, not stale.
    MatchBridge w{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    CHECK(w.accept(global_pos(3, 65534, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::Accepted);
    CHECK(w.accept(global_pos(3, 65535, 0, 0, 0, 1, 0, 0), 0.004) == MatchAccept::Accepted);
    CHECK(w.accept(global_pos(3, 0, 0, 0, 0, 1, 0, 0), 0.008) == MatchAccept::Accepted);
    CHECK(w.accept(global_pos(3, 1, 0, 0, 0, 1, 0, 0), 0.012) == MatchAccept::Accepted);
}

PHX_TEST(bridge_resynchronizes_sequence_after_command_timeout) {
    Kinematics kin;
    MatchBridgeConfig cfg;
    cfg.expected_robot_id = 3;
    cfg.command_timeout_s = 1.0;
    MatchBridge b{cfg, kin};
    CHECK(b.accept(global_pos(3, 2000, 0, 0, 0, 1, 0, 0), 0.0) == MatchAccept::Accepted);
    CHECK(b.accept(global_pos(3, 0, 0, 0, 0, 1, 0, 0), 0.5) == MatchAccept::StaleSeq);
    CHECK(b.accept(global_pos(3, 0, 0, 0, 0, 1, 0, 0), 1.01) == MatchAccept::Accepted);
    CHECK(b.accept(global_pos(3, 0, 0, 0, 0, 1, 0, 0), 1.02) == MatchAccept::StaleSeq);
}

PHX_TEST(bridge_first_vision_gate_blocks_motion_until_fix) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    Plant p;
    // Frames arrive WITHOUT vision (sentinels): valid commands, no fix.
    for (int i = 0; i < 100; ++i) {
        b.accept(global_pos_no_vision(3, static_cast<uint16_t>(i + 1), 1.0, 0.0, 0.0),
                 i * kDt);
        const BridgeTick t = b.tick(i * kDt, kDt, p.odo(), 0.0, BallContactObs{});
        CHECK(!t.motion_enabled);
        CHECK(!t.ctrl.energize);
        for (double v : t.ctrl.wheel_rev_s) CHECK(v == 0.0);
    }
    // The first frame WITH vision: fix snaps, motion enables.
    b.accept(global_pos(3, 500, p.x, p.y, p.th, 1.0, 0.0, 0.0), 100 * kDt);
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
            b.accept(local_vel(3, seq++, 0.5, 0.0, 0.0, p.x, p.y, p.th), now);
        }
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        p.wheels = t.ctrl.wheel_rev_s;
        p.step(kDt);
        now += kDt;
    }
    const double x_before_silence = p.x;
    CHECK(x_before_silence > 0.1);  // it really moved
    // Silence. The robot must ramp down (not cliff), actively brake until
    // measured motion lands, and then coast.
    bool saw_energized_after_timeout = false;
    double ramp_measured = -1.0;
    for (int i = 0; i < 500 && ramp_measured < 0; ++i) {  // up to 2 s
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        if (t.emergency && t.ctrl.energize) saw_energized_after_timeout = true;
        if (!t.ctrl.energize && t.emergency) ramp_measured = now - (1.0 + 1.0);
        // Energized zero velocity is active braking by the moteus inner
        // loops. Once de-energized, retain the plant velocity to represent
        // coasting (this simple plant has no passive drag model).
        if (t.ctrl.energize) p.wheels = t.ctrl.wheel_rev_s;
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
        if (i % 4 == 0) b.accept(local_vel(3, seq++, 0.6, 0.0, 0.0, p.x, p.y, p.th), now);
        const BridgeTick t = b.tick(now, kDt, p.odo(), p.odo().ang, BallContactObs{});
        p.wheels = t.ctrl.wheel_rev_s;
        p.step(kDt);
        now += kDt;
    }
    CHECK(p.odo().lin.x > 0.3);
    // Server sends EMERGENCY: controlled stop while frames keep coming.
    for (int i = 0; i < 250; ++i) {
        if (i % 4 == 0) b.accept(emergency(3, seq++, p.x, p.y, p.th), now);
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
            b.accept(global_pos(3, seq++, p.x - gvx * 0.040 + noise_x,
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
    std::string f = header(3, 1, 1);
    vision(f, 0, 0, 0, 160);
    pack::put16(f, 16, 2000);
    pack::put16(f, 18, 0);
    pack::put16(f, 20, 0);
    pack::put16(f, 22, 0);
    b.accept(f, 0.0);
    b.tick(0.0, kDt, phx::Twist{}, 0.0, BallContactObs{});
    BridgeTick t;
    for (int i = 0; i < 100; ++i) t = b.tick(i * kDt, kDt, p.odo(), 0.0, BallContactObs{});
    CHECK_NEAR(t.ctrl.wheel_rev_s[0], 10.0 / (2.0 * phx::kPi), 1e-9);
    CHECK_NEAR(t.ctrl.wheel_rev_s[1], 0.0, 1e-9);
    CHECK_NEAR(t.ctrl.wheel_rev_s[2], 0.0, 1e-9);
    CHECK_NEAR(t.ctrl.wheel_rev_s[3], 0.0, 1e-9);
}
