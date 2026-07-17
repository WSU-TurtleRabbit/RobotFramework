// Ports of the phoenix-rf MoveExecutor plant tests (via phoenix-core
// tests/cpp/test_motion_executor.cpp): the executor runs at 250 Hz against a
// perfect plant (commanded twist executes exactly, odometry reports it
// back), while a simulated server re-sends MV2 frames at 50 Hz.
#include <cmath>
#include <string>

#include "Motion/phx/executor.h"
#include "Motion/phx/testing.h"

using namespace phx;

namespace {

constexpr double kDt = 0.004;  // 250 Hz

Mv2Command frame(uint32_t seq, MoveKind kind, MoveMode mode) {
    Mv2Command f;
    f.robot_id = 5;
    f.seq = seq;
    f.kind = kind;
    f.mode = mode;
    return f;
}

struct Plant {
    MoveExecutor exec;
    Pose pose;  // ground truth
    double now_ms = 0;
    uint32_t seq = 0;
    Twist last_twist;

    explicit Plant(const MotionConfig& cfg = MotionConfig{}) : exec(cfg) {}

    void send(Mv2Command f) {
        f.seq = ++seq;
        f.px = pose.pos.x;
        f.py = pose.pos.y;
        f.ptheta = pose.heading;
        exec.accept(f, static_cast<uint64_t>(now_ms));
    }

    ExecOutput tick() {
        now_ms += kDt * 1000.0;
        ExecInput in;
        in.now_ms = static_cast<uint64_t>(now_ms);
        in.dt = kDt;
        in.odo_body = last_twist;
        in.imu_yaw_radps = last_twist.ang;
        const ExecOutput out = exec.tick(in);
        const Twist t = out.energize ? out.twist : Twist{};
        const double s = std::sin(pose.heading), c = std::cos(pose.heading);
        pose.pos.x += (t.lin.x * c - t.lin.y * s) * kDt;
        pose.pos.y += (t.lin.x * s + t.lin.y * c) * kDt;
        pose.heading = wrap_angle(pose.heading + t.ang * kDt);
        last_twist = t;
        return out;
    }

    // Run with the server re-sending the same frame at 50 Hz.
    ExecOutput run(Mv2Command f, int ticks) {
        ExecOutput out;
        for (int i = 0; i < ticks; ++i) {
            if (i % 5 == 0) send(f);
            out = tick();
        }
        return out;
    }
};

}  // namespace

PHX_TEST(exec_move_converges_and_settles) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 1.0;
    f.ty = 0.5;
    f.ttheta = kPi / 2;
    const ExecOutput out = p.run(f, 1500);  // 6 s, ample
    CHECK(out.arrived);
    CHECK_NEAR(p.pose.pos.x, 1.0, 0.06);
    CHECK_NEAR(p.pose.pos.y, 0.5, 0.06);
    CHECK(std::abs(angle_diff(kPi / 2, p.pose.heading)) < 0.09);
    // Settled: no residual commanded motion.
    CHECK(out.twist.lin.norm() < 0.02);
    CHECK(std::abs(out.twist.ang) < 0.06);
}

PHX_TEST(exec_respects_mode_caps_and_accel) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 3.0;
    const ModeLimits lim = ModeLimits::fast_travel();
    Twist prev;
    for (int i = 0; i < 1000; ++i) {
        if (i % 5 == 0) p.send(f);
        const ExecOutput out = p.tick();
        CHECK(out.twist.lin.norm() <= lim.max_speed_mps + 1e-6);
        CHECK(std::abs(out.twist.ang) <= lim.max_w_radps + 1e-6);
        // Per-tick velocity change bounded by the accel cap.
        CHECK((out.twist.lin - prev.lin).norm() <= lim.max_accel_mps2 * kDt + 1e-6);
        prev = out.twist;
        if (!out.energize) break;
    }
    // It actually gets up to speed on a 3 m run.
    Plant p2;
    double top = 0;
    for (int i = 0; i < 750; ++i) {
        if (i % 5 == 0) p2.send(f);
        top = std::max(top, p2.tick().twist.lin.norm());
    }
    CHECK(top > 0.9 * lim.max_speed_mps);
}

PHX_TEST(exec_precision_mode_is_tighter) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::PrecisionAlign);
    f.tx = 0.3;
    const ExecOutput out = p.run(f, 1500);
    CHECK(out.arrived);
    CHECK_NEAR(p.pose.pos.x, 0.3, 0.02);  // 12 mm arrive radius + slack
    // Never exceeded the ALIGN cap.
    Plant p2;
    double top = 0;
    for (int i = 0; i < 1000; ++i) {
        if (i % 5 == 0) p2.send(f);
        top = std::max(top, p2.tick().twist.lin.norm());
    }
    CHECK(top <= ModeLimits::precision_align().max_speed_mps + 1e-6);
}

PHX_TEST(exec_wire_caps_only_tighten) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 3.0;
    f.max_speed_mps = 0.5;   // tighter than FAST's 2.5
    f.max_accel_mps2 = 9.0;  // looser than FAST's 2.5 -> ignored
    double top = 0;
    Twist prev;
    double max_dv = 0;
    for (int i = 0; i < 1000; ++i) {
        if (i % 5 == 0) p.send(f);
        const ExecOutput out = p.tick();
        top = std::max(top, out.twist.lin.norm());
        max_dv = std::max(max_dv, (out.twist.lin - prev.lin).norm());
        prev = out.twist;
    }
    CHECK(top <= 0.5 + 1e-6);
    CHECK(max_dv <= ModeLimits::fast_travel().max_accel_mps2 * kDt + 1e-6);
}

PHX_TEST(exec_watchdog_brake_then_coast) {
    MotionConfig cfg;
    Plant p(cfg);
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 5.0;
    // Get moving with fresh frames.
    for (int i = 0; i < 250; ++i) {
        if (i % 5 == 0) p.send(f);
        p.tick();
    }
    CHECK(p.last_twist.lin.norm() > 1.0);
    // Stop sending. Within brake window: still fresh.
    ExecOutput out;
    bool saw_brake = false, saw_coast = false;
    double speed_at_coast = 1e9;
    for (int i = 0; i < 500; ++i) {  // 2 s silence
        out = p.tick();
        if (out.wd == WatchdogTier::Brake) saw_brake = true;
        if (out.wd == WatchdogTier::Coast && !saw_coast) {
            saw_coast = true;
            speed_at_coast = p.last_twist.lin.norm();
        }
    }
    CHECK(saw_brake);
    CHECK(saw_coast);
    CHECK(!out.energize);
    // The brake tier actually slowed the robot before coast cut in.
    CHECK(speed_at_coast < 0.2);
}

PHX_TEST(exec_brake_command_stops_then_coasts) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 5.0;
    for (int i = 0; i < 250; ++i) {
        if (i % 5 == 0) p.send(f);
        p.tick();
    }
    const double entry = p.last_twist.lin.norm();
    CHECK(entry > 1.0);
    Mv2Command b = frame(0, MoveKind::Brake, MoveMode::Brake);
    Twist prev = p.last_twist;
    bool coasted = false;
    for (int i = 0; i < 750; ++i) {
        if (i % 5 == 0) p.send(b);
        const ExecOutput out = p.tick();
        if (!out.energize) {
            // Coast releases the motors — the commanded twist snaps to zero
            // by design, so the decel bound only applies while energized.
            coasted = true;
            break;
        }
        CHECK((out.twist.lin - prev.lin).norm() <=
              MotionConfig{}.brake_decel_mps2 * kDt + 1e-6);
        prev = out.twist;
    }
    CHECK(coasted);
    CHECK(p.last_twist.lin.norm() < 1e-9);  // fully stopped when coasting
}

PHX_TEST(exec_disable_coasts_immediately) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 5.0;
    for (int i = 0; i < 250; ++i) {
        if (i % 5 == 0) p.send(f);
        p.tick();
    }
    p.send(frame(0, MoveKind::Disable, MoveMode::FastTravel));
    const ExecOutput out = p.tick();
    CHECK(!out.energize);
    CHECK(out.twist.lin.norm() < 1e-9);
}

PHX_TEST(exec_seq_reorder_and_dup_dropped) {
    MoveExecutor ex{MotionConfig{}};
    Mv2Command a = frame(10, MoveKind::Move, MoveMode::FastTravel);
    ex.accept(a, 0);
    CHECK(ex.last_seq() == 10);
    // Older frame ignored.
    Mv2Command older = frame(9, MoveKind::Disable, MoveMode::FastTravel);
    ex.accept(older, 1);
    CHECK(ex.last_seq() == 10);
    CHECK(std::string(ex.kind_word()) == "MOVE");
    // Duplicate ignored.
    Mv2Command dup = frame(10, MoveKind::Disable, MoveMode::FastTravel);
    ex.accept(dup, 2);
    CHECK(std::string(ex.kind_word()) == "MOVE");
    // Newer accepted (wrapping-safe).
    Mv2Command newer = frame(11, MoveKind::Hold, MoveMode::HoldPosition);
    ex.accept(newer, 3);
    CHECK(ex.last_seq() == 11);
    CHECK(std::string(ex.kind_word()) == "HOLD");
}

PHX_TEST(exec_pose_snap_on_teleport) {
    MoveExecutor ex{MotionConfig{}};
    Mv2Command f = frame(1, MoveKind::Move, MoveMode::FastTravel);
    f.px = 0;
    f.py = 0;
    ex.accept(f, 0);
    REQUIRE(ex.estimate().has_value());
    CHECK_NEAR(ex.estimate()->pos.x, 0.0, 1e-9);
    // Far-away server pose (> snap_dist 0.35 m): estimate snaps, not blends.
    Mv2Command g = frame(2, MoveKind::Move, MoveMode::FastTravel);
    g.px = 1.0;
    g.py = 1.0;
    ex.accept(g, 20);
    CHECK_NEAR(ex.estimate()->pos.x, 1.0, 1e-9);
    CHECK_NEAR(ex.estimate()->pos.y, 1.0, 1e-9);
    // Nearby server pose: blends by pose_gain (0.35).
    Mv2Command h = frame(3, MoveKind::Move, MoveMode::FastTravel);
    h.px = 1.1;
    h.py = 1.0;
    ex.accept(h, 40);
    CHECK_NEAR(ex.estimate()->pos.x, 1.0 + 0.35 * 0.1, 1e-9);
}

PHX_TEST(exec_hold_latches_first_pose) {
    MoveExecutor ex{MotionConfig{}};
    Mv2Command h = frame(1, MoveKind::Hold, MoveMode::HoldPosition);
    h.px = 0.0;
    h.py = 0.0;
    ex.accept(h, 0);
    // Server pose drifts +x over subsequent HOLD frames (robot being pushed).
    for (uint32_t s = 2; s <= 6; ++s) {
        Mv2Command d = frame(s, MoveKind::Hold, MoveMode::HoldPosition);
        d.px = 0.05 * static_cast<double>(s - 1);
        d.py = 0.0;
        ex.accept(d, s * 20);
    }
    // The executor must push BACK toward the latched pose (negative body x,
    // since heading is 0 and the estimate has drifted +x).
    ExecInput in;
    in.now_ms = 140;
    in.dt = kDt;
    const ExecOutput out = ex.tick(in);
    CHECK(out.energize);
    CHECK(out.desired.lin.x < -1e-4);
}

PHX_TEST(exec_arrive_speed_passes_through_target) {
    Plant p;
    Mv2Command f = frame(0, MoveKind::Move, MoveMode::FastTravel);
    f.tx = 1.0;
    f.arrive_speed_mps = 0.5;
    double speed_at_closest = 0;
    double closest = 1e9;
    bool passed = false;
    for (int i = 0; i < 1000; ++i) {
        if (i % 5 == 0) p.send(f);
        p.tick();
        const double d = std::abs(p.pose.pos.x - 1.0);
        if (d < closest) {
            closest = d;
            speed_at_closest = p.last_twist.lin.norm();
        }
        if (p.pose.pos.x > 1.02) {
            passed = true;
            break;
        }
    }
    CHECK(passed);            // it does not stop at the target
    CHECK(closest < 0.05);    // and it actually reaches it
    CHECK(speed_at_closest > 0.1);  // carrying speed through
}

PHX_TEST(exec_no_command_is_coast) {
    MoveExecutor ex{MotionConfig{}};
    ExecInput in;
    in.now_ms = 5;
    in.dt = kDt;
    const ExecOutput out = ex.tick(in);
    CHECK(!out.energize);
    CHECK(out.wd == WatchdogTier::Coast);
    CHECK(out.dist_m < 0);
}
