// MotionBridge: the strict frame gate between the UDP loop and the vendored
// executor — id validation, kick edge detection, gyro sign, watchdog echo.
#include <cmath>
#include <string>

#include "Motion/MotionBridge.h"
#include "Motion/phx/testing.h"

using rf::MotionBridge;
using rf::Mv2Accept;

namespace {

std::string mv2_line(int id, int seq, const std::string& kindmode = "MOVE FAST",
                     const std::string& kick = "0", const std::string& dribble = "0") {
    return "MV2 " + std::to_string(id) + " " + std::to_string(seq) + " " + kindmode +
           " 0 0 0 0 0 0 1 0 0 0 0 0 0 0 " + kick + " " + dribble + " 0 5.5";
}

}  // namespace

PHX_TEST(bridge_accepts_golden_frame_and_reports_status) {
    MotionBridge b(phx::MotionConfig{}, 1.0, 5);
    CHECK(!b.active());
    CHECK(b.mv_seq() == -1);
    const std::string golden =
        "MV2 5 1234 MOVE ALIGN -1.2345 0.5 1.5708 0.25 -0.1 0.05 -1.2845 0.55 "
        "1.5708 0.35 1.5 1.2 8 0 0 1 18 1782911401.487";
    CHECK(b.accept_frame(golden, 0) == Mv2Accept::Accepted);
    CHECK(b.active());
    CHECK(b.mv_seq() == 1234);
    CHECK(std::string(b.kind_word()) == "MOVE");
    CHECK(b.dribble());
}

PHX_TEST(bridge_rejects_malformed_without_engaging) {
    MotionBridge b(phx::MotionConfig{}, 1.0, -1);
    CHECK(b.accept_frame("MV2 garbage", 0) == Mv2Accept::Malformed);
    CHECK(b.accept_frame("MV2 1 9 MOVE FAST nan 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5", 0) ==
          Mv2Accept::Malformed);
    CHECK(!b.active());
    // A coasting bridge outputs no motion.
    const phx::ExecOutput out = b.tick(10, 0.01, phx::Twist{}, std::nan(""));
    CHECK(!out.energize);
    CHECK(b.wd_state() == 2);  // coast
}

PHX_TEST(bridge_validates_robot_id) {
    MotionBridge b(phx::MotionConfig{}, 1.0, 3);
    CHECK(b.accept_frame(mv2_line(4, 1), 0) == Mv2Accept::WrongId);
    CHECK(!b.active());
    CHECK(b.accept_frame(mv2_line(3, 1), 0) == Mv2Accept::Accepted);
    CHECK(b.active());

    // -1 accepts any id.
    MotionBridge any(phx::MotionConfig{}, 1.0, -1);
    CHECK(any.accept_frame(mv2_line(7, 1), 0) == Mv2Accept::Accepted);
}

PHX_TEST(bridge_kick_fires_once_per_edge) {
    MotionBridge b(phx::MotionConfig{}, 1.0, -1);
    CHECK(b.accept_frame(mv2_line(1, 1, "MOVE FAST", "0"), 0) == Mv2Accept::Accepted);
    CHECK(!b.take_kick());
    // Rising edge arms exactly one kick...
    CHECK(b.accept_frame(mv2_line(1, 2, "MOVE FAST", "1"), 20) == Mv2Accept::Accepted);
    CHECK(b.take_kick());
    CHECK(!b.take_kick());
    // ...held-high re-sends must NOT re-fire...
    CHECK(b.accept_frame(mv2_line(1, 3, "MOVE FAST", "1"), 40) == Mv2Accept::Accepted);
    CHECK(!b.take_kick());
    // ...and a 0 then 1 re-arms.
    CHECK(b.accept_frame(mv2_line(1, 4, "MOVE FAST", "0"), 60) == Mv2Accept::Accepted);
    CHECK(b.accept_frame(mv2_line(1, 5, "MOVE FAST", "1"), 80) == Mv2Accept::Accepted);
    CHECK(b.take_kick());
}

PHX_TEST(bridge_clear_stands_down) {
    MotionBridge b(phx::MotionConfig{}, 1.0, -1);
    CHECK(b.accept_frame(mv2_line(1, 1), 0) == Mv2Accept::Accepted);
    CHECK(b.active());
    b.clear();
    CHECK(!b.active());
    CHECK(std::string(b.kind_word()) == "-");
    const phx::ExecOutput out = b.tick(10, 0.01, phx::Twist{}, std::nan(""));
    CHECK(!out.energize);
}

PHX_TEST(bridge_applies_gyro_sign) {
    // With sign -1, a raw gyro of +1 rad/s must integrate the heading
    // NEGATIVE. (This is the config knob for a flipped pi3hat mounting.)
    MotionBridge b(phx::MotionConfig{}, -1.0, -1);
    CHECK(b.accept_frame(mv2_line(1, 1), 0) == Mv2Accept::Accepted);  // ptheta = 0
    phx::ExecOutput out;
    for (int i = 1; i <= 100; ++i) {
        out = b.tick(i * 10, 0.01, phx::Twist{}, 1.0);  // raw +1 rad/s for 1 s
    }
    CHECK(out.est.heading < -0.9);
    CHECK(out.est.heading > -1.1);

    // NaN (IMU down) must stay NaN — odometry yaw (zero here) is used.
    MotionBridge b2(phx::MotionConfig{}, -1.0, -1);
    CHECK(b2.accept_frame(mv2_line(1, 1), 0) == Mv2Accept::Accepted);
    phx::ExecOutput out2;
    for (int i = 1; i <= 100; ++i) {
        out2 = b2.tick(i * 10, 0.01, phx::Twist{}, std::nan(""));
    }
    CHECK_NEAR(out2.est.heading, 0.0, 1e-9);
}

PHX_TEST(bridge_reports_target_distance_mm) {
    MotionBridge b(phx::MotionConfig{}, 1.0, -1);
    CHECK(b.tgt_dist_mm() == -1);
    // Target 1 m away on +x (mv2_line puts tx=1).
    CHECK(b.accept_frame(mv2_line(1, 1), 0) == Mv2Accept::Accepted);
    b.tick(10, 0.01, phx::Twist{}, std::nan(""));
    CHECK(b.tgt_dist_mm() > 900);
    CHECK(b.tgt_dist_mm() <= 1000);
}
