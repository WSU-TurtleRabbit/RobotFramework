// MatchFeedback assembly tests: what the robot reports back to the server —
// present controller-estimator pose, kicker charge model, traction
// ladder, battery curve, feature bits, seq, and the kick-counter toggle.
#include <cmath>

#include "Motion/match_bridge.h"
#include "Motion/phx/testing.h"
#include "Telemetry/match_feedback.h"
#include "pack_matchctrl.h"

using namespace rf;
using pack::global_pos;

namespace {
constexpr double kDt = 0.004;

FeedbackHealth health() {
    FeedbackHealth h;
    h.robot_id = 3;
    h.hardware_id = 11;
    h.battery_v = 15.2;
    h.arduino_connected = true;
    h.camera_running = true;
    return h;
}

BallContactObs no_ball() { return BallContactObs{}; }

BallContactObs ball_at_mouth() {
    BallContactObs o;
    o.found = true;
    o.bearing = 0.05;
    o.radius = 90.0;
    o.confidence = 0.9;
    o.age_s = 0.05;
    return o;
}
}  // namespace

PHX_TEST(feedback_uses_present_controller_estimate) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    // Give the estimator a vision fix at (0.4, -0.2, 0.5).
    b.accept(global_pos(3, 1, 0.4, -0.2, 0.5, 1.0, 0.0, 0.0), 0.0);
    b.tick(0.0, kDt, phx::Twist{}, 0.0, no_ball());
    const BridgeTick t = b.tick(
        kDt, kDt, phx::Twist{1.0, 0.0, 0.0}, 0.0, no_ball()
    );
    MatchFeedbackBuilder fb;
    const MatchFeedback out = fb.build(b, t, health(), no_ball(), 0.0);
    CHECK(out.robot_id == 3);
    CHECK(out.seq == 0);
    // Feedback must equal the state the controller consumed this tick.
    CHECK_NEAR(out.pos_x, t.est.pose.pos.x, 1e-12);
    CHECK_NEAR(out.pos_y, t.est.pose.pos.y, 1e-12);
    CHECK_NEAR(out.heading, t.est.pose.heading, 1e-12);
    CHECK_NEAR(out.vel_x, t.est.vel_global.x, 1e-12);
    CHECK(out.hardware_id == 11);
    CHECK(out.ball_pos_age_ms == 255);
}

PHX_TEST(feedback_seq_increments) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{}, kin};
    const BridgeTick t = b.tick(0.0, kDt, phx::Twist{}, 0.0, no_ball());
    MatchFeedbackBuilder fb;
    CHECK(fb.build(b, t, health(), no_ball(), 0.0).seq == 0);
    CHECK(fb.build(b, t, health(), no_ball(), 0.2).seq == 1);
    CHECK(fb.build(b, t, health(), no_ball(), 0.4).seq == 2);
}

PHX_TEST(feedback_kicker_charge_model) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    MatchFeedbackBuilder fb;
    // Never fired: fully charged.
    b.accept(global_pos(3, 1, 0, 0, 0, 0, 0, 0), 0.0);
    BridgeTick t = b.tick(0.0, kDt, phx::Twist{}, 0.0, no_ball());
    CHECK_NEAR(fb.build(b, t, health(), no_ball(), 0.0).kicker_level_v, 200.0, 1e-9);

    // ARM + ball at the mouth -> fires -> level collapses toward 0.
    std::string f = global_pos(3, 2, 0, 0, 0, 1.0, 0.0, 0.0);
    pack::kd(f, 26, pack::kd_word(300, 0, 1, 0, 0));  // ARM straight, 6 m/s
    b.accept(f, 0.004);
    t = b.tick(0.004, kDt, phx::Twist{}, 0.0, ball_at_mouth());
    REQUIRE(t.act.fire_pulse_ms.has_value());
    const MatchFeedback out = fb.build(b, t, health(), ball_at_mouth(), 0.004);
    CHECK(out.kicker_level_v < 1.0);
    CHECK(out.barrier_interrupted);
    CHECK(out.kick_counter);
    // Half the recharge later: ~half charged, counter holds.
    const MatchFeedback later = fb.build(b, t, health(), no_ball(), 0.004 + 2.5);
    CHECK_NEAR(later.kicker_level_v, 100.0, 2.0);
    CHECK(later.kick_counter == out.kick_counter);
}

PHX_TEST(feedback_kick_counter_toggles_per_kick) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    MatchFeedbackBuilder fb;
    b.accept(global_pos(3, 1, 0, 0, 0, 0, 0, 0), 0.0);
    b.tick(0.0, kDt, phx::Twist{}, 0.0, no_ball());
    // Two FORCE kicks well past the recharge guard apart.
    std::string f1 = global_pos(3, 2, 0, 0, 0, 1.0, 0.0, 0.0);
    pack::kd(f1, 26, pack::kd_word(300, 0, 2, 0, 0));  // FORCE
    b.accept(f1, 0.004);
    BridgeTick t = b.tick(0.004, kDt, phx::Twist{}, 0.0, no_ball());
    const bool first = fb.build(b, t, health(), no_ball(), 0.004).kick_counter;
    // Disarm, then FORCE again after recharge (5.2 s guard).
    b.accept(global_pos(3, 3, 0, 0, 0, 1.0, 0.0, 0.0), 6.0);
    b.tick(6.0, kDt, phx::Twist{}, 0.0, no_ball());
    std::string f2 = global_pos(3, 4, 0, 0, 0, 1.0, 0.0, 0.0);
    pack::kd(f2, 26, pack::kd_word(300, 0, 2, 0, 0));
    b.accept(f2, 6.2);
    t = b.tick(6.2, kDt, phx::Twist{}, 0.0, no_ball());
    REQUIRE(t.act.fire_pulse_ms.has_value());
    CHECK(fb.build(b, t, health(), no_ball(), 6.2).kick_counter != first);
}

PHX_TEST(feedback_maps_health_to_features_and_battery) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    b.accept(global_pos(3, 1, 0, 0, 0, 0, 0, 0), 0.0);
    const BridgeTick t = b.tick(0.0, kDt, phx::Twist{}, 0.0, no_ball());
    MatchFeedbackBuilder fb;

    FeedbackHealth h = health();
    MatchFeedback out = fb.build(b, t, h, no_ball(), 0.0);
    CHECK(out.features & kFeatureMove);
    CHECK(out.features & kFeatureDribbler);
    CHECK(out.features & kFeatureBarrier);
    CHECK(out.features & kFeatureKickStraight);
    CHECK(!(out.features & kFeatureKickChip));  // no chipper on this robot
    CHECK_NEAR(out.battery_v, 15.2, 1e-9);
    // (15.2 - 10) / (16.8 - 10) = 76.5%
    CHECK_NEAR(out.battery_percent, 100.0 * 5.2 / 6.8, 0.5);

    // Estop clears MOVE; dead Arduino clears dribbler+kick features.
    h.estop = true;
    h.arduino_connected = false;
    h.camera_running = false;
    out = fb.build(b, t, h, no_ball(), 0.0);
    CHECK(!(out.features & kFeatureMove));
    CHECK(!(out.features & kFeatureDribbler));
    CHECK(!(out.features & kFeatureBarrier));
    CHECK(!(out.features & kFeatureKickStraight));

    // Battery window edges clamp.
    h = health();
    h.battery_v = 9.0;
    CHECK_NEAR(fb.build(b, t, h, no_ball(), 0.0).battery_percent, 0.0, 1e-9);
    h.battery_v = 17.5;
    CHECK_NEAR(fb.build(b, t, h, no_ball(), 0.0).battery_percent, 100.0, 1e-9);
}

PHX_TEST(feedback_traction_and_dribble_echo) {
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    // Dribbler at 3 m/s (24 bits) with the ball at the mouth.
    std::string f = global_pos(3, 1, 0, 0, 0, 1.0, 0.0, 0.0);
    pack::kd(f, 26, pack::kd_word(0, 0, 0, 24, 0));
    b.accept(f, 0.0);
    const BridgeTick t = b.tick(0.0, kDt, phx::Twist{}, 0.0, ball_at_mouth());
    MatchFeedbackBuilder fb;
    const MatchFeedback out = fb.build(b, t, health(), ball_at_mouth(), 0.0);
    CHECK_NEAR(out.dribbler_speed, 3.0, 1e-9);
    CHECK(out.dribble_traction == DribbleTraction::Strong);
    CHECK(out.barrier_interrupted);
    CHECK(out.ball_state == 1);
}

PHX_TEST(feedback_wire_bytes_roundtrip_through_the_golden_decoder_shape) {
    // Build a full frame and decode it with the same offsets the server's
    // decode_match_feedback uses (the encoder itself is golden-pinned in
    // test_matchctrl.cpp).
    Kinematics kin;
    MatchBridge b{MatchBridgeConfig{.expected_robot_id = 3}, kin};
    b.accept(global_pos(3, 1, 0.25, -0.5, 1.0, 1.0, 0.0, 0.0), 0.0);
    const BridgeTick t = b.tick(0.0, kDt, phx::Twist{}, 0.0, no_ball());
    MatchFeedbackBuilder fb;
    const MatchFeedback out = fb.build(b, t, health(), no_ball(), 0.0);
    const auto bytes = encode_match_feedback(out);
    CHECK(bytes.size() == 35);
    CHECK(bytes[0] == 'P' && bytes[1] == 'X' && bytes[2] == 0x06);
    CHECK(bytes[3] == 3);
    CHECK(bytes[4] == 0 && bytes[5] == 0);  // seq 0
    const auto rd16 = [&](int off) {
        return static_cast<int16_t>(static_cast<uint16_t>(bytes[off]) |
                                    (static_cast<uint16_t>(bytes[off + 1]) << 8));
    };
    CHECK(rd16(6) == 250);   // 0.25 m
    CHECK(rd16(8) == -500);  // -0.5 m
    CHECK(rd16(10) == 1000); // 1.0 rad
}
