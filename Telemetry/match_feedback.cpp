#include "match_feedback.h"

#include <algorithm>
#include <cmath>

namespace rf {

MatchFeedback MatchFeedbackBuilder::build(const MatchBridge& bridge,
                                          const BridgeTick& bt,
                                          const FeedbackHealth& health,
                                          const BallContactObs& ball, double now_s) {
    MatchFeedback fb;
    fb.robot_id = health.robot_id;
    fb.seq = seq_++;

    // Report the PRESENT estimator state used by the controller. The old
    // matched-slot telemetry was delayed by the camera pipeline, so comparing
    // it with the server's present world pose invented 10-25 cm "estimator
    // errors" at competition speed. Live calibration needs the exact state
    // the control cascade is navigating from.
    fb.pos_x = bt.est.pose.pos.x;
    fb.pos_y = bt.est.pose.pos.y;
    fb.heading = bt.est.pose.heading;
    fb.vel_x = bt.est.vel_global.x;
    fb.vel_y = bt.est.vel_global.y;
    fb.ang_vel = bt.est.omega;

    // Kicker: recharge model (no charge ADC on this hardware).
    const double since_fire = now_s - bridge.actuators().last_fire_s();
    fb.kicker_max_v = health.kicker_max_v;
    fb.kicker_level_v =
        health.kicker_max_v *
        std::clamp(since_fire / std::max(0.1, health.kicker_recharge_s), 0.0, 1.0);

    // Dribbler: commanded bar speed + traction ladder.
    fb.dribbler_speed = bt.act.dribbler_speed;
    fb.dribble_traction = bt.act.traction;

    // Battery: bus voltage over the empty/full window.
    fb.battery_v = health.battery_v;
    fb.battery_percent =
        std::clamp((health.battery_v - health.battery_empty_v) /
                       std::max(0.1, health.battery_full_v - health.battery_empty_v),
                   0.0, 1.0) *
        100.0;

    fb.barrier_interrupted = bt.act.barrier;
    fb.dribbler_temp_class = 0;  // no dribbler temperature sensor
    fb.kick_counter = bt.act.kick_counter;
    fb.ball_state = ball.found ? 1 : 0;

    fb.features = 0;
    if (bt.motion_enabled && !health.estop) fb.features |= kFeatureMove;
    if (health.arduino_connected) {
        fb.features |= kFeatureDribbler | kFeatureKickStraight;
    }
    if (health.camera_running) fb.features |= kFeatureBarrier;
    fb.hardware_id = health.hardware_id;

    // No onboard ball-position estimate yet (needs camera calibration).
    fb.ball_pos_age_ms = 255;
    fb.ball_pos = std::nullopt;
    return fb;
}

}  // namespace rf
