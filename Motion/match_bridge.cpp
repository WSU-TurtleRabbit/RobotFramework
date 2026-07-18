#include "match_bridge.h"

#include <cmath>

namespace rf {

MatchBridge::MatchBridge(const MatchBridgeConfig& cfg, const Kinematics& kin,
                         const EstimatorConfig& est_cfg,
                         const TrajectoryConfig& traj_cfg,
                         const ControllerConfig& ctrl_cfg,
                         const ActuatorConfig& act_cfg)
    : cfg_(cfg),
      estimator_(est_cfg),
      follower_(traj_cfg),
      controller_(ctrl_cfg, kin),
      actuators_(act_cfg) {}

MatchAccept MatchBridge::accept(const std::string& datagram, double now_s) {
    const std::optional<MatchCtrl> mc = decode_match_ctrl(datagram);
    if (!mc) return MatchAccept::Malformed;
    if (cfg_.expected_robot_id >= 0 && mc->robot_id != cfg_.expected_robot_id) {
        return MatchAccept::WrongId;
    }
    // Wrapping seq compare: 0 = duplicate, >= 0x8000 = older (reordered).
    if (have_seq_) {
        const uint16_t ahead = static_cast<uint16_t>(mc->seq - last_seq_);
        if (ahead == 0 || ahead >= 0x8000) return MatchAccept::StaleSeq;
    }

    // Skill first; the vision fix is queued and applied at the next tick
    // right after the estimator tick (latest wins — the server sends one
    // frame per robot per camera frame).
    const MotionSetpoint next = decode_skill(*mc);
    if (!active_ || next.kind != sp_.kind) {
        switch_kind(next.kind, estimator_.output());
    }
    sp_ = next;
    if (mc->vision_pose.has_value()) {
        pending_vision_ = std::make_pair(*mc->vision_pose, mc->pos_delay_s);
    }

    active_ = true;
    have_seq_ = true;
    last_seq_ = mc->seq;
    last_accept_s_ = now_s;
    return MatchAccept::Accepted;
}

BridgeTick MatchBridge::tick(double now_s, double dt, const phx::Twist& odo_body,
                             double gyro_yaw_radps, const BallContactObs& ball) {
    last_now_s_ = now_s;
    BridgeTick out;

    // 1. State estimation (delayed-vision fusion at the control rate). The
    // vision fix carried by the newest accepted frame is applied right
    // after the estimator tick, so it inserts into fresh slots.
    estimator_.tick(now_s, dt, odo_body, gyro_yaw_radps);
    if (pending_vision_) {
        estimator_.on_vision(pending_vision_->first, pending_vision_->second);
        pending_vision_.reset();
    }
    const EstimatorOutput est = estimator_.output();
    out.est = est;

    // NaN guard on the fused state: rather coast than drive on garbage.
    if (!est.pose.finite() || !est.vel_global.finite()) {
        estimator_.reset();
        out.ctrl = ControlOutput{};
        out.act = actuators_.tick(now_s, KickerDribbler{}, ball);
        return out;
    }

    // 2. An estimator snap (first fix, vision recovery) re-anchors the
    // trajectory reference at the corrected state.
    if (estimator_.snap_count() != seen_snaps_) {
        seen_snaps_ = estimator_.snap_count();
        follower_.reset(est.pose, est.vel_global, est.omega);
        controller_.reset(est.vel_body);
    }

    // 3. Watchdog tier: no accepted frame for command_timeout -> EMERGENCY.
    const double cmd_age = active_ ? now_s - last_accept_s_ : -1.0;
    const bool timed_out = active_ && cmd_age > cfg_.command_timeout_s;
    MotionSetpoint sp = sp_;
    if (timed_out) {
        sp = MotionSetpoint{};  // kind Emergency, KD disarmed
        if (sp_.kind != MotionSetpoint::Kind::Emergency) {
            sp_.kind = MotionSetpoint::Kind::Emergency;
        }
    }
    out.emergency = sp.kind == MotionSetpoint::Kind::Emergency;
    out.skill_id = out.emergency && timed_out ? 0 : sp.skill_id;
    out.last_cmd_age_s = cmd_age;

    // 4. First-vision gate: never energize motion before the first fix.
    out.motion_enabled = !cfg_.require_vision_fix || est.has_fix;
    if (!out.motion_enabled) {
        controller_.reset(est.vel_body);
        out.ctrl = ControlOutput{};  // coast
        out.act = actuators_.tick(now_s, sp.kd, ball);
        return out;
    }

    // 5. The cascade: trajectory regeneration for pose skills, controller
    // for everything.
    TrajSample ref;
    if (sp.kind == MotionSetpoint::Kind::Pose) {
        ref = follower_.tick(dt, sp);
    }
    out.ctrl = controller_.tick(dt, sp, est, ref, gyro_yaw_radps);

    // 6. Actuators (KD rides every motion skill; EMERGENCY/timeout disarms).
    out.act = actuators_.tick(now_s, sp.kd, ball);
    return out;
}

void MatchBridge::clear() {
    active_ = false;
    sp_ = MotionSetpoint{};
    actuators_.reset();
}

void MatchBridge::switch_kind(MotionSetpoint::Kind next, const EstimatorOutput& est) {
    if (next == MotionSetpoint::Kind::Pose) {
        // Enter pose-following from the best current state, never from a
        // stale reference.
        follower_.reset(est.pose, est.vel_global, est.omega);
    }
    controller_.reset(est.vel_body);
}

}  // namespace rf
