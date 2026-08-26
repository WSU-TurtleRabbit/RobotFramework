// MatchCtrl bridge — the integration layer between the wire and the motion
// cascade, and the owner of the safety tiers that TIGERs pin onboard
// (their robot.c match flow):
//
//   * seq discipline: duplicate / reordered frames are dropped (wrapping
//     u16 compare) — a stale frame carries no new command and does NOT
//     refresh the watchdog;
//   * command-loss tier: no accepted MatchCtrl for 1 s -> the EMERGENCY
//     skill runs onboard (controlled ramp to zero, then motors off);
//   * vision tier: the estimator dead-reckons on encoders + gyro when
//     vision ages out (its own 1 s timeout) and snaps back on return;
//   * enable-after-first-vision: motion never energizes before the first
//     vision fix (a robot that doesn't know where it is must not drive);
//   * estimator snaps re-anchor the trajectory reference at the corrected
//     state, and skill-kind switches reset the controller's rate state at
//     the measured twist — no lurches, ever.
//
// The cascade it orchestrates: MatchCtrl -> skills (setpoint+limits) ->
// estimator (delayed fusion) -> trajectory (per-tick regeneration, pose
// skills) -> controller (Panthera cascade) -> wheel setpoints; the KD field
// -> actuators policy. Pure and deterministic: the superloop feeds
// datagrams, time, odometry, gyro, and ball observations in, and takes
// wheel + actuator commands out.
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "actuators.h"
#include "adaptive.h"
#include "controller.h"
#include "estimator.h"
#include "skills.h"
#include "trajectory.h"

namespace rf {

struct MatchBridgeConfig {
    int expected_robot_id = -1;           // -1 = accept any (bench)
    double command_timeout_s = 1.0;       // no MatchCtrl -> EMERGENCY (TIGERs tier)
    bool require_vision_fix = true;       // motion only after the first fix
};

enum class MatchAccept {
    Accepted,   // valid, addressed to us, fresh seq: pipeline updated
    Malformed,  // not a MatchCtrl frame (never touches motion state)
    WrongId,    // valid frame for another robot
    StaleSeq,   // duplicate / reordered: dropped, watchdog NOT refreshed
};

// Everything the superloop (and MatchFeedback assembly) needs out of a tick.
struct BridgeTick {
    ControlOutput ctrl;       // wheel setpoints + energize
    TrajSample ref;           // current onboard trajectory reference
    ActuatorOutput act;       // kicker/dribbler decision
    EstimatorOutput est;      // estimator passthrough (feedback pose source)
    AugmentationOutput augmentation; // ID/adaptive/RL/safety diagnostics
    bool emergency = false;   // running the onboard EMERGENCY (timeout/skill)
    bool motion_enabled = false;  // past the first-vision gate
    int skill_id = -1;        // active skill (-1 = none yet)
    double last_cmd_age_s = -1.0;  // age of the last accepted frame
};

class MatchBridge {
public:
    MatchBridge(const MatchBridgeConfig& cfg, const Kinematics& kin,
                const EstimatorConfig& est_cfg = {},
                const TrajectoryConfig& traj_cfg = {},
                const ControllerConfig& ctrl_cfg = {},
                const ActuatorConfig& act_cfg = {},
                const AugmentationConfig& augmentation_cfg = {});

    // One received datagram. now_s must share the clock base with tick().
    MatchAccept accept(const std::string& datagram, double now_s);

    // One control tick: estimator update, watchdog tiers, skill dispatch,
    // trajectory regeneration (pose skills), controller, actuators.
    BridgeTick tick(double now_s, double dt, const phx::Twist& odo_body,
                    double gyro_yaw_radps, const BallContactObs& ball,
                    const RuntimeFeedback& feedback = {});

    // Operator STOP / estop: drop the command, disarm, coast immediately.
    void clear();

    bool active() const { return active_; }
    // Robot id of the last accepted frame (-1 = none); feedback identity
    // when the bridge accepts any id (bench mode).
    int last_robot_id() const { return last_robot_id_; }
    const MotionSetpoint& setpoint() const { return sp_; }
    const FusionEstimator& estimator() const { return estimator_; }
    FusionEstimator& estimator() { return estimator_; }
    const Actuators& actuators() const { return actuators_; }
    const MotionAugmentor& augmentor() const { return augmentor_; }
    MotionAugmentor& augmentor() { return augmentor_; }

private:
    void switch_kind(MotionSetpoint::Kind next, const EstimatorOutput& est);

    MatchBridgeConfig cfg_;
    FusionEstimator estimator_;
    TrajectoryFollower follower_;
    Controller controller_;
    Actuators actuators_;
    MotionAugmentor augmentor_;

    MotionSetpoint sp_{};                 // active setpoint (Emergency at boot)
    bool active_ = false;                 // a frame has been accepted
    int last_robot_id_ = -1;
    bool have_seq_ = false;
    uint16_t last_seq_ = 0;
    double last_accept_s_ = -1e9;
    double last_now_s_ = 0.0;
    uint32_t seen_snaps_ = 0;
    // Latest vision fix carried by an accepted frame, applied at the next
    // tick right after the estimator tick (so on_vision never lands before
    // the estimator has started, and fixes always insert into fresh slots).
    std::optional<std::pair<VisionPose, double>> pending_vision_;
};

}  // namespace rf
