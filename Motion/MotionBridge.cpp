#include "MotionBridge.h"

#include <cmath>

namespace rf {

Mv2Accept MotionBridge::accept_frame(const std::string& payload, uint64_t now_ms) {
    const std::optional<phx::Mv2Command> cmd = phx::parse_mv2(payload);
    if (!cmd) return Mv2Accept::Malformed;
    if (expected_id_ >= 0 && cmd->robot_id != static_cast<uint32_t>(expected_id_)) {
        return Mv2Accept::WrongId;
    }

    // Kick edge detection across ACCEPTED frames only: a 0 -> 1 transition
    // arms one kick; held-high re-sends don't re-fire.
    if (cmd->kick && !prev_frame_kick_) kick_pending_ = true;
    prev_frame_kick_ = cmd->kick;

    exec_.accept(*cmd, now_ms);
    return Mv2Accept::Accepted;
}

phx::ExecOutput MotionBridge::tick(uint64_t now_ms, double dt, const phx::Twist& odo_body,
                                   double imu_yaw_radps) {
    phx::ExecInput in;
    in.now_ms = now_ms;
    in.dt = dt;
    in.odo_body = odo_body;
    in.imu_yaw_radps =
        std::isnan(imu_yaw_radps) ? imu_yaw_radps : yaw_rate_sign_ * imu_yaw_radps;
    const phx::ExecOutput out = exec_.tick(in);
    last_wd_ = out.wd;
    last_dist_m_ = out.dist_m;
    return out;
}

bool MotionBridge::take_kick() {
    const bool k = kick_pending_;
    kick_pending_ = false;
    return k;
}

bool MotionBridge::dribble() const {
    bool kick = false, dribble = false;
    exec_.kick_dribble(kick, dribble);
    return dribble;
}

int64_t MotionBridge::tgt_dist_mm() const {
    if (last_dist_m_ < 0.0) return -1;
    return static_cast<int64_t>(std::lround(last_dist_m_ * 1000.0));
}

}  // namespace rf
