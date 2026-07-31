#include "estimator.h"

#include <algorithm>
#include <cmath>

#include "phx/angle.h"

namespace rf {

FusionEstimator::FusionEstimator(const EstimatorConfig& cfg) : cfg_(cfg) {
    ring_.resize(static_cast<std::size_t>(std::max(8, cfg_.slots)));
    compute_gains();
}

void FusionEstimator::reset() {
    for (Slot& s : ring_) s = Slot{};
    head_ = 0;
    ticks_ = 0;
    started_ = false;
    has_fix_ = false;
    last_vision_t_s = -1e9;
    consecutive_rejects_ = 0;
    rejected_total_ = 0;
    last_meas_.reset();
    last_vision_heading_.reset();
    last_vision_heading_t_s_ = -1e9;
    // snap_count_ is kept: consumers compare, they don't absolute-count.
}

// Steady-state Kalman gains for the constant-velocity model
//   x' = x + v*dt, v' = v + process,  z = x + noise
// via Riccati iteration (converged long before 2000 steps at our rates).
// The result is the classic alpha-beta filter, but with gains derived from
// an explicit noise model instead of hand-picked.
void FusionEstimator::compute_gains() {
    const double dt = std::clamp(cfg_.measurement_dt_s, 0.005, 0.2);
    const double q = cfg_.process_accel_std * cfg_.process_accel_std;
    const double r = cfg_.meas_std_m * cfg_.meas_std_m;
    // Continuous white-noise acceleration model discretized:
    const double q00 = q * dt * dt * dt * dt / 4.0;
    const double q01 = q * dt * dt * dt / 2.0;
    const double q11 = q * dt * dt;
    double p00 = 1.0, p01 = 0.0, p11 = 1.0;
    for (int i = 0; i < 2000; ++i) {
        // Predict: P = F P F' + Q with F = [[1, dt], [0, 1]].
        const double n00 = p00 + 2.0 * dt * p01 + dt * dt * p11 + q00;
        const double n01 = p01 + dt * p11 + q01;
        const double n11 = p11 + q11;
        // Update: K = P H' / (H P H' + R), H = [1, 0].
        const double s = n00 + r;
        const double k0 = n00 / s;
        const double k1 = n01 / s;
        p00 = (1.0 - k0) * n00;
        p01 = (1.0 - k0) * n01;
        p11 = n11 - k1 * n01;
        k_pos_ = k0;
        k_vel_ = k1;
    }
    if (cfg_.vision_pos_gain >= 0.0) {
        k_pos_ = std::clamp(cfg_.vision_pos_gain, 0.0, 1.0);
    }
    if (cfg_.vision_vel_gain >= 0.0) {
        k_vel_ = std::max(0.0, cfg_.vision_vel_gain);
    }
}

FusionEstimator::State FusionEstimator::propagate(const State& s, double odo_vx,
                                                  double odo_vy, double gyro_w,
                                                  double dt) const {
    State o = s;
    o.theta = phx::wrap_angle(s.theta + gyro_w * dt);
    // Odometry is measured in the body frame of the step; rotate into global
    // with the step's starting heading.
    const double c = std::cos(s.theta), si = std::sin(s.theta);
    const double godo_x = odo_vx * c - odo_vy * si;
    const double godo_y = odo_vx * si + odo_vy * c;
    o.vgx = s.vgx + cfg_.odo_vel_gain * (godo_x - s.vgx);
    o.vgy = s.vgy + cfg_.odo_vel_gain * (godo_y - s.vgy);
    // Trapezoidal position integration (semi-implicit would do; this halves
    // the velocity-step error during hard accel).
    o.x = s.x + 0.5 * (s.vgx + o.vgx) * dt;
    o.y = s.y + 0.5 * (s.vgy + o.vgy) * dt;
    return o;
}

void FusionEstimator::tick(double now_s, double dt_in, const phx::Twist& odo_body,
                           double gyro_yaw_radps) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    if (!started_) {
        ring_[head_].t_s = now_s;
        ring_[head_].dt = dt;
        ticks_ = 1;
        started_ = true;
        return;
    }
    // Gyro fallback: pi3hat down -> wheel-odometry yaw (noisier, but moving
    // beats blind). Non-finite odometry keeps the last velocity estimate.
    const double gyro_w = std::isfinite(gyro_yaw_radps) ? gyro_yaw_radps : odo_body.ang;
    const double odo_vx = std::isfinite(odo_body.lin.x) ? odo_body.lin.x : 0.0;
    const double odo_vy = std::isfinite(odo_body.lin.y) ? odo_body.lin.y : 0.0;

    const Slot& prev = ring_[head_];
    const State s{prev.x, prev.y, prev.theta, prev.vgx, prev.vgy};
    const State o = propagate(s, odo_vx, odo_vy, gyro_w, dt);

    head_ = (head_ + 1) % static_cast<int>(ring_.size());
    ++ticks_;
    Slot& slot = ring_[head_];
    slot.t_s = now_s;
    slot.dt = dt;
    slot.odo_vx = odo_vx;
    slot.odo_vy = odo_vy;
    slot.gyro_w = gyro_w;
    slot.x = o.x;
    slot.y = o.y;
    slot.theta = o.theta;
    slot.vgx = o.vgx;
    slot.vgy = o.vgy;
}

void FusionEstimator::on_vision(const VisionPose& pose, double pos_delay_s) {
    if (!started_ || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.heading)) {
        return;
    }
    const int n = static_cast<int>(ring_.size());
    const double age_s = std::max(0.0, pos_delay_s) + cfg_.capture_delay_s;
    const double target_t = ring_[head_].t_s - age_s;

    // Walk back to the slot at (or just before) the measurement time, never
    // deeper than the ring has actually run (unused slots carry t_s = 0 and
    // would swallow a valid old measurement).
    const int valid = std::min(ticks_, n);
    int back = 0;
    while (back < valid - 1) {
        const int idx = (head_ - back - 1 + n) % n;
        if (ring_[idx].t_s <= target_t) break;
        ++back;
    }
    const int k = (head_ - back + n) % n;
    const Slot& at = ring_[k];

    const double rx = pose.x - at.x;
    const double ry = pose.y - at.y;
    const double rtheta = phx::angle_diff(pose.heading, at.theta);

    // Snap conditions: no fix yet, vision timed out (we dead-reckoned), or a
    // run of rejections (we were lost, not the camera).
    const bool timed_out = (ring_[head_].t_s - last_vision_t_s) > cfg_.vision_timeout_s;
    bool snap = !has_fix_ || timed_out || consecutive_rejects_ >= cfg_.gate_snap_after;

    if (!snap) {
        const double dist = std::hypot(rx, ry);
        const double limit = std::max(cfg_.gate_min_dist_m, cfg_.gate_speed_mps * age_s);
        if (dist > limit) {
            ++consecutive_rejects_;
            ++rejected_total_;
            return;
        }
    }

    // Correction at the measurement's OWN slot. A snap (first fix, vision
    // recovery, or a lost-robot run of rejections) trusts the vision pose
    // completely — there is no usable prior to blend with. Otherwise a
    // steady-state Kalman update: position innovation corrects position AND
    // (through the cross-gain) velocity — this is what pays back wheel slip
    // between fixes.
    State s{at.x, at.y, at.theta, at.vgx, at.vgy};
    if (snap) {
        s.x = pose.x;
        s.y = pose.y;
        s.theta = phx::wrap_angle(pose.heading);
        last_vision_heading_ = pose.heading;
        last_vision_heading_t_s_ = at.t_s;
    } else {
        s.x += k_pos_ * rx;
        s.y += k_pos_ * ry;
        s.vgx += k_vel_ * rx;
        s.vgy += k_vel_ * ry;
        bool temporal_heading_ok = true;
        if (last_vision_heading_.has_value()) {
            const double dt = std::clamp(
                at.t_s - last_vision_heading_t_s_, 0.0, 0.05);
            const double temporal_limit =
                std::max(0.0, cfg_.vision_heading_jump_tolerance_rad) +
                std::max(0.0, cfg_.vision_heading_rate_max_rad_s) * dt;
            temporal_heading_ok =
                std::fabs(phx::angle_diff(pose.heading, *last_vision_heading_)) <=
                temporal_limit;
        }
        const bool agrees_with_gyro =
            std::fabs(rtheta) <=
            std::max(0.0, cfg_.vision_heading_estimator_tolerance_rad);
        const bool heading_ok =
            std::fabs(rtheta) <= std::max(0.0, cfg_.theta_gate_rad) &&
            (temporal_heading_ok || agrees_with_gyro);
        if (heading_ok) {
            s.theta = phx::wrap_angle(s.theta + cfg_.theta_gain * rtheta);
            last_vision_heading_ = pose.heading;
            last_vision_heading_t_s_ = at.t_s;
        }
    }

    last_meas_ = TimedState{phx::Pose{phx::Vec2{s.x, s.y}, s.theta},
                            phx::Vec2{s.vgx, s.vgy}, at.t_s};
    last_vision_t_s = ring_[head_].t_s;
    has_fix_ = true;
    consecutive_rejects_ = 0;
    if (snap) ++snap_count_;

    // Replay from the corrected slot to the present through the stored
    // gyro/odometry inputs, then drag every slot toward the corrected
    // history with the tracking gain (TIGERs' smoothing: the output moves
    // smoothly instead of snapping per camera frame).
    const double g = snap ? 1.0 : std::clamp(cfg_.tracking_gain, 0.0, 1.0);
    for (int j = 0; j <= back; ++j) {
        const int idx = (k + j) % n;
        Slot& slot = ring_[idx];
        if (j > 0) {
            s = propagate(s, slot.odo_vx, slot.odo_vy, slot.gyro_w, slot.dt);
        }
        slot.x += g * (s.x - slot.x);
        slot.y += g * (s.y - slot.y);
        slot.theta = phx::wrap_angle(
            slot.theta + g * phx::angle_diff(s.theta, slot.theta));
        slot.vgx += g * (s.vgx - slot.vgx);
        slot.vgy += g * (s.vgy - slot.vgy);
    }
}

EstimatorOutput FusionEstimator::output() const {
    EstimatorOutput o;
    const Slot& s = ring_[head_];
    o.pose.pos = {s.x, s.y};
    o.pose.heading = s.theta;
    o.vel_global = {s.vgx, s.vgy};
    // Body frame for the controller: rotate by -heading.
    const double c = std::cos(s.theta), si = std::sin(s.theta);
    o.vel_body.lin = {s.vgx * c + s.vgy * si, -s.vgx * si + s.vgy * c};
    o.vel_body.ang = s.gyro_w;
    o.omega = s.gyro_w;
    o.has_fix = has_fix_;
    o.vision_alive =
        has_fix_ && (s.t_s - last_vision_t_s) <= cfg_.vision_timeout_s;
    o.rejected = rejected_total_;
    return o;
}

}  // namespace rf
