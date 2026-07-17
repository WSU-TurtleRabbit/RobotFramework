// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3. Only the include paths
// were adapted; the logic is byte-for-byte the server's.
//
// The MV2 move executor — the robot-side "reflexes" of the TIGERs-style
// split, ported 1:1 from the field-proven Rust implementation in phoenix-rf
// (crates/protocol/src/motion.rs, verified on Robot B). The server streams
// MV2 frames (current pose from vision + target pose + mode + limits) at
// 30–100 Hz; this module turns them into a smooth body twist at the control
// rate (250 Hz), so wheel motion never depends on Wi-Fi/vision cadence.
//
// Pure and deterministic: no clocks, no sockets — the caller feeds
// now_ms/dt/odometry in, gets a twist + energize decision out.
//
// Pipeline per tick:
//   1. Pose estimate — dead-reckon the last server pose with wheel odometry
//      (position) and the gyro (heading); each accepted frame BLENDS the
//      server pose in (complementary filter) instead of snapping.
//   2. Control law — braking-envelope P law toward the target pose, with
//      settle latches so there is no twitching around the target.
//   3. Shaping — acceleration- AND jerk-limited twist shaper (S-curve),
//      per-mode limits, optionally tightened by the frame's caps.
//   4. Watchdog — frames stop → BRAKE tier (active smooth stop) → COAST tier
//      (motor output cut).
//
// This header runs on BOTH the server (simulator's virtual onboard executor)
// and the robot (this repository). Keep it free of server-only includes.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "angle.h"
#include "pose.h"
#include "wire.h"

namespace phx {

enum class WatchdogTier : uint8_t { Fresh = 0, Brake = 1, Coast = 2 };

// One movement mode's limit/gain profile. Speeds m/s, accels m/s^2,
// jerks m/s^3; angular in rad-based equivalents. Defaults are the tuned
// values proven on the real robot — change them in config, not here.
struct ModeLimits {
    double max_speed_mps = 2.5;
    double max_w_radps = 6.0;
    double max_accel_mps2 = 2.5;
    double max_w_accel_radps2 = 15.0;
    double max_jerk_mps3 = 15.0;
    double max_w_jerk_radps3 = 60.0;
    double kp_pos = 2.5;             // desired speed near target = kp_pos * dist
    double kp_ang = 4.0;             // rad/s per rad
    double arrive_radius_m = 0.05;   // settle latch engages inside this
    double arrive_release_m = 0.03;  // and releases at radius + release
    double arrive_heading_rad = 0.08;
    double heading_release_rad = 0.04;
    double min_speed_mps = 0.06;     // creep floor outside the arrive window
    double min_w_radps = 0.15;

    static ModeLimits fast_travel() { return {}; }

    static ModeLimits ball_approach() {
        ModeLimits l;
        l.max_speed_mps = 0.8;
        l.max_w_radps = 3.0;
        l.max_accel_mps2 = 1.2;
        l.max_w_accel_radps2 = 8.0;
        l.max_jerk_mps3 = 8.0;
        l.max_w_jerk_radps3 = 40.0;
        l.kp_pos = 2.2;
        l.kp_ang = 3.5;
        l.arrive_radius_m = 0.03;
        l.arrive_release_m = 0.02;
        l.arrive_heading_rad = 0.05;
        l.heading_release_rad = 0.03;
        l.min_speed_mps = 0.05;
        l.min_w_radps = 0.12;
        return l;
    }

    static ModeLimits precision_align() {
        ModeLimits l;
        l.max_speed_mps = 0.35;
        l.max_w_radps = 1.5;
        l.max_accel_mps2 = 1.0;
        l.max_w_accel_radps2 = 6.0;
        l.max_jerk_mps3 = 6.0;
        l.max_w_jerk_radps3 = 30.0;
        l.kp_pos = 2.0;
        l.kp_ang = 3.0;
        l.arrive_radius_m = 0.012;
        l.arrive_release_m = 0.008;
        l.arrive_heading_rad = 0.03;
        l.heading_release_rad = 0.02;
        l.min_speed_mps = 0.04;
        l.min_w_radps = 0.10;
        return l;
    }

    static ModeLimits hold_position() {
        ModeLimits l;
        l.max_speed_mps = 0.3;
        l.max_w_radps = 2.0;
        l.max_accel_mps2 = 1.5;
        l.max_w_accel_radps2 = 10.0;
        l.max_jerk_mps3 = 10.0;
        l.max_w_jerk_radps3 = 50.0;
        l.kp_pos = 3.0;
        l.kp_ang = 4.0;
        l.arrive_radius_m = 0.01;
        l.arrive_release_m = 0.01;
        l.arrive_heading_rad = 0.02;
        l.heading_release_rad = 0.02;
        l.min_speed_mps = 0.0;
        l.min_w_radps = 0.0;
        return l;
    }

    // Tighten by the per-command caps in the frame (0 = no override).
    // Wire caps can only LOWER a limit.
    ModeLimits tightened(const Mv2Command& cmd) const {
        ModeLimits l = *this;
        const auto cap = [](double& lim, double wire) {
            if (wire > 0.0 && wire < lim) lim = wire;
        };
        cap(l.max_speed_mps, cmd.max_speed_mps);
        cap(l.max_w_radps, cmd.max_w_radps);
        cap(l.max_accel_mps2, cmd.max_accel_mps2);
        cap(l.max_jerk_mps3, cmd.max_jerk_mps3);
        return l;
    }
};

struct MotionConfig {
    uint64_t brake_after_ms = 300;   // no fresh frame -> BRAKE tier
    uint64_t coast_after_ms = 1200;  // still nothing -> COAST tier
    double brake_decel_mps2 = 3.5;   // BRAKE deceleration (stronger than modes)
    double brake_w_decel_radps2 = 12.0;
    double brake_jerk_mps3 = 30.0;
    double pose_gain = 0.35;    // server-pose correction per frame, 0..1
    double heading_gain = 0.5;
    double snap_dist_m = 0.35;  // error beyond this snaps to the server pose
    ModeLimits fast_travel = ModeLimits::fast_travel();
    ModeLimits ball_approach = ModeLimits::ball_approach();
    ModeLimits precision_align = ModeLimits::precision_align();
    ModeLimits hold_position = ModeLimits::hold_position();

    const ModeLimits& limits(MoveMode m) const {
        switch (m) {
            case MoveMode::FastTravel: return fast_travel;
            case MoveMode::BallApproach: return ball_approach;
            case MoveMode::PrecisionAlign: return precision_align;
            case MoveMode::HoldPosition:
            case MoveMode::Brake: return hold_position;
        }
        return fast_travel;
    }
};

struct ShapeLimits {
    double accel = 0, jerk = 0, w_accel = 0, w_jerk = 0;
};

// Acceleration- and jerk-limited twist shaper (S-curve velocity profile).
// Translation is shaped as a 2D vector (diagonals aren't bent toward an
// axis); yaw is shaped independently.
class TwistShaper {
public:
    Twist current() const { return vel_; }

    // Reset to a known twist (zero on coast/estop so resume can't lurch).
    void reset(const Twist& vel) {
        vel_ = vel;
        acc_ = Twist{};
    }

    // Advance one tick toward `target`, bounding accel and jerk. The accel
    // toward a shrinking velocity error is additionally capped at
    // sqrt(2*jerk*|dv|) so the accel itself ramps back to zero right as the
    // velocity reaches its target — that makes the profile an S-curve.
    Twist step(const Twist& target, double dt_in, const ShapeLimits& lim) {
        const double dt = std::clamp(dt_in, 1e-4, 0.05);

        // --- translation, 2D ---
        const Vec2 dv = target.lin - vel_.lin;
        const double dv_len = dv.norm();
        const double acc_cap = std::min(lim.accel, std::sqrt(2.0 * lim.jerk * dv_len));
        const Vec2 want_acc = (dv / dt).clamped(acc_cap);
        const Vec2 jerk_step = (want_acc - Vec2{acc_.lin.x, acc_.lin.y}).clamped(lim.jerk * dt);
        acc_.lin += jerk_step;
        vel_.lin += acc_.lin * dt;
        // Snap tiny residuals (sub-mm/s) so we don't limit-cycle at the target.
        if ((target.lin - vel_.lin).norm() < 2e-4 &&
            acc_.lin.norm() <= lim.jerk * dt + 1e-9) {
            vel_.lin = target.lin;
            acc_.lin = Vec2{};
        }

        // --- yaw, 1D ---
        const double dw = target.ang - vel_.ang;
        const double w_cap = std::min(lim.w_accel, std::sqrt(2.0 * lim.w_jerk * std::abs(dw)));
        const double ad = std::clamp(dw / dt, -w_cap, w_cap);
        const double j = std::clamp(ad - acc_.ang, -lim.w_jerk * dt, lim.w_jerk * dt);
        acc_.ang += j;
        vel_.ang += acc_.ang * dt;
        if (std::abs(target.ang - vel_.ang) < 2e-4 &&
            std::abs(acc_.ang) <= lim.w_jerk * dt + 1e-9) {
            vel_.ang = target.ang;
            acc_.ang = 0.0;
        }

        return vel_;
    }

private:
    Twist vel_{};
    Twist acc_{};
};

// What the caller feeds the executor each control tick.
struct ExecInput {
    uint64_t now_ms = 0;  // caller-monotonic clock, same base as accept()
    double dt = 0.004;    // seconds since last tick
    Twist odo_body;       // measured body twist from wheel odometry
    // Measured yaw rate from the gyro, rad/s CCW+. NaN = IMU down (wheel
    // odometry yaw is used instead).
    double imu_yaw_radps = std::nan("");
};

// The executor's verdict for one tick.
struct ExecOutput {
    Twist twist;            // body twist to actuate (mode/accel/jerk shaped)
    bool energize = false;  // false = coast (cut motor output)
    WatchdogTier wd = WatchdogTier::Coast;
    double dist_m = -1.0;   // distance to the active MOVE target (-1 = none)
    double heading_err_rad = 0.0;
    bool arrived = false;   // position AND heading settled
    Pose est;               // current pose estimate (telemetry/logging)
    Twist desired;          // pre-shaping desired twist (logging)

    static ExecOutput idle(WatchdogTier wd, const Pose& est) {
        ExecOutput o;
        o.wd = wd;
        o.est = est;
        return o;
    }
};

// One per robot; owns the pose estimate, settle latches, and shaper state.
class MoveExecutor {
public:
    MoveExecutor() = default;
    explicit MoveExecutor(const MotionConfig& cfg) : cfg_(cfg) {}

    const MotionConfig& config() const { return cfg_; }

    int64_t last_seq() const { return last_seq_ ? static_cast<int64_t>(*last_seq_) : -1; }

    const char* kind_word() const {
        if (!cmd_) return "-";
        switch (cmd_->kind) {
            case MoveKind::Move: return "MOVE";
            case MoveKind::Hold: return "HOLD";
            case MoveKind::Brake: return "BRAKE";
            case MoveKind::Disable: return "DISABLE";
        }
        return "-";
    }

    bool active() const { return cmd_.has_value(); }

    void kick_dribble(bool& kick, bool& dribble) const {
        kick = cmd_ ? cmd_->kick : false;
        dribble = cmd_ ? cmd_->dribble : false;
    }

    const std::optional<Mv2Command>& active_cmd() const { return cmd_; }
    const std::optional<Pose>& estimate() const { return est_; }

    // Drop all command state (operator STOP): coast until the next frame.
    void clear() {
        cmd_.reset();
        rx_ms_.reset();
        hold_pose_.reset();
        shaper_.reset(Twist{});
        settled_ = false;
        heading_settled_ = false;
        brake_done_ = false;
    }

    // Accept a frame. Duplicates and reordered (older) frames are dropped by
    // sequence number (wrapping compare).
    void accept(const Mv2Command& cmd, uint64_t now_ms) {
        if (last_seq_) {
            const uint32_t ahead = cmd.seq - *last_seq_;
            if (ahead == 0 || ahead > UINT32_MAX / 2) return;
        }

        // Fuse the server pose: extrapolate by its own age, then blend.
        const double age_s = static_cast<double>(cmd.pose_age_ms) / 1000.0;
        Pose server;
        server.pos = {cmd.px + cmd.vx * age_s, cmd.py + cmd.vy * age_s};
        server.heading = wrap_angle(cmd.ptheta + cmd.vw * age_s);
        if (!est_) {
            est_ = server;
        } else {
            const Vec2 d = server.pos - est_->pos;
            if (d.norm() > cfg_.snap_dist_m) {
                est_ = server;
            } else {
                est_->pos += d * cfg_.pose_gain;
                est_->heading = wrap_angle(
                    est_->heading +
                    cfg_.heading_gain * angle_diff(server.heading, est_->heading));
            }
        }

        // HOLD latches the pose at the moment it first arrives.
        const bool was_hold = cmd_ && cmd_->kind == MoveKind::Hold;
        if (cmd.kind == MoveKind::Hold) {
            if (!was_hold || !hold_pose_) hold_pose_ = est_;
        } else {
            hold_pose_.reset();
        }

        // A fresh command re-arms braking.
        if (cmd.kind != MoveKind::Brake) brake_done_ = false;

        cmd_ = cmd;
        rx_ms_ = now_ms;
        last_seq_ = cmd.seq;
    }

    ExecOutput tick(const ExecInput& in) {
        // 1. Dead-reckon the pose estimate with odometry + gyro.
        if (est_) {
            const double dt = std::clamp(in.dt, 0.0, 0.05);
            const double yaw_rate =
                std::isnan(in.imu_yaw_radps) ? in.odo_body.ang : in.imu_yaw_radps;
            const double s = std::sin(est_->heading), c = std::cos(est_->heading);
            est_->pos.x += (in.odo_body.lin.x * c - in.odo_body.lin.y * s) * dt;
            est_->pos.y += (in.odo_body.lin.x * s + in.odo_body.lin.y * c) * dt;
            est_->heading = wrap_angle(est_->heading + yaw_rate * dt);
        }
        const Pose est = est_.value_or(Pose{});

        // 2. Watchdog tier.
        if (!cmd_ || !rx_ms_) {
            shaper_.reset(Twist{});
            return ExecOutput::idle(WatchdogTier::Coast, est);
        }
        const uint64_t age_ms = in.now_ms >= *rx_ms_ ? in.now_ms - *rx_ms_ : 0;
        const WatchdogTier wd = age_ms > cfg_.coast_after_ms  ? WatchdogTier::Coast
                                : age_ms > cfg_.brake_after_ms ? WatchdogTier::Brake
                                                               : WatchdogTier::Fresh;
        if (wd == WatchdogTier::Coast) {
            shaper_.reset(Twist{});
            return ExecOutput::idle(WatchdogTier::Coast, est);
        }

        // 3. Effective kind: watchdog-brake behaves like a BRAKE frame;
        // DISABLE coasts immediately.
        const MoveKind kind = wd == WatchdogTier::Brake ? MoveKind::Brake : cmd_->kind;
        if (kind == MoveKind::Disable) {
            shaper_.reset(Twist{});
            return ExecOutput::idle(wd, est);
        }
        if (kind == MoveKind::Brake) return brake_tick(in, wd, est);

        // MOVE / HOLD: pose law toward the target.
        Pose target;
        ModeLimits lim;
        if (kind == MoveKind::Hold) {
            if (!hold_pose_) hold_pose_ = est;
            target = *hold_pose_;
            lim = cfg_.hold_position;
        } else {
            target.pos = {cmd_->tx, cmd_->ty};
            target.heading = wrap_angle(cmd_->ttheta);
            lim = cfg_.limits(cmd_->mode).tightened(*cmd_);
        }
        const double arrive_speed =
            kind == MoveKind::Hold ? 0.0 : std::max(cmd_->arrive_speed_mps, 0.0);

        // --- translation ---
        const Vec2 e = target.pos - est.pos;
        const double dist = e.norm();
        if (settled_) {
            if (dist > lim.arrive_radius_m + lim.arrive_release_m) settled_ = false;
        } else if (dist <= lim.arrive_radius_m && arrive_speed <= 0.0) {
            settled_ = true;
        }
        double desired_speed = 0.0;
        if (!settled_ && dist >= 1e-9) {
            // Braking envelope (0.8 margin for the jerk phase), terminal P
            // cap, mode cap, creep floor.
            const double envelope = std::sqrt(
                arrive_speed * arrive_speed + 2.0 * 0.8 * lim.max_accel_mps2 * dist);
            double v = std::min({envelope, lim.kp_pos * dist, lim.max_speed_mps});
            if (v < lim.min_speed_mps) v = lim.min_speed_mps;
            desired_speed = v;
        }
        const Vec2 dir = dist > 1e-9 ? e / dist : Vec2{};
        // World -> body.
        const double s = std::sin(est.heading), c = std::cos(est.heading);
        const Vec2 vw = dir * desired_speed;
        const double bvx = vw.x * c + vw.y * s;
        const double bvy = -vw.x * s + vw.y * c;

        // --- heading ---
        const double herr = angle_diff(target.heading, est.heading);
        if (heading_settled_) {
            if (std::abs(herr) > lim.arrive_heading_rad + lim.heading_release_rad)
                heading_settled_ = false;
        } else if (std::abs(herr) <= lim.arrive_heading_rad) {
            heading_settled_ = true;
        }
        double desired_w = 0.0;
        if (!heading_settled_) {
            const double envelope =
                std::sqrt(2.0 * 0.8 * lim.max_w_accel_radps2 * std::abs(herr));
            double w = std::clamp(lim.kp_ang * herr, -envelope, envelope);
            w = std::clamp(w, -lim.max_w_radps, lim.max_w_radps);
            if (std::abs(w) < lim.min_w_radps)
                w = lim.min_w_radps * (herr >= 0 ? 1.0 : -1.0);
            desired_w = w;
        }

        const Twist desired{bvx, bvy, desired_w};
        Twist shaped = shaper_.step(desired, in.dt,
                                    ShapeLimits{lim.max_accel_mps2, lim.max_jerk_mps3,
                                                lim.max_w_accel_radps2, lim.max_w_jerk_radps3});
        // Hard mode-cap invariant: the S-curve integration can overshoot the
        // cap by up to one accel*dt step; trim it (direction-preserving).
        shaped.lin = shaped.lin.clamped(lim.max_speed_mps);
        shaped.ang = std::clamp(shaped.ang, -lim.max_w_radps, lim.max_w_radps);

        ExecOutput out;
        out.twist = shaped;
        out.energize = true;
        out.wd = wd;
        out.dist_m = dist;
        out.heading_err_rad = herr;
        out.arrived = settled_ && heading_settled_;
        out.est = est;
        out.desired = desired;
        return out;
    }

private:
    // BRAKE: shape hard toward zero; once both the commanded twist and the
    // measured odometry are stopped, coast until a new command arms motion.
    ExecOutput brake_tick(const ExecInput& in, WatchdogTier wd, const Pose& est) {
        const Twist shaped =
            shaper_.step(Twist{}, in.dt,
                         ShapeLimits{cfg_.brake_decel_mps2, cfg_.brake_jerk_mps3,
                                     cfg_.brake_w_decel_radps2, cfg_.brake_jerk_mps3 * 4.0});
        const bool cmd_stopped = shaped.lin.norm() < 0.02 && std::abs(shaped.ang) < 0.05;
        const bool meas_stopped =
            in.odo_body.lin.norm() < 0.05 && std::abs(in.odo_body.ang) < 0.1;
        if (cmd_stopped && meas_stopped) {
            brake_done_ = true;
            shaper_.reset(Twist{});
        }
        ExecOutput out;
        out.twist = brake_done_ ? Twist{} : shaped;
        out.energize = !brake_done_;
        out.wd = wd;
        out.dist_m = -1.0;
        out.heading_err_rad = 0.0;
        out.arrived = brake_done_;
        out.est = est;
        out.desired = Twist{};
        return out;
    }

    MotionConfig cfg_{};
    std::optional<Mv2Command> cmd_;
    std::optional<uint64_t> rx_ms_;
    std::optional<uint32_t> last_seq_;
    std::optional<Pose> est_;
    std::optional<Pose> hold_pose_;
    TwistShaper shaper_;
    bool settled_ = false;
    bool heading_settled_ = false;
    bool brake_done_ = false;
};

}  // namespace phx
