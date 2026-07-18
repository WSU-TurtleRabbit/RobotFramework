// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3. Extracted from
// phoenix/motion/executor.h (the accel+jerk S-curve TwistShaper) — the
// executor itself left with the MV2 protocol; the shaper stays: it shapes
// velocity-skill setpoints in the MatchCtrl cascade.
//
// Acceleration- and jerk-limited twist shaper (S-curve velocity profile).
// Translation is shaped as a 2D vector (diagonals aren't bent toward an
// axis); yaw is shaped independently. Pure and deterministic.
#pragma once

#include <algorithm>
#include <cmath>

#include "pose.h"

namespace phx {

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

}  // namespace phx
