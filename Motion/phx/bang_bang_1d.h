// Time-optimal 1D trajectory for a double integrator with velocity and
// acceleration limits (classic "bang-bang" profile, TIGERs Mannheim style),
// generalized to a nonzero terminal velocity so a single command model covers
// stop-at-point, pass-through, and moving interception.
//
// The planner returns a piecewise-constant-acceleration profile with at most
// four segments: [optional brake from over-speed] + [ramp] + [cruise] + [ramp].
// No heap allocation — this header is shared with the robot firmware.
//
// Frames/units: metres, seconds. Position is 1D; the 2D planner composes two
// of these with a shared limit split (see bang_bang_2d.h).
// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace phx {

class BangBang1D {
public:
    struct Segment {
        double dt = 0;  // duration
        double x0 = 0;  // position at segment start
        double v0 = 0;  // velocity at segment start
        double a = 0;   // constant acceleration over the segment
    };

    // Plan from state (x0, v0) to state (x1, v1) under |v| <= vmax, |a| <= amax.
    // v1 is clamped into [-vmax, vmax]. Handles |v0| > vmax by braking first.
    static BangBang1D plan(double x0, double v0, double x1, double v1, double vmax,
                           double amax, double brake_max = -1.0) {
        BangBang1D tr;
        vmax = std::max(vmax, kMinLimit);
        amax = std::max(amax, kMinLimit);
        brake_max = brake_max > 0.0 ? brake_max : amax;
        brake_max = std::max(brake_max, kMinLimit);
        v1 = std::clamp(v1, -vmax, vmax);

        double x = x0, v = v0;
        // Entering over the velocity limit (limit change, collision, handover):
        // brake to the limit first, then plan the remainder.
        if (std::abs(v) > vmax) {
            const double dir = v > 0 ? 1.0 : -1.0;
            const double dt = (std::abs(v) - vmax) / brake_max;
            tr.push(dt, x, v, -dir * brake_max);
            x += v * dt - 0.5 * dir * brake_max * dt * dt;
            v = dir * vmax;
        }

        Candidate best =
            choose_profile(x, v, x1, v1, vmax, amax, brake_max);
        if (!best.valid) {
            // Defensive numeric corner (unreachable in exact arithmetic: one
            // family always covers the displacement). Brake to rest, then plan
            // from standstill, where both root constraints are provably
            // satisfiable for every displacement.
            const double dir = v >= 0 ? 1.0 : -1.0;
            const double t_stop = std::abs(v) / brake_max;
            tr.push(t_stop, x, v, -dir * brake_max);
            x += v * t_stop - 0.5 * dir * brake_max * t_stop * t_stop;
            v = 0.0;
            best = choose_profile(x, v, x1, v1, vmax, amax, brake_max);
        }
        for (int i = 0; i < 3; ++i) {
            if (best.t[i] <= 0.0) continue;
            tr.push(best.t[i], x, v, best.a[i]);
            x += v * best.t[i] + 0.5 * best.a[i] * best.t[i] * best.t[i];
            v += best.a[i] * best.t[i];
        }

        tr.end_x_ = x;
        tr.end_v_ = v;
        return tr;
    }

    // State sampling. Before t=0 the initial state is returned; after the end
    // the profile continues at constant terminal velocity (honest for nonzero
    // v1; a natural hold for v1 = 0).
    double pos(double t) const {
        if (n_ == 0) return end_x_ + end_v_ * std::max(0.0, t);
        if (t <= 0) return seg_[0].x0;
        double rem = t;
        for (int i = 0; i < n_; ++i) {
            const Segment& s = seg_[i];
            if (rem <= s.dt) return s.x0 + s.v0 * rem + 0.5 * s.a * rem * rem;
            rem -= s.dt;
        }
        return end_x_ + end_v_ * rem;
    }

    double vel(double t) const {
        if (n_ == 0) return end_v_;
        if (t <= 0) return seg_[0].v0;
        double rem = t;
        for (int i = 0; i < n_; ++i) {
            const Segment& s = seg_[i];
            if (rem <= s.dt) return s.v0 + s.a * rem;
            rem -= s.dt;
        }
        return end_v_;
    }

    double acc(double t) const {
        if (t < 0) return 0;
        double rem = t;
        for (int i = 0; i < n_; ++i) {
            const Segment& s = seg_[i];
            if (rem <= s.dt) return s.a;
            rem -= s.dt;
        }
        return 0;
    }

    double total_time() const { return total_; }
    double end_pos() const { return end_x_; }
    double end_vel() const { return end_v_; }
    int segment_count() const { return n_; }
    const Segment& segment(int i) const { return seg_[static_cast<size_t>(i)]; }

private:
    static constexpr double kMinLimit = 1e-6;
    static constexpr double kEps = 1e-9;

    struct Candidate {
        bool valid = false;
        double t[3] = {0, 0, 0};
        double a[3] = {0, 0, 0};
        double total = 0;
    };

    // Build the ramp-(cruise)-ramp profile whose first acceleration has sign
    // `dir`. dir=+1: accelerate to a peak, then decelerate ("up-down").
    // dir=-1: decelerate to a valley, then accelerate ("down-up").
    static Candidate make_profile(double dir, double x0, double v0, double x1,
                                  double v1, double vmax, double accel_max,
                                  double brake_max) {
        Candidate c;
        const double accel = dir * accel_max;
        const double brake = -dir * brake_max;
        const double s = x1 - x0;
        double arg =
            (2.0 * dir * s + v0 * v0 / accel_max + v1 * v1 / brake_max) /
            (1.0 / accel_max + 1.0 / brake_max);
        if (arg < -kEps) return c;  // this family cannot cover s
        arg = std::max(arg, 0.0);
        const double root = std::sqrt(arg);

        // Extreme (peak/valley) velocity between the two ramps. Both signed
        // roots solve the displacement equation; validity requires nonnegative
        // ramp durations, and among valid roots the one closer to the
        // endpoints is faster.
        double vext = 0;
        bool found = false;
        if (dir > 0) {
            // need vext >= max(v0, v1); prefer the smaller valid root
            for (double cand : {root, -root}) {
                if (cand >= std::max(v0, v1) - kTolFor(vmax)) {
                    if (!found || cand < vext) { vext = cand; found = true; }
                }
            }
        } else {
            // need vext <= min(v0, v1); prefer the larger valid root
            for (double cand : {-root, root}) {
                if (cand <= std::min(v0, v1) + kTolFor(vmax)) {
                    if (!found || cand > vext) { vext = cand; found = true; }
                }
            }
        }
        if (!found) return c;

        if (std::abs(vext) <= vmax) {
            c.t[0] = std::abs(vext - v0) / accel_max;
            c.a[0] = accel;
            c.t[1] = 0;
            c.t[2] = std::abs(v1 - vext) / brake_max;
            c.a[2] = brake;
        } else {
            // Clamp at the velocity limit and insert a cruise segment.
            const double vc = dir * vmax;
            const double d1 = (vc * vc - v0 * v0) / (2.0 * accel);
            const double d3 = (v1 * v1 - vc * vc) / (2.0 * brake);
            double d2 = s - d1 - d3;
            double t2 = d2 / vc;
            if (t2 < -kTolFor(vmax)) return c;  // numeric corner: not this family
            t2 = std::max(t2, 0.0);
            c.t[0] = std::abs(vc - v0) / accel_max;
            c.a[0] = accel;
            c.t[1] = t2;
            c.a[1] = 0;
            c.t[2] = std::abs(v1 - vc) / brake_max;
            c.a[2] = brake;
        }
        c.total = c.t[0] + c.t[1] + c.t[2];
        c.valid = true;
        return c;
    }

    static Candidate choose_profile(double x0, double v0, double x1, double v1,
                                    double vmax, double accel_max,
                                    double brake_max) {
        const Candidate up =
            make_profile(+1.0, x0, v0, x1, v1, vmax, accel_max, brake_max);
        const Candidate down =
            make_profile(-1.0, x0, v0, x1, v1, vmax, accel_max, brake_max);
        if (up.valid && down.valid) return up.total <= down.total ? up : down;
        return up.valid ? up : down;
    }

    static double kTolFor(double scale) { return kEps * std::max(1.0, scale); }

    void push(double dt, double x0, double v0, double a) {
        if (dt <= 0.0) return;
        seg_[static_cast<size_t>(n_)] = {dt, x0, v0, a};
        ++n_;
        total_ += dt;
    }

    // Worst case: over-speed brake + defensive brake-to-rest + ramp/cruise/ramp.
    std::array<Segment, 5> seg_{};
    int n_ = 0;
    double total_ = 0;
    double end_x_ = 0;
    double end_v_ = 0;
};

}  // namespace phx
