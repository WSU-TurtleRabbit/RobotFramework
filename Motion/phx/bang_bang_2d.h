// 2D time-optimal trajectory to rest: two BangBang1D axes with the velocity
// and acceleration budgets split by an angle alpha (x gets cos(alpha), y gets
// sin(alpha)). Alpha is found by bisection so both axes arrive together — the
// TIGERs Mannheim synchronization scheme. The planar speed and acceleration
// never exceed the scalar limits: sqrt((L cos a)^2 + (L sin a)^2) = L.
//
// Terminal velocity is deliberately ZERO here. Synchronized point-to-point
// arrival with a nonzero terminal velocity is ill-posed in the bang-bang
// family once the terminal speed pins an axis budget (re-targeting then saves
// time exactly as fast as it adds drift). Pass-through and arrive-with-speed
// semantics belong to the braking-envelope follower (motion follower module),
// which consumes the protocol's arrive-speed field directly; this planner
// provides time parameterization, ETAs, and reference profiles.
//
// Shared with the robot firmware — no heap allocation, no dependencies.
// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3. Only the include paths
// were adapted; the logic is byte-for-byte the server's.
#pragma once

#include <algorithm>
#include <cmath>

#include "vec2.h"
#include "bang_bang_1d.h"

namespace phx {

class BangBang2D {
public:
    // Plan from (p0, v0) to rest at p1 under planar |v| <= vmax, |a| <= amax.
    static BangBang2D plan(Vec2 p0, Vec2 v0, Vec2 p1, double vmax, double amax,
                           double brake_max = -1.0) {
        BangBang2D tr;
        vmax = std::max(vmax, 1e-6);
        amax = std::max(amax, 1e-6);
        brake_max = brake_max > 0.0 ? brake_max : amax;

        const double lo = 1e-3, hi = kPiHalf - 1e-3;
        const auto plan_at = [&](double alpha, BangBang1D& x, BangBang1D& y) {
            const double c = std::cos(alpha), s = std::sin(alpha);
            x = BangBang1D::plan(
                p0.x, v0.x, p1.x, 0.0, vmax * c, amax * c, brake_max * c);
            y = BangBang1D::plan(
                p0.y, v0.y, p1.y, 0.0, vmax * s, amax * s, brake_max * s);
        };

        // f(alpha) = Tx - Ty is monotonically increasing: growing alpha takes
        // budget from x and gives it to y. Bisect for the root. If one axis
        // dominates even at the window edge, the other simply finishes early
        // and holds position (terminal velocity is zero, so no drift).
        BangBang1D x, y;
        double alpha;
        plan_at(lo, x, y);
        if (x.total_time() - y.total_time() >= 0) {
            alpha = lo;
        } else {
            plan_at(hi, x, y);
            if (x.total_time() - y.total_time() <= 0) {
                alpha = hi;
            } else {
                double a = lo, b = hi;
                for (int i = 0; i < 48; ++i) {
                    const double mid = 0.5 * (a + b);
                    plan_at(mid, x, y);
                    if (x.total_time() - y.total_time() < 0)
                        a = mid;
                    else
                        b = mid;
                }
                alpha = 0.5 * (a + b);
                plan_at(alpha, x, y);
            }
        }
        tr.alpha_ = alpha;
        tr.x_ = x;
        tr.y_ = y;
        return tr;
    }

    Vec2 pos(double t) const { return {x_.pos(t), y_.pos(t)}; }
    Vec2 vel(double t) const { return {x_.vel(t), y_.vel(t)}; }
    Vec2 acc(double t) const { return {x_.acc(t), y_.acc(t)}; }
    double total_time() const { return std::max(x_.total_time(), y_.total_time()); }
    Vec2 end_pos() const { return {x_.end_pos(), y_.end_pos()}; }
    double alpha() const { return alpha_; }
    const BangBang1D& x_axis() const { return x_; }
    const BangBang1D& y_axis() const { return y_; }

private:
    static constexpr double kPiHalf = 1.5707963267948966;

    BangBang1D x_;
    BangBang1D y_;
    double alpha_ = 0.7853981633974483;
};

}  // namespace phx
