// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3.
//
// Planar pose and twist. Global frame: SSL field coordinates in metres,
// heading in radians CCW from +x. Body frame: +x forward, +y left.
// This header is shared with the server — keep it dependency-free.
#pragma once

#include "angle.h"
#include "vec2.h"

namespace phx {

struct Pose {
    Vec2 pos;            // metres, global frame
    double heading = 0;  // radians, wrapped (-pi, pi]

    bool finite() const { return pos.finite() && std::isfinite(heading); }
};

// A planar velocity. Frame depends on context and is documented at each use:
// global twists for world state, body twists on the robot wire.
struct Twist {
    Vec2 lin;         // m/s
    double ang = 0;   // rad/s, CCW positive

    constexpr Twist() = default;
    constexpr Twist(Vec2 lin_, double ang_) : lin(lin_), ang(ang_) {}
    constexpr Twist(double vx, double vy, double w) : lin(vx, vy), ang(w) {}

    Twist operator+(const Twist& o) const { return {lin + o.lin, ang + o.ang}; }
    Twist operator-(const Twist& o) const { return {lin - o.lin, ang - o.ang}; }
    Twist operator*(double s) const { return {lin * s, ang * s}; }

    bool finite() const { return lin.finite() && std::isfinite(ang); }
};

// Rotate a global-frame twist into the body frame of a robot at `heading`.
inline Twist global_to_body(const Twist& global, double heading) {
    return {global.lin.rotated(-heading), global.ang};
}

// Rotate a body-frame twist into the global frame for a robot at `heading`.
inline Twist body_to_global(const Twist& body, double heading) {
    return {body.lin.rotated(heading), body.ang};
}

}  // namespace phx
