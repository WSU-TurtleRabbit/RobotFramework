// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3.
//
// Angle utilities. All angles are radians; wrapped range is (-pi, pi].
// This header is shared with the server — keep it dependency-free.
#pragma once

#include <cmath>
#include <numbers>

namespace phx {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Wrap into (-pi, pi].
inline double wrap_angle(double rad) {
    double a = std::fmod(rad + kPi, kTwoPi);
    if (a <= 0.0) a += kTwoPi;
    return a - kPi;
}

// Shortest signed rotation that takes `from` onto `to`, in (-pi, pi].
inline double angle_diff(double to, double from) { return wrap_angle(to - from); }

inline double deg_to_rad(double deg) { return deg * (kPi / 180.0); }
inline double rad_to_deg(double rad) { return rad * (180.0 / kPi); }

}  // namespace phx
