// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3.
//
// 2D vector math. SI units throughout: metres, radians, seconds.
// This header is shared with the server — keep it dependency-free.
#pragma once

#include <cmath>

namespace phx {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator-() const { return {-x, -y}; }
    constexpr Vec2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vec2 operator/(double s) const { return {x / s, y / s}; }
    constexpr Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

    constexpr double dot(Vec2 o) const { return x * o.x + y * o.y; }
    // z-component of the 3D cross product; positive when `o` is CCW of *this.
    constexpr double cross(Vec2 o) const { return x * o.y - y * o.x; }
    constexpr double norm_sq() const { return x * x + y * y; }
    double norm() const { return std::sqrt(norm_sq()); }
    double angle() const { return std::atan2(y, x); }

    double distance_to(Vec2 o) const { return (o - *this).norm(); }

    // Unit vector; returns (0,0) for vectors shorter than `eps` instead of NaN.
    Vec2 normalized(double eps = 1e-9) const {
        const double n = norm();
        return n > eps ? Vec2{x / n, y / n} : Vec2{};
    }

    // Clamp magnitude to `max_norm` (no-op for shorter vectors).
    Vec2 clamped(double max_norm) const {
        const double n = norm();
        if (n <= max_norm || n < 1e-12) return *this;
        return *this * (max_norm / n);
    }

    // Rotate CCW by `rad`.
    Vec2 rotated(double rad) const {
        const double c = std::cos(rad), s = std::sin(rad);
        return {x * c - y * s, x * s + y * c};
    }

    static Vec2 from_angle(double rad, double len = 1.0) {
        return {len * std::cos(rad), len * std::sin(rad)};
    }

    bool finite() const { return std::isfinite(x) && std::isfinite(y); }
};

constexpr Vec2 operator*(double s, Vec2 v) { return v * s; }

}  // namespace phx
