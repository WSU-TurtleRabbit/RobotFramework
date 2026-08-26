#include "kinematics.h"

#include <algorithm>
#include <optional>

namespace {

double west_blend(double vx, double vy) {
    if (vx >= 0.0) return 0.0;
    const double ax = std::abs(vx);
    const double ay = std::abs(vy);
    return std::clamp((ax - ay) / std::max(ax, 1e-9), 0.0, 1.0);
}

double east_blend(double vx, double vy) {
    if (vx <= 0.0) return 0.0;
    const double ax = std::abs(vx);
    const double ay = std::abs(vy);
    return std::clamp((ax - ay) / std::max(ax, 1e-9), 0.0, 1.0);
}

// Solve a 3x3 linear system by cofactor inversion. nullopt if near-singular
// (degenerate wheel geometry) — the caller then reports a zero twist rather
// than garbage.
std::optional<std::array<double, 3>> solve3(const double a[3][3], const double b[3]) {
    const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                       a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                       a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (std::abs(det) < 1e-12) return std::nullopt;
    const double inv_det = 1.0 / det;
    // Adjugate (transpose of cofactors) times b.
    const double c[3][3] = {
        {(a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
         (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
         (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det},
        {(a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
         (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
         (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det},
        {(a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
         (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
         (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det},
    };
    return std::array<double, 3>{
        c[0][0] * b[0] + c[0][1] * b[1] + c[0][2] * b[2],
        c[1][0] * b[0] + c[1][1] * b[1] + c[1][2] * b[2],
        c[2][0] * b[0] + c[2][1] * b[1] + c[2][2] * b[2],
    };
}

}  // namespace

std::array<double, 4> Kinematics::inverse(const BodyTwist& t) const {
    std::array<double, 4> out{};
    const double lateral_scale = std::max(0.1, body_lateral_scale);
    const double west = west_blend(t.vx, t.vy);
    const double east = east_blend(t.vx, t.vy);
    const double compensated_w =
        t.w + yaw_ff_from_vx * t.vx + yaw_ff_from_vy * t.vy;
    for (int i = 0; i < 4; ++i) {
        const auto a = wheels[i].row();
        const double surface_mps =
            a[0] * t.vx + a[1] * (t.vy * lateral_scale) + a[2] * compensated_w;
        const double direction_scale =
            surface_mps >= 0.0 ? wheel_command_scale_positive[i]
                               : wheel_command_scale_negative[i];
        const double west_scale =
            1.0 + west * (wheel_command_scale_west[i] - 1.0);
        const double east_scale =
            1.0 + east * (wheel_command_scale_east[i] - 1.0);
        out[i] =
            surface_mps / meters_per_motor_rev
            * std::clamp(wheel_command_scale[i], 0.5, 1.5)
            * std::clamp(direction_scale, 0.8, 1.2)
            * std::clamp(west_scale, 0.8, 1.2)
            * std::clamp(east_scale, 0.8, 1.2);
    }
    return out;
}

BodyTwist Kinematics::forward(const std::array<double, 4>& motor_rev_s) const {
    // Wheel surface speeds (m/s).
    double speeds[4];
    for (int i = 0; i < 4; ++i) {
        speeds[i] = motor_rev_s[i] * meters_per_motor_rev;
    }
    // Build J (4x3) and solve (J^T J) x = J^T b.
    std::array<double, 3> rows[4] = {
        wheels[0].row(), wheels[1].row(), wheels[2].row(), wheels[3].row()};
    double ata[3][3] = {};
    double atb[3] = {};
    for (int i = 0; i < 4; ++i) {
        for (int a = 0; a < 3; ++a) {
            atb[a] += rows[i][a] * speeds[i];
            for (int b = 0; b < 3; ++b) {
                ata[a][b] += rows[i][a] * rows[i][b];
            }
        }
    }
    const auto x = solve3(ata, atb);
    if (!x) return BodyTwist{};
    const double vx = (*x)[0];
    const double vy = (*x)[1] / std::max(0.1, body_lateral_scale);
    const double w = (*x)[2] - yaw_ff_from_vx * vx - yaw_ff_from_vy * vy;
    return BodyTwist{vx, vy, w};
}

double Kinematics::peak_motor_rev_s(const BodyTwist& t) const {
    const auto v = inverse(t);
    double m = 0.0;
    for (const double x : v) m = std::max(m, std::abs(x));
    return m;
}
