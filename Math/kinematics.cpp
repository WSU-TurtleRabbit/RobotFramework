#include "kinematics.h"

#include <algorithm>

std::array<double, 4> Kinematics::inverse(const BodyTwist& t) const {
    std::array<double, 4> out{};
    for (int i = 0; i < 4; ++i) {
        const auto a = wheels[i].row();
        const double surface_mps = a[0] * t.vx + a[1] * t.vy + a[2] * t.w;
        out[i] = surface_mps / meters_per_motor_rev;
    }
    return out;
}

double Kinematics::peak_motor_rev_s(const BodyTwist& t) const {
    const auto v = inverse(t);
    double m = 0.0;
    for (const double x : v) m = std::max(m, std::abs(x));
    return m;
}
