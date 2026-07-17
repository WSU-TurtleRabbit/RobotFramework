// Inverse kinematics: geometry table, cross-product moment arms, and the
// rev/s unit chain. Test expectations ported from phoenix-rf
// crates/protocol/src/kinematics.rs (verified on hardware).
#include <cmath>

#include "Math/kinematics.h"
#include "Motion/phx/testing.h"

PHX_TEST(kin_rear_wheel_angle_is_135_not_130) {
    // Regression for the legacy typo (wheel_math.h W3_ANG = -130): the rear
    // left wheel's drive angle must be -135 like its mirror twin's -45.
    const Kinematics k;
    CHECK_NEAR(k.wheels[2].drive_angle_deg, -135.0, 1e-12);
}

PHX_TEST(kin_straight_x_drive_is_symmetric) {
    // Driving straight forward must load the wheels symmetrically:
    // front pair mirrored (w1 = -w4), rear pair mirrored (w2 = -w3).
    // With the old -130 typo the rear pair broke symmetry and the robot
    // curved when commanded straight.
    const Kinematics k;
    const auto out = k.inverse(BodyTwist{1.0, 0.0, 0.0});
    CHECK_NEAR(out[0], -out[3], 1e-9);
    CHECK_NEAR(out[1], -out[2], 1e-9);
    CHECK(std::abs(out[0]) > 0.1);
    CHECK(std::abs(out[1]) > 0.1);
}

PHX_TEST(kin_straight_y_drive_is_symmetric) {
    const Kinematics k;
    const auto out = k.inverse(BodyTwist{0.0, 1.0, 0.0});
    CHECK_NEAR(out[0], out[3], 1e-9);
    CHECK_NEAR(out[1], out[2], 1e-9);
}

PHX_TEST(kin_moment_arms_are_equal_and_positive) {
    // The cross-product arm (x*sinB - y*cosB) must resolve to ~+|r| for every
    // wheel of this tangential layout — that is what makes pure rotation
    // clean. (The legacy code used |r| blindly; correct here by geometry.)
    const Kinematics k;
    const double r0 = k.wheels[0].row()[2];
    CHECK(r0 > 0.0);
    for (const auto& w : k.wheels) {
        CHECK_NEAR(w.row()[2], r0, 1e-3);
    }
}

PHX_TEST(kin_pure_rotation_spins_all_wheels_same_sign_and_similar) {
    // The geometry is only ~symmetric (front |r| 73.5mm, rear 73.7mm), so
    // pure yaw drives the wheels within ~0.5% of each other, same sign.
    const Kinematics k;
    const auto out = k.inverse(BodyTwist{0.0, 0.0, 1.0});
    double mean = 0.0;
    for (const double v : out) mean += v;
    mean /= 4.0;
    CHECK(std::abs(mean) > 0.0);
    for (const double v : out) {
        CHECK((v > 0) == (mean > 0));
        CHECK(std::abs(v - mean) / std::abs(mean) < 0.02);
    }
}

PHX_TEST(kin_scale_is_physical_direct_drive_rev_s) {
    // 1 m/s on a perfectly aligned wheel is 1/(2*pi*R) ~ 4.75 motor REV/s.
    // The legacy chain divided by R alone (~29.9 "rev/s" actually rad/s) — a
    // hidden 2*pi error.
    const Kinematics k;
    const double rev_per_mps = 1.0 / k.meters_per_motor_rev;
    CHECK_NEAR(rev_per_mps, 4.75, 0.05);
    // And through inverse(): no wheel demand for a 1 m/s twist may exceed it.
    CHECK(k.peak_motor_rev_s(BodyTwist{1.0, 0.0, 0.0}) <= rev_per_mps + 1e-9);
}

PHX_TEST(kin_sign_flip_inverts_that_wheel_only) {
    Kinematics k;
    const auto base = k.inverse(BodyTwist{0.0, 0.0, 1.0});
    k.wheels[2].sign = -1.0;
    const auto flipped = k.inverse(BodyTwist{0.0, 0.0, 1.0});
    CHECK_NEAR(flipped[2], -base[2], 1e-9);
    CHECK_NEAR(flipped[0], base[0], 1e-9);
    CHECK_NEAR(flipped[1], base[1], 1e-9);
    CHECK_NEAR(flipped[3], base[3], 1e-9);
}
