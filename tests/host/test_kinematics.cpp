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

PHX_TEST(kin_forward_pure_rotation_has_no_net_translation) {
    const Kinematics k;
    const auto wheels = k.inverse(BodyTwist{0.0, 0.0, 1.5});
    const BodyTwist est = k.forward(wheels);
    CHECK_NEAR(est.vx, 0.0, 1e-6);
    CHECK_NEAR(est.vy, 0.0, 1e-6);
    CHECK_NEAR(est.w, 1.5, 1e-6);
}

PHX_TEST(kin_inverse_forward_roundtrip_twist_grid) {
    // The wheel odometry (FK) must reproduce any commanded twist that IK
    // emitted — across a grid of translations, rotations, and mixtures.
    const Kinematics k;
    const double vxs[] = {-0.6, -0.2, 0.0, 0.3, 0.5, 1.2};
    const double vys[] = {-0.4, 0.0, 0.1, 0.4};
    const double ws[] = {-1.2, 0.0, 0.8, 2.0};
    for (const double vx : vxs) {
        for (const double vy : vys) {
            for (const double w : ws) {
                const BodyTwist t{vx, vy, w};
                const BodyTwist est = k.forward(k.inverse(t));
                CHECK_NEAR(est.vx, t.vx, 1e-6);
                CHECK_NEAR(est.vy, t.vy, 1e-6);
                CHECK_NEAR(est.w, t.w, 1e-6);
            }
        }
    }
}

PHX_TEST(kin_lateral_effectiveness_scale_is_symmetric_with_odometry) {
    Kinematics nominal;
    Kinematics calibrated;
    calibrated.body_lateral_scale = 1.084;
    const BodyTwist lateral{0.0, 1.0, 0.0};
    const auto nominal_wheels = nominal.inverse(lateral);
    const auto calibrated_wheels = calibrated.inverse(lateral);
    for (int i = 0; i < 4; ++i) {
        CHECK_NEAR(
            std::abs(calibrated_wheels[i] / nominal_wheels[i]),
            1.084,
            1e-9
        );
    }
    const BodyTwist recovered = calibrated.forward(calibrated_wheels);
    CHECK_NEAR(recovered.vx, lateral.vx, 1e-9);
    CHECK_NEAR(recovered.vy, lateral.vy, 1e-9);
    CHECK_NEAR(recovered.w, lateral.w, 1e-9);
}

PHX_TEST(kin_translation_yaw_coupling_compensation_roundtrips) {
    Kinematics k;
    k.yaw_ff_from_vx = 0.288;
    k.yaw_ff_from_vy = 0.361;

    const BodyTwist translation{1.0, -0.4, 0.0};
    const auto compensated_wheels = k.inverse(translation);
    const auto uncompensated_wheels = Kinematics{}.inverse(translation);
    bool changed = false;
    for (int i = 0; i < 4; ++i) {
        changed = changed
            || std::abs(compensated_wheels[i] - uncompensated_wheels[i]) > 1e-6;
    }
    CHECK(changed);

    const BodyTwist recovered = k.forward(compensated_wheels);
    CHECK_NEAR(recovered.vx, translation.vx, 1e-9);
    CHECK_NEAR(recovered.vy, translation.vy, 1e-9);
    CHECK_NEAR(recovered.w, translation.w, 1e-9);
}

PHX_TEST(kin_wheel_command_scale_does_not_fabricate_forward_odometry) {
    Kinematics k;
    const BodyTwist t{0.4, -0.2, 0.3};
    const auto nominal = k.inverse(t);
    k.wheel_command_scale = {{1.01, 1.02, 0.99, 1.03}};
    const auto corrected = k.inverse(t);
    for (int i = 0; i < 4; ++i) {
        CHECK_NEAR(
            corrected[i],
            nominal[i] * k.wheel_command_scale[i],
            1e-9
        );
    }
    // FK reads physical encoder speeds directly and must not divide by a
    // command-side normalization factor.
    const BodyTwist measured = k.forward(nominal);
    CHECK_NEAR(measured.vx, t.vx, 1e-9);
    CHECK_NEAR(measured.vy, t.vy, 1e-9);
    CHECK_NEAR(measured.w, t.w, 1e-9);
}

PHX_TEST(kin_pure_west_command_trim_does_not_touch_diagonals) {
    Kinematics tuned;
    tuned.wheel_command_scale_west = {{0.98, 1.0, 1.0, 1.013}};
    Kinematics nominal;

    const BodyTwist west{-2.0, 0.0, 0.0};
    const auto tuned_west = tuned.inverse(west);
    const auto nominal_west = nominal.inverse(west);
    CHECK_NEAR(tuned_west[0], nominal_west[0] * 0.98, 1e-9);
    CHECK_NEAR(tuned_west[3], nominal_west[3] * 1.013, 1e-9);

    const BodyTwist diagonal{-2.0, 2.0, 0.0};
    const auto tuned_diagonal = tuned.inverse(diagonal);
    const auto nominal_diagonal = nominal.inverse(diagonal);
    for (int i = 0; i < 4; ++i) {
        CHECK_NEAR(tuned_diagonal[i], nominal_diagonal[i], 1e-9);
    }
}

PHX_TEST(kin_pure_east_command_trim_does_not_touch_diagonals_or_west) {
    Kinematics tuned;
    tuned.wheel_command_scale_east = {{1.0, 1.02, 0.98, 1.0}};
    Kinematics nominal;

    const auto tuned_east = tuned.inverse({2.0, 0.0, 0.0});
    const auto nominal_east = nominal.inverse({2.0, 0.0, 0.0});
    CHECK_NEAR(tuned_east[1], nominal_east[1] * 1.02, 1e-9);
    CHECK_NEAR(tuned_east[2], nominal_east[2] * 0.98, 1e-9);

    for (const BodyTwist untouched :
         {BodyTwist{2.0, 2.0, 0.0}, BodyTwist{-2.0, 0.0, 0.0}}) {
        const auto a = tuned.inverse(untouched);
        const auto b = nominal.inverse(untouched);
        for (int i = 0; i < 4; ++i) CHECK_NEAR(a[i], b[i], 1e-9);
    }
}

PHX_TEST(kin_wheel_command_scale_can_equalize_positive_and_negative_directions) {
    Kinematics k;
    k.wheel_command_scale_positive = {{1.01, 1.02, 1.03, 1.04}};
    k.wheel_command_scale_negative = {{0.99, 0.98, 0.97, 0.96}};
    const auto nominal_positive = Kinematics{}.inverse({0.6, 0.0, 0.0});
    const auto corrected_positive = k.inverse({0.6, 0.0, 0.0});
    const auto nominal_negative = Kinematics{}.inverse({-0.6, 0.0, 0.0});
    const auto corrected_negative = k.inverse({-0.6, 0.0, 0.0});
    for (int i = 0; i < 4; ++i) {
        const double expected_positive = nominal_positive[i] >= 0.0
            ? k.wheel_command_scale_positive[i]
            : k.wheel_command_scale_negative[i];
        const double expected_negative = nominal_negative[i] >= 0.0
            ? k.wheel_command_scale_positive[i]
            : k.wheel_command_scale_negative[i];
        CHECK_NEAR(
            corrected_positive[i], nominal_positive[i] * expected_positive, 1e-9);
        CHECK_NEAR(
            corrected_negative[i], nominal_negative[i] * expected_negative, 1e-9);
    }
}

PHX_TEST(kin_forward_zero_wheels_is_zero_twist) {
    const Kinematics k;
    const BodyTwist est = k.forward({0.0, 0.0, 0.0, 0.0});
    CHECK(est.vx == 0.0);
    CHECK(est.vy == 0.0);
    CHECK(est.w == 0.0);
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
