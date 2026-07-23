// Holonomic 4-wheel omni kinematics — pure geometry, no I/O, no config
// loading. Ported from the field-proven Rust implementation in phoenix-rf
// (crates/protocol/src/kinematics.rs, verified on hardware).
//
// Body frame (matches the command channel):
//   vx = forward, m/s (+)     vy = strafe-left, m/s (+)     w = yaw, rad/s CCW+
//
// Each wheel i is described by its contact position r_i = (x_i, y_i) in the
// body frame (meters) and a drive-direction angle beta_i — the body-frame
// direction the contact point is pushed when the motor turns positive. The
// scalar surface speed the motor must produce is the projection of the
// wheel's contact velocity onto that drive direction:
//
//   contact_velocity_i = v_body + w x r_i = (vx - w*y_i,  vy + w*x_i)
//   surface_i [m/s] = sign_i * (cosB*vx + sinB*vy + (x_i*sinB - y_i*cosB)*w)
//
// The (x*sinB - y*cosB) term is the wheel's true moment arm (z-component of
// r x drive_dir). The legacy wheel_math used |r_i| instead, which only
// coincides with the true arm for perfectly tangential wheels.
//
// Converting surface speed to a moteus velocity command (output rev/s) uses
// ONE measured calibration constant, meters_per_motor_rev — the distance the
// wheel carries the body per motor output revolution (2*pi*R for a
// direct-drive wheel, scaled by any gearbox):
//
//   motor_rev_s_i = surface_i / meters_per_motor_rev
//
// The legacy wheel_math divided surface speed by the wheel RADIUS and sent
// the result (rad/s) straight to moteus, which interprets the velocity field
// as REV/s — a hidden 2*pi scale error, until now absorbed by tuned limits.
#pragma once

#include <array>
#include <cmath>
#include <numbers>

// A body-frame velocity command / estimate.
struct BodyTwist {
    double vx = 0.0;  // m/s, forward
    double vy = 0.0;  // m/s, strafe-left
    double w = 0.0;   // rad/s, CCW+

    BodyTwist() = default;
    BodyTwist(double vx_, double vy_, double w_) : vx(vx_), vy(vy_), w(w_) {}
};

// Geometry + polarity of one omni wheel.
struct Wheel {
    double x_m = 0.0;              // contact position, body frame, +x forward
    double y_m = 0.0;              // contact position, body frame, +y left
    double drive_angle_deg = 0.0;  // drive direction beta, body frame
    double sign = 1.0;             // motor polarity; flip if a wheel spins backward

    // Row [a_vx, a_vy, a_w] mapping body twist -> wheel surface speed (m/s).
    std::array<double, 3> row() const {
        const double b = drive_angle_deg * (std::numbers::pi / 180.0);
        const double sb = std::sin(b), cb = std::cos(b);
        return {sign * cb, sign * sb, sign * (x_m * sb - y_m * cb)};
    }
};

// Full drive geometry: the four wheels and the scale from wheel surface
// speed to motor revolutions. Plain data — construct with the defaults and
// override members from config.
struct Kinematics {
    static constexpr double kDefaultWheelRadiusM = 0.0335;

    // Wheel geometry, hardware-verified frame (drive test 2026-07-04 on the
    // Rust port: the legacy table was rotated 90 deg — commanded forward
    // drove the robot to its physical RIGHT; positions here are the legacy
    // mechanical table rotated -90: (x,y) = (y_legacy, -x_legacy)).
    //
    // Mount angles, degrees (documented; row order 1=FR, 2=RR, 3=RL, 4=FL):
    //   wheel 1: +30      wheel 2: -45
    //   wheel 3: -135     wheel 4: +150
    // Wheel 3 was -130 in the legacy table (Math/wheel_math.h) — a typo for
    // -135: positions are at +-45/+-135-ish bearings and every other angle
    // matches its bearing. The -130 made wheel 3's response non-orthogonal,
    // coupling vx/vy/omega so the robot curved when driving straight.
    std::array<Wheel, 4> wheels = {{
        {0.03687, -0.0636, 30.0, 1.0},
        {-0.05214, -0.05214, -45.0, 1.0},
        {-0.05214, 0.05214, -135.0, 1.0},
        {0.03687, 0.0636, 150.0, 1.0},
    }};

    // Meters of body travel per motor output revolution. Default assumes a
    // direct-drive wheel of kDefaultWheelRadiusM (= 2*pi*R). CALIBRATE this
    // on hardware: command a known motor rev count and measure travel.
    // Config override: Motor.yaml `metersPerMotorRev`.
    double meters_per_motor_rev = 2.0 * std::numbers::pi * kDefaultWheelRadiusM;
    // Identified chassis lateral effectiveness correction. Omni rollers and
    // load geometry can make strafe travel weaker than forward travel even
    // when every motor tracks its velocity setpoint. IK applies this gain to
    // the body-y demand; FK divides it back out so odometry stays in physical
    // m/s. Config override: Motor.yaml `bodyLateralScale`.
    double body_lateral_scale = 1.0;
    // Per-controller velocity tracking normalization, CAN order 1..4.
    // These scale only IK commands; FK always consumes the real measured
    // motor speeds, so odometry is never fabricated.
    std::array<double, 4> wheel_command_scale = {{1.0, 1.0, 1.0, 1.0}};
    // Small direction-specific residual multipliers. Moteus/friction
    // response is not perfectly symmetric through zero; keeping this
    // separate from the base factor permits sign calibration without
    // changing encoder odometry.
    std::array<double, 4> wheel_command_scale_positive =
        {{1.0, 1.0, 1.0, 1.0}};
    std::array<double, 4> wheel_command_scale_negative =
        {{1.0, 1.0, 1.0, 1.0}};

    // Inverse kinematics: body twist -> the four motor velocity setpoints
    // (output rev/s), ready for moteus PositionMode::Command::velocity.
    // Index i corresponds to motor id i+1.
    std::array<double, 4> inverse(const BodyTwist& t) const;

    // Forward kinematics: measured motor velocities (rev/s) -> best-fit body
    // twist, least squares over the over-determined 4x3 system (J^T J x =
    // J^T b via a 3x3 cofactor inverse). This is the wheel-odometry input to
    // the onboard motion executor and a telemetry cross-check.
    BodyTwist forward(const std::array<double, 4>& motor_rev_s) const;

    // Peak motor speed (rev/s magnitude) that inverse() would demand — useful
    // for pre-scaling a command into a motor speed budget without bending
    // its direction.
    double peak_motor_rev_s(const BodyTwist& t) const;
};
