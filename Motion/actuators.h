// Kicker + dribbler policy — turns the decoded 24-bit KickerDribbler field
// into firing decisions and ESC settings, TIGERs semantics (their
// skill_basics.c kicker handling) adapted to OUR hardware reality:
//
//   * ARM is continuous: the server re-sends ARM every frame and the robot
//     fires ON ITS OWN ball-contact signal. Our robots have no IR
//     break-beam (phoenix-rf never had one); the onboard ball camera is the
//     stand-in: a big, centered, confident, fresh detection = ball at the
//     mouth. That same signal is reported as the MatchFeedback barrier bit,
//     so the server's contact ladder (barrier -> traction -> vision) works.
//   * FORCE fires immediately (once per activation edge; a persistent
//     FORCE re-fires only after the capacitor recharge guard).
//   * ARM_TIME arms like ARM but with a raw discharge duration.
//   * DISARM clears. Kick with speed 0 never fires (oracle semantics).
//   * The dribbler is an ESC: speed maps to microseconds above the stop
//     pulse. The force field has no actuator on our hardware (documented,
//     ignored). Traction report: OFF / IDLE (spinning, no ball) / STRONG
//     (spinning + ball contact) — mirrors the server-side oracle
//     (fake_robot.py) exactly.
//
// All mappings that are hardware-true (kick speed -> pulse ms, dribble
// speed -> microseconds) are CONFIG and must be identified on the real
// robots (commissioning); the defaults preserve the historical behavior
// (~10 ms pulse for a match kick, 1600 us full dribble).
//
// Pure and deterministic: no serial, no clocks — the caller feeds time in.
#pragma once

#include <optional>

#include "../Networks/matchctrl.h"

namespace rf {

struct ActuatorConfig {
    // Kick speed (m/s) -> solenoid pulse (ms): pulse = ms_per_mps * speed +
    // bias, clamped. IDENTIFY on hardware.
    double kick_ms_per_mps = 1.5;
    double kick_ms_bias = 2.0;
    double kick_pulse_min_ms = 2.0;
    double kick_pulse_max_ms = 25.0;
    // Persistent FORCE re-fire guard, seconds. Deliberately LONGER than the
    // Arduino's own 5 s capacitor-recharge guard so a re-fire isn't
    // silently swallowed by the firmware timeout.
    double kicker_recharge_s = 5.2;
    // Dribbler bar speed (m/s) -> ESC microseconds above the 1500 us stop.
    // Historical full-on is 1600 us (= 100 above stop); IDENTIFY per robot.
    double dribble_us_per_mps = 20.0;
    // Ball-contact thresholds (camera stand-in for the IR barrier).
    double barrier_radius_px = 55.0;      // ball this big = at the mouth
    double barrier_bearing_rad = 0.35;    // and roughly centered
    double barrier_min_confidence = 0.3;  // and a real detection
    double barrier_max_age_s = 0.3;       // and fresh
};

// The freshest onboard ball observation, reduced to what the policy needs
// (filled from BallDetection's BallObservation by the caller).
struct BallContactObs {
    bool found = false;
    double bearing = 0.0;     // rad from the camera axis
    double radius = 0.0;      // px
    double confidence = 0.0;  // 0..1
    double age_s = 1e9;       // seconds since the observation
};

struct ActuatorOutput {
    // A kick to fire THIS tick: solenoid pulse in ms (edge-triggered).
    std::optional<double> fire_pulse_ms;
    bool armed = false;  // ARM latched, waiting for ball contact
    // Dribbler ESC microseconds above the 1500 us stop (0 = stop).
    int dribbler_us = 0;
    // Feedback signals (MatchFeedback).
    bool barrier = false;  // ball-contact signal this tick
    DribbleTraction traction = DribbleTraction::Off;
    double dribbler_speed = 0.0;  // commanded bar speed echo, m/s
};

class Actuators {
public:
    explicit Actuators(const ActuatorConfig& cfg = {}) : cfg_(cfg) {}

    ActuatorOutput tick(double now_s, const KickerDribbler& kd,
                        const BallContactObs& obs);

    // On emergency/estop: disarm and stop the dribbler. The recharge clock
    // is kept (hardware protection is not resettable by software state).
    void reset();

    bool ball_contact(const BallContactObs& obs) const;

private:
    ActuatorConfig cfg_;
    KickerMode prev_mode_ = KickerMode::Disarm;
    double last_fire_s_ = -1e9;
    bool barrier_prev_ = false;  // armed fire is edge-triggered on contact
};

}  // namespace rf
