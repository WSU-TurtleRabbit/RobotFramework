#include "actuators.h"

#include <algorithm>
#include <cmath>

namespace rf {

bool Actuators::ball_contact(const BallContactObs& obs) const {
    return obs.found && obs.age_s <= cfg_.barrier_max_age_s &&
           obs.confidence >= cfg_.barrier_min_confidence &&
           std::fabs(obs.bearing) <= cfg_.barrier_bearing_rad &&
           obs.radius >= cfg_.barrier_radius_px;
}

ActuatorOutput Actuators::tick(double now_s, const KickerDribbler& kd,
                               const BallContactObs& obs) {
    ActuatorOutput out;
    const bool contact = ball_contact(obs);
    out.barrier = contact;

    // --- kicker ---
    const bool armed = kd.kick_mode == KickerMode::Arm ||
                       kd.kick_mode == KickerMode::ArmTime;
    out.armed = armed;
    double pulse_ms = 0.0;
    if (kd.kick_mode == KickerMode::ArmTime) {
        pulse_ms = std::clamp(kd.kick_time_us * 1e-3, cfg_.kick_pulse_min_ms,
                              255.0 /* protocol byte */);
    } else {
        pulse_ms = std::clamp(cfg_.kick_ms_per_mps * kd.kick_speed + cfg_.kick_ms_bias,
                              cfg_.kick_pulse_min_ms, cfg_.kick_pulse_max_ms);
    }
    const bool has_energy = kd.kick_mode == KickerMode::ArmTime
                                ? kd.kick_time_us > 0.0
                                : kd.kick_speed > 0.0;

    bool fire = false;
    switch (kd.kick_mode) {
    case KickerMode::Force:
        // Immediately, once per activation edge (a persistent FORCE may
        // re-fire only after the capacitor has had its recharge time).
        fire = has_energy && (prev_mode_ != KickerMode::Force ||
                              now_s - last_fire_s_ >= cfg_.kicker_recharge_s);
        break;
    case KickerMode::Arm:
    case KickerMode::ArmTime:
        // On the robot's own ball contact, edge-triggered: the ball must
        // LEAVE the mouth before the next armed shot (no double-kicks while
        // it sits there).
        fire = has_energy && contact && !barrier_prev_;
        break;
    case KickerMode::Disarm:
    default:
        break;
    }
    if (fire) {
        out.fire_pulse_ms = pulse_ms;
        last_fire_s_ = now_s;
    }

    // --- dribbler (level-held) ---
    out.dribbler_speed = kd.dribbler_speed;
    out.dribbler_us = static_cast<int>(std::lround(
        std::clamp(cfg_.dribble_us_per_mps * kd.dribbler_speed, 0.0, 255.0)));
    // Traction report (the server's contact fallback after the barrier):
    // OFF when stopped, IDLE when spinning without the ball, STRONG when
    // spinning with contact — mirrors fake_robot.py exactly.
    out.traction = kd.dribbler_speed > 0.01
                       ? (contact ? DribbleTraction::Strong : DribbleTraction::Idle)
                       : DribbleTraction::Off;

    prev_mode_ = kd.kick_mode;
    barrier_prev_ = contact;
    return out;
}

void Actuators::reset() {
    prev_mode_ = KickerMode::Disarm;
    barrier_prev_ = false;
}

}  // namespace rf
