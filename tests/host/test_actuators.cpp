// Kicker/dribbler policy tests: TIGERs ARM/FORCE/DISARM/ARM_TIME semantics
// on our hardware (camera ball-contact as the break-beam stand-in).
#include <cmath>

#include "Motion/actuators.h"
#include "Motion/phx/testing.h"

using namespace rf;

namespace {

KickerDribbler kd(KickerMode mode, double speed = 6.0, double drib = 0.0) {
    KickerDribbler k;
    k.kick_mode = mode;
    k.kick_speed = speed;
    k.dribbler_speed = drib;
    return k;
}

BallContactObs ball(double radius = 80.0, double bearing = 0.0, double age = 0.05,
                    double conf = 0.9) {
    BallContactObs o;
    o.found = true;
    o.bearing = bearing;
    o.radius = radius;
    o.confidence = conf;
    o.age_s = age;
    return o;
}

}  // namespace

PHX_TEST(actuators_disarm_never_fires_but_dribbles) {
    Actuators a;
    const ActuatorOutput out = a.tick(1.0, kd(KickerMode::Disarm, 6.0, 3.0), ball());
    CHECK(!out.fire_pulse_ms.has_value());
    CHECK(!out.armed);
    CHECK(out.dribbler_us > 0);
    CHECK(out.traction == DribbleTraction::Strong);  // dribbling + contact
}

PHX_TEST(actuators_force_fires_on_edge_and_respects_recharge) {
    Actuators a;
    // Rising edge: fires immediately.
    ActuatorOutput out = a.tick(1.0, kd(KickerMode::Force, 6.0), BallContactObs{});
    REQUIRE(out.fire_pulse_ms.has_value());
    CHECK_NEAR(*out.fire_pulse_ms, 1.5 * 6.0 + 2.0, 1e-9);
    // Held FORCE: no machine-gunning — next shot only after the recharge.
    out = a.tick(2.0, kd(KickerMode::Force, 6.0), BallContactObs{});
    CHECK(!out.fire_pulse_ms.has_value());
    out = a.tick(1.0 + 5.3, kd(KickerMode::Force, 6.0), BallContactObs{});
    CHECK(out.fire_pulse_ms.has_value());
    // FORCE with zero kick speed never fires (oracle semantics).
    Actuators b;
    out = b.tick(1.0, kd(KickerMode::Force, 0.0), BallContactObs{});
    CHECK(!out.fire_pulse_ms.has_value());
}

PHX_TEST(actuators_arm_waits_for_contact_and_edge_triggers) {
    Actuators a;
    // Armed, no ball: nothing.
    ActuatorOutput out = a.tick(1.0, kd(KickerMode::Arm, 6.0), BallContactObs{});
    CHECK(!out.fire_pulse_ms.has_value());
    CHECK(out.armed);
    // Ball arrives at the mouth: fire.
    out = a.tick(1.1, kd(KickerMode::Arm, 6.0), ball());
    REQUIRE(out.fire_pulse_ms.has_value());
    // Ball sits there (contact held): NO double-kick.
    out = a.tick(1.2, kd(KickerMode::Arm, 6.0), ball());
    CHECK(!out.fire_pulse_ms.has_value());
    // Ball leaves and returns: fires again (re-armed continuously).
    out = a.tick(1.3, kd(KickerMode::Arm, 6.0), BallContactObs{});
    out = a.tick(1.4, kd(KickerMode::Arm, 6.0), ball());
    CHECK(out.fire_pulse_ms.has_value());
}

PHX_TEST(actuators_arm_time_uses_raw_discharge_time) {
    Actuators a;
    KickerDribbler k;
    k.kick_mode = KickerMode::ArmTime;
    k.kick_time_us = 5000.0;  // 5 ms
    const ActuatorOutput out = a.tick(1.0, k, ball());
    REQUIRE(out.fire_pulse_ms.has_value());
    CHECK_NEAR(*out.fire_pulse_ms, 5.0, 1e-9);
}

PHX_TEST(actuators_pulse_mapping_clamps) {
    Actuators a;
    // Tiny kick speed still gets the minimum pulse.
    ActuatorOutput out = a.tick(1.0, kd(KickerMode::Force, 0.05), BallContactObs{});
    REQUIRE(out.fire_pulse_ms.has_value());
    CHECK_NEAR(*out.fire_pulse_ms, 2.0 + 1.5 * 0.05, 1e-9);
    // Huge kick speed clamps at the max pulse.
    Actuators b;
    out = b.tick(1.0, kd(KickerMode::Force, 60.0), BallContactObs{});
    REQUIRE(out.fire_pulse_ms.has_value());
    CHECK_NEAR(*out.fire_pulse_ms, 25.0, 1e-9);
}

PHX_TEST(actuators_barrier_thresholds) {
    Actuators a;
    CHECK(a.ball_contact(ball()));
    CHECK(!a.ball_contact(ball(30.0)));                 // too far (small)
    CHECK(!a.ball_contact(ball(80.0, 0.6)));            // off-axis
    CHECK(!a.ball_contact(ball(80.0, 0.0, 0.5)));       // stale
    CHECK(!a.ball_contact(ball(80.0, 0.0, 0.05, 0.1))); // unsure
    BallContactObs none;
    CHECK(!a.ball_contact(none));                       // not found
}

PHX_TEST(actuators_dribbler_mapping_and_traction_ladder) {
    Actuators a;
    // Off: nothing, traction OFF.
    ActuatorOutput out = a.tick(1.0, kd(KickerMode::Disarm, 0.0, 0.0), ball());
    CHECK(out.dribbler_us == 0);
    CHECK(out.traction == DribbleTraction::Off);
    // Spinning without contact: IDLE.
    out = a.tick(1.0, kd(KickerMode::Disarm, 0.0, 4.0), BallContactObs{});
    CHECK(out.dribbler_us == 80);  // 20 us per m/s
    CHECK(out.traction == DribbleTraction::Idle);
    // Spinning with contact: STRONG.
    out = a.tick(1.0, kd(KickerMode::Disarm, 0.0, 4.0), ball());
    CHECK(out.traction == DribbleTraction::Strong);
    // Saturation at the protocol byte.
    out = a.tick(1.0, kd(KickerMode::Disarm, 0.0, 7.875), BallContactObs{});
    CHECK(out.dribbler_us <= 255);
}

PHX_TEST(actuators_reset_disarms) {
    Actuators a;
    a.tick(1.0, kd(KickerMode::Arm, 6.0), ball());  // fires
    a.reset();
    // After reset the previous contact is forgotten: a held contact is a
    // NEW edge — but the same held FORCE must edge again too.
    const ActuatorOutput out = a.tick(1.1, kd(KickerMode::Force, 6.0), BallContactObs{});
    CHECK(out.fire_pulse_ms.has_value());
}
