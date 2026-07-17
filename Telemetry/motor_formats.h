// The moteus query/command formats this robot uses — factored into a pure
// header (only mjbots/moteus/moteus_protocol.h, no pi3hat) so the host test
// suite can verify them without Pi hardware.
#pragma once

#include "moteus_protocol.h"

namespace rf {

// What we ask every motor to report each cycle.
//
// q_current was previously left at the library default (kIgnore,
// mjbots/moteus/moteus_protocol.h:341), so Query::Parse returned NaN and the
// main loop's overcurrent trip compared `NaN > limit` — which is always
// false: the protection never fired. Requesting it as a float makes the trip
// real (and telemetry m{id}_cur meaningful).
inline mjbots::moteus::Query::Format robot_query_format() {
    mjbots::moteus::Query::Format f;
    f.q_current = mjbots::moteus::kFloat;
    return f;
}

// How we shape every position (velocity) command.
//
// watchdog_timeout was previously unset (kIgnore): moteus fell back to its
// board-configured default, so if the Pi process hung, the wheels kept
// spinning the last command until that default expired. We now arm the
// per-command watchdog explicitly — a hardware-level failsafe independent of
// any software path: motors stop themselves `watchdog_timeout` seconds after
// the last command frame.
//
// velocity_limit / accel_limit let moteus itself bound each setpoint change
// (rev/s and rev/s^2): controller-level smoothing between our 100 Hz updates
// and a runaway backstop. NaN command values leave them at the controller
// default, so enabling the registers is behavior-neutral until configured.
inline mjbots::moteus::PositionMode::Format robot_position_format() {
    mjbots::moteus::PositionMode::Format f;
    f.watchdog_timeout = mjbots::moteus::kFloat;
    f.velocity_limit = mjbots::moteus::kFloat;
    f.accel_limit = mjbots::moteus::kFloat;
    return f;
}

}  // namespace rf
