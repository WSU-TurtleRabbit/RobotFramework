// Strict command decode: a packet either decodes completely or is rejected
// as a whole — malformed input must never move the robot. Golden strings
// mirror the server's encoder tests.
#include <string>

#include "Motion/phx/testing.h"
#include "Networks/decode.h"

PHX_TEST(decode_golden_v1_command) {
    cmdDecoder d;
    CHECK(d.decode_cmd("1 0.26 0 -0.45 0 0 1782911401.487") == CmdType::Velocity);
    CHECK(d.id == 1);
    CHECK_NEAR(d.velocity_x, 0.26, 1e-9);
    CHECK_NEAR(d.velocity_y, 0.0, 1e-9);
    CHECK_NEAR(d.velocity_w, -0.45, 1e-9);
    CHECK(!d.kick);
    CHECK(!d.dribble);
    CHECK_NEAR(d.time, 1782911401.487, 1e-6);
}

PHX_TEST(decode_kick_dribble_are_bare_one) {
    cmdDecoder d;
    CHECK(d.decode_cmd("5 0 0 0 1 1 0") == CmdType::Velocity);
    CHECK(d.kick);
    CHECK(d.dribble);
}

PHX_TEST(decode_fields_initialized_before_first_packet) {
    // The old decoder's members were uninitialized — reading them before the
    // first packet was undefined behavior. Now they are well-defined zeros.
    cmdDecoder d;
    CHECK(d.id == -1);
    CHECK(d.velocity_x == 0.0);
    CHECK(d.velocity_y == 0.0);
    CHECK(d.velocity_w == 0.0);
    CHECK(!d.kick);
    CHECK(!d.dribble);
    CHECK(d.time == 0.0);
}

PHX_TEST(decode_short_packet_rejected_and_leaves_no_stale_motion) {
    cmdDecoder d;
    CHECK(d.decode_cmd("2 0.5 0.5 1.0 1 1 42.0") == CmdType::Velocity);
    CHECK_NEAR(d.velocity_x, 0.5, 1e-9);
    // Short packet: the whole thing is rejected — and the previous command's
    // motion fields must NOT survive (the legacy istringstream>> decoder
    // drove the robot on exactly this leftover state).
    CHECK(d.decode_cmd("2 0.9") == CmdType::Malformed);
    CHECK(d.velocity_x == 0.0);
    CHECK(d.velocity_y == 0.0);
    CHECK(d.velocity_w == 0.0);
    CHECK(!d.kick);
    CHECK(!d.dribble);
}

PHX_TEST(decode_exact_field_count_required) {
    cmdDecoder d;
    CHECK(d.decode_cmd("1 0 0 0 0 0") == CmdType::Malformed);           // 6 fields
    CHECK(d.decode_cmd("1 0.1 0.2 0.3 0 0 5 extra") == CmdType::Malformed);  // 8 fields
    CHECK(d.decode_cmd("") == CmdType::Malformed);
    CHECK(d.decode_cmd("   ") == CmdType::Malformed);
}

PHX_TEST(decode_nonfinite_and_garbage_rejected) {
    cmdDecoder d;
    CHECK(d.decode_cmd("1 nan 0 0 0 0 5") == CmdType::Malformed);
    CHECK(d.decode_cmd("1 0 inf 0 0 0 5") == CmdType::Malformed);
    CHECK(d.decode_cmd("1 0 0 -inf 0 0 5") == CmdType::Malformed);
    CHECK(d.decode_cmd("1 0.2x 0 0 0 0 5") == CmdType::Malformed);  // trailing junk
    CHECK(d.decode_cmd("x 0 0 0 0 0 5") == CmdType::Malformed);     // non-numeric id
    CHECK(d.decode_cmd("-1 0 0 0 0 0 5") == CmdType::Malformed);    // negative id
    CHECK(d.decode_cmd("garbage here") == CmdType::Malformed);
}

PHX_TEST(decode_opcodes_classified) {
    cmdDecoder d;
    CHECK(d.decode_cmd("STOP") == CmdType::Stop);
    CHECK(d.decode_cmd("  STOP\n") == CmdType::Stop);
    CHECK(d.decode_cmd("PING") == CmdType::Ping);
    CHECK(d.decode_cmd("CALIBRATE") == CmdType::Calibrate);
    CHECK(d.decode_cmd("REBOOT") == CmdType::Malformed);  // unknown opcode
}

PHX_TEST(decode_validates_robot_id) {
    cmdDecoder d;
    d.expected_id = 3;
    // Addressed to us: accepted.
    CHECK(d.decode_cmd("3 0.1 0 0 0 0 1.0") == CmdType::Velocity);
    CHECK_NEAR(d.velocity_x, 0.1, 1e-9);
    // Addressed to a teammate: valid frame, but motion must stay zero.
    CHECK(d.decode_cmd("4 0.9 0.9 0.9 1 1 1.0") == CmdType::WrongId);
    CHECK(d.velocity_x == 0.0);
    CHECK(d.velocity_y == 0.0);
    CHECK(d.velocity_w == 0.0);
    CHECK(!d.kick);
    CHECK(!d.dribble);
    // Default -1 accepts any id.
    d.expected_id = -1;
    CHECK(d.decode_cmd("7 0.1 0 0 0 0 1.0") == CmdType::Velocity);
}
