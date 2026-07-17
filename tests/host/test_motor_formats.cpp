// The moteus formats the robot configures: q_current must actually be
// requested (the stock default ignores it, so the overcurrent gate compared
// NaN > limit and never fired) and every position command must arm the
// hardware watchdog. moteus_protocol.h is hardware-free, so these run on the
// host.
#include <cmath>
#include <cstdint>

#include "Motion/phx/testing.h"
#include "Telemetry/motor_formats.h"

using namespace mjbots;

namespace {

// Bytes a Query request frame occupies for a given format.
uint8_t query_frame_size(const moteus::Query::Format& f) {
    uint8_t data[64] = {};
    uint8_t size = 0;
    moteus::WriteCanData frame(data, &size);
    moteus::Query::Make(&frame, f);
    return size;
}

// Expected reply payload size for a given query format.
uint8_t query_reply_size(const moteus::Query::Format& f) {
    uint8_t data[64] = {};
    uint8_t size = 0;
    moteus::WriteCanData frame(data, &size);
    return moteus::Query::Make(&frame, f);
}

uint8_t position_frame_size(const moteus::PositionMode::Format& f, double watchdog_s) {
    uint8_t data[64] = {};
    uint8_t size = 0;
    moteus::WriteCanData frame(data, &size);
    moteus::PositionMode::Command cmd;
    cmd.position = std::nan("");
    cmd.velocity = 1.0;
    cmd.watchdog_timeout = watchdog_s;
    moteus::PositionMode::Make(&frame, cmd, f);
    return size;
}

}  // namespace

PHX_TEST(motor_query_format_requests_q_current) {
    // Regression: the library default leaves q_current at kIgnore
    // (moteus_protocol.h:341) -> Parse returns NaN -> `NaN > limit` never
    // trips. Our format must request it.
    const moteus::Query::Format def;
    CHECK(def.q_current == moteus::kIgnore);  // documents the default we fix

    const moteus::Query::Format ours = rf::robot_query_format();
    CHECK(ours.q_current == moteus::kFloat);

    // And it changes the wire: the request grows and the expected reply
    // carries at least 4 more bytes (one float register).
    CHECK(query_frame_size(ours) > query_frame_size(def));
    CHECK(query_reply_size(ours) >= query_reply_size(def) + 4);
}

PHX_TEST(motor_position_format_arms_watchdog) {
    const moteus::PositionMode::Format def;
    CHECK(def.watchdog_timeout == moteus::kIgnore);  // stock: watchdog never sent

    const moteus::PositionMode::Format ours = rf::robot_position_format();
    CHECK(ours.watchdog_timeout == moteus::kFloat);

    // The command frame actually carries the watchdog register write.
    CHECK(position_frame_size(ours, 0.1) >= position_frame_size(def, 0.1) + 4);
}

PHX_TEST(motor_parse_reads_q_current_from_reply) {
    // A synthetic reply containing MODE (int8) and Q CURRENT (float): with
    // the register actually reported, Parse must fill q_current — the value
    // the overcurrent trip compares.
    uint8_t data[64] = {};
    uint8_t size = 0;
    moteus::WriteCanData frame(data, &size);

    frame.Write<int8_t>(moteus::Multiplex::kReplyInt8 | 0x01);
    frame.WriteVaruint(moteus::Register::kMode);
    frame.Write<int8_t>(moteus::Mode::kPosition);

    frame.Write<int8_t>(moteus::Multiplex::kReplyFloat | 0x01);
    frame.WriteVaruint(moteus::Register::kQCurrent);
    frame.Write<float>(7.25f);

    const auto parsed = moteus::Query::Parse(data, size);
    CHECK(parsed.mode == moteus::Mode::kPosition);
    CHECK_NEAR(parsed.q_current, 7.25, 1e-6);
}
