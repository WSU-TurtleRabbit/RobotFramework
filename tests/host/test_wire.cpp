// MV2 wire format. The golden string is PINNED across repos (phoenix-rf
// crates/protocol/src/wire.rs and phoenix-core robot wire tests hold the
// same bytes) — change all of them together or none.
#include <cmath>
#include <string>

#include "Motion/phx/testing.h"
#include "Motion/phx/wire.h"
#include "Networks/decode.h"

using namespace phx;

PHX_TEST(mv2_golden_string_parses) {
    // GOLDEN — byte-for-byte the string pinned in phoenix-rf's tests.
    const std::string s =
        "MV2 5 1234 MOVE ALIGN -1.2345 0.5 1.5708 0.25 -0.1 0.05 -1.2845 0.55 "
        "1.5708 0.35 1.5 1.2 8 0 0 1 18 1782911401.487";
    const auto m = parse_mv2(s);
    REQUIRE(m.has_value());
    CHECK(m->robot_id == 5);
    CHECK(m->seq == 1234);
    CHECK(m->kind == MoveKind::Move);
    CHECK(m->mode == MoveMode::PrecisionAlign);
    CHECK_NEAR(m->px, -1.2345, 1e-9);
    CHECK_NEAR(m->py, 0.5, 1e-9);
    CHECK_NEAR(m->ptheta, 1.5708, 1e-9);
    CHECK_NEAR(m->vx, 0.25, 1e-9);
    CHECK_NEAR(m->vy, -0.1, 1e-9);
    CHECK_NEAR(m->vw, 0.05, 1e-9);
    CHECK_NEAR(m->tx, -1.2845, 1e-9);
    CHECK_NEAR(m->ty, 0.55, 1e-9);
    CHECK_NEAR(m->ttheta, 1.5708, 1e-9);
    CHECK_NEAR(m->max_speed_mps, 0.35, 1e-9);
    CHECK_NEAR(m->max_w_radps, 1.5, 1e-9);
    CHECK_NEAR(m->max_accel_mps2, 1.2, 1e-9);
    CHECK_NEAR(m->max_jerk_mps3, 8.0, 1e-9);
    CHECK(m->arrive_speed_mps == 0.0);
    CHECK(!m->kick);
    CHECK(m->dribble);
    CHECK(m->pose_age_ms == 18);
    CHECK_NEAR(m->time_set, 1782911401.487, 1e-6);
}

PHX_TEST(mv2_all_kinds_and_modes_parse) {
    const struct { const char* w; MoveKind k; } kinds[] = {
        {"MOVE", MoveKind::Move},
        {"HOLD", MoveKind::Hold},
        {"BRAKE", MoveKind::Brake},
        {"DISABLE", MoveKind::Disable},
    };
    const struct { const char* w; MoveMode m; } modes[] = {
        {"FAST", MoveMode::FastTravel},
        {"BALL", MoveMode::BallApproach},
        {"ALIGN", MoveMode::PrecisionAlign},
        {"HOLD", MoveMode::HoldPosition},
        {"BRAKE", MoveMode::Brake},
    };
    for (const auto& k : kinds) {
        for (const auto& mo : modes) {
            const std::string s = std::string("MV2 1 9 ") + k.w + " " + mo.w +
                                  " 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5";
            const auto m = parse_mv2(s);
            REQUIRE(m.has_value());
            CHECK(m->kind == k.k);
            CHECK(m->mode == mo.m);
        }
    }
}

PHX_TEST(mv2_malformed_is_rejected_never_partial) {
    // Truncated.
    CHECK(!parse_mv2("MV2 5 1 MOVE FAST 0 0 0").has_value());
    // Unknown kind / mode words.
    CHECK(!parse_mv2("MV2 1 9 FLY FAST 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5").has_value());
    CHECK(!parse_mv2("MV2 1 9 MOVE WARP 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5").has_value());
    // Non-finite motion field.
    CHECK(!parse_mv2("MV2 1 9 MOVE FAST nan 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5").has_value());
    // Wrong prefix.
    CHECK(!parse_mv2("MV3 1 9 MOVE FAST 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5").has_value());
    // Negative ids/seq are not unsigned fields.
    CHECK(!parse_mv2("MV2 -1 9 MOVE FAST 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5").has_value());
    // Extra field.
    CHECK(!parse_mv2("MV2 1 9 MOVE FAST 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5 junk").has_value());
}

PHX_TEST(mv2_routes_as_move_never_v1_velocity) {
    // An MV2 frame must never decode as a legacy velocity command (23 fields
    // cannot pass the strict 7-field decoder, and classification routes the
    // raw payload to the bridge instead).
    cmdDecoder d;
    CHECK(d.decode_cmd("MV2 1 9 MOVE FAST 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5.5") == CmdType::Move);
    CHECK(d.velocity_x == 0.0);
    CHECK(d.velocity_y == 0.0);
    CHECK(d.velocity_w == 0.0);
    // A malformed MV2-prefixed line still classifies Move (bridge rejects it
    // on parse) — it must not reach the v1 decoder either.
    CHECK(d.decode_cmd("MV2 garbage") == CmdType::Move);
    CHECK(d.decode_cmd("MV2") == CmdType::Move);
}

PHX_TEST(wire_fmt_double_matches_rust_push_kv_f) {
    CHECK(fmt_wire_double(0.0) == "0");
    CHECK(fmt_wire_double(12.0) == "12");
    CHECK(fmt_wire_double(11.9) == "11.9");
    CHECK(fmt_wire_double(1.23456) == "1.2346");  // 4 decimals, rounded
    CHECK(fmt_wire_double(-0.00001) == "0");      // "-0" collapses to "0"
    CHECK(fmt_wire_double(std::nan("")) == "nan");
    CHECK(fmt_wire_double(HUGE_VAL) == "inf");
    CHECK(fmt_wire_double(-HUGE_VAL) == "-inf");
}
