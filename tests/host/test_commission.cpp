// The pure half of `debugger commission`. These run on the host.
//
// What they defend: the ceiling is derived (never invented) and the weakest
// readable backstop wins; no setting can pass its cap and hitting the cap is
// reported; the wheel clamp preserves direction; every non-finite value in
// the NDJSON becomes null (finding B-04); a profile survives a round trip
// and a swapped controller is detected; the identity encodings match
// moteus_tool's own.
//
// And for the operator-settable ceiling: a value above the derived one is
// detected with its ratio (the warn-and-confirm condition), a value the
// operator sets is honoured exactly with NO upper bound (500 stays 500),
// lowering a ceiling pulls the setting down onto it, and the override state
// survives into every cycle record and into the signed profile as
// approved_under_override.
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "Motion/phx/testing.h"
#include "tools/debugger/commission_core.h"

using namespace rf::dbg::commission;

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool contains(const std::string& hay, const std::string& needle) {
    return hay.find(needle) != std::string::npos;
}

BackstopReading live(int id, double v, double a, double maxv = kNan) {
    BackstopReading b;
    b.id = id;
    b.reachable = true;
    b.default_velocity_limit_rev_s = v;
    b.default_accel_limit_rev_s2 = a;
    b.max_velocity_rev_s = maxv;
    return b;
}

CeilingInputs four_live(double v, double a) {
    CeilingInputs in;
    for (int id = 1; id <= 4; ++id) in.controllers.push_back(live(id, v, a));
    in.yaml_velocity_limit_rev_s = 15.0;
    in.yaml_accel_limit_rev_s2 = 20.0;
    return in;
}

Caps caps(double bv, double bw, double vl, double al) {
    Caps c;
    c.body_vel_mps = bv;
    c.body_omega_radps = bw;
    c.velocity_limit_rev_s = vl;
    c.accel_limit_rev_s2 = al;
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// Ceiling derivation
// ---------------------------------------------------------------------------

PHX_TEST(commission_ceiling_is_80pct_of_live_backstop) {
    const Ceiling c = derive_ceiling(four_live(10.0, 20.0));
    CHECK(c.ok);
    CHECK_NEAR(c.velocity_limit_rev_s, 8.0, 1e-12);
    CHECK_NEAR(c.accel_limit_rev_s2, 16.0, 1e-12);
    CHECK(contains(c.velocity_source, "servo.default_velocity_limit"));
    CHECK(contains(c.accel_source, "servo.default_accel_limit"));
    CHECK(!c.derivation.empty());
    // The working must show the arithmetic, not just the answer.
    bool shows = false;
    for (const auto& line : c.derivation) {
        if (contains(line, "0.80 x 10.00") && contains(line, "8.000")) shows = true;
    }
    CHECK(shows);
}

PHX_TEST(commission_ceiling_weakest_controller_governs) {
    CeilingInputs in = four_live(12.0, 30.0);
    in.controllers[2].default_velocity_limit_rev_s = 9.0;   // controller 3 is the weakest
    in.controllers[1].default_accel_limit_rev_s2 = 18.0;    // controller 2 is the weakest
    const Ceiling c = derive_ceiling(in);
    CHECK(c.ok);
    CHECK_NEAR(c.velocity_limit_rev_s, 0.8 * 9.0, 1e-12);
    CHECK_NEAR(c.accel_limit_rev_s2, 0.8 * 18.0, 1e-12);
    CHECK(contains(c.velocity_source, "controller 3"));
    CHECK(contains(c.accel_source, "controller 2"));
}

PHX_TEST(commission_ceiling_yaml_stands_in_per_controller) {
    // One controller has no limit of its own (nan); Motor.yaml stands in for
    // it. With yaml 15 and the others at 20, yaml is the binding candidate.
    CeilingInputs in = four_live(20.0, 40.0);
    in.controllers[3].default_velocity_limit_rev_s = kNan;
    in.controllers[3].default_accel_limit_rev_s2 = kNan;
    const Ceiling c = derive_ceiling(in);
    CHECK(c.ok);
    CHECK_NEAR(c.velocity_limit_rev_s, 0.8 * 15.0, 1e-12);
    CHECK_NEAR(c.accel_limit_rev_s2, 0.8 * 20.0, 1e-12);
    CHECK(contains(c.velocity_source, "Motor.yaml velocityLimit"));
    CHECK(contains(c.velocity_source, "controller 4"));
    CHECK(contains(c.accel_source, "Motor.yaml accelLimit"));
    bool said_nan = false;
    for (const auto& line : c.derivation) {
        if (contains(line, "controller 4") && contains(line, "nan")) said_nan = true;
    }
    CHECK(said_nan);
}

PHX_TEST(commission_ceiling_all_yaml_when_no_controller_has_a_limit) {
    CeilingInputs in = four_live(kNan, kNan);
    const Ceiling c = derive_ceiling(in);
    CHECK(c.ok);
    CHECK_NEAR(c.velocity_limit_rev_s, 12.0, 1e-12);
    CHECK_NEAR(c.accel_limit_rev_s2, 16.0, 1e-12);
    CHECK(contains(c.velocity_source, "Motor.yaml"));
}

PHX_TEST(commission_ceiling_unreachable_controller_uses_yaml) {
    CeilingInputs in = four_live(10.0, 20.0);
    in.controllers[0] = BackstopReading{};  // never answered
    in.controllers[0].id = 1;
    in.yaml_velocity_limit_rev_s = 6.0;  // lower than the live 10
    const Ceiling c = derive_ceiling(in);
    CHECK(c.ok);
    CHECK_NEAR(c.velocity_limit_rev_s, 0.8 * 6.0, 1e-12);
    bool said = false;
    for (const auto& line : c.derivation) {
        if (contains(line, "controller 1") && contains(line, "did not answer")) said = true;
    }
    CHECK(said);
}

PHX_TEST(commission_ceiling_refuses_without_any_backstop) {
    CeilingInputs in = four_live(kNan, kNan);
    in.yaml_velocity_limit_rev_s = kNan;
    Ceiling c = derive_ceiling(in);
    CHECK(!c.ok);
    CHECK(std::isnan(c.velocity_limit_rev_s));
    CHECK(contains(c.error, "velocity"));

    in = four_live(kNan, kNan);
    in.yaml_accel_limit_rev_s2 = kNan;
    c = derive_ceiling(in);
    CHECK(!c.ok);
    CHECK(contains(c.error, "accel"));

    // Non-positive and infinite candidates are not backstops.
    in = four_live(0.0, -5.0);
    in.yaml_velocity_limit_rev_s = kInf;
    in.yaml_accel_limit_rev_s2 = 0.0;
    c = derive_ceiling(in);
    CHECK(!c.ok);
}

PHX_TEST(commission_ceiling_max_velocity_is_a_candidate) {
    CeilingInputs in = four_live(50.0, 20.0);
    in.controllers[1].max_velocity_rev_s = 30.0;  // derate onset below the default limit
    const Ceiling c = derive_ceiling(in);
    CHECK(c.ok);
    CHECK_NEAR(c.velocity_limit_rev_s, 0.8 * 30.0, 1e-12);
    CHECK(contains(c.velocity_source, "servo.max_velocity"));
    // The factory default 500 never binds against a real limit.
    CeilingInputs in2 = four_live(10.0, 20.0);
    for (auto& b : in2.controllers) b.max_velocity_rev_s = 500.0;
    CHECK_NEAR(derive_ceiling(in2).velocity_limit_rev_s, 8.0, 1e-12);
}

PHX_TEST(commission_ceiling_fraction_must_be_sane) {
    CeilingInputs in = four_live(10.0, 20.0);
    in.fraction = 1.5;
    CHECK(!derive_ceiling(in).ok);
    in.fraction = 0.0;
    CHECK(!derive_ceiling(in).ok);
    in.fraction = 0.5;
    CHECK_NEAR(derive_ceiling(in).velocity_limit_rev_s, 5.0, 1e-12);
}

PHX_TEST(commission_body_caps_keep_every_direction_inside_the_ceiling) {
    Kinematics kin;  // default geometry, unit scales
    kin.meters_per_motor_rev = 0.2171;
    const double ceiling = 8.0;
    const BodyCaps bc = derive_body_caps(kin, ceiling, 100.0, 100.0);
    CHECK(std::isfinite(bc.linear_mps) && bc.linear_mps > 0.0);
    CHECK(std::isfinite(bc.angular_radps) && bc.angular_radps > 0.0);
    // Sweep directions at the cap: peak wheel demand never exceeds the ceiling.
    for (int i = 0; i < 360; i += 5) {
        const double a = i * 3.14159265358979 / 180.0;
        const BodyTwist t{bc.linear_mps * std::cos(a), bc.linear_mps * std::sin(a), 0.0};
        CHECK(kin.peak_motor_rev_s(t) <= ceiling + 1e-9);
    }
    CHECK(kin.peak_motor_rev_s(BodyTwist{0, 0, bc.angular_radps}) <= ceiling + 1e-9);
    CHECK(kin.peak_motor_rev_s(BodyTwist{0, 0, -bc.angular_radps}) <= ceiling + 1e-9);
    // And the cap is tight: the worst direction actually reaches the ceiling.
    CHECK_NEAR(bc.worst_linear_gain_rev_s_per_mps * bc.linear_mps, ceiling, 1e-6);
    CHECK(contains(bc.linear_source, "wheel ceiling"));
}

PHX_TEST(commission_body_caps_respect_the_configured_envelope) {
    Kinematics kin;
    kin.meters_per_motor_rev = 0.2171;
    const BodyCaps bc = derive_body_caps(kin, 8.0, 1.0, 2.5);
    CHECK_NEAR(bc.linear_mps, 1.0, 1e-12);
    CHECK_NEAR(bc.angular_radps, 2.5, 1e-12);
    CHECK(contains(bc.linear_source, "cappedLinear"));
    CHECK(contains(bc.angular_source, "cappedAngular"));
    // No ceiling -> no cap.
    const BodyCaps none = derive_body_caps(kin, kNan, 1.0, 2.5);
    CHECK(std::isnan(none.linear_mps));
}

// ---------------------------------------------------------------------------
// Settings and clamps
// ---------------------------------------------------------------------------

PHX_TEST(commission_step_never_passes_the_cap_and_says_so) {
    Settings s;
    s.velocity_limit_rev_s = 7.5;
    const Caps c = caps(2.0, 3.0, 8.0, 16.0);
    StepResult r = step_setting(s, Setting::VelocityLimit, +1, c);
    CHECK(r.changed);
    CHECK_NEAR(s.velocity_limit_rev_s, 8.0, 1e-12);
    CHECK(!r.at_ceiling);  // landed exactly on the cap by a full step
    r = step_setting(s, Setting::VelocityLimit, +1, c);
    CHECK(!r.changed);
    CHECK(r.at_ceiling);
    CHECK_NEAR(s.velocity_limit_rev_s, 8.0, 1e-12);
    // A step that would cross the cap lands ON it and reports the ceiling
    // (7.9 + 0.5 snaps to 8.5, which is over the 8.0 cap).
    s.velocity_limit_rev_s = 7.9;
    r = step_setting(s, Setting::VelocityLimit, +1, c);
    CHECK(r.changed);
    CHECK(r.at_ceiling);
    CHECK_NEAR(s.velocity_limit_rev_s, 8.0, 1e-12);
    // Many presses in a row can never exceed the cap.
    for (int i = 0; i < 1000; ++i) step_setting(s, Setting::VelocityLimit, +1, c);
    CHECK(s.velocity_limit_rev_s <= 8.0 + 1e-12);
    for (int i = 0; i < 1000; ++i) step_setting(s, Setting::AccelLimit, +1, c);
    CHECK(s.accel_limit_rev_s2 <= 16.0 + 1e-12);
    for (int i = 0; i < 1000; ++i) step_setting(s, Setting::BodyVel, +1, c);
    CHECK(s.body_vel_mps <= 2.0 + 1e-12);
    for (int i = 0; i < 1000; ++i) step_setting(s, Setting::BodyOmega, +1, c);
    CHECK(s.body_omega_radps <= 3.0 + 1e-12);
}

PHX_TEST(commission_step_has_a_floor_and_snaps_to_the_grid) {
    Settings s;
    s.body_vel_mps = 0.3;
    const Caps c = caps(2.0, 3.0, 8.0, 16.0);
    for (int i = 0; i < 10; ++i) step_setting(s, Setting::BodyVel, -1, c);
    CHECK_NEAR(s.body_vel_mps, setting_floor(Setting::BodyVel), 1e-12);
    const StepResult r = step_setting(s, Setting::BodyVel, -1, c);
    CHECK(r.at_floor);
    CHECK(!r.changed);
    // 0.1 + 0.1 + 0.1 must be exactly the grid value, not 0.30000000000000004.
    s.body_vel_mps = 0.0;
    for (int i = 0; i < 3; ++i) step_setting(s, Setting::BodyVel, +1, c);
    CHECK(s.body_vel_mps == 0.3);
}

PHX_TEST(commission_step_refuses_when_there_is_no_cap) {
    Settings s;
    s.velocity_limit_rev_s = 4.0;
    Caps c = caps(2.0, 3.0, kNan, 16.0);
    const StepResult r = step_setting(s, Setting::VelocityLimit, +1, c);
    CHECK(!r.changed);
    CHECK(r.at_ceiling);
    CHECK_NEAR(s.velocity_limit_rev_s, 4.0, 1e-12);
    c.velocity_limit_rev_s = 0.0;
    CHECK(!step_setting(s, Setting::VelocityLimit, +1, c).changed);
}

PHX_TEST(commission_clamp_settings_pulls_a_high_start_down) {
    Settings s;
    s.body_vel_mps = 5.0;
    s.body_omega_radps = 50.0;
    s.velocity_limit_rev_s = 15.0;  // Motor.yaml's value, above the 8.0 ceiling
    s.accel_limit_rev_s2 = 20.0;
    clamp_settings(s, caps(1.5, 6.0, 8.0, 16.0));
    CHECK_NEAR(s.body_vel_mps, 1.5, 1e-12);
    CHECK_NEAR(s.body_omega_radps, 6.0, 1e-12);
    CHECK_NEAR(s.velocity_limit_rev_s, 8.0, 1e-12);
    CHECK_NEAR(s.accel_limit_rev_s2, 16.0, 1e-12);
    Settings z;  // zeros come up to the floor, nan to something finite
    z.body_vel_mps = kNan;
    clamp_settings(z, caps(1.5, 6.0, 8.0, 16.0));
    CHECK(std::isfinite(z.body_vel_mps));
    CHECK(z.velocity_limit_rev_s >= setting_floor(Setting::VelocityLimit) - 1e-12);
}

PHX_TEST(commission_wheel_clamp_preserves_direction) {
    const std::array<double, 4> d{{4.0, -2.0, 1.0, -8.0}};
    const WheelClamp c = clamp_wheels(d, 4.0);
    CHECK(c.clipped);
    CHECK_NEAR(c.factor, 0.5, 1e-12);
    CHECK_NEAR(c.peak_before, 8.0, 1e-12);
    CHECK_NEAR(c.wheels[0], 2.0, 1e-12);
    CHECK_NEAR(c.wheels[1], -1.0, 1e-12);
    CHECK_NEAR(c.wheels[2], 0.5, 1e-12);
    CHECK_NEAR(c.wheels[3], -4.0, 1e-12);
    // Inside the cap: untouched.
    const WheelClamp u = clamp_wheels({{1.0, -1.0, 0.5, 0.0}}, 4.0);
    CHECK(!u.clipped);
    CHECK_NEAR(u.factor, 1.0, 1e-12);
    CHECK_NEAR(u.wheels[0], 1.0, 1e-12);
    // No cap -> nothing is commanded.
    const WheelClamp n = clamp_wheels(d, kNan);
    CHECK(n.clipped);
    for (double w : n.wheels) CHECK(w == 0.0);
    const WheelClamp z = clamp_wheels(d, 0.0);
    for (double w : z.wheels) CHECK(w == 0.0);
}

// ---------------------------------------------------------------------------
// NDJSON
// ---------------------------------------------------------------------------

PHX_TEST(commission_json_nonfinite_is_null_never_zero) {
    CHECK(json_number(kNan) == "null");
    CHECK(json_number(kInf) == "null");
    CHECK(json_number(-kInf) == "null");
    CHECK(json_number(0.0) == "0");
    CHECK(json_number(1.5) == "1.5");
    CHECK(json_number(-0.25) == "-0.25");
    CHECK(!contains(json_number(1e300), "inf"));
}

PHX_TEST(commission_json_strings_are_escaped) {
    CHECK(json_string("plain") == "\"plain\"");
    CHECK(json_string("say \"hi\"") == "\"say \\\"hi\\\"\"");
    CHECK(json_string("back\\slash") == "\"back\\\\slash\"");
    CHECK(json_string("line\nbreak\ttab") == "\"line\\nbreak\\ttab\"");
    CHECK(json_string(std::string("nul\x01") ) == "\"nul\\u0001\"");
    // UTF-8 passes through untouched.
    CHECK(json_string("\xc3\xa9") == "\"\xc3\xa9\"");
}

PHX_TEST(commission_cycle_record_missing_wheel_is_null_and_parses) {
    CycleRecord r;
    r.seq = 77;
    r.t_mono_s = 1.25;
    r.t_wall_unix_s = 1788000000.5;
    r.dt_s = kNan;  // first cycle has no dt
    r.twist_requested = BodyTwist{0.3, 0.0, kNan};
    r.energized = true;
    std::snprintf(r.state, sizeof r.state, "%s", "drive");
    std::snprintf(r.clip, sizeof r.clip, "%s", "velocity_limit");
    for (int i = 0; i < 4; ++i) r.wheels[i].id = i + 1;
    r.wheels[0].replied = true;
    r.wheels[0].cmd_rev_s = 1.5;
    r.wheels[0].velocity_rev_s = 1.48;
    r.wheels[0].q_current_a = 0.7;
    r.wheels[0].mode = 10;
    r.wheels[0].fault = 0;
    r.wheels[0].temperature_c = kNan;  // replied but this field was absent
    // wheel 2 did not reply at all; its numbers must NOT leak even if set
    r.wheels[1].replied = false;
    r.wheels[1].velocity_rev_s = 0.0;
    r.wheels[1].cmd_rev_s = 1.5;
    r.imu.present = false;
    r.imu.yaw_dps = 12.0;  // stale value; must not be emitted when !present

    const std::string line = encode_cycle(r);
    CHECK(!contains(line, "\n"));
    CHECK(!contains(line, "nan"));
    CHECK(!contains(line, "inf"));
    CHECK(contains(line, "\"seq\":77"));
    CHECK(contains(line, "\"dt_s\":null"));
    CHECK(contains(line, "\"twist_requested\":[0.3,0,null]"));
    CHECK(contains(line, "\"t_wall\":\"2026-08-29T"));  // 1788000000 is 2026-08-29 UTC

    JsonValue v;
    std::string err;
    REQUIRE(parse_json(line, &v, &err));
    const JsonValue* wheels = v.get("wheels");
    REQUIRE(wheels != nullptr && wheels->type == JsonValue::Array && wheels->arr.size() == 4);
    const JsonValue& w1 = wheels->arr[0];
    CHECK(w1.get("replied")->bool_or(false));
    CHECK_NEAR(w1.get("vel_rev_s")->number_or(-1), 1.48, 1e-9);
    CHECK(w1.get("temp_c")->type == JsonValue::Null);
    CHECK_NEAR(w1.get("mode")->number_or(-1), 10, 1e-9);
    const JsonValue& w2 = wheels->arr[1];
    CHECK(!w2.get("replied")->bool_or(true));
    CHECK(w2.get("vel_rev_s")->type == JsonValue::Null);   // not 0
    CHECK(w2.get("q_current_a")->type == JsonValue::Null);
    CHECK(w2.get("mode")->type == JsonValue::Null);
    CHECK(w2.get("fault")->type == JsonValue::Null);
    CHECK_NEAR(w2.get("cmd_rev_s")->number_or(-1), 1.5, 1e-9);  // what we SENT is known
    const JsonValue* imu = v.get("imu");
    REQUIRE(imu != nullptr);
    CHECK(!imu->get("present")->bool_or(true));
    CHECK(imu->get("rate_dps")->arr[2].type == JsonValue::Null);
    CHECK(v.get("clip")->string_or("") == "velocity_limit");
    CHECK(v.get("settings") != nullptr && v.get("caps") != nullptr);
}

PHX_TEST(commission_event_record_carries_fields_and_parses) {
    const std::string line = encode_event(
        12, 3.5, 1788000000.0, "limit_change",
        {{"setting", json_string("velocity_limit_rev_s")},
         {"from", json_number(4.0)},
         {"to", json_number(4.5)},
         {"cap", json_number(kNan)},
         {"note", json_string("quote \" here")}});
    CHECK(!contains(line, "\n"));
    JsonValue v;
    std::string err;
    REQUIRE(parse_json(line, &v, &err));
    CHECK(v.get("type")->string_or("") == "event");
    CHECK(v.get("event")->string_or("") == "limit_change");
    CHECK_NEAR(v.get("seq")->number_or(0), 12, 1e-9);
    CHECK_NEAR(v.get("to")->number_or(0), 4.5, 1e-9);
    CHECK(v.get("cap")->type == JsonValue::Null);
    CHECK(v.get("note")->string_or("") == "quote \" here");
    CHECK(contains(v.get("t_wall")->string_or(""), "Z"));
}

PHX_TEST(commission_iso8601_utc) {
    CHECK(iso8601_utc(0.0) == "1970-01-01T00:00:00.000Z");
    CHECK(iso8601_utc(1788000000.5) == "2026-08-29T10:40:00.500Z");
    CHECK(iso8601_utc(kNan).empty());
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

PHX_TEST(commission_serial_matches_moteus_tool) {
    // Ground truth: real controllers on this robot, decoded from the
    // moteus-cal-<serial>-<date>.log filenames moteus_tool writes.
    CHECK(serial_base64(0x310032, 0x4b305019, 0x20383859) == "ADEAMkswUBkgODhZ");
    CHECK(serial_base64(0x39002b, 0x4b305019, 0x20383859) == "ADkAK0swUBkgODhZ");
    CHECK(serial_base64(0x3c0035, 0x54305014, 0x2039334e) == "ADwANVQwUBQgOTNO");
    CHECK(serial_base64(0x500028, 0x484e500e, 0x20343356) == "AFAAKEhOUA4gNDNW");
    // High bits: the words are unsigned.
    CHECK(serial_base64(0xFFFFFFFFu, 0x80000001u, 0x0u) == "/////4AAAAEAAAAA");
    CHECK(serial_base64(0, 0, 0) == "AAAAAAAAAAAAAAAA");
    CHECK(serial_base64(0, 0, 1) == "AAAAAAAAAAAAAAAB");
    CHECK(serial_base64(1, 2, 3).size() == 16);
}

PHX_TEST(commission_cpuinfo_serial) {
    const std::string text =
        "processor\t: 0\nmodel name\t: ARMv7\n\nHardware\t: BCM2835\n"
        "Revision\t: c03114\nSerial\t\t: 100000007eea876b\nModel\t\t: Raspberry Pi 4\n";
    CHECK(parse_cpuinfo_serial(text) == "100000007eea876b");
    CHECK(parse_cpuinfo_serial("processor: 0\n").empty());
    CHECK(parse_cpuinfo_serial("").empty());
}

PHX_TEST(commission_conf_get_parsing) {
    DiagValue v = parse_conf_get_double("10.000000\r\n");
    CHECK(v.answered);
    CHECK_NEAR(v.value, 10.0, 1e-12);
    v = parse_conf_get_double("nan");
    CHECK(v.answered);       // the controller answered: it has no limit
    CHECK(std::isnan(v.value));
    v = parse_conf_get_double("ERR unknown group");
    CHECK(!v.answered);
    CHECK(std::isnan(v.value));
    v = parse_conf_get_double("");
    CHECK(!v.answered);
    v = parse_conf_get_double("OK");
    CHECK(!v.answered);
}

PHX_TEST(commission_diag_text_parsing) {
    const std::string dump =
        "git.hash.0 101\r\ngit.hash.1 0\r\ngit.hash.2 255\r\ngit.dirty 1\r\n"
        "git.timestamp 1723800000\r\n\r\nOK\r\n";
    const auto kv = parse_diag_text(dump);
    CHECK(kv.size() == 5);
    CHECK(kv.at("git.dirty") == "1");
    CHECK(git_hash_from_diag(kv) == "6500ff");
    const auto one = parse_diag_text("git.hash 657B5C5D1F\n");
    CHECK(git_hash_from_diag(one) == "657b5c5d1f");
    CHECK(git_hash_from_diag(parse_diag_text("firmware.version 264\n")).empty());

    const auto fw = parse_diag_text(
        "firmware.version 264\nfirmware.serial_number.0 3211314\n"
        "firmware.serial_number.1 1261457433\nfirmware.serial_number.2 540555353\n"
        "firmware.model 0\nfirmware.hwrev 8\n");
    CHECK(serial_from_firmware_diag(fw) == "ADEAMkswUBkgODhZ");
    CHECK(fw.at("firmware.hwrev") == "8");
    CHECK(serial_from_firmware_diag(parse_diag_text("firmware.version 264\n")).empty());
}

// ---------------------------------------------------------------------------
// Profile
// ---------------------------------------------------------------------------

namespace {
Profile sample_profile() {
    Profile p;
    p.pi_serial = "100000007eea876b";
    for (int i = 1; i <= 4; ++i) {
        ControllerIdentity c;
        c.id = i;
        c.bus = i;
        c.serial = "SERIAL" + std::to_string(i) + "XXXXXXXXXX";
        c.git_hash = std::string(40, static_cast<char>('a' + i));
        c.git_dirty = (i == 4);
        c.git_timestamp = 1723800000 + i;
        c.abi_version = 0x10100;
        c.register_map = 5;
        c.model = 0;
        c.hwrev = i == 3 ? "" : "8";
        p.controllers.push_back(c);
    }
    p.approved.body_vel_mps = 1.2;
    p.approved.body_omega_radps = 2.5;
    p.approved.velocity_limit_rev_s = 6.5;
    p.approved.accel_limit_rev_s2 = 12.0;
    p.caps.body_vel_mps = 1.74;
    p.caps.body_omega_radps = 6.0;
    p.caps.velocity_limit_rev_s = 8.0;
    p.caps.accel_limit_rev_s2 = 16.0;
    p.caps_derived = p.caps;  // approved inside the derived ceiling
    p.ceiling_mode = "derived";
    p.approved_under_override = false;
    p.ceiling_velocity_source = "controller 2 servo.default_velocity_limit 10.00 rev/s";
    p.ceiling_accel_source = "controller 1 servo.default_accel_limit 20.00 rev/s^2";
    p.ceiling_derivation = {"line one", "line \"two\""};
    p.yaml_velocity_limit_rev_s = 15.0;
    p.yaml_accel_limit_rev_s2 = 20.0;
    p.meters_per_motor_rev = 0.2171;
    p.operator_name = "D. Gurung";
    p.note = "smooth to 1.2 m/s, no yaw drift; \"wheel 3\" warm";
    p.approved_utc = "2026-09-07T03:04:05.000Z";
    p.log_path = "/home/pi/RobotFramework/logs/commission/x.ndjson";
    p.config_dir = "/home/pi/RobotFramework/config";
    p.window.from_seq = 1200;
    p.window.to_seq = 5400;
    p.window.from_mono_s = 4.8;
    p.window.to_mono_s = 21.6;
    p.window.drive_cycles = 3000;
    p.window.peak_cmd_wheel_rev_s = 6.4;
    p.window.peak_measured_wheel_rev_s = 6.3;
    p.window.peak_q_current_a = 7.2;
    p.window.peak_temperature_c = kNan;
    p.window.faults_seen = {};
    p.window.limit_codes_seen = {99, 104};
    return p;
}
}  // namespace

PHX_TEST(commission_profile_round_trip) {
    const Profile p = sample_profile();
    const std::string text = encode_profile(p);
    CHECK(contains(text, "\"type\": \"robot_profile\""));
    CHECK(!contains(text, "nan"));
    Profile q;
    std::string err;
    REQUIRE(decode_profile(text, &q, &err));
    CHECK(q.schema == kSchemaVersion);
    CHECK(q.pi_serial == p.pi_serial);
    REQUIRE(q.controllers.size() == 4);
    for (int i = 0; i < 4; ++i) {
        CHECK(q.controllers[i].id == p.controllers[i].id);
        CHECK(q.controllers[i].serial == p.controllers[i].serial);
        CHECK(q.controllers[i].git_hash == p.controllers[i].git_hash);
        CHECK(q.controllers[i].git_dirty == p.controllers[i].git_dirty);
        CHECK(q.controllers[i].git_timestamp == p.controllers[i].git_timestamp);
        CHECK(q.controllers[i].abi_version == p.controllers[i].abi_version);
        CHECK(q.controllers[i].hwrev == p.controllers[i].hwrev);
    }
    CHECK_NEAR(q.approved.velocity_limit_rev_s, 6.5, 1e-9);
    CHECK_NEAR(q.approved.accel_limit_rev_s2, 12.0, 1e-9);
    CHECK_NEAR(q.approved.body_vel_mps, 1.2, 1e-9);
    CHECK_NEAR(q.caps.velocity_limit_rev_s, 8.0, 1e-9);
    CHECK_NEAR(q.caps_derived.velocity_limit_rev_s, 8.0, 1e-9);
    CHECK(q.ceiling_mode == "derived");
    CHECK(!q.approved_under_override);
    CHECK(contains(text, "\"approved_under_override\": false"));
    CHECK_NEAR(q.ceiling_fraction, 0.8, 1e-9);
    CHECK(q.ceiling_velocity_source == p.ceiling_velocity_source);
    REQUIRE(q.ceiling_derivation.size() == 2);
    CHECK(q.ceiling_derivation[1] == "line \"two\"");
    CHECK(q.operator_name == p.operator_name);
    CHECK(q.note == p.note);
    CHECK(q.approved_utc == p.approved_utc);
    CHECK(q.log_path == p.log_path);
    CHECK(q.window.from_seq == 1200 && q.window.to_seq == 5400);
    CHECK(q.window.drive_cycles == 3000);
    CHECK_NEAR(q.window.peak_q_current_a, 7.2, 1e-9);
    CHECK(std::isnan(q.window.peak_temperature_c));  // null came back as NaN, not 0
    REQUIRE(q.window.limit_codes_seen.size() == 2);
    CHECK(q.window.limit_codes_seen[1] == 104);
    CHECK(q.window.faults_seen.empty());
    CHECK_NEAR(q.meters_per_motor_rev, 0.2171, 1e-12);
    // Encoding the decoded profile again is byte-identical: nothing is lost.
    CHECK(encode_profile(q) == text);
}

PHX_TEST(commission_profile_decode_rejects_garbage) {
    Profile q;
    std::string err;
    CHECK(!decode_profile("", &q, &err));
    CHECK(!decode_profile("{", &q, &err));
    CHECK(!decode_profile("[1,2]", &q, &err));
    CHECK(!decode_profile("{\"type\":\"other\",\"schema\":1,\"pi_serial\":\"x\"}", &q, &err));
    CHECK(!decode_profile("{\"type\":\"robot_profile\",\"schema\":99,\"pi_serial\":\"x\"}", &q, &err));
    CHECK(contains(err, "schema"));
    // Schema 1 had no override fields; no such file was ever written and a
    // reader must not guess what its `caps` meant.
    CHECK(!decode_profile("{\"type\":\"robot_profile\",\"schema\":1,\"pi_serial\":\"x\"}", &q, &err));
    CHECK(contains(err, "schema"));
    CHECK(!decode_profile("{\"type\":\"robot_profile\",\"schema\":2}", &q, &err));
    CHECK(contains(err, "pi_serial"));
}

PHX_TEST(commission_compare_controllers_detects_a_swapped_board) {
    const Profile p = sample_profile();
    CHECK(compare_controllers(p.controllers, p.controllers).empty());

    auto live = p.controllers;
    live[2].serial = "DIFFERENTBOARDXX";
    auto diff = compare_controllers(p.controllers, live);
    REQUIRE(diff.size() == 1);
    CHECK(contains(diff[0], "controller 3"));
    CHECK(contains(diff[0], "serial changed"));

    live = p.controllers;
    live[0].git_hash = std::string(40, 'z');
    diff = compare_controllers(p.controllers, live);
    REQUIRE(diff.size() == 1);
    CHECK(contains(diff[0], "firmware changed"));

    live = p.controllers;
    live.pop_back();
    diff = compare_controllers(p.controllers, live);
    REQUIRE(diff.size() == 1);
    CHECK(contains(diff[0], "not present now"));

    live = p.controllers;
    ControllerIdentity extra;
    extra.id = 5;
    extra.serial = "FIFTHWHEELXXXXXX";
    live.push_back(extra);
    diff = compare_controllers(p.controllers, live);
    REQUIRE(diff.size() == 1);
    CHECK(contains(diff[0], "not in the approved set"));

    // Order independence: compared by CAN id, not by position.
    live = p.controllers;
    std::swap(live[0], live[3]);
    CHECK(compare_controllers(p.controllers, live).empty());
}

// ---------------------------------------------------------------------------
// JSON reader
// ---------------------------------------------------------------------------

PHX_TEST(commission_json_reader_basics) {
    JsonValue v;
    std::string err;
    REQUIRE(parse_json(" {\"a\": [1, -2.5e1, true, null, \"s\\u00e9\\n\"], \"b\": {}} ", &v, &err));
    const JsonValue* a = v.get("a");
    REQUIRE(a != nullptr && a->type == JsonValue::Array && a->arr.size() == 5);
    CHECK_NEAR(a->arr[0].num, 1.0, 1e-12);
    CHECK_NEAR(a->arr[1].num, -25.0, 1e-12);
    CHECK(a->arr[2].type == JsonValue::Bool && a->arr[2].b);
    CHECK(a->arr[3].type == JsonValue::Null);
    CHECK(a->arr[4].str == "s\xc3\xa9\n");
    CHECK(v.get("b")->type == JsonValue::Object);
    CHECK(v.get("missing") == nullptr);
    CHECK(!parse_json("{\"a\":}", &v, &err));
    CHECK(!parse_json("{\"a\":1} x", &v, &err));
    CHECK(!parse_json("\"unterminated", &v, &err));
    CHECK(!parse_json("[1,]", &v, &err));
    CHECK(!parse_json("nul", &v, &err));
}

// ---------------------------------------------------------------------------
// Operator-settable ceiling (the C menu)
// ---------------------------------------------------------------------------

PHX_TEST(commission_override_status_detects_above_and_ratio) {
    const Caps derived = caps(1.718, 6.0, 8.0, 16.0);

    // Untouched: nothing manual, nothing above, mode "derived".
    OverrideStatus ov = override_status(derived, derived);
    CHECK(!ov.any_manual);
    CHECK(!ov.any_override);
    CHECK(std::string(ov.mode()) == "derived");
    for (Setting w : kAllSettings) {
        CHECK(!ov.of(w).manual);
        CHECK(!ov.of(w).above);
        CHECK_NEAR(ov.of(w).ratio, 1.0, 1e-12);
    }

    // Velocity raised to 25 over a derived 8: above, ratio 3.125; accel raised to 40 over 16: 2.5.
    Caps force = derived;
    force.velocity_limit_rev_s = 25.0;
    force.accel_limit_rev_s2 = 40.0;
    ov = override_status(force, derived);
    CHECK(ov.any_manual);
    CHECK(ov.any_override);
    CHECK(std::string(ov.mode()) == "manual");
    CHECK(ov.of(Setting::VelocityLimit).above);
    CHECK_NEAR(ov.of(Setting::VelocityLimit).ratio, 3.125, 1e-12);
    CHECK_NEAR(ov.of(Setting::VelocityLimit).in_force, 25.0, 1e-12);
    CHECK_NEAR(ov.of(Setting::VelocityLimit).derived, 8.0, 1e-12);
    CHECK(ov.of(Setting::AccelLimit).above);
    CHECK_NEAR(ov.of(Setting::AccelLimit).ratio, 2.5, 1e-12);
    CHECK(!ov.of(Setting::BodyVel).manual);
    CHECK(!ov.of(Setting::BodyOmega).manual);

    // Lowered below derived: manual but NOT an override, and the mode still
    // says "manual" so a reader knows the operator touched it.
    force = derived;
    force.velocity_limit_rev_s = 4.0;
    ov = override_status(force, derived);
    CHECK(ov.any_manual);
    CHECK(!ov.any_override);
    CHECK(std::string(ov.mode()) == "manual");
    CHECK(ov.of(Setting::VelocityLimit).manual);
    CHECK(!ov.of(Setting::VelocityLimit).above);
    CHECK_NEAR(ov.of(Setting::VelocityLimit).ratio, 0.5, 1e-12);

    // A value in force with no derived reference counts as an override.
    Caps no_ref = derived;
    no_ref.body_vel_mps = kNan;
    ov = override_status(derived, no_ref);
    CHECK(ov.of(Setting::BodyVel).above);
    CHECK(ov.any_override);
    CHECK(std::isnan(ov.of(Setting::BodyVel).ratio));

    // Exactly equal within tolerance is not manual.
    force = derived;
    force.accel_limit_rev_s2 = 16.0 + 1e-12;
    CHECK(!override_status(force, derived).any_manual);
}

PHX_TEST(commission_propose_ceiling_classifies_the_threshold) {
    const Caps derived = caps(1.718, 6.0, 8.0, 16.0);

    // At or below derived: valid, no confirmation needed.
    CeilingProposal p = propose_ceiling(Setting::VelocityLimit, 8.0, derived);
    CHECK(p.valid);
    CHECK(!p.above_derived);
    CHECK(!p.needs_confirm());
    CHECK_NEAR(p.ratio, 1.0, 1e-12);
    p = propose_ceiling(Setting::VelocityLimit, 4.0, derived);
    CHECK(p.valid && !p.needs_confirm());
    CHECK_NEAR(p.ratio, 0.5, 1e-12);

    // Just above derived: warn and confirm, with the ratio to show.
    p = propose_ceiling(Setting::VelocityLimit, 8.5, derived);
    CHECK(p.valid);
    CHECK(p.above_derived);
    CHECK(p.needs_confirm());
    CHECK_NEAR(p.ratio, 8.5 / 8.0, 1e-12);
    CHECK_NEAR(p.derived, 8.0, 1e-12);
    // Far above: still valid (no upper bound), just a bigger ratio.
    p = propose_ceiling(Setting::VelocityLimit, 500.0, derived);
    CHECK(p.valid);
    CHECK(p.needs_confirm());
    CHECK_NEAR(p.ratio, 62.5, 1e-12);
    CHECK(p.error.empty());
    p = propose_ceiling(Setting::AccelLimit, 40.0, derived);
    CHECK(p.needs_confirm());
    CHECK_NEAR(p.ratio, 2.5, 1e-12);

    // Invalid: zero, negative, nan, inf. Never invalid for being large.
    CHECK(!propose_ceiling(Setting::VelocityLimit, 0.0, derived).valid);
    CHECK(!propose_ceiling(Setting::VelocityLimit, -1.0, derived).valid);
    CHECK(!propose_ceiling(Setting::VelocityLimit, kNan, derived).valid);
    CHECK(!propose_ceiling(Setting::VelocityLimit, kInf, derived).valid);
    CHECK(!propose_ceiling(Setting::VelocityLimit, 0.0, derived).error.empty());
    CHECK(!propose_ceiling(Setting::VelocityLimit, 0.0, derived).needs_confirm());

    // No derived reference: anything is "above" (the conservative reading).
    Caps no_ref = derived;
    no_ref.body_omega_radps = kNan;
    p = propose_ceiling(Setting::BodyOmega, 1.0, no_ref);
    CHECK(p.valid);
    CHECK(p.above_derived);
    CHECK(std::isnan(p.ratio));
}

PHX_TEST(commission_parse_positive_double_text) {
    double v = 0.0;
    std::string err;
    CHECK(parse_positive_double("25", &v, &err) && v == 25.0);
    CHECK(parse_positive_double(" 25.5 ", &v, &err) && v == 25.5);
    CHECK(parse_positive_double("2.5e1", &v, &err) && v == 25.0);
    CHECK(parse_positive_double(".5", &v, &err) && v == 0.5);
    CHECK(parse_positive_double("+3", &v, &err) && v == 3.0);
    CHECK(parse_positive_double("500", &v, &err) && v == 500.0);
    CHECK(parse_positive_double("1e6", &v, &err) && v == 1e6);

    CHECK(!parse_positive_double("", &v, &err));
    CHECK(!parse_positive_double("   ", &v, &err));
    CHECK(!parse_positive_double("abc", &v, &err));
    CHECK(!parse_positive_double("25x", &v, &err));  // strtod would stop early and accept
    CHECK(!parse_positive_double("-3", &v, &err));
    CHECK(!parse_positive_double("0", &v, &err));
    CHECK(!parse_positive_double("0.0", &v, &err));
    CHECK(!parse_positive_double("nan", &v, &err));
    CHECK(!parse_positive_double("inf", &v, &err));
    CHECK(!parse_positive_double("0x10", &v, &err));
    CHECK(!parse_positive_double("1,5", &v, &err));
    CHECK(!err.empty());

    // The text route agrees with the numeric route.
    const Caps derived = caps(1.718, 6.0, 8.0, 16.0);
    const CeilingProposal p = propose_ceiling_text(Setting::VelocityLimit, " 25 ", derived);
    CHECK(p.valid && p.needs_confirm());
    CHECK_NEAR(p.value, 25.0, 1e-12);
    CHECK_NEAR(p.ratio, 3.125, 1e-12);
    const CeilingProposal bad = propose_ceiling_text(Setting::VelocityLimit, "lots", derived);
    CHECK(!bad.valid);
    CHECK(contains(bad.error, "lots"));
    CHECK_NEAR(bad.derived, 8.0, 1e-12);  // still tells the screen what derived is
}

PHX_TEST(commission_apply_ceiling_has_no_upper_bound) {
    // The explicit request: if the operator writes 500, the tool uses 500.
    Caps force = caps(1.718, 6.0, 8.0, 16.0);
    Settings s;
    s.velocity_limit_rev_s = 4.0;
    s.accel_limit_rev_s2 = 8.0;
    s.body_vel_mps = 0.3;
    s.body_omega_radps = 1.0;

    CeilingChange r = apply_ceiling(force, s, Setting::VelocityLimit, 25.0);
    CHECK(r.changed);
    CHECK_NEAR(r.before, 8.0, 1e-12);
    CHECK_NEAR(r.after, 25.0, 1e-12);
    CHECK_NEAR(force.velocity_limit_rev_s, 25.0, 1e-12);  // honoured, not clamped
    CHECK(!r.setting_clamped);
    CHECK_NEAR(s.velocity_limit_rev_s, 4.0, 1e-12);       // the setting did not move up

    r = apply_ceiling(force, s, Setting::VelocityLimit, 500.0);
    CHECK_NEAR(force.velocity_limit_rev_s, 500.0, 1e-12);
    r = apply_ceiling(force, s, Setting::AccelLimit, 1e6);
    CHECK_NEAR(force.accel_limit_rev_s2, 1e6, 1e-12);
    r = apply_ceiling(force, s, Setting::BodyVel, 50.0);
    CHECK_NEAR(force.body_vel_mps, 50.0, 1e-12);
    r = apply_ceiling(force, s, Setting::BodyOmega, 100.0);
    CHECK_NEAR(force.body_omega_radps, 100.0, 1e-12);

    // The raised ceiling is effective: the step keys can now walk the
    // setting all the way up to it, and stop there.
    for (int i = 0; i < 2000; ++i) step_setting(s, Setting::VelocityLimit, +1, force);
    CHECK_NEAR(s.velocity_limit_rev_s, 500.0, 1e-9);
    CHECK(step_setting(s, Setting::VelocityLimit, +1, force).at_ceiling);
    // And the wheel clamp honours it: a 100 rev/s demand is untouched under 500.
    const WheelClamp wc = clamp_wheels({{100.0, -100.0, 50.0, 0.0}}, force.velocity_limit_rev_s);
    CHECK(!wc.clipped);
    CHECK_NEAR(wc.wheels[0], 100.0, 1e-12);

    // Same value again: not a change.
    r = apply_ceiling(force, s, Setting::VelocityLimit, 500.0);
    CHECK(!r.changed);

    // Invalid values are ignored, not clamped into range.
    r = apply_ceiling(force, s, Setting::VelocityLimit, 0.0);
    CHECK(!r.changed);
    CHECK_NEAR(force.velocity_limit_rev_s, 500.0, 1e-12);
    r = apply_ceiling(force, s, Setting::VelocityLimit, -5.0);
    CHECK(!r.changed);
    r = apply_ceiling(force, s, Setting::VelocityLimit, kNan);
    CHECK(!r.changed);
    r = apply_ceiling(force, s, Setting::VelocityLimit, kInf);
    CHECK(!r.changed);
    CHECK_NEAR(force.velocity_limit_rev_s, 500.0, 1e-12);
}

PHX_TEST(commission_apply_ceiling_lowering_pulls_the_setting_down) {
    Caps force = caps(1.718, 6.0, 8.0, 16.0);
    Settings s;
    s.velocity_limit_rev_s = 6.0;
    s.accel_limit_rev_s2 = 3.0;

    // Setting 6.0 is above a new ceiling of 4.0: it is pulled onto it and reported.
    CeilingChange r = apply_ceiling(force, s, Setting::VelocityLimit, 4.0);
    CHECK(r.changed);
    CHECK(r.setting_clamped);
    CHECK_NEAR(r.setting_before, 6.0, 1e-12);
    CHECK_NEAR(r.setting_after, 4.0, 1e-12);
    CHECK_NEAR(s.velocity_limit_rev_s, 4.0, 1e-12);
    CHECK_NEAR(force.velocity_limit_rev_s, 4.0, 1e-12);
    // Setting 3.0 is below a new ceiling of 4.0: untouched.
    r = apply_ceiling(force, s, Setting::AccelLimit, 4.0);
    CHECK(r.changed);
    CHECK(!r.setting_clamped);
    CHECK_NEAR(s.accel_limit_rev_s2, 3.0, 1e-12);
    // Settings never pass the ceiling in force afterwards either.
    for (int i = 0; i < 100; ++i) step_setting(s, Setting::VelocityLimit, +1, force);
    CHECK(s.velocity_limit_rev_s <= 4.0 + 1e-12);
}

PHX_TEST(commission_cycle_record_carries_both_ceilings_and_the_override_flag) {
    CycleRecord r;
    r.seq = 5;
    r.t_mono_s = 0.02;
    r.t_wall_unix_s = 1788000000.0;
    r.caps = caps(1.718, 6.0, 25.0, 16.0);
    r.caps_derived = caps(1.718, 6.0, 8.0, 16.0);
    r.ceiling_manual = true;
    r.override_active = true;
    std::snprintf(r.state, sizeof r.state, "%s", "menu");
    for (int i = 0; i < 4; ++i) r.wheels[i].id = i + 1;

    const std::string line = encode_cycle(r);
    CHECK(contains(line, "\"ceiling_mode\":\"manual\""));
    CHECK(contains(line, "\"override_active\":true"));
    JsonValue v;
    std::string err;
    REQUIRE(parse_json(line, &v, &err));
    REQUIRE(v.get("caps") != nullptr && v.get("caps_derived") != nullptr);
    CHECK_NEAR(v.get("caps")->get("velocity_limit_rev_s")->number_or(-1), 25.0, 1e-9);
    CHECK_NEAR(v.get("caps_derived")->get("velocity_limit_rev_s")->number_or(-1), 8.0, 1e-9);
    CHECK(v.get("override_active")->bool_or(false));
    CHECK(v.get("ceiling_mode")->string_or("") == "manual");
    CHECK(v.get("state")->string_or("") == "menu");

    // The plain case says so explicitly, too: never absent, never ambiguous.
    r.caps = r.caps_derived;
    r.ceiling_manual = false;
    r.override_active = false;
    const std::string plain = encode_cycle(r);
    CHECK(contains(plain, "\"ceiling_mode\":\"derived\""));
    CHECK(contains(plain, "\"override_active\":false"));
}

PHX_TEST(commission_profile_round_trip_carries_the_override) {
    // A limit approved at 25 rev/s under an override must be distinguishable
    // later from one approved inside the derived ceiling.
    Profile p = sample_profile();
    p.approved.velocity_limit_rev_s = 25.0;
    p.approved.accel_limit_rev_s2 = 30.0;
    p.caps = caps(1.718, 6.0, 25.0, 40.0);
    p.caps_derived = caps(1.718, 6.0, 8.0, 16.0);
    p.ceiling_mode = "manual";
    p.approved_under_override = true;

    const std::string text = encode_profile(p);
    CHECK(contains(text, "\"approved_under_override\": true"));
    CHECK(contains(text, "\"mode\": \"manual\""));
    CHECK(contains(text, "\"caps_derived\""));
    CHECK(contains(text, "\"schema\": 2"));

    Profile q;
    std::string err;
    REQUIRE(decode_profile(text, &q, &err));
    CHECK(q.approved_under_override);
    CHECK(q.ceiling_mode == "manual");
    CHECK_NEAR(q.approved.velocity_limit_rev_s, 25.0, 1e-9);
    CHECK_NEAR(q.caps.velocity_limit_rev_s, 25.0, 1e-9);
    CHECK_NEAR(q.caps.accel_limit_rev_s2, 40.0, 1e-9);
    CHECK_NEAR(q.caps_derived.velocity_limit_rev_s, 8.0, 1e-9);
    CHECK_NEAR(q.caps_derived.accel_limit_rev_s2, 16.0, 1e-9);
    // The override is recoverable from the two ceilings alone as well.
    const OverrideStatus ov = override_status(q.caps, q.caps_derived);
    CHECK(ov.any_override);
    CHECK_NEAR(ov.of(Setting::VelocityLimit).ratio, 3.125, 1e-9);
    CHECK_NEAR(ov.of(Setting::AccelLimit).ratio, 2.5, 1e-9);
    CHECK(encode_profile(q) == text);

    // The same approved numbers inside a derived ceiling read as NOT an override.
    Profile inside = sample_profile();
    inside.approved.velocity_limit_rev_s = 6.5;
    REQUIRE(decode_profile(encode_profile(inside), &q, &err));
    CHECK(!q.approved_under_override);
    CHECK(q.ceiling_mode == "derived");
    CHECK(!override_status(q.caps, q.caps_derived).any_override);
}

PHX_TEST(commission_parse_ceiling_spec_for_the_test_flag) {
    std::vector<std::pair<Setting, double>> items;
    std::string err;
    REQUIRE(parse_ceiling_spec("velocity=25,accel=40", &items, &err));
    REQUIRE(items.size() == 2);
    CHECK(items[0].first == Setting::VelocityLimit && items[0].second == 25.0);
    CHECK(items[1].first == Setting::AccelLimit && items[1].second == 40.0);
    REQUIRE(parse_ceiling_spec(" body = 5 , yaw=12 ,", &items, &err));
    REQUIRE(items.size() == 2);
    CHECK(items[0].first == Setting::BodyVel && items[0].second == 5.0);
    CHECK(items[1].first == Setting::BodyOmega && items[1].second == 12.0);
    REQUIRE(parse_ceiling_spec("velocity=500", &items, &err));
    CHECK(items[0].second == 500.0);  // no upper bound here either

    CHECK(!parse_ceiling_spec("", &items, &err));
    CHECK(!parse_ceiling_spec("velocity", &items, &err));
    CHECK(!parse_ceiling_spec("speed=5", &items, &err));
    CHECK(contains(err, "speed"));
    CHECK(!parse_ceiling_spec("velocity=0", &items, &err));
    CHECK(!parse_ceiling_spec("velocity=abc", &items, &err));
    CHECK(!parse_ceiling_spec("velocity=-2", &items, &err));
}

PHX_TEST(commission_describe_max_velocity_for_the_warning) {
    std::vector<BackstopReading> c;
    for (int id = 1; id <= 4; ++id) c.push_back(live(id, 10.0, 20.0, 10.0));
    CHECK(describe_max_velocity(c) == "10.00 rev/s on all four controllers");

    c[2].max_velocity_rev_s = 45.0;
    CHECK(describe_max_velocity(c) == "10.00 rev/s on controllers 1,2,4; 45.00 rev/s on controller 3");

    c[3].max_velocity_rev_s = kNan;
    CHECK(describe_max_velocity(c) ==
          "10.00 rev/s on controllers 1,2; 45.00 rev/s on controller 3; unreadable on controller 4");

    for (auto& b : c) b.max_velocity_rev_s = kNan;
    CHECK(describe_max_velocity(c) == "unreadable on every controller");
    CHECK(describe_max_velocity({}) == "unreadable on every controller");
}
