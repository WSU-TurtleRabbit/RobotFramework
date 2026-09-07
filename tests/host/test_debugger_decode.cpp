// The debugger's status-word decode tables. Pure, so they run on the host.
//
// The property these tests defend is not "the names are spelled right" — it is
// that the tool can never present a limiting condition as a fault, a fault as
// healthy, or an unrecognised code as fine. Each of those would send someone
// chasing the wrong thing on a bench with a robot on blocks.
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "Motion/phx/testing.h"
#include "tools/debugger/decode.h"

using namespace rf::dbg;

namespace {

bool named(const CodeInfo& c, const char* want) {
    return std::strcmp(c.name, want) == 0;
}

}  // namespace

PHX_TEST(debugger_decode_success_is_the_only_ok_fault) {
    // 0 is the only fault value that may read as healthy. Everything else in
    // the hard-fault range must be presented as a fault.
    CHECK(decode_fault(0).severity == Severity::Ok);
    for (int code = 1; code <= 7; ++code) {
        CHECK(decode_fault(code).severity == Severity::Fault);
        CHECK(!decode_fault(code).is_limit);
    }
    for (int code = 32; code <= 50; ++code) {
        const auto info = decode_fault(code);
        CHECK(info.severity != Severity::Ok);
        CHECK(!info.is_limit);
    }
}

PHX_TEST(debugger_decode_limits_are_never_faults) {
    // 96..105 are live limit reasons reported while a control mode runs.
    // Presenting one as a latched fault would send an operator looking for a
    // failure that never happened.
    for (int code = 96; code <= 105; ++code) {
        const auto info = decode_fault(code);
        CHECK(info.is_limit);
        CHECK(info.severity != Severity::Fault);
        CHECK(info.severity != Severity::Ok);
    }
}

PHX_TEST(debugger_decode_known_codes_match_firmware) {
    // Spot-check the codes that are easiest to confuse, against moteus/fw/error.h.
    CHECK(named(decode_fault(33), "motor_driver"));       // gate driver
    CHECK(named(decode_fault(34), "over_voltage"));
    CHECK(named(decode_fault(36), "motor_not_configured"));
    CHECK(named(decode_fault(39), "start_outside_limit"));
    // Bus under-voltage is 40. The audit had this as 33 in one place; 33 is
    // the gate-driver fault and has a different threshold entirely.
    CHECK(named(decode_fault(40), "under_voltage"));
    CHECK(named(decode_fault(99), "limit_max_current"));
    CHECK(named(decode_fault(103), "limit_position_bounds"));
    CHECK(named(decode_fault(104), "limit_flux_braking"));
}

PHX_TEST(debugger_decode_unknown_code_is_not_healthy) {
    // A controller running firmware newer than this build must not have its
    // unrecognised status silently rendered as "ok".
    for (int code : {8, 31, 51, 95, 106, 127}) {
        const auto info = decode_fault(code);
        CHECK(named(info, "unknown"));
        CHECK(info.severity != Severity::Ok);
        CHECK(info.code == code);
    }
    // An unknown code in the limit range is still flagged as a limit, so it
    // is not escalated into a phantom hard fault either.
    CHECK(decode_fault(106).is_limit);
    CHECK(!decode_fault(51).is_limit);
}

PHX_TEST(debugger_decode_modes_cover_the_enum) {
    for (int mode = 0; mode <= 15; ++mode) {
        CHECK(!named(decode_mode(mode), "invalid"));
    }
    CHECK(named(decode_mode(10), "position"));
    CHECK(named(decode_mode(11), "position_timeout"));
    CHECK(named(decode_mode(1), "fault"));
    // Outside the enum must not be presented as a real mode.
    CHECK(named(decode_mode(16), "invalid"));
    CHECK(named(decode_mode(-1), "invalid"));
}

PHX_TEST(debugger_decode_position_timeout_is_a_warning_not_ok) {
    // Mode 11 means the per-command watchdog expired: the host loop stalled or
    // CAN dropped. It is a real signal and must not render as a normal mode.
    const auto info = decode_mode(11);
    CHECK(info.severity == Severity::Warning);
}

PHX_TEST(debugger_mode_energized_classification) {
    // Used to decide whether it is safe to say "the motors are dead".
    CHECK(!mode_is_energized(0));   // stopped
    CHECK(!mode_is_energized(1));   // fault
    CHECK(!mode_is_energized(11));  // position_timeout: output already cut
    CHECK(mode_is_energized(10));   // position
    CHECK(mode_is_energized(12));   // zero_velocity damps, which is driving
    CHECK(mode_is_energized(15));   // brake
}

PHX_TEST(debugger_torque_constant_uses_the_moteus_convention) {
    // moteus: Kt = ((3/2)(1/sqrt(3))(60/2pi)) / Kv ~ 8.2699 / Kv.
    // The generic catalog relation 9.5493/KV is a DIFFERENT convention and
    // overstates torque by exactly 2/sqrt(3). Using it makes a healthy wheel
    // look like it is under-delivering by 15%.
    CHECK_NEAR(kMoteusTorqueFactor, 8.2699, 1e-3);
    CHECK_NEAR(torque_constant_nm_per_a(330.0), 8.2699 / 330.0, 1e-9);
    CHECK_NEAR(torque_constant_nm_per_a(330.0), 0.02506, 1e-5);

    // The generic convention would give 0.02894 — assert we are NOT that.
    CHECK(std::abs(torque_constant_nm_per_a(330.0) - 9.5493 / 330.0) > 1e-4);

    const double ratio = (9.5493 / 330.0) / torque_constant_nm_per_a(330.0);
    CHECK_NEAR(ratio, 2.0 / std::sqrt(3.0), 1e-4);
}

PHX_TEST(debugger_torque_constant_refuses_to_invent_a_value) {
    // A missing or nonsensical Kv must propagate as NaN, not as a plausible
    // number that would silently feed a torque estimate.
    CHECK(std::isnan(torque_constant_nm_per_a(0.0)));
    CHECK(std::isnan(torque_constant_nm_per_a(-330.0)));
    CHECK(std::isnan(torque_constant_nm_per_a(std::nan(""))));
    CHECK(std::isnan(torque_constant_nm_per_a(
        std::numeric_limits<double>::infinity())));
}

PHX_TEST(debugger_severity_words_are_stable) {
    // These strings end up in logs that people grep.
    CHECK(std::strcmp(severity_word(Severity::Ok), "ok") == 0);
    CHECK(std::strcmp(severity_word(Severity::Notice), "notice") == 0);
    CHECK(std::strcmp(severity_word(Severity::Warning), "warn") == 0);
    CHECK(std::strcmp(severity_word(Severity::Fault), "FAULT") == 0);
}
