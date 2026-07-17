// Safety supervisor tests, ported 1:1 from phoenix-rf
// crates/protocol/src/safety.rs.
#include <cmath>

#include "Motion/phx/testing.h"
#include "Safety/supervisor.h"

using rf::DriveMode;
using rf::MotorObs;
using rf::SafetyConfig;
using rf::StopReason;
using rf::Supervisor;

namespace {

MotorObs ok(int id) {
    MotorObs m;
    m.id = id;
    m.replied = true;
    m.fault = 0;
    m.temperature = 30.0;
    m.current = 1.0;
    return m;
}

}  // namespace

PHX_TEST(safety_safe_mode_scales_translation_preserving_direction) {
    SafetyConfig cfg;
    cfg.mode = DriveMode::Safe;
    cfg.safe_linear_mps = 1.0;
    const Supervisor s(cfg);
    // Diagonal 3-4-5 -> speed 5, capped to 1, direction preserved (0.6,0.8).
    // (The legacy SAFE mode would have STOPPED the robot instead.)
    const BodyTwist out = s.shape_twist(BodyTwist{3.0, 4.0, 0.0});
    CHECK_NEAR(out.vx, 0.6, 1e-9);
    CHECK_NEAR(out.vy, 0.8, 1e-9);
    CHECK_NEAR(std::sqrt(out.vx * out.vx + out.vy * out.vy), 1.0, 1e-9);
}

PHX_TEST(safety_unsafe_mode_passes_through) {
    SafetyConfig cfg;
    cfg.mode = DriveMode::Unsafe;
    const Supervisor s(cfg);
    const BodyTwist out = s.shape_twist(BodyTwist{9.0, 9.0, 9.0});
    CHECK(out.vx == 9.0);
    CHECK(out.vy == 9.0);
    CHECK(out.w == 9.0);
}

PHX_TEST(safety_angular_is_clamped) {
    SafetyConfig cfg;
    cfg.mode = DriveMode::Capped;
    cfg.capped_angular_rps = 2.0;
    const Supervisor s(cfg);
    CHECK(s.shape_twist(BodyTwist{0.0, 0.0, 5.0}).w == 2.0);
    CHECK(s.shape_twist(BodyTwist{0.0, 0.0, -5.0}).w == -2.0);
}

PHX_TEST(safety_stale_command_gates_motion) {
    SafetyConfig cfg;
    cfg.command_timeout_ms = 200;
    const Supervisor s(cfg);
    CHECK(!s.motion_gate(1000, 900).has_value());  // 100ms fresh
    CHECK(s.motion_gate(1000, 700) == std::optional<StopReason>(StopReason::command_stale()));
    CHECK(s.motion_gate(1000, std::nullopt) ==
          std::optional<StopReason>(StopReason::command_stale()));
}

PHX_TEST(safety_sustained_fault_latches_after_grace) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 3;
    Supervisor s(cfg);
    MotorObs faulted = ok(2);
    faulted.fault = 33;
    CHECK(s.observe({faulted}, 12.0).empty());  // 1
    CHECK(s.observe({faulted}, 12.0).empty());  // 2
    const auto newly = s.observe({faulted}, 12.0);  // 3 -> trip
    REQUIRE(newly.size() == 1);
    CHECK(newly[0] == StopReason::motor_fault(2));
    CHECK(s.estop());
    // Motion gated by estop now.
    CHECK(s.motion_gate(0, 0).has_value());
}

PHX_TEST(safety_transient_fault_does_not_latch) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 3;
    Supervisor s(cfg);
    MotorObs faulted = ok(1);
    faulted.fault = 33;
    s.observe({faulted}, 12.0);
    s.observe({ok(1)}, 12.0);  // recovered -> counter resets
    s.observe({faulted}, 12.0);
    CHECK(!s.estop());
}

PHX_TEST(safety_overtemp_and_undervoltage_trip) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    cfg.trip_temp_c = 70.0;
    cfg.min_bus_voltage = 10.0;
    Supervisor s(cfg);
    MotorObs hot = ok(3);
    hot.temperature = 72.0;
    auto newly = s.observe({hot}, 12.0);
    REQUIRE(newly.size() == 1);
    CHECK(newly[0] == StopReason::over_temp(3));
    s.clear();
    newly = s.observe({ok(1)}, 9.0);
    REQUIRE(newly.size() == 1);
    CHECK(newly[0] == StopReason::under_voltage());
}

PHX_TEST(safety_zero_voltage_reading_is_not_undervoltage) {
    // Before any motor replies, avg voltage is 0 — must not trip.
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    Supervisor s(cfg);
    CHECK(s.observe({}, 0.0).empty());
    CHECK(!s.estop());
}

PHX_TEST(safety_clear_recovers) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    Supervisor s(cfg);
    s.trip(StopReason::estop());
    CHECK(s.estop());
    s.clear();
    CHECK(!s.estop());
    CHECK(!s.motion_gate(0, 0).has_value());
}

PHX_TEST(safety_overcurrent_uses_its_own_grace) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    cfg.current_grace_ticks = 3;
    cfg.trip_current_a = 14.0;
    Supervisor s(cfg);
    MotorObs hot = ok(4);
    hot.current = 17.0;
    // Two ticks of 17A — an accel transient — must NOT trip...
    CHECK(s.observe({hot}, 12.0).empty());
    CHECK(s.observe({hot}, 12.0).empty());
    // ...but a sustained third does.
    const auto newly = s.observe({hot}, 12.0);
    REQUIRE(newly.size() == 1);
    CHECK(newly[0] == StopReason::over_current(4));
    CHECK(s.estop());
}

PHX_TEST(safety_latched_trip_auto_recovers_when_healthy_and_idle) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    cfg.current_grace_ticks = 1;
    cfg.recovery_s = 3.0;
    Supervisor s(cfg);
    MotorObs hot = ok(4);
    hot.current = 17.0;
    s.observe({hot}, 12.0);
    CHECK(s.estop());
    // Healthy again, operator idle: hold-off counts from first healthy tick.
    s.observe({ok(4)}, 12.0);
    CHECK(!s.try_recover(1000, true).has_value());  // t0
    s.observe({ok(4)}, 12.0);
    CHECK(!s.try_recover(2000, true).has_value());  // +1s
    s.observe({ok(4)}, 12.0);
    const auto rec = s.try_recover(4200, true);  // +3.2s
    REQUIRE(rec.has_value());
    REQUIRE(rec->size() == 1);
    CHECK((*rec)[0] == StopReason::over_current(4));
    CHECK(!s.estop());
    CHECK(!s.motion_gate(4200, 4200).has_value());  // driving again
}

PHX_TEST(safety_recovery_blocked_while_operator_commands_motion) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    cfg.current_grace_ticks = 1;
    cfg.recovery_s = 1.0;
    Supervisor s(cfg);
    MotorObs hot = ok(4);
    hot.current = 17.0;
    s.observe({hot}, 12.0);
    CHECK(s.estop());
    s.observe({ok(4)}, 12.0);
    CHECK(!s.try_recover(1000, false).has_value());  // sticks held -> no
    CHECK(!s.try_recover(9000, false).has_value());  // ...however long
    // Releasing the sticks restarts the hold-off from scratch.
    CHECK(!s.try_recover(9100, true).has_value());
    const auto rec = s.try_recover(10200, true);
    REQUIRE(rec.has_value());
    CHECK((*rec)[0] == StopReason::over_current(4));
}

PHX_TEST(safety_recovery_needs_hysteresis_and_can_be_disabled) {
    // Still warm (65 C with a 70 C trip): not "healthy", never recovers.
    {
        SafetyConfig cfg;
        cfg.fault_grace_ticks = 1;
        cfg.recovery_s = 1.0;
        cfg.trip_temp_c = 70.0;
        Supervisor s(cfg);
        MotorObs hot = ok(3);
        hot.temperature = 72.0;
        s.observe({hot}, 12.0);
        CHECK(s.estop());
        MotorObs warm = ok(3);
        warm.temperature = 65.0;
        s.observe({warm}, 12.0);
        CHECK(!s.try_recover(60000, true).has_value());
        CHECK(s.estop());
    }

    // recovery_s = 0 disables auto-recovery entirely.
    {
        SafetyConfig cfg;
        cfg.fault_grace_ticks = 1;
        cfg.current_grace_ticks = 1;
        cfg.recovery_s = 0.0;
        Supervisor s(cfg);
        MotorObs hot = ok(4);
        hot.current = 17.0;
        s.observe({hot}, 12.0);
        s.observe({ok(4)}, 12.0);
        CHECK(!s.try_recover(600000, true).has_value());
        CHECK(s.estop());
    }
}

PHX_TEST(safety_stale_numbers_from_silent_motor_are_not_evaluated) {
    SafetyConfig cfg;
    cfg.fault_grace_ticks = 1;
    Supervisor s(cfg);
    // A motor that didn't reply, carrying stale hot/fault numbers, must not trip.
    MotorObs silent;
    silent.id = 4;
    silent.replied = false;
    silent.fault = 33;
    silent.temperature = 99.0;
    silent.current = 50.0;
    CHECK(s.observe({silent}, 12.0).empty());
    CHECK(!s.estop());
}
