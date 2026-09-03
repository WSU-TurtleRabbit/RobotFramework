// Safety supervisor — pure decision logic, deterministic, fully unit-tested.
// Ported from the field-proven Rust implementation in phoenix-rf
// (crates/protocol/src/safety.rs).
//
// Owns every reason the robot must not (or must stop) moving:
//   * command staleness (lost comms -> stop),
//   * per-motor faults / over-temperature / over-current,
//   * bus under-voltage,
//   * the operator drive-mode speed envelope (safe / capped / unsafe).
//
// It never touches hardware; the superloop feeds it observations and applies
// its verdicts. Trips use a grace count so a single noisy sample can't
// nuisance-stop the robot. Recovery is either explicit (operator STOP ->
// clear()) or automatic: once every trip condition has been healthy (with
// hysteresis) for `recovery_s` continuous seconds AND the operator is
// commanding zero, the latch releases on its own — a tripped robot must
// never need a power cycle.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "../Math/kinematics.h"

namespace rf {

enum class DriveMode {
    Safe,    // conservative envelope for bring-up and untested changes
    Capped,  // full competition envelope, direction-preserving scale-to-fit
    Unsafe,  // no software envelope (still watchdog- and fault-protected)
};

struct SafetyConfig {
    DriveMode mode = DriveMode::Safe;
    // Body linear speed cap (magnitude of the vx/vy vector), m/s, per mode.
    double safe_linear_mps = 0.6;
    double safe_angular_rps = 1.5;
    double capped_linear_mps = 2.5;
    double capped_angular_rps = 6.0;
    // A command older than this stops the robot.
    uint64_t command_timeout_ms = 250;
    // Per-motor board temperature that trips a protective stop (below the
    // moteus hard fault_temperature of 75 C, so we stop first).
    double trip_temp_c = 70.0;
    // Bus voltage below which we protective-stop (sagging battery).
    double min_bus_voltage = 10.0;
    // Sustained per-motor current that trips a stop, A.
    double trip_current_a = 10.0;
    // Consecutive bad observations before a threshold trip latches.
    uint32_t fault_grace_ticks = 5;
    // Consecutive over-current observations before the trip latches. Kept
    // separate from fault_grace_ticks: q-current legitimately spikes past
    // the trip level for tens of milliseconds during hard acceleration, so
    // this must span a real stall, not a transient.
    uint32_t current_grace_ticks = 75;
    // Auto-recovery hold-off, s: a latched trip clears itself after every
    // trip condition has been healthy (with hysteresis) for this long while
    // the operator commands zero. 0 disables auto-recovery (latch until
    // STOP/restart).
    double recovery_s = 3.0;
    // moteus per-command watchdog, s (hardware failsafe; applied by
    // Telemetry, carried here so one struct describes the whole policy).
    double watchdog_s = 0.10;
};

// A per-motor observation the supervisor evaluates.
struct MotorObs {
    int id = 0;
    bool replied = false;
    int fault = 0;
    double temperature = 0.0;
    double current = 0.0;
};

// Why the robot is (or should be) stopped.
struct StopReason {
    enum Kind {
        CommandStale,
        Estop,
        MotorFault,
        OverTemp,
        OverCurrent,
        UnderVoltage,
    };
    Kind kind = Estop;
    int motor = 0;  // meaningful for the per-motor kinds

    bool operator==(const StopReason& o) const {
        return kind == o.kind && motor == o.motor;
    }
    bool operator!=(const StopReason& o) const { return !(*this == o); }

    static StopReason command_stale() { return {CommandStale, 0}; }
    static StopReason estop() { return {Estop, 0}; }
    static StopReason motor_fault(int id) { return {MotorFault, id}; }
    static StopReason over_temp(int id) { return {OverTemp, id}; }
    static StopReason over_current(int id) { return {OverCurrent, id}; }
    static StopReason under_voltage() { return {UnderVoltage, 0}; }

    const char* word() const {
        switch (kind) {
            case CommandStale: return "command_stale";
            case Estop: return "estop";
            case MotorFault: return "motor_fault";
            case OverTemp: return "over_temp";
            case OverCurrent: return "over_current";
            case UnderVoltage: return "under_voltage";
        }
        return "estop";
    }
};

class Supervisor {
public:
    Supervisor() = default;
    explicit Supervisor(const SafetyConfig& cfg) : cfg_(cfg) {}

    const SafetyConfig& config() const { return cfg_; }
    double watchdog_s() const { return cfg_.watchdog_s; }
    bool estop() const { return estop_latched_; }
    const std::vector<StopReason>& reasons() const { return latched_reasons_; }

    // Trip the estop latch (e.g. explicit external fault).
    void trip(const StopReason& reason);

    // Clear the estop latch and all grace counters (operator STOP recovery).
    void clear();

    // Shape a commanded twist into the active drive-mode envelope, preserving
    // direction (the translation vector is scaled as a UNIT, so a diagonal
    // command doesn't get bent toward an axis — and unlike the old SAFE mode,
    // an over-limit command is shaped, not turned into a hard stop).
    BodyTwist shape_twist(const BodyTwist& t) const;

    // Is motion currently allowed, given comms freshness and the estop latch?
    // Returns the reason to hold zero, or nullopt if motion is permitted.
    // last_cmd_ms: nullopt = no valid command ever received.
    std::optional<StopReason> motion_gate(uint64_t now_ms,
                                          std::optional<uint64_t> last_cmd_ms) const;

    // Evaluate motor + voltage observations. Latches an estop (with a grace
    // count) on a sustained fault/over-temp/over-current/under-voltage.
    // Returns the newly-tripped reasons this call (empty if nothing new).
    std::vector<StopReason> observe(const std::vector<MotorObs>& motors, double avg_voltage);

    // Auto-recovery: call every tick after observe(). If the estop is
    // latched, every trip condition has stayed healthy for recovery_s
    // continuous seconds, and the operator is commanding zero (so releasing
    // the latch can't lurch the robot), the latch clears itself. Returns the
    // released reasons for logging; nullopt if still latched (or not latched).
    std::optional<std::vector<StopReason>> try_recover(uint64_t now_ms, bool operator_idle);

private:
    void latch(const StopReason& reason, std::vector<StopReason>& newly);

    SafetyConfig cfg_{};
    bool estop_latched_ = false;
    std::vector<StopReason> latched_reasons_;
    // grace counters
    std::map<int, uint32_t> temp_bad_;
    std::map<int, uint32_t> cur_bad_;
    std::map<int, uint32_t> fault_bad_;
    uint32_t volt_bad_ = 0;
    // auto-recovery: was the last observation fully healthy (with
    // hysteresis), and since when (monotonic ms) has it stayed that way?
    bool observed_healthy_ = false;
    std::optional<uint64_t> healthy_since_ms_;
};

}  // namespace rf
