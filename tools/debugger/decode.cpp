#include "decode.h"

#include <cmath>
#include <limits>

namespace rf::dbg {

const char* severity_word(Severity s) {
    switch (s) {
        case Severity::Ok: return "ok";
        case Severity::Notice: return "notice";
        case Severity::Warning: return "warn";
        case Severity::Fault: return "FAULT";
    }
    return "warn";
}

namespace {

constexpr CodeInfo kFaults[] = {
    {0, "success", "no fault latched", Severity::Ok, "", false},

    // 1-7: peripheral/transport faults inside the controller. Rare in the
    // field; they mean the board's own DMA or UART misbehaved.
    {1, "dma_stream_transfer", "internal DMA transfer error",
     Severity::Fault, "power-cycle the controller; if it repeats, the board is suspect", false},
    {2, "dma_stream_fifo", "internal DMA FIFO error",
     Severity::Fault, "power-cycle the controller; if it repeats, the board is suspect", false},
    {3, "uart_overrun", "diagnostic UART overrun", Severity::Fault, "", false},
    {4, "uart_framing", "diagnostic UART framing error", Severity::Fault, "", false},
    {5, "uart_noise", "diagnostic UART noise", Severity::Fault, "", false},
    {6, "uart_buffer_overrun", "diagnostic UART buffer overrun", Severity::Fault, "", false},
    {7, "uart_parity", "diagnostic UART parity error", Severity::Fault, "", false},

    // 32-50: the operational faults.
    {32, "calibration", "stored motor calibration is missing or invalid",
     Severity::Fault, "run `debugger calibrate --id N` on this controller", false},
    {33, "motor_driver", "DRV gate driver reported a fault",
     Severity::Fault, "check phase wiring and supply; NOT the bus under-voltage fault (that is 40)", false},
    {34, "over_voltage", "bus voltage exceeded servo.max_voltage",
     Severity::Fault, "regeneration into a pack that cannot sink it; reduce accel_limit before retrying", false},
    {35, "encoder", "encoder read failed or is implausible",
     Severity::Fault, "check the encoder cable and magnet seating", false},
    {36, "motor_not_configured", "motor.poles == 0: never calibrated",
     Severity::Fault, "expected on a fresh or config-wiped controller; calibrate, then restore config", false},
    {37, "pwm_cycle_overrun", "control ISR overran its PWM cycle",
     Severity::Fault, "firmware-level timing violation; report it", false},
    {38, "over_temperature", "board temperature exceeded servo.fault_temperature",
     Severity::Fault, "let it cool; check airflow and whether a wheel is binding", false},
    {39, "start_outside_limit", "commanded start position outside servopos bounds",
     Severity::Fault, "servopos.position_min/max default to +-0.01 rev; a wheel must have them NaN", false},
    {40, "under_voltage", "bus voltage below the hard 4.0 V floor",
     Severity::Fault, "this is the bus under-voltage fault; the pack limit is a separate supervisor trip", false},
    {41, "config_changed", "configuration changed while running",
     Severity::Warning, "something wrote config mid-operation; re-check with `debugger config diff`", false},
    {42, "theta_invalid", "commutation angle invalid",
     Severity::Fault, "encoder or commutation source misconfigured", false},
    {43, "position_invalid", "output position invalid",
     Severity::Fault, "check motor_position.output.source configuration", false},
    {44, "driver_enable", "gate driver failed to enable",
     Severity::Fault, "power and driver-enable line", false},
    {45, "stop_position_deprecated", "deprecated stop_position used",
     Severity::Warning, "a client sent a deprecated field", false},
    {46, "timing_violation", "internal timing violation", Severity::Fault, "", false},
    {47, "bemf_ff_no_accel_limit", "BEMF feedforward enabled without an accel limit",
     Severity::Fault, "set an accel_limit or disable bemf_feedforward", false},
    {48, "invalid_limits", "configured limits are inconsistent",
     Severity::Fault, "check velocity/accel/position limit configuration", false},
    {49, "position_control_error", "position tracking error exceeded its bound",
     Severity::Fault, "a stalled or obstructed wheel is the usual cause", false},
    {50, "velocity_control_error", "velocity tracking error exceeded its bound",
     Severity::Fault, "a stalled or obstructed wheel is the usual cause", false},

    // 96-105: live limit reasons. NOT faults. The controller is still
    // running; something is capping its output right now.
    {96, "limit_max_velocity", "output capped by servo.max_velocity",
     Severity::Notice, "current derating starts here; normal near top speed", true},
    {97, "limit_max_power", "output capped by servo.max_power_W",
     Severity::Notice, "", true},
    {98, "limit_max_voltage", "output capped by the voltage ceiling",
     Severity::Notice, "", true},
    {99, "limit_max_current", "output capped by servo.max_current_A",
     Severity::Notice, "seen during ordinary driving means the cap is below real demand", true},
    {100, "limit_fet_temperature", "output derated by board temperature",
     Severity::Warning, "thermal derating is active; check airflow and load", true},
    {101, "limit_motor_temperature", "output derated by motor temperature",
     Severity::Warning, "requires a motor thermistor to be fitted and configured", true},
    {102, "limit_max_torque", "output capped by the commanded maximum_torque",
     Severity::Notice, "this is our own per-command cap, not a board limit", true},
    {103, "limit_position_bounds", "output capped by servopos position bounds",
     Severity::Warning, "a continuously-rotating wheel needs position_min/max = NaN", true},
    {104, "limit_flux_braking", "flux braking is dissipating energy",
     Severity::Notice, "voltage-triggered; expected during hard deceleration", true},
    {105, "limit_field_weakening", "field weakening is active",
     Severity::Notice, "expected above base speed", true},
};

constexpr CodeInfo kModes[] = {
    {0, "stopped", "de-energized, holding nothing", Severity::Ok, "", false},
    {1, "fault", "latched fault; see the fault code", Severity::Fault,
     "decode the fault register to find out why", false},
    {2, "enabling", "powering up the driver", Severity::Ok, "", false},
    {3, "calibrating", "running calibration", Severity::Notice,
     "do not interrupt; do not command motion", false},
    {4, "calibration_complete", "calibration finished", Severity::Ok, "", false},
    {5, "pwm", "raw PWM mode", Severity::Notice, "", false},
    {6, "voltage", "raw voltage mode", Severity::Notice, "", false},
    {7, "voltage_foc", "voltage FOC mode", Severity::Notice, "", false},
    {8, "voltage_dq", "voltage DQ mode", Severity::Notice, "", false},
    {9, "current", "current mode", Severity::Ok, "", false},
    {10, "position", "position/velocity mode — the normal driving mode",
     Severity::Ok, "", false},
    {11, "position_timeout", "the per-command watchdog expired",
     Severity::Warning,
     "no command frame arrived in time: the host loop stalled, or CAN dropped", false},
    {12, "zero_velocity", "actively damping to zero velocity", Severity::Ok, "", false},
    {13, "stay_within", "holding inside a position band", Severity::Ok, "", false},
    {14, "measure_inductance", "measuring inductance", Severity::Notice, "", false},
    {15, "brake", "phase-short brake", Severity::Ok, "", false},
};

}  // namespace

CodeInfo decode_fault(int code) {
    for (const auto& e : kFaults) {
        if (e.code == code) return e;
    }
    CodeInfo unknown;
    unknown.code = code;
    unknown.name = "unknown";
    unknown.meaning = "code not present in this firmware's error table";
    // Deliberately not Ok: an unrecognised code must never read as healthy.
    unknown.severity = Severity::Warning;
    unknown.action = "check the controller's firmware version against this build";
    unknown.is_limit = code >= 96;
    return unknown;
}

CodeInfo decode_mode(int mode) {
    for (const auto& e : kModes) {
        if (e.code == mode) return e;
    }
    CodeInfo unknown;
    unknown.code = mode;
    unknown.name = "invalid";
    unknown.meaning = "mode outside the known enum";
    unknown.severity = Severity::Warning;
    return unknown;
}

bool mode_is_energized(int mode) {
    switch (mode) {
        case 5:   // pwm
        case 6:   // voltage
        case 7:   // voltage_foc
        case 8:   // voltage_dq
        case 9:   // current
        case 10:  // position
        case 12:  // zero_velocity
        case 13:  // stay_within
        case 15:  // brake
            return true;
        default:
            return false;
    }
}

double torque_constant_nm_per_a(double motor_kv) {
    if (!std::isfinite(motor_kv) || motor_kv <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return kMoteusTorqueFactor / motor_kv;
}

}  // namespace rf::dbg
