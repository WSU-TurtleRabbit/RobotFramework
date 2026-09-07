// Decode tables for moteus status words — pure, no I/O, no hardware.
//
// Kept free of pi3hat/moteus transport headers so the host test suite can
// verify every table without a Pi. Everything here is sourced from the
// firmware itself, not from documentation:
//   fault + limit codes  moteus/fw/error.h
//   modes                mjbots/moteus/moteus_protocol.h  enum class Mode
//
// The distinction this file exists to enforce: codes >= 96 in register 0x00f
// are NOT hard faults. They are "something is limiting my output right now"
// notices reported while a control mode is running. Treating a flux-braking
// notice as a fault, or a real over-voltage as a notice, are both wrong and
// both easy to do by accident.
#pragma once

#include <string>

namespace rf::dbg {

// How a status code should be presented, and how urgently.
enum class Severity {
    Ok,       // nothing wrong
    Notice,   // output is being limited; expected during hard driving
    Warning,  // recoverable, but someone should look
    Fault,    // latched hard fault; the controller has stopped
};

const char* severity_word(Severity s);

struct CodeInfo {
    int code = 0;
    const char* name = "unknown";
    // What it means in this robot's terms, not a restatement of the name.
    const char* meaning = "unrecognised code";
    Severity severity = Severity::Warning;
    // The next diagnostic step, empty when there is nothing generic to say.
    const char* action = "";
    // True for 96..105: a live limit reason, not a latched fault.
    bool is_limit = false;
};

// Decode register 0x00f. Accepts hard faults (1-7, 32-50), limit reasons
// (96-105), and 0 (success). Unknown codes come back named "unknown" with
// Severity::Warning rather than being silently reported as healthy.
CodeInfo decode_fault(int code);

// Decode register 0x000. Values outside the enum return "invalid".
CodeInfo decode_mode(int mode);

// True when the mode means the controller is actively driving the motor.
// kPosition, kCurrent, kVoltage*, kPwm, kStayWithin, kZeroVelocity, kBrake.
bool mode_is_energized(int mode);

// moteus computes torque as kTorqueFactor / motor.Kv, where
// kTorqueFactor = (3/2) * (1/sqrt(3)) * (60 / 2*pi) ~ 8.2699.
//
// The generic motor-catalog relation Kt = 9.5493 / KV is a DIFFERENT
// convention and overstates torque by exactly 2/sqrt(3) ~ 15.47%. Anything
// comparing a measured q-current against a predicted torque must use this
// one, or it will read a healthy wheel as under-performing.
inline constexpr double kMoteusTorqueFactor = 8.2699;

// Estimated output torque constant, N-m/A, from a controller's calibrated Kv.
// Returns NaN for a non-finite or non-positive Kv rather than inventing one.
double torque_constant_nm_per_a(double motor_kv);

}  // namespace rf::dbg
