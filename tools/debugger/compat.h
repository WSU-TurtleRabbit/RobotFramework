// Compatibility shim: one debugger source tree against two generations of
// Telemetry/Telemetry.h.
//
// The tool was written against a newer Telemetry API than some robots run.
// The two shapes this header bridges, verified by reading both trees:
//
//   OLD (e.g. the robot at commit 86a7f05, branch main):
//     struct MotorTelemetry { double temperature, voltage, velocity, position,
//                             current; int mode, fault; };
//     Telemetry::cycle(velocity_map, energize, ff_torque_nm)
//     rf::robot_query_format() requests q_current only.
//
//   NEW (the authoring tree):
//     MotorTelemetry additionally has `double torque` and `double d_current`,
//     every numeric field is NaN-initialised and mode/fault default to -1.
//     Telemetry::cycle(velocity_map, energize, ff_torque_nm, max_torque_nm)
//     rf::robot_query_format() also requests d_current.
//
// Rather than fork main.cpp or drop the d-current/torque columns, the
// accessors below detect at compile time whether MotorTelemetry carries a
// field and return NaN when it does not. NaN is what the rest of the tool
// already treats as "missing" (term.h num() renders it as "--"), so on an OLD
// tree the columns show "--" and on a NEW tree they show real values, with
// no #ifdef and no per-tree build flag.
//
// The cycle() signature needs no shim: the debugger only ever calls the
// two-argument form `cycle(map, energize)`, which both generations accept
// because the trailing parameters are defaulted. If a future subcommand needs
// max_torque_nm, add a `has_max_torque_param<Telemetry>` detection here in
// the same style rather than calling the four-argument form directly.
#pragma once

#include <limits>
#include <type_traits>
#include <utility>

namespace rf::dbg::compat {

// Detection idiom (std::void_t). `has_d_current<T>::value` is true iff
// `std::declval<const T&>().d_current` is a well-formed expression.
template <typename T, typename = void>
struct has_d_current : std::false_type {};
template <typename T>
struct has_d_current<T, std::void_t<decltype(std::declval<const T&>().d_current)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_torque : std::false_type {};
template <typename T>
struct has_torque<T, std::void_t<decltype(std::declval<const T&>().torque)>>
    : std::true_type {};

// d-axis phase current, A. NaN when this Telemetry build does not report it.
template <typename T>
double d_current_of(const T& m) {
    if constexpr (has_d_current<T>::value) {
        return static_cast<double>(m.d_current);
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

// Estimated output torque, N-m. NaN when this Telemetry build does not report it.
template <typename T>
double torque_of(const T& m) {
    if constexpr (has_torque<T>::value) {
        return static_cast<double>(m.torque);
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
}

// A short footer note so an operator reading "--" in the d-cur/torque columns
// knows it is the build, not the wheel, that is silent. Empty on a NEW tree.
template <typename T>
constexpr const char* missing_fields_note() {
    if constexpr (has_d_current<T>::value && has_torque<T>::value) {
        return "";
    } else if constexpr (!has_d_current<T>::value && !has_torque<T>::value) {
        return "  d-cur/torque: not reported by this Telemetry build.";
    } else if constexpr (!has_d_current<T>::value) {
        return "  d-cur: not reported by this Telemetry build.";
    } else {
        return "  torque: not reported by this Telemetry build.";
    }
}

}  // namespace rf::dbg::compat
