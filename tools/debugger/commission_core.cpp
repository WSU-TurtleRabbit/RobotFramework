#include "commission_core.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <numbers>
#include <sstream>

namespace rf::dbg::commission {

namespace {

std::string fmt(const char* f, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, f, v);
    return buf;
}

bool positive_finite(double v) { return std::isfinite(v) && v > 0.0; }

std::string trim(const std::string& s) {
    std::size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

bool all_hex(const std::string& s) {
    if (s.empty()) return false;
    for (char c : s) if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Ceiling
// ---------------------------------------------------------------------------

Ceiling derive_ceiling(const CeilingInputs& in) {
    Ceiling out;
    auto& d = out.derivation;

    struct Candidate {
        double value;
        std::string source;
    };
    std::vector<Candidate> vel, acc;

    d.push_back("ceiling = " + fmt("%.0f", in.fraction * 100.0) +
                "% of the smallest readable backstop, per quantity");
    d.push_back("Motor.yaml velocityLimit=" + fmt("%.2f", in.yaml_velocity_limit_rev_s) +
                " rev/s  accelLimit=" + fmt("%.2f", in.yaml_accel_limit_rev_s2) +
                " rev/s^2  (sent per command by the runtime; stands in where a "
                "controller has no limit of its own)");

    if (in.controllers.empty()) {
        d.push_back("no controllers reported; using Motor.yaml alone");
    }

    for (const auto& c : in.controllers) {
        const std::string who = "controller " + std::to_string(c.id);
        if (!c.reachable) {
            d.push_back(who + ": diagnostic channel did not answer; Motor.yaml stands in");
        }
        // velocity
        if (positive_finite(c.default_velocity_limit_rev_s)) {
            vel.push_back({c.default_velocity_limit_rev_s,
                           who + " servo.default_velocity_limit " +
                               fmt("%.2f", c.default_velocity_limit_rev_s) + " rev/s"});
            d.push_back(who + ": servo.default_velocity_limit = " +
                        fmt("%.2f", c.default_velocity_limit_rev_s) + " rev/s (live)");
        } else {
            if (c.reachable) {
                d.push_back(who + ": servo.default_velocity_limit = nan (no controller-side "
                            "velocity limit); Motor.yaml velocityLimit stands in");
            }
            if (positive_finite(in.yaml_velocity_limit_rev_s)) {
                vel.push_back({in.yaml_velocity_limit_rev_s,
                               "Motor.yaml velocityLimit " +
                                   fmt("%.2f", in.yaml_velocity_limit_rev_s) +
                                   " rev/s (standing in for " + who + ")"});
            }
        }
        if (positive_finite(c.max_velocity_rev_s)) {
            vel.push_back({c.max_velocity_rev_s,
                           who + " servo.max_velocity " + fmt("%.2f", c.max_velocity_rev_s) +
                               " rev/s"});
            d.push_back(who + ": servo.max_velocity = " + fmt("%.2f", c.max_velocity_rev_s) +
                        " rev/s (derate onset; also a candidate)");
        }
        // accel
        if (positive_finite(c.default_accel_limit_rev_s2)) {
            acc.push_back({c.default_accel_limit_rev_s2,
                           who + " servo.default_accel_limit " +
                               fmt("%.2f", c.default_accel_limit_rev_s2) + " rev/s^2"});
            d.push_back(who + ": servo.default_accel_limit = " +
                        fmt("%.2f", c.default_accel_limit_rev_s2) + " rev/s^2 (live)");
        } else {
            if (c.reachable) {
                d.push_back(who + ": servo.default_accel_limit = nan (no controller-side "
                            "accel limit); Motor.yaml accelLimit stands in");
            }
            if (positive_finite(in.yaml_accel_limit_rev_s2)) {
                acc.push_back({in.yaml_accel_limit_rev_s2,
                               "Motor.yaml accelLimit " +
                                   fmt("%.2f", in.yaml_accel_limit_rev_s2) +
                                   " rev/s^2 (standing in for " + who + ")"});
            }
        }
    }
    if (in.controllers.empty()) {
        if (positive_finite(in.yaml_velocity_limit_rev_s)) {
            vel.push_back({in.yaml_velocity_limit_rev_s,
                           "Motor.yaml velocityLimit " +
                               fmt("%.2f", in.yaml_velocity_limit_rev_s) + " rev/s"});
        }
        if (positive_finite(in.yaml_accel_limit_rev_s2)) {
            acc.push_back({in.yaml_accel_limit_rev_s2,
                           "Motor.yaml accelLimit " +
                               fmt("%.2f", in.yaml_accel_limit_rev_s2) + " rev/s^2"});
        }
    }

    if (!(in.fraction > 0.0 && in.fraction <= 1.0)) {
        out.error = "ceiling fraction must be in (0, 1]";
        d.push_back("REFUSED: " + out.error);
        return out;
    }
    if (vel.empty()) {
        out.error = "no readable velocity backstop: every controller reports no limit and "
                    "Motor.yaml has no velocityLimit";
        d.push_back("REFUSED: " + out.error);
        return out;
    }
    if (acc.empty()) {
        out.error = "no readable accel backstop: every controller reports no limit and "
                    "Motor.yaml has no accelLimit";
        d.push_back("REFUSED: " + out.error);
        return out;
    }

    const auto vmin = std::min_element(vel.begin(), vel.end(),
                                       [](const Candidate& a, const Candidate& b) {
                                           return a.value < b.value;
                                       });
    const auto amin = std::min_element(acc.begin(), acc.end(),
                                       [](const Candidate& a, const Candidate& b) {
                                           return a.value < b.value;
                                       });
    out.velocity_limit_rev_s = in.fraction * vmin->value;
    out.accel_limit_rev_s2 = in.fraction * amin->value;
    out.velocity_source = vmin->source;
    out.accel_source = amin->source;
    d.push_back("velocity ceiling = " + fmt("%.2f", in.fraction) + " x " +
                fmt("%.2f", vmin->value) + " = " + fmt("%.3f", out.velocity_limit_rev_s) +
                " rev/s   [bound by " + vmin->source + "]");
    d.push_back("accel ceiling    = " + fmt("%.2f", in.fraction) + " x " +
                fmt("%.2f", amin->value) + " = " + fmt("%.3f", out.accel_limit_rev_s2) +
                " rev/s^2 [bound by " + amin->source + "]");
    out.ok = true;
    return out;
}

BodyCaps derive_body_caps(const Kinematics& kin, double wheel_ceiling_rev_s,
                          double envelope_linear_mps, double envelope_angular_radps) {
    BodyCaps out;
    if (!positive_finite(wheel_ceiling_rev_s)) return out;

    // Worst-case wheel demand for a 1 m/s translation, over every direction.
    // Goes through inverse() so the per-wheel tracking scales, the lateral
    // scale and the yaw feed-forward coupling are all included.
    double worst = 0.0;
    constexpr int kSteps = 360;
    for (int i = 0; i < kSteps; ++i) {
        const double a = 2.0 * std::numbers::pi * i / kSteps;
        const double g = kin.peak_motor_rev_s(BodyTwist{std::cos(a), std::sin(a), 0.0});
        worst = std::max(worst, g);
    }
    const double gw = std::max(kin.peak_motor_rev_s(BodyTwist{0.0, 0.0, 1.0}),
                               kin.peak_motor_rev_s(BodyTwist{0.0, 0.0, -1.0}));
    out.worst_linear_gain_rev_s_per_mps = worst;
    out.angular_gain_rev_s_per_radps = gw;

    if (worst > 0.0) {
        const double from_wheels = wheel_ceiling_rev_s / worst;
        if (positive_finite(envelope_linear_mps) && envelope_linear_mps < from_wheels) {
            out.linear_mps = envelope_linear_mps;
            out.linear_source = "Safety.yaml envelope.cappedLinear " +
                                fmt("%.2f", envelope_linear_mps) + " m/s";
        } else {
            out.linear_mps = from_wheels;
            out.linear_source = "wheel ceiling " + fmt("%.3f", wheel_ceiling_rev_s) +
                                " rev/s / worst-direction gain " + fmt("%.3f", worst) +
                                " rev/s per m/s";
        }
    }
    if (gw > 0.0) {
        const double from_wheels = wheel_ceiling_rev_s / gw;
        if (positive_finite(envelope_angular_radps) && envelope_angular_radps < from_wheels) {
            out.angular_radps = envelope_angular_radps;
            out.angular_source = "Safety.yaml envelope.cappedAngular " +
                                 fmt("%.2f", envelope_angular_radps) + " rad/s";
        } else {
            out.angular_radps = from_wheels;
            out.angular_source = "wheel ceiling " + fmt("%.3f", wheel_ceiling_rev_s) +
                                 " rev/s / yaw gain " + fmt("%.3f", gw) + " rev/s per rad/s";
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

const char* setting_name(Setting s) {
    switch (s) {
        case Setting::BodyVel: return "body_vel_mps";
        case Setting::BodyOmega: return "body_omega_radps";
        case Setting::VelocityLimit: return "velocity_limit_rev_s";
        case Setting::AccelLimit: return "accel_limit_rev_s2";
    }
    return "?";
}

const char* setting_label(Setting s) {
    switch (s) {
        case Setting::BodyVel: return "body speed";
        case Setting::BodyOmega: return "body yaw rate";
        case Setting::VelocityLimit: return "moteus velocity_limit";
        case Setting::AccelLimit: return "moteus accel_limit";
    }
    return "?";
}

const char* setting_unit(Setting s) {
    switch (s) {
        case Setting::BodyVel: return "m/s";
        case Setting::BodyOmega: return "rad/s";
        case Setting::VelocityLimit: return "rev/s";
        case Setting::AccelLimit: return "rev/s^2";
    }
    return "";
}

double setting_step(Setting s) {
    switch (s) {
        case Setting::BodyVel: return 0.10;
        case Setting::BodyOmega: return 0.25;
        case Setting::VelocityLimit: return 0.50;
        case Setting::AccelLimit: return 1.00;
    }
    return 0.0;
}

double setting_floor(Setting s) {
    // One step: a setting of exactly zero would make a key press a no-op and
    // is not a meaningful thing to sign off on.
    return setting_step(s);
}

double get_setting(const Settings& s, Setting which) {
    switch (which) {
        case Setting::BodyVel: return s.body_vel_mps;
        case Setting::BodyOmega: return s.body_omega_radps;
        case Setting::VelocityLimit: return s.velocity_limit_rev_s;
        case Setting::AccelLimit: return s.accel_limit_rev_s2;
    }
    return 0.0;
}

double get_cap(const Caps& c, Setting which) {
    switch (which) {
        case Setting::BodyVel: return c.body_vel_mps;
        case Setting::BodyOmega: return c.body_omega_radps;
        case Setting::VelocityLimit: return c.velocity_limit_rev_s;
        case Setting::AccelLimit: return c.accel_limit_rev_s2;
    }
    return 0.0;
}

namespace {
double& setting_ref(Settings& s, Setting which) {
    switch (which) {
        case Setting::BodyVel: return s.body_vel_mps;
        case Setting::BodyOmega: return s.body_omega_radps;
        case Setting::VelocityLimit: return s.velocity_limit_rev_s;
        case Setting::AccelLimit: return s.accel_limit_rev_s2;
    }
    return s.body_vel_mps;
}

double& cap_ref(Caps& c, Setting which) {
    switch (which) {
        case Setting::BodyVel: return c.body_vel_mps;
        case Setting::BodyOmega: return c.body_omega_radps;
        case Setting::VelocityLimit: return c.velocity_limit_rev_s;
        case Setting::AccelLimit: return c.accel_limit_rev_s2;
    }
    return c.body_vel_mps;
}

// Snap to the step grid so repeated +/- never accumulates binary drift
// (0.1 + 0.1 + 0.1 != 0.3) and the screen never shows 0.30000000004. The
// grid is k / (1/step): every step here has an exactly representable
// reciprocal (10, 4, 2, 1), and k/10 is the same double as the literal 0.3,
// whereas 3 * 0.1 is not.
double snap(double v, double step) {
    const double inv = std::round(1.0 / step);
    return std::round(v * inv) / inv;
}
}  // namespace

StepResult step_setting(Settings& s, Setting which, int direction, const Caps& caps) {
    StepResult r;
    double& v = setting_ref(s, which);
    r.before = v;
    const double cap = get_cap(caps, which);
    const double floor = setting_floor(which);
    const double step = setting_step(which);
    if (!std::isfinite(cap) || cap <= 0.0) {
        // No cap means no permission. Refuse to move at all.
        r.after = v;
        r.at_ceiling = true;
        return r;
    }
    double want = snap(v + (direction > 0 ? step : -step), step);
    if (want > cap + 1e-9) {
        want = cap;
        r.at_ceiling = true;
    }
    if (want < floor - 1e-9) {
        want = std::min(floor, cap);
        r.at_floor = true;
    }
    if (direction > 0 && std::abs(v - cap) < 1e-9) r.at_ceiling = true;
    r.changed = std::abs(want - v) > 1e-9;
    v = want;
    r.after = v;
    return r;
}

void clamp_settings(Settings& s, const Caps& caps) {
    for (Setting which : {Setting::BodyVel, Setting::BodyOmega, Setting::VelocityLimit,
                          Setting::AccelLimit}) {
        double& v = setting_ref(s, which);
        const double cap = get_cap(caps, which);
        if (!std::isfinite(v)) v = 0.0;
        if (std::isfinite(cap)) v = std::min(v, cap);
        v = std::max(v, std::min(setting_floor(which), std::isfinite(cap) ? cap : v));
    }
}

WheelClamp clamp_wheels(const std::array<double, 4>& demand, double cap_rev_s) {
    WheelClamp r;
    r.wheels = demand;
    for (double w : demand) {
        if (std::isfinite(w)) r.peak_before = std::max(r.peak_before, std::abs(w));
    }
    if (!(cap_rev_s > 0.0) || !std::isfinite(cap_rev_s)) {
        // No cap: command nothing. Refusing is the safe direction.
        r.wheels = {{0.0, 0.0, 0.0, 0.0}};
        r.factor = 0.0;
        r.clipped = r.peak_before > 0.0;
        return r;
    }
    if (r.peak_before > cap_rev_s) {
        r.factor = cap_rev_s / r.peak_before;
        for (double& w : r.wheels) w *= r.factor;
        r.clipped = true;
    }
    return r;
}

// ---------------------------------------------------------------------------
// Operator-settable ceiling
// ---------------------------------------------------------------------------

OverrideStatus override_status(const Caps& in_force, const Caps& derived) {
    OverrideStatus out;
    for (Setting which : kAllSettings) {
        auto& it = out.items[static_cast<int>(which)];
        it.in_force = get_cap(in_force, which);
        it.derived = get_cap(derived, which);
        const bool have_force = std::isfinite(it.in_force);
        const bool have_derived = positive_finite(it.derived);
        if (have_force && have_derived) {
            it.manual = std::abs(it.in_force - it.derived) > 1e-9;
            it.above = it.in_force > it.derived + 1e-9;
            it.ratio = it.in_force / it.derived;
        } else if (have_force && !have_derived) {
            // A value in force with nothing derived to measure it against:
            // that is an override by definition, and the ratio is unknown.
            it.manual = true;
            it.above = true;
        } else {
            it.manual = false;
            it.above = false;
        }
        out.any_manual = out.any_manual || it.manual;
        out.any_override = out.any_override || it.above;
    }
    return out;
}

CeilingProposal propose_ceiling(Setting which, double value, const Caps& derived) {
    CeilingProposal p;
    p.which = which;
    p.value = value;
    p.derived = get_cap(derived, which);
    if (!std::isfinite(value)) {
        p.error = "not a finite number";
        return p;
    }
    if (!(value > 0.0)) {
        p.error = "a ceiling must be greater than zero (zero would command nothing)";
        return p;
    }
    p.valid = true;
    if (positive_finite(p.derived)) {
        p.above_derived = value > p.derived + 1e-9;
        p.ratio = value / p.derived;
    } else {
        p.above_derived = true;  // nothing to be inside of
    }
    return p;
}

bool parse_positive_double(const std::string& text, double* out, std::string* error) {
    const std::string t = trim(text);
    if (t.empty()) {
        if (error) *error = "empty";
        return false;
    }
    // strtod would happily accept "nan", "inf", "0x1p3" and "25x" (stopping
    // early); insist on a plain decimal/exponent number and nothing else.
    for (char c : t) {
        if (!(std::isdigit(static_cast<unsigned char>(c)) || c == '.' || c == 'e' || c == 'E' ||
              c == '+' || c == '-')) {
            if (error) *error = "not a number: '" + t + "'";
            return false;
        }
    }
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || *end != '\0') {
        if (error) *error = "not a number: '" + t + "'";
        return false;
    }
    if (!std::isfinite(v)) {
        if (error) *error = "not a finite number: '" + t + "'";
        return false;
    }
    if (!(v > 0.0)) {
        if (error) *error = "must be greater than zero: '" + t + "'";
        return false;
    }
    *out = v;
    return true;
}

CeilingProposal propose_ceiling_text(Setting which, const std::string& text, const Caps& derived) {
    double v = kNaN;
    std::string err;
    if (!parse_positive_double(text, &v, &err)) {
        CeilingProposal p;
        p.which = which;
        p.derived = get_cap(derived, which);
        p.error = err;
        return p;
    }
    return propose_ceiling(which, v, derived);
}

CeilingChange apply_ceiling(Caps& in_force, Settings& settings, Setting which, double value) {
    CeilingChange r;
    r.which = which;
    double& cap = cap_ref(in_force, which);
    r.before = cap;
    r.after = cap;
    double& setting = setting_ref(settings, which);
    r.setting_before = setting;
    r.setting_after = setting;
    if (!positive_finite(value)) return r;  // ignored, never clamped into range
    // Deliberately no upper bound. The caller has already warned and
    // confirmed if this is above the derived ceiling.
    cap = value;
    r.after = cap;
    r.changed = !std::isfinite(r.before) || std::abs(r.after - r.before) > 1e-9;
    if (std::isfinite(setting) && setting > cap + 1e-9) {
        setting = cap;
        r.setting_clamped = true;
    }
    r.setting_after = setting;
    return r;
}

bool parse_ceiling_spec(const std::string& spec, std::vector<std::pair<Setting, double>>* out,
                        std::string* error) {
    out->clear();
    std::string item;
    auto flush = [&]() -> bool {
        const std::string t = trim(item);
        item.clear();
        if (t.empty()) return true;
        const auto eq = t.find('=');
        if (eq == std::string::npos) {
            if (error) *error = "expected name=value, got '" + t + "'";
            return false;
        }
        const std::string name = trim(t.substr(0, eq));
        Setting which = Setting::VelocityLimit;
        if (name == "velocity") which = Setting::VelocityLimit;
        else if (name == "accel") which = Setting::AccelLimit;
        else if (name == "body") which = Setting::BodyVel;
        else if (name == "yaw") which = Setting::BodyOmega;
        else {
            if (error) *error = "unknown ceiling '" + name + "' (velocity | accel | body | yaw)";
            return false;
        }
        double v = kNaN;
        std::string err;
        if (!parse_positive_double(t.substr(eq + 1), &v, &err)) {
            if (error) *error = name + ": " + err;
            return false;
        }
        out->emplace_back(which, v);
        return true;
    };
    for (char c : spec) {
        if (c == ',') {
            if (!flush()) return false;
        } else {
            item += c;
        }
    }
    if (!flush()) return false;
    if (out->empty()) {
        if (error) *error = "no ceilings given";
        return false;
    }
    return true;
}

std::string describe_max_velocity(const std::vector<BackstopReading>& controllers) {
    // Group ids by their max_velocity so "all the same" reads as one number.
    std::map<std::string, std::vector<int>> by_value;
    std::vector<int> unreadable;
    for (const auto& c : controllers) {
        if (positive_finite(c.max_velocity_rev_s)) {
            by_value[fmt("%.2f", c.max_velocity_rev_s)].push_back(c.id);
        } else {
            unreadable.push_back(c.id);
        }
    }
    if (by_value.empty()) return "unreadable on every controller";
    auto ids = [](const std::vector<int>& v) {
        std::string s;
        for (std::size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + std::to_string(v[i]);
        return s;
    };
    if (by_value.size() == 1 && unreadable.empty()) {
        const auto& [val, v] = *by_value.begin();
        if (v.size() == 4) return val + " rev/s on all four controllers";
        return val + " rev/s on controller" + (v.size() > 1 ? "s " : " ") + ids(v);
    }
    std::string out;
    for (const auto& [val, v] : by_value) {
        if (!out.empty()) out += "; ";
        out += val + " rev/s on controller" + (v.size() > 1 ? "s " : " ") + ids(v);
    }
    if (!unreadable.empty()) {
        out += "; unreadable on controller" + std::string(unreadable.size() > 1 ? "s " : " ") +
               ids(unreadable);
    }
    return out;
}

// ---------------------------------------------------------------------------
// JSON encoding
// ---------------------------------------------------------------------------

std::string json_string(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (const unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            default:
                if (c < 0x20 || c == 0x7f) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    out += '"';
    return out;
}

std::string json_number(double v) {
    if (!std::isfinite(v)) return "null";
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.9g", v);
    return buf;
}

std::string json_int(int64_t v) { return std::to_string(v); }

std::string json_bool(bool v) { return v ? "true" : "false"; }

std::string json_string_array(const std::vector<std::string>& v) {
    std::string out = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) out += ',';
        out += json_string(v[i]);
    }
    return out + "]";
}

std::string json_int_array(const std::vector<int>& v) {
    std::string out = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i) out += ',';
        out += std::to_string(v[i]);
    }
    return out + "]";
}

std::string iso8601_utc(double unix_s) {
    if (!std::isfinite(unix_s)) return "";
    const double whole = std::floor(unix_s);
    const int ms = static_cast<int>(std::floor((unix_s - whole) * 1000.0 + 0.5)) % 1000;
    const std::time_t t = static_cast<std::time_t>(whole);
    std::tm tm{};
#ifdef _WIN32
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[40];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", tm.tm_year + 1900,
                  tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
    return buf;
}

namespace {

void twist_json(std::string& o, const BodyTwist& t) {
    o += '[';
    o += json_number(t.vx);
    o += ',';
    o += json_number(t.vy);
    o += ',';
    o += json_number(t.w);
    o += ']';
}

void settings_json(std::string& o, const Settings& s) {
    o += "{\"body_vel_mps\":";
    o += json_number(s.body_vel_mps);
    o += ",\"body_omega_radps\":";
    o += json_number(s.body_omega_radps);
    o += ",\"velocity_limit_rev_s\":";
    o += json_number(s.velocity_limit_rev_s);
    o += ",\"accel_limit_rev_s2\":";
    o += json_number(s.accel_limit_rev_s2);
    o += '}';
}

void caps_json(std::string& o, const Caps& c) {
    o += "{\"body_vel_mps\":";
    o += json_number(c.body_vel_mps);
    o += ",\"body_omega_radps\":";
    o += json_number(c.body_omega_radps);
    o += ",\"velocity_limit_rev_s\":";
    o += json_number(c.velocity_limit_rev_s);
    o += ",\"accel_limit_rev_s2\":";
    o += json_number(c.accel_limit_rev_s2);
    o += '}';
}

}  // namespace

std::string encode_cycle(const CycleRecord& r) {
    std::string o;
    o.reserve(1400);
    o += "{\"type\":\"cycle\",\"seq\":";
    o += std::to_string(r.seq);
    o += ",\"t_mono_s\":";
    o += json_number(r.t_mono_s);
    o += ",\"t_wall\":";
    o += json_string(iso8601_utc(r.t_wall_unix_s));
    o += ",\"t_wall_unix_s\":";
    o += json_number(r.t_wall_unix_s);
    o += ",\"dt_s\":";
    o += json_number(r.dt_s);
    o += ",\"state\":";
    o += json_string(r.state);
    o += ",\"energized\":";
    o += json_bool(r.energized);
    o += ",\"hold\":";
    o += json_bool(r.hold);
    if (r.hold_reason[0] != '\0') {
        o += ",\"hold_reason\":";
        o += json_string(r.hold_reason);
    }
    o += ",\"clip\":";
    o += json_string(r.clip);
    o += ",\"deadline_missed\":";
    o += json_bool(r.deadline_missed);
    o += ",\"twist_requested\":";
    twist_json(o, r.twist_requested);
    o += ",\"twist_applied\":";
    twist_json(o, r.twist_applied);
    o += ",\"settings\":";
    settings_json(o, r.settings);
    o += ",\"caps\":";
    caps_json(o, r.caps);
    o += ",\"caps_derived\":";
    caps_json(o, r.caps_derived);
    o += ",\"ceiling_mode\":";
    o += r.ceiling_manual ? "\"manual\"" : "\"derived\"";
    o += ",\"override_active\":";
    o += json_bool(r.override_active);
    o += ",\"wheels\":[";
    for (std::size_t i = 0; i < r.wheels.size(); ++i) {
        const auto& w = r.wheels[i];
        if (i) o += ',';
        o += "{\"id\":";
        o += std::to_string(w.id);
        o += ",\"replied\":";
        o += json_bool(w.replied);
        o += ",\"cmd_rev_s\":";
        o += json_number(w.cmd_rev_s);
        o += ",\"vel_rev_s\":";
        o += json_number(w.replied ? w.velocity_rev_s : kNaN);
        o += ",\"pos_rev\":";
        o += json_number(w.replied ? w.position_rev : kNaN);
        o += ",\"q_current_a\":";
        o += json_number(w.replied ? w.q_current_a : kNaN);
        o += ",\"temp_c\":";
        o += json_number(w.replied ? w.temperature_c : kNaN);
        o += ",\"voltage_v\":";
        o += json_number(w.replied ? w.voltage_v : kNaN);
        o += ",\"mode\":";
        o += w.replied ? std::to_string(w.mode) : "null";
        o += ",\"fault\":";
        o += w.replied ? std::to_string(w.fault) : "null";
        o += '}';
    }
    o += "],\"imu\":{\"present\":";
    o += json_bool(r.imu.present);
    o += ",\"rate_dps\":[";
    o += json_number(r.imu.present ? r.imu.roll_dps : kNaN);
    o += ',';
    o += json_number(r.imu.present ? r.imu.pitch_dps : kNaN);
    o += ',';
    o += json_number(r.imu.present ? r.imu.yaw_dps : kNaN);
    o += "],\"heading_deg\":";
    o += json_number(r.imu.present ? r.imu.heading_deg : kNaN);
    o += ",\"accel_mps2\":[";
    o += json_number(r.imu.present ? r.imu.accel_x_mps2 : kNaN);
    o += ',';
    o += json_number(r.imu.present ? r.imu.accel_y_mps2 : kNaN);
    o += ',';
    o += json_number(r.imu.present ? r.imu.accel_z_mps2 : kNaN);
    o += "]}}";
    return o;
}

std::string encode_event(uint64_t seq, double t_mono_s, double t_wall_unix_s,
                         const std::string& kind, const JsonFields& fields) {
    std::string o = "{\"type\":\"event\",\"seq\":" + std::to_string(seq) +
                    ",\"t_mono_s\":" + json_number(t_mono_s) +
                    ",\"t_wall\":" + json_string(iso8601_utc(t_wall_unix_s)) +
                    ",\"t_wall_unix_s\":" + json_number(t_wall_unix_s) +
                    ",\"event\":" + json_string(kind);
    for (const auto& [k, v] : fields) {
        o += ',';
        o += json_string(k);
        o += ':';
        o += v.empty() ? "null" : v;
    }
    o += '}';
    return o;
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

std::string serial_base64(uint32_t s1, uint32_t s2, uint32_t s3) {
    // 12 bytes big-endian == the 96-bit number (s1<<64 | s2<<32 | s3); standard
    // base64 of a 12-byte string is exactly its 16 base-64 digits, MSB first.
    static const char* kDigits =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    unsigned char bytes[12];
    const uint32_t words[3] = {s1, s2, s3};
    for (int i = 0; i < 3; ++i) {
        bytes[i * 4 + 0] = static_cast<unsigned char>((words[i] >> 24) & 0xff);
        bytes[i * 4 + 1] = static_cast<unsigned char>((words[i] >> 16) & 0xff);
        bytes[i * 4 + 2] = static_cast<unsigned char>((words[i] >> 8) & 0xff);
        bytes[i * 4 + 3] = static_cast<unsigned char>(words[i] & 0xff);
    }
    std::string out;
    out.reserve(16);
    for (int g = 0; g < 4; ++g) {
        const uint32_t n = (static_cast<uint32_t>(bytes[g * 3]) << 16) |
                           (static_cast<uint32_t>(bytes[g * 3 + 1]) << 8) |
                           static_cast<uint32_t>(bytes[g * 3 + 2]);
        out += kDigits[(n >> 18) & 63];
        out += kDigits[(n >> 12) & 63];
        out += kDigits[(n >> 6) & 63];
        out += kDigits[n & 63];
    }
    return out;
}

std::string parse_cpuinfo_serial(const std::string& text) {
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("Serial", 0) != 0) continue;
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        return trim(line.substr(colon + 1));
    }
    return "";
}

DiagValue parse_conf_get_double(const std::string& line) {
    DiagValue r;
    r.raw = trim(line);
    if (r.raw.empty()) return r;
    if (r.raw.rfind("ERR", 0) == 0) return r;
    char* end = nullptr;
    const double v = std::strtod(r.raw.c_str(), &end);
    if (end == r.raw.c_str()) return r;  // nothing numeric
    r.answered = true;
    r.value = v;
    return r;
}

std::map<std::string, std::string> parse_diag_text(const std::string& transcript) {
    std::map<std::string, std::string> kv;
    std::string line;
    auto flush = [&] {
        const std::string t = trim(line);
        line.clear();
        if (t.empty() || t == "OK") return;
        const auto sp = t.find_first_of(" \t");
        if (sp == std::string::npos) return;
        kv[t.substr(0, sp)] = trim(t.substr(sp + 1));
    };
    for (char c : transcript) {
        if (c == '\n' || c == '\r') flush();
        else line += c;
    }
    flush();
    return kv;
}

std::string git_hash_from_diag(const std::map<std::string, std::string>& kv) {
    const auto whole = kv.find("git.hash");
    if (whole != kv.end()) {
        std::string h = whole->second;
        if (h.rfind("0x", 0) == 0) h = h.substr(2);
        if (all_hex(h) && h.size() >= 8) {
            std::transform(h.begin(), h.end(), h.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            return h;
        }
    }
    std::map<int, int> bytes;
    for (const auto& [k, v] : kv) {
        if (k.rfind("git.hash.", 0) != 0) continue;
        const int idx = std::atoi(k.c_str() + 9);
        const long b = std::strtol(v.c_str(), nullptr, 0);
        if (idx < 0 || idx > 63 || b < 0 || b > 255) continue;
        bytes[idx] = static_cast<int>(b);
    }
    if (bytes.empty()) return "";
    std::string out;
    for (const auto& [idx, b] : bytes) {
        char buf[4];
        std::snprintf(buf, sizeof buf, "%02x", b);
        out += buf;
    }
    return out;
}

std::string serial_from_firmware_diag(const std::map<std::string, std::string>& kv) {
    uint32_t w[3];
    for (int i = 0; i < 3; ++i) {
        const auto it = kv.find("firmware.serial_number." + std::to_string(i));
        if (it == kv.end()) return "";
        char* end = nullptr;
        const long long v = std::strtoll(it->second.c_str(), &end, 0);
        if (end == it->second.c_str()) return "";
        w[i] = static_cast<uint32_t>(static_cast<int64_t>(v));
    }
    return serial_base64(w[0], w[1], w[2]);
}

// ---------------------------------------------------------------------------
// Profile
// ---------------------------------------------------------------------------

namespace {

std::string controller_json(const ControllerIdentity& c) {
    return "{\"id\":" + std::to_string(c.id) + ",\"bus\":" + std::to_string(c.bus) +
           ",\"serial\":" + json_string(c.serial) + ",\"git_hash\":" + json_string(c.git_hash) +
           ",\"git_dirty\":" + json_bool(c.git_dirty) +
           ",\"git_timestamp\":" + std::to_string(c.git_timestamp) +
           ",\"abi_version\":" + std::to_string(c.abi_version) +
           ",\"register_map\":" + std::to_string(c.register_map) +
           ",\"model\":" + std::to_string(c.model) + ",\"hwrev\":" + json_string(c.hwrev) + "}";
}

}  // namespace

std::string encode_profile(const Profile& p) {
    std::string o;
    o += "{\n";
    o += "  \"type\": \"robot_profile\",\n";
    o += "  \"schema\": " + std::to_string(p.schema) + ",\n";
    o += "  \"tool\": " + json_string(p.tool) + ",\n";
    o += "  \"pi_serial\": " + json_string(p.pi_serial) + ",\n";
    o += "  \"controllers\": [\n";
    for (std::size_t i = 0; i < p.controllers.size(); ++i) {
        o += "    " + controller_json(p.controllers[i]) +
             (i + 1 < p.controllers.size() ? ",\n" : "\n");
    }
    o += "  ],\n";
    o += "  \"approved\": ";
    settings_json(o, p.approved);
    o += ",\n";
    o += "  \"ceiling\": {\n";
    o += "    \"mode\": " + json_string(p.ceiling_mode) + ",\n";
    o += "    \"approved_under_override\": " + json_bool(p.approved_under_override) + ",\n";
    o += "    \"fraction\": " + json_number(p.ceiling_fraction) + ",\n";
    o += "    \"caps\": ";
    caps_json(o, p.caps);
    o += ",\n";
    o += "    \"caps_derived\": ";
    caps_json(o, p.caps_derived);
    o += ",\n";
    o += "    \"velocity_source\": " + json_string(p.ceiling_velocity_source) + ",\n";
    o += "    \"accel_source\": " + json_string(p.ceiling_accel_source) + ",\n";
    o += "    \"derivation\": " + json_string_array(p.ceiling_derivation) + "\n";
    o += "  },\n";
    o += "  \"config\": {\n";
    o += "    \"config_dir\": " + json_string(p.config_dir) + ",\n";
    o += "    \"yaml_velocity_limit_rev_s\": " + json_number(p.yaml_velocity_limit_rev_s) + ",\n";
    o += "    \"yaml_accel_limit_rev_s2\": " + json_number(p.yaml_accel_limit_rev_s2) + ",\n";
    o += "    \"meters_per_motor_rev\": " + json_number(p.meters_per_motor_rev) + "\n";
    o += "  },\n";
    o += "  \"operator\": " + json_string(p.operator_name) + ",\n";
    o += "  \"note\": " + json_string(p.note) + ",\n";
    o += "  \"approved_utc\": " + json_string(p.approved_utc) + ",\n";
    o += "  \"log_path\": " + json_string(p.log_path) + ",\n";
    const auto& w = p.window;
    o += "  \"window\": {\n";
    o += "    \"from_seq\": " + std::to_string(w.from_seq) + ",\n";
    o += "    \"to_seq\": " + std::to_string(w.to_seq) + ",\n";
    o += "    \"from_mono_s\": " + json_number(w.from_mono_s) + ",\n";
    o += "    \"to_mono_s\": " + json_number(w.to_mono_s) + ",\n";
    o += "    \"drive_cycles\": " + std::to_string(w.drive_cycles) + ",\n";
    o += "    \"peak_cmd_wheel_rev_s\": " + json_number(w.peak_cmd_wheel_rev_s) + ",\n";
    o += "    \"peak_measured_wheel_rev_s\": " + json_number(w.peak_measured_wheel_rev_s) + ",\n";
    o += "    \"peak_q_current_a\": " + json_number(w.peak_q_current_a) + ",\n";
    o += "    \"peak_temperature_c\": " + json_number(w.peak_temperature_c) + ",\n";
    o += "    \"faults_seen\": " + json_int_array(w.faults_seen) + ",\n";
    o += "    \"limit_codes_seen\": " + json_int_array(w.limit_codes_seen) + "\n";
    o += "  }\n";
    o += "}\n";
    return o;
}

namespace {

int64_t int_or(const JsonValue* v, int64_t fallback) {
    if (v == nullptr || v->type != JsonValue::Number) return fallback;
    return static_cast<int64_t>(v->num);
}

std::vector<int> int_array(const JsonValue* v) {
    std::vector<int> out;
    if (v == nullptr || v->type != JsonValue::Array) return out;
    for (const auto& e : v->arr) {
        if (e.type == JsonValue::Number) out.push_back(static_cast<int>(e.num));
    }
    return out;
}

std::vector<std::string> string_array(const JsonValue* v) {
    std::vector<std::string> out;
    if (v == nullptr || v->type != JsonValue::Array) return out;
    for (const auto& e : v->arr) {
        if (e.type == JsonValue::String) out.push_back(e.str);
    }
    return out;
}

void settings_from(const JsonValue* v, Settings* s) {
    if (v == nullptr) return;
    s->body_vel_mps = v->get("body_vel_mps") ? v->get("body_vel_mps")->number_or(kNaN) : kNaN;
    s->body_omega_radps =
        v->get("body_omega_radps") ? v->get("body_omega_radps")->number_or(kNaN) : kNaN;
    s->velocity_limit_rev_s =
        v->get("velocity_limit_rev_s") ? v->get("velocity_limit_rev_s")->number_or(kNaN) : kNaN;
    s->accel_limit_rev_s2 =
        v->get("accel_limit_rev_s2") ? v->get("accel_limit_rev_s2")->number_or(kNaN) : kNaN;
}

void caps_from(const JsonValue* v, Caps* c) {
    Settings s;
    settings_from(v, &s);
    c->body_vel_mps = s.body_vel_mps;
    c->body_omega_radps = s.body_omega_radps;
    c->velocity_limit_rev_s = s.velocity_limit_rev_s;
    c->accel_limit_rev_s2 = s.accel_limit_rev_s2;
}

double num_of(const JsonValue* obj, const char* key) {
    if (obj == nullptr) return kNaN;
    const JsonValue* v = obj->get(key);
    return v ? v->number_or(kNaN) : kNaN;
}

std::string str_of(const JsonValue* obj, const char* key) {
    if (obj == nullptr) return "";
    const JsonValue* v = obj->get(key);
    return v ? v->string_or("") : "";
}

}  // namespace

bool decode_profile(const std::string& json, Profile* out, std::string* error) {
    JsonValue root;
    std::string err;
    if (!parse_json(json, &root, &err)) {
        if (error) *error = "not JSON: " + err;
        return false;
    }
    if (root.type != JsonValue::Object) {
        if (error) *error = "top level is not an object";
        return false;
    }
    if (str_of(&root, "type") != "robot_profile") {
        if (error) *error = "not a robot_profile document";
        return false;
    }
    Profile p;
    p.schema = static_cast<int>(int_or(root.get("schema"), 0));
    if (p.schema != kSchemaVersion) {
        if (error) *error = "unsupported schema " + std::to_string(p.schema);
        return false;
    }
    p.tool = str_of(&root, "tool");
    p.pi_serial = str_of(&root, "pi_serial");
    if (p.pi_serial.empty()) {
        if (error) *error = "missing pi_serial";
        return false;
    }
    if (const JsonValue* cs = root.get("controllers"); cs && cs->type == JsonValue::Array) {
        for (const auto& c : cs->arr) {
            if (c.type != JsonValue::Object) continue;
            ControllerIdentity id;
            id.id = static_cast<int>(int_or(c.get("id"), 0));
            id.bus = static_cast<int>(int_or(c.get("bus"), 0));
            id.serial = str_of(&c, "serial");
            id.git_hash = str_of(&c, "git_hash");
            id.git_dirty = c.get("git_dirty") ? c.get("git_dirty")->bool_or(false) : false;
            id.git_timestamp = int_or(c.get("git_timestamp"), 0);
            id.abi_version = int_or(c.get("abi_version"), -1);
            id.register_map = int_or(c.get("register_map"), -1);
            id.model = int_or(c.get("model"), -1);
            id.hwrev = str_of(&c, "hwrev");
            p.controllers.push_back(id);
        }
    }
    settings_from(root.get("approved"), &p.approved);
    if (const JsonValue* ce = root.get("ceiling")) {
        p.ceiling_fraction = num_of(ce, "fraction");
        caps_from(ce->get("caps"), &p.caps);
        caps_from(ce->get("caps_derived"), &p.caps_derived);
        p.ceiling_mode = str_of(ce, "mode");
        p.approved_under_override = ce->get("approved_under_override")
                                        ? ce->get("approved_under_override")->bool_or(false)
                                        : false;
        p.ceiling_velocity_source = str_of(ce, "velocity_source");
        p.ceiling_accel_source = str_of(ce, "accel_source");
        p.ceiling_derivation = string_array(ce->get("derivation"));
    }
    if (const JsonValue* cf = root.get("config")) {
        p.config_dir = str_of(cf, "config_dir");
        p.yaml_velocity_limit_rev_s = num_of(cf, "yaml_velocity_limit_rev_s");
        p.yaml_accel_limit_rev_s2 = num_of(cf, "yaml_accel_limit_rev_s2");
        p.meters_per_motor_rev = num_of(cf, "meters_per_motor_rev");
    }
    p.operator_name = str_of(&root, "operator");
    p.note = str_of(&root, "note");
    p.approved_utc = str_of(&root, "approved_utc");
    p.log_path = str_of(&root, "log_path");
    if (const JsonValue* w = root.get("window")) {
        p.window.from_seq = static_cast<uint64_t>(int_or(w->get("from_seq"), 0));
        p.window.to_seq = static_cast<uint64_t>(int_or(w->get("to_seq"), 0));
        p.window.from_mono_s = num_of(w, "from_mono_s");
        p.window.to_mono_s = num_of(w, "to_mono_s");
        p.window.drive_cycles = static_cast<uint64_t>(int_or(w->get("drive_cycles"), 0));
        p.window.peak_cmd_wheel_rev_s = num_of(w, "peak_cmd_wheel_rev_s");
        p.window.peak_measured_wheel_rev_s = num_of(w, "peak_measured_wheel_rev_s");
        p.window.peak_q_current_a = num_of(w, "peak_q_current_a");
        p.window.peak_temperature_c = num_of(w, "peak_temperature_c");
        p.window.faults_seen = int_array(w->get("faults_seen"));
        p.window.limit_codes_seen = int_array(w->get("limit_codes_seen"));
    }
    *out = p;
    return true;
}

std::vector<std::string> compare_controllers(const std::vector<ControllerIdentity>& recorded,
                                             const std::vector<ControllerIdentity>& live) {
    std::vector<std::string> out;
    std::map<int, const ControllerIdentity*> rec, now;
    for (const auto& c : recorded) rec[c.id] = &c;
    for (const auto& c : live) now[c.id] = &c;

    for (const auto& [id, r] : rec) {
        const auto it = now.find(id);
        if (it == now.end()) {
            out.push_back("controller " + std::to_string(id) + " (serial " + r->serial +
                          ") was approved but is not present now");
            continue;
        }
        const ControllerIdentity* l = it->second;
        if (r->serial != l->serial) {
            out.push_back("controller " + std::to_string(id) + " serial changed: approved " +
                          (r->serial.empty() ? "?" : r->serial) + " -> now " +
                          (l->serial.empty() ? "?" : l->serial) + " (a different board)");
        } else if (r->git_hash != l->git_hash) {
            out.push_back("controller " + std::to_string(id) + " firmware changed: approved " +
                          (r->git_hash.empty() ? "?" : r->git_hash.substr(0, 8)) + " -> now " +
                          (l->git_hash.empty() ? "?" : l->git_hash.substr(0, 8)) +
                          " (same board, reflashed)");
        } else if (r->git_dirty != l->git_dirty) {
            out.push_back("controller " + std::to_string(id) + " git_dirty flag changed");
        }
    }
    for (const auto& [id, l] : now) {
        if (rec.find(id) == rec.end()) {
            out.push_back("controller " + std::to_string(id) + " (serial " + l->serial +
                          ") is present now but was not in the approved set");
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// JSON reader
// ---------------------------------------------------------------------------

const JsonValue* JsonValue::get(const std::string& key) const {
    if (type != Object) return nullptr;
    for (const auto& [k, v] : obj) {
        if (k == key) return &v;
    }
    return nullptr;
}

double JsonValue::number_or(double fallback) const {
    return type == Number ? num : fallback;
}

std::string JsonValue::string_or(const std::string& fallback) const {
    return type == String ? str : fallback;
}

bool JsonValue::bool_or(bool fallback) const { return type == Bool ? b : fallback; }

namespace {

class Parser {
public:
    Parser(const std::string& t, std::string* err) : t_(t), err_(err) {}

    bool parse(JsonValue* out) {
        ws();
        if (!value(out)) return false;
        ws();
        if (i_ != t_.size()) return fail("trailing characters");
        return true;
    }

private:
    bool fail(const std::string& what) {
        if (err_ && err_->empty()) *err_ = what + " at offset " + std::to_string(i_);
        return false;
    }
    void ws() {
        while (i_ < t_.size() && std::isspace(static_cast<unsigned char>(t_[i_]))) ++i_;
    }
    bool lit(const char* s, JsonValue* out, JsonValue::Type type, bool b) {
        const std::size_t n = std::strlen(s);
        if (t_.compare(i_, n, s) != 0) return fail("bad literal");
        i_ += n;
        out->type = type;
        out->b = b;
        return true;
    }
    bool value(JsonValue* out) {
        if (i_ >= t_.size()) return fail("unexpected end");
        const char c = t_[i_];
        if (c == '{') return object(out);
        if (c == '[') return array(out);
        if (c == '"') {
            out->type = JsonValue::String;
            return string(&out->str);
        }
        if (c == 't') return lit("true", out, JsonValue::Bool, true);
        if (c == 'f') return lit("false", out, JsonValue::Bool, false);
        if (c == 'n') return lit("null", out, JsonValue::Null, false);
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return number(out);
        return fail("unexpected character");
    }
    bool number(JsonValue* out) {
        const char* start = t_.c_str() + i_;
        char* end = nullptr;
        const double v = std::strtod(start, &end);
        if (end == start) return fail("bad number");
        i_ += static_cast<std::size_t>(end - start);
        out->type = JsonValue::Number;
        out->num = v;
        return true;
    }
    bool hex4(unsigned* v) {
        if (i_ + 4 > t_.size()) return fail("bad \\u escape");
        unsigned x = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = t_[i_ + k];
            x <<= 4;
            if (c >= '0' && c <= '9') x |= c - '0';
            else if (c >= 'a' && c <= 'f') x |= 10 + c - 'a';
            else if (c >= 'A' && c <= 'F') x |= 10 + c - 'A';
            else return fail("bad \\u escape");
        }
        i_ += 4;
        *v = x;
        return true;
    }
    static void utf8(std::string* s, unsigned cp) {
        if (cp < 0x80) {
            *s += static_cast<char>(cp);
        } else if (cp < 0x800) {
            *s += static_cast<char>(0xC0 | (cp >> 6));
            *s += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            *s += static_cast<char>(0xE0 | (cp >> 12));
            *s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            *s += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            *s += static_cast<char>(0xF0 | (cp >> 18));
            *s += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            *s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            *s += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }
    bool string(std::string* out) {
        if (t_[i_] != '"') return fail("expected string");
        ++i_;
        out->clear();
        while (i_ < t_.size()) {
            const char c = t_[i_++];
            if (c == '"') return true;
            if (c != '\\') {
                *out += c;
                continue;
            }
            if (i_ >= t_.size()) return fail("bad escape");
            const char e = t_[i_++];
            switch (e) {
                case '"': *out += '"'; break;
                case '\\': *out += '\\'; break;
                case '/': *out += '/'; break;
                case 'b': *out += '\b'; break;
                case 'f': *out += '\f'; break;
                case 'n': *out += '\n'; break;
                case 'r': *out += '\r'; break;
                case 't': *out += '\t'; break;
                case 'u': {
                    unsigned cp = 0;
                    if (!hex4(&cp)) return false;
                    if (cp >= 0xD800 && cp <= 0xDBFF && t_.compare(i_, 2, "\\u") == 0) {
                        i_ += 2;
                        unsigned lo = 0;
                        if (!hex4(&lo)) return false;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    utf8(out, cp);
                    break;
                }
                default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }
    bool array(JsonValue* out) {
        out->type = JsonValue::Array;
        ++i_;
        ws();
        if (i_ < t_.size() && t_[i_] == ']') {
            ++i_;
            return true;
        }
        for (;;) {
            JsonValue v;
            ws();
            if (!value(&v)) return false;
            out->arr.push_back(std::move(v));
            ws();
            if (i_ >= t_.size()) return fail("unterminated array");
            if (t_[i_] == ',') {
                ++i_;
                continue;
            }
            if (t_[i_] == ']') {
                ++i_;
                return true;
            }
            return fail("expected , or ]");
        }
    }
    bool object(JsonValue* out) {
        out->type = JsonValue::Object;
        ++i_;
        ws();
        if (i_ < t_.size() && t_[i_] == '}') {
            ++i_;
            return true;
        }
        for (;;) {
            ws();
            std::string key;
            if (i_ >= t_.size() || t_[i_] != '"') return fail("expected key");
            if (!string(&key)) return false;
            ws();
            if (i_ >= t_.size() || t_[i_] != ':') return fail("expected :");
            ++i_;
            ws();
            JsonValue v;
            if (!value(&v)) return false;
            out->obj.emplace_back(std::move(key), std::move(v));
            ws();
            if (i_ >= t_.size()) return fail("unterminated object");
            if (t_[i_] == ',') {
                ++i_;
                continue;
            }
            if (t_[i_] == '}') {
                ++i_;
                return true;
            }
            return fail("expected , or }");
        }
    }

    const std::string& t_;
    std::string* err_;
    std::size_t i_ = 0;
};

}  // namespace

bool parse_json(const std::string& text, JsonValue* out, std::string* error) {
    if (error) error->clear();
    Parser p(text, error);
    JsonValue v;
    if (!p.parse(&v)) return false;
    *out = std::move(v);
    return true;
}

}  // namespace rf::dbg::commission
