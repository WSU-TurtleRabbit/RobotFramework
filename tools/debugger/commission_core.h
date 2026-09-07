// commission_core — the pure half of `debugger commission`.
//
// Everything here is free of hardware, terminal and YAML headers so the host
// test suite can verify it without a Pi: the ceiling derivation, the operator
// setting clamps, the wheel-level clamp, the NDJSON record encoders, the
// profile file encoder/decoder and the controller-identity encodings.
//
// The discipline this file enforces, in order of how much it matters:
//
//   * A ceiling is DERIVED from readable backstops (the controllers' own
//     servo.default_velocity_limit / default_accel_limit, or Motor.yaml where
//     a controller has none) at a fixed fraction. It is never a constant in
//     this file. If nothing readable bounds a quantity, derive_ceiling()
//     refuses and the tool does not run.
//   * The ceiling IN FORCE starts equal to the derived one and may be set to
//     ANY positive value by the operator during the session (the `C` menu in
//     commission.cpp). There is deliberately no upper bound: apply_ceiling()
//     never clamps and never refuses a finite positive number. What it does
//     instead is classify: propose_ceiling() says whether a value is above
//     the derived threshold and by what ratio, so the caller can warn and
//     demand an explicit confirmation before applying it. Nothing here is
//     read from or written to disk; ceilings are session state only.
//   * Every record that leaves this file carries BOTH ceilings and an
//     explicit override flag, so a limit approved at 25 rev/s under an
//     override can never be mistaken later for one approved inside the
//     derived ceiling (Profile::approved_under_override).
//   * Every numeric field that can be missing serialises as JSON null, never
//     as 0. A silent wheel that reads as a stopped wheel is finding B-04.
//   * The clamp functions are the ONLY way settings move, and they report
//     when they hit the ceiling so the screen can say so instead of quietly
//     clipping. Settings are clamped to the ceiling in force; the ceiling in
//     force is clamped to nothing.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../../Math/kinematics.h"

namespace rf::dbg::commission {

// Schema 2 added the operator-settable ceiling: caps_derived, ceiling_mode
// and approved_under_override in the profile and the cycle record. No schema
// 1 file was ever written (robot_profiles/ did not exist), so the decoder
// accepts 2 only.
inline constexpr int kSchemaVersion = 2;
inline constexpr const char* kToolVersion = "debugger-commission/2";
inline constexpr double kCeilingFraction = 0.80;
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---------------------------------------------------------------------------
// Ceiling derivation
// ---------------------------------------------------------------------------

// What one controller told us over its diagnostic channel. NaN means "the
// controller has no such limit" (moteus's own convention) OR the value was
// unreadable; `reachable` separates the two for the derivation text.
struct BackstopReading {
    int id = 0;
    bool reachable = false;
    double default_velocity_limit_rev_s = kNaN;  // servo.default_velocity_limit
    double default_accel_limit_rev_s2 = kNaN;    // servo.default_accel_limit
    double max_velocity_rev_s = kNaN;            // servo.max_velocity (derate onset)
};

struct CeilingInputs {
    std::vector<BackstopReading> controllers;
    double yaml_velocity_limit_rev_s = kNaN;  // config/Motor.yaml velocityLimit
    double yaml_accel_limit_rev_s2 = kNaN;    // config/Motor.yaml accelLimit
    double fraction = kCeilingFraction;
};

struct Ceiling {
    bool ok = false;
    double velocity_limit_rev_s = kNaN;
    double accel_limit_rev_s2 = kNaN;
    // Which candidate bound each quantity, in words ("controller 3
    // servo.default_velocity_limit 10.00 rev/s").
    std::string velocity_source;
    std::string accel_source;
    // The full working, one line per step. Printed at startup and written
    // into the log header and the profile.
    std::vector<std::string> derivation;
    std::string error;  // set when !ok
};

// ceiling = fraction * min(candidates). Per controller the velocity candidate
// is servo.default_velocity_limit if finite, else Motor.yaml velocityLimit
// (the value the runtime sends every cycle, so the controller effectively
// runs under it); servo.max_velocity is an additional candidate when finite.
// Same shape for accel without a max_* analogue. Non-positive or non-finite
// candidates are ignored; if a quantity ends up with no candidate at all the
// result is !ok.
Ceiling derive_ceiling(const CeilingInputs& in);

// A wheel-speed ceiling implies a body-twist cap through the calibrated
// kinematics: the largest pure translation (worst direction) and the largest
// pure rotation whose peak wheel demand stays inside the ceiling. Each is
// then min'd with the robot's own configured envelope (Safety.yaml
// envelope.cappedLinear / cappedAngular) so the tool never exceeds what the
// match stack itself would allow.
struct BodyCaps {
    double linear_mps = kNaN;
    double angular_radps = kNaN;
    double worst_linear_gain_rev_s_per_mps = kNaN;  // max over directions
    double angular_gain_rev_s_per_radps = kNaN;
    std::string linear_source;
    std::string angular_source;
};
BodyCaps derive_body_caps(const Kinematics& kin, double wheel_ceiling_rev_s,
                          double envelope_linear_mps, double envelope_angular_radps);

// ---------------------------------------------------------------------------
// Operator settings and clamps
// ---------------------------------------------------------------------------

struct Settings {
    double body_vel_mps = 0.0;
    double body_omega_radps = 0.0;
    double velocity_limit_rev_s = 0.0;
    double accel_limit_rev_s2 = 0.0;
};

struct Caps {
    double body_vel_mps = 0.0;
    double body_omega_radps = 0.0;
    double velocity_limit_rev_s = 0.0;
    double accel_limit_rev_s2 = 0.0;
};

enum class Setting { BodyVel, BodyOmega, VelocityLimit, AccelLimit };
inline constexpr Setting kAllSettings[] = {Setting::BodyVel, Setting::BodyOmega,
                                           Setting::VelocityLimit, Setting::AccelLimit};

const char* setting_name(Setting s);   // "body_vel_mps", ...
const char* setting_label(Setting s);  // "body speed", ...
const char* setting_unit(Setting s);   // "m/s", ...
double setting_step(Setting s);
double setting_floor(Setting s);
double get_setting(const Settings& s, Setting which);
double get_cap(const Caps& c, Setting which);

struct StepResult {
    bool changed = false;
    bool at_ceiling = false;  // the request was cut short (or refused) by the cap
    bool at_floor = false;
    double before = 0.0;
    double after = 0.0;
};

// Move one setting by +/-1 step. Never exceeds its cap, never goes below
// its floor. A request that would cross the cap lands exactly ON the cap and
// reports at_ceiling so the caller can say so; a request while already on
// the cap changes nothing and still reports at_ceiling.
StepResult step_setting(Settings& s, Setting which, int direction, const Caps& caps);

// Clamp every setting into [floor, cap]; used once at startup so the initial
// values can never start above the ceiling.
void clamp_settings(Settings& s, const Caps& caps);

// Direction-preserving wheel clamp: if any |wheel| > cap, scale all four by
// cap/peak. `clipped` says whether anything changed.
struct WheelClamp {
    std::array<double, 4> wheels{};
    double factor = 1.0;
    double peak_before = 0.0;
    bool clipped = false;
};
WheelClamp clamp_wheels(const std::array<double, 4>& demand, double cap_rev_s);

// ---------------------------------------------------------------------------
// Operator-settable ceiling (the `C` menu)
// ---------------------------------------------------------------------------
//
// Two Caps live side by side for the whole session: `derived` (what the
// backstops allow at the fixed fraction; never changes after startup) and
// the ceiling `in_force` (starts equal to derived; the operator may set any
// of its four values to any positive number). Everything below compares the
// two; nothing below bounds the second.

struct OverrideStatus {
    struct Item {
        double in_force = kNaN;
        double derived = kNaN;
        bool manual = false;   // in_force differs from derived at all
        bool above = false;    // in_force exceeds derived (or derived is unknown)
        double ratio = kNaN;   // in_force / derived when derived is positive finite
    };
    Item items[4]{};           // indexed by static_cast<int>(Setting)
    bool any_manual = false;
    bool any_override = false;

    const Item& of(Setting s) const { return items[static_cast<int>(s)]; }
    // "derived" when every value is still the derived one, else "manual".
    const char* mode() const { return any_manual ? "manual" : "derived"; }
};
OverrideStatus override_status(const Caps& in_force, const Caps& derived);

// Classify a value the operator wants to set, WITHOUT applying it. `valid`
// is false only for a non-finite or non-positive number (a ceiling of zero
// would command nothing; a negative one is meaningless) — never for being
// too large. `above_derived` is the warn-and-confirm condition; it is also
// true when no derived reference exists, because "unbounded" is the
// conservative reading of "no reference".
struct CeilingProposal {
    bool valid = false;
    std::string error;         // why not valid, in words
    Setting which = Setting::VelocityLimit;
    double value = kNaN;
    double derived = kNaN;
    bool above_derived = false;
    double ratio = kNaN;       // value / derived
    bool needs_confirm() const { return valid && above_derived; }
};
CeilingProposal propose_ceiling(Setting which, double value, const Caps& derived);

// The text the operator typed -> a positive finite double. Accepts leading
// and trailing whitespace and the usual decimal / exponent forms; rejects
// empty text, trailing junk ("25x"), zero, negatives, nan and inf.
bool parse_positive_double(const std::string& text, double* out, std::string* error);
CeilingProposal propose_ceiling_text(Setting which, const std::string& text, const Caps& derived);

// Set one ceiling value. No upper bound: 500 is stored as 500. A non-finite
// or non-positive value is ignored (changed == false) rather than clamped.
// If the operator's current setting is now above the new ceiling it is
// pulled down onto it (settings never pass the ceiling in force), and that
// is reported so it can be logged and shown.
struct CeilingChange {
    Setting which = Setting::VelocityLimit;
    bool changed = false;
    double before = kNaN;
    double after = kNaN;
    bool setting_clamped = false;
    double setting_before = kNaN;
    double setting_after = kNaN;
};
CeilingChange apply_ceiling(Caps& in_force, Settings& settings, Setting which, double value);

// "velocity=25,accel=40,body=5,yaw=12" -> (Setting, value) pairs. Names:
// velocity | accel | body | yaw (the four ceilings). Used only by the
// test-only --test-ceiling flag (dry-run) so the startup print can be
// verified on hardware without a keyboard; the interactive menu does not go
// through this.
bool parse_ceiling_spec(const std::string& spec, std::vector<std::pair<Setting, double>>* out,
                        std::string* error);

// One line saying what servo.max_velocity is across the controllers, for
// the over-threshold warning: "10.00 rev/s on all four controllers", or
// "10.00 rev/s on controllers 1,2,4; 45.00 rev/s on controller 3", or
// "unreadable on every controller".
std::string describe_max_velocity(const std::vector<BackstopReading>& controllers);

// ---------------------------------------------------------------------------
// Records
// ---------------------------------------------------------------------------

struct WheelSample {
    int id = 0;
    bool replied = false;
    double cmd_rev_s = kNaN;       // what we sent this cycle (NaN when de-energized)
    double velocity_rev_s = kNaN;  // measured
    double position_rev = kNaN;
    double q_current_a = kNaN;
    double temperature_c = kNaN;
    double voltage_v = kNaN;
    int mode = -1;
    int fault = -1;
};

struct ImuSample {
    bool present = false;
    double roll_dps = kNaN, pitch_dps = kNaN, yaw_dps = kNaN;
    double heading_deg = kNaN;
    double accel_x_mps2 = kNaN, accel_y_mps2 = kNaN, accel_z_mps2 = kNaN;
};

// One control cycle. Fixed-size (no std::string) so the control loop can
// copy it into the sink queue without allocating.
struct CycleRecord {
    uint64_t seq = 0;
    double t_mono_s = kNaN;
    double t_wall_unix_s = kNaN;
    double dt_s = kNaN;
    BodyTwist twist_requested{};  // what the operator's keys ask for
    BodyTwist twist_applied{};    // after the wheel-level clamp
    std::array<WheelSample, 4> wheels{};
    ImuSample imu{};
    Settings settings{};
    Caps caps{};           // the ceiling in force this cycle
    Caps caps_derived{};   // what the backstops allow; constant for the session
    bool ceiling_manual = false;   // caps != caps_derived anywhere
    bool override_active = false;  // caps > caps_derived anywhere
    bool energized = false;
    bool hold = false;
    // "" | "velocity_limit" | "ceiling"  — what clipped the wheel demand
    char clip[24] = {};
    // "drive" | "hold" | "prompt" | "menu"
    char state[16] = {};
    char hold_reason[64] = {};
    bool deadline_missed = false;
};

// NDJSON encoders. Every non-finite double becomes null.
std::string encode_cycle(const CycleRecord& r);

// An operator/tool event. `fields` are (key, already-encoded JSON value).
using JsonFields = std::vector<std::pair<std::string, std::string>>;
std::string encode_event(uint64_t seq, double t_mono_s, double t_wall_unix_s,
                         const std::string& kind, const JsonFields& fields);

// JSON scalar encoders (public because the hardware side builds event
// fields with them).
std::string json_string(const std::string& s);
std::string json_number(double v);  // "null" when non-finite
std::string json_int(int64_t v);
std::string json_bool(bool v);
std::string json_string_array(const std::vector<std::string>& v);
std::string json_int_array(const std::vector<int>& v);

// 2026-09-07T01:02:03.456Z from unix seconds; "" when non-finite.
std::string iso8601_utc(double unix_s);

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

struct ControllerIdentity {
    int id = 0;
    int bus = 0;
    std::string serial;     // moteus_tool's 16-char base64 form; "" if unread
    std::string git_hash;   // 40 hex chars; "" if unread
    bool git_dirty = false;
    int64_t git_timestamp = 0;
    int64_t abi_version = -1;   // register 0x101
    int64_t register_map = -1;  // register 0x102
    int64_t model = -1;         // register 0x100
    std::string hwrev;          // from `tel get firmware` when the build reports it
};

// moteus_tool's serial encoding: the 96-bit (s1<<64 | s2<<32 | s3) rendered
// as 16 base-64 digits, alphabet A-Za-z0-9+/, most significant first. This
// is the string in `moteus-cal-<serial>-<date>.log` filenames and in
// `moteus_tool --info`, so a profile can be matched against either.
std::string serial_base64(uint32_t s1, uint32_t s2, uint32_t s3);

// The `Serial` line of /proc/cpuinfo, or "" if absent.
std::string parse_cpuinfo_serial(const std::string& cpuinfo_text);

// `conf get <name>` answers one line: a number, "nan", or "ERR ...".
struct DiagValue {
    bool answered = false;  // a non-empty, non-ERR line came back
    double value = kNaN;    // NaN when "nan", unreadable or an error
    std::string raw;
};
DiagValue parse_conf_get_double(const std::string& line);

// Text-mode diagnostic dumps ("tel get X" / "conf enumerate X") are lines of
// `<dotted.name> <value>`. Returns name -> raw value text. Tolerates CRLF,
// blank lines and a trailing OK.
std::map<std::string, std::string> parse_diag_text(const std::string& transcript);

// git.hash from a parsed `tel get git` dump. Accepts either one line
// `git.hash <hex>` or twenty lines `git.hash.N <byte>`. "" if absent.
std::string git_hash_from_diag(const std::map<std::string, std::string>& kv);

// firmware.serial_number.{0,1,2} from a parsed `tel get firmware` dump, in
// the same base-64 form. "" if absent.
std::string serial_from_firmware_diag(const std::map<std::string, std::string>& kv);

// ---------------------------------------------------------------------------
// Profile
// ---------------------------------------------------------------------------

struct ApprovalWindow {
    uint64_t from_seq = 0, to_seq = 0;
    double from_mono_s = kNaN, to_mono_s = kNaN;
    uint64_t drive_cycles = 0;  // energized cycles with a non-zero twist
    double peak_cmd_wheel_rev_s = kNaN;
    double peak_measured_wheel_rev_s = kNaN;
    double peak_q_current_a = kNaN;
    double peak_temperature_c = kNaN;
    std::vector<int> faults_seen;       // hard faults (<96), distinct
    std::vector<int> limit_codes_seen;  // limit reasons (>=96), distinct
};

struct Profile {
    int schema = kSchemaVersion;
    std::string tool = kToolVersion;
    std::string pi_serial;
    std::vector<ControllerIdentity> controllers;
    Settings approved{};
    Caps caps{};          // the ceiling IN FORCE at sign-off (may be an override)
    Caps caps_derived{};  // what the backstops allowed at the fraction below
    // "derived" if caps == caps_derived on every value, else "manual".
    std::string ceiling_mode = "derived";
    // True when any value of `caps` exceeded `caps_derived` at the moment of
    // sign-off. THE field a reviewer must check: a limit approved under an
    // override was approved with the tool's own safety margin removed.
    bool approved_under_override = false;
    double ceiling_fraction = kCeilingFraction;
    std::string ceiling_velocity_source;
    std::string ceiling_accel_source;
    std::vector<std::string> ceiling_derivation;
    double yaml_velocity_limit_rev_s = kNaN;
    double yaml_accel_limit_rev_s2 = kNaN;
    double meters_per_motor_rev = kNaN;
    std::string operator_name;
    std::string note;
    std::string approved_utc;
    std::string log_path;
    std::string config_dir;
    ApprovalWindow window{};
};

std::string encode_profile(const Profile& p);
bool decode_profile(const std::string& json, Profile* out, std::string* error);

// Human-readable mismatches between what a profile recorded and what is
// present now. Empty means the drivetrain is unchanged. Compared by CAN id:
// serial first (a different board), then firmware hash (the same board
// reflashed), then presence.
std::vector<std::string> compare_controllers(const std::vector<ControllerIdentity>& recorded,
                                             const std::vector<ControllerIdentity>& live);

// ---------------------------------------------------------------------------
// Minimal JSON reader (enough for our own profile files)
// ---------------------------------------------------------------------------

struct JsonValue {
    enum Type { Null, Bool, Number, String, Array, Object };
    Type type = Null;
    bool b = false;
    double num = 0.0;
    std::string str;
    std::vector<JsonValue> arr;
    std::vector<std::pair<std::string, JsonValue>> obj;

    const JsonValue* get(const std::string& key) const;  // Object member or nullptr
    double number_or(double fallback) const;
    std::string string_or(const std::string& fallback) const;
    bool bool_or(bool fallback) const;
};
bool parse_json(const std::string& text, JsonValue* out, std::string* error);

}  // namespace rf::dbg::commission
