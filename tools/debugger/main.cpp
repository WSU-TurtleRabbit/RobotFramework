// debugger — field diagnostic console for the RobotFramework drivetrain.
//
// One binary, no GUI, no ncurses: everything works over a plain ssh session.
// It exists because diagnosing this robot previously meant reading four
// different tools' output and doing the unit conversions by hand.
//
// Design rules, all of them load-bearing:
//
//   * A reply is not a value. Every moteus telemetry field is NaN-initialised;
//     a reply can arrive with fields missing. Missing renders as "--", never
//     as 0. (Substituting 0 for a missing wheel reply is finding B-04.)
//   * Never invent a CAN map. If config/Motor.yaml will not load, this tool
//     refuses to run rather than falling back to {1:1,2:2,3:3,4:4}, which is
//     the silent-default behaviour finding B-03 is about.
//   * q-current is phase current, not pack current. Labelled accordingly.
//   * Register 0x00f carries both hard faults and live limit reasons (>= 96).
//     They are displayed differently because they mean different things.
//   * ABI (0x101) and register-map version (0x102) are different quantities.
//   * Torque uses moteus's own convention, Kt = 8.2699/Kv.
//   * Nothing energizes a wheel without an explicit subcommand and a typed
//     confirmation, and every exit path stops the motors.
//
// Usage: `debugger help`.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "Telemetry.h"
#include "motor_formats.h"
#include "../../Math/kinematics.h"
#include "../../Math/wheel_math.h"

#include "commission.h"
#include "compat.h"
#include "decode.h"
#include "term.h"

namespace {

using namespace rf::dbg;
using mjbots::moteus::CanFdFrame;
using mjbots::moteus::Controller;
using mjbots::moteus::Query;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---------------------------------------------------------------------------
// Options and configuration
// ---------------------------------------------------------------------------

struct Options {
    std::string command = "monitor";
    std::string config_dir;
    double hz = 5.0;
    int id = -1;
    double velocity = 0.0;
    double duration_s = 0.0;
    double vx = 0.0, vy = 0.0, w = 0.0;
    std::string out_path;
    bool no_color = false;
    bool once = false;
    bool assume_yes = false;
    bool json = false;
    int max_scan_id = 16;
    int max_scan_bus = 5;
    // commission
    std::string stream_to;
    std::string log_dir;
    bool dry_run = false;
    std::string test_ceiling;  // hidden, test-only, needs --dry-run
    bool want_help = false;
};

// Everything the debugger needs from the YAML tree, loaded strictly.
struct RobotConfig {
    std::map<int, int> motor_map;          // CAN id -> bus
    // Read strictly here for selftest and as a cross-check against the
    // runtime loader (Wheel_math) that build_kinematics() defers to.
    double meters_per_motor_rev = kNaN;
    double velocity_limit_rev_s = kNaN;    // Motor.yaml velocityLimit
    double accel_limit_rev_s2 = kNaN;      // Motor.yaml accelLimit
    double watchdog_s = 0.1;
    double trip_current_a = kNaN;
    double trip_temp_c = kNaN;
    double min_bus_voltage = kNaN;
    double max_bus_voltage = kNaN;
    double feedback_grace_ms = kNaN;
    int motor_interval_ms = 4;
    int robot_id = -1;
    std::string config_dir;
};

[[noreturn]] void die(const std::string& msg) {
    std::cerr << Style::red() << "error: " << Style::reset() << msg << '\n';
    std::exit(2);
}

// Find the directory holding Main.yaml/Motor.yaml. Checked in order so the
// binary works from build/, from the repo root, and from an install prefix.
std::string discover_config_dir(const std::string& override_dir) {
    const std::vector<std::string> candidates =
        override_dir.empty()
            ? std::vector<std::string>{"../config", "config", "../../config",
                                       "RobotFramework/config"}
            : std::vector<std::string>{override_dir};
    for (const auto& c : candidates) {
        std::ifstream probe(c + "/Motor.yaml");
        if (!probe.good()) continue;
        // Absolute from here on: the header prints it, `calibrate` prints
        // moteus_tool commands containing it, and main() chdir()s into it,
        // after which a relative "../config" would no longer mean anything.
        std::error_code ec;
        const auto abs = std::filesystem::canonical(c, ec);
        return ec ? c : abs.string();
    }
    die("could not find config/Motor.yaml. Pass --config-dir <path>. Looked in: " +
        [&] {
            std::string s;
            for (const auto& c : candidates) s += c + " ";
            return s;
        }());
}

template <typename T>
bool read_scalar(const YAML::Node& node, const char* key, T* out) {
    if (!node || !node[key]) return false;
    try { *out = node[key].as<T>(); return true; } catch (...) { return false; }
}

RobotConfig load_config(const std::string& dir) {
    RobotConfig cfg;
    cfg.config_dir = dir;

    // Motor.yaml is mandatory and is NOT allowed to fall back to a guessed
    // map — a wrong CAN map silently attributes one wheel's telemetry to
    // another, which is exactly the class of fault this tool exists to find.
    YAML::Node motor;
    try {
        motor = YAML::LoadFile(dir + "/Motor.yaml");
    } catch (const std::exception& e) {
        die(std::string("cannot parse ") + dir + "/Motor.yaml: " + e.what() +
            "\n       Refusing to continue with a guessed CAN map.");
    }
    if (!motor["motorMap"]) {
        die(dir + "/Motor.yaml has no motorMap. Refusing to guess one.");
    }
    for (auto it = motor["motorMap"].begin(); it != motor["motorMap"].end(); ++it) {
        cfg.motor_map[it->first.as<int>()] = it->second.as<int>();
    }
    if (cfg.motor_map.empty()) die("motorMap is empty.");

    read_scalar(motor, "metersPerMotorRev", &cfg.meters_per_motor_rev);
    read_scalar(motor, "velocityLimit", &cfg.velocity_limit_rev_s);
    read_scalar(motor, "accelLimit", &cfg.accel_limit_rev_s2);

    // Safety.yaml and Main.yaml are advisory here: absence degrades the health
    // verdicts to "unknown", it does not invent thresholds.
    try {
        YAML::Node s = YAML::LoadFile(dir + "/Safety.yaml");
        read_scalar(s, "watchdogTimeout", &cfg.watchdog_s);
        read_scalar(s, "currentLimit", &cfg.trip_current_a);
        read_scalar(s, "tempLimit", &cfg.trip_temp_c);
        read_scalar(s, "minBusVoltage", &cfg.min_bus_voltage);
        read_scalar(s, "maxBusVoltage", &cfg.max_bus_voltage);
        read_scalar(s, "feedbackGraceMs", &cfg.feedback_grace_ms);
    } catch (...) {}

    try {
        YAML::Node m = YAML::LoadFile(dir + "/Main.yaml");
        // Main.yaml spells the key `intervals` (lower case) and
        // RobotFramework.cpp reads it that way; accept both so the tick shown
        // here is the one the runtime actually uses.
        for (const char* key : {"intervals", "Intervals"}) {
            if (m[key] && m[key]["Motor_interval"]) {
                cfg.motor_interval_ms = m[key]["Motor_interval"].as<int>();
                break;
            }
        }
        read_scalar(m, "Robot_id", &cfg.robot_id);
    } catch (...) {}

    return cfg;
}

// ---------------------------------------------------------------------------
// Formatting helpers shared by the views
// ---------------------------------------------------------------------------

std::string fault_cell(int fault) {
    if (fault < 0) return std::string(Style::grey()) + "--" + Style::reset();
    const auto info = decode_fault(fault);
    if (fault == 0) return std::string(Style::green()) + "ok" + Style::reset();
    std::ostringstream os;
    os << Style::of(info.severity) << info.code << ' ' << info.name << Style::reset();
    return os.str();
}

std::string mode_cell(int mode) {
    if (mode < 0) return std::string(Style::grey()) + "--" + Style::reset();
    const auto info = decode_mode(mode);
    std::ostringstream os;
    os << Style::of(info.severity) << info.name << Style::reset();
    return os.str();
}

// A wheel that did not reply at all is a different state from a wheel that
// replied with bad numbers; both are different from healthy.
std::string presence_cell(bool replied) {
    return replied ? std::string(Style::green()) + "yes" + Style::reset()
                   : std::string(Style::red()) + "NO" + Style::reset();
}

void print_header(const RobotConfig& cfg, const std::string& title) {
    std::cout << Style::bold() << title << Style::reset()
              << Style::grey() << "   config=" << cfg.config_dir;
    if (cfg.robot_id >= 0) std::cout << "  robot_id=" << cfg.robot_id;
    std::cout << "  tick=" << cfg.motor_interval_ms << "ms"
              << Style::reset() << "\n\n";
}

// ---------------------------------------------------------------------------
// Kinematics helper — the "expected vs actual" core
// ---------------------------------------------------------------------------

// Use the runtime's own Motor.yaml loader so "expected" here is exactly what
// RobotFramework commands. Kinematics::inverse() also applies the sign-
// specific, east/west-sector and yaw-coupling terms (motorCommandScale
// Positive/Negative/West/East, bodyYawCouplingCompensation); re-reading a
// subset of the keys here would quietly disagree with the robot by a few
// percent per wheel. Wheel_math reads ../config/Motor.yaml relative to the
// cwd, which main() has already moved into the config directory.
Kinematics build_kinematics(const RobotConfig& cfg) {
    const Kinematics kin = Wheel_math().kinematics();
    // Wheel_math keeps its physical default on any load error rather than
    // failing. We read the same key strictly, so a mismatch means the loader
    // silently fell back — refuse rather than present a guessed scale.
    if (std::isfinite(cfg.meters_per_motor_rev) &&
        std::abs(kin.meters_per_motor_rev - cfg.meters_per_motor_rev) > 1e-9) {
        die("Wheel_math loaded metersPerMotorRev=" + num(kin.meters_per_motor_rev, 5) +
            " but " + cfg.config_dir + "/Motor.yaml says " +
            num(cfg.meters_per_motor_rev, 5) + "; the runtime loader fell back to a default.");
    }
    return kin;
}

// The same geometry with every per-wheel tracking correction set to 1, for
// the pure IK/FK consistency check. Kinematics::forward() consumes measured
// speeds and deliberately does not invert those corrections, so a round trip
// through the *calibrated* model differs from the command by design.
Kinematics geometry_only(Kinematics kin) {
    kin.wheel_command_scale = {{1.0, 1.0, 1.0, 1.0}};
    kin.wheel_command_scale_positive = {{1.0, 1.0, 1.0, 1.0}};
    kin.wheel_command_scale_negative = {{1.0, 1.0, 1.0, 1.0}};
    kin.wheel_command_scale_west = {{1.0, 1.0, 1.0, 1.0}};
    kin.wheel_command_scale_east = {{1.0, 1.0, 1.0, 1.0}};
    return kin;
}

// Measured motor velocities in CAN order 1..4, NaN where a wheel is silent.
std::array<double, 4> measured_array(const std::map<int, MotorTelemetry>& t) {
    std::array<double, 4> out{{kNaN, kNaN, kNaN, kNaN}};
    for (int i = 0; i < 4; ++i) {
        const auto it = t.find(i + 1);
        if (it != t.end()) out[i] = it->second.velocity;
    }
    return out;
}

bool all_finite(const std::array<double, 4>& a) {
    for (double v : a) if (!std::isfinite(v)) return false;
    return true;
}

// ---------------------------------------------------------------------------
// Probing: query arbitrary (id, bus) pairs with an extended register set
// ---------------------------------------------------------------------------

// The stock robot query plus the identity and diagnostic registers the
// running loop never asks for. Extra entries must stay sorted by register.
Query::Format extended_query_format() {
    Query::Format f = rf::robot_query_format();
    f.abs_position = mjbots::moteus::kFloat;
    f.power = mjbots::moteus::kFloat;
    f.motor_temperature = mjbots::moteus::kFloat;
    f.trajectory_complete = mjbots::moteus::kInt8;
    return f;
}

Query::Format identity_query_format() {
    // Query::Format is NOT empty by default: it already asks for position,
    // velocity, torque (kFloat) and voltage, temperature (kInt8). An identity
    // probe wants none of those, so switch them off explicitly and keep only
    // mode + fault alongside the three identity registers.
    Query::Format f;
    f.mode = mjbots::moteus::kInt8;
    f.fault = mjbots::moteus::kInt8;
    f.position = mjbots::moteus::kIgnore;
    f.velocity = mjbots::moteus::kIgnore;
    f.torque = mjbots::moteus::kIgnore;
    f.voltage = mjbots::moteus::kIgnore;
    f.temperature = mjbots::moteus::kIgnore;
    // Registers 0x100..0x102 form one contiguous int32 block. Query::Make
    // requires extra[] sorted by register number (it is) and encodes the
    // block as a single read; the reply is 12 bytes of payload plus headers,
    // well inside one CAN-FD frame.
    int n = 0;
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kModelNumber),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kFirmwareVersion),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kRegisterMapVersion),
                    mjbots::moteus::kInt32};
    return f;
}

double extra_value(const Query::Result& r, int reg) {
    for (const auto& e : r.extra) {
        if (e.register_number == reg) return e.value;
        if (e.register_number == std::numeric_limits<int16_t>::max()) break;
    }
    return kNaN;
}

std::string hex_or_dash(double v, int width) {
    if (!std::isfinite(v)) return "--";
    std::ostringstream os;
    os << "0x" << std::hex << std::setfill('0') << std::setw(width)
       << static_cast<long>(v);
    return os.str();
}

// ---------------------------------------------------------------------------
// Subcommands
// ---------------------------------------------------------------------------

int cmd_faults(const Options&) {
    std::cout << Style::bold() << "moteus register 0x00f — fault and limit codes\n"
              << Style::reset()
              << Style::grey()
              << "Codes >= 96 are NOT faults: they are live 'my output is being limited'\n"
                 "notices reported while a control mode is running.\n"
              << Style::reset() << '\n';

    Table hard({"code", "name", "sev", "meaning", "what to do"});
    Table limit({"code", "name", "sev", "meaning", "what to do"});
    for (int c = 0; c <= 105; ++c) {
        const auto info = decode_fault(c);
        if (std::string(info.name) == "unknown") continue;
        auto& t = info.is_limit ? limit : hard;
        t.row({std::to_string(info.code),
               std::string(Style::of(info.severity)) + info.name + Style::reset(),
               severity_word(info.severity), info.meaning, info.action});
    }
    hard.print();
    std::cout << '\n' << Style::bold() << "Limit reasons (not faults)\n" << Style::reset();
    limit.print();

    std::cout << '\n' << Style::bold() << "Controller modes (register 0x000)\n"
              << Style::reset();
    Table modes({"mode", "name", "meaning"});
    for (int m = 0; m <= 15; ++m) {
        const auto info = decode_mode(m);
        if (std::string(info.name) == "invalid") continue;
        modes.row({std::to_string(m),
                   std::string(Style::of(info.severity)) + info.name + Style::reset(),
                   info.meaning});
    }
    modes.print();
    return 0;
}

int cmd_kinematics(const Options& opt, const RobotConfig& cfg) {
    const auto kin = build_kinematics(cfg);
    print_header(cfg, "kinematics — commanded body twist to wheel demand");

    const BodyTwist t{opt.vx, opt.vy, opt.w};
    const auto wheels = kin.inverse(t);

    std::cout << "commanded twist   vx=" << num(t.vx, 3) << " m/s   vy="
              << num(t.vy, 3) << " m/s   w=" << num(t.w, 3) << " rad/s\n"
              << Style::grey()
              << "metersPerMotorRev=" << num(kin.meters_per_motor_rev, 5)
              << "  bodyLateralScale=" << num(kin.body_lateral_scale, 4)
              << "  yawCoupling=(" << num(kin.yaw_ff_from_vx, 3) << ","
              << num(kin.yaw_ff_from_vy, 3) << ") rad/s per m/s"
              << Style::reset() << "\n\n";

    Table tb({"id", "pos", "role", "mount", "arm(m)", "demand(rev/s)", "surface(m/s)", "vs limit"});
    static const char* kRole[] = {"FR", "RR", "RL", "FL"};
    for (int i = 0; i < 4; ++i) {
        const auto& wh = kin.wheels[i];
        const auto row = wh.row();
        const double arm = row[2];
        const double surface = wheels[i] * kin.meters_per_motor_rev;
        std::string verdict = "--";
        if (std::isfinite(cfg.velocity_limit_rev_s)) {
            const double frac = std::abs(wheels[i]) / cfg.velocity_limit_rev_s;
            std::ostringstream os;
            os << (frac > 1.0 ? Style::red() : frac > 0.9 ? Style::yellow() : Style::green())
               << num(frac * 100.0, 0) << "%" << Style::reset();
            verdict = os.str();
        }
        std::ostringstream pos;
        pos << "(" << num(wh.x_m, 4) << "," << num(wh.y_m, 4) << ")";
        tb.row({std::to_string(i + 1), pos.str(), kRole[i],
                num(wh.drive_angle_deg, 0) + "deg", num(std::abs(arm), 4),
                num(wheels[i], 3), num(surface, 3), verdict});
    }
    tb.print();

    // Round-trip through forward kinematics on the bare geometry: if this
    // does not return the commanded twist, the drive matrix and its inverse
    // disagree. Then the same on the calibrated demand above, which is
    // expected to differ by the per-wheel tracking scales FK does not invert.
    const auto pure = geometry_only(kin);
    const auto back = pure.forward(pure.inverse(t));
    const auto back_cal = kin.forward(wheels);
    std::cout << "\n" << Style::grey() << "FK round-trip (geometry only)   vx=" << num(back.vx, 4)
              << "  vy=" << num(back.vy, 4) << "  w=" << num(back.w, 4)
              << "   (must match the commanded twist)" << Style::reset() << '\n'
              << Style::grey() << "FK of the calibrated demand      vx=" << num(back_cal.vx, 4)
              << "  vy=" << num(back_cal.vy, 4) << "  w=" << num(back_cal.w, 4)
              << "   (differs by the per-wheel tracking scales; expected)"
              << Style::reset() << '\n';

    if (std::isfinite(cfg.velocity_limit_rev_s)) {
        const double peak = kin.peak_motor_rev_s(t);
        std::cout << "peak wheel demand " << num(peak, 3) << " rev/s against velocityLimit "
                  << num(cfg.velocity_limit_rev_s, 2) << " rev/s";
        if (peak > cfg.velocity_limit_rev_s) {
            std::cout << Style::red() << "  -> moteus will clip this command" << Style::reset();
        }
        std::cout << '\n';
    }
    return 0;
}

int cmd_scan(const Options& opt, const RobotConfig& cfg) {
    print_header(cfg, "scan — which CAN id answers on which bus");
    std::cout << Style::grey()
              << "Probing ids 1.." << opt.max_scan_id << " on buses 1.." << opt.max_scan_bus
              << ". Query only; nothing is energized.\n" << Style::reset() << '\n';

    // Deliberately NO servo_map here. Pi3HatMoteusTransport::CHILD_FindBus
    // treats a frame bus of 0 *or 1* as "unspecified" and consults the
    // servo_map first, so with Motor.yaml's map loaded a bus-1 probe of id 2
    // would silently be transmitted on bus 2 and its reply discarded below as
    // wrong-bus. With an empty map the fallback is bus 1, which is exactly
    // what the bus-1 pass means.
    mjbots::pi3hat::Pi3HatMoteusTransport::Options topt;
    auto transport = std::make_shared<mjbots::pi3hat::Pi3HatMoteusTransport>(topt);

    const auto qf = identity_query_format();
    std::map<std::pair<int, int>, Query::Result> found;

    // One pass per bus. Buses 2..5 are carried explicitly by each frame;
    // bus 1 relies on the empty-map fallback explained above.
    for (int bus = 1; bus <= opt.max_scan_bus; ++bus) {
        std::vector<CanFdFrame> frames;
        std::vector<std::shared_ptr<Controller>> holders;
        for (int id = 1; id <= opt.max_scan_id; ++id) {
            Controller::Options copt;
            copt.id = id;
            copt.bus = bus;
            copt.transport = transport;
            copt.query_format = qf;
            auto c = std::make_shared<Controller>(copt);
            holders.push_back(c);
            frames.push_back(c->MakeQuery());
        }
        std::vector<CanFdFrame> replies;
        mjbots::moteus::BlockingCallback cbk;
        transport->Cycle(frames.data(), frames.size(), &replies,
                         nullptr, nullptr, nullptr, cbk.callback());
        cbk.Wait();
        for (const auto& f : replies) {
            if (f.bus != bus) continue;
            found[{bus, f.source}] = Query::Parse(f.data, f.size);
        }
    }

    if (found.empty()) {
        std::cout << Style::red() << "No controller answered on any bus.\n" << Style::reset()
                  << Style::grey()
                  << "Check power, the pi3hat ribbon, CAN termination, and that no other\n"
                     "process (RobotFramework, moteus_tool, tview) holds the pi3hat.\n"
                  << Style::reset();
        return 1;
    }

    Table tb({"bus", "id", "in Motor.yaml", "expected bus", "mode", "fault", "abi(0x101)", "regmap(0x102)"});
    std::set<int> seen_ids;
    bool mismatch = false, duplicate = false;
    for (const auto& [key, res] : found) {
        const auto [bus, id] = key;
        const auto cit = cfg.motor_map.find(id);
        const bool in_map = cit != cfg.motor_map.end();
        const bool right_bus = in_map && cit->second == bus;
        if (in_map && !right_bus) mismatch = true;
        if (!seen_ids.insert(id).second) duplicate = true;

        std::string expected = in_map ? std::to_string(cit->second) : "--";
        if (in_map && !right_bus) expected = std::string(Style::red()) + expected + Style::reset();
        tb.row({std::to_string(bus), std::to_string(id),
                in_map ? std::string(Style::green()) + "yes" + Style::reset()
                       : std::string(Style::yellow()) + "no" + Style::reset(),
                expected, mode_cell(static_cast<int>(res.mode)),
                fault_cell(static_cast<int>(res.fault)),
                hex_or_dash(extra_value(res, 0x101), 4),
                hex_or_dash(extra_value(res, 0x102), 2)});
    }
    tb.print();

    std::cout << '\n';
    int problems = 0;
    for (const auto& [id, bus] : cfg.motor_map) {
        if (found.find({bus, id}) == found.end()) {
            std::cout << Style::red() << "MISSING " << Style::reset()
                      << "id " << id << " expected on bus " << bus
                      << " did not answer.\n";
            ++problems;
        }
    }
    if (mismatch) {
        std::cout << Style::red() << "MISMATCH " << Style::reset()
                  << "a controller answered on a different bus than Motor.yaml declares.\n";
        ++problems;
    }
    if (duplicate) {
        std::cout << Style::red() << "DUPLICATE " << Style::reset()
                  << "the same CAN id answered on more than one bus. The running loop DROPS\n"
                     "          duplicate replies, so that wheel would silently read as absent.\n";
        ++problems;
    }
    if (problems == 0) {
        std::cout << Style::green() << "OK " << Style::reset()
                  << "every configured controller answered on its declared bus, and nothing else did.\n";
    }
    return problems == 0 ? 0 : 1;
}

int cmd_info(const Options&, const RobotConfig& cfg) {
    print_header(cfg, "info — controller identity and firmware");

    mjbots::pi3hat::Pi3HatMoteusTransport::Options topt;
    topt.servo_map = cfg.motor_map;
    auto transport = std::make_shared<mjbots::pi3hat::Pi3HatMoteusTransport>(topt);

    const auto qf = identity_query_format();
    std::vector<CanFdFrame> frames;
    std::vector<std::shared_ptr<Controller>> holders;
    for (const auto& [id, bus] : cfg.motor_map) {
        Controller::Options copt;
        copt.id = id; copt.bus = bus; copt.transport = transport; copt.query_format = qf;
        auto c = std::make_shared<Controller>(copt);
        holders.push_back(c);
        frames.push_back(c->MakeQuery());
    }
    std::vector<CanFdFrame> replies;
    mjbots::moteus::BlockingCallback cbk;
    transport->Cycle(frames.data(), frames.size(), &replies, nullptr, nullptr,
                     nullptr, cbk.callback());
    cbk.Wait();

    std::map<int, Query::Result> byid;
    for (const auto& f : replies) byid[f.source] = Query::Parse(f.data, f.size);

    Table tb({"id", "bus", "replied", "model(0x100)", "abi(0x101)", "regmap(0x102)", "hwrev", "serial"});
    std::vector<std::string> notes;
    for (const auto& [id, bus] : cfg.motor_map) {
        const auto it = byid.find(id);
        if (it == byid.end()) {
            tb.row({std::to_string(id), std::to_string(bus), presence_cell(false),
                    "--", "--", "--", "--", "--"});
            continue;
        }
        // Hardware revision is NOT register-mapped and, as far as the
        // reference client goes, not on the diagnostic text channel either:
        // moteus_tool reads identity via `tel get firmware` (a binary,
        // schema-described telemetry record carrying version/serial/model)
        // and never reports a hwrev at all. "d hwrev" below is UNVERIFIED
        // against the firmware. If the controller rejects it, it answers
        // "ERR ..." and never "OK", so DiagnosticCommand(kExpectOK) drains
        // five empty reads (~1 ms), returns ETIMEDOUT through
        // BlockingCallback::Wait() — which does not throw — and leaves the
        // response empty: the column renders "--". Serial (registers
        // 0x120..0x122) is register-mapped but not fetched yet.
        std::string hwrev = "--", serial = "--";
        for (const auto& h : holders) {
            if (h->options().id != id) continue;
            try {
                std::string fw = h->DiagnosticCommand("d hwrev");
                fw.erase(std::remove(fw.begin(), fw.end(), '\n'), fw.end());
                fw.erase(std::remove(fw.begin(), fw.end(), '\r'), fw.end());
                if (!fw.empty() && fw.rfind("ERR", 0) != 0) hwrev = fw;
            } catch (...) {}
            break;
        }
        // An r4.11 must report hwrev 8; hwrev 7 is r4.5b-r4.8 and takes the
        // reduced gate-drive path intended for a different board revision.
        if (hwrev == "7") {
            hwrev = std::string(Style::yellow()) + hwrev + Style::reset();
        }
        tb.row({std::to_string(id), std::to_string(bus), presence_cell(true),
                hex_or_dash(extra_value(it->second, 0x100), 4),
                hex_or_dash(extra_value(it->second, 0x101), 4),
                hex_or_dash(extra_value(it->second, 0x102), 2),
                hwrev, serial});
    }
    tb.print();

    // Cross-check ABI consistency across the fleet: a mixed fleet is finding
    // B-06 and silently changes what every other number means.
    std::set<long> abis, regmaps;
    for (const auto& [id, res] : byid) {
        const double a = extra_value(res, 0x101);
        const double r = extra_value(res, 0x102);
        if (std::isfinite(a)) abis.insert(static_cast<long>(a));
        if (std::isfinite(r)) regmaps.insert(static_cast<long>(r));
    }
    std::cout << '\n';
    if (abis.size() > 1) {
        std::cout << Style::red() << "MIXED ABI " << Style::reset()
                  << "controllers are running different firmware ABIs. Standardise before\n"
                     "          trusting any cross-wheel comparison.\n";
    }
    if (regmaps.size() > 1) {
        std::cout << Style::red() << "MIXED REGISTER MAP " << Style::reset()
                  << "a register-map mismatch is byte-level misinterpretation, not a warning.\n";
    }
    if (abis.size() <= 1 && regmaps.size() <= 1 && !byid.empty()) {
        std::cout << Style::green() << "OK " << Style::reset()
                  << "firmware ABI and register map are consistent across every controller.\n";
    }
    std::cout << Style::grey()
              << "\nABI (0x101) and register-map version (0x102) are different quantities;\n"
                 "this tool never conflates them. For an r4.11 the expected hardware\n"
                 "revision is 8 (hwrev 7 is r4.5b-r4.8).\n" << Style::reset();
    return 0;
}

// Live dashboard: the default view.
int cmd_monitor(const Options& opt, const RobotConfig& cfg) {
    Telemetry telem;
    const auto kin = build_kinematics(cfg);
    install_signal_handlers();

    FrameWriter frame;
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(0.2, opt.hz)));

    std::map<int, int> miss_streak;
    long tick = 0;

    while (!interrupted()) {
        // energize=false: this sends STOP frames, so telemetry flows while the
        // wheels stay dead. Monitoring must never move the robot.
        const auto data = telem.cycle({}, /*energize=*/false);
        ++tick;

        std::ostringstream head;
        head << Style::bold() << "monitor" << Style::reset()
             << Style::grey() << "  tick " << tick
             << "  @" << num(opt.hz, 1) << " Hz"
             << "  config=" << cfg.config_dir
             << "   (motors are NOT energized)" << Style::reset();

        Table tb({"id", "bus", "rep", "mode", "fault / limit", "pos(rev)", "vel(rev/s)",
                  "q-cur(A)", "d-cur(A)", "torque(Nm)", "temp(C)", "volt(V)"});
        for (const auto& [id, bus] : cfg.motor_map) {
            const auto it = data.find(id);
            const bool replied = it != data.end();
            if (!replied) {
                miss_streak[id]++;
                tb.row({std::to_string(id), std::to_string(bus), presence_cell(false),
                        "--", "--", "--", "--", "--", "--", "--", "--", "--"});
                continue;
            }
            miss_streak[id] = 0;
            const auto& m = it->second;

            std::string cur = num(m.current, 2);
            if (std::isfinite(m.current) && std::isfinite(cfg.trip_current_a) &&
                std::abs(m.current) >= cfg.trip_current_a) {
                cur = std::string(Style::red()) + cur + Style::reset();
            }
            std::string temp = num(m.temperature, 1);
            if (std::isfinite(m.temperature) && std::isfinite(cfg.trip_temp_c) &&
                m.temperature >= cfg.trip_temp_c) {
                temp = std::string(Style::red()) + temp + Style::reset();
            }
            tb.row({std::to_string(id), std::to_string(bus), presence_cell(true),
                    mode_cell(m.mode), fault_cell(m.fault),
                    num(m.position, 3), num(m.velocity, 3), cur,
                    num(compat::d_current_of(m), 2), num(compat::torque_of(m), 3),
                    temp, num(m.voltage, 1)});
        }

        // Wheel odometry -> body twist. Only meaningful with all four wheels.
        const auto meas = measured_array(data);
        std::ostringstream body;
        if (all_finite(meas)) {
            const auto tw = kin.forward(meas);
            body << "body (from wheel odometry)   vx=" << num(tw.vx, 3)
                 << " m/s  vy=" << num(tw.vy, 3) << " m/s  w=" << num(tw.w, 3) << " rad/s";
        } else {
            body << Style::yellow()
                 << "body twist unavailable: at least one wheel velocity is missing"
                 << Style::reset();
        }

        std::ostringstream imu;
        if (telem.attitude_present) {
            imu << "imu   yaw_rate=" << num(telem.imu_yaw_dps, 2) << " deg/s  heading="
                << num(telem.imu_heading_deg, 1) << " deg  accel=("
                << num(telem.imu_accel_x_mps2, 2) << "," << num(telem.imu_accel_y_mps2, 2)
                << "," << num(telem.imu_accel_z_mps2, 2) << ") m/s2";
        } else {
            imu << Style::yellow() << "imu   no attitude sample in this cycle" << Style::reset();
        }

        std::ostringstream warn;
        for (const auto& [id, streak] : miss_streak) {
            if (streak > 0) {
                warn << Style::red() << "wheel " << id << " silent for " << streak
                     << " consecutive cycles" << Style::reset() << '\n';
            }
        }

        frame.begin_frame();
        std::cout << head.str() << "\n\n";
        tb.print();
        std::cout << '\n' << body.str() << '\n' << imu.str() << '\n';
        const std::string w = warn.str();
        if (!w.empty()) std::cout << w;
        std::cout << Style::grey()
                  << "q-cur is PHASE current, not pack current."
                  << compat::missing_fields_note<MotorTelemetry>()
                  << "  Ctrl-C to stop." << Style::reset() << '\n';

        int lines = 2 + tb.line_count() + 4;
        for (char ch : w) if (ch == '\n') ++lines;
        frame.end_frame(lines);

        if (opt.once) break;
        std::this_thread::sleep_for(period);
    }

    std::cout << "\n" << Style::grey() << "stopping controllers..." << Style::reset() << '\n';
    for (auto& [id, c] : telem.controllers) { (void)id; c->SetStop(); }
    return 0;
}

int cmd_imu(const Options& opt, const RobotConfig& cfg) {
    Telemetry telem;
    install_signal_handlers();
    print_header(cfg, "imu — pi3hat attitude and rates");
    FrameWriter frame;
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(0.2, opt.hz)));
    while (!interrupted()) {
        telem.cycle({}, /*energize=*/false);
        Table tb({"channel", "value", "unit"});
        tb.row({"present", telem.attitude_present ? "yes" : "NO", ""});
        tb.row({"roll rate", num(telem.imu_roll_dps, 3), "deg/s"});
        tb.row({"pitch rate", num(telem.imu_pitch_dps, 3), "deg/s"});
        tb.row({"yaw rate (raw)", num(telem.imu_yaw_dps, 3), "deg/s"});
        tb.row({"heading", num(telem.imu_heading_deg, 2), "deg"});
        tb.row({"accel x", num(telem.imu_accel_x_mps2, 3), "m/s2"});
        tb.row({"accel y", num(telem.imu_accel_y_mps2, 3), "m/s2"});
        tb.row({"accel z", num(telem.imu_accel_z_mps2, 3), "m/s2"});
        frame.begin_frame();
        tb.print();
        std::cout << Style::grey()
                  << "yaw rate is the RAW gyro z axis; mounting polarity is applied downstream.\n"
                  << Style::reset();
        frame.end_frame(tb.line_count() + 1);
        if (opt.once) break;
        std::this_thread::sleep_for(period);
    }
    for (auto& [id, c] : telem.controllers) { (void)id; c->SetStop(); }
    return 0;
}

int cmd_config(const Options& opt, const RobotConfig& cfg,
               const std::vector<std::string>& rest) {
    const std::string sub = rest.empty() ? "dump" : rest[0];
    print_header(cfg, "config — live controller configuration");

    if (sub == "apply") {
        std::cout << Style::red() << "refusing.\n" << Style::reset()
                  << "Applying configuration writes persistent state to every controller.\n"
                     "Use moteus_tool directly, and use --restore-config, NOT --write-config:\n\n"
                  << Style::bold()
                  << "  moteus_tool --pi3hat-cfg <cfg> --target <id> \\\n"
                     "              --restore-config " << cfg.config_dir << "/moteus_velocity.cfg\n\n"
                  << Style::reset()
                  << Style::grey()
                  << "--write-config sends each line verbatim with no `conf set` prefix and\n"
                     "would fail every line of a dump-shaped file.\n" << Style::reset();
        return 2;
    }

    Telemetry telem;
    // One STOP-only cycle first, and only talk to controllers that answered
    // it. DiagnosticCommand counts *empty* replies toward its timeout, so a
    // controller that never replies at all would keep it re-issuing reads.
    const auto present = telem.cycle({}, /*energize=*/false);
    for (const auto& [id, bus] : cfg.motor_map) {
        (void)bus;
        auto it = telem.controllers.find(id);
        if (it == telem.controllers.end()) continue;
        std::cout << Style::bold() << "--- controller " << id << " ---" << Style::reset() << '\n';
        if (present.find(id) == present.end()) {
            std::cout << Style::red() << "  no reply to a query; skipping the diagnostic channel\n"
                      << Style::reset();
            continue;
        }
        try {
            const std::string dump = it->second->DiagnosticCommand("conf enumerate");
            if (dump.empty()) {
                std::cout << Style::yellow() << "  no response\n" << Style::reset();
            } else {
                std::cout << dump;
                if (dump.back() != '\n') std::cout << '\n';
            }
        } catch (const std::exception& e) {
            std::cout << Style::red() << "  diagnostic read failed: " << e.what()
                      << Style::reset() << '\n';
        }
    }
    std::cout << Style::grey()
              << "\nThis is the LIVE configuration. " << cfg.config_dir
              << "/moteus_velocity.cfg is never\napplied by any code in this repo, so the two"
                 " can differ without warning.\n" << Style::reset();
    (void)opt;
    return 0;
}

int cmd_selftest(const Options& opt, const RobotConfig& cfg) {
    print_header(cfg, "selftest — pre-drive gate");
    int failures = 0;
    auto check = [&](bool ok, const std::string& name, const std::string& detail) {
        std::cout << (ok ? std::string(Style::green()) + "  PASS  " + Style::reset()
                         : std::string(Style::red()) + "  FAIL  " + Style::reset())
                  << rpad(name, 34) << Style::grey() << detail << Style::reset() << '\n';
        if (!ok) ++failures;
    };

    check(!cfg.motor_map.empty(), "CAN map loaded",
          std::to_string(cfg.motor_map.size()) + " controllers declared");
    check(std::isfinite(cfg.meters_per_motor_rev) && cfg.meters_per_motor_rev > 0,
          "metersPerMotorRev calibrated", num(cfg.meters_per_motor_rev, 5) + " m/rev");
    check(std::isfinite(cfg.velocity_limit_rev_s), "velocityLimit set",
          num(cfg.velocity_limit_rev_s, 2) + " rev/s");
    check(std::isfinite(cfg.accel_limit_rev_s2), "accelLimit set",
          num(cfg.accel_limit_rev_s2, 2) + " rev/s2");
    check(cfg.watchdog_s > 0.0, "moteus watchdog armed", num(cfg.watchdog_s, 3) + " s");

    Telemetry telem;
    const auto data = telem.cycle({}, /*energize=*/false);
    for (const auto& [id, bus] : cfg.motor_map) {
        const auto it = data.find(id);
        check(it != data.end(), "controller " + std::to_string(id) + " replies",
              "bus " + std::to_string(bus));
        if (it == data.end()) continue;
        const auto& m = it->second;
        check(m.fault == 0, "controller " + std::to_string(id) + " fault-free",
              decode_fault(m.fault).name);
        check(std::isfinite(m.voltage), "controller " + std::to_string(id) + " reports voltage",
              num(m.voltage, 1) + " V");
        check(std::isfinite(m.current), "controller " + std::to_string(id) + " reports current",
              num(m.current, 2) + " A phase");
    }
    check(telem.attitude_present, "pi3hat IMU sample present",
          telem.attitude_present ? "attitude received" : "no attitude in cycle");

    std::cout << '\n';
    if (failures == 0) {
        std::cout << Style::green() << "selftest passed." << Style::reset() << '\n';
    } else {
        std::cout << Style::red() << "selftest failed: " << failures << " check(s)."
                  << Style::reset() << '\n';
    }
    for (auto& [id, c] : telem.controllers) { (void)id; c->SetStop(); }
    (void)opt;
    return failures == 0 ? 0 : 1;
}

int cmd_record(const Options& opt, const RobotConfig& cfg) {
    if (opt.out_path.empty()) die("record needs --out <file>");
    std::ofstream out(opt.out_path);
    if (!out) die("cannot open " + opt.out_path + " for writing");
    install_signal_handlers();

    out << "t_s,id,bus,replied,mode,fault,position_rev,velocity_rev_s,"
           "q_current_a,d_current_a,torque_nm,temp_c,voltage_v\n";

    Telemetry telem;
    const auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(0.2, opt.hz)));
    const auto t0 = std::chrono::steady_clock::now();
    long rows = 0;

    std::cout << "recording to " << opt.out_path << " at " << num(opt.hz, 1)
              << " Hz. Ctrl-C to stop.\n";
    while (!interrupted()) {
        const auto data = telem.cycle({}, /*energize=*/false);
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (opt.duration_s > 0.0 && t >= opt.duration_s) break;
        for (const auto& [id, bus] : cfg.motor_map) {
            const auto it = data.find(id);
            out << num(t, 4) << ',' << id << ',' << bus << ',';
            if (it == data.end()) {
                // Empty fields, not zeros: a gap must stay a gap downstream.
                out << "0,,,,,,,,,\n";
                continue;
            }
            const auto& m = it->second;
            auto f = [](double v) { return std::isfinite(v) ? num(v, 6) : std::string(); };
            out << "1," << m.mode << ',' << m.fault << ',' << f(m.position) << ','
                << f(m.velocity) << ',' << f(m.current) << ','
                << f(compat::d_current_of(m)) << ',' << f(compat::torque_of(m)) << ','
                << f(m.temperature) << ',' << f(m.voltage) << '\n';
            ++rows;
        }
        std::this_thread::sleep_for(period);
    }
    out.flush();
    std::cout << "wrote " << rows << " rows to " << opt.out_path << '\n';
    for (auto& [id, c] : telem.controllers) { (void)id; c->SetStop(); }
    return 0;
}

bool confirm(const std::string& prompt, bool assume_yes) {
    if (assume_yes) return true;
    if (!::isatty(STDIN_FILENO)) {
        std::cerr << "refusing: this action needs confirmation and stdin is not a terminal.\n"
                     "Pass --yes only if you have physically checked the robot.\n";
        return false;
    }
    std::cout << Style::yellow() << prompt << Style::reset()
              << "\nType exactly 'yes' to continue: " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "yes";
}

int cmd_spin(const Options& opt, const RobotConfig& cfg) {
    if (opt.id < 0) die("spin needs --id <can id>");
    if (cfg.motor_map.find(opt.id) == cfg.motor_map.end()) {
        die("id " + std::to_string(opt.id) + " is not in the CAN map");
    }
    if (!std::isfinite(opt.velocity)) die("--vel must be finite");
    const double dur = opt.duration_s > 0 ? opt.duration_s : 2.0;

    std::cout << Style::bold() << "spin — single wheel, open bench test\n" << Style::reset()
              << "  wheel id      " << opt.id << '\n'
              << "  velocity      " << num(opt.velocity, 3) << " rev/s\n"
              << "  duration      " << num(dur, 2) << " s\n"
              << "  watchdog      " << num(cfg.watchdog_s, 3) << " s\n\n"
              << Style::red()
              << "THIS TURNS A WHEEL. The robot must be on blocks with the wheel clear.\n"
              << Style::reset();
    if (!confirm("Is the robot on blocks with wheel " + std::to_string(opt.id) + " free to turn?",
                 opt.assume_yes)) {
        std::cout << "aborted.\n";
        return 1;
    }

    Telemetry telem;
    install_signal_handlers();
    const auto period = std::chrono::milliseconds(std::max(1, cfg.motor_interval_ms));
    const auto t0 = std::chrono::steady_clock::now();
    FrameWriter frame;

    while (!interrupted()) {
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (t >= dur) break;
        std::map<int, double> cmd{{opt.id, opt.velocity}};
        const auto data = telem.cycle(cmd, /*energize=*/true);

        const auto it = data.find(opt.id);
        Table tb({"t(s)", "commanded", "measured", "error", "q-cur(A)", "temp(C)", "mode", "fault"});
        if (it == data.end()) {
            tb.row({num(t, 2), num(opt.velocity, 3), "--", "--", "--", "--", "--", "--"});
        } else {
            const auto& m = it->second;
            const double err = std::isfinite(m.velocity) ? m.velocity - opt.velocity : kNaN;
            tb.row({num(t, 2), num(opt.velocity, 3), num(m.velocity, 3), num(err, 3),
                    num(m.current, 2), num(m.temperature, 1),
                    mode_cell(m.mode), fault_cell(m.fault)});
        }
        frame.begin_frame();
        tb.print();
        frame.end_frame(tb.line_count());
        std::this_thread::sleep_for(period);
    }

    // Every exit path stops the wheel, including Ctrl-C.
    for (auto& [id, c] : telem.controllers) { (void)id; c->SetStop(); }
    std::cout << "\n" << Style::green() << "stopped." << Style::reset() << '\n';
    return 0;
}

int cmd_calibrate(const Options& opt, const RobotConfig& cfg) {
    std::cout << Style::bold() << "calibrate\n" << Style::reset()
              << Style::grey()
              << "Calibration is performed by moteus_tool, not by this binary: it writes\n"
                 "persistent motor state and must not share the CAN bus with anything else.\n"
              << Style::reset() << '\n';

    std::cout << Style::red()
              << "Stop RobotFramework before running any of these — two processes must never\n"
                 "hold the pi3hat at once.\n" << Style::reset() << '\n';

    std::cout << Style::bold() << "Per controller:\n" << Style::reset();
    for (const auto& [id, bus] : cfg.motor_map) {
        if (opt.id >= 0 && id != opt.id) continue;
        std::cout << "  # id " << id << " (bus " << bus << ")\n"
                  << "  moteus_tool --pi3hat-cfg " << bus << "=" << id
                  << " --target " << id << " --calibrate\n";
    }
    std::cout << '\n' << Style::bold() << "Then restore configuration (note: --restore-config):\n"
              << Style::reset();
    for (const auto& [id, bus] : cfg.motor_map) {
        if (opt.id >= 0 && id != opt.id) continue;
        std::cout << "  moteus_tool --pi3hat-cfg " << bus << "=" << id
                  << " --target " << id << " --restore-config "
                  << cfg.config_dir << "/moteus_velocity.cfg\n";
    }
    std::cout << '\n' << Style::grey()
              << "--write-config would send each line verbatim with no `conf set` prefix and\n"
                 "fail every line. Always --restore-config.\n"
                 "After calibrating, re-run `debugger selftest` and `debugger scan`.\n"
              << Style::reset();
    return 0;
}

int cmd_help() {
    std::cout << Style::bold() << "debugger" << Style::reset()
              << " — field diagnostic console for the RobotFramework drivetrain\n\n"
              << Style::bold() << "USAGE\n" << Style::reset()
              << "  debugger [command] [options]\n\n"
              << Style::bold() << "COMMANDS\n" << Style::reset()
              << "  monitor      live telemetry for every wheel (default). Motors stay dead.\n"
              << "  scan         which CAN id answers on which bus; finds dupes and orphans\n"
              << "  info         model, firmware ABI (0x101), register map (0x102) per controller\n"
              << "  selftest     pre-drive gate: map, calibration, replies, faults, IMU\n"
              << "  faults       decode table for every fault and limit-reason code\n"
              << "  kinematics   body twist -> per-wheel demand, with an FK round-trip check\n"
              << "  imu          live pi3hat attitude, rates and acceleration\n"
              << "  config dump  read live controller configuration over the diagnostic channel\n"
              << "  record       capture telemetry to CSV for offline analysis\n"
              << "  spin         " << Style::red() << "MOVES A WHEEL" << Style::reset()
              << " — single-wheel jog, needs confirmation\n"
              << "  calibrate    print the exact moteus_tool commands for this robot\n"
              << "  commission   " << Style::red() << "MOVES THE ROBOT" << Style::reset()
              << " — keyboard drive while raising limits under a derived ceiling;\n"
                 "               logs every cycle, writes a signed profile. `commission --help`\n"
              << "  help         this text\n\n"
              << Style::bold() << "OPTIONS\n" << Style::reset()
              << "  --config-dir <path>   where Motor.yaml lives (default: search ../config, config)\n"
              << "  --hz <n>              refresh rate for live views (default 5)\n"
              << "  --once                render one frame and exit (for scripts and pipes)\n"
              << "  --id <n>              target a single CAN id\n"
              << "  --vel <rev/s>         velocity for spin\n"
              << "  --duration <s>        run length for spin and record\n"
              << "  --vx --vy --w         body twist for kinematics\n"
              << "  --out <file>          output path for record\n"
              << "  --no-color            disable ANSI colour (also honours NO_COLOR)\n"
              << "  --yes                 skip the confirmation prompt for guarded actions\n"
              << "  --stream-to <ip:port> commission: also stream records by UDP\n"
              << "  --log-dir <dir>       commission: where the NDJSON log goes\n"
              << "  --dry-run             commission: identity, ceiling, banner; no drive loop\n\n"
              << Style::grey()
              << "Missing telemetry always renders as '--', never as 0.\n"
                 "q-current is phase current, not pack current.\n"
                 "Codes >= 96 in the fault column are live limit reasons, not faults.\n"
              << Style::reset();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    std::vector<std::string> rest;

    auto need = [&](int& i, const char* flag) -> std::string {
        if (i + 1 >= argc) die(std::string("missing value for ") + flag);
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config-dir") opt.config_dir = need(i, "--config-dir");
        else if (a == "--hz") opt.hz = std::atof(need(i, "--hz").c_str());
        else if (a == "--id") opt.id = std::atoi(need(i, "--id").c_str());
        else if (a == "--vel") opt.velocity = std::atof(need(i, "--vel").c_str());
        else if (a == "--duration") opt.duration_s = std::atof(need(i, "--duration").c_str());
        else if (a == "--vx") opt.vx = std::atof(need(i, "--vx").c_str());
        else if (a == "--vy") opt.vy = std::atof(need(i, "--vy").c_str());
        else if (a == "--w") opt.w = std::atof(need(i, "--w").c_str());
        else if (a == "--out") opt.out_path = need(i, "--out");
        else if (a == "--no-color" || a == "--no-colour") opt.no_color = true;
        else if (a == "--once") opt.once = true;
        else if (a == "--yes") opt.assume_yes = true;
        else if (a == "--stream-to") opt.stream_to = need(i, "--stream-to");
        else if (a == "--log-dir") opt.log_dir = need(i, "--log-dir");
        else if (a == "--dry-run") opt.dry_run = true;
        // Hidden, test-only (commission --dry-run): pre-set ceilings so the
        // startup print can be checked in every state. Refused by run()
        // without --dry-run.
        else if (a == "--test-ceiling") opt.test_ceiling = need(i, "--test-ceiling");
        else if (a == "-h" || a == "--help") { opt.want_help = true; }
        else if (!a.empty() && a[0] == '-') die("unknown option: " + a);
        else if (rest.empty() && opt.command == "monitor") opt.command = a;
        else rest.push_back(a);
    }

    init_style(opt.no_color);

    if (opt.want_help) {
        if (opt.command == "commission") { rf::dbg::commission::print_help(); return 0; }
        return cmd_help();
    }
    if (opt.command == "help") {
        if (!rest.empty() && rest[0] == "commission") { rf::dbg::commission::print_help(); return 0; }
        return cmd_help();
    }
    if (opt.command == "faults") return cmd_faults(opt);

    const std::string dir = discover_config_dir(opt.config_dir);
    const RobotConfig cfg = load_config(dir);

    // Telemetry and Wheel_math resolve their own YAML with "../config/...",
    // so run from inside the config directory: "../config/X" then resolves
    // back to the same file. Done after load_config so our own reads used
    // the path we discovered. Only `calibrate` needs neither.
    if (opt.command != "calibrate") {
        // Anything the user named relative to where they ran us must be
        // pinned before the cwd moves, or `record --out wheels.csv` would
        // land inside the config directory.
        if (!opt.out_path.empty()) {
            std::error_code ec;
            const auto abs = std::filesystem::absolute(opt.out_path, ec);
            if (!ec) opt.out_path = abs.string();
        }
        if (!opt.log_dir.empty()) {
            std::error_code ec;
            const auto abs = std::filesystem::absolute(opt.log_dir, ec);
            if (!ec) opt.log_dir = abs.string();
        }
        if (::chdir(dir.c_str()) != 0) {
            die("cannot enter config directory " + dir);
        }
        // The trick only works when the directory is literally named "config"
        // under the tree root. Verify, rather than let Telemetry fall back to
        // its guessed {1:1,2:2,3:3,4:4} map (finding B-03) on a bad path.
        std::error_code ec;
        const bool same = std::filesystem::equivalent(
            "../config/Motor.yaml", dir + "/Motor.yaml", ec);
        if (ec || !same) {
            die("Telemetry hardcodes ../config/Motor.yaml, which from inside " + dir +
                " does not resolve to the Motor.yaml just loaded.\n       "
                "--config-dir must point at a directory named 'config' under the tree root.");
        }
    }

    if (opt.command == "kinematics") return cmd_kinematics(opt, cfg);
    if (opt.command == "calibrate") return cmd_calibrate(opt, cfg);
    if (opt.command == "scan") return cmd_scan(opt, cfg);
    if (opt.command == "info") return cmd_info(opt, cfg);
    if (opt.command == "selftest") return cmd_selftest(opt, cfg);
    if (opt.command == "monitor") return cmd_monitor(opt, cfg);
    if (opt.command == "imu") return cmd_imu(opt, cfg);
    if (opt.command == "config") return cmd_config(opt, cfg, rest);
    if (opt.command == "record") return cmd_record(opt, cfg);
    if (opt.command == "spin") return cmd_spin(opt, cfg);
    if (opt.command == "commission") {
        rf::dbg::commission::RunOptions ro;
        ro.config_dir = dir;
        ro.motor_map = cfg.motor_map;
        ro.yaml_velocity_limit_rev_s = cfg.velocity_limit_rev_s;
        ro.yaml_accel_limit_rev_s2 = cfg.accel_limit_rev_s2;
        ro.meters_per_motor_rev = cfg.meters_per_motor_rev;
        ro.motor_interval_ms = cfg.motor_interval_ms;
        ro.robot_id = cfg.robot_id;
        ro.stream_to = opt.stream_to;
        ro.log_dir = opt.log_dir;
        ro.dry_run = opt.dry_run;
        ro.assume_yes = opt.assume_yes;
        ro.test_ceiling = opt.test_ceiling;
        try {
            return rf::dbg::commission::run(ro);
        } catch (const std::exception& e) {
            // run() stops the motors on the way out (StopGuard); this only reports.
            die(std::string("commission: ") + e.what());
        }
    }

    std::cerr << "unknown command: " << opt.command << "\n\n";
    cmd_help();
    return 2;
}
