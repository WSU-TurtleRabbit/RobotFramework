#include "commission.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "Telemetry.h"
#include "../../Math/kinematics.h"
#include "../../Math/wheel_math.h"
#include "../../Safety/supervisor.h"

#include "commission_core.h"
#include "compat.h"
#include "decode.h"
#include "term.h"

namespace rf::dbg::commission {

namespace {

using mjbots::moteus::CanFdFrame;
using mjbots::moteus::Controller;
using mjbots::moteus::Query;
using Clock = std::chrono::steady_clock;

constexpr int kScreenEveryCycles = 25;   // 10 Hz at 4 ms
constexpr std::size_t kQueueCapacity = 4096;  // ~16 s at 250 Hz
constexpr int kMissingReplyHoldCycles = 25;   // 100 ms at 4 ms
constexpr double kMaxWatchdogS = 0.5;

// ---------------------------------------------------------------------------
// Terminal
// ---------------------------------------------------------------------------

// Lifted from tools/RemoteControl.cpp: raw-ish mode so single keys arrive
// without Enter, restored on every exit path. ISIG stays on so Ctrl-C still
// raises SIGINT and goes through the same stop path as everything else.
class RawTerminal {
public:
    RawTerminal() {
        if (::isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &saved_) == 0) {
            ok_ = true;
            termios raw = saved_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
    }
    ~RawTerminal() {
        if (ok_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
    RawTerminal(const RawTerminal&) = delete;
    RawTerminal& operator=(const RawTerminal&) = delete;

private:
    termios saved_{};
    bool ok_ = false;
};

int read_key() {
    unsigned char c;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return n == 1 ? static_cast<int>(c) : -1;
}

std::atomic<bool> g_hangup{false};
extern "C" void on_hangup(int) { g_hangup.store(true); }

bool must_stop() { return interrupted() || g_hangup.load(); }

// ---------------------------------------------------------------------------
// Stop guard: the one place SetStop() lives for the exit paths
// ---------------------------------------------------------------------------

class StopGuard {
public:
    explicit StopGuard(Telemetry& t) : t_(t) {}
    ~StopGuard() { stop(); }
    void stop() {
        // Twice with a gap: a STOP that lands while a reply is in flight can
        // be dropped by a busy transport; the second one is cheap insurance.
        for (int pass = 0; pass < 2; ++pass) {
            for (auto& [id, c] : t_.controllers) {
                (void)id;
                try { c->SetStop(); } catch (...) {}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    Telemetry& t_;
};

// ---------------------------------------------------------------------------
// Record sink: bounded queue -> worker -> NDJSON file (+ best-effort UDP)
// ---------------------------------------------------------------------------

class Sink {
public:
    struct Stats {
        uint64_t queued = 0;
        uint64_t dropped = 0;      // queue full: record never reached disk or stream
        uint64_t written = 0;      // lines on disk
        uint64_t udp_sent = 0;
        uint64_t udp_failed = 0;   // sendto() did not accept the datagram
        bool healthy = false;      // file still writable
        bool streaming = false;
    };

    ~Sink() { close(); }

    bool open(const std::string& path, const std::string& stream_to, std::string* err) {
        file_.open(path, std::ios::out | std::ios::trunc);
        if (!file_) {
            *err = "cannot open " + path;
            return false;
        }
        path_ = path;
        if (!stream_to.empty()) {
            if (!open_udp(stream_to, err)) return false;
        }
        healthy_.store(true);
        stopping_.store(false);
        thread_ = std::thread(&Sink::worker, this);
        return true;
    }

    void push_cycle(const CycleRecord& r) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (q_.size() >= kQueueCapacity) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            q_.push_back(Item{true, r, {}});
            queued_.fetch_add(1, std::memory_order_relaxed);
        }
        cv_.notify_one();
    }

    // Events are rare and are what a reviewer reads first, so they are not
    // subject to the cycle-record cap (they are still bounded by rarity).
    void push_line(std::string line) {
        {
            std::lock_guard<std::mutex> lock(m_);
            q_.push_back(Item{false, {}, std::move(line)});
            queued_.fetch_add(1, std::memory_order_relaxed);
        }
        cv_.notify_one();
    }

    Stats stats() const {
        Stats s;
        s.queued = queued_.load(std::memory_order_relaxed);
        s.dropped = dropped_.load(std::memory_order_relaxed);
        s.written = written_.load(std::memory_order_relaxed);
        s.udp_sent = udp_sent_.load(std::memory_order_relaxed);
        s.udp_failed = udp_failed_.load(std::memory_order_relaxed);
        s.healthy = healthy_.load(std::memory_order_relaxed);
        s.streaming = udp_fd_ >= 0;
        return s;
    }

    bool healthy() const { return healthy_.load(std::memory_order_relaxed); }
    const std::string& path() const { return path_; }

    void close() {
        stopping_.store(true);
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
        if (file_.is_open()) {
            file_.flush();
            file_.close();
        }
        if (udp_fd_ >= 0) {
            ::close(udp_fd_);
            udp_fd_ = -1;
        }
    }

private:
    struct Item {
        bool is_cycle;
        CycleRecord rec;
        std::string line;
    };

    bool open_udp(const std::string& target, std::string* err) {
        const auto colon = target.rfind(':');
        if (colon == std::string::npos) {
            *err = "--stream-to needs ip:port";
            return false;
        }
        const std::string host = target.substr(0, colon);
        const int port = std::atoi(target.substr(colon + 1).c_str());
        if (port <= 0 || port > 65535) {
            *err = "--stream-to port out of range";
            return false;
        }
        std::memset(&target_, 0, sizeof target_);
        target_.sin_family = AF_INET;
        target_.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &target_.sin_addr) != 1) {
            *err = "--stream-to host must be an IPv4 dotted quad: " + host;
            return false;
        }
        udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd_ < 0) {
            *err = "socket(): " + std::string(std::strerror(errno));
            return false;
        }
        int one = 1;
        ::setsockopt(udp_fd_, SOL_SOCKET, SO_BROADCAST, &one, sizeof one);
        ::fcntl(udp_fd_, F_SETFL, O_NONBLOCK);
        return true;
    }

    void worker() {
        std::deque<Item> batch;
        auto last_flush = Clock::now();
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(m_);
                cv_.wait_for(lock, std::chrono::milliseconds(100),
                             [&] { return !q_.empty() || stopping_.load(); });
                if (q_.empty() && stopping_.load()) break;
                batch.swap(q_);
            }
            for (const Item& it : batch) {
                const std::string line = it.is_cycle ? encode_cycle(it.rec) : it.line;
                if (healthy_.load(std::memory_order_relaxed)) {
                    file_ << line << '\n';
                    if (!file_) {
                        healthy_.store(false, std::memory_order_relaxed);
                    } else {
                        written_.fetch_add(1, std::memory_order_relaxed);
                    }
                }
                if (udp_fd_ >= 0) {
                    const ssize_t n = ::sendto(udp_fd_, line.data(), line.size(), MSG_DONTWAIT,
                                               reinterpret_cast<sockaddr*>(&target_),
                                               sizeof target_);
                    if (n == static_cast<ssize_t>(line.size())) {
                        udp_sent_.fetch_add(1, std::memory_order_relaxed);
                    } else {
                        udp_failed_.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
            batch.clear();
            const auto now = Clock::now();
            if (now - last_flush > std::chrono::milliseconds(200)) {
                file_.flush();
                if (!file_) healthy_.store(false, std::memory_order_relaxed);
                last_flush = now;
            }
        }
        file_.flush();
    }

    std::deque<Item> q_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::thread thread_;
    std::ofstream file_;
    std::string path_;
    int udp_fd_ = -1;
    sockaddr_in target_{};
    std::atomic<bool> stopping_{false};
    std::atomic<bool> healthy_{false};
    std::atomic<uint64_t> queued_{0}, dropped_{0}, written_{0}, udp_sent_{0}, udp_failed_{0};
};

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

double wall_now_s() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string utc_stamp_compact() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y%m%dT%H%M%SZ", &tm);
    return buf;
}

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Files created under sudo belong to root; hand them back to the invoking
// user so `git status` and later edits behave. Best effort.
void chown_to_invoker(const std::string& path) {
    const char* uid = std::getenv("SUDO_UID");
    const char* gid = std::getenv("SUDO_GID");
    if (uid == nullptr || gid == nullptr) return;
    (void)::chown(path.c_str(), static_cast<uid_t>(std::atoi(uid)),
                  static_cast<gid_t>(std::atoi(gid)));
}

double extra_value(const Query::Result& r, int reg) {
    for (const auto& e : r.extra) {
        if (e.register_number == reg) return e.value;
        if (e.register_number == std::numeric_limits<int16_t>::max()) break;
    }
    return kNaN;
}

int64_t int_or_minus1(double v) {
    return std::isfinite(v) ? static_cast<int64_t>(v) : -1;
}

uint32_t u32_from_register(double v) {
    // kInt32 registers arrive as a double holding an int32; the serial words
    // are unsigned, so reinterpret the sign bit rather than clamp it.
    return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int64_t>(v)));
}

bool confirm_yes(const std::string& prompt, bool assume_yes) {
    if (assume_yes) return true;
    if (!::isatty(STDIN_FILENO)) {
        std::cerr << "refusing: this action needs a typed confirmation and stdin is not a "
                     "terminal.\nPass --yes only if you have physically checked the robot.\n";
        return false;
    }
    std::cout << Style::yellow() << prompt << Style::reset()
              << "\nType exactly 'yes' to continue: " << std::flush;
    std::string answer;
    std::getline(std::cin, answer);
    return answer == "yes";
}

// ---------------------------------------------------------------------------
// Controller identity and backstops (query + diagnostic channel; nothing
// here energizes a wheel)
// ---------------------------------------------------------------------------

// Write a diagnostic command and collect everything the controller says
// until it goes quiet. Used for `tel get X`, whose text-mode reply is a
// multi-line dump with no trailing OK, which DiagnosticCommand(kExpectOK)
// cannot terminate on.
std::string diag_transcript(Controller& c, const std::string& cmd) {
    std::string out;
    try {
        c.DiagnosticWrite(cmd + "\n", 1);
        int quiet = 0;
        const auto deadline = Clock::now() + std::chrono::milliseconds(400);
        while (Clock::now() < deadline) {
            const std::string s = c.DiagnosticRead(1);
            if (s.empty()) {
                if (++quiet >= 6) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } else {
                quiet = 0;
                out += s;
            }
        }
    } catch (...) {
    }
    return out;
}

DiagValue conf_get(Controller& c, const std::string& name) {
    try {
        return parse_conf_get_double(
            c.DiagnosticCommand("conf get " + name, Controller::kExpectSingleLine));
    } catch (...) {
        return DiagValue{};
    }
}

std::vector<ControllerIdentity> read_identities(Telemetry& telem,
                                                const std::map<int, int>& motor_map) {
    // Identity registers ride on a dedicated query format over the SAME
    // transport as Telemetry (two transports must never share the pi3hat).
    Query::Format f;
    f.mode = mjbots::moteus::kInt8;
    f.fault = mjbots::moteus::kInt8;
    f.position = mjbots::moteus::kIgnore;
    f.velocity = mjbots::moteus::kIgnore;
    f.torque = mjbots::moteus::kIgnore;
    f.voltage = mjbots::moteus::kIgnore;
    f.temperature = mjbots::moteus::kIgnore;
    int n = 0;
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kModelNumber),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kFirmwareVersion),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kRegisterMapVersion),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kSerialNumber1),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kSerialNumber2),
                    mjbots::moteus::kInt32};
    f.extra[n++] = {static_cast<int16_t>(mjbots::moteus::Register::kSerialNumber3),
                    mjbots::moteus::kInt32};

    std::vector<CanFdFrame> frames;
    std::vector<std::shared_ptr<Controller>> holders;
    for (const auto& [id, bus] : motor_map) {
        Controller::Options copt;
        copt.id = id;
        copt.bus = bus;
        copt.transport = telem.transport;
        copt.query_format = f;
        auto c = std::make_shared<Controller>(copt);
        holders.push_back(c);
        frames.push_back(c->MakeQuery());
    }
    std::vector<CanFdFrame> replies;
    mjbots::moteus::BlockingCallback cbk;
    telem.transport->Cycle(frames.data(), frames.size(), &replies, nullptr, nullptr, nullptr,
                           cbk.callback());
    cbk.Wait();
    std::map<int, Query::Result> byid;
    for (const auto& fr : replies) byid[fr.source] = Query::Parse(fr.data, fr.size);

    std::vector<ControllerIdentity> out;
    for (const auto& [id, bus] : motor_map) {
        ControllerIdentity ci;
        ci.id = id;
        ci.bus = bus;
        const auto it = byid.find(id);
        if (it != byid.end()) {
            ci.model = int_or_minus1(extra_value(it->second, 0x100));
            ci.abi_version = int_or_minus1(extra_value(it->second, 0x101));
            ci.register_map = int_or_minus1(extra_value(it->second, 0x102));
            const double s1 = extra_value(it->second, 0x120);
            const double s2 = extra_value(it->second, 0x121);
            const double s3 = extra_value(it->second, 0x122);
            if (std::isfinite(s1) && std::isfinite(s2) && std::isfinite(s3)) {
                ci.serial = serial_base64(u32_from_register(s1), u32_from_register(s2),
                                          u32_from_register(s3));
            }
        }
        // Firmware git hash (and a second route to the serial) over the
        // diagnostic channel, through Telemetry's own controller objects.
        auto tc = telem.controllers.find(id);
        if (tc != telem.controllers.end() && it != byid.end()) {
            Controller& c = *tc->second;
            try { c.DiagnosticFlush(1, 0.05); } catch (...) {}
            try { c.DiagnosticCommand("tel fmt git 1"); } catch (...) {}
            const auto git = parse_diag_text(diag_transcript(c, "tel get git"));
            ci.git_hash = git_hash_from_diag(git);
            if (auto d = git.find("git.dirty"); d != git.end()) {
                ci.git_dirty = std::atoi(d->second.c_str()) != 0;
            }
            if (auto ts = git.find("git.timestamp"); ts != git.end()) {
                ci.git_timestamp = std::strtoll(ts->second.c_str(), nullptr, 0);
            }
            try { c.DiagnosticCommand("tel fmt firmware 1"); } catch (...) {}
            const auto fw = parse_diag_text(diag_transcript(c, "tel get firmware"));
            if (ci.serial.empty()) ci.serial = serial_from_firmware_diag(fw);
            if (auto h = fw.find("firmware.hwrev"); h != fw.end()) ci.hwrev = h->second;
        }
        out.push_back(ci);
    }
    return out;
}

std::vector<BackstopReading> read_backstops(Telemetry& telem, const std::map<int, int>& motor_map,
                                            const std::set<int>& present) {
    std::vector<BackstopReading> out;
    for (const auto& [id, bus] : motor_map) {
        (void)bus;
        BackstopReading b;
        b.id = id;
        auto tc = telem.controllers.find(id);
        if (tc == telem.controllers.end() || present.count(id) == 0) {
            out.push_back(b);
            continue;
        }
        Controller& c = *tc->second;
        // Drain anything a previous diagnostic exchange left behind, so the
        // single-line answer below cannot be a stale line mis-read as a limit.
        try { c.DiagnosticFlush(1, 0.05); } catch (...) {}
        const DiagValue v = conf_get(c, "servo.default_velocity_limit");
        const DiagValue a = conf_get(c, "servo.default_accel_limit");
        const DiagValue m = conf_get(c, "servo.max_velocity");
        b.reachable = v.answered || a.answered || m.answered;
        b.default_velocity_limit_rev_s = v.value;
        b.default_accel_limit_rev_s2 = a.value;
        b.max_velocity_rev_s = m.value;
        out.push_back(b);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Prior profiles
// ---------------------------------------------------------------------------

struct PriorProfile {
    bool found = false;
    std::string path;
    Profile profile;
    std::vector<std::string> mismatches;
    std::vector<std::string> unreadable;  // files that did not parse
};

PriorProfile find_prior_profile(const std::string& dir, const std::string& pi_serial,
                                const std::vector<ControllerIdentity>& live) {
    PriorProfile out;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return out;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        if (!e.is_regular_file()) continue;
        if (e.path().extension() != ".json") continue;
        Profile p;
        std::string err;
        if (!decode_profile(read_file(e.path().string()), &p, &err)) {
            out.unreadable.push_back(e.path().filename().string() + ": " + err);
            continue;
        }
        if (p.pi_serial != pi_serial) continue;
        if (!out.found || p.approved_utc > out.profile.approved_utc) {
            out.found = true;
            out.path = e.path().string();
            out.profile = p;
        }
    }
    if (out.found) out.mismatches = compare_controllers(out.profile.controllers, live);
    return out;
}

// ---------------------------------------------------------------------------
// Safety.yaml (envelope caps and trip thresholds, same mapping as the main
// binary so the tool's HOLD behaves like the match stack's estop)
// ---------------------------------------------------------------------------

struct SafetyYaml {
    rf::SafetyConfig cfg;
    double current_grace_ms = 300.0;
    double fault_grace_ms = 50.0;
    bool loaded = false;
};

SafetyYaml load_safety(const std::string& config_dir, int interval_ms) {
    SafetyYaml s;
    try {
        YAML::Node y = YAML::LoadFile(config_dir + "/Safety.yaml");
        if (y["envelope"]) {
            YAML::Node env = y["envelope"];
            if (env["safeLinear"]) s.cfg.safe_linear_mps = env["safeLinear"].as<double>();
            if (env["safeAngular"]) s.cfg.safe_angular_rps = env["safeAngular"].as<double>();
            if (env["cappedLinear"]) s.cfg.capped_linear_mps = env["cappedLinear"].as<double>();
            if (env["cappedAngular"]) s.cfg.capped_angular_rps = env["cappedAngular"].as<double>();
        }
        if (y["currentLimit"]) s.cfg.trip_current_a = y["currentLimit"].as<double>();
        if (y["currentGraceMs"]) s.current_grace_ms = y["currentGraceMs"].as<double>();
        if (y["faultGraceMs"]) s.fault_grace_ms = y["faultGraceMs"].as<double>();
        if (y["tripTempC"]) s.cfg.trip_temp_c = y["tripTempC"].as<double>();
        if (y["minBusVoltage"]) s.cfg.min_bus_voltage = y["minBusVoltage"].as<double>();
        if (y["watchdogTimeout"]) s.cfg.watchdog_s = y["watchdogTimeout"].as<double>();
        s.loaded = true;
    } catch (...) {
        s.loaded = false;
    }
    s.cfg.mode = rf::DriveMode::Capped;
    // Explicit acknowledgement only: a commissioning trip must be looked at,
    // not auto-cleared half a second later.
    s.cfg.recovery_s = 0.0;
    s.cfg.current_grace_ticks = static_cast<uint32_t>(
        std::max(1.0, s.current_grace_ms / std::max(1, interval_ms)));
    s.cfg.fault_grace_ticks = static_cast<uint32_t>(
        std::max(1.0, s.fault_grace_ms / std::max(1, interval_ms)));
    return s;
}

// ---------------------------------------------------------------------------
// Session state
// ---------------------------------------------------------------------------

enum class Mode {
    Drive,
    PromptNote,
    PromptSignName,
    PromptSignNote,
    MenuSelect,   // the `C` ceiling menu: pick 1-4 / R / close
    MenuEdit,     // typing a new value for one ceiling
    MenuConfirm,  // the value is above the derived threshold: Y to accept
};

bool in_menu(Mode m) {
    return m == Mode::MenuSelect || m == Mode::MenuEdit || m == Mode::MenuConfirm;
}

struct Session {
    // static
    std::string pi_serial;
    std::vector<ControllerIdentity> controllers;
    Ceiling ceiling;
    BodyCaps body_caps;
    Caps derived_caps;  // what the backstops allow; fixed for the session
    Caps caps;          // the ceiling in force; the `C` menu may change it
    OverrideStatus ov;  // caps vs derived_caps, refreshed on every change
    std::string max_velocity_note;  // "10.00 rev/s on all four controllers"
    double trip_current_a = kNaN;
    Settings settings;
    Kinematics kin;
    std::string log_path;
    std::string profile_dir;
    std::string config_dir;
    RunOptions opt;
    // dynamic
    Mode mode = Mode::Drive;
    BodyTwist twist_req;
    std::string drive_word = "STOP";
    bool hold = false;
    std::string hold_reason;
    std::string message;
    Clock::time_point message_until;
    std::string prompt_buf;
    std::string signoff_name;
    Setting menu_setting = Setting::VelocityLimit;  // which ceiling MenuEdit is typing
    CeilingProposal menu_proposal;                  // what MenuConfirm is asking about
    uint64_t seq = 0;
    Clock::time_point t0;
    uint64_t deadline_misses = 0;
    std::map<int, int> missing_streak;
    ApprovalWindow window;
    std::string last_profile_path;
    uint64_t profiles_written = 0;
    bool prior_mismatch = false;
};

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

// The words behind each derived ceiling, for the AT CEILING message and the
// over-threshold warning.
std::string ceiling_source(const Session& s, Setting which) {
    switch (which) {
        case Setting::VelocityLimit: return s.ceiling.velocity_source;
        case Setting::AccelLimit: return s.ceiling.accel_source;
        case Setting::BodyVel: return s.body_caps.linear_source;
        case Setting::BodyOmega: return s.body_caps.angular_source;
    }
    return "";
}

std::string caps_to_json(const Caps& c) {
    return "{\"body_vel_mps\":" + json_number(c.body_vel_mps) +
           ",\"body_omega_radps\":" + json_number(c.body_omega_radps) +
           ",\"velocity_limit_rev_s\":" + json_number(c.velocity_limit_rev_s) +
           ",\"accel_limit_rev_s2\":" + json_number(c.accel_limit_rev_s2) + "}";
}

std::string settings_to_json(const Settings& v) {
    return "{\"body_vel_mps\":" + json_number(v.body_vel_mps) +
           ",\"body_omega_radps\":" + json_number(v.body_omega_radps) +
           ",\"velocity_limit_rev_s\":" + json_number(v.velocity_limit_rev_s) +
           ",\"accel_limit_rev_s2\":" + json_number(v.accel_limit_rev_s2) + "}";
}

// Printed at startup and whenever the ceiling in force changes: the ceiling
// in force, the derived one underneath it ALWAYS, and a loud line when the
// first exceeds the second.
void print_ceiling_block(const Session& s) {
    const auto& ov = s.ov;
    std::cout << '\n' << Style::bold() << "CEILING IN FORCE" << Style::reset() << "   velocity_limit "
              << Style::bold() << num(s.caps.velocity_limit_rev_s, 3) << " rev/s" << Style::reset()
              << "   accel_limit " << Style::bold() << num(s.caps.accel_limit_rev_s2, 3) << " rev/s^2"
              << Style::reset() << "   body " << Style::bold() << num(s.caps.body_vel_mps, 3) << " m/s"
              << Style::reset() << " / " << Style::bold() << num(s.caps.body_omega_radps, 3) << " rad/s"
              << Style::reset();
    if (ov.any_override) {
        std::cout << "   " << Style::red() << Style::bold() << "[MANUAL OVERRIDE]" << Style::reset();
    } else if (ov.any_manual) {
        std::cout << "   " << Style::cyan() << "[MANUAL, below derived]" << Style::reset();
    } else {
        std::cout << "   " << Style::green() << "[DERIVED]" << Style::reset();
    }
    std::cout << '\n';
    std::cout << "  derived would be  velocity_limit " << num(s.derived_caps.velocity_limit_rev_s, 3)
              << " rev/s   accel_limit " << num(s.derived_caps.accel_limit_rev_s2, 3)
              << " rev/s^2   body " << num(s.derived_caps.body_vel_mps, 3) << " m/s / "
              << num(s.derived_caps.body_omega_radps, 3) << " rad/s  ("
              << static_cast<int>(std::lround(kCeilingFraction * 100.0))
              << "% of smallest backstop)\n";
    if (ov.any_override) {
        std::cout << Style::red() << Style::bold() << "  OVERRIDE ACTIVE: ";
        bool first = true;
        for (Setting which : {Setting::VelocityLimit, Setting::AccelLimit, Setting::BodyVel,
                              Setting::BodyOmega}) {
            const auto& it = ov.of(which);
            if (!it.above) continue;
            std::cout << (first ? "" : ", ") << num(it.ratio, 1) << "x the derived "
                      << setting_label(which) << " ceiling";
            first = false;
        }
        std::cout << Style::reset() << '\n'
                  << Style::red() << "  servo.max_velocity is " << s.max_velocity_note
                  << "; above it the controllers derate current toward zero themselves.\n"
                  << "  The tool will command what you set; the hardware decides what it delivers.\n"
                  << Style::reset();
    }
}

std::string setting_row(const Session& s, Setting which, const char* keys) {
    const double v = get_setting(s.settings, which);
    const double cap = get_cap(s.caps, which);
    const auto& ov = s.ov.of(which);
    const double head = cap - v;
    std::ostringstream os;
    os << rpad(std::string(setting_label(which)) + " (" + setting_unit(which) + ")", 30)
       << lpad(num(v, 2), 8) << lpad(num(cap, 2), 10) << lpad(num(ov.derived, 2), 10)
       << lpad(num(head, 2), 10) << "   " << rpad(keys, 6);
    if (ov.above) {
        os << Style::red() << Style::bold() << "OVERRIDE " << num(ov.ratio, 1) << "x derived"
           << Style::reset() << ' ';
    } else if (ov.manual) {
        os << Style::cyan() << "manual, below derived" << Style::reset() << ' ';
    }
    if (std::isfinite(cap) && std::abs(head) < 1e-9) {
        os << Style::yellow() << "AT CEILING" << Style::reset();
    }
    return os.str();
}

// The `C` menu, drawn in the prompt area of the live frame so the wheel
// table above it stays visible (the operator can see the wheels are at
// zero while they type).
void render_menu(std::ostream& o, const Session& s) {
    o << Style::cyan() << Style::bold() << "CEILINGS" << Style::reset() << Style::grey()
      << "  session only, nothing is written to disk; every change is logged; robot held at zero"
      << Style::reset();
    if (s.ov.any_override) {
        o << "   " << Style::red() << Style::bold() << "[OVERRIDE ACTIVE]" << Style::reset();
    }
    o << '\n';
    o << Style::bold() << "  #  " << rpad("ceiling", 32) << lpad("in force", 10)
      << lpad("derived", 10) << "   status" << Style::reset() << '\n';
    int n = 1;
    for (Setting which : {Setting::VelocityLimit, Setting::AccelLimit, Setting::BodyVel,
                          Setting::BodyOmega}) {
        const auto& it = s.ov.of(which);
        o << "  " << n++ << "  "
          << rpad(std::string(setting_label(which)) + " (" + setting_unit(which) + ")", 32)
          << lpad(num(it.in_force, 3), 10) << lpad(num(it.derived, 3), 10) << "   ";
        if (it.above) {
            o << Style::red() << Style::bold() << "OVERRIDE " << num(it.ratio, 1) << "x derived"
              << Style::reset();
        } else if (it.manual) {
            o << Style::cyan() << "manual, below derived" << Style::reset();
        } else {
            o << Style::green() << "derived" << Style::reset();
        }
        o << '\n';
    }
    o << Style::grey() << "  servo.max_velocity: " << s.max_velocity_note
      << " (above it the controller derates current toward zero itself)" << Style::reset() << '\n';

    if (s.mode == Mode::MenuSelect) {
        o << Style::cyan() << "  1-4 set a ceiling   R reset all to derived   C or Enter close"
          << Style::reset() << '\n';
    } else if (s.mode == Mode::MenuEdit) {
        const auto& it = s.ov.of(s.menu_setting);
        o << Style::cyan() << "  new " << setting_label(s.menu_setting) << " ceiling ("
          << setting_unit(s.menu_setting) << ")  [in force " << num(it.in_force, 3) << ", derived "
          << num(it.derived, 3) << "]  Enter to set, empty Enter cancels: " << Style::reset()
          << s.prompt_buf << "_\n";
    } else if (s.mode == Mode::MenuConfirm) {
        const auto& p = s.menu_proposal;
        const char* unit = setting_unit(p.which);
        o << "  " << setting_label(p.which) << " ceiling: " << Style::bold() << num(p.value, 3)
          << ' ' << unit << Style::reset() << '\n';
        o << Style::red() << Style::bold() << "  !! ABOVE DERIVED THRESHOLD !!" << Style::reset()
          << "  derived " << num(p.derived, 3) << ' ' << unit << " [" << ceiling_source(s, p.which)
          << "] - you set " << Style::red() << Style::bold() << num(p.ratio, 1) << "x"
          << Style::reset() << " that\n";
        switch (p.which) {
            case Setting::VelocityLimit:
                o << "     controller servo.max_velocity is " << s.max_velocity_note
                  << "; above that they derate current\n"
                     "     toward zero themselves, so commanding more does not produce more. "
                     "This value replaces the\n"
                     "     controller's own default_velocity_limit for every command the tool sends.\n";
                break;
            case Setting::AccelLimit:
                o << "     there is no controller-side accel derate: an accel_limit this high is "
                     "bounded only by the\n"
                     "     controller current limit and the supervisor current trip ("
                  << num(s.trip_current_a, 1) << " A). Expect current-limit codes.\n";
                break;
            case Setting::BodyVel:
                o << "     " << num(p.value, 3) << " m/s needs " << Style::bold()
                  << num(p.value * s.body_caps.worst_linear_gain_rev_s_per_mps, 2) << " rev/s"
                  << Style::reset() << " on the worst wheel (gain "
                  << num(s.body_caps.worst_linear_gain_rev_s_per_mps, 3)
                  << " rev/s per m/s); the velocity ceiling in force is "
                  << num(s.caps.velocity_limit_rev_s, 3) << " rev/s and the wheel clamp\n"
                     "     still applies. servo.max_velocity is " << s.max_velocity_note << ".\n";
                break;
            case Setting::BodyOmega:
                o << "     " << num(p.value, 3) << " rad/s needs " << Style::bold()
                  << num(p.value * s.body_caps.angular_gain_rev_s_per_radps, 2) << " rev/s"
                  << Style::reset() << " per wheel (yaw gain "
                  << num(s.body_caps.angular_gain_rev_s_per_radps, 3)
                  << " rev/s per rad/s); the velocity ceiling in force is "
                  << num(s.caps.velocity_limit_rev_s, 3) << " rev/s and the wheel clamp\n"
                     "     still applies. servo.max_velocity is " << s.max_velocity_note << ".\n";
                break;
        }
        o << Style::yellow() << Style::bold() << "  Press Y to accept, any other key to cancel."
          << Style::reset() << '\n';
    }
}

std::string render_frame(const Session& s, const std::map<int, MotorTelemetry>& data,
                         const Telemetry& telem, const std::array<double, 4>& cmd_wheels,
                         double peak_cmd, double wheel_cap, const char* clip, bool energized,
                         double dt_s, const Sink::Stats& st) {
    std::ostringstream o;
    const double t = std::chrono::duration<double>(Clock::now() - s.t0).count();

    o << Style::bold() << "commission" << Style::reset() << Style::grey() << "  seq " << s.seq
      << "  t=" << num(t, 1) << "s  tick " << s.opt.motor_interval_ms << "ms (last dt "
      << num(dt_s * 1000.0, 2) << " ms, misses " << s.deadline_misses << ")" << Style::reset();
    // Persistent, on the first line of every frame: it never scrolls away.
    if (s.ov.any_override) {
        o << "   " << Style::red() << Style::bold() << "[OVERRIDE] ceiling above derived"
          << Style::reset();
    } else if (s.ov.any_manual) {
        o << "   " << Style::cyan() << "[manual ceiling, below derived]" << Style::reset();
    }
    o << '\n';
    o << Style::grey() << "log " << (st.healthy ? "" : "FAILED ") << st.written << " written / "
      << st.dropped << " dropped";
    if (st.streaming) {
        o << "   stream " << s.opt.stream_to << "  " << st.udp_sent << " sent / " << st.udp_failed
          << " failed";
    } else {
        o << "   stream off";
    }
    o << Style::reset() << '\n';
    o << Style::grey() << "pi " << s.pi_serial << "   controllers";
    for (const auto& c : s.controllers) {
        o << "  " << c.id << ':' << (c.serial.empty() ? "?" : c.serial)
          << (c.git_hash.empty() ? "" : "@" + c.git_hash.substr(0, 7));
    }
    o << Style::reset() << '\n';
    if (s.prior_mismatch) {
        o << Style::red() << Style::bold()
          << "DRIVETRAIN CHANGED since the last approved profile (see startup warning)"
          << Style::reset() << '\n';
    } else {
        o << '\n';
    }

    o << Style::bold() << rpad("setting", 30) << lpad("now", 8) << lpad("ceiling", 10)
      << lpad("derived", 10) << lpad("headroom", 10) << "   keys" << Style::reset() << '\n';
    o << setting_row(s, Setting::BodyVel, "[ ]") << '\n';
    o << setting_row(s, Setting::BodyOmega, "{ }") << '\n';
    o << setting_row(s, Setting::VelocityLimit, "- =") << '\n';
    o << setting_row(s, Setting::AccelLimit, "_ +") << '\n';
    o << '\n';

    // Drive line
    o << Style::bold() << rpad(s.drive_word, 6) << Style::reset() << " twist(" << num(s.twist_req.vx, 2)
      << "," << num(s.twist_req.vy, 2) << "," << num(s.twist_req.w, 2) << ")  wheel cmd rev/s [";
    for (int i = 0; i < 4; ++i) o << (i ? " " : "") << num(cmd_wheels[i], 2);
    o << "]  peak " << num(peak_cmd, 2) << " / cap " << num(wheel_cap, 2) << "  ";
    if (!energized) {
        o << Style::yellow() << "DE-ENERGIZED (STOP frames)" << Style::reset();
    } else if (clip[0] != '\0') {
        o << Style::yellow() << "CLIPPED by " << clip << Style::reset();
    } else {
        o << Style::green() << "energized" << Style::reset();
    }
    o << '\n';

    Table tb({"id", "rep", "mode", "fault/limit", "cmd", "meas", "err", "pos", "q-cur(A)", "temp",
              "volt"});
    for (int i = 0; i < 4; ++i) {
        const int id = i + 1;
        const auto it = data.find(id);
        if (it == data.end()) {
            tb.row({std::to_string(id), std::string(Style::red()) + "NO" + Style::reset(), "--",
                    "--", num(cmd_wheels[i], 2), "--", "--", "--", "--", "--", "--"});
            continue;
        }
        const auto& m = it->second;
        const double err = (std::isfinite(m.velocity) && std::isfinite(cmd_wheels[i]))
                               ? m.velocity - cmd_wheels[i]
                               : kNaN;
        std::string mode_s, fault_s;
        {
            const auto mi = decode_mode(m.mode);
            mode_s = std::string(Style::of(mi.severity)) + mi.name + Style::reset();
            if (m.fault == 0) {
                fault_s = std::string(Style::green()) + "ok" + Style::reset();
            } else {
                const auto fi = decode_fault(m.fault);
                fault_s = std::string(Style::of(fi.severity)) + std::to_string(fi.code) + " " +
                          fi.name + Style::reset();
            }
        }
        tb.row({std::to_string(id), std::string(Style::green()) + "yes" + Style::reset(), mode_s,
                fault_s, num(cmd_wheels[i], 2), num(m.velocity, 2), num(err, 2),
                num(m.position, 1), num(m.current, 2), num(m.temperature, 0),
                num(m.voltage, 1)});
    }
    // Table prints straight to stdout; capture via a temporary buffer swap.
    {
        std::ostringstream cap;
        auto* old = std::cout.rdbuf(cap.rdbuf());
        tb.print();
        std::cout.rdbuf(old);
        o << cap.str();
    }

    if (telem.attitude_present) {
        o << "imu  rate(r,p,y)=(" << num(telem.imu_roll_dps, 1) << "," << num(telem.imu_pitch_dps, 1)
          << "," << num(telem.imu_yaw_dps, 1) << ") dps  heading " << num(telem.imu_heading_deg, 1)
          << " deg  accel(" << num(telem.imu_accel_x_mps2, 2) << ","
          << num(telem.imu_accel_y_mps2, 2) << "," << num(telem.imu_accel_z_mps2, 2) << ") m/s2\n";
    } else {
        o << Style::yellow() << "imu  no attitude sample this cycle" << Style::reset() << '\n';
    }

    // Window / status
    o << Style::grey() << "window since seq " << s.window.from_seq << ": " << s.window.drive_cycles
      << " drive cycles, peak meas " << num(s.window.peak_measured_wheel_rev_s, 2)
      << " rev/s, peak |q| " << num(s.window.peak_q_current_a, 2) << " A, faults "
      << s.window.faults_seen.size() << ", profiles written " << s.profiles_written
      << Style::reset() << '\n';

    if (s.hold) {
        o << Style::red() << Style::bold() << "HOLD: " << s.hold_reason
          << "   (SPACE to clear once the cause is gone"
          << (in_menu(s.mode) ? "; close the menu first" : "") << ")" << Style::reset() << '\n';
    }
    if (in_menu(s.mode)) {
        render_menu(o, s);  // shown even under HOLD so the operator can close it
    } else if (s.hold) {
        // the HOLD line above is the whole story
    } else if (s.mode == Mode::PromptNote) {
        o << Style::cyan() << "NOTE (Enter to record, empty Enter cancels): " << Style::reset()
          << s.prompt_buf << "_\n";
    } else if (s.mode == Mode::PromptSignName) {
        o << Style::cyan() << "SIGN-OFF 1/2  operator name (Enter; empty cancels): "
          << Style::reset() << s.prompt_buf << "_\n";
    } else if (s.mode == Mode::PromptSignNote) {
        o << Style::cyan() << "SIGN-OFF 2/2  observation note for " << s.signoff_name
          << " (Enter; empty cancels): " << Style::reset() << s.prompt_buf << "_\n";
    } else if (!s.message.empty() && Clock::now() < s.message_until) {
        o << Style::yellow() << s.message << Style::reset() << '\n';
    } else {
        o << '\n';
    }
    o << Style::grey()
      << "W/S fwd-back  A/D strafe  Q/E rotate  SPACE stop/clear  C ceilings  N note  P sign-off  X quit   "
      << Style::reset() << Style::red() << Style::bold() << "ESC ABORT" << Style::reset() << '\n';
    return o.str();
}

// ---------------------------------------------------------------------------
// The run
// ---------------------------------------------------------------------------

void say(Session& s, const std::string& msg, double seconds = 4.0) {
    s.message = msg;
    s.message_until = Clock::now() + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
}

void reset_window(Session& s, double t_mono) {
    s.window = ApprovalWindow{};
    s.window.from_seq = s.seq;
    s.window.from_mono_s = t_mono;
}

void enter_hold(Session& s, Sink& sink, double t_mono, const std::string& reason) {
    if (s.hold && s.hold_reason == reason) return;
    s.hold = true;
    s.hold_reason = reason;
    s.twist_req = BodyTwist{};
    s.drive_word = "HOLD";
    sink.push_line(encode_event(s.seq, t_mono, wall_now_s(), "hold",
                                {{"reason", json_string(reason)}}));
}

// ---------------------------------------------------------------------------
// The `C` ceiling menu
// ---------------------------------------------------------------------------

// Every ceiling change in the session goes through here, whatever asked for
// it (menu value, menu reset, or the test-only flag): apply with NO upper
// bound, refresh the override status, log an event that carries both
// ceilings and the ratio, reset the approval window (you must drive again
// under the new ceiling before you can sign it off), tell the operator.
void change_ceiling(Session& s, Sink& sink, double t_mono, double wall, Setting which,
                    double value, bool confirmed_over_threshold, const char* source) {
    const CeilingProposal p = propose_ceiling(which, value, s.derived_caps);
    const CeilingChange r = apply_ceiling(s.caps, s.settings, which, value);
    s.ov = override_status(s.caps, s.derived_caps);
    sink.push_line(encode_event(
        s.seq, t_mono, wall, "ceiling_change",
        {{"setting", json_string(setting_name(which))},
         {"source", json_string(source)},
         {"from", json_number(r.before)},
         {"to", json_number(r.after)},
         {"changed", json_bool(r.changed)},
         {"derived", json_number(p.derived)},
         {"above_derived", json_bool(p.above_derived)},
         {"ratio_to_derived", json_number(p.ratio)},
         {"confirmed_over_threshold", json_bool(confirmed_over_threshold)},
         {"setting_clamped", json_bool(r.setting_clamped)},
         {"setting_from", json_number(r.setting_before)},
         {"setting_to", json_number(r.setting_after)},
         {"ceiling", caps_to_json(s.caps)},
         {"ceiling_derived", caps_to_json(s.derived_caps)},
         {"ceiling_mode", json_string(s.ov.mode())},
         {"override_active", json_bool(s.ov.any_override)}}));
    if (r.changed) reset_window(s, t_mono);
    std::string msg = std::string(setting_label(which)) + " ceiling " + num(r.before, 3) + " -> " +
                      num(r.after, 3) + " " + setting_unit(which);
    if (p.above_derived) msg += "   OVERRIDE " + num(p.ratio, 1) + "x derived";
    if (r.setting_clamped) {
        msg += "   (setting pulled down " + num(r.setting_before, 2) + " -> " +
               num(r.setting_after, 2) + ")";
    }
    say(s, msg, 6.0);
}

// Keys while the menu is open. The loop holds the robot at zero for as long
// as in_menu(s.mode); this only routes keys and never touches twist_req.
void handle_menu_key(Session& s, Sink& sink, int k, double t_mono, double wall) {
    static constexpr Setting kMenuOrder[] = {Setting::VelocityLimit, Setting::AccelLimit,
                                             Setting::BodyVel, Setting::BodyOmega};
    if (s.mode == Mode::MenuSelect) {
        switch (k) {
            case '1': case '2': case '3': case '4':
                s.menu_setting = kMenuOrder[k - '1'];
                s.prompt_buf.clear();
                s.mode = Mode::MenuEdit;
                break;
            case 'r': case 'R':
                for (Setting which : kAllSettings) {
                    change_ceiling(s, sink, t_mono, wall, which, get_cap(s.derived_caps, which),
                                   false, "menu_reset");
                }
                say(s, "all four ceilings reset to derived", 4.0);
                break;
            case 'c': case 'C': case '\r': case '\n':
                s.mode = Mode::Drive;
                sink.push_line(encode_event(s.seq, t_mono, wall, "ceiling_menu_closed",
                                            {{"ceiling", caps_to_json(s.caps)},
                                             {"ceiling_mode", json_string(s.ov.mode())},
                                             {"override_active", json_bool(s.ov.any_override)}}));
                say(s, "ceiling menu closed; robot is stopped - press a direction key to drive", 4.0);
                break;
            default: break;
        }
        return;
    }
    if (s.mode == Mode::MenuEdit) {
        if (k == '\r' || k == '\n') {
            const std::string text = s.prompt_buf;
            s.prompt_buf.clear();
            if (text.find_first_not_of(" \t") == std::string::npos) {
                s.mode = Mode::MenuSelect;
                say(s, "cancelled; ceiling unchanged", 3.0);
                return;
            }
            const CeilingProposal p = propose_ceiling_text(s.menu_setting, text, s.derived_caps);
            if (!p.valid) {
                sink.push_line(encode_event(s.seq, t_mono, wall, "ceiling_refused",
                                            {{"setting", json_string(setting_name(s.menu_setting))},
                                             {"text", json_string(text)},
                                             {"reason", json_string(p.error)}}));
                say(s, "refused: " + p.error + " - type a positive number", 5.0);
                return;  // stay in the editor
            }
            if (p.needs_confirm()) {
                // Warn at the keystroke, not later: the value is not applied
                // until Y is pressed on the next screen.
                s.menu_proposal = p;
                s.mode = Mode::MenuConfirm;
                sink.push_line(encode_event(s.seq, t_mono, wall, "ceiling_warned",
                                            {{"setting", json_string(setting_name(p.which))},
                                             {"value", json_number(p.value)},
                                             {"derived", json_number(p.derived)},
                                             {"ratio_to_derived", json_number(p.ratio)}}));
                return;
            }
            change_ceiling(s, sink, t_mono, wall, p.which, p.value, false, "menu");
            s.mode = Mode::MenuSelect;
        } else if (k == 127 || k == 8) {
            if (!s.prompt_buf.empty()) s.prompt_buf.pop_back();
        } else if (k == 21) {  // Ctrl-U
            s.prompt_buf.clear();
        } else if (k >= 32 && k < 127 && s.prompt_buf.size() < 32) {
            s.prompt_buf += static_cast<char>(k);
        }
        return;
    }
    if (s.mode == Mode::MenuConfirm) {
        const CeilingProposal p = s.menu_proposal;
        if (k == 'y' || k == 'Y') {
            change_ceiling(s, sink, t_mono, wall, p.which, p.value, true, "menu");
        } else {
            sink.push_line(encode_event(s.seq, t_mono, wall, "ceiling_declined",
                                        {{"setting", json_string(setting_name(p.which))},
                                         {"value", json_number(p.value)},
                                         {"derived", json_number(p.derived)},
                                         {"ratio_to_derived", json_number(p.ratio)}}));
            say(s, "cancelled; " + std::string(setting_label(p.which)) + " ceiling unchanged at " +
                       num(get_cap(s.caps, p.which), 3) + " " + setting_unit(p.which), 4.0);
        }
        s.mode = Mode::MenuSelect;
        return;
    }
}

}  // namespace

void print_help() {
    std::cout << Style::bold() << "debugger commission" << Style::reset()
              << " — supervised per-robot limit commissioning. " << Style::red() << Style::bold()
              << "MOVES THE ROBOT." << Style::reset() << "\n\n"
              << Style::bold() << "USAGE\n" << Style::reset()
              << "  sudo ./debugger commission [--config-dir <dir>] [--stream-to <ip:port>]\n"
                 "                             [--log-dir <dir>] [--dry-run] [--yes] [--no-color]\n\n"
              << Style::bold() << "WHAT IT DOES\n" << Style::reset()
              << "  Drives the robot by keyboard through the calibrated kinematics at the\n"
                 "  configured Motor_interval, while you raise or lower the moteus per-command\n"
                 "  velocity_limit / accel_limit and the body-speed envelope. Every cycle is\n"
                 "  logged to NDJSON on the Pi (authoritative) and optionally streamed by UDP\n"
                 "  (best effort, never blocks the loop). A sign-off writes a signed profile\n"
                 "  under config/robot_profiles/ — the only write under config/.\n\n"
              << Style::bold() << "CEILING\n" << Style::reset()
              << "  DERIVED at start: 80% of the smallest readable backstop - each controller's\n"
                 "  live servo.default_velocity_limit / default_accel_limit (Motor.yaml's\n"
                 "  velocityLimit / accelLimit standing in where a controller has none) and\n"
                 "  servo.max_velocity. Printed with its derivation before the first keypress.\n"
                 "  The -/= _/+ [/] {/} keys can never pass the ceiling in force.\n"
                 "  The C menu sets any of the four ceilings to ANY positive value, session\n"
                 "  only (nothing is written to disk; every run starts derived). A value above\n"
                 "  the derived one is warned at the keystroke and needs an explicit Y. While\n"
                 "  an override is in force the screen carries a persistent [OVERRIDE] marker,\n"
                 "  every record carries both ceilings, and a sign-off is stamped\n"
                 "  approved_under_override: true.\n\n"
              << Style::bold() << "KEYS\n" << Style::reset()
              << "  W / S       forward / back            A / D   strafe left / right\n"
                 "  Q / E       rotate CCW / CW           SPACE   stop (also clears a HOLD)\n"
                 "  [ / ]       body speed  -/+ 0.10 m/s  { / }   body yaw rate -/+ 0.25 rad/s\n"
                 "  - / =       moteus velocity_limit -/+ 0.5 rev/s\n"
                 "  _ / +       moteus accel_limit    -/+ 1.0 rev/s^2\n"
                 "  C           ceiling menu (robot held at zero while open):\n"
                 "                1-4 pick a ceiling, type a value, Enter; Y confirms an\n"
                 "                over-threshold value; R resets all to derived; C closes\n"
                 "  N           record a note (typed, Enter)\n"
                 "  P           sign off: name, note -> config/robot_profiles/profile_*.json\n"
                 "  X           quit (stops motors, closes the log)\n"
                 "  ESC         ABORT: zero twist, SetStop on every controller, exit\n\n"
              << Style::bold() << "OPTIONS\n" << Style::reset()
              << "  --stream-to <ip:port>  also send every record by UDP (tools/telemetry_receiver.py)\n"
                 "  --log-dir <dir>        where the NDJSON log goes (default <repo>/logs/commission)\n"
                 "  --dry-run              print identity, ceiling and banner, then exit; no drive loop\n"
                 "  --yes                  skip the typed confirmation (only if you checked the robot)\n\n"
              << Style::grey()
              << "Requires sudo (pi3hat) and robotframework.service stopped. Missing telemetry\n"
                 "is logged as null, never 0. The moteus per-command watchdog stays armed.\n"
              << Style::reset();
}

int run(const RunOptions& opt) {
    Session s;
    s.opt = opt;
    s.config_dir = opt.config_dir;
    s.profile_dir = (std::filesystem::path(opt.config_dir) / "robot_profiles").string();

    // ---- identity: Pi ----------------------------------------------------
    s.pi_serial = parse_cpuinfo_serial(read_file("/proc/cpuinfo"));
    if (s.pi_serial.empty()) {
        std::cerr << Style::red() << "error: " << Style::reset()
                  << "/proc/cpuinfo has no Serial line; a profile cannot be keyed. Refusing.\n";
        return 2;
    }

    // ---- preflight ---------------------------------------------------------
    // The pi3hat is driven through /dev/mem, which needs root, and an mmap of
    // /dev/mem is NOT exclusive: a second process would silently share the
    // SPI pins with the running service and corrupt both. The transport
    // opens the device on its own thread, so a failure there is not
    // catchable here; check first and refuse with a plain message.
    if (::geteuid() != 0) {
        std::cerr << Style::red() << "error: " << Style::reset()
                  << "commission talks to the pi3hat, which needs /dev/mem. Run it with sudo.\n";
        return 2;
    }
    {
        const int st = std::system("systemctl is-active --quiet robotframework.service 2>/dev/null");
        if (st != -1 && WIFEXITED(st) && WEXITSTATUS(st) == 0) {
            std::cerr << Style::red() << "error: " << Style::reset()
                      << "robotframework.service is running and holds the pi3hat. Two processes must\n"
                         "       never share it. Stop it first, and start it again afterwards:\n"
                         "         sudo systemctl stop robotframework.service\n"
                         "         sudo systemctl start robotframework.service\n";
            return 2;
        }
    }

    if (opt.motor_map.size() != 4) {
        std::cerr << Style::red() << "error: " << Style::reset()
                  << "commission needs exactly the four drive controllers in motorMap (found "
                  << opt.motor_map.size() << ").\n";
        return 2;
    }
    for (int id = 1; id <= 4; ++id) {
        if (opt.motor_map.find(id) == opt.motor_map.end()) {
            std::cerr << Style::red() << "error: " << Style::reset() << "motorMap lacks id " << id
                      << "; Kinematics assumes ids 1..4 = FR,RR,RL,FL.\n";
            return 2;
        }
    }

    const SafetyYaml safety = load_safety(opt.config_dir, opt.motor_interval_ms);

    // ---- kinematics (the runtime's own loader, cross-checked) ------------
    s.kin = Wheel_math().kinematics();
    if (std::isfinite(opt.meters_per_motor_rev) &&
        std::abs(s.kin.meters_per_motor_rev - opt.meters_per_motor_rev) > 1e-9) {
        std::cerr << Style::red() << "error: " << Style::reset()
                  << "Wheel_math loaded metersPerMotorRev=" << num(s.kin.meters_per_motor_rev, 5)
                  << " but Motor.yaml says " << num(opt.meters_per_motor_rev, 5)
                  << "; the runtime loader fell back to a default. Refusing.\n";
        return 2;
    }

    // ---- hardware --------------------------------------------------------
    std::cout << Style::grey() << "opening pi3hat and controllers (STOP frames only)..."
              << Style::reset() << std::endl;
    Telemetry telem;
    StopGuard stop_guard(telem);

    if (!(telem.watchdog_timeout_s > 0.0) || !std::isfinite(telem.watchdog_timeout_s) ||
        telem.watchdog_timeout_s > kMaxWatchdogS) {
        std::cerr << Style::red() << "error: " << Style::reset()
                  << "moteus watchdog_timeout is " << num(telem.watchdog_timeout_s, 3)
                  << " s; it must be finite, > 0 and <= " << kMaxWatchdogS
                  << " s. Fix config/Safety.yaml watchdogTimeout. Refusing.\n";
        return 2;
    }

    const auto first = telem.cycle({}, /*energize=*/false);
    std::set<int> present;
    for (const auto& [id, m] : first) { (void)m; present.insert(id); }
    bool all_present = true;
    for (const auto& [id, bus] : opt.motor_map) {
        if (present.count(id) == 0) {
            std::cerr << Style::red() << "MISSING " << Style::reset() << "controller " << id
                      << " (bus " << bus << ") did not answer a STOP query.\n";
            all_present = false;
        }
    }
    if (!all_present) {
        std::cerr << "Refusing: commissioning needs every drive controller present.\n";
        return 1;
    }

    // ---- identity: controllers ------------------------------------------
    std::cout << Style::grey() << "reading controller identity and backstops..." << Style::reset()
              << std::endl;
    s.controllers = read_identities(telem, opt.motor_map);
    const auto backstops = read_backstops(telem, opt.motor_map, present);

    std::cout << '\n' << Style::bold() << "identity" << Style::reset() << '\n';
    std::cout << "  pi cpu serial   " << s.pi_serial << '\n';
    {
        Table tb({"id", "bus", "serial", "git hash", "dirty", "abi(0x101)", "regmap", "hwrev"});
        for (const auto& c : s.controllers) {
            std::ostringstream abi;
            if (c.abi_version >= 0) abi << "0x" << std::hex << c.abi_version;
            else abi << "--";
            tb.row({std::to_string(c.id), std::to_string(c.bus),
                    c.serial.empty() ? std::string(Style::red()) + "unread" + Style::reset()
                                     : c.serial,
                    c.git_hash.empty() ? std::string(Style::yellow()) + "unread" + Style::reset()
                                       : c.git_hash,
                    c.git_dirty ? std::string(Style::yellow()) + "YES" + Style::reset() : "no",
                    abi.str(), c.register_map >= 0 ? std::to_string(c.register_map) : "--",
                    c.hwrev.empty() ? "--" : c.hwrev});
        }
        tb.print();
    }
    {
        std::set<std::string> hashes;
        for (const auto& c : s.controllers) if (!c.git_hash.empty()) hashes.insert(c.git_hash);
        if (hashes.size() > 1) {
            std::cout << Style::yellow() << "note: " << hashes.size()
                      << " different firmware builds across the four controllers; the profile "
                         "records each.\n" << Style::reset();
        }
        for (const auto& c : s.controllers) {
            if (c.serial.empty()) {
                std::cerr << Style::red() << "error: " << Style::reset() << "controller " << c.id
                          << " serial number is unreadable; the profile cannot identify the "
                             "drivetrain. Refusing.\n";
                return 1;
            }
        }
    }

    // ---- ceiling ---------------------------------------------------------
    CeilingInputs cin;
    cin.controllers = backstops;
    cin.yaml_velocity_limit_rev_s = opt.yaml_velocity_limit_rev_s;
    cin.yaml_accel_limit_rev_s2 = opt.yaml_accel_limit_rev_s2;
    s.ceiling = derive_ceiling(cin);
    std::cout << '\n' << Style::bold() << "ceiling derivation" << Style::reset() << '\n';
    for (const auto& line : s.ceiling.derivation) std::cout << "  " << line << '\n';
    if (!s.ceiling.ok) {
        std::cerr << Style::red() << "error: " << Style::reset() << s.ceiling.error << ". Refusing.\n";
        return 1;
    }
    s.body_caps = derive_body_caps(s.kin, s.ceiling.velocity_limit_rev_s,
                                   safety.cfg.capped_linear_mps, safety.cfg.capped_angular_rps);
    s.caps.velocity_limit_rev_s = s.ceiling.velocity_limit_rev_s;
    s.caps.accel_limit_rev_s2 = s.ceiling.accel_limit_rev_s2;
    s.caps.body_vel_mps = s.body_caps.linear_mps;
    s.caps.body_omega_radps = s.body_caps.angular_radps;
    std::cout << "  body speed cap   = " << num(s.caps.body_vel_mps, 3) << " m/s   [" << s.body_caps.linear_source
              << "]\n"
              << "  body yaw cap     = " << num(s.caps.body_omega_radps, 3) << " rad/s [" << s.body_caps.angular_source
              << "]\n"
              << "  (worst-direction wheel gain " << num(s.body_caps.worst_linear_gain_rev_s_per_mps, 3)
              << " rev/s per m/s, yaw gain " << num(s.body_caps.angular_gain_rev_s_per_radps, 3)
              << " rev/s per rad/s, through the calibrated kinematics)\n";
    if (!safety.loaded) {
        std::cout << Style::yellow() << "  Safety.yaml did not load; envelope caps are the supervisor defaults.\n"
                  << Style::reset();
    }

    // The derived ceiling is fixed from here on; the ceiling in force starts
    // equal to it and only the `C` menu (or, in a dry run, --test-ceiling)
    // moves it.
    s.derived_caps = s.caps;
    s.max_velocity_note = describe_max_velocity(backstops);
    s.trip_current_a = safety.cfg.trip_current_a;

    if (!opt.test_ceiling.empty()) {
        // Test-only: lets the startup print be verified on hardware in every
        // ceiling state without a keyboard. Refused outside --dry-run so the
        // menu, with its warning and Y, stays the only way to change a
        // ceiling in a session that can move the robot.
        if (!opt.dry_run) {
            std::cerr << Style::red() << "error: " << Style::reset()
                      << "--test-ceiling is a test-only flag and needs --dry-run. In a live session "
                         "the C menu is the only way to set a ceiling.\n";
            return 2;
        }
        std::vector<std::pair<Setting, double>> items;
        std::string err;
        if (!parse_ceiling_spec(opt.test_ceiling, &items, &err)) {
            std::cerr << Style::red() << "error: " << Style::reset() << "--test-ceiling: " << err
                      << '\n';
            return 2;
        }
        std::cout << '\n' << Style::yellow() << "--test-ceiling (dry run only):" << Style::reset()
                  << '\n';
        for (const auto& [which, v] : items) {
            const CeilingProposal p = propose_ceiling(which, v, s.derived_caps);
            const CeilingChange r = apply_ceiling(s.caps, s.settings, which, v);
            std::cout << "  " << setting_label(which) << " ceiling " << num(r.before, 3) << " -> "
                      << num(r.after, 3) << ' ' << setting_unit(which);
            if (p.above_derived) {
                std::cout << "   " << Style::red() << Style::bold() << "ABOVE DERIVED "
                          << num(p.ratio, 1) << "x" << Style::reset();
            } else if (p.valid && std::abs(p.value - p.derived) > 1e-9) {
                std::cout << "   " << Style::cyan() << "below derived" << Style::reset();
            }
            std::cout << '\n';
        }
    }
    s.ov = override_status(s.caps, s.derived_caps);
    print_ceiling_block(s);

    // ---- starting settings ----------------------------------------------
    // Start at half the DERIVED ceiling (or the configured value if lower):
    // a commissioning run ramps UP from something known-gentle, and that
    // starting point does not move with an override. clamp_settings then
    // guarantees nothing starts above the ceiling in force.
    s.settings.velocity_limit_rev_s =
        std::min(std::isfinite(opt.yaml_velocity_limit_rev_s) ? opt.yaml_velocity_limit_rev_s : 1e9,
                 std::floor(0.5 * s.derived_caps.velocity_limit_rev_s / 0.5) * 0.5);
    s.settings.accel_limit_rev_s2 =
        std::min(std::isfinite(opt.yaml_accel_limit_rev_s2) ? opt.yaml_accel_limit_rev_s2 : 1e9,
                 std::floor(0.5 * s.derived_caps.accel_limit_rev_s2));
    s.settings.body_vel_mps = 0.30;
    s.settings.body_omega_radps = 1.00;
    clamp_settings(s.settings, s.caps);
    std::cout << "starting settings  body " << num(s.settings.body_vel_mps, 2) << " m/s, "
              << num(s.settings.body_omega_radps, 2) << " rad/s   velocity_limit "
              << num(s.settings.velocity_limit_rev_s, 2) << " rev/s   accel_limit "
              << num(s.settings.accel_limit_rev_s2, 2) << " rev/s^2\n";

    // ---- prior profile ---------------------------------------------------
    const PriorProfile prior = find_prior_profile(s.profile_dir, s.pi_serial, s.controllers);
    std::cout << '\n';
    for (const auto& u : prior.unreadable) {
        std::cout << Style::yellow() << "  unreadable profile ignored: " << u << Style::reset() << '\n';
    }
    if (!prior.found) {
        std::cout << Style::grey() << "no prior profile for pi " << s.pi_serial << " in "
                  << s.profile_dir << Style::reset() << '\n';
    } else {
        std::cout << "prior profile " << prior.path << "\n  approved " << prior.profile.approved_utc
                  << " by " << prior.profile.operator_name << ": velocity_limit "
                  << num(prior.profile.approved.velocity_limit_rev_s, 2) << " rev/s, accel_limit "
                  << num(prior.profile.approved.accel_limit_rev_s2, 2) << " rev/s^2, body "
                  << num(prior.profile.approved.body_vel_mps, 2) << " m/s\n";
        if (prior.mismatches.empty()) {
            std::cout << Style::green() << "  drivetrain matches the approved profile.\n" << Style::reset();
        } else {
            s.prior_mismatch = true;
            std::cout << Style::red() << Style::bold()
                      << "\n  ******************************************************************\n"
                         "  *  WARNING: THE DRIVETRAIN HAS CHANGED SINCE THESE LIMITS WERE     *\n"
                         "  *  APPROVED. The prior profile does not describe this robot.       *\n"
                         "  ******************************************************************\n"
                      << Style::reset();
            for (const auto& m : prior.mismatches) {
                std::cout << Style::red() << "  - " << m << Style::reset() << '\n';
            }
            std::cout << '\n';
        }
    }

    // ---- banner ----------------------------------------------------------
    std::cout << '\n' << Style::red() << Style::bold()
              << "==================================================================\n"
                 "  THIS TOOL MOVES THE ROBOT. Wheels will turn at up to "
              << num(s.caps.body_vel_mps, 2) << " m/s.\n"
                 "  Clear the area. Keep a hand near ESC. The robot has no vision\n"
                 "  and no field awareness here: it goes exactly where you send it.\n"
                 "==================================================================\n"
              << Style::reset()
              << "  tick " << opt.motor_interval_ms << " ms   watchdog " << num(telem.watchdog_timeout_s, 3)
              << " s   trips: |q| >= " << num(safety.cfg.trip_current_a, 1) << " A for "
              << static_cast<int>(safety.current_grace_ms) << " ms, temp >= "
              << num(safety.cfg.trip_temp_c, 0) << " C, bus < " << num(safety.cfg.min_bus_voltage, 1)
              << " V, any hard fault, " << kMissingReplyHoldCycles << " missed replies, position_timeout\n"
              << "  keys: W/S A/D Q/E drive  SPACE stop  [ ] { } body  - = velocity_limit  _ + accel_limit\n"
                 "        C ceilings  N note  P sign-off  X quit  ESC ABORT\n";

    if (opt.dry_run) {
        std::cout << '\n' << Style::green() << "DRY RUN" << Style::reset()
                  << ": exiting before the drive loop. No position command was sent; only STOP "
                     "and query frames.\n";
        return 0;
    }
    if (!confirm_yes("Is the area clear and is someone at the keyboard ready to press ESC?",
                     opt.assume_yes)) {
        std::cout << "aborted before any motion.\n";
        return 1;
    }

    // ---- log -------------------------------------------------------------
    std::string log_dir = opt.log_dir;
    if (log_dir.empty()) {
        log_dir = (std::filesystem::path(opt.config_dir).parent_path() / "logs" / "commission").string();
    }
    {
        std::error_code ec;
        std::filesystem::create_directories(log_dir, ec);
        if (ec) {
            std::cerr << Style::red() << "error: " << Style::reset() << "cannot create " << log_dir
                      << ": " << ec.message() << '\n';
            return 1;
        }
        chown_to_invoker(log_dir);
    }
    s.log_path = (std::filesystem::path(log_dir) /
                  ("commission_" + s.pi_serial + "_" + utc_stamp_compact() + ".ndjson")).string();
    Sink sink;
    {
        std::string err;
        if (!sink.open(s.log_path, opt.stream_to, &err)) {
            std::cerr << Style::red() << "error: " << Style::reset() << err << '\n';
            return 1;
        }
        chown_to_invoker(s.log_path);
    }

    s.t0 = Clock::now();
    {
        // Header record: everything a reader needs to interpret the cycles.
        std::string ctl = "[";
        for (std::size_t i = 0; i < s.controllers.size(); ++i) {
            const auto& c = s.controllers[i];
            if (i) ctl += ',';
            ctl += "{\"id\":" + std::to_string(c.id) + ",\"bus\":" + std::to_string(c.bus) +
                   ",\"serial\":" + json_string(c.serial) + ",\"git_hash\":" + json_string(c.git_hash) +
                   ",\"git_dirty\":" + json_bool(c.git_dirty) +
                   ",\"abi_version\":" + std::to_string(c.abi_version) +
                   ",\"register_map\":" + std::to_string(c.register_map) + "}";
        }
        ctl += "]";
        sink.push_line(encode_event(
            0, 0.0, wall_now_s(), "header",
            {{"schema", json_int(kSchemaVersion)},
             {"tool", json_string(kToolVersion)},
             {"pi_serial", json_string(s.pi_serial)},
             {"robot_id", json_int(opt.robot_id)},
             {"controllers", ctl},
             {"ceiling", caps_to_json(s.caps)},              // in force at the start
             {"ceiling_derived", caps_to_json(s.derived_caps)},
             {"ceiling_mode", json_string(s.ov.mode())},
             {"override_active", json_bool(s.ov.any_override)},
             {"ceiling_settable", json_string("C menu, session only, no upper bound; see ceiling_change events")},
             {"ceiling_fraction", json_number(kCeilingFraction)},
             {"ceiling_velocity_source", json_string(s.ceiling.velocity_source)},
             {"ceiling_accel_source", json_string(s.ceiling.accel_source)},
             {"ceiling_derivation", json_string_array(s.ceiling.derivation)},
             {"settings_initial", settings_to_json(s.settings)},
             {"yaml_velocity_limit_rev_s", json_number(opt.yaml_velocity_limit_rev_s)},
             {"yaml_accel_limit_rev_s2", json_number(opt.yaml_accel_limit_rev_s2)},
             {"meters_per_motor_rev", json_number(s.kin.meters_per_motor_rev)},
             {"tick_ms", json_int(opt.motor_interval_ms)},
             {"watchdog_s", json_number(telem.watchdog_timeout_s)},
             {"trip_current_a", json_number(safety.cfg.trip_current_a)},
             {"trip_temp_c", json_number(safety.cfg.trip_temp_c)},
             {"min_bus_voltage", json_number(safety.cfg.min_bus_voltage)},
             {"config_dir", json_string(opt.config_dir)},
             {"log_path", json_string(s.log_path)},
             {"stream_to", json_string(opt.stream_to)},
             {"prior_profile", json_string(prior.found ? prior.path : "")},
             {"prior_profile_mismatches", json_string_array(prior.mismatches)},
             {"wheel_order", json_string("CAN id 1..4 = FR,RR,RL,FL")},
             {"units", json_string("wheels rev/s (output), q_current A phase, twist m/s m/s rad/s body frame +x fwd +y left")}}));
    }

    // ---- loop ------------------------------------------------------------
    rf::Supervisor sup(safety.cfg);
    install_signal_handlers();
    std::signal(SIGHUP, on_hangup);
    std::signal(SIGPIPE, SIG_IGN);

    const auto period = std::chrono::milliseconds(std::max(1, opt.motor_interval_ms));
    auto next = Clock::now();
    auto last_cycle = next;
    FrameWriter frame;
    std::string exit_reason = "quit";
    reset_window(s, 0.0);

    std::array<double, 4> cmd_wheels{{kNaN, kNaN, kNaN, kNaN}};
    double last_dt = 0.0;
    std::map<int, MotorTelemetry> data;

    {
        RawTerminal raw;
        while (true) {
            const auto now = Clock::now();
            const double t_mono = std::chrono::duration<double>(now - s.t0).count();
            const double wall = wall_now_s();
            last_dt = std::chrono::duration<double>(now - last_cycle).count();
            last_cycle = now;
            ++s.seq;

            if (must_stop()) {
                exit_reason = "abort";
                sink.push_line(encode_event(s.seq, t_mono, wall, "abort",
                                            {{"reason", json_string(g_hangup.load() ? "SIGHUP" : "signal")}}));
                break;
            }

            // ---- keys ---------------------------------------------------
            bool abort = false, quit = false;
            for (int k = read_key(); k != -1; k = read_key()) {
                if (k == 27) { abort = true; break; }
                if (k == 3) { abort = true; break; }
                if (in_menu(s.mode)) {
                    handle_menu_key(s, sink, k, t_mono, wall);
                    continue;
                }
                if (s.mode != Mode::Drive) {
                    // line editor
                    if (k == '\r' || k == '\n') {
                        const std::string text = s.prompt_buf;
                        s.prompt_buf.clear();
                        if (s.mode == Mode::PromptNote) {
                            if (text.empty()) {
                                say(s, "note cancelled");
                            } else {
                                sink.push_line(encode_event(s.seq, t_mono, wall, "note",
                                                            {{"text", json_string(text)}}));
                                say(s, "note recorded");
                            }
                            s.mode = Mode::Drive;
                        } else if (s.mode == Mode::PromptSignName) {
                            if (text.empty()) {
                                sink.push_line(encode_event(s.seq, t_mono, wall, "signoff_refused",
                                                            {{"reason", json_string("no operator name")}}));
                                say(s, "sign-off refused: an operator name is required");
                                s.mode = Mode::Drive;
                            } else {
                                s.signoff_name = text;
                                s.mode = Mode::PromptSignNote;
                            }
                        } else if (s.mode == Mode::PromptSignNote) {
                            s.mode = Mode::Drive;
                            if (text.empty()) {
                                sink.push_line(encode_event(s.seq, t_mono, wall, "signoff_refused",
                                                            {{"reason", json_string("no observation note")}}));
                                say(s, "sign-off refused: an observation note is required");
                            } else if (s.hold) {
                                sink.push_line(encode_event(s.seq, t_mono, wall, "signoff_refused",
                                                            {{"reason", json_string("hold active: " + s.hold_reason)}}));
                                say(s, "sign-off refused: a HOLD is active (" + s.hold_reason + ")");
                            } else if (s.window.drive_cycles == 0) {
                                sink.push_line(encode_event(s.seq, t_mono, wall, "signoff_refused",
                                                            {{"reason", json_string("no drive cycles at these settings")}}));
                                say(s, "sign-off refused: you have not driven at these settings yet");
                            } else {
                                Profile p;
                                p.pi_serial = s.pi_serial;
                                p.controllers = s.controllers;
                                p.approved = s.settings;
                                p.caps = s.caps;
                                p.caps_derived = s.derived_caps;
                                p.ceiling_mode = s.ov.mode();
                                p.approved_under_override = s.ov.any_override;
                                p.ceiling_velocity_source = s.ceiling.velocity_source;
                                p.ceiling_accel_source = s.ceiling.accel_source;
                                p.ceiling_derivation = s.ceiling.derivation;
                                p.yaml_velocity_limit_rev_s = opt.yaml_velocity_limit_rev_s;
                                p.yaml_accel_limit_rev_s2 = opt.yaml_accel_limit_rev_s2;
                                p.meters_per_motor_rev = s.kin.meters_per_motor_rev;
                                p.operator_name = s.signoff_name;
                                p.note = text;
                                p.approved_utc = iso8601_utc(wall);
                                p.log_path = s.log_path;
                                p.config_dir = opt.config_dir;
                                p.window = s.window;
                                p.window.to_seq = s.seq;
                                p.window.to_mono_s = t_mono;
                                std::error_code ec;
                                std::filesystem::create_directories(s.profile_dir, ec);
                                chown_to_invoker(s.profile_dir);
                                const std::string path =
                                    (std::filesystem::path(s.profile_dir) /
                                     ("profile_" + s.pi_serial + "_" + utc_stamp_compact() + ".json")).string();
                                bool ok = !ec;
                                if (ok) {
                                    std::ofstream out(path, std::ios::out | std::ios::trunc);
                                    out << encode_profile(p);
                                    ok = static_cast<bool>(out);
                                    out.close();
                                    chown_to_invoker(path);
                                }
                                if (ok) {
                                    ++s.profiles_written;
                                    s.last_profile_path = path;
                                    sink.push_line(encode_event(
                                        s.seq, t_mono, wall, "signoff",
                                        {{"operator", json_string(s.signoff_name)},
                                         {"note", json_string(text)},
                                         {"profile_path", json_string(path)},
                                         {"approved", settings_to_json(s.settings)},
                                         {"ceiling", caps_to_json(s.caps)},
                                         {"ceiling_derived", caps_to_json(s.derived_caps)},
                                         {"ceiling_mode", json_string(s.ov.mode())},
                                         {"approved_under_override", json_bool(s.ov.any_override)},
                                         {"window_from_seq", json_int(static_cast<int64_t>(p.window.from_seq))},
                                         {"window_to_seq", json_int(static_cast<int64_t>(p.window.to_seq))},
                                         {"window_drive_cycles", json_int(static_cast<int64_t>(p.window.drive_cycles))}}));
                                    say(s, std::string(s.ov.any_override ? "SIGNED OFF UNDER OVERRIDE -> "
                                                                         : "SIGNED OFF -> ") + path, 8.0);
                                } else {
                                    sink.push_line(encode_event(s.seq, t_mono, wall, "signoff_failed",
                                                                {{"path", json_string(path)}}));
                                    say(s, "sign-off FAILED: could not write " + path, 8.0);
                                }
                            }
                        }
                    } else if (k == 127 || k == 8) {
                        if (!s.prompt_buf.empty()) s.prompt_buf.pop_back();
                    } else if (k == 21) {  // Ctrl-U
                        s.prompt_buf.clear();
                    } else if (k >= 32 && k < 127 && s.prompt_buf.size() < 160) {
                        s.prompt_buf += static_cast<char>(k);
                    }
                    continue;
                }

                // ---- drive-mode keys ----
                auto set_dir = [&](double vx, double vy, double w, const char* word) {
                    if (s.hold) {
                        say(s, "HOLD is active: press SPACE to clear it first");
                        return;
                    }
                    s.twist_req = BodyTwist{vx, vy, w};
                    s.drive_word = word;
                };
                auto adjust = [&](Setting which, int dir) {
                    const StepResult r = step_setting(s.settings, which, dir, s.caps);
                    if (r.changed) {
                        sink.push_line(encode_event(
                            s.seq, t_mono, wall, "limit_change",
                            {{"setting", json_string(setting_name(which))},
                             {"from", json_number(r.before)},
                             {"to", json_number(r.after)},
                             {"cap", json_number(get_cap(s.caps, which))},
                             {"at_ceiling", json_bool(r.at_ceiling)}}));
                        reset_window(s, t_mono);
                        // A latched direction keeps its magnitude in step with
                        // the envelope.
                        if (which == Setting::BodyVel) {
                            if (s.twist_req.vx != 0.0)
                                s.twist_req.vx = (s.twist_req.vx > 0 ? 1 : -1) * s.settings.body_vel_mps;
                            if (s.twist_req.vy != 0.0)
                                s.twist_req.vy = (s.twist_req.vy > 0 ? 1 : -1) * s.settings.body_vel_mps;
                        } else if (which == Setting::BodyOmega) {
                            if (s.twist_req.w != 0.0)
                                s.twist_req.w = (s.twist_req.w > 0 ? 1 : -1) * s.settings.body_omega_radps;
                        }
                    }
                    if (r.at_ceiling) {
                        const auto& ov = s.ov.of(which);
                        const std::string src = ov.manual
                            ? std::string("set in the C menu; derived ") + num(ov.derived, 2) + " " +
                                  setting_unit(which)
                            : ceiling_source(s, which);
                        say(s, std::string("CEILING: ") + setting_label(which) + " cannot exceed " +
                                   num(get_cap(s.caps, which), 2) + " " + setting_unit(which) +
                                   "  [" + src + "]  (C to change the ceiling)", 5.0);
                        if (!r.changed) {
                            sink.push_line(encode_event(
                                s.seq, t_mono, wall, "limit_refused",
                                {{"setting", json_string(setting_name(which))},
                                 {"value", json_number(r.before)},
                                 {"cap", json_number(get_cap(s.caps, which))}}));
                        }
                    } else if (r.at_floor) {
                        say(s, std::string(setting_label(which)) + " is at its floor");
                    } else if (r.changed) {
                        say(s, std::string(setting_label(which)) + " " + num(r.before, 2) + " -> " +
                                   num(r.after, 2) + " " + setting_unit(which) + "   (headroom " +
                                   num(get_cap(s.caps, which) - r.after, 2) + ")", 3.0);
                    }
                };
                switch (k) {
                    case 'w': case 'W': set_dir(s.settings.body_vel_mps, 0, 0, "FWD"); break;
                    case 's': case 'S': set_dir(-s.settings.body_vel_mps, 0, 0, "BACK"); break;
                    case 'a': case 'A': set_dir(0, s.settings.body_vel_mps, 0, "LEFT"); break;
                    case 'd': case 'D': set_dir(0, -s.settings.body_vel_mps, 0, "RIGHT"); break;
                    case 'q': case 'Q': set_dir(0, 0, s.settings.body_omega_radps, "CCW"); break;
                    case 'e': case 'E': set_dir(0, 0, -s.settings.body_omega_radps, "CW"); break;
                    case ' ':
                        s.twist_req = BodyTwist{};
                        s.drive_word = "STOP";
                        if (s.hold) {
                            s.hold = false;
                            s.hold_reason.clear();
                            sup.clear();
                            s.missing_streak.clear();
                            sink.push_line(encode_event(s.seq, t_mono, wall, "hold_cleared", {}));
                            say(s, "hold cleared; motors hold zero again - press a direction key to drive");
                            reset_window(s, t_mono);
                        }
                        break;
                    case '[': adjust(Setting::BodyVel, -1); break;
                    case ']': adjust(Setting::BodyVel, +1); break;
                    case '{': adjust(Setting::BodyOmega, -1); break;
                    case '}': adjust(Setting::BodyOmega, +1); break;
                    case '-': adjust(Setting::VelocityLimit, -1); break;
                    case '=': adjust(Setting::VelocityLimit, +1); break;
                    case '_': adjust(Setting::AccelLimit, -1); break;
                    case '+': adjust(Setting::AccelLimit, +1); break;
                    case 'n': case 'N':
                        s.twist_req = BodyTwist{};
                        s.drive_word = "STOP";
                        s.prompt_buf.clear();
                        s.mode = Mode::PromptNote;
                        break;
                    case 'p': case 'P':
                        s.twist_req = BodyTwist{};
                        s.drive_word = "STOP";
                        s.prompt_buf.clear();
                        s.mode = Mode::PromptSignName;
                        break;
                    case 'c': case 'C':
                        // Zero the twist here AND in the loop below: the
                        // robot is held at zero for the whole time the menu
                        // is open, whatever was latched before.
                        s.twist_req = BodyTwist{};
                        s.drive_word = "STOP";
                        s.prompt_buf.clear();
                        s.mode = Mode::MenuSelect;
                        sink.push_line(encode_event(s.seq, t_mono, wall, "ceiling_menu_opened",
                                                    {{"ceiling", caps_to_json(s.caps)},
                                                     {"ceiling_derived", caps_to_json(s.derived_caps)},
                                                     {"ceiling_mode", json_string(s.ov.mode())},
                                                     {"override_active", json_bool(s.ov.any_override)}}));
                        break;
                    case 'x': case 'X': quit = true; break;
                    default: break;
                }
            }
            if (abort) {
                exit_reason = "abort";
                sink.push_line(encode_event(s.seq, t_mono, wall, "abort", {{"reason", json_string("ESC")}}));
                break;
            }
            if (quit) {
                exit_reason = "quit";
                sink.push_line(encode_event(s.seq, t_mono, wall, "quit", {}));
                break;
            }

            // ---- log health is a hold condition ------------------------
            if (!sink.healthy()) {
                enter_hold(s, sink, t_mono, "log write failed: the authoritative record is not being kept");
            }

            // ---- command -----------------------------------------------
            // While the ceiling menu is open the robot is commanded to ZERO
            // and held there: still energized, so it brakes to a stop under
            // its accel_limit and resists being pushed, rather than coasting
            // on STOP frames. A latched direction can never keep running
            // while someone is typing numbers.
            if (in_menu(s.mode)) {
                s.twist_req = BodyTwist{};
                s.drive_word = "STOP";
            }
            const bool energized = (s.mode == Mode::Drive || in_menu(s.mode)) && !s.hold;
            BodyTwist applied{};
            const char* clip = "";
            double peak_cmd = 0.0;
            const double wheel_cap = std::min(s.caps.velocity_limit_rev_s, s.settings.velocity_limit_rev_s);
            if (energized) {
                const auto demand = s.kin.inverse(s.twist_req);
                const WheelClamp wc = clamp_wheels(demand, wheel_cap);
                cmd_wheels = wc.wheels;
                peak_cmd = wc.peak_before;
                if (wc.clipped) {
                    clip = (s.settings.velocity_limit_rev_s < s.caps.velocity_limit_rev_s - 1e-9)
                               ? "velocity_limit" : "ceiling";
                }
                applied = BodyTwist{s.twist_req.vx * wc.factor, s.twist_req.vy * wc.factor,
                                    s.twist_req.w * wc.factor};
                // The per-command shaping the operator is commissioning.
                telem.velocity_limit_rev_s = s.settings.velocity_limit_rev_s;
                telem.accel_limit_rev_s2 = s.settings.accel_limit_rev_s2;
                std::map<int, double> cmd;
                for (int i = 0; i < 4; ++i) cmd[i + 1] = cmd_wheels[i];
                data = telem.cycle(cmd, /*energize=*/true);
            } else {
                cmd_wheels = {{kNaN, kNaN, kNaN, kNaN}};
                data = telem.cycle({}, /*energize=*/false);
            }

            // ---- observe -----------------------------------------------
            std::vector<rf::MotorObs> obs;
            double vsum = 0.0;
            int vn = 0;
            for (const auto& [id, bus] : opt.motor_map) {
                (void)bus;
                rf::MotorObs m;
                m.id = id;
                const auto it = data.find(id);
                m.replied = it != data.end();
                if (m.replied) {
                    m.fault = it->second.fault;
                    m.temperature = std::isfinite(it->second.temperature) ? it->second.temperature : 0.0;
                    m.current = std::isfinite(it->second.current) ? it->second.current : 0.0;
                    if (std::isfinite(it->second.voltage)) { vsum += it->second.voltage; ++vn; }
                    s.missing_streak[id] = 0;
                    if (it->second.mode == 11) {
                        enter_hold(s, sink, t_mono, "position_timeout on wheel " + std::to_string(id) +
                                                        ": the moteus watchdog expired (loop stalled or CAN dropped)");
                    }
                } else {
                    const int streak = ++s.missing_streak[id];
                    if (streak >= kMissingReplyHoldCycles) {
                        enter_hold(s, sink, t_mono, "no reply from wheel " + std::to_string(id) + " for " +
                                                        std::to_string(streak) + " cycles");
                    }
                }
                obs.push_back(m);
            }
            const auto newly = sup.observe(obs, vn > 0 ? vsum / vn : 0.0);
            if (!newly.empty()) {
                std::string why;
                for (const auto& r : newly) {
                    if (!why.empty()) why += ", ";
                    why += r.word();
                    if (r.motor > 0) {
                        why += " wheel " + std::to_string(r.motor);
                        if (r.kind == rf::StopReason::MotorFault) {
                            const auto it = data.find(r.motor);
                            if (it != data.end()) {
                                why += " (code " + std::to_string(it->second.fault) + " " +
                                       decode_fault(it->second.fault).name + ")";
                            }
                        }
                    }
                }
                enter_hold(s, sink, t_mono, why);
            }

            // ---- record ------------------------------------------------
            CycleRecord rec;
            rec.seq = s.seq;
            rec.t_mono_s = t_mono;
            rec.t_wall_unix_s = wall;
            rec.dt_s = last_dt;
            rec.twist_requested = s.twist_req;
            rec.twist_applied = applied;
            rec.settings = s.settings;
            rec.caps = s.caps;
            rec.caps_derived = s.derived_caps;
            rec.ceiling_manual = s.ov.any_manual;
            rec.override_active = s.ov.any_override;
            rec.energized = energized;
            rec.hold = s.hold;
            std::snprintf(rec.clip, sizeof rec.clip, "%s", clip);
            std::snprintf(rec.state, sizeof rec.state, "%s",
                          s.hold ? "hold"
                                 : (s.mode == Mode::Drive ? "drive"
                                                          : (in_menu(s.mode) ? "menu" : "prompt")));
            std::snprintf(rec.hold_reason, sizeof rec.hold_reason, "%s", s.hold_reason.c_str());
            for (int i = 0; i < 4; ++i) {
                WheelSample& w = rec.wheels[i];
                w.id = i + 1;
                w.cmd_rev_s = cmd_wheels[i];
                const auto it = data.find(i + 1);
                if (it != data.end()) {
                    w.replied = true;
                    w.velocity_rev_s = it->second.velocity;
                    w.position_rev = it->second.position;
                    w.q_current_a = it->second.current;
                    w.temperature_c = it->second.temperature;
                    w.voltage_v = it->second.voltage;
                    w.mode = it->second.mode;
                    w.fault = it->second.fault;
                }
            }
            rec.imu.present = telem.attitude_present;
            rec.imu.roll_dps = telem.imu_roll_dps;
            rec.imu.pitch_dps = telem.imu_pitch_dps;
            rec.imu.yaw_dps = telem.imu_yaw_dps;
            rec.imu.heading_deg = telem.imu_heading_deg;
            rec.imu.accel_x_mps2 = telem.imu_accel_x_mps2;
            rec.imu.accel_y_mps2 = telem.imu_accel_y_mps2;
            rec.imu.accel_z_mps2 = telem.imu_accel_z_mps2;

            // ---- window stats ------------------------------------------
            {
                auto& w = s.window;
                const bool driving = energized && (std::abs(applied.vx) + std::abs(applied.vy) +
                                                   std::abs(applied.w)) > 1e-9;
                if (driving) ++w.drive_cycles;
                for (const auto& ws : rec.wheels) {
                    if (std::isfinite(ws.cmd_rev_s)) {
                        w.peak_cmd_wheel_rev_s = std::isfinite(w.peak_cmd_wheel_rev_s)
                            ? std::max(w.peak_cmd_wheel_rev_s, std::abs(ws.cmd_rev_s)) : std::abs(ws.cmd_rev_s);
                    }
                    if (!ws.replied) continue;
                    if (std::isfinite(ws.velocity_rev_s)) {
                        w.peak_measured_wheel_rev_s = std::isfinite(w.peak_measured_wheel_rev_s)
                            ? std::max(w.peak_measured_wheel_rev_s, std::abs(ws.velocity_rev_s)) : std::abs(ws.velocity_rev_s);
                    }
                    if (std::isfinite(ws.q_current_a)) {
                        w.peak_q_current_a = std::isfinite(w.peak_q_current_a)
                            ? std::max(w.peak_q_current_a, std::abs(ws.q_current_a)) : std::abs(ws.q_current_a);
                    }
                    if (std::isfinite(ws.temperature_c)) {
                        w.peak_temperature_c = std::isfinite(w.peak_temperature_c)
                            ? std::max(w.peak_temperature_c, ws.temperature_c) : ws.temperature_c;
                    }
                    if (ws.fault != 0) {
                        auto& v = ws.fault >= 96 ? w.limit_codes_seen : w.faults_seen;
                        if (std::find(v.begin(), v.end(), ws.fault) == v.end()) v.push_back(ws.fault);
                    }
                }
            }

            // ---- timing ------------------------------------------------
            next += period;
            const auto after = Clock::now();
            if (after > next + period) {
                ++s.deadline_misses;
                rec.deadline_missed = true;
                // Do not try to catch up a long stall with a burst of frames.
                if (after > next + 5 * period) next = after;
            }
            sink.push_cycle(rec);

            // ---- screen ------------------------------------------------
            if (s.seq % kScreenEveryCycles == 1) {
                const std::string f = render_frame(s, data, telem, cmd_wheels, peak_cmd, wheel_cap,
                                                   clip, energized, last_dt, sink.stats());
                frame.begin_frame();
                std::cout << f;
                int lines = 0;
                for (char ch : f) if (ch == '\n') ++lines;
                frame.end_frame(lines);
            }

            std::this_thread::sleep_until(next);
        }
    }

    // ---- exit ------------------------------------------------------------
    stop_guard.stop();
    sink.close();
    const auto st = sink.stats();
    std::cout << "\n\n" << (exit_reason == "abort" ? Style::red() : Style::green())
              << (exit_reason == "abort" ? "ABORTED" : "stopped") << Style::reset()
              << " — motors stopped (SetStop x2). "
              << "log " << s.log_path << ": " << st.written << " records written, " << st.dropped
              << " dropped";
    if (st.streaming) std::cout << "; stream " << st.udp_sent << " sent, " << st.udp_failed << " failed";
    std::cout << "; deadline misses " << s.deadline_misses << '\n';
    if (s.profiles_written > 0) {
        std::cout << "profiles written: " << s.profiles_written << " (last " << s.last_profile_path << ")\n";
    } else {
        std::cout << "no profile written.\n";
    }
    return exit_reason == "abort" ? 130 : 0;
}

}  // namespace rf::dbg::commission
