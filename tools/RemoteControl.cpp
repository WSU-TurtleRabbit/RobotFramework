// RemoteControl — drive the robot by hand with WASD, straight to the motors.
//
// WHY THIS EXISTS
// ---------------
// The normal path (RobotFramework + MatchCtrl) refuses to move unless the
// match bridge says so:
//
//     motion_enabled = !require_vision_fix || est.has_fix       (match_bridge)
//     MOVE           = motion_enabled && !health.estop          (match_feedback)
//
// With SSL-Vision down, or with the Supervisor latched, MOVE is withheld and
// every remote command is discarded no matter what sends it. That is correct
// for autonomous play — a robot that cannot see must not drive itself — but
// it also makes the robot impossible to reposition by hand exactly when you
// most need to.
//
// This binary skips that stack entirely. It talks to the moteus controllers
// through Telemetry::cycle(), the same direct path tests/MultiMotor.cpp uses.
// No vision, no match bridge, no MOVE gate, no UDP.
//
// It still uses the REAL calibration: Wheel_math loads ../config/Motor.yaml
// (metersPerMotorRev, bodyLateralScale, per-wheel trims), so a commanded body
// twist comes out as the same wheel speeds the match stack would produce.
// Run it from build/ so that relative path resolves, exactly like the main
// binary does.
//
// SAFETY. There is no envelope here, no vision, and no field awareness. The
// robot goes where you point it and will happily drive off the carpet. What
// you do get:
//   * a hard speed cap (kMaxVel / kMaxOmega)
//   * SetStop() on every exit path, including Ctrl+C and exceptions
//   * the moteus per-command watchdog: motors self-stop ~100 ms after the
//     last frame, so if this process dies the robot does too
//   * per-motor current / temperature / FAULT printed live, which is also
//     how you spot a wheel that is dragging
//
// CONTROLS (latched: a direction persists until changed or stopped)
//   W / S   forward / back        A / D   strafe left / right
//   Q / E   rotate CCW / CW       SPACE   stop
//   [ / ]   slower / faster       X, Ctrl-C  quit
//
// Build:  add to CMakeLists.txt ->  build_executable(RemoteControl tools/RemoteControl.cpp)
// Run:    cd ~/RobotFramework/build && sudo ./RemoteControl

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "Telemetry.h"
#include "arduino.h"
#include "wheel_math.h"

namespace {

// Hard caps. Deliberately low: this is a hand-driving tool used near people,
// not a speed run. Raise only if you know why you need to.
constexpr double kMaxVel = 1.00;      // m/s
constexpr double kMaxOmega = 2.50;    // rad/s
constexpr double kVelStep = 0.10;
constexpr double kStartVel = 0.30;
constexpr double kStartOmega = 1.20;
constexpr auto kCycle = std::chrono::milliseconds(20);   // 50 Hz

// --- Kicker / dribbler (Arduino on /dev/ttyACM*, NOT the moteus bus) ---
// Wire format, from RobotFramework.cpp: {'k', pulse_ms} fires the solenoid,
// {'d', us} sets dribbler speed, 'S' stops the dribbler. Both payloads are a
// single byte, so 1..255.
//
// Scaling comes from config/Motion.yaml `actuators`:
//   kick_ms_per_mps: 1.5     -> a 4 m/s kick is a 6 ms pulse
//   dribble_us_per_mps: 20   -> the byte is ESC microseconds ABOVE the 1500
//                               stop point, so 3 m/s bar speed is ~60
constexpr int kKickMsDefault = 6;        // ~4 m/s
constexpr int kDribbleUsDefault = 60;    // ~3 m/s bar speed
constexpr int kKickMsMax = 20;           // refuse absurd solenoid pulses
// The capacitor needs ~4.6 s to recharge (measured on robot 2, 2026-08-31).
// Firing sooner just clicks with a half-charged cap and looks like a misfire.
constexpr double kKickCooldownS = 5.0;

// --- Network mode (--listen PORT) ---------------------------------------
// The keyboard mode only works if you are sitting at the robot. With --listen
// this becomes a headless drive daemon: a UDP socket takes plain-text
// commands, so the operator can drive from a laptop while the robot keeps
// bypassing MatchCtrl entirely (no vision gate, no MOVE, no Supervisor).
//
// Wire format, one datagram per command, ASCII:
//     "v <vx> <vy> <w>"   body-frame m/s and rad/s
//     "k"                 fire the kicker
//     "d <us>"            dribbler ESC microseconds (0 = stop)
//     "s"                 stop
//
// DEADMAN: the sender must repeat while it wants motion. If nothing arrives
// within kNetTimeoutS the robot is zeroed — so a closed laptop, a dropped
// wifi link or a killed client all stop the robot by the same mechanism.
constexpr double kNetTimeoutS = 0.35;
constexpr int kNetBufBytes = 256;

std::atomic<bool> g_stop{false};
void OnSignal(int) { g_stop.store(true, std::memory_order_relaxed); }

// Put the terminal in raw-ish mode so single keys arrive without Enter, and
// restore it on the way out however we leave.
class RawTerminal {
public:
    RawTerminal() {
        if (tcgetattr(STDIN_FILENO, &saved_) == 0) {
            ok_ = true;
            termios raw = saved_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0;      // non-blocking read
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        }
    }
    ~RawTerminal() {
        if (ok_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
private:
    termios saved_{};
    bool ok_ = false;
};

int ReadKey() {
    unsigned char c;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return n == 1 ? static_cast<int>(c) : -1;
}

// Non-blocking UDP command socket for --listen mode.
class NetCommand {
public:
    bool open(int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return false;
        int one = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        a.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0) {
            ::close(fd_); fd_ = -1; return false;
        }
        ::fcntl(fd_, F_SETFL, O_NONBLOCK);
        return true;
    }
    ~NetCommand() { if (fd_ >= 0) ::close(fd_); }

    // Drain every pending datagram; last one wins. Returns true if any arrived.
    bool poll(double& vx, double& vy, double& w, bool& kick, int& dribble_us) {
        if (fd_ < 0) return false;
        char buf[kNetBufBytes];
        bool got = false;
        for (;;) {
            const ssize_t n = ::recv(fd_, buf, sizeof buf - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            got = true;
            double a = 0, b = 0, c = 0;
            int us = 0;
            if (buf[0] == 'v' && std::sscanf(buf + 1, "%lf %lf %lf", &a, &b, &c) == 3) {
                vx = a; vy = b; w = c;
            } else if (buf[0] == 'k') {
                kick = true;
            } else if (buf[0] == 'd' && std::sscanf(buf + 1, "%d", &us) == 1) {
                dribble_us = us;
            } else if (buf[0] == 's') {
                vx = vy = w = 0.0;
            }
        }
        return got;
    }

private:
    int fd_ = -1;
};

// Names follow moteus/fw/error.h; tools/debugger/decode.cpp holds the full
// table with meanings. An earlier version of this table was misaligned from
// 37 on (37 is pwm_cycle_overrun, not over-temp; 38 is over_temperature, not
// start-fail; 39 is start_outside_limit, not over-current).
const char* FaultText(int fault) {
    switch (fault) {
        case 0:  return "-";
        case 32: return "calibration";
        case 33: return "motor-driver";
        case 34: return "over-voltage";
        case 35: return "encoder";
        case 36: return "not-configured";
        case 37: return "pwm-overrun";
        case 38: return "over-temp";
        case 39: return "start-outside-limit";
        case 40: return "under-voltage";
        case 41: return "config-changed";
        case 42: return "theta-invalid";
        case 43: return "position-invalid";
        case 44: return "driver-enable";
        case 45: return "stop-pos-deprecated";
        case 46: return "timing-violation";
        case 47: return "bemf-ff-no-accel";
        case 48: return "invalid-limits";
        case 49: return "pos-ctrl-error";
        case 50: return "vel-ctrl-error";
        default:
            // Codes >= 96 are live "output is being limited" reasons
            // (max_velocity, max_power, max_voltage, max_current, fet/motor
            // temperature, max_torque, position bounds, flux braking, field
            // weakening), not latched faults.
            return fault >= 96 ? "limit" : "fault";
    }
}

}  // namespace

int main(int argc, char** argv) {
    double vel = kStartVel;
    double omega = kStartOmega;
    int kick_ms = kKickMsDefault;
    int dribble_us = kDribbleUsDefault;
    int listen_port = 0;          // 0 = keyboard mode
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--vel" && i + 1 < argc) vel = std::stod(argv[++i]);
        else if (a == "--omega" && i + 1 < argc) omega = std::stod(argv[++i]);
        else if (a == "--kick-ms" && i + 1 < argc) kick_ms = std::stoi(argv[++i]);
        else if (a == "--dribble-us" && i + 1 < argc) dribble_us = std::stoi(argv[++i]);
        else if (a == "--listen" && i + 1 < argc) listen_port = std::stoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::cout << "usage: RemoteControl [--vel M/S] [--omega RAD/S]\n"
                         "                     [--kick-ms N] [--dribble-us N]\n"
                         "  WASD drive, QE rotate, SPACE stop, [ ] speed\n"
                         "  K kick, F dribbler toggle, X quit\n";
            return 0;
        }
    }
    if (kick_ms < 1) kick_ms = 1;
    if (kick_ms > kKickMsMax) kick_ms = kKickMsMax;
    if (dribble_us < 1) dribble_us = 1;
    if (dribble_us > 255) dribble_us = 255;
    if (vel > kMaxVel) vel = kMaxVel;
    if (omega > kMaxOmega) omega = kMaxOmega;

    // Loads ../config/Motor.yaml — run from build/, same as the main binary.
    Wheel_math wheels;
    Telemetry telemetry;

    // The kicker/dribbler live on the Arduino, a completely separate link
    // from the moteus bus. Absent on some chassis, so this is optional: if
    // there is no Arduino the drive half still works.
    Arduino arduino;
    const bool has_arduino = arduino.findArduino();
    std::cout << (has_arduino
                  ? "Arduino found on " + arduino.getPort() + " — kick/dribble enabled\n"
                  : "no Arduino — kick/dribble unavailable on this chassis\n");

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    // Clear any latched controller fault before commanding motion. This is
    // the moteus-level stop, not the Supervisor's estop, so it works even
    // when the match stack is refusing to move.
    for (const auto& c : telemetry.controllers) c.second->SetStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "\nRemoteControl — direct motor drive (no vision, no MOVE gate)\n"
              << "  W/S fwd-back   A/D strafe   Q/E rotate   SPACE stop\n"
              << "  [ ] speed      K kick (" << kick_ms << " ms)   "
              << "F dribbler (" << dribble_us << " us)   X quit\n\n";

    double vx = 0.0, vy = 0.0, vw = 0.0;
    bool dribbling = false;
    auto last_kick = std::chrono::steady_clock::now() -
                     std::chrono::seconds(60);   // ready immediately
    std::string note;
    auto next = std::chrono::steady_clock::now();

    // ---- NETWORK MODE: headless drive daemon ----------------------------
    if (listen_port > 0) {
        NetCommand net;
        if (!net.open(listen_port)) {
            std::cerr << "RemoteControl: cannot bind UDP " << listen_port << "\n";
            return 1;
        }
        std::cout << "listening on UDP " << listen_port
                  << "  ('v vx vy w' | 'k' | 'd us' | 's')\n"
                  << "  deadman: motion stops if nothing arrives for "
                  << kNetTimeoutS << "s\n\n";
        auto last_cmd = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        auto next_n = std::chrono::steady_clock::now();
        int net_dribble = 0;
        while (!g_stop.load(std::memory_order_relaxed)) {
            bool kick = false;
            int want_drib = net_dribble;
            if (net.poll(vx, vy, vw, kick, want_drib))
                last_cmd = std::chrono::steady_clock::now();

            const double idle = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_cmd).count();
            if (idle > kNetTimeoutS) { vx = vy = vw = 0.0; }   // deadman

            if (has_arduino && want_drib != net_dribble) {
                net_dribble = want_drib;
                if (net_dribble > 0) {
                    const char b[2] = {'d', static_cast<char>(
                        net_dribble > 255 ? 255 : net_dribble)};
                    arduino.sendBytes(b, 2);
                } else {
                    arduino.sendCommand('S');
                }
            }
            if (kick && has_arduino) {
                const double since = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_kick).count();
                if (since >= kKickCooldownS) {
                    const char b[2] = {'k', static_cast<char>(kick_ms)};
                    arduino.sendBytes(b, 2);
                    last_kick = std::chrono::steady_clock::now();
                    note = "KICK";
                }
            }

            const std::vector<double> w = wheels.calculate(vx, vy, vw);
            std::map<int, double> cmd;
            for (size_t i = 0; i < w.size() && i < 4; ++i)
                cmd[static_cast<int>(i) + 1] = w[i];
            const auto status = telemetry.cycle(cmd);

            std::printf("\r\033[K%-6s net%s cmd(%+.2f,%+.2f,%+.2f) %s |",
                        note.c_str(), idle > kNetTimeoutS ? "(idle)" : "     ",
                        vx, vy, vw, net_dribble > 0 ? "DRIB" : "    ");
            for (const auto& kv : status)
                std::printf(" %d:%.1fA/%.0fC%s", kv.first, kv.second.current,
                            kv.second.temperature,
                            kv.second.fault ? FaultText(kv.second.fault) : "");
            std::fflush(stdout);
            next_n += kCycle;
            std::this_thread::sleep_until(next_n);
        }
        if (has_arduino) arduino.sendCommand('S');
        for (const auto& c : telemetry.controllers) c.second->SetStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "\nstopped.\n";
        return 0;
    }

    {
        RawTerminal raw;
        while (!g_stop.load(std::memory_order_relaxed)) {
            // Drain every pending key so a burst cannot lag the loop.
            for (int k = ReadKey(); k != -1; k = ReadKey()) {
                switch (k) {
                    case 'w': case 'W': vx =  vel; vy = 0; vw = 0; note = "FWD";   break;
                    case 's': case 'S': vx = -vel; vy = 0; vw = 0; note = "BACK";  break;
                    case 'a': case 'A': vx = 0; vy =  vel; vw = 0; note = "LEFT";  break;
                    case 'd': case 'D': vx = 0; vy = -vel; vw = 0; note = "RIGHT"; break;
                    case 'q': case 'Q': vx = 0; vy = 0; vw =  omega; note = "CCW"; break;
                    case 'e': case 'E': vx = 0; vy = 0; vw = -omega; note = "CW";  break;
                    case ' ':           vx = vy = vw = 0; note = "STOP";           break;
                    case '[': vel = (vel - kVelStep < 0.05) ? 0.05 : vel - kVelStep;
                              note = "slower"; break;
                    case ']': vel = (vel + kVelStep > kMaxVel) ? kMaxVel : vel + kVelStep;
                              note = "faster"; break;
                    case 'k': case 'K': {
                        if (!has_arduino) { note = "no kicker"; break; }
                        const double since = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - last_kick).count();
                        if (since < kKickCooldownS) {
                            char buf[48];
                            std::snprintf(buf, sizeof buf, "recharging %.1fs",
                                          kKickCooldownS - since);
                            note = buf;
                            break;
                        }
                        const char bytes[2] = {'k', static_cast<char>(kick_ms)};
                        note = arduino.sendBytes(bytes, 2) ? "KICK" : "kick FAILED";
                        last_kick = std::chrono::steady_clock::now();
                        break;
                    }
                    case 'f': case 'F': {
                        if (!has_arduino) { note = "no dribbler"; break; }
                        dribbling = !dribbling;
                        if (dribbling) {
                            const char bytes[2] = {'d', static_cast<char>(dribble_us)};
                            note = arduino.sendBytes(bytes, 2) ? "DRIBBLE on"
                                                              : "dribble FAILED";
                        } else {
                            // 'S' is the stop command; the byte form expects a
                            // non-zero speed, so zero is sent this way.
                            arduino.sendCommand('S');
                            note = "dribble off";
                        }
                        break;
                    }
                    case 'x': case 'X': case 3: g_stop.store(true); break;
                    default: break;
                }
                // A held direction keeps its magnitude when speed changes.
                if (k == '[' || k == ']') {
                    if (vx != 0.0) vx = (vx > 0 ? vel : -vel);
                    if (vy != 0.0) vy = (vy > 0 ? vel : -vel);
                }
            }
            if (g_stop.load(std::memory_order_relaxed)) break;

            const std::vector<double> w = wheels.calculate(vx, vy, vw);
            std::map<int, double> cmd;
            for (size_t i = 0; i < w.size() && i < 4; ++i)
                cmd[static_cast<int>(i) + 1] = w[i];

            const auto status = telemetry.cycle(cmd);

            // One rewriting status line: command, then per-motor current /
            // temperature / fault. Current is what exposes a dragging wheel.
            std::printf("\r\033[K%-14s v=%.2f w=%.2f %s | cmd(%+.2f,%+.2f,%+.2f) |",
                        note.c_str(), vel, omega, dribbling ? "DRIB" : "    ",
                        vx, vy, vw);
            for (const auto& kv : status) {
                std::printf(" %d:%.1fA/%.0fC%s", kv.first, kv.second.current,
                            kv.second.temperature,
                            kv.second.fault ? FaultText(kv.second.fault) : "");
            }
            std::fflush(stdout);

            next += kCycle;
            std::this_thread::sleep_until(next);
        }
    }

    // Every exit path stops the motors AND the dribbler. The moteus watchdog
    // would catch the wheels ~100 ms after the last frame, but nothing would
    // stop a spinning dribbler bar, so send it explicitly.
    if (has_arduino) arduino.sendCommand('S');
    for (const auto& c : telemetry.controllers) c.second->SetStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "\nstopped.\n";
    return 0;
}
