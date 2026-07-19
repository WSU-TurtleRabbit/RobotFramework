// Host-side sim robot for the live interop check: the REAL firmware modules
// (wire decode, skills, estimator, trajectory, controller, actuators,
// match_bridge, MatchFeedback encode) wrapped in a perfect-tracking plant
// and a ball, speaking UDP like the Pi superloop does. The real
// phoenix-server drives it end to end over loopback (see interop_e2e.py).
//
//   sim_robot [command_port] [robot_id]
//
// stdin/stdout: prints `TRUTH x y th bx by bvx bvy` at 60 Hz for the test
// harness (the fake SSL-Vision source). Accepts `SIM BALL x y` text
// datagrams to place the ball. Everything else behaves exactly like the
// robot: MatchCtrl in on command_port, MatchFeedback back to the sender.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX  // windows.h min/max macros poison std::min/std::max
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
using socklen_t = int;
static void sock_init() { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); }
static void sock_close(int fd) { closesocket(fd); }
static int sock_err() { return WSAGetLastError(); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
static void sock_init() {}
static void sock_close(int fd) { close(fd); }
static int sock_err() { return errno; }
#endif

#include "Motion/match_bridge.h"
#include "Networks/matchctrl.h"
#include "Telemetry/match_feedback.h"

using namespace rf;

namespace {

constexpr double kDt = 0.004;          // 250 Hz, like the Pi superloop
constexpr double kMouthDist = 0.105;   // fake_robot.py's sticky-dribble distance
constexpr double kBarrierDist = 0.115; // ... and its barrier model
constexpr double kBarrierFacing = 0.5;
constexpr double kBallFriction = 0.5;  // m/s^2

struct Plant {
    double x = 0, y = 0, th = 0;
    std::array<double, 4> wheels{};
    Kinematics kin;

    phx::Twist odo() const {
        const BodyTwist b = kin.forward(wheels);
        return phx::Twist{b.vx, b.vy, b.w};
    }
    void step(double dt) {
        const BodyTwist o = kin.forward(wheels);
        th = phx::wrap_angle(th + o.w * dt);
        const double c = std::cos(th), s = std::sin(th);
        x += (o.vx * c - o.vy * s) * dt;
        y += (o.vx * s + o.vy * c) * dt;
    }
    double speed() const {
        const BodyTwist o = kin.forward(wheels);
        return std::hypot(o.vx, o.vy);
    }
};

struct Ball {
    double x = 0, y = 0, vx = 0, vy = 0;
    void step(double dt) {
        const double sp = std::hypot(vx, vy);
        if (sp > 1e-9) {
            const double ns = std::max(0.0, sp - kBallFriction * dt);
            vx *= ns / sp;
            vy *= ns / sp;
        }
        x += vx * dt;
        y += vy * dt;
    }
    double speed() const { return std::hypot(vx, vy); }
};

}  // namespace

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::atoi(argv[1]) : 50514;
    const int robot_id = argc > 2 ? std::atoi(argv[2]) : 0;

    sock_init();
    const int sock = static_cast<int>(socket(AF_INET, SOCK_DGRAM, 0));
    if (sock < 0) {
        std::fprintf(stderr, "socket failed: %d\n", sock_err());
        return 1;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "bind %d failed: %d\n", port, sock_err());
        return 1;
    }
    // Non-blocking receive.
#ifdef _WIN32
    u_long nb = 1;
    ioctlsocket(sock, FIONBIO, &nb);
#else
    struct timeval tv {0, 0};
    tv.tv_usec = 2000;  // 2 ms: poll cadence, not a stall
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    MatchBridgeConfig bcfg;
    bcfg.expected_robot_id = robot_id;
    MatchBridge bridge(bcfg, Plant{}.kin);
    MatchFeedbackBuilder feedback;
    FeedbackHealth health;
    health.robot_id = robot_id;
    health.hardware_id = robot_id;
    health.battery_v = 16.0;
    health.arduino_connected = true;
    health.camera_running = true;

    Plant plant;
    Ball ball;
    // ball starts somewhere harmless
    ball.x = 3.0;
    ball.y = 3.0;

    std::fprintf(stderr, "sim_robot listening on %d, id %d\n", port, robot_id);

    const auto t0 = std::chrono::steady_clock::now();
    double now_s = 0.0;
    double next_truth = 0.0;
    int fb_divider = 0;
    sockaddr_in last_sender{};
    bool have_sender = false;
    BridgeTick bt;

    while (true) {
        // --- drain datagrams ---
        char buf[2048];
        sockaddr_in from{};
        socklen_t fromlen = sizeof(from);
        const int n = static_cast<int>(
            recvfrom(sock, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &fromlen));
        if (n > 0) {
            have_sender = true;
            last_sender = from;
            const std::string msg(buf, buf + n);
            if (msg.rfind("SIM ", 0) == 0) {
                double bx = 0, by = 0;
                if (std::sscanf(msg.c_str(), "SIM BALL %lf %lf", &bx, &by) == 2) {
                    ball.x = bx;
                    ball.y = by;
                    ball.vx = ball.vy = 0;
                }
            } else {
                // Wire-level log for the harness (STOP-cap verification):
                // what did the server actually send?
                const auto mc = decode_match_ctrl(msg);
                if (mc && mc->skill_id == static_cast<int>(SkillId::GlobalPos)) {
                    const GlobalPosSkill gp = decode_global_pos(mc->skill_data);
                    std::fprintf(stderr, "SKILL 4 vel_max %.4f\n", gp.vel_max_xy);
                } else if (mc) {
                    std::fprintf(stderr, "SKILL %d\n", mc->skill_id);
                }
                bridge.accept(msg, now_s);
            }
        }

        // --- one control tick ---
        const phx::Twist odo = plant.odo();
        // Ball contact = the break-beam stand-in (fake_robot's model).
        const double bdx = ball.x - plant.x, bdy = ball.y - plant.y;
        const double bdist = std::hypot(bdx, bdy);
        const double bang = std::atan2(bdy, bdx);
        const bool barrier = bdist <= kBarrierDist &&
                             std::fabs(phx::angle_diff(bang, plant.th)) <= kBarrierFacing;
        BallContactObs obs;
        obs.found = barrier;
        obs.bearing = 0.0;
        obs.radius = barrier ? 90.0 : 0.0;
        obs.confidence = 1.0;
        obs.age_s = 0.0;

        bt = bridge.tick(now_s, kDt, odo, odo.ang, obs);
        plant.wheels = bt.ctrl.wheel_rev_s;
        plant.step(kDt);

        // Kicker: fire edge launches the ball (kick speed from the pulse map).
        if (bt.act.fire_pulse_ms.has_value()) {
            const double kick_speed = std::max(0.0, (*bt.act.fire_pulse_ms - 2.0) / 1.5);
            const double c = std::cos(plant.th), s = std::sin(plant.th);
            ball.vx = c * kick_speed;
            ball.vy = s * kick_speed;
            ball.x = plant.x + c * (kBarrierDist + 0.05);
            ball.y = plant.y + s * (kBarrierDist + 0.05);
        } else if (bt.act.dribbler_speed > 0.01 && barrier) {
            // Sticky dribbler (fake_robot semantics): ball pinned at the mouth.
            const double c = std::cos(plant.th), s = std::sin(plant.th);
            ball.x = plant.x + c * kMouthDist;
            ball.y = plant.y + s * kMouthDist;
            const BodyTwist o = plant.kin.forward(plant.wheels);
            ball.vx = o.vx * c - o.vy * s;
            ball.vy = o.vx * s + o.vy * c;
        }
        ball.step(kDt);

        // --- MatchFeedback at 50 Hz to the last sender ---
        if (++fb_divider >= 5 && have_sender) {
            fb_divider = 0;
            obs.found = barrier;  // feedback barrier = the contact signal
            const MatchFeedback fb = feedback.build(bridge, bt, health, obs, now_s);
            const auto bytes = encode_match_feedback(fb);
            sendto(sock, reinterpret_cast<const char*>(bytes.data()),
                   static_cast<int>(bytes.size()), 0,
                   reinterpret_cast<sockaddr*>(&last_sender), sizeof(last_sender));
        }

        // --- truth beacon at 60 Hz (the harness's fake vision) ---
        if (now_s >= next_truth) {
            next_truth += 1.0 / 60.0;
            std::printf("TRUTH %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", plant.x, plant.y,
                        plant.th, ball.x, ball.y, ball.vx, ball.vy);
            std::fflush(stdout);
        }

        now_s += kDt;
        const auto want = t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(now_s));
        std::this_thread::sleep_until(want);
    }
}
