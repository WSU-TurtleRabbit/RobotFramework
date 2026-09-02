// Discovery beacon — the robot announces itself to whoever is listening.
//
// Phoenix Server's roster used to be a static id -> IP table; on a DHCP
// field the addresses change every session and a freshly booted robot was
// invisible until someone edited a file. So once per second the robot
// broadcasts one small JSON datagram to UDP kBeaconPort:
//
//   {"phoenix_beacon": 1, "robot_id": 5, "hardware_id": 0, "rid": "A",
//    "hostname": "raspberrypi", "fw": "…", "features": 11,
//    "battery_v": 23.9, "profile_id": 1001, "rl_mode": 0,
//    "uptime_s": 812.5, "command_port": 50514}
//
// The server lists beacons under "Discovered" and an OPERATOR adopts the
// robot into the roster (never automatically — three chassis claimed id 6
// on 2026-08-26). The server trusts the datagram's source IP, never the
// payload, for the address.
//
// beacon_json() is pure and host-testable; BeaconSender is the POSIX
// broadcast socket and compiles to a stub elsewhere.
#pragma once

#include <cstdint>
#include <string>

namespace rf {

inline constexpr int kBeaconPort = 50515;
inline constexpr double kBeaconIntervalS = 1.0;
inline constexpr const char* kRobotFrameworkVersion = "2026.09-phoenix2";

struct BeaconInfo {
    int robot_id = -1;
    int hardware_id = 0;
    std::string rid;         // asset letter from Main.yaml (Robot_rid)
    std::string hostname;
    std::string firmware = kRobotFrameworkVersion;
    uint16_t features = 0;   // the MatchFeedback features word
    double battery_v = 0.0;
    int profile_id = 0;      // onboard motion profile id (Motion.yaml profile.id)
    int rl_mode = 0;         // 0 off, 1 collect, 2 shadow, 3 bounded
    bool adaptive = false;
    double uptime_s = 0.0;
    int command_port = 50514;
};

// One JSON object, single line, ASCII, no trailing newline.
std::string beacon_json(const BeaconInfo& info);

class BeaconSender {
public:
    // target is the broadcast address; port the listener port.
    BeaconSender(int port = kBeaconPort, const std::string& target = "255.255.255.255");
    ~BeaconSender();
    BeaconSender(const BeaconSender&) = delete;
    BeaconSender& operator=(const BeaconSender&) = delete;

    bool ok() const { return fd_ >= 0; }
    // Sends the datagram; false when the socket is unavailable or sendto
    // failed (counted in tx_errors()). Never throws, never blocks.
    bool send(const std::string& payload);
    unsigned long long tx_errors() const { return tx_err_; }
    unsigned long long sent() const { return sent_; }

private:
    int fd_ = -1;
    int port_;
    std::string target_;
    unsigned long long tx_err_ = 0;
    unsigned long long sent_ = 0;
};

}  // namespace rf
