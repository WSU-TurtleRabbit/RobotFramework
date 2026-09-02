#include "beacon.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#if defined(__unix__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace rf {
namespace {

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::string num(double v, int decimals) {
    if (!std::isfinite(v)) return "0";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return buf;
}

}  // namespace

std::string beacon_json(const BeaconInfo& info) {
    std::string s = "{\"phoenix_beacon\":1";
    s += ",\"robot_id\":" + std::to_string(info.robot_id);
    s += ",\"hardware_id\":" + std::to_string(info.hardware_id);
    s += ",\"rid\":\"" + json_escape(info.rid) + "\"";
    s += ",\"hostname\":\"" + json_escape(info.hostname) + "\"";
    s += ",\"fw\":\"" + json_escape(info.firmware) + "\"";
    s += ",\"features\":" + std::to_string(static_cast<unsigned>(info.features));
    s += ",\"battery_v\":" + num(info.battery_v, 2);
    s += ",\"profile_id\":" + std::to_string(info.profile_id);
    s += ",\"rl_mode\":" + std::to_string(info.rl_mode);
    s += std::string(",\"adaptive\":") + (info.adaptive ? "true" : "false");
    s += ",\"uptime_s\":" + num(info.uptime_s, 1);
    s += ",\"command_port\":" + std::to_string(info.command_port);
    s += "}";
    return s;
}

#if defined(__unix__) || defined(__APPLE__)

BeaconSender::BeaconSender(int port, const std::string& target)
    : port_(port), target_(target) {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return;
    int yes = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)) < 0) {
        close(fd_);
        fd_ = -1;
    }
}

BeaconSender::~BeaconSender() {
    if (fd_ >= 0) close(fd_);
}

bool BeaconSender::send(const std::string& payload) {
    if (fd_ < 0) return false;
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, target_.c_str(), &addr.sin_addr) != 1) {
        tx_err_++;
        return false;
    }
    const ssize_t n = sendto(fd_, payload.data(), payload.size(), MSG_DONTWAIT,
                             reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (n < 0) {
        tx_err_++;
        return false;
    }
    sent_++;
    return true;
}

#else  // host builds without BSD sockets (MSVC): a stub that never sends.

BeaconSender::BeaconSender(int port, const std::string& target)
    : fd_(-1), port_(port), target_(target) {}
BeaconSender::~BeaconSender() = default;
bool BeaconSender::send(const std::string&) {
    tx_err_++;
    return false;
}

#endif

}  // namespace rf
