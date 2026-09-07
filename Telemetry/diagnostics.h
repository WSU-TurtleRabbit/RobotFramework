// Observational local IPC only. This class never reads CAN or sends robot commands.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rf::diagnostics {

constexpr double kMissing = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kPacketBytes = 668;
constexpr std::size_t kMotionValues = 33;
using Packet = std::array<unsigned char, kPacketBytes>;

struct Motor {
    double requested = kMissing, sent = kMissing;
    double velocity = kMissing, position = kMissing, current = kMissing;
    double voltage = kMissing, temperature = kMissing;
    int32_t mode = -1, fault = -1;
    bool replied = false, energized = false;
    uint64_t age_us = 0;
};

// Order is mirrored explicitly in diagnostics/ipc.py; this is private local IPC.
struct Snapshot {
    int32_t robot_id = -1;
    uint64_t cycle = 0, acquired_us = 0;
    std::array<Motor, 4> motors{};
    std::array<double, kMotionValues> motion{};
    // bits: IMU valid, vision alive, vision confidence valid, supervisor estop,
    // control deadline missed, Arduino connected, dribbler commanded active,
    // MatchCtrl bridge active.
    uint32_t flags = 0;
    uint64_t kick_requested = 0, kick_sent = 0;
};

Packet encode(const Snapshot& sample, const std::array<unsigned char, 16>& producer,
              uint64_t offered, uint64_t dropped) noexcept;
uint64_t monotonic_us() noexcept;

class SnapshotSender {
public:
    // Null/empty path disables IPC. Configuration and randomness are startup work.
    explicit SnapshotSender(const char* path) noexcept;
    ~SnapshotSender();
    SnapshotSender(const SnapshotSender&) = delete;
    SnapshotSender& operator=(const SnapshotSender&) = delete;
    bool due(uint64_t now_us) noexcept;
    bool offer(const Snapshot& sample) noexcept;
    bool enabled() const noexcept { return fd_ >= 0; }
    uint64_t dropped() const noexcept { return dropped_; }
private:
    int fd_ = -1;
    std::array<unsigned char, 16> producer_{};
    std::array<char, 108> path_{};
    uint64_t next_us_ = 0, offered_ = 0, dropped_ = 0;
};

}  // namespace rf::diagnostics
