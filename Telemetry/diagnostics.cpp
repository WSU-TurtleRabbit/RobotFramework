#include "diagnostics.h"

#include <chrono>
#include <cstring>
#ifdef __linux__
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace rf::diagnostics {
namespace {
class Encoder {
public:
    Packet bytes{};
    void integer(uint64_t value, unsigned width) noexcept {
        for (unsigned i = 0; i < width; ++i) bytes[offset_++] = (value >> (8 * i)) & 255;
    }
    void number(double value) noexcept {
        static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
        uint64_t bits;
        std::memcpy(&bits, &value, sizeof bits);
        integer(bits, 8);
    }
private:
    std::size_t offset_ = 0;
};
}

Packet encode(const Snapshot& sample, const std::array<unsigned char, 16>& producer,
              uint64_t offered, uint64_t dropped) noexcept {
    Encoder out;
    for (const unsigned char c : {'P', 'H', 'I', '1'}) out.integer(c, 1);
    out.integer(1, 4);
    for (const auto c : producer) out.integer(c, 1);
    out.integer(static_cast<uint32_t>(sample.robot_id), 4);
    out.integer(0, 4);
    out.integer(sample.cycle, 8);
    out.integer(sample.acquired_us, 8);
    out.integer(offered, 8);
    out.integer(dropped, 8);
    for (const Motor& motor : sample.motors) {
        for (const double value : {motor.requested, motor.sent, motor.velocity,
                                  motor.position, motor.current, motor.voltage,
                                  motor.temperature}) out.number(value);
        out.integer(static_cast<uint32_t>(motor.mode), 4);
        out.integer(static_cast<uint32_t>(motor.fault), 4);
        out.integer(motor.replied ? 1 : 0, 4);
        out.integer(motor.energized ? 1 : 0, 4);
        out.integer(motor.age_us, 8);
    }
    for (double value : sample.motion) out.number(value);
    out.integer(sample.flags, 4);
    out.integer(sample.kick_requested, 8);
    out.integer(sample.kick_sent, 8);
    return out.bytes;
}

uint64_t monotonic_us() noexcept {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

SnapshotSender::SnapshotSender(const char* path) noexcept {
#ifdef __linux__
    if (!path || !*path || std::strlen(path) >= path_.size()) return;
    // Never block waiting for entropy. Failure disables only diagnostics.
    if (::getrandom(producer_.data(), producer_.size(), GRND_NONBLOCK) !=
        static_cast<ssize_t>(producer_.size())) return;
    std::memcpy(path_.data(), path, std::strlen(path) + 1);
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
#else
    (void)path;
#endif
}

SnapshotSender::~SnapshotSender() {
#ifdef __linux__
    if (fd_ >= 0) ::close(fd_);
#endif
}

bool SnapshotSender::due(uint64_t now_us) noexcept {
    if (fd_ < 0 || now_us < next_us_) return false;
    next_us_ = now_us + 10000; // at most 100 Hz; no backlog or catch-up
    return true;
}

bool SnapshotSender::offer(const Snapshot& sample) noexcept {
#ifdef __linux__
    if (fd_ < 0) return false;
    ++offered_;
    const Packet packet = encode(sample, producer_, offered_, dropped_);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path_.data(), path_.size());
    const ssize_t sent = ::sendto(fd_, packet.data(), packet.size(),
                                  MSG_DONTWAIT | MSG_NOSIGNAL,
                                  reinterpret_cast<sockaddr*>(&address), sizeof address);
    if (sent == static_cast<ssize_t>(packet.size())) return true;
    ++dropped_;
#else
    (void)sample;
#endif
    return false;
}

}  // namespace rf::diagnostics
