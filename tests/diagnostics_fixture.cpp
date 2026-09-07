// Hardware-free producer ABI fixture and Unix-datagram failure isolation checks.
#include "Telemetry/diagnostics.h"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#ifdef __linux__
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

int main(int argc, char** argv) {
    using namespace rf::diagnostics;
    static_assert(kPacketBytes == 668);
    Snapshot sample;
    sample.robot_id = 2;
    sample.cycle = 42;
    sample.acquired_us = 1000000;
    sample.motors[0] = Motor{3.5, 2.5, 2.25, 12.0, -1.5, 23.5, 40.0,
                             10, 0, true, true, 0};
    // Controller request survives as evidence when a supervisor STOP makes
    // final submitted velocity unavailable and energized=false.
    sample.motors[1].requested = 4.5;
    sample.flags = 129;
    sample.kick_requested = 2;
    sample.kick_sent = 1;
    std::array<unsigned char, 16> process{};
    process[0] = 0xAB;
    const auto packet = encode(sample, process, 5, 1);
    assert(packet[0] == 'P' && packet[24] == 2 && packet[32] == 42);
    if (argc == 2) {
        std::ofstream file(argv[1], std::ios::binary);
        file.write(reinterpret_cast<const char*>(packet.data()), packet.size());
        assert(file.good());
    }
    SnapshotSender disabled(nullptr);
    assert(!disabled.enabled() && !disabled.due(1000000));
#ifdef __linux__
    char path[108];
    std::snprintf(path, sizeof path, "/tmp/rf-telemetry-test-%d.sock", int(getpid()));
    SnapshotSender sender(path);
    assert(sender.enabled());
    assert(sender.due(1) && !sender.due(2) && sender.due(10001));
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 10000; ++i) assert(!sender.offer(sample));
    assert(sender.dropped() == 10000);
    int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    assert(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strcpy(address.sun_path, path);
    assert(::bind(receiver, reinterpret_cast<sockaddr*>(&address), sizeof address) == 0);
    assert(sender.offer(sample));
    Packet received;
    assert(::recv(receiver, received.data(), received.size(), 0) == ssize_t(kPacketBytes));
    for (int i = 0; i < 10000; ++i) sender.offer(sample); // never drain: queue fills
    assert(sender.dropped() > 10000);
    ::close(receiver);
    ::unlink(path);
    assert(!sender.offer(sample));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    assert(elapsed < std::chrono::seconds(5)); // catches accidental blocking send
#endif
    return 0;
}
