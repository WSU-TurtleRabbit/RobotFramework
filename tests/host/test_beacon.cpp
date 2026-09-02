// Discovery beacon: the JSON the robot broadcasts must carry exactly the
// keys phoenix-server/phoenix/core/discovery.py::parse_beacon reads, escape
// strings, and stay one line.
#include <limits>
#include <string>

#include "Motion/phx/testing.h"
#include "Networks/beacon.h"

using namespace rf;

namespace {
bool contains(const std::string& hay, const char* needle) {
    return hay.find(needle) != std::string::npos;
}
}  // namespace

PHX_TEST(beacon_json_has_every_key_the_server_reads) {
    BeaconInfo info;
    info.robot_id = 5;
    info.hardware_id = 0;
    info.rid = "A";
    info.hostname = "raspberrypi";
    info.features = 11;
    info.battery_v = 23.94;
    info.profile_id = 1001;
    info.rl_mode = 3;
    info.adaptive = true;
    info.uptime_s = 812.5;
    info.command_port = 50514;
    const std::string s = beacon_json(info);
    CHECK(s.front() == '{' && s.back() == '}');
    CHECK(s.find('\n') == std::string::npos);
    CHECK(contains(s, "\"phoenix_beacon\":1"));
    CHECK(contains(s, "\"robot_id\":5"));
    CHECK(contains(s, "\"hardware_id\":0"));
    CHECK(contains(s, "\"rid\":\"A\""));
    CHECK(contains(s, "\"hostname\":\"raspberrypi\""));
    CHECK(contains(s, "\"fw\":\""));
    CHECK(contains(s, "\"features\":11"));
    CHECK(contains(s, "\"battery_v\":23.94"));
    CHECK(contains(s, "\"profile_id\":1001"));
    CHECK(contains(s, "\"rl_mode\":3"));
    CHECK(contains(s, "\"adaptive\":true"));
    CHECK(contains(s, "\"uptime_s\":812.5"));
    CHECK(contains(s, "\"command_port\":50514"));
}

PHX_TEST(beacon_json_escapes_strings_and_survives_non_finite) {
    BeaconInfo info;
    info.robot_id = 2;
    info.hostname = "we\"ird\\name";
    info.battery_v = std::numeric_limits<double>::quiet_NaN();  // must not produce invalid JSON
    const std::string s = beacon_json(info);
    CHECK(contains(s, "\"hostname\":\"we\\\"ird\\\\name\""));
    CHECK(contains(s, "\"battery_v\":0"));
    CHECK(contains(s, "\"adaptive\":false"));
}

PHX_TEST(beacon_sender_never_throws_when_unavailable) {
    // On the host the sender may be a stub; on a Pi it is a real socket.
    // Either way construction and send() are safe to call from the superloop.
    BeaconSender sender(0, "not-an-address");
    const bool sent = sender.send("{}");
    CHECK(sent || sender.tx_errors() >= 1);
}
