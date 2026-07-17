// Telemetry protocol v2 encoder: the exact key set the server's parser
// requires, ported from the phoenix-rf wire.rs tests (same golden sets).
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "Motion/phx/testing.h"
#include "Telemetry/telemetry_wire.h"

using rf::MotorTelem;
using rf::TelemetrySnapshot;

namespace {

std::map<std::string, std::string> kv_map(const std::string& s) {
    std::map<std::string, std::string> out;
    size_t pos = 0;
    while (pos <= s.size()) {
        const size_t comma = s.find(',', pos);
        const std::string pair =
            s.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
        pos = comma == std::string::npos ? s.size() + 1 : comma + 1;
        const size_t eq = pair.find('=');
        if (eq != std::string::npos) out[pair.substr(0, eq)] = pair.substr(eq + 1);
    }
    return out;
}

MotorTelem absent(int id) {
    MotorTelem m;
    m.id = id;
    return m;
}

}  // namespace

PHX_TEST(telemetry_advertises_mv2_capability) {
    const TelemetrySnapshot snap;
    const std::string s = snap.encode();
    for (const char* key : {"mv=2", "mv_seq=-1", "wd=0", "mv_kind=-", "tgt_mm=-1"}) {
        CHECK(s.find(key) != std::string::npos);
    }
}

PHX_TEST(telemetry_encodes_all_required_v2_fields) {
    TelemetrySnapshot snap;
    snap.state = "active";
    snap.voltage = 12.0;
    snap.rid = "B";
    snap.seq = 42;
    snap.up_ms = 1234;
    snap.ifip = "218.148.92.35";
    snap.vmin = 11.9;
    snap.m_ok = 4;
    snap.m_exp = 4;
    snap.cmd_age_ms = 20;
    snap.cmd_rx = 99;
    snap.cmd_last_id = 5;
    snap.arduino_connected = true;
    snap.estop = false;
    snap.cycle_ms = 1.2;
    snap.robot_ts_ms = 1719000000123ull;
    for (int id = 1; id <= 4; ++id) {
        MotorTelem m;
        m.id = id;
        m.ok = true;
        m.mode = 10;
        m.fault = 0;
        m.temperature = 30.0;
        m.voltage = 12.0;
        m.velocity = 0.5;
        m.current = 1.0;
        m.cal = "ok";
        snap.motors.push_back(m);
    }
    const std::string s = snap.encode();
    // Must contain every field the server's parser requires (same golden
    // list as the Rust test).
    for (const char* key : {
             "state=active", "voltage=12", "ball=0", "px=0", "py=0", "r=0", "bearing=0",
             "conf=0", "ts_ms=1719000000123", "proto=2", "rid=B", "seq=42", "up_ms=1234",
             "ifip=218.148.92.35", "vmin=11.9", "m_ok=4", "m_exp=4", "cmd_age_ms=20",
             "cmd_rx=99", "cmd_last_id=5", "ard=1", "cam=0", "estop=0", "tx_err=0",
             "cycle_ms=1.2", "m1_ok=1", "m1_mode=10", "m1_fault=0", "m4_cur=1",
         }) {
        CHECK(s.find(key) != std::string::npos);
    }
}

PHX_TEST(telemetry_encoded_has_no_missing_required_fields) {
    // The definitive compatibility test: encode, then re-check with the
    // server's required-field list.
    TelemetrySnapshot snap;
    snap.rid = "B";
    for (int id = 1; id <= 4; ++id) snap.motors.push_back(absent(id));
    const auto map = kv_map(snap.encode());
    for (const char* req : {
             "state", "voltage", "ball", "px", "py", "r", "bearing", "conf", "ts_ms",
             "proto", "rid", "seq", "up_ms", "ifip", "vmin", "m_ok", "m_exp",
             "cmd_age_ms", "cmd_rx", "cmd_last_id", "ard", "cam", "estop", "tx_err",
             "cycle_ms",
         }) {
        CHECK(map.count(req) == 1);
    }
    for (int id = 1; id <= 4; ++id) {
        for (const char* suf : {"ok", "mode", "fault", "temp", "volt", "vel", "cur"}) {
            const std::string k = "m" + std::to_string(id) + "_" + suf;
            CHECK(map.count(k) == 1);
        }
    }
    // Additive v2+ keys are present too.
    for (const char* k : {"imu_yaw_dps", "heading_deg", "odo_vx", "odo_vy", "odo_w",
                          "loop_ms", "loop_jitter_ms", "imu_ok", "needs_cal",
                          "cal_state", "cfg_fixed", "mv", "mv_seq", "wd", "mv_kind",
                          "tgt_mm"}) {
        CHECK(map.count(k) == 1);
    }
}

PHX_TEST(telemetry_v1_keys_come_first) {
    // Old dashboards read the leading v1 block; it must stay in front.
    const TelemetrySnapshot snap;
    const std::string s = snap.encode();
    CHECK(s.rfind("state=", 0) == 0);  // starts with state=
    const size_t ts = s.find("ts_ms=");
    const size_t proto = s.find("proto=2");
    REQUIRE(ts != std::string::npos);
    REQUIRE(proto != std::string::npos);
    CHECK(ts < proto);  // whole v1 core precedes the v2 block
    // NaN optionals render as parseable words, not garbage.
    TelemetrySnapshot n;
    n.imu_yaw_dps = std::nan("");
    n.heading_deg = std::nan("");
    const std::string sn = n.encode();
    CHECK(sn.find("imu_yaw_dps=nan") != std::string::npos);
    CHECK(sn.find("heading_deg=nan") != std::string::npos);
}

PHX_TEST(telemetry_motor_block_sorted_and_formatted) {
    TelemetrySnapshot snap;
    // Insert out of order; encode must sort by id.
    MotorTelem m3 = absent(3);
    m3.ok = true;
    m3.temperature = 31.26;
    m3.voltage = 12.345;
    m3.velocity = 0.123456;
    m3.current = 2.5;
    MotorTelem m1 = absent(1);
    snap.motors.push_back(m3);
    snap.motors.push_back(m1);
    const std::string s = snap.encode();
    const size_t p1 = s.find("m1_ok=");
    const size_t p3 = s.find("m3_ok=");
    REQUIRE(p1 != std::string::npos);
    REQUIRE(p3 != std::string::npos);
    CHECK(p1 < p3);
    // Fixed-width formats: temp %.1f, volt %.2f, vel %.3f, cur %.2f.
    CHECK(s.find("m3_temp=31.3") != std::string::npos);
    CHECK(s.find("m3_volt=12.35") != std::string::npos);
    CHECK(s.find("m3_vel=0.123") != std::string::npos);
    CHECK(s.find("m3_cur=2.50") != std::string::npos);
    CHECK(s.find("m1_cal=unknown") != std::string::npos);
}
