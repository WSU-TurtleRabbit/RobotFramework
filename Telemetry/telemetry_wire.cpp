#include "telemetry_wire.h"

#include <algorithm>
#include <cstdio>

#include "../Motion/phx/wire.h"  // fmt_wire_double — matches Rust push_kv_f

namespace rf {

namespace {

void push_kv_f(std::string& s, const char* key, double v) {
    s.push_back(',');
    s += key;
    s.push_back('=');
    s += phx::fmt_wire_double(v);
}

void push_kv_str(std::string& s, const char* key, const std::string& v) {
    s.push_back(',');
    s += key;
    s.push_back('=');
    s += v;
}

void push_kv_u64(std::string& s, const char* key, uint64_t v) {
    push_kv_str(s, key, std::to_string(v));
}

void push_kv_i64(std::string& s, const char* key, int64_t v) {
    push_kv_str(s, key, std::to_string(v));
}

void push_kv_bit(std::string& s, const char* key, bool v) {
    s.push_back(',');
    s += key;
    s.push_back('=');
    s.push_back(v ? '1' : '0');
}

std::string fixed(double v, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return buf;
}

}  // namespace

std::string TelemetrySnapshot::encode() const {
    std::string s;
    s.reserve(768);

    // v1 core (exact key names / order the pre-v2 dashboards expect).
    s += "state=";
    s += state;
    push_kv_f(s, "voltage", voltage);
    push_kv_bit(s, "ball", ball_found);
    push_kv_f(s, "px", ball_px);
    push_kv_f(s, "py", ball_py);
    push_kv_f(s, "r", ball_radius);
    push_kv_f(s, "bearing", ball_bearing);
    push_kv_f(s, "conf", ball_confidence);
    push_kv_u64(s, "ts_ms", robot_ts_ms);

    // v2 identity & link.
    s += ",proto=2";
    push_kv_str(s, "rid", rid);
    push_kv_u64(s, "seq", seq);
    push_kv_u64(s, "up_ms", up_ms);
    push_kv_str(s, "ifip", ifip);
    push_kv_f(s, "vmin", vmin);
    push_kv_u64(s, "m_ok", m_ok);
    push_kv_u64(s, "m_exp", m_exp);
    push_kv_i64(s, "cmd_age_ms", cmd_age_ms);
    push_kv_u64(s, "cmd_rx", cmd_rx);
    push_kv_i64(s, "cmd_last_id", cmd_last_id);
    push_kv_bit(s, "ard", arduino_connected);
    push_kv_bit(s, "cam", camera_running);
    push_kv_bit(s, "estop", estop);
    push_kv_u64(s, "tx_err", tx_err);
    push_kv_f(s, "cycle_ms", cycle_ms);

    // Per-motor block (sorted by id): m{id}_{ok,mode,fault,temp,volt,vel,cur,cal}.
    std::vector<MotorTelem> sorted = motors;
    std::sort(sorted.begin(), sorted.end(),
              [](const MotorTelem& a, const MotorTelem& b) { return a.id < b.id; });
    for (const MotorTelem& m : sorted) {
        const std::string p = ",m" + std::to_string(m.id) + "_";
        s += p;
        s += "ok=";
        s.push_back(m.ok ? '1' : '0');
        s += p + "mode=" + std::to_string(m.mode);
        s += p + "fault=" + std::to_string(m.fault);
        s += p + "temp=" + fixed(m.temperature, 1);
        s += p + "volt=" + fixed(m.voltage, 2);
        s += p + "vel=" + fixed(m.velocity, 3);
        s += p + "cur=" + fixed(m.current, 2);
        s += p + "cal=" + m.cal;
    }

    // v2+ additive: IMU / odometry / loop health (parsers ignore unknowns).
    push_kv_f(s, "imu_yaw_dps", imu_yaw_dps);
    push_kv_f(s, "heading_deg", heading_deg);
    push_kv_f(s, "odo_vx", odo_vx);
    push_kv_f(s, "odo_vy", odo_vy);
    push_kv_f(s, "odo_w", odo_w);
    push_kv_f(s, "loop_ms", loop_ms);
    push_kv_f(s, "loop_jitter_ms", loop_jitter_ms);
    push_kv_bit(s, "imu_ok", imu_ok);
    // calibration / self-heal status.
    push_kv_bit(s, "needs_cal", needs_cal);
    push_kv_str(s, "cal_state", cal_state);
    push_kv_u64(s, "cfg_fixed", cfg_fixed);
    // MV2 move-executor status. `mv=2` is the capability flag the server
    // gates MV2 frames on — never remove it while the executor exists.
    s += ",mv=2";
    push_kv_i64(s, "mv_seq", mv_seq);
    push_kv_i64(s, "wd", wd_state);
    push_kv_str(s, "mv_kind", mv_kind);
    push_kv_i64(s, "tgt_mm", tgt_dist_mm);
    return s;
}

}  // namespace rf
