#include "motion_logger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

namespace rf {
namespace {

void number(std::ostream& out, double value) {
    if (std::isfinite(value)) out << value;
    else out << "null";
}

void escaped(std::ostream& out, const std::string& value) {
    out << '"';
    for (const char c : value) {
        if (c == '"' || c == '\\') out << '\\';
        if (static_cast<unsigned char>(c) >= 0x20) out << c;
    }
    out << '"';
}

template <typename Array>
void numeric_array(std::ostream& out, const Array& values) {
    out << '[';
    bool first = true;
    for (const auto value : values) {
        if (!first) out << ',';
        first = false;
        number(out, static_cast<double>(value));
    }
    out << ']';
}

void bool_array(std::ostream& out, const std::array<bool, 4>& values) {
    out << '[';
    for (int i = 0; i < 4; ++i) {
        if (i) out << ',';
        out << (values[i] ? "true" : "false");
    }
    out << ']';
}

void vec(std::ostream& out, const phx::Vec2& value) {
    out << '['; number(out, value.x); out << ','; number(out, value.y); out << ']';
}

void twist(std::ostream& out, const phx::Twist& value) {
    out << '['; number(out, value.lin.x); out << ',';
    number(out, value.lin.y); out << ','; number(out, value.ang); out << ']';
}

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm value{};
#ifdef _WIN32
    localtime_s(&value, &time);
#else
    localtime_r(&time, &value);
#endif
    std::ostringstream out;
    out << std::put_time(&value, "%Y%m%d_%H%M%S");
    return out.str();
}

}  // namespace

std::string encode_motion_telemetry_header(int robot_id,
                                           const std::string& controller_abi) {
    std::ostringstream out;
    out << "{\"record_type\":\"schema\",\"schema_version\":"
        << kMotionTelemetrySchema << ",\"robot_id\":" << robot_id
        << ",\"controller_abi\":";
    escaped(out, controller_abi);
    out << ",\"clock\":\"steady_monotonic_seconds\""
        << ",\"frames\":{\"global\":\"SSL field +X,+Y,CCW\""
        << ",\"body\":\"robot +X forward,+Y left,CCW\""
        << ",\"wheel_order\":\"CAN [FR,RR,RL,FL]\"}"
        << ",\"units\":{\"position\":\"m\",\"heading\":\"rad\""
        << ",\"velocity\":\"m/s\",\"angular_velocity\":\"rad/s\""
        << ",\"acceleration\":\"m/s^2\",\"wheel\":\"output_rev/s\""
        << ",\"current\":\"A_q\",\"voltage\":\"V\""
        << ",\"temperature\":\"degC\"}}";
    return out.str();
}

std::string encode_motion_telemetry_sample(const MotionTelemetrySample& s) {
    std::ostringstream out;
    out.precision(10);
    out << "{\"record_type\":\"sample\",\"schema_version\":"
        << kMotionTelemetrySchema << ",\"seq\":" << s.sequence
        << ",\"robot_id\":" << s.robot_id << ",\"skill_id\":" << s.skill_id;
    out << ",\"t_mono_s\":"; number(out, s.mono_time_s);
    out << ",\"dt_s\":"; number(out, s.dt_s);
    out << ",\"control_elapsed_s\":"; number(out, s.control_elapsed_s);
    out << ",\"command_age_s\":"; number(out, s.command_age_s);
    out << ",\"target_pose_global\":["; number(out, s.target_pose.pos.x); out << ',';
    number(out, s.target_pose.pos.y); out << ','; number(out, s.target_pose.heading); out << ']';
    out << ",\"estimate_pose_global\":["; number(out, s.estimate_pose.pos.x); out << ',';
    number(out, s.estimate_pose.pos.y); out << ','; number(out, s.estimate_pose.heading); out << ']';
    out << ",\"estimate_velocity_global\":"; twist(out, s.estimate_velocity_global);
    out << ",\"reference_pose_global\":["; number(out, s.reference.pose.pos.x); out << ',';
    number(out, s.reference.pose.pos.y); out << ','; number(out, s.reference.pose.heading); out << ']';
    out << ",\"reference_velocity_global\":["; number(out, s.reference.vel.x); out << ',';
    number(out, s.reference.vel.y); out << ','; number(out, s.reference.omega); out << ']';
    out << ",\"reference_acceleration_global\":["; number(out, s.reference.acc.x); out << ',';
    number(out, s.reference.acc.y); out << ','; number(out, s.reference.alpha); out << ']';
    out << ",\"model_body\":"; twist(out, s.model_body);
    out << ",\"adaptive_delta_body\":"; twist(out, s.adaptive_delta);
    out << ",\"rl_proposed_body\":"; twist(out, s.rl_proposed);
    out << ",\"rl_applied_body\":"; twist(out, s.rl_applied);
    out << ",\"safety_output_body\":"; twist(out, s.safety_body);
    out << ",\"wheel_target_rev_s\":"; numeric_array(out, s.wheel_target_rev_s);
    out << ",\"wheel_measured_rev_s\":"; numeric_array(out, s.wheel_measured_rev_s);
    out << ",\"wheel_position_rev\":"; numeric_array(out, s.wheel_position_rev);
    out << ",\"motor_current_a\":"; numeric_array(out, s.motor_current_a);
    out << ",\"motor_voltage_v\":"; numeric_array(out, s.motor_voltage_v);
    out << ",\"motor_temperature_c\":"; numeric_array(out, s.motor_temperature_c);
    out << ",\"motor_fault\":"; numeric_array(out, s.motor_fault);
    out << ",\"motor_mode\":"; numeric_array(out, s.motor_mode);
    out << ",\"motor_replied\":"; bool_array(out, s.motor_replied);
    out << ",\"imu_rate_raw_dps\":"; numeric_array(out, s.imu_rate_dps);
    out << ",\"imu_accel_raw_mps2\":"; numeric_array(out, s.imu_accel_raw_mps2);
    out << ",\"imu_accel_body_mps2\":"; vec(out, s.imu_accel_body_mps2);
    out << ",\"imu_available\":" << (s.imu_available ? "true" : "false");
    out << ",\"vision\":{\"alive\":" << (s.vision_alive ? "true" : "false")
        << ",\"age_s\":"; number(out, s.vision_age_s);
    out << ",\"delay_s\":"; number(out, s.vision_delay_s);
    out << ",\"innovation_m\":"; number(out, s.vision_innovation_m);
    out << ",\"heading_innovation_rad\":"; number(out, s.vision_heading_innovation_rad);
    out << ",\"confidence_available\":"
        << (s.vision_confidence_available ? "true" : "false")
        << ",\"confidence\":"; number(out, s.vision_confidence); out << '}';
    out << ",\"surface\":{\"response_positive_per_s\":";
    numeric_array(out, s.surface.response_positive_per_s);
    out << ",\"response_negative_per_s\":";
    numeric_array(out, s.surface.response_negative_per_s);
    out << ",\"braking_response_per_s\":";
    numeric_array(out, s.surface.braking_response_per_s);
    out << ",\"rolling_drag_per_s\":"; numeric_array(out, s.surface.rolling_drag_per_s);
    out << ",\"confidence_axis\":"; numeric_array(out, s.surface.confidence_axis);
    out << ",\"slip_body\":["; number(out, s.surface.slip_longitudinal_mps); out << ',';
    number(out, s.surface.slip_lateral_mps); out << ',';
    number(out, s.surface.slip_yaw_radps); out << ']';
    out << ",\"battery_response_scale\":"; number(out, s.surface.battery_response_scale);
    out << ",\"actuation_delay_s\":"; number(out, s.surface.actuation_delay_s);
    out << ",\"confidence\":"; number(out, s.surface.confidence);
    out << ",\"coverage\":"; number(out, s.surface.coverage);
    out << ",\"accepted_samples\":" << s.surface.accepted_samples
        << ",\"rejected_samples\":" << s.surface.rejected_samples << '}';
    out << ",\"rl_mode\":"; escaped(out, rl_mode_name(s.rl_mode));
    out << ",\"policy_version\":"; escaped(out, s.policy_version);
    out << ",\"safety_interventions\":" << s.safety_interventions
        << ",\"estimator_applied\":" << (s.estimator_applied ? "true" : "false")
        << ",\"policy_evaluated\":" << (s.policy_evaluated ? "true" : "false")
        << ",\"rl_healthy\":" << (s.rl_healthy ? "true" : "false")
        << ",\"rl_auto_disabled\":" << (s.rl_auto_disabled ? "true" : "false")
        << ",\"motor_saturated\":" << (s.motor_saturated ? "true" : "false")
        << ",\"deadline_missed\":" << (s.deadline_missed ? "true" : "false")
        << ",\"kick_active\":" << (s.kick_active ? "true" : "false")
        << ",\"dribbler_active\":" << (s.dribbler_active ? "true" : "false")
        << ",\"collision\":" << (s.collision ? "true" : "false") << '}';
    return out.str();
}

AsyncMotionLogger::~AsyncMotionLogger() { stop(); }

std::string AsyncMotionLogger::path() const {
    std::lock_guard<std::mutex> lock(path_mutex_);
    return path_;
}

bool AsyncMotionLogger::open_segment() {
    std::string name = "motion_v1_robot" + std::to_string(robot_id_) + "_" +
                       session_stamp_;
    if (segment_index_ > 0) {
        std::ostringstream part;
        part << "_p" << std::setw(3) << std::setfill('0') << segment_index_;
        name += part.str();
    }
    name += ".jsonl";
    const std::string path = (std::filesystem::path(directory_) / name).string();
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_) return false;
    const std::string header =
        encode_motion_telemetry_header(robot_id_, controller_abi_);
    file_ << header << '\n';
    file_.flush();
    if (!file_) return false;
    segment_bytes_ = header.size() + 1;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_ = path;
    }
    return true;
}

void AsyncMotionLogger::enforce_retention() {
    if (limits_.max_total_bytes == 0) return;
    struct Entry {
        std::filesystem::path path;
        std::filesystem::file_time_type mtime;
        uint64_t size;
    };
    std::vector<Entry> entries;
    uint64_t total = 0;
    const std::string active =
        std::filesystem::path(path()).filename().string();
    std::error_code ec;
    for (std::filesystem::directory_iterator it(directory_, ec), end;
         !ec && it != end; it.increment(ec)) {
        std::error_code fec;
        if (!it->is_regular_file(fec) || fec) continue;
        const std::string name = it->path().filename().string();
        if (name.rfind("motion_v1_", 0) != 0) continue;
        if (name.size() < 6 || name.substr(name.size() - 6) != ".jsonl") continue;
        const uint64_t size = it->file_size(fec);
        if (fec) continue;
        total += size;
        if (name == active) continue;  // never delete the active file
        const auto mtime = std::filesystem::last_write_time(it->path(), fec);
        if (fec) continue;
        entries.push_back({it->path(), mtime, size});
    }
    std::sort(entries.begin(), entries.end(),
              [](const Entry& a, const Entry& b) { return a.mtime < b.mtime; });
    for (const Entry& entry : entries) {
        if (total <= limits_.max_total_bytes) break;
        std::error_code rec;
        if (std::filesystem::remove(entry.path, rec) && !rec) {
            total -= entry.size;
            retention_deleted_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

bool AsyncMotionLogger::start(const std::string& directory, int robot_id,
                              double rate_hz, const std::string& controller_abi,
                              const MotionLoggerLimits& limits) {
    stop();
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) return false;
    directory_ = directory;
    robot_id_ = robot_id;
    controller_abi_ = controller_abi;
    limits_ = limits;
    session_stamp_ = timestamp();
    segment_index_ = 0;
    dropped_.store(0, std::memory_order_relaxed);
    rotations_.store(0, std::memory_order_relaxed);
    retention_deleted_.store(0, std::memory_order_relaxed);
    if (!open_segment()) return false;
    enforce_retention();  // trim logs left over from earlier sessions
    period_s_ = 1.0 / std::clamp(rate_hz, 1.0, 250.0);
    next_sample_s_ = 0.0;
    head_ = tail_ = count_ = 0;
    stopping_.store(false, std::memory_order_release);
    healthy_.store(true, std::memory_order_release);
    thread_ = std::thread(&AsyncMotionLogger::worker, this);
    return true;
}

void AsyncMotionLogger::stop() {
    stopping_.store(true, std::memory_order_release);
    ready_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
    healthy_.store(false, std::memory_order_relaxed);
}

bool AsyncMotionLogger::due(double mono_time_s) {
    if (!healthy()) return false;
    if (next_sample_s_ == 0.0 || mono_time_s >= next_sample_s_) {
        next_sample_s_ = mono_time_s + period_s_;
        return true;
    }
    return false;
}

bool AsyncMotionLogger::push(const MotionTelemetrySample& sample) {
    if (!healthy()) return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == kCapacity) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        queue_[head_] = sample;
        head_ = (head_ + 1) % kCapacity;
        ++count_;
    }
    ready_.notify_one();
    return true;
}

void AsyncMotionLogger::worker() {
    while (true) {
        MotionTelemetrySample sample;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [&] {
                return count_ > 0 || stopping_.load(std::memory_order_acquire);
            });
            if (count_ == 0 && stopping_.load(std::memory_order_acquire)) break;
            sample = queue_[tail_];
            tail_ = (tail_ + 1) % kCapacity;
            --count_;
        }
        const std::string line = encode_motion_telemetry_sample(sample);
        file_ << line << '\n';
        if (!file_) {
            healthy_.store(false, std::memory_order_release);
            stopping_.store(true, std::memory_order_release);
            continue;
        }
        segment_bytes_ += line.size() + 1;
        if (limits_.max_file_bytes != 0 &&
            segment_bytes_ >= limits_.max_file_bytes) {
            file_.flush();
            file_.close();
            ++segment_index_;
            if (!open_segment()) {
                healthy_.store(false, std::memory_order_release);
                stopping_.store(true, std::memory_order_release);
                continue;
            }
            rotations_.fetch_add(1, std::memory_order_relaxed);
            enforce_retention();
        }
    }
    file_.flush();
}

}  // namespace rf
