#include "adaptive.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string_view>
#include <vector>

#include "phx/angle.h"

namespace rf {
namespace {

double component(const phx::Twist& value, int axis) {
    if (axis == 0) return value.lin.x;
    if (axis == 1) return value.lin.y;
    return value.ang;
}

void set_component(phx::Twist& value, int axis, double item) {
    if (axis == 0) value.lin.x = item;
    else if (axis == 1) value.lin.y = item;
    else value.ang = item;
}

bool finite_twist(const phx::Twist& value) {
    return value.lin.finite() && std::isfinite(value.ang);
}

double slew(double previous, double desired, double amount, bool& limited) {
    const double delta = desired - previous;
    const double bounded = std::clamp(delta, -std::max(0.0, amount), std::max(0.0, amount));
    if (std::fabs(bounded - delta) > 1e-12) limited = true;
    return previous + bounded;
}

std::size_t key_position(const std::string& text, std::string_view key) {
    return text.find(std::string("\"") + std::string(key) + "\"");
}

bool parse_number(const std::string& text, std::string_view key, double& out) {
    std::size_t pos = key_position(text, key);
    if (pos == std::string::npos) return false;
    pos = text.find(':', pos);
    if (pos == std::string::npos) return false;
    const char* begin = text.c_str() + pos + 1;
    char* end = nullptr;
    out = std::strtod(begin, &end);
    return end != begin && std::isfinite(out);
}

bool parse_string(const std::string& text, std::string_view key, std::string& out) {
    std::size_t pos = key_position(text, key);
    if (pos == std::string::npos) return false;
    pos = text.find(':', pos);
    if (pos == std::string::npos) return false;
    pos = text.find('"', pos + 1);
    if (pos == std::string::npos) return false;
    const std::size_t end = text.find('"', pos + 1);
    if (end == std::string::npos) return false;
    out = text.substr(pos + 1, end - pos - 1);
    return true;
}

bool parse_array(const std::string& text, std::string_view key,
                 std::vector<double>& out) {
    std::size_t pos = key_position(text, key);
    if (pos == std::string::npos) return false;
    pos = text.find('[', pos);
    if (pos == std::string::npos) return false;
    const std::size_t end = text.find(']', pos + 1);
    if (end == std::string::npos) return false;
    out.clear();
    const char* cursor = text.c_str() + pos + 1;
    const char* finish = text.c_str() + end;
    while (cursor < finish) {
        while (cursor < finish &&
               (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' ||
                *cursor == '\n' || *cursor == ',')) {
            ++cursor;
        }
        if (cursor >= finish) break;
        char* parsed_end = nullptr;
        const double value = std::strtod(cursor, &parsed_end);
        if (parsed_end == cursor || !std::isfinite(value)) return false;
        out.push_back(value);
        cursor = parsed_end;
    }
    return true;
}

template <typename Array>
void json_array(std::ostream& out, const Array& values) {
    out << '[';
    bool first = true;
    for (const auto value : values) {
        if (!first) out << ',';
        first = false;
        out << value;
    }
    out << ']';
}

}  // namespace

const char* rl_mode_name(RlMode mode) {
    switch (mode) {
        case RlMode::Off: return "off";
        case RlMode::Collect: return "collect";
        case RlMode::Shadow: return "shadow";
        case RlMode::Bounded: return "bounded";
    }
    return "off";
}

RlMode parse_rl_mode(const std::string& value) {
    if (value == "collect") return RlMode::Collect;
    if (value == "shadow") return RlMode::Shadow;
    if (value == "bounded") return RlMode::Bounded;
    return RlMode::Off;
}

// ---------------------------------------------------------------- surface estimator

SurfaceEstimator::SurfaceEstimator(const SurfaceEstimatorConfig& cfg) : cfg_(cfg) {
    reset();
}

int SurfaceEstimator::bucket_index(int axis, bool braking, bool positive) {
    return axis * 4 + (braking ? 2 : 0) + (positive ? 1 : 0);
}

void SurfaceEstimator::reset() {
    for (int axis = 0; axis < 3; ++axis) {
        for (int phase = 0; phase < 2; ++phase) {
            for (int direction = 0; direction < 2; ++direction) {
                Rls2& bucket = buckets_[bucket_index(axis, phase != 0, direction != 0)];
                bucket = Rls2{};
                bucket.response = cfg_.default_response_per_s[axis];
                bucket.drag = cfg_.default_drag_per_s;
            }
        }
    }
    command_history_.fill(phx::Twist{});
    delay_score_.fill(10.0);
    history_head_ = 0;
    history_count_ = 0;
    delay_samples_ = 0;
    delay_updates_ = 0;
    previous_measured_body_ = {};
    have_previous_ = false;
    signed_slip_ = {};
    bus_voltage_v_ = 0.0;
    previous_correction_ = {};
    last_now_s_ = 0.0;
    last_dt_s_ = 0.004;
}

double SurfaceEstimator::bucket_confidence(const Rls2& bucket, double now_s) const {
    const double excitation = 1.0 - std::exp(-bucket.excitation / 80.0);
    const double covariance = 1.0 / (1.0 + 0.02 * (bucket.p00 + bucket.p11));
    const double age = std::max(0.0, now_s - bucket.last_update_s);
    const double freshness = std::exp(-age / std::max(0.1, cfg_.confidence_decay_s));
    return std::clamp(excitation * covariance * freshness, 0.0, 1.0);
}

void SurfaceEstimator::decay(double dt) {
    for (int axis = 0; axis < 3; ++axis) {
        for (int i = axis * 4; i < axis * 4 + 4; ++i) {
            Rls2& bucket = buckets_[i];
            const double age = std::max(0.0, last_now_s_ - bucket.last_update_s);
            if (age < cfg_.confidence_decay_s) continue;
            const double a = 1.0 - std::exp(-dt / std::max(0.1, cfg_.parameter_decay_s));
            bucket.response += a * (cfg_.default_response_per_s[axis] - bucket.response);
            bucket.drag += a * (cfg_.default_drag_per_s - bucket.drag);
        }
    }
}

phx::Twist SurfaceEstimator::delayed_command() const {
    if (history_count_ == 0) return {};
    const int lag = std::min(delay_samples_, history_count_ - 1);
    const int size = static_cast<int>(command_history_.size());
    return command_history_[(history_head_ - lag + size) % size];
}

void SurfaceEstimator::update_delay_scores(const phx::Twist& measured_accel,
                                           const phx::Twist& measured_body) {
    const int maximum = std::min({cfg_.max_delay_samples, 15, history_count_ - 1});
    if (maximum < 0) return;
    const int size = static_cast<int>(command_history_.size());
    for (int lag = 0; lag <= maximum; ++lag) {
        const phx::Twist& command =
            command_history_[(history_head_ - lag + size) % size];
        double error = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double scale = axis < 2 ? 4.0 : 12.0;
            const double predicted =
                cfg_.default_response_per_s[axis] *
                    (component(command, axis) - component(measured_body, axis)) -
                cfg_.default_drag_per_s * component(measured_body, axis);
            const double normalized =
                (component(measured_accel, axis) - predicted) / scale;
            error += std::min(25.0, normalized * normalized);
        }
        delay_score_[lag] = 0.99 * delay_score_[lag] + 0.01 * error;
    }
    ++delay_updates_;
    if (delay_updates_ >= 100) {
        int best = 0;
        for (int lag = 1; lag <= maximum; ++lag) {
            if (delay_score_[lag] < delay_score_[best]) best = lag;
        }
        // Delay changes by at most one sample per decision so one noisy event
        // cannot move the regressor across the full history.
        if (best > delay_samples_) ++delay_samples_;
        else if (best < delay_samples_) --delay_samples_;
        delay_updates_ = 0;
    }
}

void SurfaceEstimator::update(double now_s, double dt_in,
                              const phx::Twist& stable_body,
                              const EstimatorOutput& est,
                              const RuntimeFeedback& feedback,
                              bool learn_allowed) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    last_now_s_ = now_s;
    last_dt_s_ = dt;
    history_head_ = (history_head_ + 1) % static_cast<int>(command_history_.size());
    command_history_[history_head_] = finite_twist(stable_body) ? stable_body : phx::Twist{};
    history_count_ = std::min(history_count_ + 1,
                              static_cast<int>(command_history_.size()));

    phx::Twist measured = est.vel_body;
    measured.ang = est.omega;
    if (!finite_twist(measured)) {
        have_previous_ = false;
        decay(dt);
        return;
    }
    if (!have_previous_) {
        previous_measured_body_ = measured;
        have_previous_ = true;
        return;
    }

    phx::Twist acceleration;
    acceleration.lin = (measured.lin - previous_measured_body_.lin) / dt;
    acceleration.ang = (measured.ang - previous_measured_body_.ang) / dt;
    previous_measured_body_ = measured;

    // Fused velocity versus raw wheel odometry is the directly observable
    // slip indicator. Keep signed values for compensation and reporting.
    const phx::Twist slip{
        feedback.wheel_odo_body.lin.x - measured.lin.x,
        feedback.wheel_odo_body.lin.y - measured.lin.y,
        feedback.wheel_odo_body.ang - measured.ang};
    if (finite_twist(slip)) {
        const double a = 1.0 - std::exp(-dt / 0.30);
        signed_slip_.lin += (slip.lin - signed_slip_.lin) * a;
        signed_slip_.ang += (slip.ang - signed_slip_.ang) * a;
    }
    if (std::isfinite(feedback.bus_voltage_v) && feedback.bus_voltage_v > 1.0) {
        bus_voltage_v_ = feedback.bus_voltage_v;
    }

    bool healthy = cfg_.enabled && learn_allowed && est.vision_alive &&
        est.vision_age_s <= cfg_.max_vision_age_s && !feedback.motor_saturated &&
        !feedback.kick_active && !feedback.dribbler_active && !feedback.collision &&
        !feedback.control_deadline_missed;
    if (feedback.imu_accel_available &&
        feedback.imu_accel_body_mps2.norm() > cfg_.disturbance_accel_mps2) {
        healthy = false;
    }
    for (int i = 0; i < 4; ++i) {
        if (feedback.wheel_replied[i] &&
            (!std::isfinite(feedback.wheel_current_a[i]) ||
             !std::isfinite(feedback.wheel_temperature_c[i]) ||
             feedback.wheel_fault[i] != 0 ||
             std::fabs(feedback.wheel_current_a[i]) >= cfg_.max_learning_current_a ||
             feedback.wheel_temperature_c[i] >= cfg_.max_learning_temperature_c)) {
            healthy = false;
        }
    }
    if (std::fabs(acceleration.lin.x) > cfg_.max_linear_accel_mps2 ||
        std::fabs(acceleration.lin.y) > cfg_.max_linear_accel_mps2 ||
        std::fabs(acceleration.ang) > cfg_.max_angular_accel_radps2) {
        healthy = false;
    }
    if (!healthy) {
        decay(dt);
        return;
    }

    update_delay_scores(acceleration, measured);
    const phx::Twist command = delayed_command();
    const double lambda = std::clamp(cfg_.forgetting_factor, 0.95, 1.0);
    for (int axis = 0; axis < 3; ++axis) {
        const double velocity = component(measured, axis);
        const double error = component(command, axis) - velocity;
        const bool braking = velocity * error < 0.0 && std::fabs(velocity) > 0.05;
        const bool positive = std::fabs(component(command, axis)) > 0.02
                                  ? component(command, axis) >= 0.0
                                  : error >= 0.0;
        Rls2& bucket = buckets_[bucket_index(axis, braking, positive)];
        if (std::fabs(error) < cfg_.min_velocity_error_mps) {
            ++bucket.rejected;
            continue;
        }
        const double phi0 = error;
        const double phi1 = -velocity;
        const double target = component(acceleration, axis);
        const double prediction = bucket.response * phi0 + bucket.drag * phi1;
        const double innovation = target - prediction;
        bucket.residual_scale =
            0.98 * bucket.residual_scale + 0.02 * std::fabs(innovation);
        const double threshold = std::max(0.2, cfg_.huber_sigma * bucket.residual_scale);
        const double weight = std::min(1.0, threshold / std::max(1e-12, std::fabs(innovation)));

        const double pp0 = bucket.p00 * phi0 + bucket.p01 * phi1;
        const double pp1 = bucket.p01 * phi0 + bucket.p11 * phi1;
        const double denominator = lambda / std::max(0.05, weight) +
            phi0 * pp0 + phi1 * pp1;
        if (!std::isfinite(denominator) || denominator <= 1e-12) {
            ++bucket.rejected;
            continue;
        }
        const double k0 = pp0 / denominator;
        const double k1 = pp1 / denominator;
        const double response_step = std::clamp(k0 * innovation, -0.5, 0.5);
        const double drag_step = std::clamp(k1 * innovation, -0.20, 0.20);
        bucket.response = std::clamp(
            bucket.response + response_step,
            cfg_.min_response_per_s[axis], cfg_.max_response_per_s[axis]);
        bucket.drag = std::clamp(bucket.drag + drag_step, 0.0, cfg_.max_drag_per_s);

        const double p00 = (bucket.p00 - k0 * pp0) / lambda;
        const double p01a = (bucket.p01 - k0 * pp1) / lambda;
        const double p01b = (bucket.p01 - k1 * pp0) / lambda;
        const double p11 = (bucket.p11 - k1 * pp1) / lambda;
        bucket.p00 = std::clamp(p00, 1e-6, 1e4);
        bucket.p01 = std::clamp(0.5 * (p01a + p01b), -1e4, 1e4);
        bucket.p11 = std::clamp(p11, 1e-6, 1e4);
        bucket.excitation = std::min(1e6, bucket.excitation +
            std::min(25.0, phi0 * phi0 + 0.1 * phi1 * phi1));
        bucket.last_update_s = now_s;
        ++bucket.accepted;
    }
    decay(dt);
}

phx::Twist SurfaceEstimator::correction(double dt_in,
                                        const phx::Twist& stable_body,
                                        const EstimatorOutput& est,
                                        bool braking_hint) {
    phx::Twist out;
    if (!cfg_.enabled || !finite_twist(stable_body)) {
        previous_correction_ = {};
        return out;
    }
    phx::Twist measured = est.vel_body;
    measured.ang = est.omega;
    for (int axis = 0; axis < 3; ++axis) {
        const double velocity = component(measured, axis);
        const double error = component(stable_body, axis) - velocity;
        const bool braking = braking_hint ||
            (velocity * error < 0.0 && std::fabs(velocity) > 0.05);
        const bool positive = std::fabs(component(stable_body, axis)) > 0.02
                                  ? component(stable_body, axis) >= 0.0
                                  : error >= 0.0;
        const Rls2& bucket = buckets_[bucket_index(axis, braking, positive)];
        if (bucket_confidence(bucket, last_now_s_) < cfg_.confidence_threshold) continue;
        const double nominal_response = cfg_.default_response_per_s[axis];
        const double response_scale = std::clamp(
            nominal_response / std::max(cfg_.min_response_per_s[axis], bucket.response),
            0.85, 1.15);
        const double nominal_drag_ratio = cfg_.default_drag_per_s / nominal_response;
        const double identified_drag_ratio = bucket.drag / std::max(0.1, bucket.response);
        double correction_value =
            (response_scale - 1.0) * error +
            (identified_drag_ratio - nominal_drag_ratio) * velocity;
        if (axis == 0) correction_value += cfg_.slip_compensation_gain * signed_slip_.lin.x;
        if (axis == 1) correction_value += cfg_.slip_compensation_gain * signed_slip_.lin.y;
        if (axis == 2) correction_value += cfg_.slip_compensation_gain * signed_slip_.ang;
        set_component(out, axis, correction_value);
    }

    const double linear_limit =
        cfg_.correction_limit_fraction * stable_body.lin.norm();
    out.lin = out.lin.clamped(linear_limit);
    out.ang = std::clamp(out.ang,
                         -cfg_.correction_limit_fraction * std::fabs(stable_body.ang),
                         cfg_.correction_limit_fraction * std::fabs(stable_body.ang));
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    bool unused = false;
    out.lin.x = slew(previous_correction_.lin.x, out.lin.x,
                     cfg_.correction_slew_linear_mps2 * dt, unused);
    out.lin.y = slew(previous_correction_.lin.y, out.lin.y,
                     cfg_.correction_slew_linear_mps2 * dt, unused);
    out.ang = slew(previous_correction_.ang, out.ang,
                   cfg_.correction_slew_angular_radps2 * dt, unused);
    previous_correction_ = out;
    return out;
}

SurfaceContext SurfaceEstimator::context(double now_s) const {
    SurfaceContext out;
    const auto average = [](const Rls2& a, const Rls2& b,
                            double default_value, bool response) {
        const double wa = static_cast<double>(a.accepted);
        const double wb = static_cast<double>(b.accepted);
        if (wa + wb < 1.0) return default_value;
        const double va = response ? a.response : a.drag;
        const double vb = response ? b.response : b.drag;
        return (wa * va + wb * vb) / (wa + wb);
    };
    double confidence_sum = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const Rls2& drive_neg = buckets_[bucket_index(axis, false, false)];
        const Rls2& drive_pos = buckets_[bucket_index(axis, false, true)];
        const Rls2& brake_neg = buckets_[bucket_index(axis, true, false)];
        const Rls2& brake_pos = buckets_[bucket_index(axis, true, true)];
        out.response_negative_per_s[axis] = average(
            drive_neg, brake_neg, cfg_.default_response_per_s[axis], true);
        out.response_positive_per_s[axis] = average(
            drive_pos, brake_pos, cfg_.default_response_per_s[axis], true);
        out.braking_response_per_s[axis] = average(
            brake_neg, brake_pos, cfg_.default_response_per_s[axis], true);
        const double drag_a = average(
            drive_neg, drive_pos, cfg_.default_drag_per_s, false);
        const double drag_b = average(
            brake_neg, brake_pos, cfg_.default_drag_per_s, false);
        out.rolling_drag_per_s[axis] = 0.5 * (drag_a + drag_b);
        double axis_confidence = 0.0;
        for (int i = axis * 4; i < axis * 4 + 4; ++i) {
            const Rls2& bucket = buckets_[i];
            axis_confidence = std::max(axis_confidence,
                                       bucket_confidence(bucket, now_s));
            out.accepted_samples += bucket.accepted;
            out.rejected_samples += bucket.rejected;
            out.updated_mono_s = std::max(out.updated_mono_s, bucket.last_update_s);
        }
        out.confidence_axis[axis] = axis_confidence;
        confidence_sum += axis_confidence;
    }
    out.confidence = confidence_sum / 3.0;
    out.coverage = std::clamp(
        static_cast<double>(out.accepted_samples) / 600.0, 0.0, 1.0);
    out.slip_longitudinal_mps = signed_slip_.lin.x;
    out.slip_lateral_mps = signed_slip_.lin.y;
    out.slip_yaw_radps = signed_slip_.ang;
    out.battery_response_scale = bus_voltage_v_ > 1.0
        ? std::clamp(bus_voltage_v_ / std::max(1.0, cfg_.nominal_voltage_v), 0.75, 1.10)
        : 1.0;
    out.actuation_delay_s = delay_samples_ * last_dt_s_;
    return out;
}

bool SurfaceEstimator::save_profile(const std::string& path, int robot_id,
                                    const std::string& surface_id) const {
    if (path.empty()) return false;
    std::error_code ec;
    const std::filesystem::path destination(path);
    if (destination.has_parent_path()) {
        std::filesystem::create_directories(destination.parent_path(), ec);
        if (ec) return false;
    }
    const std::filesystem::path temporary = destination.string() + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    if (!out) return false;
    out.precision(17);
    std::array<double, 12> response{}, drag{}, p00{}, p01{}, p11{}, excitation{};
    std::array<double, 12> accepted{}, rejected{}, updated{};
    for (int i = 0; i < 12; ++i) {
        response[i] = buckets_[i].response;
        drag[i] = buckets_[i].drag;
        p00[i] = buckets_[i].p00;
        p01[i] = buckets_[i].p01;
        p11[i] = buckets_[i].p11;
        excitation[i] = buckets_[i].excitation;
        accepted[i] = static_cast<double>(buckets_[i].accepted);
        rejected[i] = static_cast<double>(buckets_[i].rejected);
        updated[i] = buckets_[i].last_update_s;
    }
    out << "{\n\"schema\":1,\n\"robot_id\":" << robot_id
        << ",\n\"surface_id\":\"" << surface_id << "\",\n";
    out << "\"response\":"; json_array(out, response); out << ",\n";
    out << "\"drag\":"; json_array(out, drag); out << ",\n";
    out << "\"p00\":"; json_array(out, p00); out << ",\n";
    out << "\"p01\":"; json_array(out, p01); out << ",\n";
    out << "\"p11\":"; json_array(out, p11); out << ",\n";
    out << "\"excitation\":"; json_array(out, excitation); out << ",\n";
    out << "\"accepted\":"; json_array(out, accepted); out << ",\n";
    out << "\"rejected\":"; json_array(out, rejected); out << ",\n";
    out << "\"updated\":"; json_array(out, updated); out << "\n}\n";
    out.flush();
    if (!out) return false;
    out.close();
    const std::filesystem::path previous = destination.string() + ".previous";
    std::filesystem::remove(previous, ec);
    ec.clear();
    if (std::filesystem::exists(destination, ec)) {
        ec.clear();
        std::filesystem::rename(destination, previous, ec);
        if (ec) return false;
    }
    ec.clear();
    std::filesystem::rename(temporary, destination, ec);
    if (ec) {
        std::error_code restore_ec;
        if (std::filesystem::exists(previous, restore_ec)) {
            std::filesystem::rename(previous, destination, restore_ec);
        }
        return false;
    }
    return true;
}

bool SurfaceEstimator::load_profile(const std::string& path,
                                    int expected_robot_id,
                                    const std::string& expected_surface_id) {
    std::ifstream in(path);
    if (!in) return false;
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    double schema = 0.0, robot = -1.0;
    std::string surface;
    if (!parse_number(text, "schema", schema) || schema != 1.0 ||
        !parse_number(text, "robot_id", robot) ||
        static_cast<int>(robot) != expected_robot_id ||
        !parse_string(text, "surface_id", surface) ||
        surface != expected_surface_id) {
        return false;
    }
    std::vector<double> response, drag, p00, p01, p11, excitation,
        accepted, rejected, updated;
    if (!parse_array(text, "response", response) || response.size() != 12 ||
        !parse_array(text, "drag", drag) || drag.size() != 12 ||
        !parse_array(text, "p00", p00) || p00.size() != 12 ||
        !parse_array(text, "p01", p01) || p01.size() != 12 ||
        !parse_array(text, "p11", p11) || p11.size() != 12 ||
        !parse_array(text, "excitation", excitation) || excitation.size() != 12 ||
        !parse_array(text, "accepted", accepted) || accepted.size() != 12 ||
        !parse_array(text, "rejected", rejected) || rejected.size() != 12 ||
        !parse_array(text, "updated", updated) || updated.size() != 12) {
        return false;
    }
    for (int i = 0; i < 12; ++i) {
        const int axis = i / 4;
        if (!std::isfinite(response[i]) || !std::isfinite(drag[i]) ||
            !std::isfinite(p00[i]) || !std::isfinite(p01[i]) ||
            !std::isfinite(p11[i]) || !std::isfinite(excitation[i]) ||
            !std::isfinite(accepted[i]) || !std::isfinite(rejected[i]) ||
            !std::isfinite(updated[i]) ||
            response[i] < cfg_.min_response_per_s[axis] ||
            response[i] > cfg_.max_response_per_s[axis] ||
            drag[i] < 0.0 || drag[i] > cfg_.max_drag_per_s ||
            p00[i] <= 0.0 || p11[i] <= 0.0) {
            return false;
        }
    }
    for (int i = 0; i < 12; ++i) {
        buckets_[i].response = response[i];
        buckets_[i].drag = drag[i];
        buckets_[i].p00 = p00[i];
        buckets_[i].p01 = p01[i];
        buckets_[i].p11 = p11[i];
        // A steady-clock timestamp cannot be carried across boots. Reuse the
        // bounded parameter prior, but require fresh ordinary motion before
        // confidence gates can apply it.
        buckets_[i].excitation = 0.25 * std::max(0.0, excitation[i]);
        buckets_[i].accepted = static_cast<uint64_t>(std::max(0.0, accepted[i]));
        buckets_[i].rejected = static_cast<uint64_t>(std::max(0.0, rejected[i]));
        buckets_[i].last_update_s = -1e9;
    }
    return true;
}

// ---------------------------------------------------------------- residual policy

void ResidualPolicy::clear() {
    healthy_ = false;
    hidden_dim_ = 0;
    declared_limit_fraction_ = 0.0;
    version_ = "none";
    error_.clear();
    obs_mean_.fill(0.0);
    obs_scale_.fill(1.0);
    w1_.fill(0.0);
    b1_.fill(0.0);
    w2_.fill(0.0);
    b2_.fill(0.0);
}

bool ResidualPolicy::load(const std::string& path,
                          double configured_limit_fraction) {
    clear();
    if (path.empty()) {
        error_ = "policy path is empty";
        return false;
    }
    std::ifstream in(path);
    if (!in) {
        error_ = "policy file unavailable";
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    double schema = 0.0, observation_dim = 0.0, action_dim = 0.0,
           hidden_dim = 0.0, residual_limit = 0.0;
    std::string abi;
    if (!parse_number(text, "schema", schema) || schema != 1.0 ||
        !parse_number(text, "observation_dim", observation_dim) ||
        observation_dim != kObservationDim ||
        !parse_number(text, "action_dim", action_dim) ||
        action_dim != kActionDim ||
        !parse_number(text, "hidden_dim", hidden_dim) ||
        hidden_dim < 1.0 || hidden_dim > kMaxHidden ||
        !parse_number(text, "residual_limit_fraction", residual_limit) ||
        residual_limit <= 0.0 || residual_limit > configured_limit_fraction + 1e-12 ||
        !parse_string(text, "controller_abi", abi) || abi != kControllerAbi ||
        !parse_string(text, "policy_version", version_)) {
        error_ = "policy schema, dimensions, ABI, or residual limit is incompatible";
        return false;
    }
    std::vector<double> mean, scale, w1, b1, w2, b2;
    hidden_dim_ = static_cast<int>(hidden_dim);
    if (!parse_array(text, "obs_mean", mean) || mean.size() != kObservationDim ||
        !parse_array(text, "obs_scale", scale) || scale.size() != kObservationDim ||
        !parse_array(text, "w1", w1) ||
        w1.size() != static_cast<std::size_t>(hidden_dim_ * kObservationDim) ||
        !parse_array(text, "b1", b1) || b1.size() != static_cast<std::size_t>(hidden_dim_) ||
        !parse_array(text, "w2", w2) ||
        w2.size() != static_cast<std::size_t>(kActionDim * hidden_dim_) ||
        !parse_array(text, "b2", b2) || b2.size() != kActionDim) {
        error_ = "policy tensor sizes are invalid";
        return false;
    }
    for (int i = 0; i < kObservationDim; ++i) {
        if (scale[i] <= 1e-9 || std::fabs(mean[i]) > 1e9 || scale[i] > 1e9) {
            error_ = "policy observation normalization is invalid";
            return false;
        }
        obs_mean_[i] = mean[i];
        obs_scale_[i] = scale[i];
    }
    auto reasonable = [](double value) {
        return std::isfinite(value) && std::fabs(value) <= 100.0;
    };
    if (!std::all_of(w1.begin(), w1.end(), reasonable) ||
        !std::all_of(b1.begin(), b1.end(), reasonable) ||
        !std::all_of(w2.begin(), w2.end(), reasonable) ||
        !std::all_of(b2.begin(), b2.end(), reasonable)) {
        error_ = "policy contains non-finite or unreasonable weights";
        return false;
    }
    std::copy(w1.begin(), w1.end(), w1_.begin());
    std::copy(b1.begin(), b1.end(), b1_.begin());
    std::copy(w2.begin(), w2.end(), w2_.begin());
    std::copy(b2.begin(), b2.end(), b2_.begin());
    declared_limit_fraction_ = residual_limit;
    healthy_ = true;
    return true;
}

std::array<double, ResidualPolicy::kActionDim> ResidualPolicy::infer(
    const std::array<double, kObservationDim>& observation) const {
    std::array<double, kActionDim> action{};
    if (!healthy_) return action;
    std::array<double, kMaxHidden> hidden{};
    for (int row = 0; row < hidden_dim_; ++row) {
        double value = b1_[row];
        for (int col = 0; col < kObservationDim; ++col) {
            const double normalized = std::clamp(
                (observation[col] - obs_mean_[col]) / obs_scale_[col], -8.0, 8.0);
            value += w1_[row * kObservationDim + col] * normalized;
        }
        hidden[row] = std::tanh(value);
    }
    for (int row = 0; row < kActionDim; ++row) {
        double value = b2_[row];
        for (int col = 0; col < hidden_dim_; ++col) {
            value += w2_[row * hidden_dim_ + col] * hidden[col];
        }
        action[row] = std::tanh(value);
    }
    return action;
}

// ---------------------------------------------------------------- augmentation and constraints

MotionAugmentor::MotionAugmentor(const AugmentationConfig& cfg,
                                 const Kinematics& kin)
    : cfg_(cfg), kin_(kin), estimator_(cfg.estimator) {
    if ((cfg_.rl.mode == RlMode::Shadow || cfg_.rl.mode == RlMode::Bounded) &&
        !cfg_.rl.policy_path.empty()) {
        policy_.load(cfg_.rl.policy_path, cfg_.rl.residual_limit_fraction);
    }
}

void MotionAugmentor::reset(const phx::Twist&) {
    previous_residual_ = {};
    previous_total_correction_ = {};
    previous_correction_rate_ = {};
    consecutive_interventions_ = 0;
}

bool MotionAugmentor::reload_policy() {
    rl_auto_disabled_ = false;
    consecutive_interventions_ = 0;
    return policy_.load(cfg_.rl.policy_path, cfg_.rl.residual_limit_fraction);
}

void MotionAugmentor::disable_rl() {
    rl_auto_disabled_ = true;
    previous_residual_ = {};
}

bool MotionAugmentor::telemetry_healthy(const RuntimeFeedback& feedback) const {
    if (!std::isfinite(feedback.bus_voltage_v)) return false;
    if (feedback.bus_voltage_v < cfg_.safety.min_bus_voltage_v) return false;
    for (int i = 0; i < 4; ++i) {
        if (!feedback.wheel_replied[i]) return false;
        if (!std::isfinite(feedback.wheel_current_a[i]) ||
            !std::isfinite(feedback.wheel_temperature_c[i]) ||
            feedback.wheel_fault[i] != 0 ||
            std::fabs(feedback.wheel_current_a[i]) >= cfg_.safety.max_current_a ||
            feedback.wheel_temperature_c[i] >= cfg_.safety.max_temperature_c) {
            return false;
        }
    }
    return true;
}

std::array<double, ResidualPolicy::kObservationDim> MotionAugmentor::observation(
    const MotionSetpoint& sp, const TrajSample& ref,
    const EstimatorOutput& est, const StableControl& stable,
    const RuntimeFeedback& feedback, const SurfaceContext& surface) const {
    std::array<double, ResidualPolicy::kObservationDim> out{};
    const phx::Vec2 pos_error_body =
        (ref.pose.pos - est.pose.pos).rotated(-est.pose.heading);
    const phx::Vec2 ref_velocity_body = ref.vel.rotated(-est.pose.heading);
    const phx::Vec2 ref_accel_body = ref.acc.rotated(-est.pose.heading);
    const phx::Vec2 velocity_error_body =
        ref_velocity_body - est.vel_body.lin;
    out[0] = pos_error_body.x;
    out[1] = pos_error_body.y;
    out[2] = phx::angle_diff(ref.pose.heading, est.pose.heading);
    out[3] = velocity_error_body.x;
    out[4] = velocity_error_body.y;
    out[5] = ref.omega - est.omega;
    out[6] = ref_velocity_body.x;
    out[7] = ref_velocity_body.y;
    out[8] = ref.omega;
    out[9] = ref_accel_body.x;
    out[10] = ref_accel_body.y;
    out[11] = ref.alpha;
    const std::array<double, 4> target = kin_.inverse(BodyTwist{
        stable.body.lin.x, stable.body.lin.y, stable.body.ang});
    for (int i = 0; i < 4; ++i) {
        out[12 + i] = target[i] - feedback.wheel_measured_rev_s[i];
        out[16 + i] = feedback.wheel_current_a[i];
    }
    out[20] = feedback.bus_voltage_v;
    out[21] = *std::max_element(feedback.wheel_temperature_c.begin(),
                                feedback.wheel_temperature_c.end());
    out[22] = feedback.imu_accel_available ? feedback.imu_accel_body_mps2.x : 0.0;
    out[23] = feedback.imu_accel_available ? feedback.imu_accel_body_mps2.y : 0.0;
    out[24] = est.omega;
    out[25] = 0.5 * (surface.response_positive_per_s[0] +
                     surface.response_negative_per_s[0]);
    out[26] = 0.5 * (surface.response_positive_per_s[1] +
                     surface.response_negative_per_s[1]);
    out[27] = 0.5 * (surface.response_positive_per_s[2] +
                     surface.response_negative_per_s[2]);
    out[28] = surface.confidence;
    out[29] = est.vision_age_s;
    out[30] = feedback.motor_saturated ? 1.0 : 0.0;
    out[31] = std::hypot(previous_residual_.lin.norm(), previous_residual_.ang);
    (void)sp;
    return out;
}

AugmentationOutput MotionAugmentor::step(
    double now_s, double dt_in, const MotionSetpoint& sp, const TrajSample& ref,
    const EstimatorOutput& est, const StableControl& stable,
    const RuntimeFeedback& feedback) {
    const double dt = std::clamp(dt_in, 1e-4, 0.05);
    AugmentationOutput out;
    out.body = stable.body;

    const bool supported_motion = stable.energize && !stable.direct_wheels &&
        sp.kind != MotionSetpoint::Kind::Emergency &&
        sp.kind != MotionSetpoint::Kind::WheelVel;
    const bool learn_allowed = supported_motion &&
        sp.kind != MotionSetpoint::Kind::Sine;
    estimator_.update(now_s, dt, stable.body, est, feedback, learn_allowed);
    out.surface = estimator_.context(now_s);
    if (!supported_motion) {
        previous_residual_ = {};
        previous_total_correction_ = {};
        previous_correction_rate_ = {};
        return out;
    }

    const bool braking = est.vel_body.lin.dot(stable.body.lin - est.vel_body.lin) < 0.0;
    const double best_axis_confidence = *std::max_element(
        out.surface.confidence_axis.begin(), out.surface.confidence_axis.end());
    if (cfg_.estimator.enabled &&
        best_axis_confidence >= cfg_.estimator.confidence_threshold) {
        out.adaptive_delta = estimator_.correction(dt, stable.body, est, braking);
        out.adaptive_applied = out.adaptive_delta.lin.norm() > 1e-12 ||
                               std::fabs(out.adaptive_delta.ang) > 1e-12;
    } else if (cfg_.estimator.enabled) {
        out.interventions |= kInterventionConfidence;
    }

    const bool evaluate_policy =
        cfg_.rl.mode == RlMode::Shadow || cfg_.rl.mode == RlMode::Bounded;
    const bool telemetry_ok = telemetry_healthy(feedback);
    const bool deadline_ok = !feedback.control_deadline_missed &&
        (feedback.control_elapsed_s <= 0.0 ||
         feedback.control_elapsed_s <= cfg_.rl.max_loop_elapsed_s);
    out.rl_healthy = evaluate_policy && policy_.healthy() && !rl_auto_disabled_ &&
        telemetry_ok && deadline_ok && est.vision_alive &&
        out.surface.confidence >= cfg_.rl.confidence_threshold;
    if (evaluate_policy && policy_.healthy()) {
        const auto action = policy_.infer(
            observation(sp, ref, est, stable, feedback, out.surface));
        out.policy_evaluated = true;
        const double fraction = std::min(cfg_.rl.residual_limit_fraction,
                                         policy_.declared_limit_fraction());
        const double linear_bound = fraction * stable.body.lin.norm();
        out.rl_proposed.lin = phx::Vec2{action[0], action[1]} * linear_bound;
        out.rl_proposed.lin = out.rl_proposed.lin.clamped(linear_bound);
        const double angular_bound = fraction * std::fabs(stable.body.ang);
        out.rl_proposed.ang = std::clamp(action[2] * angular_bound,
                                         -angular_bound, angular_bound);
        bool residual_slew = false;
        out.rl_proposed.lin.x = slew(
            previous_residual_.lin.x, out.rl_proposed.lin.x,
            cfg_.rl.residual_slew_linear_mps2 * dt, residual_slew);
        out.rl_proposed.lin.y = slew(
            previous_residual_.lin.y, out.rl_proposed.lin.y,
            cfg_.rl.residual_slew_linear_mps2 * dt, residual_slew);
        out.rl_proposed.ang = slew(
            previous_residual_.ang, out.rl_proposed.ang,
            cfg_.rl.residual_slew_angular_radps2 * dt, residual_slew);
        if (residual_slew) out.interventions |= kInterventionResidualSlew;
        previous_residual_ = out.rl_proposed;
    } else if (evaluate_policy) {
        out.interventions |= kInterventionPolicyHealth;
        previous_residual_ = {};
    }
    if (evaluate_policy && !telemetry_ok) out.interventions |= kInterventionTelemetry;
    if (evaluate_policy && !deadline_ok) out.interventions |= kInterventionDeadline;
    if (cfg_.rl.mode == RlMode::Bounded && out.rl_healthy) {
        out.rl_applied = out.rl_proposed;
    } else if (cfg_.rl.mode == RlMode::Bounded) {
        out.interventions |= kInterventionConfidence;
    }

    phx::Twist correction = out.adaptive_delta;
    correction.lin += out.rl_applied.lin;
    correction.ang += out.rl_applied.ang;
    const double correction_fraction =
        std::clamp(cfg_.safety.correction_limit_fraction, 0.0, 0.25);
    const double correction_linear_bound =
        correction_fraction * stable.body.lin.norm();
    const phx::Vec2 unclamped_linear = correction.lin;
    correction.lin = correction.lin.clamped(correction_linear_bound);
    if ((unclamped_linear - correction.lin).norm() > 1e-12) {
        out.interventions |= kInterventionResidualBound;
    }
    const double correction_angular_bound =
        correction_fraction * std::fabs(stable.body.ang);
    const double unclamped_angular = correction.ang;
    correction.ang = std::clamp(correction.ang,
                                 -correction_angular_bound,
                                 correction_angular_bound);
    if (std::fabs(unclamped_angular - correction.ang) > 1e-12) {
        out.interventions |= kInterventionResidualBound;
    }

    // The conventional controller already limits trajectory jerk. Preserve
    // that property after adding adaptive/RL velocity corrections by bounding
    // the change in correction rate as well. This is deterministic and is
    // applied after the combined correction bound, so neither learner can
    // bypass it.
    phx::Twist desired_rate;
    desired_rate.lin = (correction.lin - previous_total_correction_.lin) / dt;
    desired_rate.ang = (correction.ang - previous_total_correction_.ang) / dt;
    bool jerk_limited = false;
    desired_rate.lin.x = slew(
        previous_correction_rate_.lin.x, desired_rate.lin.x,
        std::max(0.0, cfg_.safety.correction_jerk_linear_mps3) * dt,
        jerk_limited);
    desired_rate.lin.y = slew(
        previous_correction_rate_.lin.y, desired_rate.lin.y,
        std::max(0.0, cfg_.safety.correction_jerk_linear_mps3) * dt,
        jerk_limited);
    desired_rate.ang = slew(
        previous_correction_rate_.ang, desired_rate.ang,
        std::max(0.0, cfg_.safety.correction_jerk_angular_radps3) * dt,
        jerk_limited);
    if (jerk_limited) out.interventions |= kInterventionCorrectionJerk;
    correction.lin = previous_total_correction_.lin + desired_rate.lin * dt;
    correction.ang = previous_total_correction_.ang + desired_rate.ang * dt;
    previous_correction_rate_ = desired_rate;
    previous_total_correction_ = correction;
    out.body.lin += correction.lin;
    out.body.ang += correction.ang;

    if (!finite_twist(out.body)) {
        out.body = finite_twist(stable.body) ? stable.body : phx::Twist{};
        out.rl_applied = {};
        out.adaptive_delta = {};
        out.interventions |= kInterventionNonFinite;
    }

    double velocity_limit = cfg_.safety.velocity_max_mps;
    double angular_limit = cfg_.safety.angular_velocity_max_radps;
    if (sp.kind == MotionSetpoint::Kind::Pose) {
        velocity_limit = std::min(velocity_limit, std::max(0.0, sp.vel_max_xy));
        angular_limit = std::min(angular_limit, std::max(0.0, sp.vel_max_w));
    }
    if (out.body.lin.norm() > velocity_limit && velocity_limit > 0.0) {
        out.body.lin = out.body.lin.clamped(velocity_limit);
        out.interventions |= kInterventionVelocity;
    }
    if (std::fabs(out.body.ang) > angular_limit && angular_limit > 0.0) {
        out.body.ang = std::clamp(out.body.ang, -angular_limit, angular_limit);
        out.interventions |= kInterventionVelocity;
    }
    const double peak = kin_.peak_motor_rev_s(
        BodyTwist{out.body.lin.x, out.body.lin.y, out.body.ang});
    if (peak > cfg_.safety.wheel_max_rev_s && peak > 1e-12) {
        const double scale = cfg_.safety.wheel_max_rev_s / peak;
        out.body.lin *= scale;
        out.body.ang *= scale;
        out.interventions |= kInterventionWheelSpeed;
    }

    const uint32_t active_safety = out.interventions &
        (kInterventionResidualBound | kInterventionVelocity |
         kInterventionWheelSpeed | kInterventionNonFinite);
    if (cfg_.rl.mode == RlMode::Bounded &&
        (out.rl_applied.lin.norm() > 1e-12 || std::fabs(out.rl_applied.ang) > 1e-12) &&
        active_safety != 0) {
        ++consecutive_interventions_;
    } else if (consecutive_interventions_ > 0) {
        --consecutive_interventions_;
    }
    if (consecutive_interventions_ >=
        std::max(1, cfg_.rl.max_consecutive_interventions)) {
        rl_auto_disabled_ = true;
        out.rl_auto_disabled = true;
        out.rl_applied = {};
        out.body = stable.body;
        out.body.lin += out.adaptive_delta.lin;
        out.body.ang += out.adaptive_delta.ang;
        if (velocity_limit > 0.0 && out.body.lin.norm() > velocity_limit) {
            out.body.lin = out.body.lin.clamped(velocity_limit);
        }
        if (angular_limit > 0.0 && std::fabs(out.body.ang) > angular_limit) {
            out.body.ang = std::clamp(out.body.ang, -angular_limit, angular_limit);
        }
        const double fallback_peak = kin_.peak_motor_rev_s(
            BodyTwist{out.body.lin.x, out.body.lin.y, out.body.ang});
        if (fallback_peak > cfg_.safety.wheel_max_rev_s && fallback_peak > 1e-12) {
            const double scale = cfg_.safety.wheel_max_rev_s / fallback_peak;
            out.body.lin *= scale;
            out.body.ang *= scale;
        }
        out.interventions |= kInterventionAutoDisable;
    } else {
        out.rl_auto_disabled = rl_auto_disabled_;
    }
    return out;
}

}  // namespace rf
