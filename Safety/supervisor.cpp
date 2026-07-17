#include "supervisor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rf {

void Supervisor::trip(const StopReason& reason) {
    estop_latched_ = true;
    if (std::find(latched_reasons_.begin(), latched_reasons_.end(), reason) ==
        latched_reasons_.end()) {
        latched_reasons_.push_back(reason);
    }
}

void Supervisor::clear() {
    estop_latched_ = false;
    latched_reasons_.clear();
    temp_bad_.clear();
    cur_bad_.clear();
    fault_bad_.clear();
    volt_bad_ = 0;
    healthy_since_ms_.reset();
}

BodyTwist Supervisor::shape_twist(const BodyTwist& t) const {
    double lin_cap = 0.0, ang_cap = 0.0;
    switch (cfg_.mode) {
        case DriveMode::Safe:
            lin_cap = cfg_.safe_linear_mps;
            ang_cap = cfg_.safe_angular_rps;
            break;
        case DriveMode::Capped:
            lin_cap = cfg_.capped_linear_mps;
            ang_cap = cfg_.capped_angular_rps;
            break;
        case DriveMode::Unsafe:
            lin_cap = std::numeric_limits<double>::infinity();
            ang_cap = std::numeric_limits<double>::infinity();
            break;
    }
    const double speed = std::sqrt(t.vx * t.vx + t.vy * t.vy);
    double vx = t.vx, vy = t.vy;
    if (speed > lin_cap && speed > 0.0) {
        const double s = lin_cap / speed;
        vx *= s;
        vy *= s;
    }
    const double w = std::clamp(t.w, -ang_cap, ang_cap);
    return BodyTwist{vx, vy, w};
}

std::optional<StopReason> Supervisor::motion_gate(
    uint64_t now_ms, std::optional<uint64_t> last_cmd_ms) const {
    if (estop_latched_) {
        return latched_reasons_.empty() ? StopReason::estop() : latched_reasons_.front();
    }
    if (!last_cmd_ms) return StopReason::command_stale();
    const uint64_t age = now_ms >= *last_cmd_ms ? now_ms - *last_cmd_ms : 0;
    if (age > cfg_.command_timeout_ms) return StopReason::command_stale();
    return std::nullopt;
}

std::vector<StopReason> Supervisor::observe(const std::vector<MotorObs>& motors,
                                            double avg_voltage) {
    std::vector<StopReason> newly;
    const uint32_t g = cfg_.fault_grace_ticks;
    const uint32_t gc = std::max<uint32_t>(cfg_.current_grace_ticks, 1);

    for (const MotorObs& m : motors) {
        if (!m.replied) {
            continue;  // stale numbers; don't evaluate thresholds on them
        }
        // Hard moteus fault code latched on the controller.
        if (m.fault != 0) {
            uint32_t& c = fault_bad_[m.id];
            ++c;
            if (c >= g) latch(StopReason::motor_fault(m.id), newly);
        } else {
            fault_bad_[m.id] = 0;
        }
        // Over-temperature.
        if (m.temperature >= cfg_.trip_temp_c) {
            uint32_t& c = temp_bad_[m.id];
            ++c;
            if (c >= g) latch(StopReason::over_temp(m.id), newly);
        } else {
            temp_bad_[m.id] = 0;
        }
        // Over-current (sustained).
        if (std::abs(m.current) >= cfg_.trip_current_a) {
            uint32_t& c = cur_bad_[m.id];
            ++c;
            if (c >= gc) latch(StopReason::over_current(m.id), newly);
        } else {
            cur_bad_[m.id] = 0;
        }
    }

    // Under-voltage (only when we actually have a reading > 0).
    if (avg_voltage > 0.0 && avg_voltage < cfg_.min_bus_voltage) {
        ++volt_bad_;
        if (volt_bad_ >= g) latch(StopReason::under_voltage(), newly);
    } else {
        volt_bad_ = 0;
    }

    // Auto-recovery health: every expected motor replying and comfortably
    // inside the envelope (hysteresis so we don't flap on the threshold),
    // and the bus voltage actually read and healthy.
    observed_healthy_ =
        !motors.empty() &&
        std::all_of(motors.begin(), motors.end(),
                    [&](const MotorObs& m) {
                        return m.replied && m.fault == 0 &&
                               m.temperature < cfg_.trip_temp_c - 10.0 &&
                               std::abs(m.current) < cfg_.trip_current_a * 0.8;
                    }) &&
        avg_voltage > cfg_.min_bus_voltage + 0.5;

    return newly;
}

std::optional<std::vector<StopReason>> Supervisor::try_recover(uint64_t now_ms,
                                                               bool operator_idle) {
    if (!estop_latched_ || cfg_.recovery_s <= 0.0) {
        healthy_since_ms_.reset();
        return std::nullopt;
    }
    if (!observed_healthy_ || !operator_idle) {
        healthy_since_ms_.reset();
        return std::nullopt;
    }
    if (!healthy_since_ms_) healthy_since_ms_ = now_ms;
    const uint64_t since = *healthy_since_ms_;
    const uint64_t elapsed = now_ms >= since ? now_ms - since : 0;
    if (elapsed < static_cast<uint64_t>(cfg_.recovery_s * 1000.0)) {
        return std::nullopt;
    }
    std::vector<StopReason> reasons = std::move(latched_reasons_);
    clear();
    return reasons;
}

void Supervisor::latch(const StopReason& reason, std::vector<StopReason>& newly) {
    if (std::find(latched_reasons_.begin(), latched_reasons_.end(), reason) ==
        latched_reasons_.end()) {
        latched_reasons_.push_back(reason);
        newly.push_back(reason);
    }
    estop_latched_ = true;
}

}  // namespace rf
