// Bounded online drivetrain/surface identification and motion augmentation.
//
// This module is pure, deterministic and hardware-free. It owns no clock,
// socket, file watcher or training thread. The real-time path supplies
// monotonic time and the previous Pi3Hat/Moteus feedback sample. Policy
// weights are loaded once from a strict, compatibility-checked actor file;
// policy training never occurs here.
#pragma once

#include <array>
#include <optional>
#include <cstdint>
#include <string>

#include "../Math/kinematics.h"
#include "controller.h"
#include "phx/pose.h"
#include "skills.h"
#include "trajectory.h"

namespace rf {

enum class RlMode { Off, Collect, Shadow, Bounded };

const char* rl_mode_name(RlMode mode);
RlMode parse_rl_mode(const std::string& value);

// Feedback available to the adaptive layer from the preceding CAN/IMU cycle.
// Every field is SI. Availability is explicit; missing sensors are never
// replaced with fabricated values.
struct RuntimeFeedback {
    std::array<double, 4> wheel_measured_rev_s{};
    std::array<double, 4> wheel_current_a{};
    std::array<double, 4> wheel_temperature_c{};
    std::array<int, 4> wheel_fault{};
    std::array<bool, 4> wheel_replied{};
    phx::Twist wheel_odo_body{};
    phx::Vec2 imu_accel_body_mps2{};
    double imu_accel_z_mps2 = 0.0;
    bool imu_accel_available = false;
    double bus_voltage_v = 0.0;
    bool motor_saturated = false;
    bool kick_active = false;
    bool dribbler_active = false;
    bool collision = false;
    bool control_deadline_missed = false;
    double control_elapsed_s = 0.0;
};

struct SurfaceEstimatorConfig {
    bool enabled = false;
    double forgetting_factor = 0.998;
    double huber_sigma = 3.0;
    double min_velocity_error_mps = 0.05;
    double max_linear_accel_mps2 = 15.0;
    double max_angular_accel_radps2 = 80.0;
    double max_learning_current_a = 9.5;
    double max_learning_temperature_c = 65.0;
    double max_vision_age_s = 0.15;
    double disturbance_accel_mps2 = 18.0;
    double confidence_threshold = 0.55;
    double confidence_decay_s = 4.0;
    double parameter_decay_s = 20.0;
    double nominal_voltage_v = 23.0;
    std::array<double, 3> default_response_per_s{{8.0, 6.0, 8.0}};
    std::array<double, 3> min_response_per_s{{1.0, 0.8, 1.0}};
    std::array<double, 3> max_response_per_s{{35.0, 30.0, 45.0}};
    double default_drag_per_s = 0.15;
    double max_drag_per_s = 6.0;
    double correction_limit_fraction = 0.08;
    double correction_slew_linear_mps2 = 0.8;
    double correction_slew_angular_radps2 = 2.0;
    double slip_compensation_gain = 0.20;
    int max_delay_samples = 15;  // 60 ms at 250 Hz
};

struct SurfaceContext {
    // Direction-dependent first-order response, estimated from
    // dv/dt = response*(v_cmd-v) - drag*v.
    std::array<double, 3> response_positive_per_s{{8.0, 6.0, 8.0}};
    std::array<double, 3> response_negative_per_s{{8.0, 6.0, 8.0}};
    std::array<double, 3> braking_response_per_s{{8.0, 6.0, 8.0}};
    std::array<double, 3> rolling_drag_per_s{{0.15, 0.15, 0.15}};
    std::array<double, 3> confidence_axis{};
    double slip_longitudinal_mps = 0.0;
    double slip_lateral_mps = 0.0;
    double slip_yaw_radps = 0.0;
    double battery_response_scale = 1.0;
    double actuation_delay_s = 0.0;
    double confidence = 0.0;
    double coverage = 0.0;
    uint64_t accepted_samples = 0;
    uint64_t rejected_samples = 0;
    double updated_mono_s = 0.0;
};

class SurfaceEstimator {
public:
    explicit SurfaceEstimator(const SurfaceEstimatorConfig& cfg = {});

    void reset();
    void update(double now_s, double dt, const phx::Twist& stable_body,
                const EstimatorOutput& est, const RuntimeFeedback& feedback,
                bool learn_allowed);
    phx::Twist correction(double dt, const phx::Twist& stable_body,
                          const EstimatorOutput& est, bool braking);
    SurfaceContext context(double now_s) const;

    // Strict profile persistence. Geometry never appears in this profile.
    bool save_profile(const std::string& path, int robot_id,
                      const std::string& surface_id) const;
    bool load_profile(const std::string& path, int expected_robot_id,
                      const std::string& expected_surface_id);

private:
    struct Rls2 {
        double response = 8.0;
        double drag = 0.15;
        double p00 = 50.0, p01 = 0.0, p11 = 50.0;
        double residual_scale = 1.0;
        double excitation = 0.0;
        double last_update_s = -1e9;
        uint64_t accepted = 0;
        uint64_t rejected = 0;
    };

    static int bucket_index(int axis, bool braking, bool positive);
    double bucket_confidence(const Rls2& bucket, double now_s) const;
    void decay(double dt);
    void update_delay_scores(const phx::Twist& measured_accel,
                             const phx::Twist& measured_body);
    phx::Twist delayed_command() const;

    SurfaceEstimatorConfig cfg_;
    std::array<Rls2, 12> buckets_{};
    std::array<phx::Twist, 32> command_history_{};
    std::array<double, 16> delay_score_{};
    int history_head_ = 0;
    int history_count_ = 0;
    int delay_samples_ = 0;
    uint64_t delay_updates_ = 0;
    phx::Twist previous_measured_body_{};
    bool have_previous_ = false;
    phx::Twist signed_slip_{};
    double bus_voltage_v_ = 0.0;
    phx::Twist previous_correction_{};
    double last_now_s_ = 0.0;
    double last_dt_s_ = 0.004;
};

struct ResidualPolicyConfig {
    RlMode mode = RlMode::Off;
    std::string policy_path;
    double residual_limit_fraction = 0.05;
    double residual_slew_linear_mps2 = 0.5;
    double residual_slew_angular_radps2 = 1.5;
    double confidence_threshold = 0.65;
    int max_consecutive_interventions = 5;
    double max_loop_elapsed_s = 0.006;
};

class ResidualPolicy {
public:
    static constexpr int kObservationDim = 32;
    static constexpr int kActionDim = 3;
    static constexpr int kMaxHidden = 32;
    static constexpr const char* kControllerAbi = "turtlerabbit-motion-v1";

    bool load(const std::string& path, double configured_limit_fraction);
    std::array<double, kActionDim> infer(
        const std::array<double, kObservationDim>& observation) const;
    void clear();

    bool healthy() const { return healthy_; }
    const std::string& version() const { return version_; }
    const std::string& error() const { return error_; }
    double declared_limit_fraction() const { return declared_limit_fraction_; }

private:
    bool healthy_ = false;
    int hidden_dim_ = 0;
    double declared_limit_fraction_ = 0.0;
    std::string version_ = "none";
    std::string error_;
    std::array<double, kObservationDim> obs_mean_{};
    std::array<double, kObservationDim> obs_scale_{};
    std::array<double, kMaxHidden * kObservationDim> w1_{};
    std::array<double, kMaxHidden> b1_{};
    std::array<double, kActionDim * kMaxHidden> w2_{};
    std::array<double, kActionDim> b2_{};
};

struct RuntimeConstraintConfig {
    double velocity_max_mps = 4.0;
    double angular_velocity_max_radps = 6.0;
    double wheel_max_rev_s = 45.0;
    double correction_limit_fraction = 0.15;
    double correction_jerk_linear_mps3 = 10.0;
    double correction_jerk_angular_radps3 = 30.0;
    double min_bus_voltage_v = 19.0;
    double max_temperature_c = 70.0;
    double max_current_a = 10.0;
};

struct AugmentationConfig {
    SurfaceEstimatorConfig estimator;
    ResidualPolicyConfig rl;
    RuntimeConstraintConfig safety;
};

enum Intervention : uint32_t {
    kInterventionNone = 0,
    kInterventionNonFinite = 1u << 0,
    kInterventionConfidence = 1u << 1,
    kInterventionResidualBound = 1u << 2,
    kInterventionResidualSlew = 1u << 3,
    kInterventionVelocity = 1u << 4,
    kInterventionWheelSpeed = 1u << 5,
    kInterventionTelemetry = 1u << 6,
    kInterventionDeadline = 1u << 7,
    kInterventionPolicyHealth = 1u << 8,
    kInterventionAutoDisable = 1u << 9,
    kInterventionCorrectionJerk = 1u << 10,
};

struct AugmentationOutput {
    phx::Twist body{};
    phx::Twist adaptive_delta{};
    phx::Twist rl_proposed{};
    phx::Twist rl_applied{};
    SurfaceContext surface{};
    uint32_t interventions = 0;
    bool adaptive_applied = false;
    bool policy_evaluated = false;
    bool rl_healthy = false;
    bool rl_auto_disabled = false;
};

class MotionAugmentor {
public:
    MotionAugmentor(const AugmentationConfig& cfg = {},
                    const Kinematics& kin = {});

    AugmentationOutput step(double now_s, double dt, const MotionSetpoint& sp,
                            const TrajSample& ref, const EstimatorOutput& est,
                            const StableControl& stable,
                            const RuntimeFeedback& feedback);
    void reset(const phx::Twist& measured_body = {});

    const SurfaceEstimator& estimator() const { return estimator_; }
    SurfaceEstimator& estimator() { return estimator_; }
    const ResidualPolicy& policy() const { return policy_; }
    RlMode rl_mode() const { return cfg_.rl.mode; }
    bool reload_policy();
    void disable_rl();
    // Runtime pushes from the server (MotionParams). A mode change clears
    // the auto-disable latch and the intervention streak; the residual
    // limit can only be LOWERED below the configured/loaded authority, never
    // raised. Both clamp to sane ranges.
    void set_rl_mode(RlMode mode);
    void set_rl_limits(std::optional<double> residual_limit_fraction,
                       std::optional<double> confidence_threshold);
    const ResidualPolicyConfig& rl_config() const { return cfg_.rl; }

private:
    std::array<double, ResidualPolicy::kObservationDim> observation(
        const MotionSetpoint& sp, const TrajSample& ref,
        const EstimatorOutput& est, const StableControl& stable,
        const RuntimeFeedback& feedback, const SurfaceContext& surface) const;
    bool telemetry_healthy(const RuntimeFeedback& feedback) const;

    AugmentationConfig cfg_;
    Kinematics kin_;
    SurfaceEstimator estimator_;
    ResidualPolicy policy_;
    phx::Twist previous_residual_{};
    phx::Twist previous_total_correction_{};
    phx::Twist previous_correction_rate_{};
    int consecutive_interventions_ = 0;
    bool rl_auto_disabled_ = false;
};

}  // namespace rf
