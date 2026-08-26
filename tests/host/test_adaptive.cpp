#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "Motion/adaptive.h"
#include "Motion/phx/testing.h"

using namespace rf;

namespace {

std::string actor_json(double output_bias = 0.5493061443340548,
                       double residual_limit = 0.05,
                       const std::string& abi = ResidualPolicy::kControllerAbi) {
    std::ostringstream out;
    auto zeros = [&](int count, double value = 0.0) {
        out << '[';
        for (int i = 0; i < count; ++i) {
            if (i) out << ',';
            out << value;
        }
        out << ']';
    };
    out << "{\"schema\":1,\"controller_abi\":\"" << abi
        << "\",\"policy_version\":\"test-actor\""
        << ",\"observation_dim\":32,\"action_dim\":3,\"hidden_dim\":1"
        << ",\"residual_limit_fraction\":" << residual_limit
        << ",\"obs_mean\":"; zeros(32);
    out << ",\"obs_scale\":"; zeros(32, 1.0);
    out << ",\"w1\":"; zeros(32);
    out << ",\"b1\":"; zeros(1);
    out << ",\"w2\":"; zeros(3);
    out << ",\"b2\":[" << output_bias << ",0,0]}";
    return out.str();
}

std::filesystem::path temp_path(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

EstimatorOutput estimate(double vx, double vy = 0.0, double w = 0.0) {
    EstimatorOutput est;
    est.has_fix = true;
    est.vision_alive = true;
    est.vision_age_s = 0.0;
    est.vel_body = phx::Twist{vx, vy, w};
    est.vel_global = {vx, vy};
    est.omega = w;
    return est;
}

}  // namespace

PHX_TEST(surface_rls_converges_on_known_longitudinal_response) {
    SurfaceEstimatorConfig cfg;
    cfg.enabled = true;
    cfg.forgetting_factor = 0.999;
    cfg.confidence_threshold = 0.25;
    cfg.max_delay_samples = 0;
    SurfaceEstimator estimator(cfg);
    RuntimeFeedback feedback;
    double velocity = 0.0;
    constexpr double dt = 0.004;
    constexpr double response = 4.5;
    constexpr double drag = 0.35;
    for (int tick = 0; tick < 2500; ++tick) {
        const double command = ((tick / 250) % 2 == 0) ? 1.2 : -1.0;
        velocity += (response * (command - velocity) - drag * velocity) * dt;
        feedback.wheel_odo_body = phx::Twist{velocity, 0.0, 0.0};
        estimator.update(tick * dt, dt, phx::Twist{command, 0.0, 0.0},
                         estimate(velocity), feedback, true);
    }
    const SurfaceContext context = estimator.context(2500 * dt);
    CHECK(context.accepted_samples > 500);
    CHECK(context.confidence_axis[0] > 0.25);
    CHECK(std::fabs(context.response_positive_per_s[0] - response) < 1.5);
    CHECK(std::fabs(context.response_negative_per_s[0] - response) < 1.5);
    CHECK(context.response_positive_per_s[0] >= cfg.min_response_per_s[0]);
    CHECK(context.response_positive_per_s[0] <= cfg.max_response_per_s[0]);
}

PHX_TEST(surface_estimator_rejects_collision_and_stale_vision) {
    SurfaceEstimatorConfig cfg;
    cfg.enabled = true;
    cfg.max_delay_samples = 0;
    SurfaceEstimator estimator(cfg);
    RuntimeFeedback feedback;
    estimator.update(0.0, 0.004, phx::Twist{1.0, 0.0, 0.0},
                     estimate(0.0), feedback, true);
    EstimatorOutput jumped = estimate(10.0);
    jumped.vision_age_s = 0.5;
    estimator.update(0.004, 0.004, phx::Twist{1.0, 0.0, 0.0},
                     jumped, feedback, true);
    const SurfaceContext context = estimator.context(0.004);
    CHECK(context.accepted_samples == 0);
    CHECK_NEAR(context.response_positive_per_s[0],
               cfg.default_response_per_s[0], 1e-12);
}

PHX_TEST(surface_profiles_are_identity_checked_and_bounded) {
    SurfaceEstimatorConfig cfg;
    cfg.enabled = true;
    SurfaceEstimator source(cfg);
    const auto path = temp_path("rf_surface_profile_test.json");
    REQUIRE(source.save_profile(path.string(), 5, "test-carpet"));
    SurfaceEstimator loaded(cfg);
    CHECK(loaded.load_profile(path.string(), 5, "test-carpet"));
    CHECK(!loaded.load_profile(path.string(), 6, "test-carpet"));
    CHECK(!loaded.load_profile(path.string(), 5, "other-carpet"));
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

PHX_TEST(residual_policy_checks_abi_dimensions_and_bound) {
    const auto path = temp_path("rf_residual_actor_test.json");
    {
        std::ofstream out(path);
        out << actor_json();
    }
    ResidualPolicy policy;
    REQUIRE(policy.load(path.string(), 0.05));
    CHECK(policy.healthy());
    CHECK(policy.version() == "test-actor");
    const auto action = policy.infer({});
    CHECK_NEAR(action[0], 0.5, 1e-6);
    CHECK_NEAR(action[1], 0.0, 1e-12);

    {
        std::ofstream out(path);
        out << actor_json(0.0, 0.10);
    }
    CHECK(!policy.load(path.string(), 0.05));
    {
        std::ofstream out(path);
        out << actor_json(0.0, 0.05, "wrong-controller");
    }
    CHECK(!policy.load(path.string(), 0.05));
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

PHX_TEST(shadow_policy_is_computed_but_never_applied) {
    const auto path = temp_path("rf_shadow_actor_test.json");
    {
        std::ofstream out(path);
        out << actor_json();
    }
    AugmentationConfig cfg;
    cfg.rl.mode = RlMode::Shadow;
    cfg.rl.policy_path = path.string();
    cfg.rl.residual_limit_fraction = 0.05;
    MotionAugmentor augmentor(cfg, Kinematics{});
    RuntimeFeedback feedback;
    feedback.bus_voltage_v = 23.0;
    StableControl stable;
    stable.body = phx::Twist{1.0, 0.0, 0.0};
    stable.energize = true;
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::LocalVel;
    const AugmentationOutput out = augmentor.step(
        1.0, 0.004, sp, TrajSample{}, estimate(0.5), stable, feedback);
    CHECK(out.policy_evaluated);
    CHECK(out.rl_proposed.lin.x > 0.0);
    CHECK_NEAR(out.rl_applied.lin.x, 0.0, 1e-12);
    CHECK_NEAR(out.body.lin.x, stable.body.lin.x, 1e-12);
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

PHX_TEST(bounded_policy_falls_back_on_low_confidence_and_deadline) {
    const auto path = temp_path("rf_bounded_actor_test.json");
    {
        std::ofstream out(path);
        out << actor_json();
    }
    AugmentationConfig cfg;
    cfg.rl.mode = RlMode::Bounded;
    cfg.rl.policy_path = path.string();
    MotionAugmentor augmentor(cfg, Kinematics{});
    RuntimeFeedback feedback;
    feedback.bus_voltage_v = 23.0;
    feedback.control_deadline_missed = true;
    StableControl stable;
    stable.body = phx::Twist{1.0, 0.0, 0.0};
    stable.energize = true;
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::LocalVel;
    const AugmentationOutput out = augmentor.step(
        1.0, 0.004, sp, TrajSample{}, estimate(0.0), stable, feedback);
    CHECK(!out.rl_healthy);
    CHECK_NEAR(out.rl_applied.lin.norm(), 0.0, 1e-12);
    CHECK_NEAR(out.body.lin.x, stable.body.lin.x, 1e-12);
    CHECK((out.interventions & kInterventionDeadline) != 0);
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

PHX_TEST(policy_health_requires_all_four_fresh_motor_replies) {
    const auto path = temp_path("rf_telemetry_actor_test.json");
    {
        std::ofstream out(path);
        out << actor_json();
    }
    AugmentationConfig cfg;
    cfg.rl.mode = RlMode::Shadow;
    cfg.rl.policy_path = path.string();
    MotionAugmentor augmentor(cfg, Kinematics{});
    RuntimeFeedback feedback;
    feedback.bus_voltage_v = 23.0;
    feedback.wheel_temperature_c.fill(30.0);
    feedback.wheel_replied = {true, true, true, false};
    StableControl stable;
    stable.body = phx::Twist{1.0, 0.0, 0.0};
    stable.energize = true;
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::LocalVel;
    AugmentationOutput out = augmentor.step(
        1.0, 0.004, sp, TrajSample{}, estimate(0.5), stable, feedback);
    CHECK((out.interventions & kInterventionTelemetry) != 0);
    feedback.wheel_replied.fill(true);
    out = augmentor.step(
        1.004, 0.004, sp, TrajSample{}, estimate(0.5), stable, feedback);
    CHECK((out.interventions & kInterventionTelemetry) == 0);
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

PHX_TEST(adaptive_and_shadow_path_has_large_host_timing_margin) {
    const auto path = temp_path("rf_timing_actor_test.json");
    {
        std::ofstream out(path);
        out << actor_json();
    }
    AugmentationConfig cfg;
    cfg.estimator.enabled = true;
    cfg.rl.mode = RlMode::Shadow;
    cfg.rl.policy_path = path.string();
    MotionAugmentor augmentor(cfg, Kinematics{});
    RuntimeFeedback feedback;
    feedback.bus_voltage_v = 23.0;
    feedback.wheel_temperature_c.fill(30.0);
    feedback.wheel_replied.fill(true);
    StableControl stable;
    stable.body = phx::Twist{1.0, 0.3, 0.2};
    stable.energize = true;
    MotionSetpoint sp;
    sp.kind = MotionSetpoint::Kind::LocalVel;
    EstimatorOutput est = estimate(0.7, 0.2, 0.1);
    const auto started = std::chrono::steady_clock::now();
    constexpr int iterations = 20000;
    for (int i = 0; i < iterations; ++i) {
        feedback.wheel_odo_body = est.vel_body;
        const AugmentationOutput out = augmentor.step(
            2.0 + i * 0.004, 0.004, sp, TrajSample{}, est, stable, feedback);
        CHECK(std::isfinite(out.body.lin.x));
    }
    const double elapsed_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - started).count();
    // This loose regression ceiling is 8x below the 4 ms loop budget even on
    // an unoptimized host build; field timing is logged independently.
    CHECK(elapsed_us / iterations < 500.0);
    std::error_code ec;
    std::filesystem::remove(path, ec);
}
