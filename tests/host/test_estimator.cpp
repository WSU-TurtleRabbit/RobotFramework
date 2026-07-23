// Delayed-vision estimator tests: the fusion must track ground truth from a
// LATE, noisy vision stream + gyro/odometry — that is the whole point of the
// TIGERs time-slot architecture. Scenarios simulate truth, deliver vision at
// 60 Hz with a measured delay, and assert the present-time output.
#include <cmath>
#include <cstdint>

#include "Motion/estimator.h"
#include "Motion/phx/testing.h"

using namespace rf;

namespace {

// Deterministic noise source (no <random> seeding roulette in tests).
struct Lcg {
    uint32_t s = 0x12345678u;
    double uniform() {  // [0, 1)
        s = s * 1664525u + 1013904223u;
        return static_cast<double>(s >> 8) / 16777216.0;
    }
    double gauss() {  // approx N(0,1), sum of 4 uniforms
        return (uniform() + uniform() + uniform() + uniform() - 2.0) * 1.732;
    }
};

constexpr double kDt = 0.004;          // 250 Hz control tick
constexpr double kVisPeriod = 1.0 / 60.0;

// Truth with a constant global velocity (and optional yaw rate); the
// estimator is fed slip-scaled odometry and delayed noisy vision.
struct World {
    double x = 0, y = 0, th = 0;
    double vx = 0, vy = 0, w = 0;
    double slip = 1.0;      // odometry = slip * truth velocity
    double vis_delay = 0.040;
    double vis_noise = 0.0;  // metres std-dev
    double vis_period = kVisPeriod;
    Lcg rng;

    FusionEstimator est;
    double t = 0.0;
    double next_vis = 0.0;

    static EstimatorConfig cfg() {
        EstimatorConfig c;
        c.capture_delay_s = 0.0;  // tests own the delay explicitly
        return c;
    }
    World() : est(cfg()) {}

    // Run `seconds` of simulation, delivering vision at 60 Hz.
    void run(double seconds) {
        const double end = t + seconds;
        while (t < end - 1e-12) {
            step();
        }
    }

    void step() {
        // Advance truth.
        x += vx * kDt;
        y += vy * kDt;
        th += w * kDt;
        t += kDt;
        // Feed sensors (body frame == global here only when th == 0; the
        // heading tests rotate, so build the honest body-frame odometry).
        const double c = std::cos(th), s = std::sin(th);
        const phx::Twist odo{(vx * c + vy * s) * slip, (-vx * s + vy * c) * slip,
                             w * slip};
        est.tick(t, kDt, odo, w);
        // Deliver a vision frame when due: the pose as it was at
        // (t - vis_delay), plus noise.
        if (t + 1e-12 >= next_vis) {
            next_vis += vis_period;
            VisionPose p;
            p.x = x - vx * vis_delay + vis_noise * rng.gauss();
            p.y = y - vy * vis_delay + vis_noise * rng.gauss();
            p.heading = th - w * vis_delay + vis_noise * 0.2 * rng.gauss();
            est.on_vision(p, vis_delay);
        }
    }

    double err_x() const { return est.output().pose.pos.x - x; }
    double err_y() const { return est.output().pose.pos.y - y; }
    double err_pos() const { return std::hypot(err_x(), err_y()); }
};

}  // namespace

PHX_TEST(estimator_first_fix_initializes_and_enables) {
    FusionEstimator est;
    est.tick(0.0, kDt, phx::Twist{}, 0.0);
    CHECK(!est.has_fix());
    CHECK(!est.output().vision_alive);

    VisionPose p{0.3, -0.2, 0.7};
    est.on_vision(p, 0.040);
    CHECK(est.has_fix());
    CHECK(est.output().vision_alive);
    // First fix hard-initializes to the vision pose (no Kalman blend).
    CHECK_NEAR(est.output().pose.pos.x, 0.3, 1e-9);
    CHECK_NEAR(est.output().pose.pos.y, -0.2, 1e-9);
    CHECK_NEAR(est.output().pose.heading, 0.7, 1e-9);
    // The measurement-matched slot is reported for MatchFeedback.
    REQUIRE(est.last_meas_state().has_value());
    CHECK_NEAR(est.last_meas_state()->pose.pos.x, 0.3, 1e-9);
}

PHX_TEST(estimator_converges_and_holds_stationary) {
    World w;
    w.x = 1.0;
    w.y = 0.5;
    w.th = 0.3;
    w.vis_noise = 0.005;  // 5 mm vision noise
    w.run(2.0);
    CHECK(w.err_pos() < 0.004);
    CHECK(std::fabs(w.est.output().pose.heading - 0.3) < 0.01);
    CHECK(w.est.output().vel_global.norm() < 0.02);
}

PHX_TEST(estimator_delayed_vision_tracks_without_lag) {
    // THE delayed-insertion test: a naive complementary filter comparing a
    // 40 ms-old measurement against the PRESENT state builds a steady-state
    // offset of ~v * delay / gain (here ~30+ mm at 0.85 m/s). Inserting the
    // measurement into the past removes it — assert single-digit mm error.
    World w;
    w.vx = 0.8;
    w.vy = 0.3;
    w.vis_delay = 0.040;
    w.vis_noise = 0.003;
    w.run(4.0);
    CHECK(w.err_pos() < 0.012);
    CHECK(std::fabs(w.est.output().vel_global.x - 0.8) < 0.03);
    CHECK(std::fabs(w.est.output().vel_global.y - 0.3) < 0.03);
}

PHX_TEST(estimator_handles_long_delay_and_slow_vision) {
    // 80 ms delay at 30 Hz (worst realistic link): still no lag offset.
    World w;
    w.vx = 0.6;
    w.vy = -0.4;
    w.vis_delay = 0.080;
    w.vis_noise = 0.005;
    w.vis_period = 1.0 / 30.0;
    w.run(5.0);
    CHECK(w.err_pos() < 0.020);
}

PHX_TEST(estimator_gates_outliers_then_snaps_when_lost) {
    World w;
    w.x = 1.0;
    w.y = 0.5;
    w.vis_noise = 0.002;
    w.run(1.0);  // converge
    CHECK(w.err_pos() < 0.01);

    // A 2 m teleport is not motion: gated out, estimate untouched.
    const double before = w.est.output().pose.pos.x;
    VisionPose bad{3.0, 0.5, 0.0};
    w.est.on_vision(bad, 0.040);
    CHECK(w.est.output().rejected == 1);
    CHECK_NEAR(w.est.output().pose.pos.x, before, 1e-9);

    // Four more rejections -> the sixth fix means WE were lost: snap to it.
    for (int i = 0; i < 4; ++i) w.est.on_vision(bad, 0.040);
    CHECK(w.est.output().rejected == 5);
    w.est.on_vision(bad, 0.040);
    CHECK(std::fabs(w.est.output().pose.pos.x - 3.0) < 0.005);
}

PHX_TEST(estimator_dead_reckons_on_timeout_and_snaps_back) {
    World w;
    w.vx = 0.5;
    w.vis_delay = 0.030;
    w.vis_noise = 0.002;
    w.run(1.0);  // converge with vision
    CHECK(w.est.output().vision_alive);

    // Vision goes dark for 1.5 s with 5% wheel slip: the estimator must
    // keep integrating (dead reckoning) and report vision dead.
    w.slip = 0.95;
    w.vis_delay = 1e9;  // no more frames (next_vis stays in the past... see below)
    // Drive blind by hand: stop delivering vision.
    const double blind_end = w.t + 1.5;
    while (w.t < blind_end) {
        w.x += w.vx * kDt;
        w.t += kDt;
        w.est.tick(w.t, kDt, phx::Twist{w.vx * w.slip, 0.0, 0.0}, 0.0);
    }
    CHECK(!w.est.output().vision_alive);
    // Slip accumulated a real drift (> 2 cm) — dead reckoning did its job.
    CHECK(std::fabs(w.err_x()) > 0.02);

    // Vision returns: first frame snaps back to truth.
    w.vis_delay = 0.030;
    VisionPose p{w.x - w.vx * 0.030, 0.0, 0.0};
    w.est.on_vision(p, 0.030);
    CHECK(w.est.output().vision_alive);
    CHECK(std::fabs(w.err_x()) < 0.02);
}

PHX_TEST(estimator_position_holds_under_wheel_slip) {
    // 10% slip with live vision: position stays honest (the Kalman position
    // correction at the delayed slot pays slip back every fix).
    World w;
    w.vx = 0.5;
    w.vy = 0.2;
    w.slip = 0.90;
    w.vis_delay = 0.040;
    w.vis_noise = 0.003;
    w.run(3.0);
    CHECK(w.err_pos() < 0.02);
}

PHX_TEST(estimator_heading_fuses_gyro_and_vision) {
    World w;
    w.w = 1.0;  // rad/s CCW
    w.vis_delay = 0.040;
    w.vis_noise = 0.005;
    w.run(2.0);
    const double herr =
        std::remainder(w.est.output().pose.heading - w.th, phx::kTwoPi);
    CHECK(std::fabs(herr) < 0.02);
    CHECK_NEAR(w.est.output().omega, 1.0, 1e-9);
}

PHX_TEST(estimator_heading_gate_rejects_marker_flip_but_keeps_position) {
    EstimatorConfig cfg;
    cfg.capture_delay_s = 0.0;
    cfg.theta_gain = 0.5;
    cfg.theta_gate_rad = 0.45;
    FusionEstimator est{cfg};
    est.tick(0.0, kDt, phx::Twist{}, 0.0);
    est.on_vision(VisionPose{0.0, 0.0, 0.0}, 0.0);
    est.tick(kDt, kDt, phx::Twist{}, 0.0);

    // A blurred marker may keep a valid centre while its decoded orientation
    // flips. Position still fuses; heading must stay on the gyro estimate.
    est.on_vision(VisionPose{0.05, 0.0, 1.2}, 0.0);
    CHECK(est.output().pose.pos.x > 0.0);
    CHECK(std::fabs(est.output().pose.heading) < 1e-9);

    // A small, physically plausible yaw innovation still corrects drift.
    est.on_vision(VisionPose{0.05, 0.0, 0.10}, 0.0);
    CHECK(est.output().pose.heading > 0.0);
    CHECK(est.output().pose.heading < 0.10);

    // The field verifier observed legitimate reverse-leg disagreement at
    // 0.30 rad. It must remain correctable rather than latching the estimator
    // into a rotated frame.
    const double before = est.output().pose.heading;
    est.on_vision(VisionPose{0.05, 0.0, 0.35}, 0.0);
    CHECK(est.output().pose.heading > before);
}

PHX_TEST(estimator_gyro_loss_falls_back_to_odometry_yaw) {
    FusionEstimator est;
    est.tick(0.0, kDt, phx::Twist{}, 0.0);
    VisionPose p{0.0, 0.0, 0.0};
    est.on_vision(p, 0.0);
    // NaN gyro: the estimator must use odometry yaw instead.
    for (int i = 1; i <= 100; ++i) {
        est.tick(i * kDt, kDt, phx::Twist{0.0, 0.0, 0.5},
                 std::numeric_limits<double>::quiet_NaN());
    }
    CHECK(std::fabs(est.output().pose.heading - 0.5 * 100 * kDt) < 0.02);
}

PHX_TEST(estimator_kalman_gains_are_sane) {
    FusionEstimator est;
    CHECK(est.k_pos() > 0.05 && est.k_pos() < 0.95);
    CHECK(est.k_vel() > 0.01 && est.k_vel() < 50.0);
}

PHX_TEST(estimator_gain_uses_vision_period_not_control_tick) {
    EstimatorConfig slow_camera;
    slow_camera.measurement_dt_s = 0.043;
    EstimatorConfig wrong_control_period;
    wrong_control_period.measurement_dt_s = kDt;
    FusionEstimator field_est{slow_camera};
    FusionEstimator underweighted{wrong_control_period};
    CHECK(field_est.k_pos() > 0.60);
    CHECK(field_est.k_pos() > 4.0 * underweighted.k_pos());
}

PHX_TEST(estimator_field_gain_overrides_are_independent) {
    EstimatorConfig cfg;
    cfg.vision_pos_gain = 0.65;
    cfg.vision_vel_gain = 1.50;
    FusionEstimator est{cfg};
    CHECK_NEAR(est.k_pos(), 0.65, 1e-12);
    CHECK_NEAR(est.k_vel(), 1.50, 1e-12);
}
