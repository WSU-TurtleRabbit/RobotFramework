#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <thread>

#include "Motion/phx/testing.h"
#include "Telemetry/motion_logger.h"

using namespace rf;

namespace {

namespace fs = std::filesystem;

fs::path fresh_temp_dir(const std::string& tag) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path dir = fs::temp_directory_path() /
                   ("rf_motion_logger_" + tag + "_" + std::to_string(stamp));
    fs::remove_all(dir);
    fs::create_directories(dir);
    return dir;
}

int count_motion_logs(const fs::path& dir) {
    int count = 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("motion_v1_", 0) == 0) ++count;
    }
    return count;
}

bool wait_for(const std::function<bool()>& done, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

}  // namespace

PHX_TEST(motion_telemetry_header_defines_schema_units_and_frames) {
    const std::string text = encode_motion_telemetry_header(
        5, ResidualPolicy::kControllerAbi);
    CHECK(text.find("\"schema_version\":1") != std::string::npos);
    CHECK(text.find("steady_monotonic_seconds") != std::string::npos);
    CHECK(text.find("robot +X forward,+Y left,CCW") != std::string::npos);
    CHECK(text.find("CAN [FR,RR,RL,FL]") != std::string::npos);
    CHECK(text.find("A_q") != std::string::npos);
}

PHX_TEST(motion_telemetry_sample_is_keyed_and_nonfinite_is_null) {
    MotionTelemetrySample sample;
    sample.robot_id = 5;
    sample.sequence = 42;
    sample.rl_mode = RlMode::Shadow;
    sample.policy_version = "actor-test";
    sample.vision_age_s = std::numeric_limits<double>::infinity();
    sample.motor_replied[0] = true;
    sample.imu_available = true;
    sample.imu_accel_body_mps2 = {1.25, -0.5};
    const std::string text = encode_motion_telemetry_sample(sample);
    CHECK(text.find("\"seq\":42") != std::string::npos);
    CHECK(text.find("\"rl_mode\":\"shadow\"") != std::string::npos);
    CHECK(text.find("\"policy_version\":\"actor-test\"") != std::string::npos);
    CHECK(text.find("\"age_s\":null") != std::string::npos);
    CHECK(text.find("\"motor_replied\":[true,false,false,false]") !=
          std::string::npos);
    CHECK(text.find("\"imu_accel_body_mps2\":[1.25,-0.5]") !=
          std::string::npos);
}

PHX_TEST(motion_logger_rotates_at_file_size_bound) {
    const fs::path dir = fresh_temp_dir("rotate");
    AsyncMotionLogger logger;
    MotionLoggerLimits limits;
    limits.max_file_bytes = 8 * 1024;   // a few samples per segment
    limits.max_total_bytes = 0;         // retention off for this test
    REQUIRE(logger.start(dir.string(), 5, 100.0,
                         ResidualPolicy::kControllerAbi, limits));
    MotionTelemetrySample sample;
    sample.robot_id = 5;
    for (int i = 0; i < 64; ++i) {
        sample.sequence = static_cast<uint64_t>(i);
        while (!logger.push(sample) && logger.healthy())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(wait_for([&] { return logger.rotations() >= 2; }, 5.0));
    logger.stop();
    CHECK(logger.dropped() == 0);
    CHECK(count_motion_logs(dir) >= 3);
    // Every segment must be self-describing: header line first.
    for (const auto& entry : fs::directory_iterator(dir)) {
        std::ifstream in(entry.path());
        std::string first;
        std::getline(in, first);
        CHECK(first.find("\"record_type\":\"schema\"") != std::string::npos);
    }
    fs::remove_all(dir);
}

PHX_TEST(motion_logger_retention_deletes_oldest_but_never_active_or_foreign) {
    const fs::path dir = fresh_temp_dir("retention");
    // A stale log from an earlier session, and an unrelated file.
    {
        std::ofstream old(dir / "motion_v1_robot5_19990101_000000.jsonl");
        old << std::string(32 * 1024, 'x');
        std::ofstream foreign(dir / "keepme.txt");
        foreign << std::string(32 * 1024, 'y');
    }
    AsyncMotionLogger logger;
    MotionLoggerLimits limits;
    limits.max_file_bytes = 8 * 1024;
    limits.max_total_bytes = 16 * 1024;
    REQUIRE(logger.start(dir.string(), 5, 100.0,
                         ResidualPolicy::kControllerAbi, limits));
    // Start-time retention: the stale 32 KiB log alone exceeds the total
    // bound, so it must already be gone.
    CHECK(wait_for([&] { return logger.retention_deleted() >= 1; }, 5.0));
    CHECK(!fs::exists(dir / "motion_v1_robot5_19990101_000000.jsonl"));
    MotionTelemetrySample sample;
    sample.robot_id = 5;
    for (int i = 0; i < 128; ++i) {
        sample.sequence = static_cast<uint64_t>(i);
        while (!logger.push(sample) && logger.healthy())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(wait_for([&] { return logger.rotations() >= 4; }, 5.0));
    logger.stop();
    // Read the final active path only after the worker has drained; earlier
    // reads can name a segment that a later rotation already retired.
    CHECK(fs::exists(logger.path()));
    CHECK(fs::exists(dir / "keepme.txt"));
    // Total of motion logs stays near the bound: at most the retention total
    // plus one full segment that was still being written.
    uint64_t total = 0;
    for (const auto& entry : fs::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("motion_v1_", 0) == 0) total += fs::file_size(entry);
    }
    CHECK(total <= limits.max_total_bytes + limits.max_file_bytes + 4096);
    fs::remove_all(dir);
}
