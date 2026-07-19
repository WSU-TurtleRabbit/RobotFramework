// Vendored from phoenix-core (github.com/rishieissocool/phoenix-core), same
// author; relicensed under this repository's GPLv3.
//
// Minimal zero-dependency test harness. We deliberately avoid vendoring a
// framework: the whole point is that `cmake && build` works on a bare
// machine, and our needs are CHECKs plus a runner with a name filter.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <vector>

namespace phx::testing {

struct TestCase {
    const char* name;
    void (*fn)();
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

struct RequireFailed : std::exception {};

struct State {
    int checks = 0;
    int failures = 0;
    const char* current = "";
};
inline State& state() {
    static State s;
    return s;
}

inline void report_failure(const char* file, int line, const char* what) {
    ++state().failures;
    std::printf("FAIL  %s\n      %s:%d  %s\n", state().current, file, line, what);
}

inline int run_all(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int ran = 0, failed_tests = 0;
    for (const auto& t : registry()) {
        if (filter && std::strstr(t.name, filter) == nullptr) continue;
        state().current = t.name;
        const int before = state().failures;
        try {
            t.fn();
        } catch (const RequireFailed&) {
            // failure already reported at the REQUIRE site
        } catch (const std::exception& e) {
            report_failure("<uncaught>", 0, e.what());
        }
        ++ran;
        if (state().failures != before) ++failed_tests;
    }
    std::printf("%s: %d test(s), %d check(s), %d failure(s)\n",
                state().failures == 0 ? "OK" : "FAILED", ran, state().checks, state().failures);
    if (ran == 0) {
        std::printf("no tests matched filter\n");
        return 2;
    }
    return state().failures == 0 ? 0 : 1;
}

}  // namespace phx::testing

#define PHX_TEST(name)                                                        \
    static void phx_test_##name();                                           \
    static ::phx::testing::Registrar phx_reg_##name(#name, &phx_test_##name); \
    static void phx_test_##name()

#define CHECK(expr)                                                     \
    do {                                                                \
        ++::phx::testing::state().checks;                               \
        if (!(expr)) ::phx::testing::report_failure(__FILE__, __LINE__, #expr); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                        \
    do {                                                                             \
        ++::phx::testing::state().checks;                                            \
        const double phx_a = (a), phx_b = (b), phx_tol = (tol);                      \
        if (!(std::fabs(phx_a - phx_b) <= phx_tol)) {                                \
            char phx_buf[256];                                                       \
            std::snprintf(phx_buf, sizeof(phx_buf), "%s == %s +/- %g  (%.9g vs %.9g)", \
                          #a, #b, phx_tol, phx_a, phx_b);                            \
            ::phx::testing::report_failure(__FILE__, __LINE__, phx_buf);             \
        }                                                                            \
    } while (0)

#define REQUIRE(expr)                                                       \
    do {                                                                    \
        ++::phx::testing::state().checks;                                   \
        if (!(expr)) {                                                      \
            ::phx::testing::report_failure(__FILE__, __LINE__, #expr);      \
            throw ::phx::testing::RequireFailed{};                          \
        }                                                                   \
    } while (0)
