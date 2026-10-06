#include "../src/drivetrain/pedaling.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition) throw std::runtime_error(std::string(message));
    }

    void test_human_crank_torque() {
        require(drivetrain::human_crank_torque(50., 0., .35) == 32.5,
                "human_crank_torque at phase zero");
        require(std::abs(drivetrain::human_crank_torque(50., std::numbers::pi / 2., .35) - 67.5) < 1e-12,
                "human_crank_torque at half turn");
    }

    void test_pedaling_policy_valid_transition() {
        const drivetrain::PedalingConfig config{
            .enabled = true,
            .coast_above_rpm = 110.,
            .resume_below_rpm = 80.,
            .stop_time_s = 1.,
            .coast_cadence_tau_s = 0.,
            .mash_cadence_rpm = 25.,
            .mash_torque_nm = 25.,
            .effort_slew_nm_s = 0.,
        };
        drivetrain::PedalingPolicy policy(config);
        const drivetrain::PedalingState state = policy.update(0., 0., 30., 20., .01);
        require(state.mode == "pedaling", "pedaling policy mode");
        require(state.effort_nm == 20., "pedaling policy effort");
        require(!state.target_phase_rad, "pedaling policy target phase");
    }

    constexpr std::array<TestCase, 2> cases{{
        {.name = "human_crank_torque", .run = test_human_crank_torque},
        {.name = "pedaling_policy_valid_transition", .run = test_pedaling_policy_valid_transition},
    }};

    int run_case(const TestCase &test_case) {
        try {
            test_case.run();
            std::cout << "PASS " << test_case.name << '\n';
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
            return 1;
        }
    }
} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the sentinel.
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--require-failure") {
        try {
            require(false, "deliberate require failure");
        } catch (const std::exception &error) {
            std::cerr << error.what() << '\n';
            return 1;
        }
        return 0;
    }
    if (arguments.size() > 2) {
        std::cerr << "usage: native_contract_tests [case-name|--require-failure]\n";
        return 2;
    }

    const std::string_view requested =
        arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all";
    int status = 0;
    for (const TestCase &test_case : cases) {
        if (requested == "all" || requested == test_case.name) {
            found = true;
            if (run_case(test_case) != 0) status = 1;
        }
    }
    if (!found) {
        std::cerr << "unknown native contract case: " << requested << '\n';
        return 2;
    }
    return status;
}
