// Completion-plan 1.18: analytic contracts plus an explicit bit-table probe.
// --dump-nonfinite prints native/libm/subtraction bits for comparison with
// NumPy's scalar AND contiguous-array paths on the same platform. No provider
// bit table is fabricated here: freeze measured expectations after that check.
#include "../src/numeric_sincos.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>

namespace {
    constexpr std::array<std::uint64_t, 4> inputs{
        UINT64_C(0x7ff0000000000000), UINT64_C(0xfff0000000000000),
        UINT64_C(0x7ff8000000000000), UINT64_C(0xfff8000000000000),
    };

    void require(bool condition, const char *message) {
        if (!condition) throw std::runtime_error(message);
    }

    void check_analytic_contracts() {
        require(std::bit_cast<std::uint64_t>(numeric::np_sin(0.)) == 0,
                "sin(+0) is +0");
        require(std::bit_cast<std::uint64_t>(numeric::np_sin(-0.)) ==
                    UINT64_C(0x8000000000000000), "sin(-0) is -0");
        require(numeric::np_cos(0.) == 1. && numeric::np_cos(-0.) == 1.,
                "cos(signed zero) is one");
        for (const auto bits : inputs) {
            const double value = std::bit_cast<double>(bits);
            require(std::isnan(numeric::np_sin(value)) &&
                        std::isnan(numeric::np_cos(value)),
                    "nonfinite trigonometric inputs return NaN");
        }
    }

    void dump_nonfinite() {
        std::cout << "input native_sin native_cos libm_sin libm_cos subtract\n";
        for (const auto bits : inputs) {
            const volatile double value = std::bit_cast<double>(bits);
            // Checker: clang-tidy misc-redundant-expression. Origin: this
            // Phase 1.18 subtraction candidate; no diagnostic run yet.
            // Reason: x-x must remain an operation to measure NaN sign/payload.
            // Reproducer: tidy this TU using its binding compile_commands entry.
            // Remove when the candidate probe is retired or the checker accepts
            // this intentional repeated-operand measurement without suppression.
            const std::array<double, 5> values{
                numeric::np_sin(value), numeric::np_cos(value),
                std::sin(value), std::cos(value),
                value - value, // NOLINT(misc-redundant-expression) preserve the candidate operation
            };
            std::cout << std::hex << std::setfill('0') << std::setw(16) << bits;
            for (const auto result : values)
                std::cout << ' ' << std::setw(16) << std::bit_cast<std::uint64_t>(result);
            std::cout << '\n';
        }
    }
}

int main(int argc, char **argv) {
    // Checker: Clang -Wunsafe-buffer-usage-in-container. Origin: Phase 1.18
    // argv span, using the existing native test/CLI pattern; no diagnostic run yet.
    // Reason: the main ABI supplies the argc bound; every access uses this span.
    // Reproducer: compile this TU with that warning and without the local pragma.
    // Remove when Clang accepts this bounded construction or a shared argv view
    // makes the raw-pointer/count construction unnecessary.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--dump-nonfinite") {
        dump_nonfinite();
        return 0;
    }
    if (arguments.size() != 1) {
        std::cerr << "usage: native_platform_sincos_tests [--dump-nonfinite]\n";
        return 2;
    }
    try {
        check_analytic_contracts();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
