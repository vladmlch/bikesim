// Completion-plan 1.18: measured bit contracts plus an explicit bit-table probe.
// --dump-nonfinite prints native/libm/subtraction bits for comparison with
// NumPy's scalar AND contiguous-array paths on the same platform.
#include "../src/numeric_sincos.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <span>
#include <stdexcept>
#include <string_view>

namespace {
    constexpr std::array<std::uint64_t, 4> inputs{
        UINT64_C(0x7ff0000000000000), UINT64_C(0xfff0000000000000),
        UINT64_C(0x7ff8000000000000), UINT64_C(0xfff8000000000000),
    };

    // Measured 2026-10-21 with NumPy 2.5.2; scalar np.float64 and contiguous
    // np.full(64, value, dtype=np.float64) agree in all lanes, under
    // np.errstate(invalid="ignore"). Rows follow inputs; columns are sin/cos.
    // Probe: uv run --frozen --no-sync --group native python
    // /private/tmp/bike-sincos-probe.py (macOS CPython 3.13.12).
    // Linux: /root/.local/bin/uv run --no-project --python
    // /opt/bike-venv/bin/python3 python /tmp/bike-sincos-probe.py (3.13.16).
    // Full reproducible probe, identities and outputs are retained in
    // migration/cpp-linux-completion-evidence.md, Task 1.18's dated entry.
#if defined(__APPLE__) && defined(__arm64__)
    constexpr std::array<std::array<std::uint64_t, 2>, 4> expected_nonfinite{{
        {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff8000000000000)},
        {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff8000000000000)},
        {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff8000000000000)},
        {UINT64_C(0xfff8000000000000), UINT64_C(0xfff8000000000000)},
    }};
#elif defined(__linux__) && defined(__aarch64__)
    constexpr std::array<std::array<std::uint64_t, 2>, 4> expected_nonfinite{{
        {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff8000000000000)},
        {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff8000000000000)},
        {UINT64_C(0x7ff8000000000000), UINT64_C(0x7ff8000000000000)},
        {UINT64_C(0xfff8000000000000), UINT64_C(0xfff8000000000000)},
    }};
#endif

    struct BitExpectation {
        std::uint64_t result;
        std::uint64_t input;
        std::string_view operation;
    };

    void require_bits(double actual, BitExpectation expected) {
        const auto actual_bits = std::bit_cast<std::uint64_t>(actual);
        if (actual_bits == expected.result) return;
        std::ostringstream message;
        message << expected.operation << " input=0x" << std::hex << std::setfill('0')
                << std::setw(16) << expected.input << " expected=0x" << std::setw(16)
                << expected.result << " actual=0x" << std::setw(16) << actual_bits;
        throw std::runtime_error(message.str());
    }

    void check_analytic_contracts() {
        require_bits(numeric::np_sin(0.),
                     {.result = 0, .input = 0, .operation = "np_sin signed zero"});
        require_bits(numeric::np_sin(-0.),
                     {.result = UINT64_C(0x8000000000000000),
                      .input = UINT64_C(0x8000000000000000), .operation = "np_sin signed zero"});
        require_bits(numeric::np_cos(0.),
                     {.result = UINT64_C(0x3ff0000000000000),
                      .input = 0, .operation = "np_cos signed zero"});
        require_bits(numeric::np_cos(-0.),
                     {.result = UINT64_C(0x3ff0000000000000),
                      .input = UINT64_C(0x8000000000000000), .operation = "np_cos signed zero"});
        // Nonzero finite controls include the known same-operand fusion
        // witness. Volatile loads keep the reference libm calls separate.
        for (const double input : {-1., 0.5, -0.57532456093834838}) {
            const volatile double value = input;
            const auto bits = std::bit_cast<std::uint64_t>(input);
            const auto sin_bits = std::bit_cast<std::uint64_t>(std::sin(value));
            const auto cos_bits = std::bit_cast<std::uint64_t>(std::cos(value));
            require_bits(numeric::np_sin(value),
                         {.result = sin_bits, .input = bits, .operation = "np_sin finite"});
            require_bits(numeric::np_cos(value),
                         {.result = cos_bits, .input = bits, .operation = "np_cos finite"});
            require_bits(numeric::math_sin(value),
                         {.result = sin_bits, .input = bits, .operation = "math_sin finite"});
            require_bits(numeric::math_cos(value),
                         {.result = cos_bits, .input = bits, .operation = "math_cos finite"});
        }
    }

    void check_nonfinite_bits() {
#if (defined(__APPLE__) && defined(__arm64__)) || (defined(__linux__) && defined(__aarch64__))
        for (std::size_t index = 0; index < inputs.size(); ++index) {
            const double value = std::bit_cast<double>(inputs[index]);
            require_bits(numeric::np_sin(value),
                         {.result = expected_nonfinite[index][0],
                          .input = inputs[index], .operation = "np_sin nonfinite"});
            require_bits(numeric::np_cos(value),
                         {.result = expected_nonfinite[index][1],
                          .input = inputs[index], .operation = "np_cos nonfinite"});
        }
#else
        throw std::runtime_error("nonfinite NumPy bits are unmeasured on this platform");
#endif
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
        check_nonfinite_bits();
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
