// test_t02.cpp — first regression for the standalone contract-test harness
// (task T02). The verbatim bootstrap body below is followed by the mandatory
// controls: pass/deliberate-failure pairs for equality, signed zero, NaN bit
// payloads, differing shapes and corrupt files; fixture lookup from an
// unrelated working directory; and nonzero-check enforcement that never
// relies on assert()/NDEBUG.
#include "support/migration_test.hpp"
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace {

    // Deliberate-failure control: returns true only when action() fails
    // through a bt::require-style std::runtime_error. A propagated
    // std::invalid_argument (a logic_error, not runtime_error) or any other
    // escape type returns false/fails the enclosing run() instead.
    bool require_failed(const std::function<void()> &action) {
        try {
            action();
        } catch (const std::runtime_error &) {
            return true;
        } catch (...) {
            return false;
        }
        return false;
    }

    // Scoped working-directory switch for the unrelated-cwd case; restores
    // even when a check throws.
    class WorkingDirectory {
      public:
        explicit WorkingDirectory(const std::filesystem::path &target)
            : saved_(std::filesystem::current_path()) {
            std::filesystem::current_path(target);
        }
        ~WorkingDirectory() {
            std::error_code ignored;
            std::filesystem::current_path(saved_, ignored);
        }
        WorkingDirectory(const WorkingDirectory &) = delete;
        WorkingDirectory &operator=(const WorkingDirectory &) = delete;
        WorkingDirectory(WorkingDirectory &&) = delete;
        WorkingDirectory &operator=(WorkingDirectory &&) = delete;

      private:
        std::filesystem::path saved_;
    };

} // namespace

int main() {
  // cppcheck's throwInEntryPoint cannot see that bt::run catches every test
  // escape; the outer try also covers std::function allocation failure, so a
  // thrown exception can never terminate the entry point.
  try {
    return bt::run([] {
    const double minus_zero = -0.0;
    bt::require(std::bit_cast<std::uint64_t>(minus_zero) !=
                std::bit_cast<std::uint64_t>(0.0), "signed-zero comparator");
    bt::invalid([] [[noreturn]] { throw std::invalid_argument("control"); });
    bt::require(bt::text(bt::fixture("harness/utf8.txt")) == "bike\n", "fixture root");

    // Mandatory controls: equality pass and deliberate failure.
    bt::golden_text("harness/utf8.txt", "bike\n");
    bt::require(
        require_failed(
            [] { bt::golden_text("harness/utf8.txt", "bikf\n"); }),
        "golden_text must reject a differing text payload");

    // binary64 pass control against a real captured fixture ([1.0]).
    const std::vector<double> single =
        bt::read_f64(bt::fixture("allocation/infeasible.f64"));
    bt::require(single.size() == 1 && single[0] == 1.0,
                "read_f64 must decode little-endian binary64");
    bt::golden_f64("allocation/infeasible.f64", single);
    bt::require(
        require_failed(
            [] { bt::golden_f64("allocation/infeasible.f64", std::array{-1.0}); }),
        "golden_f64 must reject a differing value");

    // Different shapes must fail — longer and shorter actual payloads.
    bt::require(
        require_failed([] {
            bt::golden_f64("allocation/infeasible.f64", std::array{1.0, 2.0});
        }),
        "golden_f64 must reject a longer actual payload");
    bt::require(
        require_failed([] {
            bt::golden_f64("allocation/infeasible.f64",
                           std::span<const double>{});
        }),
        "golden_f64 must reject an empty actual payload");

    // Sign of zero is significant: the fixture holds exactly [+0.0].
    const std::vector<double> zero =
        bt::read_f64(bt::fixture("model/welded_arrays/tendon_damping.f64"));
    bt::require(zero.size() == 1 &&
                    std::bit_cast<std::uint64_t>(zero[0]) == 0x0ULL,
                "tendon_damping fixture must contain exactly [+0.0]");
    bt::golden_f64("model/welded_arrays/tendon_damping.f64",
                   std::array{0.0});
    bt::require(
        require_failed([] {
            bt::golden_f64("model/welded_arrays/tendon_damping.f64",
                           std::array{-0.0});
        }),
        "golden_f64 must distinguish -0.0 from +0.0");

    // NaN bit payloads are significant: the fixture holds the canonical
    // quiet NaN [0x7ff8000000000000, +0.0].
    constexpr std::uint64_t canonical_nan = 0x7ff8000000000000ULL;
    const std::vector<double> nan_fixture =
        bt::read_f64(bt::fixture("allocation/nan_target.arrays/a000.f64"));
    bt::require(nan_fixture.size() == 2 &&
                    std::bit_cast<std::uint64_t>(nan_fixture[0]) ==
                        canonical_nan &&
                    std::bit_cast<std::uint64_t>(nan_fixture[1]) == 0x0ULL,
                "nan_target fixture must contain [canonical-qNaN, +0.0]");
    bt::golden_f64("allocation/nan_target.arrays/a000.f64",
                   std::array{std::bit_cast<double>(canonical_nan), 0.0});
    bt::require(
        require_failed([] {
            bt::golden_f64(
                "allocation/nan_target.arrays/a000.f64",
                std::array{std::bit_cast<double>(0x7ff8000000000001ULL), 0.0});
        }),
        "golden_f64 must distinguish different NaN bit payloads");
    bt::require(
        require_failed([] {
            bt::golden_f64(
                "allocation/nan_target.arrays/a000.f64",
                std::array{std::bit_cast<double>(0xfff8000000000000ULL), 0.0});
        }),
        "golden_f64 must distinguish the NaN sign bit");

    // Corrupt/truncated payloads: utf8.txt is 5 bytes, not a multiple of 8,
    // and a missing fixture is a blocking acquisition failure.
    bt::require(
        require_failed([] {
            static_cast<void>(bt::read_f64(bt::fixture("harness/utf8.txt")));
        }),
        "read_f64 must reject a non-multiple-of-8 byte stream");
    bt::require(
        require_failed([] {
            static_cast<void>(bt::fixture("harness/does_not_exist.bin"));
        }),
        "fixture() must fail for a missing file");

    // Path containment: no '..' escapes, no absolute paths, no empty input.
    bt::require(
        require_failed([] { static_cast<void>(bt::fixture("../escape")); }),
        "fixture() must reject '..'");
    bt::require(
        require_failed(
            [] { static_cast<void>(bt::fixture("harness/../../escape")); }),
        "fixture() must reject nested '..' escapes");
    bt::require(
        require_failed(
            [] { static_cast<void>(bt::fixture("/absolute/path")); }),
        "fixture() must reject absolute paths");
    bt::require(
        require_failed([] { static_cast<void>(bt::fixture("")); }),
        "fixture() must reject an empty path");

    // bt::invalid accepts only std::invalid_argument.
    bt::require(require_failed([] { bt::invalid([] {}); }),
                "bt::invalid must fail when the action does not throw");
    bt::require(
        require_failed([] {
            bt::invalid(
                [] [[noreturn]] { throw std::runtime_error("wrong type"); });
        }),
        "bt::invalid must not accept a non-invalid_argument exception");

    // bt::require is real code in every configuration: it throws
    // std::runtime_error instead of aborting, so Release/NDEBUG builds keep
    // the full check contract.
    bt::require(
        require_failed([] { bt::require(false, "deliberate failure"); }),
        "bt::require must throw std::runtime_error, never an assert abort");

    // Fixtures resolve from the CMake-supplied root regardless of the
    // process working directory (the CTest registration also launches the
    // binary from an unrelated directory).
    {
        const WorkingDirectory away(std::filesystem::temp_directory_path());
        bt::require(
            bt::text(bt::fixture("harness/utf8.txt")) == "bike\n",
            "fixture root must resolve from an unrelated working directory");
    }
  });
  } catch (...) {
    return 1;
  }
}
