// fixture_test.hpp — standalone contract-test harness. The fixture root
// arrives as the BIKE_FIXTURE_ROOT macro from the configure-generated
// bike_fixture_paths.hpp (native/tests/CMakeLists.txt),
// computed from the repository root — never the process working directory.
#pragma once

#include <cstdint>
#include <filesystem>
#include <source_location>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bt {

// Runs one test body. Returns 0 when every check passed, 1 otherwise; a body
// that ran zero bt::require checks fails closed.
int run(const std::function<void()> &test);

// Throws std::runtime_error carrying message when condition is false. Real
// code in every build configuration — never assert()/NDEBUG — so Release
// runs keep the full contract.
void require(bool condition, std::string_view message);

// Resolves relative under the CMake-supplied fixture root. Rejects empty,
// absolute and '..' paths before canonicalization and any canonicalized
// escape after it. A missing fixture is a blocking acquisition failure, not
// a skipped comparison.
std::filesystem::path fixture(std::string_view relative);

// Whole-file binary-safe read bounded at 512 MiB (test payloads are bounded;
// large trajectories are chunked into declared fixture pieces).
std::string text(const std::filesystem::path &path);

// Explicit little-endian IEEE-754 binary64 decode; the byte count must be a
// multiple of eight. Never a rounded text round-trip.
std::vector<double> read_f64(const std::filesystem::path &path);

// Record mode: environment BIKE_GOLDEN_RECORD=1 (the same switch as
// bt::oracle::Node::recording()). Golden comparisons then rewrite their
// fixture with the actual value instead of comparing — used only to
// re-record goldens after a reviewed model change.
[[nodiscard]] bool recording();

// Whole-file writes for record mode (binary-safe; binary64 little-endian).
void write_text(const std::filesystem::path &path, std::string_view content);
void write_f64(const std::filesystem::path &path, std::span<const double> values);

// Byte-exact comparison of actual against the fixture file at relative.
// Record mode rewrites the fixture when it differs.
void golden_text(std::string_view relative, std::string_view actual);

// Bitwise binary64 comparison: uint64_t object representations, so signed
// zero and NaN bit payloads are significant. No epsilon. Record mode
// rewrites the fixture when it differs.
void golden_f64(std::string_view relative, std::span<const double> actual);

// Succeeds only when action() throws std::invalid_argument (or a type
// derived from it). A plain return fails via require; any other exception
// propagates to run() as a test failure instead of falsely satisfying the
// validation contract.
template <class F> void invalid(F &&action,
    const std::source_location &loc = std::source_location::current()) {
    bool rejected = false;
    try {
        std::forward<F>(action)();
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    require(rejected, (std::string("expected std::invalid_argument at ") +=
                       loc.file_name()) += ":" + std::to_string(loc.line()));
}

} // namespace bt
