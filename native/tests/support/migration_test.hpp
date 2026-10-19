// migration_test.hpp — standalone contract-test harness shared by the
// Python→C++ migration regressions (migration/interface-contracts.md §8).
// The fixture root arrives as the BIKE_FIXTURE_ROOT compile definition wired
// by native/tests/migration/CMakeLists.txt — never the process working
// directory. Registration is one executable and one CTest name
// (migration_TNN) per task; T05 layers strict manifest/SHA-256 validation
// onto this seam.
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

// Byte-exact comparison of actual against the fixture file at relative.
void golden_text(std::string_view relative, std::string_view actual);

// Bitwise binary64 comparison: uint64_t object representations, so signed
// zero and NaN bit payloads are significant. No epsilon, no update mode.
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

// ---- strict oracle-manifest/provenance validation (task T05) --------------
// The manifest layer consumes the owned bike::io JSON boundary: parsing is
// the lossless tree (duplicate keys, trailing bytes, malformed UTF-8 all
// rejected) and every structural violation is std::invalid_argument naming
// the complete member path — the harness never lenient-parses provenance.

// One declared byte artifact inside a manifest "files" entry. shape/dtype/
// order are present together or not at all; when present the declared
// element count times the dtype's element size must equal "bytes" exactly.
struct ManifestFile {
    std::string path;
    std::int64_t bytes = 0;
    std::string sha256;
    std::vector<std::int64_t> shape;
    std::string dtype;
    std::string order;
};

// One parsed, structurally validated manifest fixture entry.
struct ManifestFixture {
    std::string name;
    std::string group;
    char comparison = 0; // B | N | P | Q | S
    std::string kind;    // accepted | invalid | diagnostic | ...
    std::vector<ManifestFile> files;
};

// Structural + provenance validation of one manifest document: schema
// marker, oracle commit/SHA-256 hex identity, tool identity, environment,
// per-fixture member sets, comparison codes, expected-kind field shapes and
// file declarations (path containment shape, 64-hex digests, byte/shape
// consistency). Throws std::invalid_argument naming the offending member.
// File digests are verified separately by manifest_verify_files().
[[nodiscard]] std::vector<ManifestFixture>
manifest_parse(std::string_view source);

// Reads each declared file through fixture() + text() and verifies byte
// count and lowercase-hex SHA-256 against the declaration. A discrepancy is
// a fail-closed require() failure naming the file — never a skip.
void manifest_verify_files(const ManifestFixture &entry);

// The parsed manifest for this build (BIKE_ORACLE_MANIFEST, read through
// the same bounded text() contract); lazy and cached. Structural
// violations propagate as std::invalid_argument — strict.
[[nodiscard]] const std::vector<ManifestFixture> &manifest();

// The one manifest entry named `name`; require() failure when absent — an
// undeclared fixture is an acquisition failure, not a substitution.
[[nodiscard]] const ManifestFixture &manifest_entry(std::string_view name);

// T05+: like fixture() but the manifest entry is structurally validated and
// every declared file digest is verified first; the returned path is still
// re-checked through the T02 traversal contract.
[[nodiscard]] std::filesystem::path verified_fixture(std::string_view name);

} // namespace bt
