// W01/V01: two independently captured runs of the S00 planar base per platform.
// A match establishes repeatability of that pinned artifact, not a regression
// comparison with a later build. Do not recapture reference files from a changed
// implementation to make a failure disappear.
#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "io/hash.hpp"
#include "io/value.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

namespace io = bike::io;
namespace fs = std::filesystem;

struct Stage {
    std::string_view name;
    std::string_view path_key;
};
constexpr std::array<Stage, 4> kStages{{
    {"state_t0", "state_t0_path"},
    {"committed_steps", "committed_steps_path"},
    {"reset", "reset_path"},
    {"probe", "probe_path"},
}};
constexpr std::array<std::string_view, 2> kPlatforms{
    "macos-arm64-accelerate", "linux-aarch64-openblas"};
constexpr std::array<std::string_view, 3> kTraces{
    "flat", "rough_uphill_savage.toml", "alpine_rocky.toml"};

const io::Value& required(const io::Value& value, std::string_view key,
                          std::string_view path) {
    const auto* result = io::find_member(io::as_object(value, path), key);
    if (result == nullptr || io::is_null(*result))
        throw std::invalid_argument(std::string(path) + "." + std::string(key) +
                                    ": missing/null; pending_verification");
    return *result;
}

std::string nonempty(const io::Value& value, std::string_view key,
                     std::string_view path) {
    const std::string field = std::string(path) + "." + std::string(key);
    const auto& result = io::as_string(required(value, key, path), field);
    bt::require(!result.empty(), field + ": must not be empty");
    return result;
}

std::string identity(const io::Value& value, std::string_view key,
                     std::string_view path, std::size_t digits) {
    auto result = nonempty(value, key, path);
    const bool hex = result.size() == digits &&
        std::all_of(result.begin(), result.end(), [](char ch) {
            return (ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f');
        });
    bt::require(hex, std::string(path) + "." + std::string(key) +
                         ": malformed lowercase-hex identity");
    return result;
}

void claim_payload(const fs::path& path, std::vector<fs::path>& used,
                   const std::string& field) {
    std::error_code error;
    bt::require(fs::is_regular_file(path, error) && !error,
                field + ": expected a regular payload file");
    for (const auto& previous : used) {
        const bool same = fs::equivalent(path, previous, error);
        bt::require(!error, field + ": cannot establish payload file identity");
        bt::require(!same && path != previous,
                    field + ": aliased payload; independent capture files required");
    }
    used.push_back(path);
}

struct Capture {
    std::string capture_id;
    std::string source_sha;
    std::string executable_sha256;
    std::string execution_identity_sha256;
    std::string config_canonical_sha256;
    std::string model_sha256;
    std::string provider;
    std::string compiler;
    std::array<fs::path, 4> paths;
    std::array<std::string, 4> payload_sha256;
};

Capture read_capture(const io::Value& value, const std::string& path,
                     std::vector<fs::path>& used_paths,
                     std::set<std::string>& used_capture_ids) {
    // This member is required before any payload reads. A null placeholder
    // cannot silently turn the comparison into a skipped or empty test.
    const auto& hashes = required(value, "payload_sha256", path);
    bt::require(io::as_object(hashes, path + ".payload_sha256").size() ==
                    kStages.size(), path + ": exactly four payload digests required");
    bt::require(nonempty(value, "status", path) == "captured",
                path + ".status: independent capture not materialized");
    Capture result;
    result.capture_id = nonempty(value, "capture_id", path);
    bt::require(used_capture_ids.emplace(result.capture_id).second,
                path + ".capture_id: must identify an independent invocation");
    result.source_sha = identity(value, "source_sha", path, 40);
    result.executable_sha256 = identity(value, "executable_sha256", path, 64);
    result.execution_identity_sha256 = identity(value, "execution_identity_sha256", path, 64);
    result.config_canonical_sha256 = identity(value, "config_canonical_sha256", path, 64);
    result.model_sha256 = identity(value, "model_sha256", path, 64);
    result.provider = nonempty(value, "provider", path);
    result.compiler = nonempty(value, "compiler", path);
    bt::require(io::as_int64(required(value, "exit_code", path), path + ".exit_code") == 0,
                path + ": capture command failed");
    const auto& command = io::as_array(required(value, "command", path), path + ".command");
    bt::require(!command.empty(), path + ".command: argv is required");
    for (const auto& argument : command)
        bt::require(!io::as_string(argument, path + ".command[]").empty(),
                    path + ".command: argv elements must be nonempty");
    // Commands are evidence only; the test never executes manifest input.
    for (std::size_t i = 0; i < kStages.size(); ++i) {
        const auto relative = nonempty(value, kStages[i].path_key, path);
        result.paths[i] = spatial_tests::fixture(relative);
        claim_payload(result.paths[i], used_paths,
                      path + "." + std::string(kStages[i].path_key));
        result.payload_sha256[i] = identity(
            hashes, kStages[i].name, path + ".payload_sha256", 64);
    }
    return result;
}

std::string decoded_digest(std::span<const double> values) {
    // Reconstruct the exact little-endian payload bits. This binds the
    // read_f64 result to the checked digest even if a fixture changed
    // between the raw-file hash and the decoder's separate file read.
    io::Sha256 digest;
    for (const double value : values) {
        const auto bits = std::bit_cast<std::uint64_t>(value);
        std::array<std::byte, 8> little_endian{};
        for (unsigned byte = 0; byte < 8; ++byte)
            little_endian[byte] = static_cast<std::byte>((bits >> (8U * byte)) & 0xffU);
        digest.update(little_endian);
    }
    constexpr std::string_view alphabet = "0123456789abcdef";
    std::string hex;
    hex.reserve(64);
    for (const auto byte : digest.finish()) {
        hex.push_back(alphabet[byte >> 4U]);
        hex.push_back(alphabet[byte & 0xfU]);
    }
    return hex;
}

std::vector<double> read_verified_state(const fs::path& path,
                                        const std::string& expected_hash,
                                        const std::string& field) {
    {
        const auto raw = spatial_tests::bytes(path);
        bt::require(!raw.empty(), field + ": empty payload cannot establish parity");
        bt::require(spatial_tests::sha256_hex(raw) == expected_hash,
                    field + ": raw payload SHA-256 mismatch");
    }
    auto values = bt::read_f64(path);
    bt::require(!values.empty(), field + ": zero state elements");
    bt::require(decoded_digest(values) == expected_hash,
                field + ": decoded bytes differ from the identified payload");
    return values;
}

void compare(const Capture& reference, const Capture& candidate,
             const std::string& base_sha, const std::string& path) {
    bt::require(reference.capture_id != candidate.capture_id,
                path + ": reference and candidate must be different captures");
    bt::require(reference.source_sha == base_sha && candidate.source_sha == base_sha,
                path + ": both repeatability captures must use the S00 base");
    bt::require(reference.provider == candidate.provider,
                path + ": provider identities differ");
    bt::require(reference.compiler == candidate.compiler,
                path + ": compiler identities differ");
    bt::require(reference.executable_sha256 == candidate.executable_sha256 &&
                    reference.execution_identity_sha256 == candidate.execution_identity_sha256,
                path + ": bitwise comparison requires the same execution artifact/identity");
    bt::require(reference.config_canonical_sha256 == candidate.config_canonical_sha256,
                path + ": canonical configuration identities differ");
    bt::require(reference.model_sha256 == candidate.model_sha256,
                path + ": model identities differ");
    for (std::size_t stage = 0; stage < kStages.size(); ++stage) {
        const auto field = path + "." + std::string(kStages[stage].name);
        const auto reference_state = read_verified_state(
            reference.paths[stage], reference.payload_sha256[stage], field + ".reference");
        const auto candidate_state = read_verified_state(
            candidate.paths[stage], candidate.payload_sha256[stage], field + ".candidate");
        bt::require(reference_state.size() == candidate_state.size(),
                    field + ": reference/candidate state lengths differ");
        for (std::size_t i = 0; i < reference_state.size(); ++i)
            bt::require(std::bit_cast<std::uint64_t>(reference_state[i]) ==
                            std::bit_cast<std::uint64_t>(candidate_state[i]),
                        field + ": bitwise mismatch at element " + std::to_string(i));
    }
}

} // namespace

int main() {
    return bt::run([] {
        const auto manifest = spatial_tests::json(
            spatial_tests::fixture("planar-baseline/manifest.json"));
        bt::require(io::as_int64(required(manifest, "manifest_version", "manifest"),
                                "manifest.manifest_version") == 1, "planar manifest version");
        const auto base_sha = identity(manifest, "base_sha", "manifest", 40);
        const auto source_baseline = spatial_tests::json(
            fs::path{BIKE_SPATIAL_SOURCE_ROOT} / "migration/spatial3d/baseline.json");
        bt::require(base_sha == identity(source_baseline, "base_sha", "S00", 40),
                    "planar captures must identify the S00 source checkpoint");
        const auto& contract = required(manifest, "capture_contract", "manifest");
        bt::require(io::as_int64(required(contract, "schema_version", "capture_contract"),
                                "capture_contract.schema_version") == 1,
                    "capture contract version");
        const auto& platforms = required(manifest, "platforms", "manifest");
        bt::require(io::as_object(platforms, "manifest.platforms").size() == kPlatforms.size(),
                    "both declared platform identities are required");
        std::vector<fs::path> used_paths;
        std::set<std::string> used_capture_ids;
        std::size_t compared = 0;
        for (const auto platform : kPlatforms) {
            const auto platform_path = "manifest.platforms." + std::string(platform);
            const auto& record = required(platforms, platform, "manifest.platforms");
            const auto& traces = io::as_array(required(record, "traces", platform_path),
                                               platform_path + ".traces");
            bt::require(traces.size() == kTraces.size(), platform_path + ": all three traces required");
            std::set<std::string> names;
            for (const auto& trace : traces) {
                const auto name = nonempty(trace, "name", platform_path + ".traces[]");
                bt::require(std::find(kTraces.begin(), kTraces.end(), name) != kTraces.end(),
                            platform_path + ": unknown trace");
                bt::require(names.emplace(name).second, platform_path + ": duplicate trace");
                const auto path = platform_path + ".traces[" + name + "]";
                const auto reference = read_capture(trace, path, used_paths, used_capture_ids);
                const auto candidate = read_capture(required(trace, "candidate", path),
                                                     path + ".candidate", used_paths, used_capture_ids);
                compare(reference, candidate, base_sha, path);
                ++compared;
            }
        }
        bt::require(compared == kPlatforms.size() * kTraces.size(),
                    "no platform or trace may be skipped");
    });
}
