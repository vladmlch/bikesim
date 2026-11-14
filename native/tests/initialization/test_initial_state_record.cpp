// The record-mode helper regenerates schema-2 initial-state documents: the
// written document must bind through the ordinary loader path (its contract
// is validated against the live model inside settle), be byte-identical when
// regenerated, and the helper must touch nothing outside record mode.
#include "support/fixture_test.hpp"
#include "support/record_initial_state.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "platform/provenance.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

namespace {

// Process-local BIKE_GOLDEN_RECORD: the suite runs without the flag, so the
// test scopes its own record-mode window and always restores the caller's
// environment (unsetenv keeps a missing variable absent either way).
class RecordFlag {
public:
    RecordFlag() { ::setenv("BIKE_GOLDEN_RECORD", "1", 1); }
    ~RecordFlag() { ::unsetenv("BIKE_GOLDEN_RECORD"); }
    RecordFlag(const RecordFlag&) = delete;
    RecordFlag& operator=(const RecordFlag&) = delete;
    RecordFlag(RecordFlag&&) = delete;
    RecordFlag& operator=(RecordFlag&&) = delete;
};

void write(const std::filesystem::path& file, std::string_view text) {
    std::ofstream output(file, std::ios::binary);
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.write(text.data(), static_cast<std::streamsize>(text.size()));
    output.close();
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] void record_mode_regenerates_the_platform_keyed_document() {
    const bt::TemporaryDirectory directory("initial_state_record");
    std::filesystem::create_directory(directory.path() / "case");
    std::filesystem::create_directory(directory.path() / "config");
    std::filesystem::create_directory(directory.path() / "tracks");
    for (const std::string_view file : {"viewer_physics_welded.toml",
             "rider_joint_envelope_anatomical.json",
             "rider_strength_reference.json"})
        std::filesystem::copy_file(bt::fixture("config/" + std::string(file)),
            directory.path() / "config" / file);
    std::filesystem::copy_file(bt::fixture("tracks/flat.toml"),
        directory.path() / "tracks/flat.toml");
    const auto request_file = directory.path() / "case/request.toml";
    write(request_file,
        "request_schema = 1\nphysics_file = '../config/viewer_physics_welded.toml'\n"
        "track_file = '../tracks/flat.toml'\nmodel_mode = 'ride'\n"
        "rider_variant = 'articulated_planar'\nspeed_kph = 0.0\n"
        "seed_decimal = '1234'\nuse_equilibrium_cache = false\n"
        "initial_state_file = '../state/doc.json'\n");
    // The platform-keyed sibling supersedes the shared spelling on the write
    // side exactly like the loader's read side: it only wins when it already
    // exists, so the test pre-creates it as the lone target.
    const auto keyed = directory.path() / "state" / bike::platform::key() /
        "doc.json";
    std::filesystem::create_directories(keyed.parent_path());
    write(keyed, "{}");

    { const RecordFlag flag; bt::record_initial_state(request_file); }
    const auto document = bt::text(keyed);
    const auto state = bike::io::parse_json(document);
    bt::require(bike::io::find_member(
                    bike::io::as_object(state, "state"), "contract_sha256") !=
                nullptr,
        "record mode writes a schema-2 transfer document");

    // The ordinary loader path accepts the regenerated document: the request
    // binds it as the transfer input and settle validates the contract inside
    // PhysicalInitialState::prepare (a stale contract throws there).
    auto request = bike::initialization::read_setup_request(request_file);
    bt::require(request.initial_state.has_value(),
        "the regenerated document binds as the request's transfer input");
    const auto options = request.equilibrium;
    auto owner = bike::initialization::make_setup(std::move(request));
    const auto result = bike::initialization::settle(*owner, options);
    bt::require(result.initial_state_sha256.has_value(),
        "the settle restores the recorded transfer instead of settling fresh");

    { const RecordFlag flag; bt::record_initial_state(request_file); }
    bt::require(bt::text(keyed) == document,
        "regeneration is byte-identical");

    bt::write_text(keyed, "sentinel");
    bt::record_initial_state(request_file);
    bt::require(bt::text(keyed) == "sentinel",
        "outside record mode the helper writes nothing");
}

} // namespace

int main() {
    return bt::run([] { record_mode_regenerates_the_platform_keyed_document(); });
}
NATIVE_DIAG_POP
