// T25 source regressions: normal TOML -> generated model -> native equilibrium
// -> sole ride owner. These tests are authored here, not executed in the
// no-build/no-run implementation session.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "app/physical_session.hpp"
#include "configuration/load.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "runtime/setup_values.hpp"
#include "terrain/profile.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace {
class Directory {
public:
    Directory() : path(std::filesystem::temp_directory_path() /
        ("bike-t25-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()))) {
        if (!std::filesystem::create_directory(path))
            throw std::runtime_error("cannot create isolated T25 test directory");
    }
    ~Directory() { std::error_code error; std::filesystem::remove_all(path, error); }
    Directory(const Directory&) = delete;
    Directory& operator=(const Directory&) = delete;
    Directory(Directory&&) = delete;
    Directory& operator=(Directory&&) = delete;
    std::filesystem::path path;
};

void write(const std::filesystem::path& file, std::string_view text) {
    std::ofstream output(file, std::ios::binary);
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.write(text.data(), static_cast<std::streamsize>(text.size()));
    output.close();
}

void same(std::span<const double> a, std::span<const double> b, std::string_view message) {
    bt::require(a.size() == b.size(), message);
    for (std::size_t i = 0; i < a.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(a[i]) == std::bit_cast<std::uint64_t>(b[i]),
            std::string(message) + " at integration index " + std::to_string(i));
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] void request_contract() {
    Directory const directory;
    std::filesystem::create_directory(directory.path / "case");
    std::filesystem::create_directory(directory.path / "config");
    std::filesystem::create_directory(directory.path / "tracks");
    for (std::string_view const file : {"viewer_physics_welded.toml", "rider_joint_envelope_anatomical.json",
                                 "rider_strength_reference.json"})
        std::filesystem::copy_file(bt::fixture("config/" + std::string(file)), directory.path / "config" / file);
    std::filesystem::copy_file(bt::fixture("tracks/flat.toml"), directory.path / "tracks/flat.toml");
    const auto request_file = directory.path / "case/request.toml";
    const std::string document =
        "request_schema = 1\nphysics_file = '../config/viewer_physics_welded.toml'\n"
        "track_file = '../tracks/flat.toml'\nmodel_mode = 'ride'\n"
        "rider_variant = 'articulated_planar'\nspeed_kph = 0.0\n"
        "seed_decimal = '184467440737095516160012345'\nuse_equilibrium_cache = false\n";
    write(request_file, document);
    auto input = bike::initialization::read_setup_request(request_file);
    bt::require(input.track.name == "flat" && input.track.length_m == 112.,
        "request resolves track relative to the declaring file");
    bt::require(input.seed_decimal == "184467440737095516160012345", "request seed is never narrowed");
    bt::require(input.model.physics.value().initial_speed_mps == 0., "explicit initial speed retained"); // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    bt::require(input.model.physics.value().articulated.joint_strength_path && // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        std::filesystem::path(input.model.physics.value().articulated.joint_strength_path.value()).is_absolute(), // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        "strength path resolves against the physics file");
    write(request_file, document + "unknown_field = 1\n");
    bool named = false;
    try { (void)bike::initialization::read_setup_request(request_file); }
    catch (const std::invalid_argument& error) {
        named = std::string(error.what()).find(request_file.string()) != std::string::npos;
    }
    bt::require(named, "invalid request identifies its source filename");

    // --request FILE --fresh must discard a saved-state input before opening
    // it, including inputs that cannot be acquired or parsed. The ordinary
    // one-argument API retains its fail-closed acquisition behavior.
    write(request_file, document + "initial_state_file = 'missing-state.json'\n");
    bool missing_state = false;
    try { (void)bike::initialization::read_setup_request(request_file); }
    catch (const std::system_error& error) {
        missing_state = error.code() == std::errc::no_such_file_or_directory;
    }
    bt::require(missing_state, "ordinary request does not silently ignore a missing saved state");
    input = bike::initialization::read_setup_request(request_file, true);
    bt::require(!input.initial_state && !input.model.physics.value().equilibrium_state_file, // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        "fresh request does not open the missing saved state");
    write(directory.path / "case/broken-state.json", "{broken-json");
    write(request_file, document + "initial_state_file = 'broken-state.json'\n");
    bt::invalid([&] { (void)bike::initialization::read_setup_request(request_file); });
    input = bike::initialization::read_setup_request(request_file, true);
    bt::require(!input.initial_state && !input.model.physics.value().equilibrium_state_file, // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        "fresh request does not parse the discarded saved state");
    write(request_file, document + "initial_state_file = 'missing-state.json'\nunknown_field = 1\n");
    bt::invalid([&] { (void)bike::initialization::read_setup_request(request_file, true); });

    write(request_file, document + "[overrides]\ninitial_front_brake = 0.25\n");
    input = bike::initialization::read_setup_request(request_file);
    bt::require(input.model.physics.value().initial_front_brake == .25, "request override precedence"); // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    input.model.physics.value().drive.motor_clutch = true; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    input.model.physics.value().drive.rotor_inertia_kgm2 = .02; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    bt::invalid([&] { (void)bike::app::prepare(input); });
    bt::require(!std::filesystem::exists(directory.path / "runs") &&
        !std::filesystem::exists(directory.path / "manifest.json"),
        "failed setup publishes no recording or successful manifest");
}

[[gnu::noinline]] void ownership_and_reset() {
    auto const request = bike::initialization::read_setup_request(bt::fixture("requests/welded_flat.toml"));
    auto early = bike::initialization::make_setup(request);
    bt::invalid([&] { (void)early->seal_bootstrap(); });
    early.reset();

    auto prepared = bike::app::prepare(request);
    bt::require(bike::io::as_bool(runtime::setupvalue::field(prepared.metadata, "sealed"), "sealed"),
        "prepare returns the completed immutable reset state");
    auto ride = bike::app::make_ride(std::move(prepared));
    const auto initial = ride->snapshot();
    bt::require(initial.step == 0 && initial.time_s == 0., "native startup seals t=0");
    const auto advance = ride->advance(3, runtime::RideControl{}, 0., 0., std::nullopt);
    bt::require(advance.step == 3, "prepared owner can advance the existing native runtime");
    ride->reset();
    const auto reset = ride->snapshot();
    bt::require(reset.step == 0 && reset.time_s == 0. && reset.generation > initial.generation,
        "reset restores time and advances generation");
    same(initial.integration_state, reset.integration_state, "reset restores the exact integration state");
    auto independent = bike::app::make_ride(bike::app::prepare(request));
    same(initial.integration_state, independent->snapshot().integration_state,
        "separate preparation has no leaked controller/material state");
    ride->close();
    bt::require(!independent->closed(), "closing one owner leaves another owner intact");

    // The immutable oracle gate stays bitwise and fail-closed when a binary
    // artifact is missing; this session does not create replacement goldens.
    bt::manifest_verify_files(bt::manifest_entry("bootstrap/native_start_welded"));
    bt::golden_f64("bootstrap/native_start_welded/integration.f64", initial.integration_state);
}

[[gnu::noinline]] void shipped_backends() {
    for (std::string_view const profile : {"welded", "distributed", "pneumatic"}) {
        auto request = bike::initialization::read_setup_request(
            bt::fixture("requests/" + std::string(profile) + "_flat.toml"));
        // The shipped pneumatic profile explicitly names a legacy pose.
        // This case deliberately asks for a fresh native equilibrium, the
        // same explicit choice exposed by bike-native-ride --fresh.
        request.model.physics.value().equilibrium_state_file.reset(); // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        request.model.physics.value().equilibrium_cache_enabled = false; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        auto ride = bike::app::make_ride(bike::app::prepare(std::move(request)));
        bt::require(ride->snapshot().step == 0, "shipped backend prepares from ordinary configuration");
        bt::require(ride->advance(1, runtime::RideControl{}, 0., 0., std::nullopt).step == 1,
            "shipped backend advances without an imported bootstrap");
    }
}
} // namespace

int main() {
    return bt::run([] {
        request_contract();
        shipped_backends();
        ownership_and_reset();
    });
}
NATIVE_DIAG_POP
