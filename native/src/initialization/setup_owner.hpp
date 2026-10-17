#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include "equilibrium.hpp"
#include "sag_fit.hpp"
#include "state_file.hpp"
#include "../model/builder.hpp"
#include "../runtime/project_config.hpp"
#include "../runtime/runtime.hpp"

namespace bike::initialization {

struct SetupRequest {
    model::BuildRequest model;
    terrain::TrackSpec track;
    runtime::ProjectionOptions runtime;
    EquilibriumOptions equilibrium;
    double start_x_m = 2.;
    std::string seed_decimal = "0";
    std::filesystem::path asset_root;
    std::optional<std::filesystem::path> cache_directory;
    // Optional compatibility input for a historical NPZ fingerprint. Native
    // startup needs no Python source tree; this reads bytes, never imports it.
    std::optional<std::filesystem::path> legacy_source_root;
    std::optional<PhysicalInitialState> initial_state;
    std::string origin;
};

// Strict request_schema=1 assembly document; referenced files and nested
// override assets resolve relative to their declaring document.
// An explicit fresh-start override can ignore initial_state_file without
// opening it; every other request/configuration/track field still validates.
[[nodiscard]] SetupRequest read_setup_request(
    const std::filesystem::path&, bool skip_initial_state = false);
// Setup seeds override a present road generator. A materialized track has
// no generator to reseed; its seed is retained only in request metadata.
// The public terrain::load_track explicit-override rejection is unchanged.
[[nodiscard]] terrain::TrackSpec load_track_for_setup(
    const std::filesystem::path&, std::string_view seed_decimal);

class SetupOwner {
public:
    ~SetupOwner();
    SetupOwner(const SetupOwner&) = delete;
    SetupOwner& operator=(const SetupOwner&) = delete;
    SetupOwner(SetupOwner&&) = delete;
    SetupOwner& operator=(SetupOwner&&) = delete;
    [[nodiscard]] runtime::RuntimeBootstrap seal_bootstrap();
    [[nodiscard]] io::Value metadata() const;
    [[nodiscard]] PhysicalInitialState capture_initial_state() const;
    [[nodiscard]] PhysicalInitialState project_initial_stations(int station_count) const;
private:
    struct Impl;
    explicit SetupOwner(std::unique_ptr<Impl>);
    std::unique_ptr<Impl> impl_;
    friend std::unique_ptr<SetupOwner> make_setup(SetupRequest);
    friend EquilibriumResult settle(SetupOwner&, const EquilibriumOptions&);
};

[[nodiscard]] std::unique_ptr<SetupOwner> make_setup(SetupRequest);
[[nodiscard]] std::unique_ptr<SetupOwner> make_setup(const std::filesystem::path&);
[[nodiscard]] SagEvaluator build_equilibrium_evaluator(SetupRequest);

// validation/environment.py source_fingerprint, for explicit legacy imports.
[[nodiscard]] std::string legacy_source_fingerprint(const std::filesystem::path&);

} // namespace bike::initialization
