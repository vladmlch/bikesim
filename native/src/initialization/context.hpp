#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include "../configuration/model_config.hpp"
#include "../io/value.hpp"
#include "../rider/spindle_controller.hpp"
#include "equilibrium.hpp"
#include "finalize.hpp"
#include "../terrain/render3d.hpp"
#include "../spatial/topology.hpp"

class Stepper;
namespace runtime { class PhysicalStep; }

namespace bike::initialization {

class BootstrapHold;

// Borrowed exclusively by SetupOwner before seal; never constructed from a
// live NativeRideRuntime. Numeric fields are resolved from the same model
// and source configuration used to construct these two owners.
struct SetupContext {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    Stepper& stepper;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    runtime::PhysicalStep& physical;
    configuration::SimulationPhysicsConfig physics;
    double start_x_m = 2.;
    // compute_ground_z(specs)/1000, in the bike model's local frame.
    double ground_z_m = 0.;
    int frame_body_id = -1;
    int front_geom_id = -1;
    int rear_geom_id = -1;
    std::optional<::rider::SpindlePose> rider_pose{};
    std::function<double(double)> rear_travel_from_shock_mm{};
    // Complete source documents: specs, mass_specs, rider and track. The
    // cache adds normalized physics, start position, compiled arrays,
    // schema and engine/provider/source identity. Reuse knobs are excluded.
    io::Value identity{};
    std::string source_identity{};
    std::string provider_identity{};
    std::string model_sha256{};
    // Historical fingerprint inputs are distinct from native identity. They
    // are populated only from explicitly supplied source bytes and the old
    // track_to_dict wire; no legacy digest overrides native cache identity.
    std::optional<std::string> legacy_source_identity{};
    std::optional<io::Value> legacy_track_identity{};
    std::optional<std::filesystem::path> cache_directory{};
    bool initializing = true;
    bool completed = false;
    std::optional<EquilibriumResult> equilibrium{};
    // Acceleration tolerance accepted by the settle request. Finalize and the
    // bootstrap release check must judge with this same value instead of their
    // own literals (plan 04 W29/W30); .05 is the historical default. The settle
    // owner is responsible for setting it from the request.
    double equilibrium_tolerance = .05;
    const PhysicalInitialState* initial_state = nullptr;
    std::shared_ptr<const terrain::Terrain3dSource> terrain3d{};
    std::optional<terrain::RenderGrid3d> render3d{};
    std::optional<spatial::Topology> topology{};
    std::shared_ptr<BootstrapHold> bootstrap_hold{};
    std::shared_ptr<SpatialEquilibriumReport> spatial_report{};
    mutable std::optional<std::string> cache_rejection_reason{};
};

// Resolves a scalar joint once, rejecting missing/free/ball joints instead
// of accidentally equating a joint id with a qpos/dof address.
struct JointAddress { int joint; int qpos; int dof; };
[[nodiscard]] JointAddress joint_address(const SetupContext&, std::string_view name);
void require_initializing(const SetupContext&);

} // namespace bike::initialization
