#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <mujoco/mujoco.h>

#include "../configuration/model_config.hpp"
#include "../geometry/specs.hpp"
#include "../kinematics/solver.hpp"
#include "../terrain/types.hpp"
#include "mass.hpp"
#include "rider_geometry.hpp"

namespace bike::model {

// Cold, owning model input. The optional solver preserves custom linkage
// reference points; the optional rider preserves include_rider precedence.
// No IDs, live state, Python objects, or borrowed configuration are stored here.
struct BuildRequest {
    geometry::BikeSpecs bike;
    std::optional<kinematics::HorstLinkageSolver> solver;
    BikeMassSpecs masses;
    std::string mode = "standard";
    bool include_rider = false;
    bool debug_markers = false;
    std::optional<configuration::RiderSpecs> rider;
    terrain::HeightFieldSpec field;
    std::string tyre_model = "sphere";
    bool crank_joint = false;
    double gear_ratio = 32.0 / 14.0;
    std::optional<configuration::SimulationPhysicsConfig> physics;
};

[[nodiscard]] std::string generate_mjcf(const BuildRequest& request);
struct ModelDeleter { void operator()(mjModel* model) const noexcept; };
using ModelOwner = std::unique_ptr<mjModel, ModelDeleter>;
// asset_root anchors relative includes/assets without changing process cwd.
[[nodiscard]] ModelOwner compile_mjcf(
    std::string_view xml, const std::filesystem::path& asset_root = {});

} // namespace bike::model
