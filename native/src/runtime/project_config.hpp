#pragma once

#include <mujoco/mujoco.h>
#include "config.hpp"
#include "../configuration/model_config.hpp"
#include "../model/builder.hpp"
#include "../terrain/types.hpp"

namespace runtime {

struct ProjectionOptions {
    bool strict = true;
    bool async_telemetry = false;
    int measurement_decimation = 1;
    RetentionMode record_retention = RetentionMode::off;
    // Legacy cruise target; the physical runtime never enables speed control.
    double speed_kph = 25.;
};

// setup.py validate_supported, on resolved values rather than filenames.
void validate_native_supported(const bike::configuration::SimulationPhysicsConfig&,
                               const bike::configuration::RiderSpecs&);
[[nodiscard]] bike::io::Value project_writer_config(
    const bike::model::BuildRequest&, const mjModel&,
    const bike::terrain::TrackSpec&, const ProjectionOptions& = {});
// Both overloads bind the supplied model to a fresh compilation of their
// typed inputs, including installed binary32 road samples. Call before
// initialization or runtime code mutates model arrays. Matching dimensions
// and names alone never satisfy this boundary.
[[nodiscard]] RuntimeConfig project_runtime(
    const bike::model::BuildRequest&, const mjModel&,
    const bike::terrain::TrackSpec&, const ProjectionOptions& = {});
// Default-bike convenience seam from the migration API index. Custom geometry,
// terrain and rider callers must use the owning BuildRequest overload above.
[[nodiscard]] RuntimeConfig project_runtime(
    const bike::configuration::SimulationPhysicsConfig&, const mjModel&);
[[nodiscard]] bike::io::Value project_rider_controller(
    const bike::configuration::SimulationPhysicsConfig&, const mjModel&);
[[nodiscard]] bike::io::Value project_articulated_pose(
    const bike::model::ArticulatedPose&);

} // namespace runtime
