#pragma once

#include <mujoco/mujoco.h>
#include "config.hpp"
#include "../configuration/model_config.hpp"
#include "../model/builder.hpp"
#include "../terrain/types.hpp"

namespace runtime {

struct ProjectionOptions {
    // The programmatic driver default is diagnostic (PhysicalRideDriver
    // strict=False); strict arrives only through an explicit check policy.
    bool strict = false;
    bool async_telemetry = false;
    // Spatial tyre execution policy only; physics coefficients stay unchanged.
    unsigned tyre_threads = 1;
    int measurement_decimation = 1;
    RetentionMode record_retention = RetentionMode::off;
    // Legacy cruise target; the physical runtime never enables speed control.
    double speed_kph = 25.;
};

// setup.py validate_supported, on resolved values rather than filenames.
void validate_native_supported(const bike::configuration::SimulationPhysicsConfig&,
                               const bike::configuration::RiderSpecs&);
// Model-independent drive projection. Own/normalize the input, then use the
// same profile resolver and wire validation as project_writer_config. This
// retains initial gearing; live shifter state is never part of configuration.
[[nodiscard]] bike::io::Value project_drive_config(
    const bike::configuration::SimulationPhysicsConfig&);
// W05's model-independent authoring seam. resolved_fields contains exactly
// the C7.0 runtime root fields except runtime_schema and spatial, with an
// explicit schema3 writer envelope. Adds owned runtime2/spatial documents
// after structural validation. This is not a runnable RuntimeConfig: W17
// owns model-bound decoding/bootstrap2, and W25 owns the pneumatic3d writer.
// Geometry keeps the C7 outer field names only: supplied names/indices are
// unbound authoring data, and a non-null pose/rider object is not decoded here.
// W17 must apply the spatial nested schemas and resolve model bindings anew;
// no numeric index accepted by this seam is a portable topology identity.
[[nodiscard]] bike::io::Value project_spatial_runtime_envelope(
    const bike::configuration::SimulationPhysicsConfig&,
    const bike::io::Value& resolved_fields);
// W17 model-bound counterpart. W25 supplies the real resolved writer3 tyre
// payload; this seam binds names, fingerprints and the shared physical source
// without promoting the still-unqualified runtime capability.
[[nodiscard]] bike::io::Value project_spatial_writer_identity(
    const bike::model::BuildRequest&, const mjModel&);
[[nodiscard]] RuntimeConfig bind_spatial_runtime(
    const bike::model::BuildRequest&, const mjModel&,
    const bike::terrain::TrackSpec&, const bike::io::Value& resolved_fields);
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
