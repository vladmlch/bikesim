#pragma once
#include <optional>
#include "../io/value.hpp"
#include "../runtime/research_metrics.hpp"
#include "../runtime/sensors.hpp"
#include "../runtime/spatial_sensor_pipeline.hpp"
#include <mujoco/mujoco.h>

namespace bike::initialization {
struct SetupContext;
struct ResearchStartup {
    runtime::SensorObservation observation;
    std::optional<runtime::spatial::SensorObservationV2> observation_v2;
    runtime::TruthGeometry geometry;
    io::Value compiled_metadata;
    double position_m = 0.;
};
[[nodiscard]] ResearchStartup prepare_research_startup(SetupContext&);
// Read-only compiled solver/contact facts shared by ordinary and research
// recording. No force evaluation, integration or controller is constructed.
[[nodiscard]] io::Value compiled_physical_metadata(const mjModel&);
} // namespace bike::initialization
