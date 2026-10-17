#pragma once
#include "../io/value.hpp"
#include "../runtime/research_metrics.hpp"
#include "../runtime/sensors.hpp"

namespace bike::initialization {
struct SetupContext;
struct ResearchStartup {
    runtime::SensorObservation observation;
    runtime::TruthGeometry geometry;
    io::Value compiled_metadata;
    double position_m = 0.;
};
[[nodiscard]] ResearchStartup prepare_research_startup(SetupContext&);
} // namespace bike::initialization
