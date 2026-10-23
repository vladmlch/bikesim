#pragma once

#include "../spatial/types.hpp"

#include <cstdint>
#include <string>

namespace runtime::spatial {

bool rtf_threshold_met(double simulated_s, double wall_s, double target);

struct SpatialPerformanceProfile {
    std::string id = "spatial-fast-v1";
    bike::spatial::RootKind root;
    bike::spatial::TyreTier tier;
    std::uint64_t grid_nx = 0, grid_ny = 0;
    double dt_s = 0.;
    std::string sensor_mode, recording_mode;
    unsigned processes = 1, tyre_threads = 1, engine_threads = 1;
    std::string source_sha, config_sha256, calibration_sha256, provider_identity;
};

struct RegionTimings {
    double terrain_rays_s = 0., tyre_elements_s = 0., engine_step_s = 0.;
    double rider_steer_s = 0., accounting_s = 0., sensors_s = 0.;
    double recording_s = 0., ui_s = 0.;
    bool sensors_pending = true, recording_pending = true, ui_pending = true;
};

} // namespace runtime::spatial
