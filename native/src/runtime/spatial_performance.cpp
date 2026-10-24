#include "spatial_measurement.hpp"
#include "setup_values.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace runtime::spatial {
bool rtf_threshold_met(double simulated_s, double wall_s, double target) {
    if (!std::isfinite(simulated_s) || simulated_s < 0. ||
        !std::isfinite(wall_s) || wall_s <= 0. || !std::isfinite(target) || target <= 0.)
        throw std::invalid_argument("spatial.performance: finite sim>=0, wall>0 and target>0 required");
    return simulated_s / wall_s >= target;
}
} // namespace runtime::spatial

namespace runtime::measurement {
using namespace setupvalue;
namespace {
void required_text(const std::string& value, std::string_view name) {
    if (value.empty() || std::ranges::all_of(value, [](char c) { return c == ' ' || c == '\t' || c == '\n'; }))
        throw std::invalid_argument("spatial.profile." + std::string(name) + ": nonempty identity required");
}
}
bike::io::Value spatial_profile_value(const spatial::SpatialPerformanceProfile& p) {
    for (const auto& [name, value] : std::array{
        std::pair{"id", &p.id}, std::pair{"source_sha", &p.source_sha},
        std::pair{"config_sha256", &p.config_sha256}, std::pair{"calibration_sha256", &p.calibration_sha256},
        std::pair{"provider_identity", &p.provider_identity}, std::pair{"sensor_mode", &p.sensor_mode},
        std::pair{"recording_mode", &p.recording_mode}}) required_text(*value, name);
    if (!std::isfinite(p.dt_s) || p.dt_s <= 0. || p.processes == 0 ||
        p.tyre_threads == 0 || p.tyre_threads > 64 || p.engine_threads == 0 ||
        ((p.grid_nx == 0) != (p.grid_ny == 0)))
        throw std::invalid_argument("spatial.profile: invalid timestep, grid shape or execution counts");
    if (p.root != bike::spatial::RootKind::free6 && p.root != bike::spatial::RootKind::constrained4 &&
        p.root != bike::spatial::RootKind::planar3)
        throw std::invalid_argument("spatial.profile.root: invalid root kind");
    if (p.tier != bike::spatial::TyreTier::fast && p.tier != bike::spatial::TyreTier::detailed)
        throw std::invalid_argument("spatial.profile.tier: invalid tyre tier");
    if (p.id == "spatial-fast-v1" && (p.tier != bike::spatial::TyreTier::fast || p.root != bike::spatial::RootKind::free6))
        throw std::invalid_argument("spatial-fast-v1 requires free root and fast tyres");
    return Value{Object{{"id", text(p.id)}, {"root", text(bike::spatial::to_string(p.root))},
        {"tier", text(bike::spatial::to_string(p.tier))}, {"grid_nx", Value{bike::io::Number{std::to_string(p.grid_nx)}}},
        {"grid_ny", Value{bike::io::Number{std::to_string(p.grid_ny)}}}, {"dt_s", real(p.dt_s)},
        {"sensor_mode", text(p.sensor_mode)}, {"recording_mode", text(p.recording_mode)},
        {"processes", integer(p.processes)}, {"tyre_threads", integer(p.tyre_threads)},
        {"engine_threads", integer(p.engine_threads)}, {"source_sha", text(p.source_sha)},
        {"config_sha256", text(p.config_sha256)}, {"calibration_sha256", text(p.calibration_sha256)},
        {"provider_identity", text(p.provider_identity)}}};
}
bike::io::Value region_timings_value(const spatial::RegionTimings& r) {
    Object out;
    for (const auto& [name, value] : std::array{
        std::pair{"terrain_rays_s", r.terrain_rays_s}, std::pair{"tyre_elements_s", r.tyre_elements_s},
        std::pair{"engine_step_s", r.engine_step_s}, std::pair{"rider_steer_s", r.rider_steer_s},
        std::pair{"accounting_s", r.accounting_s}, std::pair{"sensors_s", r.sensors_s},
        std::pair{"recording_s", r.recording_s}, std::pair{"ui_s", r.ui_s}}) {
        if (!std::isfinite(value) || value < 0.)
            throw std::invalid_argument("spatial.regions." + std::string(name) + ": finite nonnegative time required");
        out.emplace_back(name, real(value));
    }
    out.emplace_back("sensors_pending", Value{r.sensors_pending});
    out.emplace_back("recording_pending", Value{r.recording_pending});
    out.emplace_back("ui_pending", Value{r.ui_pending});
    return Value{std::move(out)};
}
} // namespace runtime::measurement
