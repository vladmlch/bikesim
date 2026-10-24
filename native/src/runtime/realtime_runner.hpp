#pragma once
#include "realtime_measurement.hpp"
#include <iosfwd>
#include <optional>
#include <string_view>

namespace bike::cli { struct ResolvedRide; }
namespace bike::initialization { struct SetupRequest; }
namespace runtime::measurement {
struct Options {
    std::filesystem::path physics_config, output;
    std::string track;
    std::optional<std::string> spatial_mode;
    std::optional<std::filesystem::path> terrain3d, demand_trace;
    std::string profile;
    unsigned tyre_threads = 1;
    bool region_timing = true;
    double duration_s = 30.;
    std::optional<double> timestep_s;
    int warmups = 1, runs = 3, record_decimation = 80;
    std::optional<std::string> seed;
    std::optional<std::filesystem::path> compare;
    bool benchmark = false, help = false;
};
[[nodiscard]] Options parse_options(std::span<const std::string_view>);
[[nodiscard]] std::unique_ptr<bike::cli::ResolvedRide> resolve_measurement_ride(const Options&);
// The injected steady clock is a cold test seam for the real driver's wall
// cap. Production uses the same monotonic source as outer measurement.
[[nodiscard]] std::unique_ptr<Driver> make_native_driver(const Options&,
    const Clock& clock = monotonic_clock);
// Cold direct fixture path: no argument parser or research session is invoked.
[[nodiscard]] std::unique_ptr<Driver> make_native_driver(
    std::unique_ptr<bike::initialization::SetupRequest>, const Options&,
    const Clock& clock = monotonic_clock);
[[nodiscard]] bike::io::Value native_build_provenance(const std::filesystem::path& build_directory);
[[nodiscard]] bike::io::Value native_provenance();
[[nodiscard]] bike::io::Value annotate_report(bike::io::Value, const Options&, double effective_dt_s);
int measure_realtime_command(std::span<const std::string_view>, std::ostream& out, std::ostream& error);
} // namespace runtime::measurement
