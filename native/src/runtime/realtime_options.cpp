#include "realtime_runner.hpp"
#include "setup_values.hpp"
#include "bike_measurement_paths.hpp"
#include "../platform/provenance.hpp"
#include "../research/environment.hpp"
#include <charconv>
#include <cmath>
#include <limits>
#include <memory>
#include <set>

namespace runtime::measurement {
namespace {
std::filesystem::path example(std::string_view relative) {
    auto local = std::filesystem::current_path() / relative;
    if (std::filesystem::is_regular_file(local)) return local;
    auto installed = bike::platform::install_prefix() / "share/bike_sim" / relative;
    if (std::filesystem::is_regular_file(installed)) return installed;
    const auto source = std::filesystem::path(build::source_root) / relative;
    return std::filesystem::is_regular_file(source) ? source : local;
}
int integer_option(std::string_view value, std::string_view name) { // NOLINT(bugprone-easily-swappable-parameters) call sites keep the declared parameter order.
    int result = 0;
    const auto *end = std::to_address(value.end());
    const auto parsed = std::from_chars(std::to_address(value.begin()), end, result);
    if (parsed.ec != std::errc{} || parsed.ptr != end)
        throw std::invalid_argument(std::string(name) + " requires an integer");
    return result;
}
double real_option(std::string_view value, std::string_view name) { // NOLINT(bugprone-easily-swappable-parameters) call sites keep the declared parameter order.
    double result = 0.;
    const auto *end = std::to_address(value.end());
    const auto parsed = std::from_chars(std::to_address(value.begin()), end, result);
    if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(result) || result <= 0.)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
    return result;
}
} // namespace
Options parse_options(std::span<const std::string_view> arguments) {
    Options out;
    std::set<std::string> seen;
    for (std::size_t i = 0; i < arguments.size(); ++i) {
        auto name = arguments[i];
        if (name == "--help" || name == "-h") { out.help = true; return out; }
        std::optional<std::string_view> inline_value;
        if (const auto split = name.find('='); split != name.npos) { inline_value = name.substr(split + 1); name = name.substr(0, split); }
        const auto canonical = name == "--decimate" ? "--record-decimation" : name;
        if (!seen.insert(std::string(canonical)).second) throw std::invalid_argument("duplicate option: " + std::string(name));
        if (name == "--benchmark") {
            if (inline_value) throw std::invalid_argument("--benchmark does not take a value");
            out.benchmark = true; continue;
        }
        if (name == "--no-region-timing") {
            if (inline_value) throw std::invalid_argument("--no-region-timing does not take a value");
            out.region_timing = false; continue;
        }
        const auto value = [&]() -> std::string_view {
            if (inline_value) return *inline_value;
            if (++i == arguments.size()) throw std::invalid_argument("missing value for " + std::string(name));
            return arguments[i];
        }();
        if (name == "--backend") { if (value != "native") throw std::invalid_argument("--backend requires native; Python is not an executable backend"); }
        else if (name == "--physics-config") out.physics_config = value;
        else if (name == "--track") out.track = value;
        else if (name == "--spatial") {
            if (value != "planar" && value != "3d") throw std::invalid_argument("--spatial requires planar or 3d");
            out.spatial_mode = value;
        }
        else if (name == "--terrain3d") {
            if (value.empty()) throw std::invalid_argument("--terrain3d requires a path");
            const std::filesystem::path path(value);
            if (path.extension() != ".toml" && path.extension() != ".terrain3d")
                throw std::invalid_argument("--terrain3d requires a .toml genspec or .terrain3d asset");
            out.terrain3d = path;
        }
        else if (name == "--profile") {
            if (value != "spatial-fast-v1") throw std::invalid_argument("--profile requires spatial-fast-v1");
            out.profile = value;
        }
        else if (name == "--threads") {
            const int threads = integer_option(value, name);
            if (threads < 1 || threads > 64) throw std::invalid_argument("--threads must be in [1,64]");
            out.tyre_threads = static_cast<unsigned>(threads);
        }
        else if (name == "--demand-trace") {
            if (value.empty()) throw std::invalid_argument("--demand-trace requires a W36 qualification TOML path");
            out.demand_trace = std::filesystem::path(value);
        }
        else if (name == "--out") out.output = value;
        else if (name == "--compare") out.compare = std::filesystem::path(value);
        else if (name == "--seed") out.seed = bike::research::validate_seed(value);
        else if (name == "--duration") out.duration_s = real_option(value, name);
        else if (name == "--dt") out.timestep_s = real_option(value, name);
        else if (name == "--runs") out.runs = integer_option(value, name);
        else if (name == "--warmup") out.warmups = integer_option(value, name);
        else if (canonical == "--record-decimation") out.record_decimation = integer_option(value, name);
        else throw std::invalid_argument("unknown option: " + std::string(name));
    }
    if (out.output.empty()) throw std::invalid_argument("--out is required and must name a directory");
    if (out.runs < 1 || out.record_decimation < 1 || out.warmups < 0)
        throw std::invalid_argument("--runs/--record-decimation must be positive and --warmup nonnegative");
    if (!out.profile.empty()) {
        if (out.spatial_mode && *out.spatial_mode == "planar")
            throw std::invalid_argument("spatial-fast-v1 conflicts with --spatial planar");
        out.spatial_mode = "3d";
        if (out.physics_config.empty()) out.physics_config = example("examples/spatial/performance_fast.toml");
        if (!out.demand_trace) out.demand_trace = example("examples/research/spatial_flat.toml");
        if (!out.seed) out.seed = "73421";
    }
    if (out.terrain3d && !out.track.empty()) throw std::invalid_argument("--terrain3d conflicts with --track");
    if (out.spatial_mode && *out.spatial_mode == "planar" && (out.terrain3d || out.tyre_threads != 1 || out.demand_trace || !out.region_timing))
        throw std::invalid_argument("spatial measurement options cannot be used with --spatial planar");
    const bool legacy_defaults = out.physics_config.empty() && (!out.spatial_mode || *out.spatial_mode == "planar") && !out.terrain3d;
    if (out.physics_config.empty()) out.physics_config = example(out.spatial_mode && *out.spatial_mode == "3d"
        ? "examples/spatial/free_fast.toml" : "examples/research/viewer_physics_welded.toml");
    // A spatial config may select its own terrain. Never inject the historical
    // planar track into it; resolve_measurement_ride selects the legacy default
    // only after the actual physics mode has been read.
    if (legacy_defaults && out.track.empty()) out.track = example("examples/research/rough_uphill_savage.toml").string();
    return out;
}
bike::io::Value annotate_report(bike::io::Value report, const Options& options, double effective_dt_s) {
    using namespace setupvalue;
    set(report, "invocation", Value{Object{{"physics_config", text(std::filesystem::absolute(options.physics_config).string())},
        {"track", text(options.track)}, {"duration_s", real(options.duration_s)},
        {"dt_override", options.timestep_s ? real(*options.timestep_s) : Value{}},
        {"effective_dt_s", real(effective_dt_s)}, {"record_decimation", integer(options.record_decimation)},
        {"monitor_strict", Value{!options.benchmark}}, {"benchmark_mode", Value{options.benchmark}},
        {"steady_state_minimum_simulated_time_s", real(options.duration_s)}}});
    set(report, "benchmark_mode", Value{options.benchmark});
    if (find(report, "profile") != nullptr) {
        auto invocation = field(report, "invocation");
        set(invocation, "spatial", text("3d"));
        set(invocation, "terrain3d", options.terrain3d ? text(std::filesystem::absolute(*options.terrain3d).string()) : Value{});
        set(invocation, "profile", text(options.profile));
        set(invocation, "tyre_threads", integer(options.tyre_threads));
        set(invocation, "region_timing", Value{options.region_timing});
        set(invocation, "demand_trace", options.demand_trace ? text(std::filesystem::absolute(*options.demand_trace).string()) : Value{});
        set(report, "invocation", std::move(invocation));
    }
    if (options.benchmark) set(report, "limitations", Value{Array{text("Benchmark mode: the reference monitor was non-strict; the measured prefix is a throughput sample of a ride whose validity is not established, never a valid-ride claim.")}});
    return report;
}
} // namespace runtime::measurement
