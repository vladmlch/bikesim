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
    const auto local = std::filesystem::current_path() / relative;
    if (std::filesystem::is_regular_file(local)) return local;
    const auto installed = bike::platform::install_prefix() / "share/bike_sim" / relative;
    if (std::filesystem::is_regular_file(installed)) return installed;
    const auto source = std::filesystem::path(build::source_root) / relative;
    return std::filesystem::is_regular_file(source) ? source : local;
}
int integer_option(std::string_view value, std::string_view name) {
    int result = 0;
    const auto *end = std::to_address(value.end());
    const auto parsed = std::from_chars(value.data(), end, result);
    if (parsed.ec != std::errc{} || parsed.ptr != end)
        throw std::invalid_argument(std::string(name) + " requires an integer");
    return result;
}
double real_option(std::string_view value, std::string_view name) {
    double result = 0.;
    const auto *end = std::to_address(value.end());
    const auto parsed = std::from_chars(value.data(), end, result);
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
        const auto value = [&]() -> std::string_view {
            if (inline_value) return *inline_value;
            if (++i == arguments.size()) throw std::invalid_argument("missing value for " + std::string(name));
            return arguments[i];
        }();
        if (name == "--backend") { if (value != "native") throw std::invalid_argument("--backend requires native; Python is not an executable backend"); }
        else if (name == "--physics-config") out.physics_config = value;
        else if (name == "--track") out.track = value;
        else if (name == "--out") out.output = value;
        else if (name == "--compare") out.compare = std::filesystem::path(value);
        else if (name == "--seed") out.seed = bike::research::validate_seed(std::string(value));
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
    if (out.physics_config.empty()) out.physics_config = example("examples/research/viewer_physics_welded.toml");
    if (out.track.empty()) out.track = example("examples/research/rough_uphill_savage.toml").string();
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
    if (options.benchmark) set(report, "limitations", Value{Array{text("Benchmark mode: the reference monitor was non-strict; the measured prefix is a throughput sample of a ride whose validity is not established, never a valid-ride claim.")}});
    return report;
}
} // namespace runtime::measurement
