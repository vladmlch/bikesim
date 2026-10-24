#include "realtime_runner.hpp"
#include "setup_values.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../platform/files.hpp"
#include <iomanip>
#include <ostream>
#include <sstream>

namespace runtime::measurement {
namespace {
using namespace setupvalue;
namespace io = bike::io;
void write(const std::filesystem::path& path, const Value& value) {
    const auto data = io::canonical_json(value) + '\n';
    bike::platform::write_exclusive(path, std::as_bytes(std::span(data)));
}
Value read(const std::filesystem::path& path) {
    const auto bytes = io::read_numpy_bytes(path); std::string text; text.reserve(bytes.size());
    for (std::byte const byte : bytes) text.push_back(static_cast<char>(byte));
    return io::parse_json(text);
}
bool yes(const Value& value) { const auto* b = std::get_if<bool>(&value.data); return b != nullptr && *b; }
std::string run_name(std::string_view phase, int index) {
    std::ostringstream name; name << phase << '-' << std::setw(2) << std::setfill('0') << index; return name.str();
}
void require_empty_output(const std::filesystem::path& path) {
    const auto status = std::filesystem::symlink_status(path);
    if (std::filesystem::is_symlink(status) || (std::filesystem::exists(status) &&
        (!std::filesystem::is_directory(status) || !std::filesystem::is_empty(path))))
        throw std::invalid_argument("--out must be a new or empty directory");
}
Value previous_runs(const std::optional<std::filesystem::path>& path) {
    if (!path) return {};
    auto value = read(*path);
    if (!std::holds_alternative<Object>(value.data) || io::as_int64(field(value, "schema_version"), "schema_version") != 1 ||
        io::as_array(field(value, "runs"), "runs").empty()) throw std::invalid_argument("--compare does not contain a measurement schema-1 run set");
    return value;
}
} // namespace
int measure_realtime_command(std::span<const std::string_view> arguments, std::ostream& out, std::ostream& error) {
    try {
        const auto options = parse_options(arguments);
        if (options.help) {
            out << "bike-measure-realtime --out DIR [--backend native] [--physics-config FILE] [--track FILE|PRESET]\n"
                << "  [--duration SECONDS] [--dt SECONDS] [--warmup COUNT] [--runs COUNT]\n"
                << "  [--record-decimation COUNT|--decimate COUNT] [--seed INTEGER] [--compare report.json] [--benchmark]\n";
            out << "  [--profile spatial-fast-v1] [--spatial planar|3d] [--terrain3d FILE] [--threads 1..64]\n"
                << "  [--demand-trace W36_QUALIFICATION.toml] [--no-region-timing]\n"
                << "  spatial acceptance needs >=1 warmup and >=3 full runs >=30 s; timing-disabled runs are overhead controls\n";
            return 0;
        }
        require_empty_output(options.output);
        const auto previous = previous_runs(options.compare);
        Array warmups, runs;
        const auto provenance = native_provenance();
        bool all_measured = true;
        double effective_dt = 0.;
        for (const auto& [phase, count] : {std::pair{"warmup", options.warmups}, std::pair{"run", options.runs}}) {
            for (int i = 0; i < count; ++i) {
                const auto name = run_name(phase, i);
                auto run = measure_run([&] { return make_native_driver(options); }, options.output / name);
                effective_dt = io::as_double(field(field(run, "comparability"), "timestep_s"), "timestep_s");
                const bool measured_run = std::string_view(phase) == "run";
                if (find(run, "profile") != nullptr) set(run, "warmup", Value{!measured_run});
                if (measured_run) set(run, "steady_state", steady_state_evidence(run, options.duration_s));
                all_measured = all_measured && yes(field(run, "measured_ok"));
                std::filesystem::create_directories(options.output);
                write(options.output / (name + ".json"), run);
                out << phase << ' ' << i + 1 << '/' << count << ": backend=native; steps="
                    << io::as_int64(field(run, "physics_steps"), "physics_steps") << "; RTF="
                    << io::canonical_json(field(run, "real_time_factor")) << "; outcome="
                    << io::canonical_json(field(field(run, "prefix"), "reason")) << "; valid="
                    << io::canonical_json(field(run, "valid_prefix"));
                if (measured_run) {
                    const auto& steady = field(run, "steady_state");
                    out << "; steady_state=" << io::as_string(field(steady, "status"), "status");
                    const auto& reasons = io::as_array(field(steady, "reasons"), "reasons");
                    if (!reasons.empty()) {
                        out << " (";
                        for (std::size_t reason = 0; reason < reasons.size(); ++reason) {
                            if (reason != 0) out << ',';
                            out << io::as_string(reasons[reason], "reason");
                        }
                        out << ')';
                    }
                }
                out << '\n';
                out.flush();
                (measured_run ? runs : warmups).push_back(std::move(run));
            }
        }
        auto report = annotate_report(build_report(warmups, runs), options, effective_dt);
        set(report, "provenance", provenance);
        for (std::string_view const key : {"git_sha", "working_tree_dirty", "machine"}) set(report, key, field(provenance, key));
        set(report, "source_sha256", field(provenance, "native_source_sha256"));
        if (!io::is_null(previous)) {
            Array checks; bool comparable = true;
            for (const auto& left : io::as_array(field(previous, "runs"), "runs")) for (const auto& right : runs) {
                auto check = compare_runs(left, right); comparable = comparable && yes(field(check, "comparable")); checks.push_back(std::move(check));
            }
            const auto* prior_field = find(previous, "median_real_time_factor");
            const auto prior = prior_field != nullptr ? *prior_field : Value{};
            const auto& current = field(report, "median_real_time_factor");
            Value speedup;
            if (comparable && !io::is_null(prior) && !io::is_null(current) && io::as_double(prior, "median") != 0.)
                speedup = real(io::as_double(current, "median") / io::as_double(prior, "median"));
            set(report, "baseline_comparison", Value{Object{{"checks", Value{std::move(checks)}}, {"comparable", Value{comparable}}, {"median_speedup", std::move(speedup)}}});
        }
        write(options.output / "report.json", report);
        out << "median RTF=" << io::canonical_json(field(report, "median_real_time_factor")) << "; report=" << (options.output / "report.json") << '\n';
        return all_measured ? 0 : 1;
    } catch (const std::exception& caught) {
        error << "MEASUREMENT ERROR: " << caught.what() << '\n'; return 2;
    }
}
} // namespace runtime::measurement
