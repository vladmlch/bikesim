#include "realtime_runner.hpp"
#include "setup_values.hpp"
#include "provenance.hpp"
#include "../app/prepare.hpp"
#include "../cli/application.hpp"
#include "../cli/physical_capture.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../legacy/controls.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../research/evidence.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace runtime::measurement {
namespace {
using namespace setupvalue;
namespace io = bike::io;
void write(const std::filesystem::path& path, const std::string& bytes) {
    bike::platform::write_exclusive(path, std::as_bytes(std::span(bytes)));
}
Value identity() {
    const auto runtime = runtime_identity();
    const auto build = bike::platform::build_identity();
    return Value{Object{{"backend", text("native")}, {"native_source_sha256", text(runtime.native_source_sha256)},
        {"execution_identity", text(bike::platform::execution_identity())}, {"executable_sha256", text(build.executable_sha256)},
        {"mujoco_version", text(runtime.mujoco_version)}, {"mujoco_library_sha256", text(bike::platform::file_sha256(runtime.mujoco_library_path))},
        {"build_context", io::parse_json(runtime.build_context_json)},
        {"source_identity_scope", text("compiled native source digest; no Python source fingerprint")}}};
}
class NativeDriver final : public Driver {
public:
    explicit NativeDriver(const Options& options, Clock clock) : clock_(std::move(clock)) {
        auto resolved = resolve_measurement_ride(options);
        auto& request = resolved->setup;
        auto prepared = bike::app::prepare(std::make_unique<bike::initialization::SetupRequest>(request));
        settings_.metadata = bike::cli::launch_metadata(request, *prepared, options.seed);
        set(settings_.metadata, "execution", identity());
        settings_.strict = request.runtime.strict;
        settings_.record_decimation = options.record_decimation;
        settings_.timestep_s = prepared->bootstrap.config.timestep_s;
        const double steps = std::ceil(options.duration_s / settings_.timestep_s);
        if (!std::isfinite(steps) || steps < 1. || steps >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            throw std::invalid_argument("duration/timestep produces an unrepresentable step count");
        settings_.max_steps = static_cast<std::int64_t>(steps);
        limits_ = bike::legacy::RunLimits::for_track(request.track.length_m, request.start_x_m,
            settings_.timestep_s);
        // Snapshot compiled vertices during setup, outside the measured clock.
        const auto assets = bike::cli::physical_assets(request, *prepared, settings_.metadata);
        for (const auto& asset : assets) if (asset.name == "terrain_vertices.npy") vertices_ = asset.content;
        if (vertices_.empty()) throw std::runtime_error("measurement setup did not produce compiled terrain vertices");
        runtime_ = bike::app::make_ride(std::move(prepared));
        runtime_->set_finish_position(limits_.finish_x_m);
        start();
    }
    [[nodiscard]] const Settings& settings() const override { return settings_; }
    [[nodiscard]] Boundary boundary() const override { const auto state = runtime_->snapshot(); return {state.step, state.time_s}; }
    [[nodiscard]] std::optional<std::string> reason() const override {
        if (terminal_) return terminal_;
        const auto state = runtime_->snapshot();
        // PhysicalRideDriver's frontend normalizes the native crash detail.
        // The complete outcome detail remains in the native snapshot.
        if (state.outcome) return state.outcome->starts_with("crash") ? "crash" : *state.outcome;
        if (state.step >= settings_.max_steps) return "duration_reached";
        return {};
    }
    void start() override { started_ = clock_(); }
    void advance(std::int64_t target, double budget) override {
        // Input rejection precedes termination, like PhysicalRideDriver.
        const auto before = runtime_->snapshot();
        if (target < before.step)
            throw std::invalid_argument("target_step must be at or after the committed step");
        if (!std::isfinite(budget) || budget < 0.)
            throw std::invalid_argument("wall_budget_s must be finite and nonnegative");
        if (reason()) return;
        // This whole-run bound is separate from each 8 ms compute slice.
        // Check before accepting any more physics, including zero-progress
        // slices; setup/export are excluded by measure_run's start() call.
        if (std::max(0., clock_() - started_) >= limits_.max_wall_clock_s) {
            terminal_ = "wall_clock_cap";
            return;
        }
        (void)runtime_->advance(std::min(target, settings_.max_steps), {}, 0., 0., budget);
        runtime_->release_samples();
    }
    void flush() override { runtime_->flush(); }
    [[nodiscard]] Value summary(std::string_view terminal) const override {
        return bike::cli::physical_summary(*runtime_, settings_.metadata, terminal);
    }
    [[nodiscard]] Value execution_identity() const override { return identity(); }
    void export_run(const std::filesystem::path& directory, std::string_view terminal) override {
        std::filesystem::create_directories(directory);
        write(directory / "summary.json", io::canonical_json(summary(terminal)) + '\n');
        bike::platform::write_exclusive(directory / "terrain_vertices.npy", vertices_);
        // Same retention-off measurement as the Python CLI. Emit the actual
        // retained column/index view; never enable recording just for export.
        const auto columns = runtime_->recorded_columns();
        std::ostringstream csv;
        std::size_t rows = 0;
        bool first = true;
        for (const auto& [name, values] : columns) {
            if (!first) csv << ',';
            csv << name;
            if (!first && rows != values.size()) throw std::runtime_error("measurement column sizes differ");
            first = false;
            rows = values.size();
        }
        if (columns.empty()) csv << "schema_version,time_s";
        csv << "\r\n";
        for (std::size_t row = 0; row < rows; ++row) {
            first = true;
            for (const auto& [name, values] : columns) {
                (void)name; if (!first) csv << ','; first = false;
                // NativePhysicalRecorder writes absent numeric cells empty.
                if (!std::isnan(values.at(row))) csv << bike::configuration::detail::repr_double(values.at(row));
            }
            csv << "\r\n";
        }
        write(directory / "telemetry.csv", csv.str());
        std::string intervals;
        for (const auto& sample : runtime_->recorded_intervals())
            intervals += io::canonical_json(bike::research::wire_value(Wire{sample->as_wire()})) + '\n';
        write(directory / "intervals.jsonl", intervals);
    }
    void close() override { runtime_->close(); }
private:
    Settings settings_;
    bike::legacy::RunLimits limits_;
    Clock clock_;
    double started_ = 0.;
    std::optional<std::string> terminal_;
    std::unique_ptr<NativeRideRuntime> runtime_;
    std::vector<std::byte> vertices_;
};
} // namespace
std::unique_ptr<bike::cli::ResolvedRide> resolve_measurement_ride(const Options& options) {
    // Same ride projection in strict and benchmark modes. Only the runtime
    // monitor stop policy changes after projection, as in ride_arguments().
    std::vector<std::string> values{"--backend", "native", "--headless", "--no-plots", "--physics-config", options.physics_config.string(),
        "--track", options.track, "--duration", bike::configuration::detail::repr_double(options.duration_s),
        "--decimate", std::to_string(options.record_decimation), "--check-policy", "diagnostic"};
    if (options.timestep_s) { values.push_back("--timestep"); values.push_back(bike::configuration::detail::repr_double(*options.timestep_s)); }
    if (options.seed) { values.push_back("--seed"); values.push_back(*options.seed); }
    std::vector<std::string_view> args; for (const auto& value : values) args.emplace_back(value);
    auto resolved = bike::cli::resolve_ride(bike::cli::parse_arguments("bike-ride", args));
    resolved->setup.runtime.strict = !options.benchmark;
    resolved->setup.runtime.record_retention = RetentionMode::off;
    return resolved;
}
std::unique_ptr<Driver> make_native_driver(const Options& options, const Clock& clock) {
    return std::make_unique<NativeDriver>(options, clock);
}
} // namespace runtime::measurement
