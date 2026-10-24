#include "realtime_runner.hpp"
#include "setup_values.hpp"
#include "provenance.hpp"
#include "spatial_measurement.hpp"
#include "spatial_timing.hpp"
#include "../app/prepare.hpp"
#include "../cli/application.hpp"
#include "../cli/physical_capture.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../io/hash.hpp"
#include "../configuration/load.hpp"
#include "../terrain/source3d.hpp"
#include "../legacy/controls.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../research/evidence.hpp"
#include <algorithm>
#include <array>
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
        {"provider_identity", text(numerics_provider_policy_json())},
        {"source_identity_scope", text("compiled native source digest; no Python source fingerprint")}}};
}
struct Demand {
    std::int64_t step = 0;
    double torque_fraction = 0., brake_fraction = 0.;
};
std::vector<Demand> read_demands(const std::filesystem::path& path, double dt) {
    const auto document = bike::configuration::detail::parse_toml_file(path);
    if (io::as_int64(field(document, "qualification_schema"), "qualification_schema") != 1)
        throw std::invalid_argument("measurement demand trace requires W36 qualification_schema=1");
    std::vector<Demand> result;
    for (const auto& row : io::as_array(field(field(document, "demand_trace"), "points"), "demand_trace.points")) {
        const auto& values = io::as_array(row, "demand_trace.points[]");
        if (values.size() != 3) throw std::invalid_argument("demand_trace row must be [time_s,torque_fraction,brake_fraction]");
        const double time = io::as_double(values[0], "demand_trace.time_s");
        const double torque = io::as_double(values[1], "demand_trace.torque_fraction");
        const double brake = io::as_double(values[2], "demand_trace.brake_fraction");
        const double index = time / dt;
        if (!std::isfinite(time) || time < 0. || !std::isfinite(index) ||
            index >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) ||
            !std::isfinite(torque) || torque < 0. || torque > 1. ||
            !std::isfinite(brake) || brake < 0. || brake > 1.)
            throw std::invalid_argument("demand_trace: invalid time or demand fraction");
        const auto step = static_cast<std::int64_t>(std::llround(index));
        if (std::abs(index - static_cast<double>(step)) > 1e-9 || (!result.empty() && step <= result.back().step))
            throw std::invalid_argument("demand_trace times must be increasing physics-step boundaries");
        result.push_back({step, torque, brake});
    }
    if (result.empty() || result.front().step != 0)
        throw std::invalid_argument("demand_trace must be nonempty and start at zero");
    return result;
}
class NativeDriver final : public Driver {
public:
    NativeDriver(std::unique_ptr<bike::initialization::SetupRequest> owner,
                 const Options& options, Clock clock) : clock_(std::move(clock)) {
        if (!owner || !clock_) throw std::invalid_argument("measurement driver requires an owned setup and clock");
        auto& request = *owner;
        if (!request.model.physics) throw std::invalid_argument("measurement setup requires physical configuration");
        request.runtime.strict = !options.benchmark;
        request.runtime.record_retention = RetentionMode::off;
        request.runtime.measurement_decimation = options.record_decimation;
        request.runtime.async_telemetry = false;
        request.runtime.tyre_threads = options.tyre_threads;
        auto prepared = bike::app::prepare(std::make_unique<bike::initialization::SetupRequest>(request));
        settings_.metadata = bike::cli::launch_metadata(request, *prepared, options.seed);
        set(settings_.metadata, "execution", identity());
        settings_.strict = request.runtime.strict;
        settings_.record_decimation = options.record_decimation;
        settings_.timestep_s = prepared->bootstrap.config.timestep_s;
        if (!std::isfinite(options.duration_s) || options.duration_s <= 0. ||
            !std::isfinite(settings_.timestep_s) || settings_.timestep_s <= 0.)
            throw std::invalid_argument("measurement duration/timestep must be finite and positive");
        const double steps = std::ceil(options.duration_s / settings_.timestep_s);
        if (!std::isfinite(steps) || steps < 1. || steps >= static_cast<double>(std::numeric_limits<std::int64_t>::max()))
            throw std::invalid_argument("duration/timestep produces an unrepresentable step count");
        settings_.max_steps = static_cast<std::int64_t>(steps);
        spatial_ = prepared->bootstrap.config.spatial.has_value();
        if (spatial_) {
            const auto& config = prepared->bootstrap.config;
            const auto& spatial_config = *config.spatial;
            if (!config.terrain3d || !prepared->bootstrap.state.spatial)
                throw std::invalid_argument("spatial measurement requires resolved terrain and bootstrap identity");
            spatial::SpatialPerformanceProfile profile;
            profile.id = options.profile.empty() ? "spatial-custom-v1" : options.profile;
            profile.root = spatial_config.root; profile.tier = spatial_config.tyre.tier;
            profile.dt_s = settings_.timestep_s;
            if (const auto* grid = config.terrain3d->grid()) { profile.grid_nx = grid->nx(); profile.grid_ny = grid->ny(); }
            profile.sensor_mode = "mounted-truth-only; research sensor pipeline absent";
            profile.recording_mode = "retention-off; no container writer";
            profile.tyre_threads = options.tyre_threads;
            profile.source_sha = io::as_string(field(field(settings_.metadata, "execution"), "native_source_sha256"), "source_sha");
            profile.config_sha256 = io::as_string(field(settings_.metadata, "configuration_sha256"), "configuration_sha256");
            profile.calibration_sha256 = prepared->bootstrap.state.spatial->calibration_sha256;
            profile.provider_identity = numerics_provider_policy_json();
            (void)spatial_profile_value(profile);
            settings_.spatial_profile = std::move(profile);
            set(settings_.metadata, "terrain_kind", text(config.terrain3d->kind() == bike::terrain::Terrain3dKind::grid ? "grid" : "profile_extrusion"));
            set(settings_.metadata, "measurement_limits", Value{Object{
                {"completion_step", integer(settings_.max_steps)}, {"corridor_half_width_m", real(spatial_config.corridor_half_width_m)},
                {"terrain_x_min_m", real(config.terrain3d->x_min_m())}, {"terrain_x_max_m", real(config.terrain3d->x_max_m())},
                {"termination", text("W33 outcomes and exact requested step; no planar finish_x")}}});
            if (options.demand_trace) {
                demands_ = read_demands(*options.demand_trace, settings_.timestep_s);
                set(settings_.metadata, "measurement_demand_sha256", text(bike::platform::file_sha256(*options.demand_trace)));
                set(settings_.metadata, "measurement_controls", text("W36 piecewise demand: motor=85*fraction Nm, motor_limit=85 Nm, human=0, symmetric brake"));
            } else set(settings_.metadata, "measurement_controls", text("default RideControl, no brake demand"));
            timing_enabled_ = options.region_timing;
            set(settings_.metadata, "region_timing", Value{timing_enabled_});
            if (timing_enabled_) timings_.calibrate();
        } else {
            if (!options.profile.empty() || options.terrain3d || options.demand_trace || options.tyre_threads != 1 || !options.region_timing)
                throw std::invalid_argument("spatial measurement options require a spatial setup");
            limits_ = bike::legacy::RunLimits::for_track(request.track.length_m, request.start_x_m, settings_.timestep_s);
            // Legacy export contract remains the compiled planar vertices.
            const auto assets = bike::cli::physical_assets(request, *prepared, settings_.metadata);
            for (const auto& asset : assets) if (asset.name == "terrain_vertices.npy") vertices_ = asset.content;
            if (vertices_.empty()) throw std::runtime_error("measurement setup did not produce compiled terrain vertices");
        }
        runtime_ = bike::app::make_ride(std::move(prepared));
        if (spatial_) runtime_->set_completion_step(settings_.max_steps);
        else runtime_->set_finish_position(limits_.finish_x_m);
        start();
    }
    [[nodiscard]] const Settings& settings() const override { return settings_; }
    [[nodiscard]] Boundary boundary() const override { const auto state = runtime_->snapshot(); return {.step=state.step, .time_s=state.time_s}; }
    [[nodiscard]] std::optional<std::string> reason() const override {
        if (terminal_) return terminal_;
        const auto state = runtime_->snapshot();
        // PhysicalRideDriver's frontend normalizes the native crash detail.
        // The complete outcome detail remains in the native snapshot.
        if (state.outcome) return state.outcome->starts_with("crash") ? "crash" : *state.outcome;
        if (state.step >= settings_.max_steps) return "duration_reached";
        return {};
    }
    void start() override {
        started_ = clock_();
        if (!std::isfinite(started_)) throw std::invalid_argument("measurement start clock must be finite");
    }
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
        const double elapsed = clock_() - started_;
        if (!std::isfinite(elapsed) || elapsed < 0.) throw std::invalid_argument("measurement clock must be finite and monotonic");
        if (elapsed >= limits_.max_wall_clock_s) {
            terminal_ = "wall_clock_cap";
            return;
        }
        RideControl control;
        double brake = 0.;
        auto until = std::min(target, settings_.max_steps);
        if (!demands_.empty()) {
            while (demand_index_ + 1 < demands_.size() && demands_[demand_index_ + 1].step <= before.step) ++demand_index_;
            const auto& demand = demands_[demand_index_];
            control.motor_torque_nm = 85. * demand.torque_fraction;
            control.motor_limit_nm = 85.; control.human_torque_nm = 0.; brake = demand.brake_fraction;
            if (demand_index_ + 1 < demands_.size()) until = std::min(until, demands_[demand_index_ + 1].step);
        }
        const spatial::timing::Binding timer(timing_enabled_ ? &timings_ : nullptr);
        (void)runtime_->advance(until, control, brake, brake, budget);
        runtime_->release_samples();
    }
    void flush() override {
        const spatial::timing::Binding timer(timing_enabled_ ? &timings_ : nullptr);
        runtime_->flush();
    }
    [[nodiscard]] Value summary(std::string_view terminal) const override {
        return bike::cli::physical_summary(*runtime_, settings_.metadata, terminal);
    }
    [[nodiscard]] Value execution_identity() const override { return identity(); }
    [[nodiscard]] MeasurementTimings timings() const override {
        auto regions = timings_.regions();
        // Raw mounted IMU is timed with the physical step; this fixture does
        // not run the research delivery/noise pipeline or a recorder/UI.
        regions.sensors_pending = true; regions.recording_pending = true; regions.ui_pending = true;
        return {.regions = regions,
            .clock_overhead_estimate_s = static_cast<double>(timings_.clock_reads()) * timings_.clock_read_cost_s,
            .clock_reads = timings_.clock_reads(),
            .available = timing_enabled_ && timings_.valid && timings_.clock_reads() != 0};
    }
    void export_run(const std::filesystem::path& directory, std::string_view terminal) override {
        std::filesystem::create_directories(directory);
        write(directory / "summary.json", io::canonical_json(summary(terminal)) + '\n');
        if (spatial_) write(directory / "spatial_profile.json", io::canonical_json(spatial_profile_value(*settings_.spatial_profile)) + '\n');
        else bike::platform::write_exclusive(directory / "terrain_vertices.npy", vertices_);
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
    bool spatial_ = false, timing_enabled_ = false;
    spatial::timing::Collector timings_;
    std::vector<Demand> demands_;
    std::size_t demand_index_ = 0;
};
} // namespace
std::unique_ptr<bike::cli::ResolvedRide> resolve_measurement_ride(const Options& options) {
    // Same ride projection in strict and benchmark modes. Only the runtime
    // monitor stop policy changes after projection, as in ride_arguments().
    std::vector<std::string> values{"--backend", "native", "--headless", "--no-plots", "--physics-config", options.physics_config.string(),
        "--duration", bike::configuration::detail::repr_double(options.duration_s),
        "--decimate", std::to_string(options.record_decimation), "--check-policy", "diagnostic"};
    if (options.spatial_mode) { values.emplace_back("--spatial"); values.push_back(*options.spatial_mode); }
    if (options.terrain3d) { values.emplace_back("--terrain3d"); values.push_back(options.terrain3d->string()); }
    if (!options.track.empty()) { values.emplace_back("--track"); values.push_back(options.track); }
    else if (!options.terrain3d) {
        const auto physics = std::make_unique<bike::configuration::SimulationPhysicsConfig>(bike::configuration::load_physics(options.physics_config));
        const bool spatial = options.spatial_mode ? *options.spatial_mode == "3d" : physics->spatial.has_value();
        if (!spatial) {
            // Resolve the same relocatable default used by parse_options.
            auto defaults = parse_options(std::array<std::string_view, 2>{"--out", "unused"});
            values.emplace_back("--track"); values.push_back(defaults.track);
        }
    }
    if (!options.profile.empty()) { values.emplace_back("--rider"); values.emplace_back("articulated_planar"); }
    if (options.tyre_threads != 1) { values.emplace_back("--threads"); values.push_back(std::to_string(options.tyre_threads)); }
    if (options.timestep_s) { values.emplace_back("--timestep"); values.push_back(bike::configuration::detail::repr_double(*options.timestep_s)); }
    if (options.seed) { values.emplace_back("--seed"); values.push_back(*options.seed); }
    std::vector<std::string_view> args; args.reserve(values.size());
for (const auto& value : values) args.emplace_back(value);
    auto resolved = bike::cli::resolve_ride(bike::cli::parse_arguments("bike-ride", args));
    resolved->setup.runtime.strict = !options.benchmark;
    resolved->setup.runtime.record_retention = RetentionMode::off;
    if (!options.profile.empty()) resolved->setup.start_x_m = 3.;
    return resolved;
}
std::unique_ptr<Driver> make_native_driver(const Options& options, const Clock& clock) {
    auto resolved = resolve_measurement_ride(options);
    return make_native_driver(std::make_unique<bike::initialization::SetupRequest>(std::move(resolved->setup)), options, clock);
}
std::unique_ptr<Driver> make_native_driver(std::unique_ptr<bike::initialization::SetupRequest> request,
    const Options& options, const Clock& clock) {
    return std::make_unique<NativeDriver>(std::move(request), options, clock);
}
} // namespace runtime::measurement
