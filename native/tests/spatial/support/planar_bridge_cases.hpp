#pragma once

// Private W18 test assembly. Every force, state and retained interval below
// comes from the production setup/runtime and W25 writer. There is deliberately
// no reduced tyre, fabricated bootstrap or capability-gate bypass here.
#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "fixture_paths.hpp"
#include "initialization/setup_owner.hpp"
#include "io/json.hpp"
#include "model_access.hpp"
#include "platform/provenance.hpp"
#include "pneumatic3d/state.hpp"
#include "runtime/runtime.hpp"
#include "terrain/spec_io.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/pneumatic_3d_tire.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace spatial_tests::bridge {
using bike::io::Value;
using Object = Value::Object;
using Array = Value::Array;
using bike::validation::RuntimeRigAccess;
namespace wire = runtime::sample_wire;

inline const Value& member(const Value& value, std::string_view name) {
    const auto* found = bike::io::find_member(bike::io::as_object(value, "bridge"), name);
    if (!found) throw std::invalid_argument("bridge manifest is missing " + std::string(name));
    return *found;
}
inline double real(const Value& value, std::string_view name) {
    return bike::io::as_double(member(value, name), "bridge." + std::string(name));
}
inline std::string string(const Value& value, std::string_view name) {
    return bike::io::as_string(member(value, name), "bridge." + std::string(name));
}
inline Value number(double value) { return bike::terrain::detail::number(value); }
inline Value integer(std::int64_t value) { return bike::terrain::detail::integer(value); }
inline std::string spatial_bytes(const runtime::RuntimeSnapshot& snapshot) {
    return snapshot.spatial ? bike::io::canonical_json(runtime::encode_spatial_bootstrap(*snapshot.spatial)) : "null";
}
inline bool same_doubles(std::span<const double> a, std::span<const double> b) {
    return std::ranges::equal(std::as_bytes(a), std::as_bytes(b));
}
inline void same_boundary(const runtime::RuntimeSnapshot& a,
                          const runtime::RuntimeSnapshot& b, bool generation = true) {
    bt::require(a.model_sha256 == b.model_sha256, "probe/restore/reset preserves model identity");
    bt::require(same_doubles(a.integration_state, b.integration_state), "complete engine integration bytes agree");
    bt::require(a.tire_state_bytes == b.tire_state_bytes && !a.tire_state_bytes.empty(),
                "real committed tyre state bytes agree and are present");
    bt::require(spatial_bytes(a) == spatial_bytes(b), "hold/brake/steer/outcome/phase causal inventory agrees");
    bt::require(a.step == b.step && a.time_s == b.time_s && a.outcome == b.outcome,
                "committed step/time/outcome agree");
    if (generation) bt::require(a.generation == b.generation, "generation agrees");
}

struct Demand {
    std::int64_t step = 0;
    double human_nm = 0., motor_nm = 0., front_brake = 0., rear_brake = 0.;
    runtime::RideControl control() const {
        runtime::RideControl result;
        result.human_torque_nm = human_nm;
        result.motor_torque_nm = motor_nm;
        return result;
    }
};
struct Recipe {
    Value manifest;
    double dt_s = 0., control_s = 0., speed_mps = 0., start_x_m = 0.;
    double persistence_s = 0., initial_tolerance = 0.;
    std::string seed;
    std::vector<Demand> demands;
};
inline Recipe read_recipe() {
    Recipe recipe;
    recipe.manifest = spatial_tests::json(spatial_tests::fixture("bridge/manifest.json"));
    bt::require(bike::io::as_int64(member(recipe.manifest, "schema"), "bridge.schema") == 1,
                "bridge fixture schema is explicit");
    const auto& config = member(recipe.manifest, "paired_request");
    recipe.dt_s = real(config, "timestep_s");
    recipe.control_s = real(config, "control_period_s");
    recipe.speed_mps = real(config, "initial_speed_mps");
    recipe.start_x_m = real(config, "start_x_m");
    recipe.persistence_s = real(config, "grounded_hold_s");
    recipe.initial_tolerance = real(config, "initial_state_tolerance");
    recipe.seed = string(config, "seed_decimal");
    bt::require(recipe.dt_s > 0. && recipe.control_s >= recipe.dt_s && recipe.persistence_s > 0.,
                "manifest contains positive physical/control/persistence timing");
    bt::require(recipe.initial_tolerance >= 0. && recipe.initial_tolerance <= 1e-12,
                "initial-state equality precondition cannot be relaxed into a dynamics tolerance");
    for (const auto& entry : bike::io::as_array(member(recipe.manifest, "demand_trace"), "bridge.demand_trace")) {
        Demand demand;
        demand.step = bike::io::as_int64(member(entry, "step"), "bridge.demand_trace.step");
        demand.human_nm = real(entry, "human_torque_nm");
        demand.motor_nm = real(entry, "motor_torque_nm");
        demand.front_brake = real(entry, "front_brake");
        demand.rear_brake = real(entry, "rear_brake");
        bt::require(demand.step >= 0 && (recipe.demands.empty() || demand.step > recipe.demands.back().step),
                    "authored demand trace has strictly increasing nonnegative step keys");
        bt::require(demand.front_brake >= 0. && demand.front_brake <= 1. &&
                    demand.rear_brake >= 0. && demand.rear_brake <= 1., "authored brake demand domain");
        recipe.demands.push_back(demand);
    }
    bt::require(!recipe.demands.empty() && recipe.demands.front().step == 0, "trace covers t=0");
    return recipe;
}
inline const Demand& at(const Recipe& recipe, std::int64_t step) {
    const auto next = std::ranges::upper_bound(recipe.demands, step, {}, &Demand::step);
    if (next == recipe.demands.begin()) throw std::invalid_argument("demand trace does not cover step");
    return *std::prev(next);
}
inline std::filesystem::path track_path(const Value& entry) {
    const std::filesystem::path relative(string(entry, "track"));
    if (relative.empty() || relative.is_absolute()) throw std::invalid_argument("bridge.track must be repository relative");
    for (const auto& part : relative)
        if (part == ".." || part == ".") throw std::invalid_argument("bridge.track cannot traverse directories");
    return std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT) / relative;
}

struct JointState { double position = 0., velocity = 0.; };
struct InitialObservation {
    double weight_n = 0.;
    double position_m = 0.;
    std::map<std::string, JointState> joints;
};
inline InitialObservation observe_initial(runtime::NativeRideRuntime& ride) {
    InitialObservation observation;
    observation.position_m = ride.render_frame().presentation.position_m;
    RuntimeRigAccess::inspect(ride, [&](const Stepper& stepper, const runtime::PhysicalStep&) {
        const auto* model = stepper.model();
        double mass = 0.;
        for (const double body_mass : model_access::readonly_buffer(model->body_mass, model->nbody)) mass += body_mass;
        observation.weight_n = mass * std::hypot(model->opt.gravity[0], model->opt.gravity[1], model->opt.gravity[2]);
        const auto types = model_access::readonly_buffer(model->jnt_type, model->njnt);
        const auto qadr = model_access::readonly_buffer(model->jnt_qposadr, model->njnt);
        const auto vadr = model_access::readonly_buffer(model->jnt_dofadr, model->njnt);
        for (mjtSize joint = 0; joint < model->njnt; ++joint) {
            const auto index = static_cast<std::size_t>(joint);
            bt::require(types[index] == mjJNT_HINGE || types[index] == mjJNT_SLIDE,
                        "paired constrained/planar fixture uses scalar joints");
            const char* name = mj_id2name(model, mjOBJ_JOINT, static_cast<int>(joint));
            bt::require(name != nullptr, "paired physical joints have names");
            const JointState state{stepper.qpos()[static_cast<std::size_t>(qadr[index])],
                                   stepper.qvel()[static_cast<std::size_t>(vadr[index])]};
            bt::require(std::isfinite(state.position) && std::isfinite(state.velocity), "finite physical initial state");
            observation.joints.emplace(name, state);
        }
    });
    bt::require(std::isfinite(observation.weight_n) && observation.weight_n > 0., "weight is measured from compiled model mass and gravity");
    return observation;
}
struct Fixture {
    std::unique_ptr<bike::initialization::SetupRequest> request;
    std::unique_ptr<runtime::RuntimeBootstrap> bootstrap;
    std::unique_ptr<runtime::NativeRideRuntime> ride;
    bike::initialization::EquilibriumResult equilibrium;
    InitialObservation initial;
};

// Copies one complete planar physical request before changing only the spatial
// envelope/backend. Longitudinal source parameters, seeds and initial inputs
// are shared; neither fixture is tuned from the other's observed results.
[[gnu::noinline]] inline std::unique_ptr<bike::initialization::SetupRequest>
make_request(const Recipe& recipe, const Value& entry, bool spatial, double speed) {
    auto request = std::make_unique<bike::initialization::SetupRequest>();
    request->track = bike::initialization::load_track_for_setup(track_path(entry), recipe.seed);
    request->seed_decimal = recipe.seed;
    request->start_x_m = recipe.start_x_m;
    request->asset_root = std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT);
    request->origin = "W18:" + string(entry, "id");
    request->model.mode = "ride";
    request->model.rider.emplace();
    request->model.rider->variant = "none";
    request->model.physics.emplace();
    auto& physics = *request->model.physics;
    physics.physics_mode = "physical";
    physics.drive_mode = "crank_effort";
    physics.pitch_assist = false;
    physics.timestep_s = recipe.dt_s;
    physics.control_period_s = recipe.control_s;
    physics.initial_speed_mps = speed;
    physics.initial_front_brake = 0.;
    physics.initial_rear_brake = 0.;
    physics.tires.backend = "pneumatic";
    physics.tires.surface_mode = "track";
    physics.tires.pneumatic.tier = "fast";
    physics.tires.pneumatic.surface.reset();
    physics.drive.transmission_model = "ideal_mid_drive";
    physics.drive.shifting.enabled = false;
    physics.drive.human_torque_nm = 0.;
    physics.drive.assist.gain = 0.;
    physics.resistance.crr = 0.;
    physics.resistance.cda_m2 = 0.;
    physics.equilibrium_cache_enabled = false;
    physics.equilibrium_state_file.reset();
    request->model.field = bike::terrain::HeightFieldSpec::for_track(request->track);
    request->runtime.record_retention = runtime::RetentionMode::full;
    request->runtime.measurement_decimation = 1;
    request->runtime.async_telemetry = false;
    request->runtime.speed_kph = speed * 3.6;
    request->equilibrium.tolerance = 1e-8;
    if (spatial) {
        physics.tires.backend = "pneumatic_3d";
        physics.spatial.emplace();
        auto& config = *physics.spatial;
        config.mode = bike::spatial::Mode::spatial3d;
        config.root = bike::spatial::RootKind::constrained4;
        config.steer.controller = bike::spatial::SteerController::off;
        config.terrain.kind = bike::spatial::TerrainRequestKind::profile_extrusion;
        config.terrain.path = track_path(entry);
        config.terrain.dx_m = bike::terrain::DEFAULT_RESOLUTION_M;
        config.tyre.tier = bike::spatial::TyreTier::fast;
        request->model.spatial = config;
        request->model.terrain3d = bike::terrain::detail::load_terrain3d_source(config.terrain);
        request->model.field.nrow = 5;
        request->model.field.ncol = 2049;
    }
    bike::configuration::post_init(physics);
    return request;
}

inline void inspect_real_writer(Fixture& fixture) {
    RuntimeRigAccess::inspect(*fixture.ride, [&](const Stepper& stepper, const runtime::PhysicalStep&) {
        bt::require(stepper.has_spatial_tire(), "constrained bridge owns the real W25 spatial writer");
        const auto& writer = stepper.spatial_tire_writer();
        const auto& spec = writer.model().spec();
        bt::require(spec.tier == bike::spatial::TyreTier::fast, "bridge uses the declared fast tyre tier");
        const auto& input = fixture.request->model.physics->tires.pneumatic;
        const std::array<const bike::configuration::TyreSpecs*, 2> tyres{&input.front, &input.rear};
        for (std::size_t side = 0; side < tyres.size(); ++side) {
            bt::require(spec.outer_radius_m[side] == tyres[side]->outer_radius_mm / 1000. &&
                        spec.casing_width_m[side] == tyres[side]->casing_width_mm / 1000., "actual writer uses the same physical tyre geometry");
            bt::require(spec.carcass[side].pressure_pa == tyres[side]->pressure_pa(), "actual writer uses the same pressure");
            bt::require(spec.brush[side].relaxation_x_m == tyres[side]->relaxation_length_mm / 1000.,
                        "actual writer preserves the planar longitudinal relaxation law input");
        }
        bt::require(writer.state().identity.terrain_sha256_hex == fixture.request->model.terrain3d->fingerprint_hex(),
                    "real tyre binds the exact same original extrusion owner");
        bt::require(!writer.state_bytes().empty(), "no empty stand-in material state");
    });
}
[[gnu::noinline]] inline std::unique_ptr<Fixture>
make_fixture(const Recipe& recipe, const Value& entry, bool spatial, double speed) {
    auto fixture = std::make_unique<Fixture>();
    fixture->request = make_request(recipe, entry, spatial, speed);
    auto owner = bike::initialization::make_setup(*fixture->request);
    // make_setup + settle owns the real cold path through settle_context.
    fixture->equilibrium = bike::initialization::settle(*owner, fixture->request->equilibrium);
    bt::require(!fixture->equilibrium.cache_hit && fixture->equilibrium.steps > 0,
                "bridge performs a fresh constrained/planar settle without cached evidence");
    fixture->bootstrap = std::make_unique<runtime::RuntimeBootstrap>(owner->seal_bootstrap());
    bt::require(fixture->bootstrap->config.timestep_s == recipe.dt_s &&
                fixture->bootstrap->config.control_period_s == recipe.control_s,
                "paired physical and control clocks match the manifest exactly");
    bt::require(fixture->bootstrap->config.monitors.grounded_hold_s == recipe.persistence_s,
                "persistence comparison uses the actual configured window");
    fixture->ride = std::make_unique<runtime::NativeRideRuntime>(*fixture->bootstrap);
    fixture->initial = observe_initial(*fixture->ride);
    if (spatial) {
        bt::require(fixture->bootstrap->state.tire3d.has_value() && fixture->bootstrap->state.spatial.has_value(),
                    "fresh constrained seal carries real typed material and spatial causal state");
        inspect_real_writer(*fixture);
    } else {
        RuntimeRigAccess::inspect(*fixture->ride, [](const Stepper& stepper, const runtime::PhysicalStep&) {
            bt::require(stepper.has_tire() && stepper.tire_writer().is_pneumatic(), "paired planar baseline is the real pneumatic backend");
        });
    }
    return fixture;
}
inline double initial_error(const Fixture& planar, const Fixture& spatial) {
    double maximum = 0.;
    for (const auto& [name, value] : planar.initial.joints) {
        const auto found = spatial.initial.joints.find(name);
        bt::require(found != spatial.initial.joints.end(), "spatial topology preserves every planar physical initial coordinate");
        maximum = std::max({maximum, std::abs(value.position - found->second.position),
                            std::abs(value.velocity - found->second.velocity)});
    }
    for (const auto& [name, value] : spatial.initial.joints) if (!planar.initial.joints.contains(name)) {
        bt::require(name == "root_y" || name == "steer_joint", "only the declared constrained coordinates may be extra");
        maximum = std::max({maximum, std::abs(value.position), std::abs(value.velocity)});
    }
    return maximum;
}

inline double channel_number(const runtime::WireObject& row, std::string_view name) {
    const double result = wire::number(wire::required(row, name));
    bt::require(std::isfinite(result), "real bridge channel is finite");
    return result;
}
struct Row {
    std::int64_t step = 0;
    double time_s = 0., end_s = 0.;
    std::array<double, 2> load{}, moment{}, longitudinal{}, lateral{};
    std::array<bool, 2> raw{}, grounded{};
};
inline Row read_row(const runtime::PhysicalSampleData& sample, bool spatial) {
    Row result;
    result.step = sample.interval_id;
    result.time_s = sample.time_s;
    result.end_s = sample.end_time_s;
    const auto& tyres = wire::object(wire::required(sample.channels, "tires"));
    constexpr std::array<const char*, 2> sides{"front", "rear"};
    for (std::size_t side = 0; side < sides.size(); ++side) {
        const auto& wheel = wire::object(wire::required(tyres, sides[side]));
        bt::require(wire::string(wire::required(wheel, "backend")) == (spatial ? "pneumatic_3d" : "pneumatic"),
                    "retained tyre sample names its real backend");
        result.load[side] = channel_number(wheel, "normal_load_n");
        result.moment[side] = channel_number(wheel, "wheel_axis_moment_nm");
        result.longitudinal[side] = channel_number(wheel, "tangent_force_n");
        const auto& force = wire::array(wire::required(wheel, "world_force_n"));
        bt::require(force.size() == 3, "world force has all three physical components");
        for (const auto& value : force) bt::require(std::isfinite(wire::number(value)), "lateral state is finite, never a planar NaN sentinel");
        result.lateral[side] = wire::number(force[1]);
        result.raw[side] = wire::boolean(wire::required(wheel, "raw_contact"));
        result.grounded[side] = wire::boolean(wire::required(wheel, "controller_grounded"));
        bt::require(result.load[side] >= 0., "normal support has the same positive-up sign convention");
    }
    return result;
}
inline void advance_trace(runtime::NativeRideRuntime& ride, const Recipe& recipe, std::int64_t final_step) {
    for (auto step = ride.snapshot().step; step < final_step; ++step) {
        const auto& demand = at(recipe, step);
        const auto boundary = ride.advance(step + 1, demand.control(), demand.front_brake, demand.rear_brake, {});
        if (boundary.outcome) break;
    }
    ride.flush();
}
inline std::vector<Row> retained(runtime::NativeRideRuntime& ride, bool spatial, double dt) {
    std::vector<Row> rows;
    for (const auto& sample : ride.recorded_intervals()) {
        bt::require(sample != nullptr, "retained interval is owned and present");
        const auto row = read_row(*sample, spatial);
        bt::require(row.step == static_cast<std::int64_t>(rows.size()), "retained coverage has no decimation/drop/gap");
        bt::require(std::abs(row.time_s - static_cast<double>(row.step) * dt) <= 1e-10 &&
                    std::abs(row.end_s - static_cast<double>(row.step + 1) * dt) <= 1e-10,
                    "retained samples cover actual consecutive physics intervals");
        rows.push_back(row);
    }
    bt::require(!rows.empty(), "bridge cannot qualify an empty producer trace");
    return rows;
}

struct Transition { std::int64_t step = 0; bool loaded = false; };
inline std::vector<Transition> transitions(const std::vector<Row>& rows, std::size_t side, bool filtered) {
    std::vector<Transition> events;
    for (const auto& row : rows) {
        const bool loaded = filtered ? row.grounded[side] : row.raw[side];
        if (events.empty() || loaded != events.back().loaded) events.push_back({row.step, loaded});
    }
    return events;
}
inline std::int64_t transition_delta(const std::vector<Transition>& a, const std::vector<Transition>& b) {
    // A missing/excess event is a failed metric, never omitted from the fold.
    if (a.size() != b.size()) return std::numeric_limits<std::int64_t>::max();
    std::int64_t maximum = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].loaded != b[i].loaded) return std::numeric_limits<std::int64_t>::max();
        maximum = std::max(maximum, std::abs(a[i].step - b[i].step));
    }
    return maximum;
}
inline double grounded_time(const std::vector<Row>& rows, std::size_t side) {
    double total = 0.;
    for (const auto& row : rows) if (row.grounded[side]) total += row.end_s - row.time_s;
    return total;
}
inline double maximum_hold(const std::vector<Row>& rows, std::size_t side) {
    std::optional<double> last_loaded;
    double maximum = 0.;
    for (const auto& row : rows) {
        if (row.raw[side]) last_loaded = row.time_s;
        if (row.grounded[side] && !row.raw[side]) {
            bt::require(last_loaded.has_value(), "persistence cannot create contact before first real load");
            maximum = std::max(maximum, row.time_s - *last_loaded);
        }
    }
    return maximum;
}

struct Metrics {
    std::array<double, 2> rms_n{}, planar_impulse{}, spatial_impulse{}, impulse_error{};
    std::array<std::int64_t, 2> transition_step_delta{};
    std::array<double, 2> persistence_transition_s{}, persistence_time_error_s{}, maximum_hold_s{};
    double planar_distance_m = 0., spatial_distance_m = 0., distance_error_m = 0.;
    double initial_state_error = 0., maximum_lateral_n = 0.;
};
inline Metrics metrics(const Fixture& planar, const Fixture& spatial, const std::vector<Row>& a,
                       const std::vector<Row>& b, double dt) {
    Metrics result;
    bt::require(a.size() == b.size(), "paired runs have the same retained coverage");
    result.initial_state_error = initial_error(planar, spatial);
    for (std::size_t side = 0; side < 2; ++side) {
        double squared = 0., observed_duration = 0.;
        for (std::size_t i = 0; i < a.size(); ++i) {
            bt::require(a[i].step == b[i].step && a[i].time_s == b[i].time_s, "paired physics timestamps match");
            const double planar_dt = a[i].end_s - a[i].time_s;
            const double spatial_dt = b[i].end_s - b[i].time_s;
            const double error = b[i].load[side] - a[i].load[side];
            squared += error * error * planar_dt;
            observed_duration += planar_dt;
            result.planar_impulse[side] += a[i].moment[side] * planar_dt;
            result.spatial_impulse[side] += b[i].moment[side] * spatial_dt;
            result.maximum_lateral_n = std::max(result.maximum_lateral_n, std::abs(b[i].lateral[side]));
        }
        bt::require(observed_duration > 0., "RMS uses a positive measured retained duration");
        result.rms_n[side] = std::sqrt(squared / observed_duration);
        result.impulse_error[side] = std::abs(result.spatial_impulse[side] - result.planar_impulse[side]);
        result.transition_step_delta[side] = transition_delta(transitions(a, side, false), transitions(b, side, false));
        result.persistence_transition_s[side] = static_cast<double>(transition_delta(transitions(a, side, true), transitions(b, side, true))) * dt;
        result.persistence_time_error_s[side] = std::abs(grounded_time(a, side) - grounded_time(b, side));
        result.maximum_hold_s[side] = std::max(maximum_hold(a, side), maximum_hold(b, side));
    }
    result.planar_distance_m = planar.ride->render_frame().presentation.position_m - planar.initial.position_m;
    result.spatial_distance_m = spatial.ride->render_frame().presentation.position_m - spatial.initial.position_m;
    result.distance_error_m = std::abs(result.spatial_distance_m - result.planar_distance_m);
    return result;
}
inline Value optional_outcome(const std::optional<std::string>& outcome) { return outcome ? Value{*outcome} : Value{}; }
inline Value report(const std::string& name, const Fixture& planar, const Fixture& spatial,
                    const Metrics& values, double duration, std::size_t intervals) {
    Array channels;
    constexpr std::array<const char*, 2> names{"front", "rear"};
    for (std::size_t side = 0; side < 2; ++side) channels.push_back(Value{Object{
        {"wheel", Value{std::string(names[side])}},
        {"normal_load_rms_error_n", number(values.rms_n[side])},
        {"planar_torque_impulse_nms", number(values.planar_impulse[side])},
        {"spatial_torque_impulse_nms", number(values.spatial_impulse[side])},
        {"torque_impulse_error_nms", number(values.impulse_error[side])},
        {"contact_transition_max_step_delta", integer(values.transition_step_delta[side])},
        {"persistence_transition_max_delta_s", number(values.persistence_transition_s[side])},
        {"persistence_grounded_time_error_s", number(values.persistence_time_error_s[side])},
        {"maximum_observed_hold_s", number(values.maximum_hold_s[side])}}});
    const auto identity = bike::platform::build_identity();
    return Value{Object{{"fixture", Value{name}}, {"requested_duration_s", number(duration)},
        {"planar_duration_s", number(planar.ride->snapshot().time_s)},
        {"spatial_duration_s", number(spatial.ride->snapshot().time_s)},
        {"retained_intervals", integer(static_cast<std::int64_t>(intervals))},
        {"weight_n", number(planar.initial.weight_n)}, {"spatial_weight_n", number(spatial.initial.weight_n)},
        {"initial_state_max_error", number(values.initial_state_error)},
        {"planar_distance_m", number(values.planar_distance_m)}, {"spatial_distance_m", number(values.spatial_distance_m)},
        {"distance_error_m", number(values.distance_error_m)}, {"maximum_lateral_force_n", number(values.maximum_lateral_n)},
        {"outcome_planar", optional_outcome(planar.ride->snapshot().outcome)},
        {"outcome_spatial", optional_outcome(spatial.ride->snapshot().outcome)},
        {"planar_model_sha256", Value{planar.ride->snapshot().model_sha256}},
        {"spatial_model_sha256", Value{spatial.ride->snapshot().model_sha256}},
        {"terrain_sha256", Value{spatial.request->model.terrain3d->fingerprint_hex()}},
        {"platform", Value{identity.platform}}, {"source_manifest_sha256", Value{identity.source_manifest_sha256}},
        {"execution_identity", Value{bike::platform::execution_identity()}}, {"channels", Value{std::move(channels)}}}};
}

inline void real_codec_probe_restore(runtime::NativeRideRuntime& ride, double dt) {
    RuntimeRigAccess::inspect(ride, [&](const Stepper& stepper, const runtime::PhysicalStep&) {
        bt::require(stepper.has_spatial_tire(), "codec assertions require the actual installed producer");
        auto& writer = stepper.spatial_tire_writer();
        const auto before = writer.state_bytes();
        const auto identity = writer.state().identity;
        auto decoded = bike::pneumatic3d::decode_tyre_state(before, identity);
        bt::require(decoded.has_value(), "W27 decodes the real producer state");
        bt::require(bike::pneumatic3d::encode_tyre_state(*decoded) == before, "real state codec canonical roundtrip");
        const auto encoded = bike::pneumatic3d::base64_canonical(before);
        const auto binary = bike::pneumatic3d::base64_decode_strict(encoded);
        bt::require(binary && *binary == before, "real state strict base64 roundtrip");
        bt::require(!bike::pneumatic3d::base64_decode_strict(encoded + "\n"), "noncanonical state encoding is rejected");
        auto wrong_identity = identity;
        bt::require(!wrong_identity.terrain_sha256_hex.empty(), "real terrain identity is populated");
        wrong_identity.terrain_sha256_hex.front() = wrong_identity.terrain_sha256_hex.front() == '0' ? '1' : '0';
        bt::require(!bike::pneumatic3d::decode_tyre_state(before, wrong_identity), "wrong terrain identity rejected before restore");
        auto corrupt = *decoded;
        corrupt.identity = wrong_identity;
        bt::require(writer.restore(corrupt) != CoreStatus::ok && writer.state_bytes() == before,
                    "rejected identity restore is atomic on the live writer");
        corrupt = *decoded;
        bt::require(!corrupt.carcass[0].deflection_m.empty(), "real material inventory has carcass cells");
        corrupt.carcass[0].deflection_m.front() = std::numeric_limits<double>::quiet_NaN();
        bt::require(writer.restore(corrupt) != CoreStatus::ok && writer.state_bytes() == before,
                    "rejected nonfinite material restore is atomic on the live writer");
        bt::require(writer.restore(*decoded) == CoreStatus::ok && writer.state_bytes() == before,
                    "valid live material restore preserves all canonical state bytes");
        std::vector<double> first(static_cast<std::size_t>(stepper.model()->nv)), second(first.size());
        bt::require(writer.try_compute_into(stepper.data(), dt, false, first) == CoreStatus::ok,
                    "real 3d writer observational force evaluation succeeds");
        bt::require(writer.try_compute_into(stepper.data(), dt, false, second) == CoreStatus::ok &&
                    same_doubles(first, second) && writer.state_bytes() == before,
                    "repeated producer probes neither advance nor replace committed material state");
    });
}
inline void integrated_state_checks(const Fixture& fixture, const Recipe& recipe) {
    auto inspected = std::make_unique<runtime::NativeRideRuntime>(*fixture.bootstrap);
    auto uninterrupted = std::make_unique<runtime::NativeRideRuntime>(*fixture.bootstrap);
    const auto before = inspected->snapshot();
    for (const auto& demand : recipe.demands) {
        const auto probe = inspected->probe_step_inputs(demand.control(), demand.front_brake, demand.rear_brake);
        bt::require(probe.has_contact_probe && !probe.components.empty(), "runtime probe evaluates the real physical writer ensemble");
        same_boundary(before, inspected->snapshot());
    }
    real_codec_probe_restore(*inspected, recipe.dt_s);
    same_boundary(before, inspected->snapshot());
    advance_trace(*inspected, recipe, 32);
    advance_trace(*uninterrupted, recipe, 32);
    same_boundary(inspected->snapshot(), uninterrupted->snapshot());
    real_codec_probe_restore(*inspected, recipe.dt_s);
    advance_trace(*inspected, recipe, 64);
    advance_trace(*uninterrupted, recipe, 64);
    same_boundary(inspected->snapshot(), uninterrupted->snapshot());
    const auto generation = inspected->snapshot().generation;
    inspected->reset();
    bt::require(inspected->snapshot().generation == generation + 1, "reset advances only the generation identity");
    same_boundary(before, inspected->snapshot(), false);
    auto fresh = std::make_unique<runtime::NativeRideRuntime>(*fixture.bootstrap);
    advance_trace(*inspected, recipe, 64);
    advance_trace(*fresh, recipe, 64);
    same_boundary(inspected->snapshot(), fresh->snapshot(), false);
}

inline void static_and_low_speed(const Recipe& recipe, const Value& entry) {
    for (const double speed : {0., .1}) {
        auto planar = make_fixture(recipe, entry, false, speed);
        auto spatial = make_fixture(recipe, entry, true, speed);
        const Demand released;
        const auto planar_probe = planar->ride->probe_step_inputs(released.control(), 0., 0.);
        const auto spatial_probe = spatial->ride->probe_step_inputs(released.control(), 0., 0.);
        bt::require(planar_probe.has_contact_probe && spatial_probe.has_contact_probe, "both low-speed producers publish real contacts");
        double spatial_load = 0., lateral = 0., planar_load = 0.;
        RuntimeRigAccess::inspect(*spatial->ride, [&](const Stepper& stepper, const runtime::PhysicalStep&) {
            for (std::size_t side = 0; side < 2; ++side)
                for (const auto& wrench : stepper.spatial_tire_writer().wrenches(side)) {
                    bt::require(wrench.valid && wrench.normal_load_n >= 0. && wrench.force_world_n[2] >= 0., "flat tyre contact has positive support sign");
                    spatial_load += wrench.normal_load_n;
                    lateral += wrench.force_world_n[1];
                }
        });
        RuntimeRigAccess::inspect(*planar->ride, [&](const Stepper& stepper, const runtime::PhysicalStep&) {
            for (const auto& wheel : stepper.tire_writer().probe_snapshot_storage())
                for (const auto& patch : wheel.patches) planar_load += patch.normal_load_n;
        });
        std::cout << bike::io::canonical_json(Value{Object{{"fixture", Value{std::string("flat-static")}},
            {"speed_mps", number(speed)}, {"weight_n", number(planar->initial.weight_n)},
            {"planar_load_n", number(planar_load)}, {"spatial_load_n", number(spatial_load)},
            {"lateral_force_n", number(lateral)}}}) << '\n';
        bt::require(std::abs(lateral) <= 1e-9, "flat static/low-speed total lateral force <= 1e-9 N");
        bt::require(std::abs(planar_load - planar->initial.weight_n) <= 1e-6 * planar->initial.weight_n,
                    "planar static normal load equals actual weight within 1e-6");
        bt::require(std::abs(spatial_load - spatial->initial.weight_n) <= 1e-6 * spatial->initial.weight_n,
                    "spatial static normal load equals actual weight within 1e-6");
        if (speed > 0.) {
            for (std::int64_t step = 0; step < 4; ++step) {
                (void)planar->ride->advance(step + 1, released.control(), .05, .05, {});
                (void)spatial->ride->advance(step + 1, released.control(), .05, .05, {});
            }
            planar->ride->flush();
            spatial->ride->flush();
            const auto a = retained(*planar->ride, false, recipe.dt_s);
            const auto b = retained(*spatial->ride, true, recipe.dt_s);
            bt::require(a.size() == b.size() && a.size() == 4, "low-speed sign comparison has four real braking intervals");
            bool nonzero_force = false, nonzero_moment = false;
            for (std::size_t i = 0; i < a.size(); ++i) for (std::size_t side = 0; side < 2; ++side) {
                if (std::abs(a[i].longitudinal[side]) > 1e-9 && std::abs(b[i].longitudinal[side]) > 1e-9) {
                    nonzero_force = true;
                    bt::require(std::signbit(a[i].longitudinal[side]) == std::signbit(b[i].longitudinal[side]),
                                "low-speed longitudinal force signs agree between real producers");
                }
                if (std::abs(a[i].moment[side]) > 1e-9 && std::abs(b[i].moment[side]) > 1e-9) {
                    nonzero_moment = true;
                    bt::require(std::signbit(a[i].moment[side]) == std::signbit(b[i].moment[side]),
                                "low-speed wheel-axis moment signs agree between real producers");
                }
            }
            bt::require(nonzero_force && nonzero_moment, "sign assertions observe nonzero braking force and moment");
        }
    }
}

inline void paired_case(const Recipe& recipe, const Value& entry, double duration, bool state_checks) {
    const std::string name = string(entry, "id");
    auto planar = make_fixture(recipe, entry, false, recipe.speed_mps);
    auto spatial = make_fixture(recipe, entry, true, recipe.speed_mps);
    const double initial = initial_error(*planar, *spatial);
    // Print this precondition separately so a W25 calibration/initialization
    // mismatch remains diagnosable even before the dynamic comparison.
    std::cout << bike::io::canonical_json(Value{Object{{"fixture", Value{name}},
        {"initial_state_max_error", number(initial)}, {"weight_n", number(planar->initial.weight_n)},
        {"spatial_weight_n", number(spatial->initial.weight_n)}}}) << '\n';
    bt::require(initial <= recipe.initial_tolerance,
                "paired bridge requires the same initial physical state; rebase needs material/energy recapture, not a hidden vector edit");
    bt::require(std::abs(planar->initial.weight_n - spatial->initial.weight_n) <= 1e-6 * planar->initial.weight_n,
                "paired models have the same measured weight");
    if (state_checks) integrated_state_checks(*spatial, recipe);
    const auto steps = static_cast<std::int64_t>(std::llround(duration / recipe.dt_s));
    bt::require(steps > 0 && std::abs(static_cast<double>(steps) * recipe.dt_s - duration) <= 1e-12,
                "requested bridge duration is an exact physics-step count");
    advance_trace(*planar->ride, recipe, steps);
    advance_trace(*spatial->ride, recipe, steps);
    const auto a = retained(*planar->ride, false, recipe.dt_s), b = retained(*spatial->ride, true, recipe.dt_s);
    const auto result = metrics(*planar, *spatial, a, b, recipe.dt_s);
    std::cout << bike::io::canonical_json(report(name, *planar, *spatial, result, duration, a.size())) << '\n';
    bt::require(planar->ride->snapshot().step == steps && spatial->ride->snapshot().step == steps,
                "bridge covers its complete requested horizon without early termination");
    bt::require(a.size() == static_cast<std::size_t>(steps), "one retained sample for every requested interval");
    bt::require(planar->ride->snapshot().outcome == spatial->ride->snapshot().outcome, "planar and constrained outcomes match");
    for (std::size_t side = 0; side < 2; ++side) {
        bt::require(result.rms_n[side] <= .02 * planar->initial.weight_n, "normal-load RMS <= 2% actual weight per wheel");
        bt::require(result.impulse_error[side] <= std::max(.01, .01 * std::abs(result.planar_impulse[side])),
                    "wheel torque impulse error <= max(0.01 N.m.s, 1% planar impulse)");
        bt::require(result.transition_step_delta[side] <= 1, "contact transitions match within one physics step");
        bt::require(result.persistence_transition_s[side] <= recipe.persistence_s + 1e-12 &&
                    result.persistence_time_error_s[side] <= recipe.persistence_s + 1e-12,
                    "persistence event/time errors <= one configured window");
        bt::require(result.maximum_hold_s[side] <= recipe.persistence_s + 1e-12,
                    "actual dropout persistence does not exceed the configured window");
    }
    bt::require(result.distance_error_m <= std::max(.01, .01 * std::abs(result.planar_distance_m)),
                "distance error <= max(0.01 m, 1% planar distance)");
}
inline void run(double duration, bool short_suite) {
    const auto recipe = read_recipe();
    const auto& fixtures = bike::io::as_array(member(recipe.manifest, "fixtures"), "bridge.fixtures");
    bt::require(fixtures.size() == 3, "flat, rough_uphill_savage and alpine_rocky are all mandatory");
    constexpr std::array<const char*, 3> names{"flat", "rough_uphill_savage", "alpine_rocky"};
    for (std::size_t i = 0; i < names.size(); ++i)
        bt::require(string(fixtures[i], "id") == names[i], "manifest cannot silently substitute a bridge track");
    if (short_suite) static_and_low_speed(recipe, fixtures.front());
    for (const auto& entry : fixtures) {
        try { paired_case(recipe, entry, duration, short_suite && string(entry, "id") == "flat"); }
        catch (const std::exception& error) {
            std::cout << bike::io::canonical_json(Value{Object{{"fixture", Value{string(entry, "id")}},
                {"requested_duration_s", number(duration)}, {"status", Value{std::string("failed")}},
                {"error", Value{std::string(error.what())}}}}) << '\n';
            throw; // Missing W25/W27/capability is a failure, never a skip.
        }
    }
}
} // namespace spatial_tests::bridge
