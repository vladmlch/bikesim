#include "setup_owner.hpp"
#include "../diag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <ranges>
#include <stdexcept>
#include "capture.hpp"
#include "context.hpp"
#include "launch_preparation.hpp"
#include "../configuration/load.hpp"
#include "../engine_identity.hpp"
#include "../io/hash.hpp"
#include "../model/export.hpp"
#include "../model/mapping.hpp"
#include "../platform/provenance.hpp"
#include "../rider/allocation.hpp"
#include "../rider/senses_codec.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/provenance.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/step.hpp"
#include "../spatial/config.hpp"
#include "../stepper.hpp"
#include "../terrain/extrusion3d.hpp"
#include "../terrain/generator3d.hpp"
#include "../terrain/grid3d.hpp"
#include "../terrain/heightfield3d.hpp"
#include "../terrain/profile.hpp"
#include "../terrain/source3d.hpp"
#include "../terrain/trackfile.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "writers/distributed_3d_tire.hpp"
#include "../io/json.hpp"

namespace bike::initialization {
namespace {
using namespace runtime::setupvalue;

void populate_heightfield(mjModel const& compiled, const SetupRequest& request) {
    const int id = model::required_id(compiled, mjOBJ_HFIELD, "road");
    const auto rows = std::views::counted(compiled.hfield_nrow, compiled.nhfield);
    const auto columns = std::views::counted(compiled.hfield_ncol, compiled.nhfield);
    const auto addresses = std::views::counted(compiled.hfield_adr, compiled.nhfield);
    const auto& field = request.model.field;
    const auto index = static_cast<std::size_t>(id);
    if (rows[index] != field.nrow || columns[index] != field.ncol)
        throw std::invalid_argument("compiled road dimensions differ from the setup heightfield");
    const auto heights = terrain::build_field_data(request.track, field);
    const auto offset = addresses[index];
    if (offset < 0 || std::cmp_greater(offset, compiled.nhfielddata) ||
        heights.size() > static_cast<std::size_t>(compiled.nhfielddata - offset))
        throw std::invalid_argument("compiled road address is outside heightfield storage");
    auto const target = std::views::counted(compiled.hfield_data, compiled.nhfielddata)
        .subspan(static_cast<std::size_t>(offset), heights.size());
    for (std::size_t i = 0; i < heights.size(); ++i) {
        if (!std::isfinite(heights[i]) || heights[i] < 0. || heights[i] > 1.)
            throw std::invalid_argument("normalized road elevation is outside [0, 1]");
        // MuJoCo stores heightfield samples as float. This assignment is the
        // same binary32 conversion made by the source hfield_data write.
        target[i] = static_cast<float>(heights[i]);
    }
}

Value equilibrium_value(const EquilibriumResult& result) {
    Value value{Object{{"steps", integer(result.steps)},
        {"residual_qacc", real(result.residual_qacc)}, {"cache_hit", Value{result.cache_hit}},
        {"fork_travel_mm", real(result.fork_travel_mm)},
        {"shock_stroke_mm", real(result.shock_stroke_mm)},
        {"rear_travel_mm", real(result.rear_travel_mm)}, {"root_z_m", real(result.root_z_m)},
        {"pitch_rad", real(result.pitch_rad)},
        {"initial_crank_phase_rad", real(result.initial_crank_phase_rad)},
        {"settled_crank_phase_rad", real(result.settled_crank_phase_rad)},
        {"state_file", result.state_file ? text(*result.state_file) : Value{}},
        {"static_front_load_n", real(result.static_front_load_n)},
        {"static_rear_load_n", real(result.static_rear_load_n)},
        {"total_vertical_force_n", real(result.total_vertical_force_n)},
        {"initial_state_target_residual_qacc", result.initial_state_target_residual_qacc
            ? real(*result.initial_state_target_residual_qacc) : Value{}},
        {"initial_state_sha256", result.initial_state_sha256 ? text(*result.initial_state_sha256) : Value{}},
        {"initial_state_source_sha256", result.initial_state_source_sha256
            ? text(*result.initial_state_source_sha256) : Value{}},
        {"initial_state_source_timestep_s", result.initial_state_source_timestep_s
            ? real(*result.initial_state_source_timestep_s) : Value{}},
        {"source_equilibrium_residual_qacc", result.source_equilibrium_residual_qacc
            ? real(*result.source_equilibrium_residual_qacc) : Value{}}}};
    // Kinetic-settle diagnostics appear only when set: the default record
    // keeps its historical keys.
    auto& object = std::get<Object>(value.data);
    if (result.relaxation) object.emplace_back("relaxation", text(*result.relaxation));
    if (result.relaxation_resets)
        object.emplace_back("relaxation_resets", integer(*result.relaxation_resets));
    if (result.settle_x_drift_m)
        object.emplace_back("settle_x_drift_m", real(*result.settle_x_drift_m));
    return value;
}

void length(io::Sha256& digest, std::uint64_t size) {
    std::array<std::byte, 8> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i)
        bytes[bytes.size() - 1 - i] = static_cast<std::byte>((size >> (8 * i)) & 255U);
    digest.update(bytes);
}

void sync_spatial_config(SetupRequest& request) {
    if (request.spatial.has_value() && request.model.spatial.has_value()) {
        const auto a = io::canonical_json(spatial::spatial_config_to_value(*request.spatial));
        const auto b = io::canonical_json(spatial::spatial_config_to_value(*request.model.spatial));
        if (a != b)
            throw std::invalid_argument("SetupRequest.spatial differs from SetupRequest.model.spatial");
    } else if (request.spatial.has_value()) {
        request.model.spatial = request.spatial;
    } else if (request.model.spatial.has_value()) {
        request.spatial = request.model.spatial;
    }
    if (!request.spatial.has_value() && request.terrain_overlay.has_value())
        throw std::invalid_argument("terrain_overlay requires spatial configuration");
}

struct SpatialTerrainBundle {
    std::unique_ptr<terrain::Grid3d> hfield_grid;
    terrain::HeightField3dBuild hf;
    std::unique_ptr<terrain::Terrain3dSource> source;
};

// Callback signature is fixed by Compliant3dTerrainCallbacks::sample_fn.
terrain::TerrainSample3d sample_terrain3d_cb(
    const void* c, double x, double y) noexcept { // NOLINT(bugprone-easily-swappable-parameters)
    return static_cast<const terrain::Terrain3dSource*>(c)->sample(x, y);
}

terrain::TorusClosestPoint3d closest_terrain3d_cb(const void* c,
                                                  const terrain::Vec3d& center,
                                                  double search_radius_m) noexcept {
    return static_cast<const terrain::Terrain3dSource*>(c)->closest_point(
        center, search_radius_m);
}

Compliant3dTerrainCallbacks terrain3d_callbacks(const terrain::Terrain3dSource& src) noexcept {
    return Compliant3dTerrainCallbacks{
        .ctx = &src,
        .sample_fn = &sample_terrain3d_cb,
        .closest_point_fn = &closest_terrain3d_cb,
    };
}

SpatialTerrainBundle build_spatial_terrain(const spatial::SpatialConfig& sp,
                                           const std::optional<terrain::Terrain3dOverlaySpec>& overlay_spec,
                                           const terrain::TrackSpec& track,
                                           double ground_z_m) {
    SpatialTerrainBundle out;
    if (overlay_spec.has_value()) {
        terrain::Terrain3dOverlaySpec overlay = *overlay_spec;
        if (overlay.datum_z_offset_m == 0.)
            overlay.datum_z_offset_m = ground_z_m;
        out.hfield_grid = std::make_unique<terrain::Grid3d>(
            terrain::build_overlay_grid3d(track, overlay));
        out.hf = terrain::build_heightfield_3d(*out.hfield_grid, ground_z_m);
        out.source = std::make_unique<terrain::Terrain3dSource>(*out.hfield_grid);
    } else {
        terrain::ExtrudedTrack3d ext(track, sp.corridor_half_width_m, ground_z_m);
        const int ny_steps = std::max(4, static_cast<int>(std::ceil((2.0 * sp.corridor_half_width_m) / 0.25)));
        const double dy_m = (2.0 * sp.corridor_half_width_m) / static_cast<double>(ny_steps);
        const auto gs = terrain::make_grid3d_spec(ext.bounds(), terrain::DEFAULT_RESOLUTION_M, dy_m);
        out.hfield_grid = std::make_unique<terrain::Grid3d>(terrain::sample_to_grid3d(ext, gs));
        out.hf = terrain::build_heightfield_3d(*out.hfield_grid, ground_z_m);
        out.source = std::make_unique<terrain::Terrain3dSource>(std::move(ext));
    }
    const auto extent = terrain::profile_extent(track, out.hf.spec.resolution_m());
    if (extent.first < out.hf.spec.min_profile_m()) {
        out.hf.spec.datum_z_m = std::max(
            out.hf.spec.datum_z_m, std::ceil((-extent.first + 1.0) * 10.0) / 10.0);
    }
    if (extent.second > out.hf.spec.max_profile_m()) {
        out.hf.spec.elevation_m = std::max(
            out.hf.spec.elevation_m,
            std::ceil((out.hf.spec.datum_z_m + extent.second + 1.0) * 10.0) / 10.0);
    }
    out.hf.spec.validate();
    return out;
}

std::vector<std::array<double, 2>> spatial_centerline_vertices(
    const terrain::Terrain3dSource& source, const terrain::HeightFieldSpec& field) {
    const auto& bounds = source.bounds();
    const int ncol = field.ncol;
    const double dx = field.resolution_m();
    std::vector<std::array<double, 2>> vertices;
    vertices.reserve(static_cast<std::size_t>(ncol));
    for (int c = 0; c < ncol; ++c) {
        const double x = (c + 1 == ncol)
            ? bounds.x_max_m
            : std::min(bounds.x_max_m, bounds.x_min_m + static_cast<double>(c) * dx);
        const auto sample = source.sample(x, 0.0);
        if (sample.status != terrain::QueryStatus3d::ok)
            throw std::invalid_argument("spatial centerline sample fell outside terrain bounds");
        vertices.push_back({x, sample.height_m});
    }
    return vertices;
}

int resolve_brake_dof(const mjModel& model, const runtime::JointReference& ref, const char* what) {
    const int joint = mj_name2id(&model, mjOBJ_JOINT, ref.joint.c_str());
    if (joint < 0)
        throw std::invalid_argument(std::string("unknown ") + what + " brake joint '" + ref.joint + "'");
    const auto jtypes = std::views::counted(model.jnt_type, model.njnt);
    const int dof_count = [&] {
        switch (jtypes[static_cast<std::size_t>(joint)]) {
        case mjJNT_FREE: return 6;
        case mjJNT_BALL: return 3;
        default: return 1;
        }
    }();
    if (ref.dof_index < 0 || ref.dof_index >= dof_count)
        throw std::invalid_argument(std::string("config.geometry.brake_dofs: ") + what + " dof_index out of range");
    return std::views::counted(model.jnt_dofadr, model.njnt)[static_cast<std::size_t>(joint)] + ref.dof_index;
}

std::unique_ptr<runtime::PhysicalStep> make_spatial_physical_step(
    Stepper& stepper, const runtime::RuntimeConfig& config,
    const terrain::Terrain3dSource& terrain3d, const terrain::HeightFieldSpec& field) {
    runtime::StepConfig step_cfg;
    step_cfg.timestep_s = config.timestep_s;
    step_cfg.control_period_s = config.control_period_s;
    step_cfg.measurement_decimation = config.measurement_decimation;
    step_cfg.strict = config.strict;
    step_cfg.crank_length_m = config.geometry.crank_length_m;
    step_cfg.road_lookahead_m = io::as_double(
        runtime::setupvalue::field(config.rider_controller, "road_lookahead_m"),
        "config.rider_controller.road_lookahead_m");
    step_cfg.rider_present = !io::is_null(config.geometry.pose);
    step_cfg.monitors = config.monitors;
    step_cfg.intent = ::rider::decode_intent_config(config.rider_intent, "config.rider_intent");
    step_cfg.reaction = ::rider::decode_reaction_config(config.rider_reaction, "config.rider_reaction");
    step_cfg.launch = ::rider::decode_launch_config(config.rider_launch, "config.rider_launch");
    step_cfg.surface_labels = ::rider::decode_surface_labels(config.surface_labels, "config.surface_labels");
    step_cfg.spatial = config.spatial;

    const auto& drv = stepper.drive().config();
    const runtime::StaticBrake brake(
        resolve_brake_dof(*stepper.model(), config.geometry.front_brake, "front"),
        resolve_brake_dof(*stepper.model(), config.geometry.rear_brake, "rear"),
        drv.brake_ceiling_nm,
        runtime::BrakeHoldSpec{
            .torque_nm = drv.brake_hold_torque_nm,
            .stiffness_nm_rad = drv.brake_hold_stiffness_nm_rad,
            .damping_nms_rad = drv.brake_hold_damping_nms_rad,
            .engage_radps = drv.brake_hold_engage_radps,
            .release_radps = drv.brake_hold_release_radps,
            .unified_ceiling = true});

    std::optional<::rider::SpindlePose> pose;
    std::optional<::rider::SpindleConfig> rider_cfg;
    if (step_cfg.rider_present) {
        pose = ::rider::decode_spindle_pose(config.geometry.pose, "config.geometry.pose");
        rider_cfg = ::rider::decode_spindle_config(config.rider_controller);
        rider_cfg->reposition_damping_nms_rad = drv.policies.pedaling.reposition_damping_nms_rad;
        rider_cfg->reposition_torque_limit_nm = step_cfg.intent.max_crank_torque_nm;
    }
    auto vertices = spatial_centerline_vertices(terrain3d, field);
    return std::make_unique<runtime::PhysicalStep>(
        stepper, std::move(step_cfg), std::move(vertices), brake, pose, rider_cfg);
}

} // namespace

std::string legacy_source_fingerprint(const std::filesystem::path& root) {
    if (!std::filesystem::is_directory(root))
        throw std::invalid_argument(root.string() + ": expected a legacy source directory");
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (entry.path().extension() != ".py") continue;
        if (entry.is_symlink() || !entry.is_regular_file())
            throw std::invalid_argument(entry.path().string() + ": expected a regular source file");
        files.push_back(entry.path().lexically_relative(root));
    }
    if (files.empty()) throw std::invalid_argument(root.string() + ": no legacy Python source files");
    std::ranges::sort(files, {}, [](const auto& path) { return path.generic_string(); });
    io::Sha256 digest;
    for (const auto& relative : files) {
        const auto file = root / relative;
        const auto size = std::filesystem::file_size(file);
        if (size > std::uintmax_t{512U} * 1024U * 1024U)
            throw std::invalid_argument(file.string() + ": source file exceeds 512 MiB");
        std::ifstream input(file, std::ios::binary);
        if (!input) throw std::runtime_error(file.string() + ": cannot read source fingerprint input");
        const auto name = relative.generic_string();
        length(digest, name.size());
        digest.update(std::as_bytes(std::span{name}));
        length(digest, size);
        std::vector<char> buffer(65536);
        std::uintmax_t consumed = 0;
        while (input) {
            input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            const auto count = input.gcount();
            if (count > 0) {
                consumed += static_cast<std::uintmax_t>(count);
                digest.update(std::as_bytes(std::span{buffer}.first(static_cast<std::size_t>(count))));
            }
        }
        if (!input.eof() || consumed != size)
            throw std::runtime_error(file.string() + ": source changed or failed during fingerprinting");
    }
    constexpr std::string_view digits = "0123456789abcdef";
    std::string result;
    result.reserve(64);
    for (const auto byte : digest.finish()) {
        result.push_back(digits[std::uint32_t{byte} >> 4U]);
        result.push_back(digits[std::uint32_t{byte} & 15U]);
    }
    return result;
}

struct SetupOwner::Impl {
    SetupRequest request;
    runtime::RuntimeConfig config;
    // Reverse destruction: context, physical step, stepper, then terrain3d (callbacks).
    std::unique_ptr<terrain::Terrain3dSource> terrain3d;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<runtime::PhysicalStep> physical;
    std::unique_ptr<SetupContext> context;
    std::optional<EquilibriumResult> result;
    // The certified equilibrium pose file, taken between the solve and
    // finalize: finalize restarts the material clocks and runs an advancing
    // tire compute that commits the carcass memory (delta_prev :=
    // delta(qpos)), so a file read from the live context afterwards would
    // restore without the hysteresis the recorded residual was measured on.
    // Only the state file needs the pre-finalize read — an initial-state
    // transfer carries the finalized launch state instead.
    std::optional<StateFile> settled_state_file;
    std::optional<std::string> sealed_model_sha256;
    std::optional<ResearchStartup> research_startup;
    bool sealed = false;
};

SetupOwner::SetupOwner(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
SetupOwner::~SetupOwner() = default;

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<SetupOwner> make_setup(SetupRequest request) {
    if (!request.model.physics)
        throw std::invalid_argument("native setup requires an explicit physical configuration");
    sync_spatial_config(request);
    // Launch part 2: the rider prepares the start before anything validates
    // or reads the computed drive members — the cog and pedal angle become
    // drive.gearing.rear_teeth / drive.crank_phase_rad for the model, the
    // writers and the recording; the lean waits in the context for the
    // settle and the intent reset.
    const auto preparation = prepare_request(request);
    configuration::post_init(*request.model.physics);
    if (request.spatial.has_value()) {
        spatial::validate_spatial_config(*request.spatial, request.model.physics->timestep_s);
        switch (request.spatial->tire_backend) {
        case spatial::SpatialTireBackend::compliant_3d: {
            const auto tire_cfg = make_compliant_3d_tire_config(request.spatial->tire_preset);
            request.model.bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.;
            request.model.bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.;
            break;
        }
        case spatial::SpatialTireBackend::distributed_3d: {
            const auto tire_cfg = make_distributed_3d_tire_config(request.spatial->tire_preset);
            request.model.bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.;
            request.model.bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.;
            break;
        }
        }
    } else if (request.model.physics->tires.backend == "pneumatic") {
        request.model.bike.front_wheel_radius =
            request.model.physics->tires.pneumatic.front.outer_radius_mm;
        request.model.bike.rear_wheel_radius =
            request.model.physics->tires.pneumatic.rear.outer_radius_mm;
    }
    if (request.model.mode != "ride")
        throw std::invalid_argument("native setup requires model mode 'ride'");
    auto resolved_rider = model::resolve_rider(request.model.rider ? model::RiderArg{*request.model.rider}
        : model::RiderArg{std::monostate{}}, request.model.include_rider, "lumped");
    runtime::validate_native_supported(*request.model.physics, resolved_rider);
    if (!std::isfinite(request.start_x_m))
        throw std::invalid_argument("setup start_x_m must be finite");
    request.track.validate();

    auto impl = std::make_unique<SetupOwner::Impl>();
    std::unique_ptr<SpatialTerrainBundle> spatial_terrain;
    const double ground_z_m = geometry::compute_ground_z(request.model.bike) / 1000.;
    if (request.spatial.has_value()) {
        spatial_terrain = std::make_unique<SpatialTerrainBundle>(
            build_spatial_terrain(*request.spatial, request.terrain_overlay, request.track, ground_z_m));
        request.model.field = spatial_terrain->hf.spec;
        impl->terrain3d = std::move(spatial_terrain->source);
    }

    request.model.field.validate();
    terrain::assert_track_fits(request.track, request.model.field);
    request.model.rider = resolved_rider;
    request.model.crank_joint = true;
    const auto compiled = model::compile_mjcf(model::generate_mjcf(request.model), request.asset_root);
    if (spatial_terrain) {
        terrain::populate_heightfield_3d(
            *compiled, *spatial_terrain->hfield_grid, spatial_terrain->hf);
    } else {
        populate_heightfield(*compiled, request);
    }
    impl->config = runtime::project_runtime(request.model, *compiled, request.track, request.runtime);
    auto image = capture_model_image(*compiled);
    if (request.spatial.has_value()) {
        auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
            nativecfg::decode(impl->config.writer_config));
        writer_cfg->tire.reset();
        writer_cfg->cruise.reset();
        switch (request.spatial->tire_backend) {
        case spatial::SpatialTireBackend::compliant_3d:
            writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
                request.spatial->tire_preset, terrain3d_callbacks(*impl->terrain3d));
            break;
        case spatial::SpatialTireBackend::distributed_3d:
            writer_cfg->distributed_3d_tire = make_distributed_3d_tire_config(
                request.spatial->tire_preset, terrain3d_callbacks(*impl->terrain3d));
            break;
        }
        if (writer_cfg->resistance.has_value()) {
            writer_cfg->resistance->allow_spatial = true;
            if (request.spatial->tire_backend == spatial::SpatialTireBackend::distributed_3d)
                writer_cfg->resistance->crr = 0.0;
        }
        impl->stepper = std::make_unique<Stepper>(image.mjb, *writer_cfg);
        impl->physical = make_spatial_physical_step(
            *impl->stepper, impl->config, *impl->terrain3d, request.model.field);
    } else {
        impl->stepper = std::make_unique<Stepper>(image.mjb, nativecfg::decode(impl->config.writer_config));
        impl->physical = runtime::make_physical_step(*impl->stepper, impl->config);
    }
    impl->request = std::move(request);
    impl->context = std::make_unique<SetupContext>(SetupContext{
        .stepper=*impl->stepper, .physical=*impl->physical, .physics=*impl->request.model.physics});
    auto& context = *impl->context;
    context.start_x_m = impl->request.start_x_m;
    context.preparation = preparation;
    context.ground_z_m = ground_z_m;
    context.frame_body_id = model::required_id(*compiled, mjOBJ_BODY, "frame");
    context.front_geom_id = model::required_id(*compiled, mjOBJ_GEOM, "geom_front_contact");
    context.rear_geom_id = model::required_id(*compiled, mjOBJ_GEOM, "geom_rear_contact");
    if (impl->physical->rider_present())
        context.rider_pose = ::rider::decode_spindle_pose(impl->config.geometry.pose, "setup.geometry.pose");
    auto solver = impl->request.model.solver
        ? std::make_shared<kinematics::HorstLinkageSolver>(*impl->request.model.solver)
        : std::make_shared<kinematics::HorstLinkageSolver>(impl->request.model.bike);
    context.rear_travel_from_shock_mm = [solver = std::move(solver)](double stroke) {
        return solver->solve_state_from_shock_stroke(stroke).wheel_travel;
    };
    context.identity = Value{Object{
        {"specs", model::bike_specs_value(impl->request.model.bike)},
        {"mass_specs", model::mass_specs_value(impl->request.model.masses)},
        {"rider", configuration::detail::plain_document(
            configuration::detail::as_object(configuration::detail::kRiderSpecsSchema, &resolved_rider))},
        {"track", terrain::identity_value(impl->request.track)}}};
    if (impl->request.spatial.has_value()) {
        std::get<Object>(context.identity.data).emplace_back(
            "spatial", spatial::spatial_config_to_value(*impl->request.spatial));
    }
    context.source_identity = runtime::runtime_identity().native_source_sha256;
    if (impl->request.legacy_source_root) {
        context.legacy_source_identity = legacy_source_fingerprint(*impl->request.legacy_source_root);
        context.legacy_track_identity = terrain::track_to_dict(impl->request.track);
    }
    const auto engine_id = engine::identity();
    const auto provider = engine_id.mujoco_version + "|" + engine_id.library_sha256 + "|" +
        engine_id.numeric_provider + "|" + platform::execution_identity();
    context.provider_identity = io::sha256(std::as_bytes(std::span{provider}));
    context.model_sha256 = image.sha256;
    context.cache_directory = impl->request.cache_directory;
    context.initial_state = impl->request.initial_state ? &*impl->request.initial_state : nullptr;
    return std::unique_ptr<SetupOwner>(new SetupOwner(std::move(impl)));
}
NATIVE_DIAG_POP

std::unique_ptr<SetupOwner> make_setup(const std::filesystem::path& request) {
    return make_setup(read_setup_request(request));
}

EquilibriumResult settle(SetupOwner& owner, const EquilibriumOptions& options) {
    auto& state = *owner.impl_;
    if (state.sealed || state.result)
        throw std::invalid_argument("setup equilibrium can only be completed once");
    if (state.context->initial_state)
        state.context->equilibrium = state.context->initial_state->prepare(*state.context);
    else
        state.context->equilibrium = settle_context(*state.context, options);
    // The file describes the state a restore of it installs, not the
    // settle loop's transient instrumentation: the integration clock and
    // the drive/rider contact clocks read as the values the restore leaves
    // behind (install_material restarts exactly these clocks after
    // committing the stored material), so a re-served capture is
    // byte-identical. capture_initial_state() deliberately keeps serving
    // the finalized context: a transfer carries the launch state — the
    // qvel a moving initial_speed_mps start writes inside finalize — not
    // the rest certificate.
    state.context->stepper.data()->time = 0.;
    state.context->stepper.drive().restart_clock();
    if (state.context->stepper.has_rider_contacts())
        state.context->stepper.rider_contacts_restart_clock();
    state.settled_state_file = capture_state(*state.context,
        state.context->equilibrium->steps,
        state.context->equilibrium->residual_qacc);
    finalize_initial_state(*state.context, state.config);
    if (state.request.research_startup)
        state.research_startup = prepare_research_startup(*state.context);
    state.context->initializing = false;
    state.context->completed = true;
    state.result = state.context->equilibrium;
    return *state.result;
}

PhysicalInitialState SetupOwner::capture_initial_state() const {
    if (!impl_->result || !impl_->context->completed)
        throw std::invalid_argument("initial-state capture requires a completed native setup");
    return PhysicalInitialState::capture(*impl_->context);
}

StateFile SetupOwner::capture_state_file() const {
    if (!impl_->result || !impl_->context->completed || !impl_->settled_state_file)
        throw std::invalid_argument("state-file capture requires a completed native setup");
    return *impl_->settled_state_file;
}

PhysicalInitialState SetupOwner::project_initial_stations(int count) const {
    if (!impl_->result || !impl_->context->completed)
        throw std::invalid_argument("station projection requires a completed native setup");
    return PhysicalInitialState::project_stations(*impl_->context, count);
}

runtime::RuntimeBootstrap SetupOwner::seal_bootstrap() {
    if (impl_->sealed) throw std::invalid_argument("setup bootstrap was already sealed");
    if (!impl_->result || !impl_->context->completed || impl_->context->initializing)
        throw std::invalid_argument("setup bootstrap requires completed equilibrium and reset");
    auto result = capture_initialized(*impl_->stepper, *impl_->physical, impl_->config);
    impl_->sealed_model_sha256 = result.model.sha256;
    impl_->sealed = true;
    return result;
}

io::Value SetupOwner::metadata() const {
    const auto& context = *impl_->context;
    Object fields{{"request_schema", integer(1)}, {"origin", text(impl_->request.origin)},
        {"seed_decimal", text(impl_->request.seed_decimal)}, {"model_mode", text(impl_->request.model.mode)},
        {"source_identity", text(context.source_identity)},
        {"legacy_source_identity", context.legacy_source_identity
            ? text(*context.legacy_source_identity) : Value{}},
        {"provider_identity", text(context.provider_identity)},
        {"model_sha256", text(impl_->sealed_model_sha256.value_or(context.model_sha256))},
        {"physics", configuration::to_value(context.physics)}, {"inputs", context.identity},
        {"equilibrium", impl_->result ? equilibrium_value(*impl_->result) : Value{}},
        {"sealed", Value{impl_->sealed}}};
    if (context.physics.physics_mode == "physical" &&
        impl_->request.model.rider &&
        impl_->request.model.rider->variant == "articulated_planar" &&
        context.physics.articulated.pedal_attachment != "spindle") {
        fields.emplace_back("allocation_solver",
            text(bike::rider::allocation_numeric_identity));
        fields.emplace_back("allocation_solver_qualification", text("pending"));
    }
    return Value{std::move(fields)};
}

ResearchStartup SetupOwner::research_initial_state() const {
    if (!impl_->research_startup)
        throw std::invalid_argument("research startup requires completed research setup");
    return *impl_->research_startup;
}

} // namespace bike::initialization
