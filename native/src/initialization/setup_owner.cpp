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
#include "spatial_equilibrium.hpp"
#include "../configuration/load.hpp"
#include "../engine_identity.hpp"
#include "../io/hash.hpp"
#include "../model/export.hpp"
#include "../model/mapping.hpp"
#include "../platform/provenance.hpp"
#include "../rider/allocation.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/provenance.hpp"
#include "../runtime/setup_values.hpp"
#include "../runtime/spatial_tire_factory.hpp"
#include "../stepper.hpp"
#include "../terrain/trackfile.hpp"
#include "../terrain/spec_io.hpp"

namespace bike::initialization {
namespace {
using namespace runtime::setupvalue;

void populate_heightfield(const mjModel& compiled, const SetupRequest& request) {
    if(request.model.spatial && request.model.spatial->mode==spatial::Mode::spatial3d) {
        if(!request.model.terrain3d)
            throw std::invalid_argument("setup.spatial.terrain: physical source missing");
        model::populate_render_heightfield(compiled,terrain::make_render_grid(*request.model.terrain3d,
            {.nrow=static_cast<std::uint32_t>(request.model.field.nrow),.ncol=static_cast<std::uint32_t>(request.model.field.ncol)}));
        return;
    }
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
    auto value=Value{Object{{"steps", integer(result.steps)},
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
    if(result.spatial) {
        const auto& s=*result.spatial;Array errors,dofs;
        for(const auto& error:s.errors)errors.push_back(text(error));
        for(const int dof:s.worst_dofs)dofs.push_back(integer(dof));
        set(value,"spatial",Value{Object{{"max_linear_accel_mps2",real(s.max_linear_accel_mps2)},
            {"max_angular_accel_radps2",real(s.max_angular_accel_radps2)},
            {"weight_residual_fraction",real(s.weight_residual_fraction)},
            {"pitch_moment_residual_nm",real(s.pitch_moment_residual_nm)},
            {"roll_moment_residual_nm",real(s.roll_moment_residual_nm)},
            {"contacts_valid",integer(s.contacts_valid)},{"contacts_total",integer(s.contacts_total)},
            {"evaluations",integer(s.evaluations)},{"errors",Value{std::move(errors)}},
            {"worst_dofs",Value{std::move(dofs)}}}});
    }
    return value;
}

void length(io::Sha256& digest, std::uint64_t size) {
    std::array<std::byte, 8> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i)
        bytes[bytes.size() - 1 - i] = static_cast<std::byte>((size >> (8 * i)) & 255U);
    digest.update(bytes);
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
    // Reverse destruction: context, physical step, then model/data/writers.
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<runtime::PhysicalStep> physical;
    std::unique_ptr<SetupContext> context;
    std::optional<EquilibriumResult> result;
    std::optional<std::string> sealed_model_sha256;
    std::optional<ResearchStartup> research_startup;
    std::optional<runtime::ModelImage> immutable_spatial_model;
    bool sealed = false;
};

SetupOwner::SetupOwner(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
SetupOwner::~SetupOwner() = default;

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
std::unique_ptr<SetupOwner> make_setup(SetupRequest request) {
    if (!request.model.physics)
        throw std::invalid_argument("native setup requires an explicit physical configuration");
    configuration::post_init(*request.model.physics);
    if(request.model.physics->spatial && request.model.physics->spatial->mode==spatial::Mode::spatial3d) {
        runtime::resolve_spatial_runtime_profiles(*request.model.physics);
        if(request.model.spatial) {
            auto independently_resolved=*request.model.physics;
            independently_resolved.spatial=request.model.spatial;
            runtime::resolve_spatial_runtime_profiles(independently_resolved);
            if(spatial::spatial_config_fingerprint_hex(*independently_resolved.spatial)!=
               spatial::spatial_config_fingerprint_hex(*request.model.physics->spatial))
                throw std::invalid_argument("setup.spatial: model and physics configurations differ");
        }
        request.model.spatial=request.model.physics->spatial;
    }
    const bool spatial_requested=request.model.physics->spatial &&
        request.model.physics->spatial->mode==spatial::Mode::spatial3d;
    if(request.runtime.tyre_threads<1 || request.runtime.tyre_threads>64 ||
       (!spatial_requested && request.runtime.tyre_threads!=1))
        throw std::invalid_argument("setup.runtime.tyre_threads: expected [1,64] for spatial, 1 for planar");
    if(!spatial_requested && (request.model.terrain3d ||
       (request.model.spatial && request.model.spatial->mode==spatial::Mode::spatial3d)))
        throw std::invalid_argument("setup.spatial: spatial model/source requires spatial physics");
    if (request.model.mode != "ride")
        throw std::invalid_argument("native setup requires model mode 'ride'");
    auto resolved_rider = model::resolve_rider(request.model.rider ? model::RiderArg{*request.model.rider}
        : model::RiderArg{std::monostate{}}, request.model.include_rider, "lumped");
    if(spatial_requested) {
        const auto& spatial_config=*request.model.physics->spatial;
        // model.spatial was already overwritten with physics.spatial above (spatial_requested
        // implies that branch ran), so the two configurations cannot differ here.
        request.model.spatial=spatial_config;
        if(!request.model.terrain3d)
            request.model.terrain3d=terrain::detail::load_terrain3d_source(spatial_config.terrain);
        if(!std::isfinite(request.start_x_m) || request.start_x_m<request.model.terrain3d->x_min_m() ||
           request.start_x_m>request.model.terrain3d->x_max_m())
            throw std::invalid_argument("setup.start_x_m: outside physical terrain domain");
        const auto* grid=request.model.terrain3d->grid();
        const double corridor=spatial_config.corridor_half_width_m;
        if(grid ? (grid->descriptor().y0_m>-corridor || grid->y_max_m()<corridor) :
                  request.model.terrain3d->half_width_m()<corridor)
            throw std::invalid_argument("setup.spatial.corridor_half_width_m: outside physical terrain domain");
    }
    runtime::validate_native_supported(*request.model.physics, resolved_rider);
    if (!std::isfinite(request.start_x_m))
        throw std::invalid_argument("setup start_x_m must be finite");
    if(!spatial_requested) request.track.validate();
    request.model.field.validate();
    if(!request.model.terrain3d)terrain::assert_track_fits(request.track, request.model.field);
    request.model.rider = resolved_rider;
    request.model.crank_joint = true;
    const auto compiled = model::compile_mjcf(model::generate_mjcf(request.model), request.asset_root);
    populate_heightfield(*compiled, request);
    auto impl = std::make_unique<SetupOwner::Impl>();
    impl->config = runtime::project_runtime(request.model, *compiled, request.track, request.runtime);
    auto image = capture_model_image(*compiled);
    if(impl->config.spatial)impl->immutable_spatial_model=image;
    impl->stepper = std::make_unique<Stepper>(image.mjb, nativecfg::decode(impl->config.writer_config));
    impl->physical = runtime::make_physical_step(*impl->stepper, impl->config);
    impl->request = std::move(request);
    impl->context = std::make_unique<SetupContext>(SetupContext{
        .stepper=*impl->stepper, .physical=*impl->physical, .physics=*impl->request.model.physics});
    auto& context = *impl->context;
    context.start_x_m = impl->request.start_x_m;
    context.ground_z_m = geometry::compute_ground_z(impl->request.model.bike) / 1000.;
    context.frame_body_id = model::required_id(*compiled, mjOBJ_BODY, "frame");
    context.front_geom_id = model::required_id(*compiled, mjOBJ_GEOM, "geom_front_contact");
    context.rear_geom_id = model::required_id(*compiled, mjOBJ_GEOM, "geom_rear_contact");
    if (impl->physical->rider_present())
        context.rider_pose = impl->config.spatial?
            ::rider::decode_spatial_spindle_pose(impl->config.geometry.pose,"setup.geometry.pose"):
            ::rider::decode_spindle_pose(impl->config.geometry.pose, "setup.geometry.pose");
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
        // Under spatial the request carries a name-only track label; the
        // physical terrain identity is bound below as terrain/topology/
        // profile digests. Keep the key present (the equilibrium cache
        // fingerprint reads it unconditionally) but emit null.
        {"track", impl->config.spatial ? Value{} : terrain::identity_value(impl->request.track)}}};
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
    if(impl->request.model.terrain3d) {
        context.terrain3d=impl->request.model.terrain3d;
        context.render3d=terrain::make_render_grid(*context.terrain3d,
            {.nrow=static_cast<std::uint32_t>(impl->request.model.field.nrow),.ncol=static_cast<std::uint32_t>(impl->request.model.field.ncol)});
        context.topology=spatial::bind_topology(*impl->stepper->model(),*impl->request.model.spatial); // NOLINT(bugprone-unchecked-optional-access) model.spatial is assigned whenever terrain3d is populated by the validation above
        auto& inputs=std::get<Object>(context.identity.data);
        inputs.emplace_back("spatial",spatial::spatial_config_to_wire(*impl->request.model.spatial)); // NOLINT(bugprone-unchecked-optional-access) model.spatial is assigned whenever terrain3d is populated by the validation above
        inputs.emplace_back("physical_terrain_sha256",text(context.terrain3d->fingerprint_hex()));
        inputs.emplace_back("topology_sha256",text(context.topology->fingerprint_hex));
        inputs.emplace_back("tyre_profile_sha256",text(context.physics.spatial->tyre.lateral_coefficients_sha256)); // NOLINT(bugprone-unchecked-optional-access) physics.spatial engagement follows model.spatial by setup contract
        inputs.emplace_back("steer_profile_sha256",text(context.physics.spatial->steer.profile_sha256)); // NOLINT(bugprone-unchecked-optional-access) physics.spatial engagement follows model.spatial by setup contract
        // The runtime seed is provenance. A generated terrain already binds
        // its seed and generation inputs through the physical terrain digest.
        inputs.emplace_back("runtime_seed_decimal",text(impl->request.seed_decimal));
    }
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
    // Prepare, settle_context and finalize judge with the request tolerance
    // through this field; it is set before either equilibrium path runs.
    state.context->equilibrium_tolerance = options.tolerance;
    if (state.context->initial_state)
        state.context->equilibrium = state.context->initial_state->prepare(*state.context);
    else
        state.context->equilibrium = settle_context(*state.context, options);
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
    if (!impl_->result || !impl_->context->completed)
        throw std::invalid_argument("state-file capture requires a completed native setup");
    return capture_state(*impl_->context, impl_->result->steps, impl_->result->residual_qacc);
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
    auto result = capture_initialized(*impl_->stepper, *impl_->physical, impl_->config,
        impl_->immutable_spatial_model?&*impl_->immutable_spatial_model:nullptr);
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
    if(context.terrain3d && context.render3d) {
        fields.emplace_back("tyre_threads",integer(impl_->config.tyre_threads));
        fields.emplace_back("terrain3d",Value{Object{
            {"physical_asset_sha256",text(context.terrain3d->fingerprint_hex())},
            {"render_nrow",integer(context.render3d->nrow)}, {"render_ncol",integer(context.render3d->ncol)},
            {"render_role",text("visual_and_crash_only")}}});
        fields.emplace_back("cache_rejection_reason",context.cache_rejection_reason?text(*context.cache_rejection_reason):Value{});
        fields.emplace_back("qualification",text("pending V06"));
        if(const auto& causal=context.physical.spatial_state();causal &&
            causal->steer.controller!=spatial::SteerController::off) {
            if(causal->steer.delay_fifo_nm.empty())
                throw std::invalid_argument("steer.delay_fifo_nm: empty FIFO with an active steer controller");
            const auto ticks=causal->steer.delay_fifo_nm.size()-1;
            fields.emplace_back("steer_delay_ticks",integer(static_cast<std::int64_t>(ticks)));
            fields.emplace_back("steer_resolved_delay_s",real(static_cast<double>(ticks)*context.physics.control_period_s));
        }
    }
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
