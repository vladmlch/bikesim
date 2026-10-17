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
#include "../stepper.hpp"
#include "../terrain/trackfile.hpp"

namespace bike::initialization {
namespace {
using namespace runtime::setupvalue;

void populate_heightfield(mjModel& compiled, const SetupRequest& request) {
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
    auto target = std::views::counted(compiled.hfield_data, compiled.nhfielddata)
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
    return Value{Object{{"steps", integer(result.steps)},
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
        if (size > 512U * 1024U * 1024U)
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
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 15]);
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
    if (request.model.mode != "ride")
        throw std::invalid_argument("native setup requires model mode 'ride'");
    auto resolved_rider = model::resolve_rider(request.model.rider ? model::RiderArg{*request.model.rider}
        : model::RiderArg{std::monostate{}}, request.model.include_rider, "lumped");
    runtime::validate_native_supported(*request.model.physics, resolved_rider);
    if (!std::isfinite(request.start_x_m))
        throw std::invalid_argument("setup start_x_m must be finite");
    request.track.validate();
    request.model.field.validate();
    terrain::assert_track_fits(request.track, request.model.field);
    request.model.rider = resolved_rider;
    request.model.crank_joint = true;
    auto compiled = model::compile_mjcf(model::generate_mjcf(request.model), request.asset_root);
    populate_heightfield(*compiled, request);
    auto impl = std::make_unique<SetupOwner::Impl>();
    impl->config = runtime::project_runtime(request.model, *compiled, request.track, request.runtime);
    auto image = capture_model_image(*compiled);
    impl->stepper = std::make_unique<Stepper>(image.mjb, nativecfg::decode(impl->config.writer_config));
    impl->physical = runtime::make_physical_step(*impl->stepper, impl->config);
    impl->request = std::move(request);
    impl->context = std::make_unique<SetupContext>(SetupContext{
        *impl->stepper, *impl->physical, *impl->request.model.physics});
    auto& context = *impl->context;
    context.start_x_m = impl->request.start_x_m;
    context.ground_z_m = geometry::compute_ground_z(impl->request.model.bike) / 1000.;
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
            text(std::string(bike::rider::allocation_numeric_identity)));
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
