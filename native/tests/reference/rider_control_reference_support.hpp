// rider_control_reference_support.hpp — shared plumbing for the batch-3.15
// rider-control ports:
//
//   tests/reference/test_leg_loop.py
//   tests/reference/test_joint_strength.py
//   tests/reference/test_planar_arms.py
//   tests/reference/test_closed_form_rider.py
//   tests/reference/test_coupled_rider_task.py
//   tests/reference/test_native_rider_intent.py
//   tests/reference/test_native_spindle_controller.py
//
// Harness only: the _compiled/_model plant helpers from
// tests/reference/test_pinned_topology.py and the seeded-env rig the
// environment-dependent records step, plus exception-message checks. The
// pinned fixture physics (tests/fixtures/migration/config/
// viewer_physics_welded.toml) already ships the oracle's
// spindle/pin/connect attachment triple, so no attachment rewrite applies;
// the compiled/env paths mirror _pinned_config/_compiled/_model verbatim.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <source_location>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "support/golden.hpp"
#include "support/migration_test.hpp"

#include "configuration/fields.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "engine_call.hpp"
#include "engine_identity.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/equilibrium.hpp"
#include "initialization/finalize.hpp"
#include "initialization/setup_owner.hpp"
#include "io/hash.hpp"
#include "kinematics/solver.hpp"
#include "model/builder.hpp"
#include "model/export.hpp"
#include "model/mapping.hpp"
#include "model/rider_geometry.hpp"
#include "platform/provenance.hpp"
#include "research/environment.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/state_codec.hpp"
#include "runtime/config.hpp"
#include "runtime/project_config.hpp"
#include "runtime/provenance.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "terrain/profile.hpp"
#include "terrain/trackfile.hpp"
#include "writer_config.hpp"
#include "writers/suspension.hpp"

namespace rider_control_reference {

namespace cfg = bike::configuration;
namespace init = bike::initialization;
namespace mdl = bike::model;
namespace rcfg = ::runtime;

// pytest.approx defaults (rel=1e-6, abs=1e-12) unless a call site narrows.
[[nodiscard]] inline bool approx(double actual, double expected,
                                 double rtol = 1e-6, double atol = 1e-12) {
    return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}

// np.allclose semantics: |a-b| <= atol + rtol*|b| elementwise.
inline void allclose(std::span<const double> actual,
                     std::span<const double> expected, double rtol,
                     double atol, std::string_view what) {
    bt::require(actual.size() == expected.size(),
                std::string(what) + ": shapes agree");
    for (std::size_t i = 0; i < actual.size(); ++i)
        bt::require(approx(actual[i], expected[i], rtol, atol),
                    std::string(what) + " element " + std::to_string(i) +
                        " is within tolerance");
}

// pytest.raises(ValueError, match=needle): the native ValueError surface is
// std::invalid_argument; the message must carry the matched substring.
template <class F>
inline void expect_invalid(
    F &&action, std::string_view needle,
    const std::source_location &loc = std::source_location::current()) {
    bool thrown = false;
    try {
        std::forward<F>(action)();
    } catch (const std::invalid_argument &failure) {
        thrown = true;
        bt::require(needle.empty() ||
                        std::string_view(failure.what()).find(needle) !=
                            std::string_view::npos,
                    (std::string("exception text lacks '") +=
                     std::string(needle)) +=
                        (std::string("':") + failure.what()));
    }
    bt::require(thrown, (std::string("expected rejection at ") +=
                         loc.file_name()) += ":" +
                            std::to_string(loc.line()));
}

// The repository root — bt::oracle::root() is <repo>/tests/fixtures/golden.
[[nodiscard]] inline std::filesystem::path repo_file(std::string_view rel) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::string(rel);
}

// ---------------------------------------------------------------------
// _compiled (test_pinned_topology.py:33-46): the physics document loads the
// pinned attachments verbatim; joint_envelope_path/joint_strength_path stay
// relative and resolve inside load_physics against the file's directory —
// the oracle's .resolve() rewrite only re-anchors them for tmp_path. The
// controller build is resolve+geometry like ArticulatedRiderController's.
// ---------------------------------------------------------------------
struct PinnedPlant {
    mdl::BuildRequest request;
    mdl::ModelOwner model;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr,
                                                          mj_deleteData};
    std::unique_ptr<rider::SpindleController> controller;
    rider::SpindlePose pose;
};

[[nodiscard]] inline PinnedPlant
pinned_model(std::string_view physics_toml =
                 "examples/research/viewer_physics_welded.toml",
             const std::function<void(::rider::SpindleConfig &)>
                 &config_patch = {}) {
    PinnedPlant plant;
    plant.request.mode = "ride";
    plant.request.rider = cfg::RiderSpecs{};
    plant.request.rider->variant = "articulated_planar";
    plant.request.physics = cfg::load_physics(repo_file(physics_toml));
    plant.request.crank_joint = true;
    plant.model = mdl::compile_mjcf(mdl::generate_mjcf(plant.request));
    plant.data.reset(engine::make_data(plant.model.get()));
    engine::forward(plant.model.get(), plant.data.get());
    const auto pose =
        mdl::geometry_pose(*plant.request.rider, plant.request.bike);
    plant.pose = ::rider::decode_spindle_pose(
        rcfg::project_articulated_pose(pose), "pose");
    auto controller_config = ::rider::decode_spindle_config(
        rcfg::project_rider_controller(*plant.request.physics,
                                       *plant.model));
    if (config_patch)
        config_patch(controller_config);
    plant.controller = std::make_unique<rider::SpindleController>(
        plant.model.get(), plant.pose, std::move(controller_config),
        plant.request.bike.crank_length / 1000.);
    return plant;
}

// ---------------------------------------------------------------------
// _model (test_pinned_topology.py:24-30): research_cli.make_environment's
// owned plant on the pinned physics. The request seed/track anchor at the
// welded_flat fixture request — its physics_file is replaced with the live
// viewer_physics_welded document (the pinned spindle/pin/connect triple the
// oracle's _pinned_config rewrites onto) and its track with the oracle's
// --track-file rough_uphill_savage. skip_initial_state: the shipped
// welded_flat.json was captured for this same pinned topology, but
// make_environment constructs from the document alone — no state transfer
// — so the rig runs init::settle_context, the same braked physical
// relaxation + trust-region refine as the oracle's construction settle.
// ---------------------------------------------------------------------
struct EnvRig {
    std::unique_ptr<init::SetupRequest> request;
    std::unique_ptr<rcfg::RuntimeConfig> config;
    std::unique_ptr<::Stepper> stepper;
    std::unique_ptr<rcfg::PhysicalStep> physical;
    rider::SpindlePose pose;
    // The live SetupContext behind make_environment — kept because
    // PhysicalInitialState::capture/prepare (test_planar_arms'
    // env.sim.physical_initial_state round trip) read the same context
    // fields make_setup populates: identity/source/provider digests and
    // the completed equilibrium evidence.
    std::optional<init::SetupContext> context;
};

// NOLINTBEGIN(modernize-make-unique) same inlined-forwarding frame budget
// as review_rig/WeldedRig.
[[nodiscard]] inline std::unique_ptr<EnvRig>
pinned_rig(const std::function<void(cfg::SimulationPhysicsConfig &)>
               &physics_overrides = {},
           std::optional<bike::terrain::TrackSpec> track = std::nullopt,
           bool braked_relaxation = true) {
    auto rig = std::make_unique<EnvRig>();
    rig->request = std::unique_ptr<init::SetupRequest>{new init::SetupRequest(
        init::read_setup_request(bt::fixture("requests/welded_flat.toml"),
                                 true))};
    auto &model = rig->request->model;
    // _model's --physics-config: the live pinned document (F1-delivered
    // budgets and activation_tau_s included), not the stale fixture copy.
    model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    auto &physics = model.physics.value();
    // The test context owns no cache identity or directory; a fresh braked
    // relaxation converges to the same state the oracle's make_environment
    // settle produces on a cache miss.
    physics.equilibrium_cache_enabled = false;
    if (physics_overrides)
        physics_overrides(physics);
    cfg::post_init(physics);
    // _model's --track-file examples/research/rough_uphill_savage.toml; the
    // request schema defaults model.field to FIELD, which only spans the
    // flat fixture — the env derivation (build_field_data(track)) sizes the
    // heightfield to the loaded profile instead.
    rig->request->track =
        track.has_value()
            ? *track
            : bike::terrain::load_track(
                  repo_file("examples/research/rough_uphill_savage.toml"));
    model.field = bike::terrain::HeightFieldSpec::for_track(
        rig->request->track);
    const auto resolved_rider = bike::model::resolve_rider(
        model.rider ? bike::model::RiderArg{*model.rider}
                    : bike::model::RiderArg{std::monostate{}},
        model.include_rider, "lumped");
    model.rider = resolved_rider;
    model.crank_joint = true;
    auto compiled = bike::model::compile_mjcf(
        bike::model::generate_mjcf(model), rig->request->asset_root);
    const auto field = static_cast<std::size_t>(
        bike::model::required_id(*compiled, mjOBJ_HFIELD, "road"));
    const auto address = std::views::counted(compiled->hfield_adr,
                                             compiled->nhfield)[field];
    const auto samples =
        bike::terrain::build_field_data(rig->request->track, model.field);
    bt::require(address >= 0 &&
                    std::cmp_less_equal(address, compiled->nhfielddata) &&
                    samples.size() <=
                        static_cast<std::size_t>(compiled->nhfielddata -
                                                 address),
                "the pinned fixture has valid compiled road storage");
    const auto heightfield =
        std::views::counted(compiled->hfield_data, compiled->nhfielddata)
            .subspan(static_cast<std::size_t>(address), samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i)
        heightfield[i] = static_cast<float>(samples[i]);
    rig->config = std::unique_ptr<rcfg::RuntimeConfig>{
        new rcfg::RuntimeConfig(
            rcfg::project_runtime(model, *compiled, rig->request->track))};
    const auto image = init::capture_model_image(*compiled);
    const auto writers = std::unique_ptr<nativecfg::WriterConfig>{
        new nativecfg::WriterConfig(
            nativecfg::decode(rig->config->writer_config))};
    rig->stepper = std::make_unique<::Stepper>(image.mjb, *writers);
    rig->physical = rcfg::make_physical_step(*rig->stepper, *rig->config);
    auto &context = rig->context.emplace(init::SetupContext{
        .stepper = *rig->stepper,
        .physical = *rig->physical,
        .physics = physics});
    context.start_x_m = rig->request->start_x_m;
    context.ground_z_m =
        bike::geometry::compute_ground_z(model.bike) / 1000.;
    context.frame_body_id =
        bike::model::required_id(*compiled, mjOBJ_BODY, "frame");
    context.front_geom_id = bike::model::required_id(
        *compiled, mjOBJ_GEOM, "geom_front_contact");
    context.rear_geom_id = bike::model::required_id(
        *compiled, mjOBJ_GEOM, "geom_rear_contact");
    if (rig->physical->rider_present()) {
        rig->pose = ::rider::decode_spindle_pose(
            rig->config->geometry.pose, "pinned.pose");
        context.rider_pose = rig->pose;
    }
    auto solver =
        model.solver
            ? std::make_shared<bike::kinematics::HorstLinkageSolver>(
                  *model.solver)
            : std::make_shared<bike::kinematics::HorstLinkageSolver>(
                  model.bike);
    context.rear_travel_from_shock_mm =
        [solver = std::move(solver)](double stroke) {
            return solver->solve_state_from_shock_stroke(stroke)
                .wheel_travel;
        };
    // make_setup's provenance quartet (setup_owner.cpp:193-208):
    // PhysicalInitialState::capture binds the model contract to these
    // digests, and prepare() re-derives them on restore.
    context.identity = bike::io::Value{bike::io::Value::Object{
        {"specs", bike::model::bike_specs_value(model.bike)},
        {"mass_specs", bike::model::mass_specs_value(model.masses)},
        {"rider", cfg::detail::plain_document(
                      cfg::detail::as_object(cfg::detail::kRiderSpecsSchema,
                                             &resolved_rider))},
        {"track", bike::terrain::identity_value(rig->request->track)}}};
    context.source_identity =
        ::runtime::runtime_identity().native_source_sha256;
    const auto engine_id = ::engine::identity();
    const auto provider = engine_id.mujoco_version + "|" +
                          engine_id.library_sha256 + "|" +
                          engine_id.numeric_provider + "|" +
                          bike::platform::execution_identity();
    context.provider_identity =
        bike::io::sha256(std::as_bytes(std::span{provider}));
    context.model_sha256 = image.sha256;
    // make_environment's construction settle: the full braked physical
    // relaxation (initialize_pose + damped stepping + trust-region refine),
    // not a bare pose. Records that read tire loads, intent signals, or
    // launch dynamics depend on the wheels actually carrying weight at t=0.
    if (braked_relaxation) {
        const auto settled = init::settle_context(context);
        (void)settled;
    } else {
        // Spec Q13: the 64000-step braked relaxation used dof damping for
        // ankle-oscillation dissipation — at damping 0 the all-welded plant
        // cannot converge, a real finding of the knob's removal. The light
        // init path mirrors test_native_coupled_rider_task.cpp's rig:
        // initialize_pose + recorded residual + finalize, no relaxation.
        init::initialize_pose(context);
        init::EquilibriumResult result;
        result.residual_qacc = init::acceleration_residual(context);
        const auto settled_qpos = rig->stepper->qpos();
        result.fork_travel_mm =
            1000. * settled_qpos[static_cast<std::size_t>(
                rig->stepper->suspension().fork_qposadr())];
        result.shock_stroke_mm =
            1000. * settled_qpos[static_cast<std::size_t>(
                rig->stepper->suspension().shock_qposadr())];
        result.rear_travel_mm =
            context.rear_travel_from_shock_mm(result.shock_stroke_mm);
        result.root_z_m = settled_qpos[static_cast<std::size_t>(
            init::joint_address(context, "root_z").qpos)];
        result.pitch_rad = settled_qpos[static_cast<std::size_t>(
            init::joint_address(context, "root_pitch").qpos)];
        result.initial_crank_phase_rad = physics.drive.crank_phase_rad;
        result.settled_crank_phase_rad =
            settled_qpos[static_cast<std::size_t>(
                init::joint_address(context, "crank_spin").qpos)];
        context.equilibrium = result;
    }
    init::finalize_initial_state(context, *rig->config);
    context.initializing = false;
    return rig;
}
// NOLINTEND(modernize-make-unique)

// env.step(): one advance_physics interval per physical control-clock tick
// — the oracle's count * control_steps loop, idling like test_rider_
// allocation's advance_env_steps (research control period .01 s).
[[nodiscard]] inline rcfg::RideControl idle_control() {
    // Python RideControl(): every torque channel None — motor gets the
    // configured pedelec assistance and human effort stays automatic
    // (rider intent drives the crank). Setting a numeric value would
    // pin the channel and disable automatic effort.
    return rcfg::RideControl{};
}

inline rcfg::RawStep advance_env_steps(rcfg::PhysicalStep &physical,
                                     std::int64_t count) {
    const auto per = std::llround(
        bike::research::ExperimentConfig{}.control_period_s /
        physical.control_clock().timestep_s);
    rcfg::RawStep last;
    for (std::int64_t i = 0; i < count * per; ++i)
        // env.step defaults: front/rear brake demand 0 (research
        // environment.py:357), external qfrc None.
        last = physical.advance_physics(0., 0., std::nullopt,
                                        idle_control());
    return last;
}

} // namespace rider_control_reference
