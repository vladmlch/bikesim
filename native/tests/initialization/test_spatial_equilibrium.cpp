// test_spatial_equilibrium.cpp — S3a 3D two-phase zero-hold equilibrium tests.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "diag.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/equilibrium.hpp"
#include "initialization/refine.hpp"
#include "initialization/setup_owner.hpp"
#include "model/builder.hpp"
#include "model_access.hpp"
#include "rider/senses_codec.hpp"
#include "rider/state_codec.hpp"
#include "runtime/project_config.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/step.hpp"
#include "spatial/config.hpp"
#include "spatial/frames.hpp"
#include "spatial/types.hpp"
#include "stepper.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/grid3d.hpp"
#include "terrain/heightfield3d.hpp"
#include "terrain/profile.hpp"
#include "writer_config.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "writers/drivetrain.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

NATIVE_DIAG_PUSH
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace {
namespace cfg = bike::configuration;
namespace init = bike::initialization;
namespace mdl = bike::model;
namespace sp = bike::spatial;
namespace trn = bike::terrain;

sp::SpatialConfig make_spatial_config() {
    sp::SpatialConfig out{};
    out.tire_backend = sp::SpatialTireBackend::compliant_3d;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = sp::SpatialStartMode::moving;
    out.corridor_half_width_m = 1.5;
    out.capsize_roll_rad = 1.0;
    out.steer.range_deg = 45.0;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.010;
    out.steer.max_torque_nm = 35.0;
    return out;
}

cfg::SimulationPhysicsConfig make_spatial_physics() {
    auto phys = std::make_unique<cfg::SimulationPhysicsConfig>(cfg::default_physics());
    phys->physics_mode = "physical";
    phys->drive_mode = "crank_effort";
    phys->pitch_assist = false;
    phys->seated_climb.enabled = false;
    phys->articulated.pedal_attachment = "flat";
    phys->articulated.saddle_attachment = "flat";
    phys->articulated.grip_attachment = "spring";
    phys->drive.transmission_model = "ideal_mid_drive";
    phys->drive.motor_clutch = false;
    phys->drive.human_torque_nm = 0.0;
    phys->drive.shifting.enabled = false;
    phys->tires.backend = "compliant_2d";
    phys->tires.surface_mode = "track";
    phys->initial_speed_mps = 3.0;
    phys->initial_front_brake = 1.0;
    phys->initial_rear_brake = 1.0;
    phys->equilibrium_cache_enabled = false;
    phys->equilibrium_state_file.reset();
    cfg::post_init(*phys);
    return *phys;
}

struct SpatialScenario {
    std::string_view label;
    bool include_rider = false;
    double grade = 0.0;
    double bank_deg = 0.0;
    double start_x_m = 15.0;
};

init::SetupRequest build_spatial_request(const SpatialScenario& sc) {
    init::SetupRequest req;
    req.track = trn::flat(60.0);
    if (sc.grade != 0.0) {
        req.track.grade_profile = trn::GradeProfile{{{0.0, sc.grade}, {req.track.length_m, sc.grade}}};
    }
    req.start_x_m = sc.start_x_m;
    req.model.mode = "ride";
    req.model.include_rider = sc.include_rider;
    req.model.rider = mdl::rider_from_args(sc.include_rider ? "lumped" : "none");
    req.model.physics = make_spatial_physics();
    req.spatial = make_spatial_config();
    req.model.field = trn::HeightFieldSpec::for_track(req.track);

    if (sc.bank_deg != 0.0) {
        trn::Terrain3dOverlaySpec ov;
        ov.half_width_m = 1.5;
        ov.dx_m = 0.25;
        ov.dy_m = 0.25;
        ov.lead_in_m = 2.0;
        ov.lead_out_m = 2.0;
        ov.cross_slopes.push_back(trn::CrossSlopeSegment3d{
            .start_m = 4.0,
            .end_m = 52.0,
            .transition_m = 2.0,
            .bank_rad = sc.bank_deg * std::numbers::pi / 180.0,
        });
        req.terrain_overlay = ov;
    }
    return req;
}

void verify_zero_passive_damping_and_stiffness(const mjModel* m, std::string_view label) {
    const auto stiffness = model_access::readonly_buffer(m->jnt_stiffness, m->njnt, "jnt_stiffness");
    const auto damping = model_access::readonly_buffer(m->dof_damping, m->nv, "dof_damping");
    for (double const k : stiffness) {
        bt::require(k == 0.0, std::string(label) + ": jnt_stiffness remains zero");
    }
    for (double const c : damping) {
        bt::require(c == 0.0, std::string(label) + ": dof_damping remains zero");
    }
}

[[gnu::noinline]] void test_planar_shield_intact() {
    auto req = init::read_setup_request(bt::fixture("requests/welded_flat.toml"));
    if (!req.model.physics.has_value()) {
        bt::require(false, "welded_flat fixture carries physics config");
        return;
    }
    req.model.physics->equilibrium_cache_enabled = false;
    req.model.physics->equilibrium_state_file.reset();
    req.initial_state.reset();
    req.model.physics->initial_speed_mps = 0.0;
    const auto owner = init::make_setup(req);
    const auto res = init::settle(*owner);
    bt::require(res.steps > 0 && res.residual_qacc <= 0.05 && !res.cache_hit,
                "planar welded_flat settles cleanly through exact planar path");
    bt::require(res.static_front_load_n > 0.0 && res.static_rear_load_n > 0.0,
                "planar welded_flat reports positive static wheel loads");
}

[[gnu::noinline]] void test_setup_owner_spatial_matrix() {
    const std::array<SpatialScenario, 14> scenarios{{
        {.label = "none_flat", .include_rider = false, .grade = 0.00, .bank_deg = 0.0},
        {.label = "none_up15", .include_rider = false, .grade = 0.15, .bank_deg = 0.0},
        {.label = "none_up35", .include_rider = false, .grade = 0.35, .bank_deg = 0.0},
        {.label = "none_bank_pos4", .include_rider = false, .grade = 0.00, .bank_deg = 4.0},
        {.label = "none_bank_pos8", .include_rider = false, .grade = 0.00, .bank_deg = 8.0},
        {.label = "none_up35_bank_pos8", .include_rider = false, .grade = 0.35, .bank_deg = 8.0},
        {.label = "none_up20_bank_neg8", .include_rider = false, .grade = 0.20, .bank_deg = -8.0},
        {.label = "lumped_flat", .include_rider = true, .grade = 0.00, .bank_deg = 0.0},
        {.label = "lumped_up15", .include_rider = true, .grade = 0.15, .bank_deg = 0.0},
        {.label = "lumped_up35", .include_rider = true, .grade = 0.35, .bank_deg = 0.0},
        {.label = "lumped_bank_pos4", .include_rider = true, .grade = 0.00, .bank_deg = 4.0},
        {.label = "lumped_bank_pos8", .include_rider = true, .grade = 0.00, .bank_deg = 8.0},
        {.label = "lumped_up35_bank_pos8", .include_rider = true, .grade = 0.35, .bank_deg = 8.0},
        {.label = "lumped_up20_bank_neg8", .include_rider = true, .grade = 0.20, .bank_deg = -8.0},
    }};

    for (const auto& sc : scenarios) {
        const auto owner = init::make_setup(build_spatial_request(sc));
        const auto res = init::settle(*owner);
        bt::require(res.steps > 0 && std::isfinite(res.residual_qacc) &&
                        res.residual_qacc <= 0.05 && !res.cache_hit,
                    std::string(sc.label) + ": SetupOwner::settle achieves ||qacc||_inf <= 0.05");
        bt::require(res.static_front_load_n > 0.0 && res.static_rear_load_n > 0.0,
                    std::string(sc.label) + ": both wheels carry positive static load");

        const double expected_weight_n = (sc.include_rider ? 104.70 : 24.40) * 9.81;
        const double weight_err =
            std::abs(res.total_vertical_force_n - expected_weight_n) / expected_weight_n;
        bt::require(weight_err <= 0.01,
                    std::string(sc.label) + ": vertical tire force matches total weight within 1%");

        const auto state_file = owner->capture_state_file();
        bt::require(!state_file.arrays.empty(),
                    std::string(sc.label) + ": state file captured cleanly");
        const auto bootstrap = owner->seal_bootstrap();
        bt::require(bootstrap.model.sha256.size() == 64,
                    std::string(sc.label) + ": bootstrap sealed cleanly");
    }
}

[[gnu::noinline]] void test_direct_two_phase_invariants() {
    const std::array<SpatialScenario, 4> cases{{
        {.label = "direct_none_up35_bank8", .include_rider = false, .grade = 0.35, .bank_deg = 8.0},
        {.label = "direct_lumped_up35_bank8", .include_rider = true, .grade = 0.35, .bank_deg = 8.0},
        {.label = "direct_lumped_up15_bank_neg8", .include_rider = true, .grade = 0.15, .bank_deg = -8.0},
        {.label = "direct_lumped_flat", .include_rider = true, .grade = 0.0, .bank_deg = 0.0},
    }};

    for (const auto& sc : cases) {
        auto resolved_req = build_spatial_request(sc);
        if (!resolved_req.spatial.has_value() || !resolved_req.model.physics.has_value()) {
            bt::require(false, "spatial request requires spatial and physics configs");
            return;
        }
        const sp::SpatialConfig spatial_cfg = *resolved_req.spatial;
        const cfg::SimulationPhysicsConfig physics_cfg = *resolved_req.model.physics;
        resolved_req.model.spatial = spatial_cfg;
        const double ground_z_m = bike::geometry::compute_ground_z(resolved_req.model.bike) / 1000.;
        const auto tire_cfg = make_compliant_3d_tire_config(spatial_cfg.tire_preset);
        resolved_req.model.bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.;
        resolved_req.model.bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.;
        resolved_req.model.crank_joint = true;

        std::unique_ptr<trn::Terrain3dSource> terrain3d;
        std::unique_ptr<trn::Grid3d> hfield_grid;
        trn::HeightField3dBuild hf{};
        if (resolved_req.terrain_overlay.has_value()) {
            trn::Terrain3dOverlaySpec ov = *resolved_req.terrain_overlay;
            if (ov.datum_z_offset_m == 0.) ov.datum_z_offset_m = ground_z_m;
            hfield_grid = std::make_unique<trn::Grid3d>(trn::build_overlay_grid3d(resolved_req.track, ov));
            hf = trn::build_heightfield_3d(*hfield_grid, ground_z_m);
            terrain3d = std::make_unique<trn::Terrain3dSource>(*hfield_grid);
        } else {
            const trn::ExtrudedTrack3d ext(resolved_req.track, spatial_cfg.corridor_half_width_m, ground_z_m);
            const trn::Grid3dSpec gs = trn::make_grid3d_spec(ext.bounds(), trn::DEFAULT_RESOLUTION_M, 0.25);
            hfield_grid = std::make_unique<trn::Grid3d>(trn::sample_to_grid3d(ext, gs));
            hf = trn::build_heightfield_3d(*hfield_grid, ground_z_m);
            terrain3d = std::make_unique<trn::Terrain3dSource>(ext);
        }
        resolved_req.model.field = hf.spec;
        const auto compiled = mdl::compile_mjcf(mdl::generate_mjcf(resolved_req.model));
        trn::populate_heightfield_3d(*compiled, *hfield_grid, hf);
        const auto rt_cfg = runtime::project_runtime(resolved_req.model, *compiled, resolved_req.track);

        auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(nativecfg::decode(rt_cfg.writer_config));
        writer_cfg->tire.reset();
        writer_cfg->cruise.reset();
        writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
            spatial_cfg.tire_preset, make_terrain3d_callbacks(*terrain3d));
        if (writer_cfg->resistance.has_value()) writer_cfg->resistance->allow_spatial = true;

        Stepper stepper(init::capture_model_image(*compiled).mjb, *writer_cfg);
        const mjModel* m = stepper.model();
        const int fw_dof = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr")
            [static_cast<std::size_t>(model_access::resolve_id(m, mjOBJ_JOINT, "front_wheel_spin"))];
        const int rw_dof = model_access::readonly_buffer(m->jnt_dofadr, m->njnt, "jnt_dofadr")
            [static_cast<std::size_t>(model_access::resolve_id(m, mjOBJ_JOINT, "rear_wheel_spin"))];
        const auto& drv = stepper.drive().config();
        const runtime::StaticBrake brake(
            fw_dof, rw_dof, drv.brake_ceiling_nm,
            runtime::BrakeHoldSpec{
                .torque_nm = drv.brake_hold_torque_nm,
                .stiffness_nm_rad = drv.brake_hold_stiffness_nm_rad,
                .damping_nms_rad = drv.brake_hold_damping_nms_rad,
                .engage_radps = drv.brake_hold_engage_radps,
                .release_radps = drv.brake_hold_release_radps,
                .unified_ceiling = true});

        runtime::StepConfig step_cfg;
        step_cfg.timestep_s = rt_cfg.timestep_s;
        step_cfg.control_period_s = rt_cfg.control_period_s;
        step_cfg.measurement_decimation = 1;
        step_cfg.crank_length_m = rt_cfg.geometry.crank_length_m;
        step_cfg.road_lookahead_m = 2.0;
        step_cfg.rider_present = false;
        step_cfg.monitors = rt_cfg.monitors;
        step_cfg.intent = rider::decode_intent_config(rt_cfg.rider_intent, "config.rider_intent");
        step_cfg.reaction = rider::decode_reaction_config(rt_cfg.rider_reaction, "config.rider_reaction");
        step_cfg.launch = rider::decode_launch_config(rt_cfg.rider_launch, "config.rider_launch");
        step_cfg.surface_labels = rider::decode_surface_labels(rt_cfg.surface_labels, "config.surface_labels");
        step_cfg.spatial = resolved_req.spatial;

        std::vector<std::array<double, 2>> vertices = trn::profile(resolved_req.track, 0.25);
        for (auto& pt : vertices) pt[1] += ground_z_m;
        runtime::PhysicalStep physical(stepper, std::move(step_cfg), std::move(vertices), brake, std::nullopt, std::nullopt);

        init::SetupContext ctx{
            .stepper = stepper,
            .physical = physical,
            .physics = physics_cfg,
            .start_x_m = resolved_req.start_x_m,
            .ground_z_m = ground_z_m,
            .frame_body_id = model_access::resolve_id(m, mjOBJ_BODY, "frame"),
            .front_geom_id = model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact"),
            .rear_geom_id = model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact"),
            .rear_travel_from_shock_mm = [](double s) { return s * 2.5; },
        };

        // 1. Verify initial_pose placement (phi == 0 in world gravity, yaw == 0, steer == 0, tires clear).
        init::initialize_pose(ctx);
        const auto free_root = init::find_free_root(m);
        if (!free_root.has_value()) {
            bt::require(false, "spatial model must have root_free");
            return;
        }
        const auto fq = static_cast<std::size_t>(free_root->qpos);
        const auto q_init = stepper.qpos();
        const sp::Quat quat_init{q_init[fq + 3U], q_init[fq + 4U], q_init[fq + 5U], q_init[fq + 6U]};
        const auto eul_init = sp::euler_zyx(quat_init);
        bt::require(std::abs(q_init[fq] - resolved_req.start_x_m) <= 1e-12 &&
                        std::abs(q_init[fq + 1U]) <= 1e-12 &&
                        std::abs(eul_init.roll_rad) <= 1e-12 &&
                        std::abs(eul_init.yaw_rad) <= 1e-12,
                    std::string(sc.label) + ": initial_pose seeds upright phi=0, y=0, yaw=0");
        bt::require(std::abs(eul_init.pitch_rad - (-std::atan(sc.grade))) <= 1e-6,
                    std::string(sc.label) + ": initial_pose pitch matches longitudinal grade");

        // 2. Verify Phase A hold activation and Phase B zero-hold convergence.
        bt::require(!init::phase_a_hold_active(ctx), "Phase A hold inactive before scope");
        {
            const init::PhaseAHoldScope scope(ctx);
            bt::require(init::phase_a_hold_active(ctx), "Phase A hold active inside scope");
        }
        bt::require(!init::phase_a_hold_active(ctx), "Phase A hold inactive after scope exit");

        const auto eq_res = init::settle_context(ctx);
        bt::require(!init::phase_a_hold_active(ctx),
                    std::string(sc.label) + ": Phase A hold remains off after settle_context");
        bt::require(eq_res.residual_qacc <= 0.05,
                    std::string(sc.label) + ": Phase B ||qacc||_inf <= 0.05");

        const auto moments = init::spatial_equilibrium_moments(ctx);
        bt::require(std::abs(moments.roll_nm) <= 0.1 &&
                        std::abs(moments.pitch_nm) <= 0.1 &&
                        std::abs(moments.steer_nm) <= 0.1,
                    std::string(sc.label) + ": |M_x|, |M_y|, |tau_steer| <= 0.1 N*m");

        // Verify zero external hold in Phase B by re-evaluating apply_initial_forces(false).
        const auto q_before = std::vector<double>(stepper.qpos().begin(), stepper.qpos().end());
        init::apply_initial_forces(ctx, false);
        stepper.forward();
        bt::require(init::acceleration_residual(ctx) <= 0.05,
                    std::string(sc.label) + ": zero-hold re-evaluation preserves ||qacc||_inf <= 0.05");
        const auto q_after = stepper.qpos();
        bt::require(std::abs(q_after[fq + 1U] - q_before[fq + 1U]) == 0.0,
                    std::string(sc.label) + ": qpos invariant on zero-hold re-evaluation");

        // Verify Phase B polish keeps horizontal X/Y, yaw, and wheel spin locked from post-Phase-A.
        const sp::Quat quat_final{q_after[fq + 3U], q_after[fq + 4U], q_after[fq + 5U], q_after[fq + 6U]};
        const auto eul_final = sp::euler_zyx(quat_final);
        bt::require(std::abs(eul_final.yaw_rad) <= 0.05 && std::abs(q_after[fq + 1U]) <= 0.02,
                    std::string(sc.label) + ": settled pose stays near corridor y~0, yaw~0");

        verify_zero_passive_damping_and_stiffness(m, sc.label);
    }
}

} // namespace
NATIVE_DIAG_POP

int main() {
    return bt::run([] {
        test_planar_shield_intact();
        test_setup_owner_spatial_matrix();
        test_direct_two_phase_invariants();
    });
}
