// test_spatial_ride.cpp — S3d end-to-end 20-second 3D moving-start rides
// (none & lumped), SteerBalance runtime plumbing, CrashDetector thresholds,
// NativeRideRuntime root_free/ColumnLayout support, and 0-bit planar shield.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "diag.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/finalize.hpp"
#include "initialization/setup_owner.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "rider/senses_codec.hpp"
#include "rider/state_codec.hpp"
#include "runtime/accounting.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/config.hpp"
#include "runtime/control.hpp"
#include "runtime/project_config.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/step.hpp"
#include "spatial/frames.hpp"
#include "spatial/types.hpp"
#include "stepper.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/grid3d.hpp"
#include "terrain/heightfield3d.hpp"
#include "terrain/profile.hpp"
#include "terrain/source3d.hpp"
#include "writer_config.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "writers/drivetrain.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
namespace cfg = bike::configuration;
namespace init = bike::initialization;
namespace mdl = bike::model;
namespace sp = bike::spatial;
namespace trn = bike::terrain;

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRad = kPi / 180.0;

sp::SpatialConfig make_spatial_config() {
    sp::SpatialConfig out{};
    out.tire_backend = sp::SpatialTireBackend::compliant_3d;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = sp::SpatialStartMode::moving;
    out.corridor_half_width_m = 1.5;
    out.capsize_roll_rad = 1.0;
    out.steer.range_deg = 45.0;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.100;
    out.steer.max_torque_nm = 35.0;
    return out;
}

cfg::SimulationPhysicsConfig make_ride_physics(double speed_mps,
                                               std::string_view drive_mode,
                                               double human_torque_nm,
                                               bool motor_clutch) {
    auto phys = std::make_unique<cfg::SimulationPhysicsConfig>(
        cfg::default_physics());
    phys->physics_mode = "physical";
    phys->drive_mode = std::string(drive_mode);
    phys->pitch_assist = false;
    phys->seated_climb.enabled = false;
    phys->articulated.pedal_attachment = "flat";
    phys->articulated.saddle_attachment = "flat";
    phys->articulated.grip_attachment = "spring";
    phys->drive.transmission_model = "ideal_mid_drive";
    phys->drive.motor_clutch = motor_clutch;
    phys->drive.human_torque_nm = human_torque_nm;
    phys->drive.shifting.enabled = true;
    phys->drive.shifting.target_cadence_min_rpm = 75.0;
    phys->drive.shifting.target_cadence_max_rpm = 85.0;
    phys->tires.backend = "compliant_2d";
    phys->tires.surface_mode = "track";
    phys->initial_speed_mps = speed_mps;
    cfg::post_init(*phys);
    return *phys;
}

int joint_qposadr(const mjModel &model, const char *name) {
    const int jid = model_access::resolve_id(&model, mjOBJ_JOINT, name);
    return model_access::readonly_buffer(model.jnt_qposadr, model.njnt,
                                         "jnt_qposadr")
        [static_cast<std::size_t>(jid)];
}

int joint_dofadr(const mjModel &model, const char *name) {
    const int jid = model_access::resolve_id(&model, mjOBJ_JOINT, name);
    return model_access::readonly_buffer(model.jnt_dofadr, model.njnt,
                                         "jnt_dofadr")
        [static_cast<std::size_t>(jid)];
}

struct SpatialRideRig {
    trn::TrackSpec track{};
    std::unique_ptr<trn::Terrain3dSource> terrain{};
    std::unique_ptr<runtime::RuntimeConfig> rt_cfg{};
    std::unique_ptr<Stepper> stepper{};
    std::unique_ptr<runtime::PhysicalStep> physical{};
    std::unique_ptr<cfg::SimulationPhysicsConfig> physics{};
    std::string_view rider_variant = "none";
    double ground_z_m = 0.0;
    int root_free_qpos = 0;
    int root_free_dof = 0;
    int steer_qpos = 0;
    int steer_dof = 0;
};

struct RideScenarioSpec {
    std::string_view label;
    std::string_view rider_variant = "none";
    double track_length_m = 160.0;
    double grade = 0.0;
    double cross_slope_deg = 0.0;
    double start_x_m = 10.0;
    double y0_m = 0.0;
    double v0_mps = 5.0;
    double human_torque_nm = 0.0;
    double motor_torque_nm = 0.0;
    double max_abs_y_end_m = 0.05;
    std::string_view drive_mode = "coast";
    bool motor_clutch = false;
    int steps = 16000;
};

// NOLINTBEGIN(modernize-make-unique) keep stack frame small
std::unique_ptr<SpatialRideRig> build_ride_rig(const RideScenarioSpec &spec) {
    auto rig = std::make_unique<SpatialRideRig>();
    rig->rider_variant = spec.rider_variant;
    rig->track = trn::flat(spec.track_length_m);
    if (spec.grade != 0.0) {
        rig->track.grade_profile = trn::GradeProfile{
            {{0.0, spec.grade}, {spec.track_length_m, spec.grade}}};
    }

    rig->physics = std::make_unique<cfg::SimulationPhysicsConfig>(
        make_ride_physics(spec.v0_mps, spec.drive_mode, spec.human_torque_nm,
                          spec.motor_clutch));

    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args(spec.rider_variant);
    req->include_rider = (spec.rider_variant != "none");
    req->physics = *rig->physics;
    req->spatial = make_spatial_config();

    const auto tire_cfg =
        make_compliant_3d_tire_config(req->spatial->tire_preset);
    req->bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.0;
    req->bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.0;
    rig->ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.0;

    std::unique_ptr<trn::Grid3d> hfield_grid;
    trn::HeightField3dBuild hf{};
    if (spec.cross_slope_deg != 0.0) {
        trn::Terrain3dOverlaySpec ov;
        ov.half_width_m = req->spatial->corridor_half_width_m;
        ov.dx_m = 0.25;
        ov.dy_m = 0.25;
        ov.lead_in_m = 2.0;
        ov.lead_out_m = 2.0;
        ov.datum_z_offset_m = rig->ground_z_m;
        ov.cross_slopes.push_back(trn::CrossSlopeSegment3d{
            .start_m = 4.0,
            .end_m = spec.track_length_m - 6.0,
            .transition_m = 2.0,
            .bank_rad = spec.cross_slope_deg * kDegToRad,
        });
        hfield_grid = std::make_unique<trn::Grid3d>(
            trn::build_overlay_grid3d(rig->track, ov));
        hf = trn::build_heightfield_3d(*hfield_grid, rig->ground_z_m);
        rig->terrain = std::make_unique<trn::Terrain3dSource>(*hfield_grid);
    } else {
        const trn::ExtrudedTrack3d ext(
            rig->track, req->spatial->corridor_half_width_m, rig->ground_z_m,
            0.25);
        const trn::Grid3dSpec gs =
            trn::make_grid3d_spec(ext.bounds(), 0.25, 0.25);
        hfield_grid =
            std::make_unique<trn::Grid3d>(trn::sample_to_grid3d(ext, gs));
        hf = trn::build_heightfield_3d(*hfield_grid, rig->ground_z_m);
        rig->terrain = std::make_unique<trn::Terrain3dSource>(ext);
    }
    req->field = hf.spec;
    req->crank_joint = true;

    auto compiled = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    trn::populate_heightfield_3d(*compiled, *hfield_grid, hf);
    rig->rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, rig->track));
    const auto image = init::capture_model_image(*compiled);

    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(rig->rt_cfg->writer_config));
    writer_cfg->tire.reset();
    writer_cfg->cruise.reset();
    writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
        req->spatial->tire_preset, make_terrain3d_callbacks(*rig->terrain));
    if (writer_cfg->resistance.has_value()) {
        writer_cfg->resistance->allow_spatial = true;
    }

    rig->stepper = std::make_unique<Stepper>(image.mjb, *writer_cfg);
    const mjModel &m = *rig->stepper->model();
    rig->root_free_qpos = joint_qposadr(m, "root_free");
    rig->root_free_dof = joint_dofadr(m, "root_free");
    rig->steer_qpos = joint_qposadr(m, "steer_joint");
    rig->steer_dof = joint_dofadr(m, "steer_joint");

    runtime::StepConfig step_cfg;
    step_cfg.timestep_s = rig->rt_cfg->timestep_s;
    step_cfg.control_period_s = rig->rt_cfg->control_period_s;
    step_cfg.measurement_decimation = 1;
    step_cfg.strict = false;
    step_cfg.crank_length_m = rig->rt_cfg->geometry.crank_length_m;
    step_cfg.road_lookahead_m = 2.0;
    step_cfg.rider_present = false;
    step_cfg.monitors = rig->rt_cfg->monitors;
    step_cfg.intent = rider::decode_intent_config(
        rig->rt_cfg->rider_intent, "config.rider_intent");
    step_cfg.reaction = rider::decode_reaction_config(
        rig->rt_cfg->rider_reaction, "config.rider_reaction");
    step_cfg.launch = rider::decode_launch_config(
        rig->rt_cfg->rider_launch, "config.rider_launch");
    step_cfg.surface_labels = rider::decode_surface_labels(
        rig->rt_cfg->surface_labels, "config.surface_labels");
    step_cfg.spatial = rig->rt_cfg->spatial;

    const auto &drv = rig->stepper->drive().config();
    const runtime::StaticBrake brake(
        joint_dofadr(m, "front_wheel_spin"), joint_dofadr(m, "rear_wheel_spin"),
        drv.brake_ceiling_nm,
        runtime::BrakeHoldSpec{
            .torque_nm = drv.brake_hold_torque_nm,
            .stiffness_nm_rad = drv.brake_hold_stiffness_nm_rad,
            .damping_nms_rad = drv.brake_hold_damping_nms_rad,
            .engage_radps = drv.brake_hold_engage_radps,
            .release_radps = drv.brake_hold_release_radps,
            .unified_ceiling = true});

    std::vector<std::array<double, 2>> vertices = trn::profile(rig->track, 0.25);
    for (auto &pt : vertices) {
        pt[1] += rig->ground_z_m;
    }
    rig->physical = std::make_unique<runtime::PhysicalStep>(
        *rig->stepper, std::move(step_cfg), std::move(vertices), brake,
        std::nullopt, std::nullopt);
    return rig;
}
// NOLINTEND(modernize-make-unique)

void settle_and_finalize_moving_start(const SpatialRideRig &rig,
                                      const RideScenarioSpec &spec) {
    const mjModel *m = rig.stepper->model();
    const mjData *d = rig.stepper->data();
    const auto qa = static_cast<std::size_t>(rig.root_free_qpos);
    const auto da = static_cast<std::size_t>(rig.root_free_dof);

    {
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        std::ranges::fill(qpos, 0.0);
        std::ranges::fill(qvel, 0.0);

        const auto surf = rig.terrain->sample(spec.start_x_m, spec.y0_m);
        bt::require(surf.status == trn::QueryStatus3d::ok,
                    std::string(spec.label) + ": initial terrain sample ok");
        const double slope_x = -surf.normal_world[0] / surf.normal_world[2];
        const double pitch0 = -std::atan(slope_x);
        const sp::Quat q_att = sp::quat_from_euler_zyx(0.0, pitch0, 0.0);
        qpos[qa + 0U] = spec.start_x_m;
        qpos[qa + 1U] = spec.y0_m;
        qpos[qa + 2U] = surf.height_m - rig.ground_z_m + 0.005;
        qpos[qa + 3U] = q_att[0];
        qpos[qa + 4U] = q_att[1];
        qpos[qa + 5U] = q_att[2];
        qpos[qa + 6U] = q_att[3];
        rig.stepper->note_posvel_stale();
        rig.stepper->forward();

        const int fg =
            model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact");
        const int rg =
            model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact");
        const auto xpos =
            model_access::readonly_buffer(d->geom_xpos, 3 * m->ngeom, "xpos");
        const auto tire_cfg =
            make_compliant_3d_tire_config(sp::SpatialTirePreset::mtb_2p8_1p2bar);
        const std::array<double, 2> radii{
            tire_cfg.front.outer_radius_m, tire_cfg.rear.outer_radius_m};
        const std::array<int, 2> geoms{fg, rg};
        double max_shift_z = -1e9;
        for (std::size_t s = 0; s < 2; ++s) {
            const auto g = static_cast<std::size_t>(geoms[s]);
            const trn::Vec3d center{xpos[3 * g], xpos[3 * g + 1], xpos[3 * g + 2]};
            const auto q = rig.terrain->closest_point(center, 1.0);
            bt::require(q.status == trn::QueryStatus3d::ok,
                        std::string(spec.label) + ": closest_point ok");
            const double cur_delta = radii[s] - q.signed_distance_m;
            const double dz =
                (cur_delta - 0.002) / std::max(0.2, q.normal_world[2]);
            max_shift_z = std::max(max_shift_z, dz);
        }
        qpos[qa + 2U] += max_shift_z;
        rig.stepper->note_posvel_stale();
        rig.stepper->forward();
        bool drive_dirty = false;
        rig.stepper->mutate([&] {
            rig.stepper->drive().reset();
            drive_dirty = rig.stepper->drive().take_model_dirty();
        });
        if (drive_dirty) {
            rig.stepper->note_posvel_stale();
            rig.stepper->forward();
        }
        rig.stepper->tire_writer().reset();
        rig.stepper->tire_writer().restart_clock();
    }

    // Brief Phase-A kinetic settle under initializing = true (SteerBalance
    // MUST remain disengaged while initializing == true).
    auto init_state =
        std::make_unique<runtime::StepState>(rig.physical->state());
    init_state->initializing = true;
    rig.physical->restore(*init_state);

    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::vector<double> hold(nv, 0.0);
    const runtime::RideControl idle{};
    const double dt = rig.rt_cfg->timestep_s;
    const double max_d_roll = 0.4 * 2.0 / dt;
    const double max_d_y = 0.4 * 20.0 / dt;
    const double max_d_steer = 0.4 * 0.14 / dt;
    for (int k = 0; k < 400; ++k) {
        const auto qp = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
        const auto qv = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
        const sp::Quat q_curr{
            qp[qa + 3U], qp[qa + 4U], qp[qa + 5U], qp[qa + 6U]};
        const sp::EulerZyx eul = sp::euler_zyx(q_curr);
        const sp::Vec3 omega_b{qv[da + 3U], qv[da + 4U], qv[da + 5U]};
        const sp::EulerRatesZyx rates =
            sp::body_gyro_to_euler_zyx_rates(eul, omega_b);

        std::ranges::fill(hold, 0.0);
        hold[da + 1U] =
            -15000.0 * (qp[qa + 1U] - spec.y0_m) -
            std::min(1500.0, max_d_y) * qv[da + 1U];
        hold[da + 3U] =
            -4000.0 * eul.roll_rad -
            std::min(400.0, max_d_roll) * rates.roll_rate_rad_s;
        const double tau_yaw =
            -1500.0 * eul.yaw_rad - 150.0 * rates.yaw_rate_rad_s;
        const sp::Vec3 tau_yaw_b =
            sp::rotate_world_to_body(q_curr, {0.0, 0.0, tau_yaw});
        hold[da + 3U] += tau_yaw_b[0];
        hold[da + 4U] += tau_yaw_b[1];
        hold[da + 5U] += tau_yaw_b[2];
        hold[static_cast<std::size_t>(rig.steer_dof)] =
            -500.0 * qp[static_cast<std::size_t>(rig.steer_qpos)] -
            std::min(15.0, max_d_steer) *
                qv[static_cast<std::size_t>(rig.steer_dof)];

        (void)rig.physical->apply_forces(false, true, 1.0, 1.0, hold, idle);
        if (k == 0) {
            const auto probe_init = rig.physical->probe_step_inputs(idle, 1.0, 1.0);
            const bool has_steer_comp = std::ranges::any_of(
                probe_init.components, [](const auto &entry) {
                    return entry.first == "act_rider_steer";
                });
            bt::require(
                !has_steer_comp && rig.physical->direct_steer_torque_nm() == 0.0,
                std::string(spec.label) +
                    ": SteerBalance stays inactive while initializing == true");
        }
        rig.stepper->mutate([&] { engine::step(rig.stepper->model(), rig.stepper->data()); });
        rig.stepper->note_posvel_stale();
        if (k % 40 == 39) {
            const auto qv_mut =
                model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            std::ranges::fill(qv_mut, 0.0);
            rig.stepper->note_posvel_stale();
            rig.stepper->forward();
        }
    }

    init::SetupContext ctx{
        .stepper = *rig.stepper,
        .physical = *rig.physical,
        .physics = *rig.physics,
        .start_x_m = spec.start_x_m,
        .ground_z_m = rig.ground_z_m,
        .frame_body_id = model_access::resolve_id(m, mjOBJ_BODY, "frame"),
        .front_geom_id =
            model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact"),
        .rear_geom_id =
            model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact"),
    };
    init::EquilibriumResult eq{};
    eq.steps = 400;
    eq.residual_qacc = 0.0;
    ctx.equilibrium = eq;
    init::finalize_initial_state(ctx, *rig.rt_cfg);
    bt::require(ctx.completed,
                std::string(spec.label) + ": finalize_initial_state completed");
}

[[gnu::noinline]] void run_20s_ride_scenario(const RideScenarioSpec &spec) {
    auto rig = build_ride_rig(spec);
    bt::require(rig->physical->is_spatial(),
                std::string(spec.label) + ": PhysicalStep is spatial");
    bt::require(rig->physical->has_steer_balance(),
                std::string(spec.label) + ": PhysicalStep engages SteerBalance");
    bt::require(
        rig->physical->crash_detector().has_root_free() &&
            std::abs(rig->physical->crash_detector().capsize_roll_rad() - 1.0) <
                1e-12 &&
            std::abs(rig->physical->crash_detector().corridor_half_width_m() -
                     1.5) < 1e-12,
        std::string(spec.label) +
            ": CrashDetector reads capsize/corridor thresholds from StepConfig::spatial");

    settle_and_finalize_moving_start(*rig, spec);

    const mjModel *m = rig->stepper->model();
    const mjData *d = rig->stepper->data();
    const auto [init_energy_j, scale_j] =
        rig->physical->measure_energy_and_scale();

    runtime::AccountingConfig acc_cfg{};
    acc_cfg.timestep_s = rig->rt_cfg->timestep_s;
    acc_cfg.period_steps =
        static_cast<std::size_t>(rig->rt_cfg->control_period_steps);
    acc_cfg.measurement_decimation = 1;
    acc_cfg.retention = runtime::RetentionMode::off;
    acc_cfg.async_telemetry = false;
    acc_cfg.max_qpos = static_cast<std::size_t>(m->nq);
    acc_cfg.max_dofs = static_cast<std::size_t>(m->nv);
    acc_cfg.columns = {
        .root_x_qpos = static_cast<std::size_t>(rig->root_free_qpos),
        .root_x_dof = static_cast<std::size_t>(rig->root_free_dof),
        .root_pitch_qpos = static_cast<std::size_t>(rig->root_free_qpos + 4)};

    runtime::AccountingState acc_init{};
    acc_init.energy.initial_energy_j = init_energy_j;
    acc_init.energy.energy_scale_j = scale_j;
    acc_init.energy.mechanical_energy_j = init_energy_j;
    acc_init.monitor.strict = false;

    runtime::PeriodAccounting accounting(
        acc_cfg, acc_init, /*generation=*/1, nullptr, nullptr);

    runtime::RideControl ctrl{};
    if (spec.drive_mode == "crank_effort") {
        if (spec.human_torque_nm > 0.0) {
            ctrl.human_torque_nm = spec.human_torque_nm;
        }
        if (spec.motor_torque_nm > 0.0) {
            ctrl.motor_torque_nm = spec.motor_torque_nm;
            ctrl.motor_limit_nm = spec.motor_torque_nm + 10.0;
        }
    }

    double max_abs_roll = 0.0;
    double max_residual_ratio = 0.0;
    bool saw_steer_component = false;
    const auto qa = static_cast<std::size_t>(rig->root_free_qpos);

    for (int step = 0; step < spec.steps; ++step) {
        runtime::RawStep raw =
            rig->physical->advance_physics(0.0, 0.0, std::nullopt, ctrl);
        if (rig->physical->crash().has_value()) {
            const auto qp_c = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
            const auto qv_c = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
            const sp::Quat qc{qp_c[qa + 3U], qp_c[qa + 4U], qp_c[qa + 5U], qp_c[qa + 6U]};
            const sp::EulerZyx ec = sp::euler_zyx(qc);
            bt::require(false,
                        std::string(spec.label) + ": crash=" + rig->physical->crash()->cause +
                            " step=" + std::to_string(step) +
                            " x=" + std::to_string(qp_c[qa]) +
                            " y=" + std::to_string(qp_c[qa + 1U]) +
                            " vx=" + std::to_string(qv_c[static_cast<std::size_t>(rig->root_free_dof)]) +
                            " roll=" + std::to_string(ec.roll_rad));
        }

        if (!saw_steer_component) {
            for (const auto &[name, vec] : raw.components) {
                if (name == "act_rider_steer" &&
                    vec[static_cast<std::size_t>(rig->steer_dof)] != 0.0) {
                    saw_steer_component = true;
                    break;
                }
            }
        }

        const auto qp = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
        const sp::Quat q_curr{
            qp[qa + 3U], qp[qa + 4U], qp[qa + 5U], qp[qa + 6U]};
        const sp::EulerZyx eul = sp::euler_zyx(q_curr);
        bt::require(eul.valid,
                    std::string(spec.label) + ": valid attitude throughout");
        max_abs_roll = std::max(max_abs_roll, std::abs(eul.roll_rad));

        accounting.push(std::move(raw));
        if (accounting.full()) {
            accounting.close_period();
            const auto &e = accounting.state().energy;
            const double ratio = std::abs(e.residual_j) / e.energy_scale_j;
            max_residual_ratio = std::max(max_residual_ratio, ratio);
        }
    }
    accounting.flush();
    {
        const auto &e = accounting.state().energy;
        const double ratio = std::abs(e.residual_j) / e.energy_scale_j;
        max_residual_ratio = std::max(max_residual_ratio, ratio);
    }

    const auto qp_end = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
    const double y_end = qp_end[qa + 1U];

    bt::require(max_abs_roll < 0.35,
                std::string(spec.label) + ": |roll| < 0.35 rad throughout");
    bt::require(std::abs(y_end) < spec.max_abs_y_end_m,
                std::string(spec.label) + ": |y_end| < " +
                    std::to_string(spec.max_abs_y_end_m) + " m (got " +
                    std::to_string(y_end) + ")");
    bt::require(max_residual_ratio <= 0.01,
                std::string(spec.label) + ": |energy.residual_j| / scale <= 0.01 (got " +
                    std::to_string(max_residual_ratio) + ")");
    if (spec.y0_m != 0.0 || spec.cross_slope_deg != 0.0) {
        bt::require(saw_steer_component,
                    std::string(spec.label) +
                        ": SteerBalance emitted act_rider_steer");
    }
}

NATIVE_DIAG_PUSH
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] void test_planar_shield_and_gating() {
    // 1. Planar build keeps SteerBalance disengaged and planar CrashDetector intact.
    {
        auto req = std::make_unique<init::SetupRequest>(
            init::read_setup_request(bt::fixture("requests/welded_flat.toml")));
        auto owner = init::make_setup(std::move(*req));
        (void)init::settle(*owner);
        auto boot = std::make_unique<runtime::RuntimeBootstrap>(
            owner->seal_bootstrap());
        auto rt = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot));
        const auto res = rt->advance(20, runtime::RideControl{}, 0.0, 0.0,
                                     std::nullopt);
        bt::require(res.step == 20 && !res.outcome.has_value(),
                    "planar NativeRideRuntime advances identically");
    }

    // 2. Spatial model with StepConfig::spatial == nullopt leaves SteerBalance disengaged.
    {
        auto rig = build_ride_rig({.label = "no_spatial_cfg", .track_length_m = 60.0});
        const mjModel *m = rig->stepper->model();
        auto no_spatial_cfg = std::make_unique<runtime::StepConfig>();
        no_spatial_cfg->timestep_s = rig->rt_cfg->timestep_s;
        no_spatial_cfg->control_period_s = rig->rt_cfg->control_period_s;
        no_spatial_cfg->measurement_decimation = 1;
        no_spatial_cfg->crank_length_m = rig->rt_cfg->geometry.crank_length_m;
        no_spatial_cfg->road_lookahead_m = 2.0;
        no_spatial_cfg->monitors = rig->rt_cfg->monitors;
        no_spatial_cfg->intent = rider::decode_intent_config(
            rig->rt_cfg->rider_intent, "config.rider_intent");
        no_spatial_cfg->reaction = rider::decode_reaction_config(
            rig->rt_cfg->rider_reaction, "config.rider_reaction");
        no_spatial_cfg->launch = rider::decode_launch_config(
            rig->rt_cfg->rider_launch, "config.rider_launch");
        no_spatial_cfg->surface_labels = rider::decode_surface_labels(
            rig->rt_cfg->surface_labels, "config.surface_labels");

        const auto &drv = rig->stepper->drive().config();
        const runtime::StaticBrake brake(
            joint_dofadr(*m, "front_wheel_spin"),
            joint_dofadr(*m, "rear_wheel_spin"), drv.brake_ceiling_nm,
            runtime::BrakeHoldSpec{
                .torque_nm = drv.brake_hold_torque_nm,
                .stiffness_nm_rad = drv.brake_hold_stiffness_nm_rad,
                .damping_nms_rad = drv.brake_hold_damping_nms_rad,
                .engage_radps = drv.brake_hold_engage_radps,
                .release_radps = drv.brake_hold_release_radps,
                .unified_ceiling = true});
        std::vector<std::array<double, 2>> verts =
            trn::profile(rig->track, 0.25);
        auto no_sb = std::make_unique<runtime::PhysicalStep>(
            *rig->stepper, *no_spatial_cfg, std::move(verts), brake,
            std::nullopt, std::nullopt);
        bt::require(no_sb->is_spatial() && !no_sb->has_steer_balance(),
                    "spatial PhysicalStep without StepConfig::spatial instantiates zero SteerBalance");
    }
}
NATIVE_DIAG_POP

[[gnu::noinline]] void test_native_ride_runtime_root_free_and_reset() {
    auto rig = build_ride_rig({
        .label = "runtime_root_free",
        .rider_variant = "none",
        .track_length_m = 80.0,
        .start_x_m = 10.0,
        .y0_m = 0.20,
        .v0_mps = 5.0,
        .drive_mode = "coast",
        .steps = 100,
    });
    settle_and_finalize_moving_start(*rig, {
        .label = "runtime_root_free",
        .rider_variant = "none",
        .track_length_m = 80.0,
        .start_x_m = 10.0,
        .y0_m = 0.20,
        .v0_mps = 5.0,
        .drive_mode = "coast",
    });

    // Build a RuntimeBootstrap with native_reference tire so NativeRideRuntime
    // can own the Stepper + PhysicalStep + PeriodAccounting directly and verify
    // ColumnLayout without root_pitch, StepConfig::spatial forwarding,
    // SteerBalance actuation, and deterministic reset().
    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args("none");
    req->include_rider = false;
    req->physics = make_ride_physics(5.0, "coast", 0.0, false);
    req->physics->tires.backend = "native_reference";
    req->physics->tires.surface_mode = "configured";
    cfg::post_init(*req->physics);
    req->spatial = make_spatial_config();
    const auto tire_cfg =
        make_compliant_3d_tire_config(req->spatial->tire_preset);
    req->bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.0;
    req->bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.0;
    const double ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.0;
    const trn::ExtrudedTrack3d ext(rig->track, 1.5, ground_z_m, 0.25);
    const trn::Grid3dSpec gs = trn::make_grid3d_spec(ext.bounds(), 0.25, 0.25);
    const trn::Grid3d grid = trn::sample_to_grid3d(ext, gs);
    const trn::HeightField3dBuild hf = trn::build_heightfield_3d(grid, ground_z_m);
    req->field = hf.spec;
    req->crank_joint = true;
    auto compiled = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    trn::populate_heightfield_3d(*compiled, grid, hf);

    auto rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, rig->track));
    rt_cfg->record_retention = runtime::RetentionMode::full;
    const auto img = init::capture_model_image(*compiled);
    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(rt_cfg->writer_config));
    if (writer_cfg->resistance.has_value()) {
        writer_cfg->resistance->allow_spatial = true;
    }
    writer_cfg->cruise.reset();
    auto stepper = std::make_unique<Stepper>(img.mjb, *writer_cfg);
    const mjModel *m = stepper->model();
    const mjData *d = stepper->data();
    const auto qpos_m = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel_m = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    const auto src_qp = model_access::readonly_buffer(
        rig->stepper->data()->qpos, m->nq, "src_qpos");
    const auto src_qv = model_access::readonly_buffer(
        rig->stepper->data()->qvel, m->nv, "src_qvel");
    std::ranges::copy(src_qp, qpos_m.begin());
    std::ranges::copy(src_qv, qvel_m.begin());
    stepper->mutate([&] {
        stepper->drive().reset();
        stepper->drive().restart_clock();
        (void)stepper->drive().take_model_dirty();
    });
    stepper->note_posvel_stale();
    stepper->forward();

    auto phys = runtime::make_physical_step(*stepper, *rt_cfg);
    bt::require(phys->is_spatial() && phys->has_steer_balance(),
                "NativeRideRuntime make_physical_step enables SteerBalance");
    const auto [e0, s0] = phys->measure_energy_and_scale();
    auto st = std::make_unique<runtime::StepState>(phys->state());
    st->generation = 1;
    st->energy.initial_energy_j = e0;
    st->energy.energy_scale_j = s0;
    st->energy.mechanical_energy_j = e0;
    phys->restore(*st);
    auto boot = std::make_unique<runtime::RuntimeBootstrap>(
        init::capture_initialized(*stepper, *phys, *rt_cfg));

    auto rt = std::make_unique<runtime::NativeRideRuntime>(std::move(*boot));
    const auto probe = rt->probe_step_inputs(runtime::RideControl{}, 0.0, 0.0);
    bt::require(!probe.ctrl.empty(), "NativeRideRuntime probe succeeds on root_free");

    const auto r1 = rt->advance(80, runtime::RideControl{}, 0.0, 0.0, std::nullopt);
    bt::require(r1.step == 80 && !r1.outcome.has_value(),
                "NativeRideRuntime advances 80 steps on root_free without crash");
    rt->flush();
    const auto cols1 = rt->recorded_columns();
    bt::require(cols1.contains("x_m") && cols1.contains("speed_mps") &&
                    cols1.contains("pitch_rad"),
                "NativeRideRuntime ColumnLayout emits x_m/speed_mps/pitch_rad without root_pitch");
    const auto snap1 = rt->snapshot();

    rt->reset();
    const auto r2 = rt->advance(80, runtime::RideControl{}, 0.0, 0.0, std::nullopt);
    bt::require(r2.step == 80 && !r2.outcome.has_value(),
                "NativeRideRuntime replays after reset()");
    const auto snap2 = rt->snapshot();
    bt::require(snap1.integration_state == snap2.integration_state,
                "NativeRideRuntime reset() resets SteerBalance deterministically");
}

[[gnu::noinline]] void test_20s_spatial_rides() {
    // 1. Rider = none: flat ExtrudedTrack3d, v0 = 5.0 m/s, y0 = +0.35 m corridor recovery
    run_20s_ride_scenario({
        .label = "none_flat_y0p35",
        .rider_variant = "none",
        .track_length_m = 160.0,
        .grade = 0.0,
        .cross_slope_deg = 0.0,
        .start_x_m = 10.0,
        .y0_m = 0.35,
        .v0_mps = 5.0,
        .human_torque_nm = 0.8,
        .drive_mode = "crank_effort",
        .motor_clutch = false,
        .steps = 16000,
    });

    // 2. Rider = none: 15% uphill, v0 = 5.0 m/s
    run_20s_ride_scenario({
        .label = "none_uphill15",
        .rider_variant = "none",
        .track_length_m = 160.0,
        .grade = 0.15,
        .cross_slope_deg = 0.0,
        .start_x_m = 10.0,
        .y0_m = 0.0,
        .v0_mps = 5.0,
        .human_torque_nm = 14.0,
        .motor_torque_nm = 10.0,
        .drive_mode = "crank_effort",
        .motor_clutch = true,
        .steps = 16000,
    });

    // 3. Rider = none: 5 deg cross-slope Grid3d, v0 = 5.0 m/s
    run_20s_ride_scenario({
        .label = "none_cross5deg",
        .rider_variant = "none",
        .track_length_m = 160.0,
        .grade = 0.0,
        .cross_slope_deg = 5.0,
        .start_x_m = 10.0,
        .y0_m = 0.0,
        .v0_mps = 5.0,
        .human_torque_nm = 2.5,
        .max_abs_y_end_m = 0.15,
        .drive_mode = "crank_effort",
        .motor_clutch = false,
        .steps = 16000,
    });

    // 4. Rider = lumped: flat ExtrudedTrack3d, v0 = 5.0 m/s, y0 = +0.35 m corridor recovery
    run_20s_ride_scenario({
        .label = "lumped_flat_y0p35",
        .rider_variant = "lumped",
        .track_length_m = 160.0,
        .grade = 0.0,
        .cross_slope_deg = 0.0,
        .start_x_m = 10.0,
        .y0_m = 0.35,
        .v0_mps = 5.0,
        .human_torque_nm = 2.0,
        .drive_mode = "crank_effort",
        .motor_clutch = false,
        .steps = 16000,
    });

    // 5. Rider = lumped: 15% uphill, v0 = 5.0 m/s
    run_20s_ride_scenario({
        .label = "lumped_uphill15",
        .rider_variant = "lumped",
        .track_length_m = 160.0,
        .grade = 0.15,
        .cross_slope_deg = 0.0,
        .start_x_m = 10.0,
        .y0_m = 0.0,
        .v0_mps = 5.0,
        .human_torque_nm = 25.0,
        .drive_mode = "crank_effort",
        .motor_clutch = false,
        .steps = 16000,
    });

    // 6. Rider = lumped: 5 deg cross-slope Grid3d, v0 = 5.0 m/s
    run_20s_ride_scenario({
        .label = "lumped_cross5deg",
        .rider_variant = "lumped",
        .track_length_m = 160.0,
        .grade = 0.0,
        .cross_slope_deg = 5.0,
        .start_x_m = 10.0,
        .y0_m = 0.0,
        .v0_mps = 5.0,
        .human_torque_nm = 3.0,
        .drive_mode = "crank_effort",
        .motor_clutch = false,
        .steps = 16000,
    });
}

} // namespace

// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        test_planar_shield_and_gating();
        test_native_ride_runtime_root_free_and_reset();
        test_20s_spatial_rides();
    });
}
