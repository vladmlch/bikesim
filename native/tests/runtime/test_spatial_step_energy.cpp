// test_spatial_step_energy.cpp — End-to-end S2 spatial 3D Stepper,
// PhysicalStep, CrashDetector, unified StaticBrake, and EnergyAccounting test.
#include "terrain/terrain3d_types.hpp"
#include "terrain/extrusion3d.hpp"
#include "terrain/grid3d.hpp"
#include "terrain/heightfield3d.hpp"
#include "terrain/profile.hpp"
#include "terrain/source3d.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "model/builder.hpp"
#include "model/checks.hpp"
#include "model_access.hpp"
#include "rider/senses_codec.hpp"
#include "rider/state_codec.hpp"
#include "runtime/accounting.hpp"
#include "runtime/control.hpp"
#include "runtime/project_config.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/step.hpp"
#include "spatial/frames.hpp"
#include "spatial/types.hpp"
#include "stepper.hpp"
#include "support/fixture_test.hpp"
#include "writer_config.hpp"
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

struct SpatialThresholds {
    double corridor_half_width_m = 1.5;
    double capsize_roll_rad = 1.0;
    double cross_slope = 0.0;
};

sp::SpatialConfig make_spatial_config(const SpatialThresholds &thresh) {
    sp::SpatialConfig out{};
    out.tire_backend = sp::SpatialTireBackend::compliant_3d;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = sp::SpatialStartMode::moving;
    out.corridor_half_width_m = thresh.corridor_half_width_m;
    out.capsize_roll_rad = thresh.capsize_roll_rad;
    out.steer.range_deg = 45.0;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.0;
    out.steer.max_torque_nm = 35.0;
    return out;
}

int joint_qposadr(const mjModel &model, const char *name) {
    const int jid = model_access::resolve_id(&model, mjOBJ_JOINT, name);
    const auto addrs = model_access::readonly_buffer(
        model.jnt_qposadr, model.njnt, "jnt_qposadr");
    return addrs[static_cast<std::size_t>(jid)];
}

int joint_dofadr(const mjModel &model, const char *name) {
    const int jid = model_access::resolve_id(&model, mjOBJ_JOINT, name);
    const auto addrs = model_access::readonly_buffer(
        model.jnt_dofadr, model.njnt, "jnt_dofadr");
    return addrs[static_cast<std::size_t>(jid)];
}

struct SpatialRig {
    trn::TrackSpec track;
    std::unique_ptr<trn::Terrain3dSource> terrain;
    std::unique_ptr<Stepper> stepper;
    std::unique_ptr<runtime::PhysicalStep> physical;
    runtime::StepConfig step_cfg;
    double brake_ceiling_nm = 200.0;
    int root_free_qpos = 0;
    int root_free_dof = 0;
    int front_wheel_dof = 0;
    int rear_wheel_dof = 0;
    int crank_spin_dof = 0;
};

struct BuiltModelData {
    runtime::ModelImage image;
    std::unique_ptr<trn::Terrain3dSource> terrain;
    std::unique_ptr<runtime::RuntimeConfig> rt_cfg;
    sp::SpatialConfig spatial_cfg;
};

// NOLINTBEGIN(modernize-make-unique) avoid frame bloat from forwarding helpers
std::unique_ptr<BuiltModelData> build_spatial_model_data(
    const trn::TrackSpec &track, const SpatialThresholds &thresh) {
    auto out = std::make_unique<BuiltModelData>();
    out->spatial_cfg = make_spatial_config(thresh);

    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->include_rider = true;
    {
        auto phys = std::make_unique<cfg::SimulationPhysicsConfig>(
            cfg::default_physics());
        phys->physics_mode = "physical";
        phys->drive_mode = "crank_effort";
        phys->pitch_assist = false;
        phys->seated_climb.enabled = false;
        phys->articulated.pedal_attachment = "flat";
        phys->articulated.saddle_attachment = "flat";
        phys->articulated.grip_attachment = "spring";
        phys->drive.transmission_model = "ideal_mid_drive";
        phys->drive.motor_clutch = false;
        phys->tires.backend = "compliant_2d";
        phys->tires.surface_mode = "track";
        req->physics = std::move(*phys);
    }
    req->spatial = out->spatial_cfg;

    const double ground_z_m =
        bike::geometry::compute_ground_z(req->bike) / 1000.0;
    const trn::ExtrudedTrack3d ext(track, 1.5, ground_z_m, 0.25);
    const trn::Grid3dSpec gs =
        trn::make_grid3d_spec(ext.bounds(), 0.25, 0.25);
    const trn::Grid3d base_grid = trn::sample_to_grid3d(ext, gs);
    std::vector<double> elevations(gs.nx * gs.ny);
    std::vector<rider::SurfaceLabel> surfaces(
        (gs.nx - 1U) * (gs.ny - 1U), rider::SurfaceLabel::hardpack);
    for (std::size_t ix = 0; ix < gs.nx; ++ix) {
        for (std::size_t iy = 0; iy < gs.ny; ++iy) {
            const double y = gs.y0_m + static_cast<double>(iy) * gs.dy_m;
            elevations[ix * gs.ny + iy] =
                base_grid.node_z(ix, iy) + thresh.cross_slope * y;
        }
    }
    const trn::Grid3d grid(gs, std::move(elevations), std::move(surfaces));
    const trn::HeightField3dBuild hf =
        trn::build_heightfield_3d(grid, ground_z_m);
    req->field = hf.spec;

    auto compiled = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    {
        const auto field = static_cast<std::size_t>(
            model_access::resolve_id(compiled.get(), mjOBJ_HFIELD, "road"));
        const auto addr = std::views::counted(
            compiled->hfield_adr, compiled->nhfield)[field];
        const auto samples = trn::build_field_data(track, req->field);
        const auto target = std::views::counted(compiled->hfield_data,
                                                compiled->nhfielddata)
                                .subspan(static_cast<std::size_t>(addr),
                                         samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i)
            target[i] = static_cast<float>(samples[i]);
    }
    out->rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, track));

    trn::populate_heightfield_3d(*compiled, grid, hf);
    out->image = init::capture_model_image(*compiled);
    out->terrain = std::make_unique<trn::Terrain3dSource>(grid);
    return out;
}

std::unique_ptr<SpatialRig> build_spatial_rig(
    const SpatialThresholds &thresh = {}) {
    auto rig = std::make_unique<SpatialRig>();
    rig->track = trn::flat(80.0);

    auto built = build_spatial_model_data(rig->track, thresh);
    rig->terrain = std::move(built->terrain);

    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(built->rt_cfg->writer_config));
    writer_cfg->tire.reset();
    writer_cfg->cruise.reset();
    writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
        sp::SpatialTirePreset::mtb_2p8_1p2bar,
        make_terrain3d_callbacks(*rig->terrain));
    if (!writer_cfg->resistance.has_value()) {
        bt::require(false, "spatial rig has resistance config");
        return rig;
    }
    writer_cfg->resistance->allow_spatial = true;
    if (!writer_cfg->drive.has_value()) {
        bt::require(false, "spatial rig has drive config");
        return rig;
    }
    const auto &drv = *writer_cfg->drive;
    rig->brake_ceiling_nm = drv.brake_ceiling_nm;

    rig->stepper = std::make_unique<Stepper>(built->image.mjb, *writer_cfg);
    bt::require(rig->stepper->has_tire() &&
                    rig->stepper->tire_writer().is_compliant_3d(),
                "Stepper engages Compliant3dTireWriter for compliant_3d");

    const mjModel &m = *rig->stepper->model();
    rig->root_free_qpos = joint_qposadr(m, "root_free");
    rig->root_free_dof = joint_dofadr(m, "root_free");
    rig->front_wheel_dof = joint_dofadr(m, "front_wheel_spin");
    rig->rear_wheel_dof = joint_dofadr(m, "rear_wheel_spin");
    rig->crank_spin_dof = joint_dofadr(m, "crank_spin");

    rig->step_cfg.timestep_s = built->rt_cfg->timestep_s;
    rig->step_cfg.control_period_s = built->rt_cfg->control_period_s;
    rig->step_cfg.measurement_decimation = 1;
    rig->step_cfg.strict = false;
    rig->step_cfg.crank_length_m = built->rt_cfg->geometry.crank_length_m;
    rig->step_cfg.road_lookahead_m = 2.0;
    rig->step_cfg.rider_present = false;
    rig->step_cfg.monitors = built->rt_cfg->monitors;
    rig->step_cfg.intent = rider::decode_intent_config(
        built->rt_cfg->rider_intent, "config.rider_intent");
    rig->step_cfg.reaction = rider::decode_reaction_config(
        built->rt_cfg->rider_reaction, "config.rider_reaction");
    rig->step_cfg.launch = rider::decode_launch_config(
        built->rt_cfg->rider_launch, "config.rider_launch");
    rig->step_cfg.surface_labels = rider::decode_surface_labels(
        built->rt_cfg->surface_labels, "config.surface_labels");
    rig->step_cfg.spatial = built->spatial_cfg;

    const runtime::StaticBrake brake(
        rig->front_wheel_dof, rig->rear_wheel_dof, drv.brake_ceiling_nm,
        runtime::BrakeHoldSpec{
            .torque_nm = drv.brake_hold_torque_nm,
            .stiffness_nm_rad = drv.brake_hold_stiffness_nm_rad,
            .damping_nms_rad = drv.brake_hold_damping_nms_rad,
            .engage_radps = drv.brake_hold_engage_radps,
            .release_radps = drv.brake_hold_release_radps,
            .unified_ceiling = true});

    std::vector<std::array<double, 2>> vertices = trn::profile(rig->track, 0.25);
    rig->physical = std::make_unique<runtime::PhysicalStep>(
        *rig->stepper, rig->step_cfg, std::move(vertices), brake,
        std::nullopt, std::nullopt);
    bt::require(rig->physical->is_spatial(),
                "PhysicalStep detects root_free spatial topology");
    bt::require(rig->physical->steer_dofadr() >= 0 &&
                    rig->physical->steer_qposadr() >= 0,
                "PhysicalStep resolves steer_joint addresses");
    return rig;
}
// NOLINTEND(modernize-make-unique)

struct BikePoseSpec {
    double x_m = 0.0;
    double y_m = 0.0;
    double z_offset_m = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double forward_speed_mps = 0.0;
};

void set_spatial_bike_pose(const SpatialRig &rig, const BikePoseSpec &pose) {
    const mjModel *m = rig.stepper->model();
    const mjData *d = rig.stepper->data();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    std::ranges::fill(qpos, 0.0);
    std::ranges::fill(qvel, 0.0);

    const auto qa = static_cast<std::size_t>(rig.root_free_qpos);
    const auto da = static_cast<std::size_t>(rig.root_free_dof);
    const sp::Quat q_att =
        sp::quat_from_euler_zyx(pose.roll_rad, pose.pitch_rad, pose.yaw_rad);
    qpos[qa + 0U] = pose.x_m;
    qpos[qa + 1U] = pose.y_m;
    qpos[qa + 2U] = pose.z_offset_m;
    qpos[qa + 3U] = q_att[0];
    qpos[qa + 4U] = q_att[1];
    qpos[qa + 5U] = q_att[2];
    qpos[qa + 6U] = q_att[3];

    const sp::Vec3 fwd = sp::rotate_body_to_world(q_att, {1.0, 0.0, 0.0});
    qvel[da + 0U] = fwd[0] * pose.forward_speed_mps;
    qvel[da + 1U] = fwd[1] * pose.forward_speed_mps;
    qvel[da + 2U] = fwd[2] * pose.forward_speed_mps;

    const auto radii = rig.stepper->tire_writer().radii();
    qvel[static_cast<std::size_t>(rig.front_wheel_dof)] =
        pose.forward_speed_mps / radii[0];
    qvel[static_cast<std::size_t>(rig.rear_wheel_dof)] =
        pose.forward_speed_mps / radii[1];

    rig.stepper->note_posvel_stale();
    rig.stepper->forward();
    bool drive_dirty = false;
    rig.stepper->mutate([&] {
        rig.stepper->drive().reset();
        rig.stepper->drive().restart_clock();
        drive_dirty = rig.stepper->drive().take_model_dirty();
    });
    if (drive_dirty) {
        rig.stepper->note_posvel_stale();
        rig.stepper->forward();
    }
    rig.stepper->tire_writer().reset();
    rig.stepper->tire_writer().restart_clock();
    rig.stepper->tire_writer().clear_settled_loss();
}

void test_crash_detector_spatial_triggers() {
    auto rig = build_spatial_rig(
        {.corridor_half_width_m = 1.5, .capsize_roll_rad = 0.85});
    const mjModel *m = rig->stepper->model();
    const mjData *d = rig->stepper->data();
    const auto qa = static_cast<std::size_t>(rig->root_free_qpos);
    const runtime::RuntimeContacts clean_contacts{};

    // 1. Capsize on excessive roll (|roll| > capsize_roll_rad).
    {
        runtime::CrashDetector det(m, 60.0, 0.85, 1.5);
        set_spatial_bike_pose(*rig,
                              {.x_m = 5.0,
                               .y_m = 0.2,
                               .roll_rad = 0.90,
                               .yaw_rad = 0.2,
                               .forward_speed_mps = 4.0});
        const auto ev = det.check(d, clean_contacts);
        if (!ev.has_value()) {
            bt::require(false,
                        "CrashDetector fires capsize when |roll| > capsize_roll_rad");
            return;
        }
        bt::require(ev->cause == "capsize",
                    "CrashDetector fires capsize when |roll| > capsize_roll_rad");
        bt::require(std::abs(ev->position_m - 5.0) < 1e-9,
                    "CrashEvent position_m records track x_m");
        // Latched event sticks even after restoring upright pose; reset clears.
        set_spatial_bike_pose(*rig, {.x_m = 6.0, .forward_speed_mps = 4.0});
        const auto latched = det.check(d, clean_contacts);
        bt::require(latched.has_value() && latched->cause == "capsize",
                    "latched CrashEvent sticks");
        det.reset();
        bt::require(!det.check(d, clean_contacts).has_value(),
                    "CrashDetector::reset clears latched crash");
    }

    // 2. Capsize on !att.valid (pitch beyond 80 deg gimbal margin).
    {
        runtime::CrashDetector det(m, 60.0, 0.85, 1.5);
        set_spatial_bike_pose(
            *rig, {.x_m = 4.0, .pitch_rad = 85.0 * (std::numbers::pi / 180.0)});
        const auto ev = det.check(d, clean_contacts);
        bt::require(ev.has_value() && ev->cause == "capsize",
                    "CrashDetector fires capsize when euler_zyx attitude is invalid");
    }

    // 3. Corridor exit on |y| > corridor_half_width_m.
    {
        runtime::CrashDetector det(m, 60.0, 0.85, 1.5);
        set_spatial_bike_pose(*rig,
                              {.x_m = 12.0,
                               .y_m = -1.55,
                               .roll_rad = 0.1,
                               .forward_speed_mps = 3.0});
        const auto ev = det.check(d, clean_contacts);
        if (!ev.has_value()) {
            bt::require(false,
                        "CrashDetector fires corridor_exit when |y| > corridor_half_width_m");
            return;
        }
        bt::require(ev->cause == "corridor_exit",
                    "CrashDetector fires corridor_exit when |y| > corridor_half_width_m");
        bt::require(std::abs(ev->position_m - 12.0) < 1e-9,
                    "corridor_exit records track x_m");
    }

    // 4. Handlebar contact fallback when upright inside corridor.
    {
        runtime::CrashDetector det(m, 60.0, 0.85, 1.5);
        set_spatial_bike_pose(*rig,
                              {.x_m = 7.0,
                               .y_m = 0.3,
                               .roll_rad = 0.1,
                               .forward_speed_mps = 3.0});
        runtime::RuntimeContacts bar_hit{};
        bar_hit.handlebar_load_n = 25.0;
        const auto ev = det.check(d, bar_hit);
        bt::require(ev.has_value() && ev->cause == "handlebar_contact",
                    "CrashDetector fires handlebar_contact when upright and inside corridor");
    }

    // Also verify invalid NaN/nonpositive threshold rejections.
    bt::invalid([&] {
        (void)runtime::CrashDetector(m, 60.0, 0.0, 1.5);
    });
    bt::invalid([&] {
        (void)runtime::CrashDetector(
            m, 60.0, 0.85, std::numeric_limits<double>::quiet_NaN());
    });
    (void)qa;
}

void test_physical_step_crash_triggers() {
    // 1. Capsize inside PhysicalStep::advance_physics.
    {
        auto rig = build_spatial_rig(
            {.corridor_half_width_m = 1.5, .capsize_roll_rad = 0.50});
        set_spatial_bike_pose(
            *rig, {.x_m = 8.0, .roll_rad = 0.60, .forward_speed_mps = 5.0});
        const runtime::RideControl ctrl{};
        (void)rig->physical->advance_physics(0.0, 0.0, std::nullopt, ctrl);
        bt::require(rig->physical->crash().has_value() &&
                        rig->physical->crash()->cause == "capsize",
                    "PhysicalStep triggers capsize on excessive roll");
    }

    // 2. Corridor exit inside PhysicalStep::advance_physics.
    {
        auto rig = build_spatial_rig(
            {.corridor_half_width_m = 1.2, .capsize_roll_rad = 1.0});
        set_spatial_bike_pose(
            *rig, {.x_m = 8.0, .y_m = 1.25, .forward_speed_mps = 5.0});
        const runtime::RideControl ctrl{};
        (void)rig->physical->advance_physics(0.0, 0.0, std::nullopt, ctrl);
        bt::require(rig->physical->crash().has_value() &&
                        rig->physical->crash()->cause == "corridor_exit",
                    "PhysicalStep triggers corridor_exit on |y| > corridor_half_width_m");
    }
}

void test_unified_static_brake_and_energy_closure() {
    // Use cross_slope = 0.04 rad/m (~2.3 deg bank) to exercise true 3D
    // non-planar Grid3d normals, oblique bounce, steer perturbation, and braking!
    auto rig = build_spatial_rig({.corridor_half_width_m = 1.5,
                                  .capsize_roll_rad = 1.0,
                                  .cross_slope = 0.04});
    const mjModel *m = rig->stepper->model();

    // Start at x = 8.0 m, y = 0.0 m, +0.012 m bounce drop, 2 deg lean, 3 deg yaw,
    // v = 6.5 m/s so bicycle self-steers cleanly through 1.6 s.
    set_spatial_bike_pose(*rig,
                          {.x_m = 8.0,
                           .z_offset_m = 0.012,
                           .roll_rad = 2.0 * (std::numbers::pi / 180.0),
                           .yaw_rad = 3.0 * (std::numbers::pi / 180.0),
                           .forward_speed_mps = 6.5});

    const auto [init_energy_j, scale_j] =
        rig->physical->measure_energy_and_scale();

    runtime::AccountingConfig acc_cfg{};
    acc_cfg.timestep_s = rig->step_cfg.timestep_s;
    acc_cfg.period_steps = 4;
    acc_cfg.measurement_decimation = 1;
    acc_cfg.retention = runtime::RetentionMode::full;
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

    constexpr int kSteps = 1280; // 1.6 s at dt = 0.00125 s
    double max_residual_ratio = 0.0;
    bool saw_steer_component = false;
    bool saw_brake_hold_active = false;
    double max_brake_ceiling_violation = 0.0;

    for (int step = 0; step < kSteps; ++step) {
        const double t = static_cast<double>(step) * rig->step_cfg.timestep_s;
        runtime::RideControl ctrl{};
        double front_brake = 0.0;
        double rear_brake = 0.0;
        double steer_torque_nm = 0.0;

        if (step >= 320 && step < 720) {
            // Steer perturbation + light pedaling torque in phase 2.
            const double phase = (t - 0.4) / 0.5;
            steer_torque_nm = 1.2 * std::sin(2.0 * std::numbers::pi * phase);
            ctrl.human_torque_nm = 12.0;
        } else if (step >= 720 && step < 1000) {
            front_brake = 0.25;
            rear_brake = 0.35;
        } else if (step >= 1000) {
            front_brake = 0.85;
            rear_brake = 0.85;
        }

        rig->physical->set_direct_steer_torque_nm(steer_torque_nm);
        runtime::RawStep raw =
            rig->physical->advance_physics(front_brake, rear_brake, std::nullopt, ctrl);

        bt::require(!rig->physical->crash().has_value(),
                    "bike remains upright and inside corridor across 1.6 s simulation");

        if (steer_torque_nm != 0.0) {
            for (const auto &[name, vec] : raw.components) {
                if (name == "act_rider_steer") {
                    saw_steer_component = true;
                    bt::require(
                        std::abs(vec[static_cast<std::size_t>(
                                     rig->physical->steer_dofadr())] -
                                 steer_torque_nm) <= 1e-12,
                        "act_rider_steer applies exact direct_steer_torque_nm");
                }
            }
        }

        // Verify unified static brake ceiling: |tau_hold| + dof_frictionloss <= ceiling * u.
        const auto frictionloss = model_access::readonly_buffer(
            m->dof_frictionloss, m->nv, "dof_frictionloss");
        double hold_f = 0.0;
        double hold_r = 0.0;
        for (const auto &[name, vec] : raw.components) {
            if (name == "brake_hold") {
                hold_f = vec[static_cast<std::size_t>(rig->front_wheel_dof)];
                hold_r = vec[static_cast<std::size_t>(rig->rear_wheel_dof)];
                if (std::abs(hold_f) > 0.0 || std::abs(hold_r) > 0.0) {
                    saw_brake_hold_active = true;
                }
            }
        }
        const double fl_f =
            frictionloss[static_cast<std::size_t>(rig->front_wheel_dof)];
        const double fl_r =
            frictionloss[static_cast<std::size_t>(rig->rear_wheel_dof)];
        const double cap_f = rig->brake_ceiling_nm * std::clamp(front_brake, 0.0, 1.0);
        const double cap_r = rig->brake_ceiling_nm * std::clamp(rear_brake, 0.0, 1.0);
        max_brake_ceiling_violation =
            std::max({max_brake_ceiling_violation,
                      (std::abs(hold_f) + fl_f) - cap_f,
                      (std::abs(hold_r) + fl_r) - cap_r});
        bt::require(std::abs(hold_f) + fl_f <= cap_f + 1e-12,
                    "front unified static brake |tau_hold| + frictionloss <= ceiling * u");
        bt::require(std::abs(hold_r) + fl_r <= cap_r + 1e-12,
                    "rear unified static brake |tau_hold| + frictionloss <= ceiling * u");

        accounting.push(std::move(raw));
        if (accounting.full()) {
            accounting.close_period();
            const auto &e = accounting.state().energy;
            const double ratio = std::abs(e.residual_j) / e.energy_scale_j;
            max_residual_ratio = std::max(max_residual_ratio, ratio);
            bt::require(ratio <= 0.01,
                        "EnergyAccounting residual <= 1% across every period");
        }
    }
    accounting.flush();

    const auto pending = accounting.prepare_samples();
    bt::require(pending.samples.size() == static_cast<std::size_t>(kSteps),
                "PeriodAccounting retains all 1280 steps");
    for (const auto &sample : pending.samples) {
        const auto *energy_wire =
            runtime::sample_wire::find(sample->channels, "energy");
        bt::require(energy_wire != nullptr, "sample carries energy channel");
        const auto &eobj = runtime::sample_wire::object(*energy_wire);
        const double res = runtime::sample_wire::number(
            runtime::sample_wire::required(eobj, "residual_j"));
        const double scl = runtime::sample_wire::number(
            runtime::sample_wire::required(eobj, "energy_scale_j"));
        const double ratio = std::abs(res) / scl;
        max_residual_ratio = std::max(max_residual_ratio, ratio);
        bt::require(ratio <= 0.01,
                    "per-step energy |residual_j| / scale <= 1%");
    }

    const auto &final_energy = accounting.state().energy;
    bt::require(saw_steer_component, "steer perturbation component emitted");
    bt::require(final_energy.active_work_j != 0.0,
                "active_work_j tracks direct steer & crank work");
    bt::require(final_energy.loss_j > 0.0,
                "loss_j accumulates 3D tire and brake dissipation");
    bt::require(max_brake_ceiling_violation <= 1e-12,
                "unified static brake ceiling held on all 1280 steps");
    bt::require(max_residual_ratio <= 0.01,
                "max |residual_j| / energy_scale_j <= 0.01");

    // Also explicitly verify unified StaticBrake latch engages at low speed inside
    // PhysicalStep::advance_physics and stays under ceiling * u.
    if (!saw_brake_hold_active) {
        set_spatial_bike_pose(*rig, {.x_m = 25.0, .forward_speed_mps = 0.05});
        const runtime::RideControl idle{};
        const runtime::RawStep held =
            rig->physical->advance_physics(1.0, 1.0, std::nullopt, idle);
        double hold_front = 0.0;
        for (const auto &[name, vec] : held.components) {
            if (name == "brake_hold") {
                hold_front = vec[static_cast<std::size_t>(rig->front_wheel_dof)];
            }
        }
        const auto fl = model_access::readonly_buffer(
            m->dof_frictionloss, m->nv, "dof_frictionloss");
        bt::require(
            std::abs(hold_front) +
                    fl[static_cast<std::size_t>(rig->front_wheel_dof)] <=
                rig->brake_ceiling_nm + 1e-12,
            "standstill unified static brake latch respects ceiling");
    }
}

} // namespace

// cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
int main() {
    return bt::run([] {
        test_crash_detector_spatial_triggers();
        test_physical_step_crash_triggers();
        test_unified_static_brake_and_energy_closure();
    });
}
