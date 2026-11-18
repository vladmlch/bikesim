// test_spatial_research_imu.cpp — S5a verification:
// 1. Synchronized post-mj_forward 6-axis IMU (stationary +9.81 m/s^2,
//    ZYX mounting * misalignment rotation, lever-arm Euler + centripetal
//    specific force matched against physical MuJoCo <accelerometer>/<gyro>).
// 2. 0-bit planar RNG & wire shield (seed.spawn(3) children[0..1] identical to
//    spawn(2), children[2] 3D gyro noise, 10-key planar wire vs 11-key spatial,
//    latency/maximum_age_s ring buffer, imu_enabled=false zeroing).
// 3. BikeObservationSpatialV1 C ABI layout & sensor_schema_version == 2U
//    validation + BikeControlV1 motor_limit_nm handling.
// 4. Singularity-free 3D WheelieTruth on 15% climb + 12 deg bank (arbitrary
//    yaw/roll, |relative_pitch_rad| <= 1e-12 grounded, +60 mm lift > 2 deg,
//    loop-out nonsingularity, WheelieTracker transitions).
// 5. Closed-loop BikeControlV1 motor_limit_nm anti-wheelie control in 3D and
//    NativeResearchRuntime 3D end-to-end execution.
#include "support/fixture_test.hpp"
#include "configuration/load.hpp"
#include "diag.hpp"
#include "geometry/specs.hpp"
#include "initialization/capture.hpp"
#include "initialization/context.hpp"
#include "initialization/finalize.hpp"
#include "initialization/research_startup.hpp"
#include "initialization/setup_owner.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "research/policy.hpp"
#include "rider/senses_codec.hpp"
#include "rider/state_codec.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/config.hpp"
#include "runtime/control.hpp"
#include "runtime/project_config.hpp"
#include "runtime/research.hpp"
#include "runtime/research_metrics.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/sensors.hpp"
#include "runtime/static_brake.hpp"
#include "runtime/step.hpp"
#include "spatial/frames.hpp"
#include "spatial/types.hpp"
#include "stepper.hpp"
#include "terrain/extrusion3d.hpp"
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
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <sstream>
#include <iomanip>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
namespace cfg = bike::configuration;
namespace init = bike::initialization;
namespace mdl = bike::model;
namespace res = bike::research;
namespace sp = bike::spatial;
namespace trn = bike::terrain;

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRad = kPi / 180.0;

template <class Source>
trn::TerrainSample3d test_sample_cb(
    const void *c, double x, double y) noexcept { // NOLINT(bugprone-easily-swappable-parameters)
    return static_cast<const Source *>(c)->sample(x, y);
}

template <class Source>
trn::TorusClosestPoint3d test_closest_cb(
    const void *c, const trn::Vec3d &center, double search_radius_m) noexcept {
    return static_cast<const Source *>(c)->closest_point(center, search_radius_m);
}

template <class Source>
Compliant3dTerrainCallbacks make_test_callbacks(const Source &src) noexcept {
    return Compliant3dTerrainCallbacks{
        .ctx = &src,
        .sample_fn = &test_sample_cb<Source>,
        .closest_point_fn = &test_closest_cb<Source>,
    };
}

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
    phys->drive.human_torque_nm = 0.0;
    phys->drive.shifting.enabled = false;
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

int sensor_adr(const mjModel &model, const char *name) {
    const int sid = model_access::resolve_id(&model, mjOBJ_SENSOR, name);
    return model_access::readonly_buffer(model.sensor_adr, model.nsensor,
                                         "sensor_adr")
        [static_cast<std::size_t>(sid)];
}

struct ImuTestRig {
    trn::TrackSpec track{};
    std::unique_ptr<trn::Terrain3dSource> terrain{};
    std::unique_ptr<runtime::RuntimeConfig> rt_cfg{};
    std::unique_ptr<Stepper> stepper{};
    std::unique_ptr<runtime::PhysicalStep> physical{};
    std::unique_ptr<cfg::SimulationPhysicsConfig> physics{};
    double ground_z_m = 0.0;
    int root_free_qpos = 0;
    int root_free_dof = 0;
    std::string mjcf_xml{};
};

struct ImuRigSpec {
    double v0_mps = 0.0;
    std::string_view drive_mode = "coast";
    bool motor_clutch = false;
    double grade = 0.0;
    std::string_view rider_variant = "none";
    int rear_teeth = 24;
    int front_teeth = 34;
};

std::unique_ptr<ImuTestRig> build_imu_rig(const sp::SpatialConfig &spatial_cfg,
                                          const ImuRigSpec &spec) {
    auto rig = std::make_unique<ImuTestRig>();
    rig->track = trn::flat(80.0);
    if (spec.grade != 0.0) {
        rig->track.grade_profile =
            trn::GradeProfile{{{0.0, spec.grade}, {80.0, spec.grade}}};
    }
    rig->physics = std::make_unique<cfg::SimulationPhysicsConfig>(
        make_ride_physics(spec.v0_mps, spec.drive_mode, spec.motor_clutch));
    rig->physics->drive.gearing.front_teeth = spec.front_teeth;
    rig->physics->drive.gearing.rear_teeth = spec.rear_teeth;
    rig->physics->drive.assist.mode = "turbo";
    cfg::post_init(*rig->physics);

    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args(spec.rider_variant);
    req->include_rider = (spec.rider_variant != "none");
    req->physics = *rig->physics;
    req->spatial = spatial_cfg;

    const auto tire_cfg =
        make_compliant_3d_tire_config(req->spatial->tire_preset);
    req->bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.0;
    req->bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.0;
    rig->ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.0;

    const trn::ExtrudedTrack3d ext(
        rig->track, req->spatial->corridor_half_width_m, rig->ground_z_m, 0.25);
    const trn::Grid3dSpec gs =
        trn::make_grid3d_spec(ext.bounds(), 0.25, 0.25);
    const auto hfield_grid =
        std::make_unique<trn::Grid3d>(trn::sample_to_grid3d(ext, gs));
    const trn::HeightField3dBuild hf =
        trn::build_heightfield_3d(*hfield_grid, rig->ground_z_m);
    rig->terrain = std::make_unique<trn::Terrain3dSource>(ext);

    req->field = hf.spec;
    req->crank_joint = true;
    rig->mjcf_xml = mdl::generate_mjcf(*req);

    auto compiled = mdl::compile_mjcf(rig->mjcf_xml);
    trn::populate_heightfield_3d(*compiled, *hfield_grid, hf);
    rig->rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, rig->track));
    const auto image = init::capture_model_image(*compiled);

    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(rig->rt_cfg->writer_config));
    writer_cfg->tire.reset();
    writer_cfg->cruise.reset();
    writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
        req->spatial->tire_preset, make_test_callbacks(*rig->terrain));
    if (writer_cfg->resistance.has_value()) {
        writer_cfg->resistance->allow_spatial = true;
    }

    rig->stepper = std::make_unique<Stepper>(image.mjb, *writer_cfg);
    const mjModel &m = *rig->stepper->model();
    rig->root_free_qpos = joint_qposadr(m, "root_free");
    rig->root_free_dof = joint_dofadr(m, "root_free");

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

    std::vector<std::array<double, 2>> verts = trn::profile(rig->track, 0.25);
    rig->physical = std::make_unique<runtime::PhysicalStep>(
        *rig->stepper, std::move(step_cfg), std::move(verts), brake,
        std::nullopt, std::nullopt);
    return rig;
}

struct SettleSpec {
    double start_x_m = 10.0;
    double v0_mps = 0.0;
};

void settle_rig(ImuTestRig &rig, SettleSpec spec) {
    const mjModel *m = rig.stepper->model();
    const mjData *d = rig.stepper->data();
    const auto qa = static_cast<std::size_t>(rig.root_free_qpos);
    const auto da = static_cast<std::size_t>(rig.root_free_dof);
    const int steer_qpos = joint_qposadr(*m, "steer_joint");
    const int steer_dof = joint_dofadr(*m, "steer_joint");

    {
        const auto qp = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qv = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        std::ranges::fill(qp, 0.0);
        std::ranges::fill(qv, 0.0);
        const auto surf = rig.terrain->sample(spec.start_x_m, 0.0);
        bt::require(surf.status == trn::QueryStatus3d::ok,
                    "settle_rig terrain sample ok");
        const double slope_x = -surf.normal_world[0] / surf.normal_world[2];
        const double pitch0 = -std::atan(slope_x);
        const sp::Quat q_att0 = sp::quat_from_euler_zyx(0.0, pitch0, 0.0);
        qp[qa] = spec.start_x_m;
        qp[qa + 1U] = 0.0;
        qp[qa + 2U] = surf.height_m - rig.ground_z_m + 0.005;
        qp[qa + 3U] = q_att0[0];
        qp[qa + 4U] = q_att0[1];
        qp[qa + 5U] = q_att0[2];
        qp[qa + 6U] = q_att0[3];
        rig.stepper->note_posvel_stale();
        rig.stepper->forward();

        const int fg =
            model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact");
        const int rg =
            model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact");
        const auto xpos =
            model_access::readonly_buffer(d->geom_xpos, 3 * m->ngeom, "xpos");
        const auto radii = rig.stepper->tire_writer().radii();
        const std::array<int, 2> geoms{fg, rg};
        double max_shift_z = -1e9;
        for (std::size_t s = 0; s < 2; ++s) {
            const auto g = static_cast<std::size_t>(geoms[s]);
            const trn::Vec3d center{xpos[3 * g], xpos[3 * g + 1], xpos[3 * g + 2]};
            const auto q = rig.terrain->closest_point(center, 1.0);
            bt::require(q.status == trn::QueryStatus3d::ok,
                        "settle_rig closest_point ok");
            const double cur_delta = radii[s] - q.signed_distance_m;
            const double dz =
                (cur_delta - 0.002) / std::max(0.2, q.normal_world[2]);
            max_shift_z = std::max(max_shift_z, dz);
        }
        qp[qa + 2U] += max_shift_z;
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

    auto init_state =
        std::make_unique<runtime::StepState>(rig.physical->state());
    init_state->initializing = true;
    rig.physical->restore(*init_state);

    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::vector<double> hold(nv, 0.0);
    const double dt = rig.rt_cfg->timestep_s;
    const double max_d_roll = 0.4 * 2.0 / dt;
    const double max_d_y = 0.4 * 20.0 / dt;
    const double max_d_steer = 0.4 * 0.14 / dt;
    for (int k = 0; k < 300; ++k) {
        const auto qp = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
        const auto qv = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
        const sp::Quat q_curr{
            qp[qa + 3U], qp[qa + 4U], qp[qa + 5U], qp[qa + 6U]};
        const sp::EulerZyx eul = sp::euler_zyx(q_curr);
        const sp::Vec3 omega_b{qv[da + 3U], qv[da + 4U], qv[da + 5U]};
        const sp::EulerRatesZyx rates =
            sp::body_gyro_to_euler_zyx_rates(eul, omega_b);
        std::ranges::fill(hold, 0.0);
        hold[da + 1U] = -15000.0 * qp[qa + 1U] -
                        std::min(1500.0, max_d_y) * qv[da + 1U];
        hold[da + 3U] = -4000.0 * eul.roll_rad -
                        std::min(400.0, max_d_roll) * rates.roll_rate_rad_s;
        const double tau_yaw =
            -1500.0 * eul.yaw_rad - 150.0 * rates.yaw_rate_rad_s;
        const sp::Vec3 tau_yaw_b =
            sp::rotate_world_to_body(q_curr, {0.0, 0.0, tau_yaw});
        hold[da + 3U] += tau_yaw_b[0];
        hold[da + 4U] += tau_yaw_b[1];
        hold[da + 5U] += tau_yaw_b[2];
        hold[static_cast<std::size_t>(steer_dof)] =
            -500.0 * qp[static_cast<std::size_t>(steer_qpos)] -
            std::min(15.0, max_d_steer) *
                qv[static_cast<std::size_t>(steer_dof)];

        (void)rig.physical->apply_forces(
            false, true, 1.0, 1.0, hold, runtime::RideControl{});
        rig.stepper->mutate(
            [&] { engine::step(rig.stepper->model(), rig.stepper->data()); });
        rig.stepper->note_posvel_stale();
        if (k % 40 == 39) {
            const auto qv_mut =
                model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            std::ranges::fill(qv_mut, 0.0);
            rig.stepper->note_posvel_stale();
            rig.stepper->forward();
        }
    }

    rig.physics->initial_speed_mps = spec.v0_mps;
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
    eq.steps = 300;
    eq.residual_qacc = 0.0;
    ctx.equilibrium = eq;
    init::finalize_initial_state(ctx, *rig.rt_cfg);
    bt::require(ctx.completed, "settle_rig finalized");
}

std::string inject_imu_site_and_sensors(std::string xml,
                                        const sp::Vec3 &pos,
                                        const sp::Quat &quat) {
    const std::string site_marker = R"(<site name="site_BB")";
    const auto pos_idx = xml.find(site_marker);
    bt::require(pos_idx != std::string::npos, "MJCF contains site_BB");
    std::ostringstream oss;
    oss << std::setprecision(17)
        << R"(<site name="site_imu" pos=")" << pos[0] << ' ' << pos[1] << ' '
        << pos[2] << R"(" quat=")" << quat[0] << ' ' << quat[1] << ' '
        << quat[2] << ' ' << quat[3] << R"(" size="0.005"/>)";
    const std::string site_tag = oss.str();
    xml.insert(pos_idx, site_tag);

    const std::string sensor_marker = "<sensor>";
    const auto s_idx = xml.find(sensor_marker);
    bt::require(s_idx != std::string::npos, "MJCF contains <sensor>");
    const std::string sensor_tags =
        "<accelerometer name=\"oracle_imu_accel\" site=\"site_imu\"/>"
        "<gyro name=\"oracle_imu_gyro\" site=\"site_imu\"/>";
    xml.insert(s_idx + sensor_marker.size(), sensor_tags);
    return xml;
}

[[gnu::noinline]] void test_stationary_and_physical_oracle_imu() {
    // 1. Stationary upright: (0, 0, +9.81) m/s^2, omega = (0, 0, 0) rad/s.
    {
        sp::SpatialConfig sc = make_spatial_config();
        sc.start_mode = sp::SpatialStartMode::standstill;
        sc.start_support = sp::SpatialStartSupportSpec{
            .release_speed_mps = 1.6,
            .lateral_stiffness_n_per_m = 4000.0,
            .lateral_damping_ns_per_m = 400.0,
            .roll_stiffness_nm_per_rad = 2500.0,
            .roll_damping_nms_per_rad = 250.0,
        };
        auto rig = build_imu_rig(
            sc, {.v0_mps = 0.0, .drive_mode = "coast", .motor_clutch = false});
        settle_rig(*rig, {.start_x_m = 10.0, .v0_mps = 0.0});
        const mjModel *m = rig->stepper->model();
        const mjData *d = rig->stepper->data();
        const auto qa = static_cast<std::size_t>(rig->root_free_qpos);
        {
            const auto qv = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            std::ranges::fill(qv, 0.0);
        }
        rig->stepper->note_posvel_stale();
        rig->stepper->forward();
        const auto probe = rig->physical->probe_step_inputs(
            runtime::RideControl{}, 1.0, 1.0);
        const auto &w = runtime::sample_wire::array(
            runtime::sample_wire::required(probe.sensors,
                                           "frame_gyro_body_rad_s"));
        bt::require(std::abs(runtime::sample_wire::number(w[0])) < 1e-9 &&
                        std::abs(runtime::sample_wire::number(w[1])) < 1e-9 &&
                        std::abs(runtime::sample_wire::number(w[2])) < 1e-9,
                    "stationary upright IMU gyro == 0");

        {
            const auto qp = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
            const auto qv = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            qp[qa + 3U] = 1.0;
            qp[qa + 4U] = 0.0;
            qp[qa + 5U] = 0.0;
            qp[qa + 6U] = 0.0;
            std::ranges::fill(qv, 0.0);
        }
        rig->stepper->note_posvel_stale();
        rig->stepper->forward();
        // Hold exact static upright equilibrium (qacc == 0) via mj_inverse residual.
        (void)rig->physical->apply_forces(
            true, false, 1.0, 1.0, std::nullopt, runtime::RideControl{});
        std::vector<double> ext_hold(static_cast<std::size_t>(m->nv), 0.0);
        {
            const auto applied_before = model_access::readonly_buffer(
                d->qfrc_applied, m->nv, "qfrc_applied");
            std::vector<double> saved_applied(
                applied_before.begin(), applied_before.end());
            rig->stepper->mutate([&] {
                const auto qa_buf =
                    model_access::mutable_buffer(d->qacc, m->nv, "qacc");
                std::ranges::fill(qa_buf, 0.0);
                mj_inverse(m, rig->stepper->data());
            });
            const auto inv = model_access::readonly_buffer(
                d->qfrc_inverse, m->nv, "qfrc_inverse");
            for (std::size_t i = 0; i < ext_hold.size(); ++i) {
                ext_hold[i] = inv[i] - saved_applied[i];
            }
        }
        // Exact q_att = I, qvel = 0, qacc = 0 check via single advance_physics step
        // holding exact static equilibrium with ext_hold:
        const runtime::RawStep raw0 = rig->physical->advance_physics(
            1.0, 1.0, ext_hold, runtime::RideControl{});
        const auto &sens0 = runtime::sample_wire::object(
            runtime::sample_wire::required(raw0.channels, "sensors"));
        const auto &f0 = runtime::sample_wire::array(
            runtime::sample_wire::required(sens0,
                                           "frame_specific_force_body_mps2"));
        const auto &w0 = runtime::sample_wire::array(
            runtime::sample_wire::required(sens0, "frame_gyro_body_rad_s"));
        const double f0x = runtime::sample_wire::number(f0[0]);
        const double f0y = runtime::sample_wire::number(f0[1]);
        const double f0z = runtime::sample_wire::number(f0[2]);
        bt::require(std::abs(f0x) < 1e-5 && std::abs(f0y) < 1e-5 &&
                        std::abs(f0z - 9.81) < 1e-5,
                    "exact stationary upright IMU == (0, 0, +9.81) m/s^2, got (" +
                        std::to_string(f0x) + ", " + std::to_string(f0y) + ", " +
                        std::to_string(f0z) + ")");
        bt::require(std::abs(runtime::sample_wire::number(w0[0])) < 1e-12 &&
                        std::abs(runtime::sample_wire::number(w0[1])) < 1e-12 &&
                        std::abs(runtime::sample_wire::number(w0[2])) < 1e-12,
                    "exact stationary upright IMU gyro == (0, 0, 0) rad/s");
    }

    // 2. Nonzero mounting offset, mounting RPY, misalignment RPY, attitude,
    //    angular velocity, and angular acceleration matched to < 1e-12 against
    //    a physical MuJoCo <accelerometer> + <gyro> on an oriented site_imu.
    {
        sp::SpatialConfig sc = make_spatial_config();
        sc.imu_mount_pos_m = {0.15, -0.04, 0.28};
        sc.imu_mount_rpy_rad = {0.12, -0.18, 0.35};
        sc.imu_misalignment_rad = {-0.03, 0.04, -0.02};

        auto rig = build_imu_rig(
            sc, {.v0_mps = 5.0, .drive_mode = "crank_effort", .motor_clutch = true});
        settle_rig(*rig, {.start_x_m = 10.0, .v0_mps = 5.0});

        const sp::Quat q_mount = sp::quat_from_euler_zyx(
            sc.imu_mount_rpy_rad[0], sc.imu_mount_rpy_rad[1],
            sc.imu_mount_rpy_rad[2]);
        const sp::Quat q_mis = sp::quat_from_euler_zyx(
            sc.imu_misalignment_rad[0], sc.imu_misalignment_rad[1],
            sc.imu_misalignment_rad[2]);
        const sp::Quat q_frame_from_imu =
            sp::quat_normalized(sp::quat_multiply(q_mount, q_mis));

        const std::string oracle_xml = inject_imu_site_and_sensors(
            rig->mjcf_xml, sc.imu_mount_pos_m, q_frame_from_imu);
        const auto oracle_model = mdl::compile_mjcf(oracle_xml);
        const mjModel *om = oracle_model.get();
        const auto od_ptr = std::unique_ptr<mjData, void (*)(mjData *)>(
            mj_makeData(om), mj_deleteData);
        mjData *od = od_ptr.get();
        const int oracle_acc_adr = sensor_adr(*om, "oracle_imu_accel");
        const int oracle_gyr_adr = sensor_adr(*om, "oracle_imu_gyro");

        // Copy compiled terrain hfield into oracle_model so contacts match.
        {
            const mjModel *rm = rig->stepper->model();
            bt::require(om->nhfielddata == rm->nhfielddata,
                        "oracle hfielddata size matches");
            const auto src_hf = model_access::readonly_buffer(
                rm->hfield_data, rm->nhfielddata, "src_hf");
            const auto dst_hf = model_access::mutable_buffer(
                om->hfield_data, om->nhfielddata, "dst_hf");
            std::ranges::copy(src_hf, dst_hf.begin());
        }

        const mjModel *m = rig->stepper->model();
        const mjData *d = rig->stepper->data();
        const auto qa = static_cast<std::size_t>(rig->root_free_qpos);
        const auto da = static_cast<std::size_t>(rig->root_free_dof);

        // Apply a nonzero 3D attitude (roll, pitch, yaw) and angular rate.
        {
            const auto qp = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
            const auto qv = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
            const sp::Quat q_att = sp::quat_from_euler_zyx(0.14, -0.22, 0.41);
            qp[qa + 3U] = q_att[0];
            qp[qa + 4U] = q_att[1];
            qp[qa + 5U] = q_att[2];
            qp[qa + 6U] = q_att[3];
            qv[da + 0U] = 4.2;
            qv[da + 1U] = -0.7;
            qv[da + 2U] = 0.35;
            qv[da + 3U] = 0.85;
            qv[da + 4U] = -1.15;
            qv[da + 5U] = 0.65;
        }
        rig->stepper->note_posvel_stale();
        rig->stepper->forward();

        runtime::RideControl ctrl{};
        ctrl.motor_torque_nm = 65.0;
        ctrl.motor_limit_nm = 80.0;
        const runtime::RawStep raw =
            rig->physical->advance_physics(0.0, 0.0, std::nullopt, ctrl);

        // Evaluate oracle mj_forward at the exact interval-start (qpos, qvel)
        // with the exact solved applied forces from raw.components!
        {
            const auto o_qp = model_access::mutable_buffer(od->qpos, om->nq, "o_qp");
            const auto o_qv = model_access::mutable_buffer(od->qvel, om->nv, "o_qv");
            const auto o_qf = model_access::mutable_buffer(
                od->qfrc_applied, om->nv, "o_qf");
            const auto o_ct = model_access::mutable_buffer(
                od->ctrl, om->nu, "o_ct");
            const auto src_fl = model_access::readonly_buffer(
                m->dof_frictionloss, m->nv, "src_fl");
            const auto dst_fl = model_access::mutable_buffer(
                om->dof_frictionloss, om->nv, "dst_fl");
            std::ranges::copy(src_fl, dst_fl.begin());
            std::ranges::copy(raw.qpos, o_qp.begin());
            std::ranges::copy(raw.qvel, o_qv.begin());
            std::ranges::fill(o_qf, 0.0);
            std::ranges::fill(o_ct, 0.0);
            for (const auto &[name, vec] : raw.components) {
                if (name == "equality" || name == "joint_limits" ||
                    name == "tendon_limit" || name == "ideal_transmission" ||
                    name == "shock_solver_limit" ||
                    name == "front_static_brake" ||
                    name == "rear_static_brake" || name == "engine_passive" || name == "native_contact" || name == "closure") {
                    continue;
                }
                for (std::size_t i = 0; i < vec.size(); ++i) {
                    o_qf[i] += vec[i];
                }
            }
        }
        mj_forward(om, od);

        const auto &sens = runtime::sample_wire::object(
            runtime::sample_wire::required(raw.channels, "sensors"));
        const auto &f_wire = runtime::sample_wire::array(
            runtime::sample_wire::required(sens, "frame_specific_force_body_mps2"));
        const auto &w_wire = runtime::sample_wire::array(
            runtime::sample_wire::required(sens, "frame_gyro_body_rad_s"));

        const auto o_sens = model_access::readonly_buffer(
            od->sensordata, om->nsensordata, "o_sens");
        double max_f_err = 0.0;
        double max_w_err = 0.0;
        for (std::size_t k = 0; k < 3; ++k) {
            const double f_got = runtime::sample_wire::number(f_wire[k]);
            const double w_got = runtime::sample_wire::number(w_wire[k]);
            const double f_ref =
                o_sens[static_cast<std::size_t>(oracle_acc_adr) + k];
            const double w_ref =
                o_sens[static_cast<std::size_t>(oracle_gyr_adr) + k];
            max_f_err = std::max(max_f_err, std::abs(f_got - f_ref));
            max_w_err = std::max(max_w_err, std::abs(w_got - w_ref));
        }
        bt::require(max_f_err < 1e-8,
                    "6-axis IMU lever-arm specific force matches MuJoCo <accelerometer> (< 1e-8), err=" +
                        std::to_string(max_f_err));
        bt::require(max_w_err < 1e-12,
                    "6-axis IMU angular rate matches MuJoCo <gyro> (< 1e-12), err=" +
                        std::to_string(max_w_err));
    }
}

[[gnu::noinline]] void test_rng_shield_and_sensor_ring_buffer() {
    res::SensorConfig sc;
    sc.sample_period_s = 0.005;
    sc.delivery.latency_s = 0.010;
    sc.delivery.maximum_age_s = 0.025;
    sc.delivery.acceleration_bias = {0.05, -0.03, 0.02};
    sc.delivery.gyro_bias = 0.01;
    sc.delivery.gyro_bias_3d_rad_s = {-0.02, -0.01, 0.04};
    sc.delivery.dropout_probability = 0.1;
    sc.delivery.imu_enabled = true;
    sc.noise.acceleration_std_mps2 = 0.15;
    sc.noise.gyro_std_rad_s = 0.08;
    sc.noise.encoder_std_rad_s = 0.02;
    sc.noise.torque_std_nm = 0.5;

    const auto planar = res::make_sensor_tape(sc, "123456789", 24, false);
    const auto spatial = res::make_sensor_tape(sc, "123456789", 24, true);

    bt::require(planar.spatial_gyro_noise.empty(),
                "planar NoiseTape has empty spatial_gyro_noise");
    bt::require(spatial.spatial_gyro_noise.size() == 24U,
                "spatial NoiseTape populates spatial_gyro_noise");
    bt::require(planar.noise.size() == spatial.noise.size() &&
                    planar.dropout_uniform.size() == spatial.dropout_uniform.size(),
                "tape row counts match");
    for (std::size_t i = 0; i < planar.noise.size(); ++i) {
        bt::require(
            std::bit_cast<std::uint64_t>(planar.dropout_uniform[i]) ==
                std::bit_cast<std::uint64_t>(spatial.dropout_uniform[i]),
            "0-bit planar dropout uniform invariance");
        for (std::size_t j = 0; j < 9U; ++j) {
            bt::require(
                std::bit_cast<std::uint64_t>(planar.noise[i][j]) ==
                    std::bit_cast<std::uint64_t>(spatial.noise[i][j]),
                "0-bit planar noise invariance");
        }
    }
    bool saw_nonzero_3d_gyro = false;
    for (const auto &row : spatial.spatial_gyro_noise) {
        if (row[0] != 0.0 || row[2] != 0.0) {
            saw_nonzero_3d_gyro = true;
            break;
        }
    }
    bt::require(saw_nonzero_3d_gyro, "children[2] draws nonzero 3D gyro noise");

    // Wire round-trip: planar 10 keys vs spatial 11 keys.
    runtime::SensorObservation obs_planar;
    obs_planar.values = {0.1, -0.2, 9.81, 0.3, 10.0, 10.2, 4.5, 12.0, 30.0};
    bt::require(obs_planar.as_wire().size() == 10U,
                "planar SensorObservation wire keeps exact 10 keys");
    const auto rt_planar =
        runtime::SensorObservation::from_wire(obs_planar.as_wire());
    bt::require(!rt_planar.angular_rate_body_rad_s.has_value(),
                "planar round-trip keeps angular_rate_body_rad_s nullopt");

    runtime::SensorObservation obs_spatial = obs_planar;
    obs_spatial.angular_rate_body_rad_s = std::array<double, 3>{0.11, -0.3, 0.22};
    const auto wire_spatial = obs_spatial.as_wire();
    bt::require(wire_spatial.size() == 11U,
                "spatial SensorObservation wire emits 11 keys");
    const auto rt_spatial = runtime::SensorObservation::from_wire(wire_spatial);
    bt::require(rt_spatial.angular_rate_body_rad_s.has_value() &&
                    (*rt_spatial.angular_rate_body_rad_s)[0] == 0.11 &&
                    (*rt_spatial.angular_rate_body_rad_s)[1] == -0.3 &&
                    (*rt_spatial.angular_rate_body_rad_s)[2] == 0.22,
                "spatial SensorObservation round-trips angular_rate_body_rad_s");

    // Ring buffer: latency_s=0.010, maximum_age_s=0.025, zero dropout for exact timing check.
    sc.delivery.dropout_probability = 0.0;
    runtime::SensorPipeline pipe(sc.delivery, spatial);
    runtime::SensorObservation raw0 = obs_spatial;
    raw0.time_s = raw0.source_time_s = 0.0;
    pipe.reset(raw0);

    // At t=0.0, cutoff = -0.010, so startup invalid sample is returned.
    const auto r0 = pipe.read(0.0);
    bt::require(!r0.valid, "t=0 with positive latency returns invalid startup sample");
    bt::require(r0.angular_rate_body_rad_s.has_value() &&
                    (*r0.angular_rate_body_rad_s)[0] == 0.0 &&
                    (*r0.angular_rate_body_rad_s)[1] == 0.0 &&
                    (*r0.angular_rate_body_rad_s)[2] == 0.0,
                "invalid startup sample zeroes 3D gyro");

    runtime::SensorObservation raw1 = obs_spatial;
    raw1.time_s = raw1.source_time_s = 0.005;
    pipe.push(raw1);

    // At t=0.015, cutoff = 0.005 -> raw1 (source_time=0.005) is delivered!
    const auto r15 = pipe.read(0.015);
    bt::require(r15.valid && std::abs(r15.source_time_s - 0.005) < 1e-12,
                "ring buffer delivers sample respecting latency_s");
    bt::require(r15.angular_rate_body_rad_s.has_value() &&
                    std::bit_cast<std::uint64_t>(r15.values[3]) ==
                        std::bit_cast<std::uint64_t>(-(*r15.angular_rate_body_rad_s)[1]),
                "pitch_rate_up == -omega_y to 0 ULP");

    // At t=0.040, age = 0.040 - 0.005 = 0.035 > maximum_age_s (0.025) -> stale!
    const auto r40 = pipe.read(0.040);
    bt::require(!r40.valid, "sample older than maximum_age_s becomes invalid");

    // imu_enabled = false zeroes both values[0..3] and angular_rate_body_rad_s.
    sc.delivery.imu_enabled = false;
    sc.delivery.latency_s = 0.0;
    runtime::SensorPipeline muted_pipe(sc.delivery, spatial);
    muted_pipe.reset(raw0);
    const auto rm = muted_pipe.read(0.0);
    bt::require(rm.valid && rm.values[0] == 0.0 && rm.values[1] == 0.0 &&
                    rm.values[2] == 0.0 && rm.values[3] == 0.0 &&
                    rm.angular_rate_body_rad_s.has_value() &&
                    (*rm.angular_rate_body_rad_s)[0] == 0.0 &&
                    (*rm.angular_rate_body_rad_s)[1] == 0.0 &&
                    (*rm.angular_rate_body_rad_s)[2] == 0.0,
                "imu_enabled=false zeroes all 6 IMU channels");
}

int stub_create(const char *, std::size_t, void **out, BikeErrorV1 *) {
    *out = nullptr;
    return 0;
}
int stub_reset(void *, const char *, std::size_t, BikeErrorV1 *) { return 0; }
int stub_act(void *, const BikeObservationV1 *, const BikeDemandV1 *,
             BikeControlV1 *, BikeErrorV1 *) {
    return 0;
}
void stub_destroy(void *) {}

[[gnu::noinline]] void test_c_abi_spatial_v1() {
    static_assert(std::is_standard_layout_v<BikeObservationSpatialV1>);
    static_assert(offsetof(BikeObservationSpatialV1, v1) == 0);
    static_assert(BIKE_SENSOR_SCHEMA_SPATIAL_6AXIS == 2U);

    const BikePolicyV1 policy_v2{
        .abi_version = BIKE_POLICY_ABI_VERSION,
        .struct_size = sizeof(BikePolicyV1),
        .required_capabilities = 0,
        .create = stub_create,
        .reset = stub_reset,
        .act = stub_act,
        .destroy = stub_destroy,
        .sensor_schema_version = BIKE_SENSOR_SCHEMA_SPATIAL_6AXIS,
    };
    res::validate_policy_abi(&policy_v2);

    BikePolicyV1 policy_bad = policy_v2;
    policy_bad.sensor_schema_version = 99U;
    bool rejected = false;
    try {
        res::validate_policy_abi(&policy_bad);
    } catch (const res::PolicyFailure &) {
        rejected = true;
    }
    bt::require(rejected, "validate_policy_abi rejects unsupported schema 99");

    runtime::SensorObservation obs;
    obs.time_s = 0.25;
    obs.source_time_s = 0.24;
    obs.valid = true;
    obs.values = {0.1, -0.2, 9.81, -0.4, 14.0, 14.2, 7.5, 45.0, 10.0};
    obs.angular_rate_body_rad_s = std::array<double, 3>{0.15, 0.4, -0.25};

    const BikeObservationSpatialV1 abi_s =
        res::policy_observation_to_spatial_abi(obs);
    bt::require(abi_s.v1.struct_size == sizeof(BikeObservationSpatialV1) &&
                    abi_s.v1.sensor_schema_version ==
                        BIKE_SENSOR_SCHEMA_SPATIAL_6AXIS &&
                    abi_s.v1.valid == 1 &&
                    abi_s.v1.valid_mask == BIKE_SENSOR_SPATIAL_ALL_FIELDS,
                "BikeObservationSpatialV1 header matches v2 contract");
    bt::require(abi_s.angular_rate_body_rad_s[0] == 0.15 &&
                    abi_s.angular_rate_body_rad_s[1] == 0.4 &&
                    abi_s.angular_rate_body_rad_s[2] == -0.25,
                "BikeObservationSpatialV1 carries 3D angular rate");

    // Pointer-interconvertibility from BikeObservationV1* -> BikeObservationSpatialV1*.
    const BikeObservationV1 *base_ptr = &abi_s.v1;
    bt::require(base_ptr->sensor_schema_version == BIKE_SENSOR_SCHEMA_SPATIAL_6AXIS,
                "base pointer sees schema version 2");
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) standard-layout prefix extension at offset 0
    const auto *recovered = reinterpret_cast<const BikeObservationSpatialV1 *>(base_ptr);
    bt::require(recovered->angular_rate_body_rad_s[2] == -0.25,
                "prefix pointer recovers BikeObservationSpatialV1");

    BikeControlV1 ctrl{};
    ctrl.struct_size = sizeof(ctrl);
    ctrl.rider_enabled = 1;
    ctrl.use_saddle = 1;
    ctrl.present_mask = BIKE_CONTROL_MOTOR_TORQUE | BIKE_CONTROL_MOTOR_LIMIT;
    ctrl.motor_torque_nm = 100.0;
    ctrl.motor_limit_nm = 32.5;
    const auto ride_ctrl = res::policy_control_from_abi(ctrl);
    bt::require(ride_ctrl.motor_torque_nm == 100.0 &&
                    ride_ctrl.motor_limit_nm == 32.5,
                "BikeControlV1 preserves motor_limit_nm");
}

runtime::PhysicalSampleData make_synthetic_sample(
    double time_s, const sp::Vec3 &f_hub, const sp::Vec3 &r_hub,
    const sp::Quat &q_frame, const sp::Vec3 &v_world, const sp::Vec3 &omega_body,
    double front_load_n, double rear_load_n) {
    runtime::PhysicalSampleData s;
    s.time_s = time_s;
    s.end_time_s = time_s + 0.00125;
    s.qpos = {f_hub[0] - 0.6, 0.5 * (f_hub[1] + r_hub[1]), 0.6,
              q_frame[0], q_frame[1], q_frame[2], q_frame[3]};
    s.qvel = {v_world[0], v_world[1], v_world[2],
              omega_body[0], omega_body[1], omega_body[2]};
    const auto make_wheel = [](const sp::Vec3 &hub, double load_n) {
        runtime::WireArray patches;
        if (load_n > 0.0) {
            patches.emplace_back(runtime::WireObject{
                {"source_geom", runtime::Wire(std::string("terrain"))},
                {"normal_load_n", runtime::Wire(load_n)}});
        }
        return runtime::WireObject{
            {"wheel_axis_m", runtime::Wire(runtime::WireArray{
                                 runtime::Wire(hub[0]), runtime::Wire(hub[1]),
                                 runtime::Wire(hub[2])})},
            {"patches", runtime::Wire(std::move(patches))},
            {"slip_mps", runtime::Wire(0.05)}};
    };
    s.channels.emplace_back(
        "tires", runtime::Wire(runtime::WireObject{
                     {"front", runtime::Wire(make_wheel(f_hub, front_load_n))},
                     {"rear", runtime::Wire(make_wheel(r_hub, rear_load_n))}}));
    s.channels.emplace_back(
        "mass",
        runtime::Wire(runtime::WireObject{
            {"com_m", runtime::Wire(runtime::WireArray{
                          runtime::Wire(0.5 * (f_hub[0] + r_hub[0])),
                          runtime::Wire(0.5 * (f_hub[1] + r_hub[1])),
                          runtime::Wire(0.8)})}}));
    return s;
}

[[gnu::noinline]] void test_3d_wheelie_truth() {
    // Tilted + banked 3D plane: 15% uphill (dz/dx = 0.15), 12 deg bank.
    const double slope_x = 0.15;
    const double slope_y = std::tan(12.0 * kDegToRad);
    const std::size_t nx = 41, ny = 41;
    const trn::Grid3dSpec gspec{
        .nx = nx, .ny = ny, .x0_m = 0.0, .y0_m = -5.0, .dx_m = 0.25, .dy_m = 0.25};
    std::vector<double> elev(nx * ny, 0.0);
    for (std::size_t ix = 0; ix < nx; ++ix) {
        const double x = gspec.x0_m + static_cast<double>(ix) * gspec.dx_m;
        for (std::size_t iy = 0; iy < ny; ++iy) {
            const double y = gspec.y0_m + static_cast<double>(iy) * gspec.dy_m;
            elev[ix * ny + iy] = slope_x * x + slope_y * y;
        }
    }
    std::vector<::rider::SurfaceLabel> surfs(
        (nx - 1U) * (ny - 1U), ::rider::SurfaceLabel::asphalt);
    const trn::Grid3d grid(gspec, std::move(elev), std::move(surfs));

    const double rf = 0.3790, rr = 0.3600, rc = 0.0355;
    runtime::TruthGeometry geom;
    geom.front_radius = rf;
    geom.rear_radius = rr;
    geom.spatial = std::make_shared<const runtime::SpatialTruthGeometry>(
        runtime::SpatialTruthGeometry{
            .root_free_qpos = 0,
            .root_free_dof = 0,
            .front_crown_radius_m = rc,
            .rear_crown_radius_m = rc,
            .terrain = make_test_callbacks(grid),
        });

    // Road unit normal.
    const double n_norm = std::sqrt(1.0 + slope_x * slope_x + slope_y * slope_y);
    const sp::Vec3 n_road{-slope_x / n_norm, -slope_y / n_norm, 1.0 / n_norm};

    // Place grounded mullet bike at yaw=37 deg, roll=18 deg.
    const double yaw = 37.0 * kDegToRad;
    const double roll = 18.0 * kDegToRad;
    const double wheelbase = 1.25;
    const double cr_x = 4.0, cr_y = 0.1;
    const double cf_x = cr_x + wheelbase * std::cos(yaw);
    const double cf_y = cr_y + wheelbase * std::sin(yaw);
    const double cr_z = slope_x * cr_x + slope_y * cr_y;
    const double cf_z = slope_x * cf_x + slope_y * cf_y;

    // Frame quaternion with yaw and roll, aligned along the wheel line.
    const double road_pitch_along =
        std::atan2(cf_z - cr_z, wheelbase);
    const sp::Quat q_frame = sp::quat_from_euler_zyx(roll, -road_pitch_along, yaw);
    const sp::Vec3 y_F = sp::rotate_body_to_world(q_frame, {0.0, 1.0, 0.0});
    const double sin_g = y_F[0] * n_road[0] + y_F[1] * n_road[1] + y_F[2] * n_road[2];
    const double c_g = std::sqrt(std::max(0.0, 1.0 - sin_g * sin_g));
    const double h_perp_f = rc + (rf - rc) * c_g;
    const double h_perp_r = rc + (rr - rc) * c_g;

    const sp::Vec3 r_hub{
        cr_x + h_perp_r * n_road[0],
        cr_y + h_perp_r * n_road[1],
        cr_z + h_perp_r * n_road[2]};
    const sp::Vec3 f_hub{
        cf_x + h_perp_f * n_road[0],
        cf_y + h_perp_f * n_road[1],
        cf_z + h_perp_f * n_road[2]};

    const auto s_grounded = make_synthetic_sample(
        0.0, f_hub, r_hub, q_frame, {5.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        450.0, 500.0);
    const auto t_grounded = runtime::research_truth(s_grounded, geom);
    bt::require(std::abs(t_grounded.relative_pitch_rad) <= 1e-12,
                "grounded 3D mullet on 15% climb + 12 deg bank has |relative_pitch_rad| <= 1e-12, got " +
                    std::to_string(t_grounded.relative_pitch_rad));
    bt::require(t_grounded.front_clearance_m == 0.0 &&
                    t_grounded.rear_clearance_m == 0.0,
                "grounded 3D clearances are zero");

    // Lift front wheel +60 mm along slope normal.
    const sp::Vec3 f_lifted{
        f_hub[0] + 0.060 * n_road[0],
        f_hub[1] + 0.060 * n_road[1],
        f_hub[2] + 0.060 * n_road[2]};
    const auto s_lifted = make_synthetic_sample(
        0.005, f_lifted, r_hub, q_frame, {5.0, 0.0, 0.0}, {0.0, -0.5, 0.0},
        0.0, 950.0);
    const auto t_lifted = runtime::research_truth(s_lifted, geom);
    bt::require(t_lifted.relative_pitch_rad > 2.0 * kDegToRad,
                "+60 mm lift yields relative_pitch_rad > 2 deg, got " +
                    std::to_string(t_lifted.relative_pitch_rad / kDegToRad));
    bt::require(t_lifted.front_clearance_m > 0.01,
                "+60 mm lift yields front_clearance_m > 10 mm");

    // WheelieTracker transitions: two_wheels -> wheelie_candidate -> wheelie.
    runtime::WheelieTracker tracker(0.02, 5.0, 0.01, 0.035);
    tracker.update(t_grounded, 0.01);
    bt::require(tracker.state() == "two_wheels", "tracker starts two_wheels");
    auto t_cand1 = t_lifted;
    t_cand1.time_s = 0.01;
    tracker.update(t_cand1, 0.01);
    bt::require(tracker.state() == "wheelie_candidate",
                "tracker enters wheelie_candidate before persistence");
    auto t_cand2 = t_lifted;
    t_cand2.time_s = 0.02;
    tracker.update(t_cand2, 0.01);
    bt::require(tracker.state() == "wheelie",
                "tracker confirms wheelie after persistence_s");

    // Loop-out nonsingularity at pitch_up = +90 deg and +100 deg.
    for (const double pitch_up : {90.0 * kDegToRad, 100.0 * kDegToRad}) {
        const sp::Quat q_loop = sp::quat_from_euler_zyx(0.05, -pitch_up, yaw);
        const sp::Vec3 x_F = sp::rotate_body_to_world(q_loop, {1.0, 0.0, 0.0});
        const sp::Vec3 f_loop{
            r_hub[0] + wheelbase * x_F[0],
            r_hub[1] + wheelbase * x_F[1],
            r_hub[2] + wheelbase * x_F[2]};
        const auto s_loop = make_synthetic_sample(
            0.1, f_loop, r_hub, q_loop, {3.0, 0.0, 0.0}, {0.0, -2.0, 0.0},
            0.0, 900.0);
        const auto t_loop = runtime::research_truth(s_loop, geom);
        bt::require(std::isfinite(t_loop.relative_pitch_rad) &&
                        t_loop.relative_pitch_rad > 1.0,
                    "loop-out 3D WheelieTruth remains finite and > 1 rad");
    }
}

struct AntiWheelieState {
    double max_pitch_rate_up = 0.0;
    bool limit_triggered = false;
    double last_limit_nm = 200.0;
};

int antiwheelie_create(const char *, std::size_t, void **out, BikeErrorV1 *) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) C ABI host owns opaque handle until destroy()
    *out = new AntiWheelieState{};
    return 0;
}
int antiwheelie_reset(void *state, const char *, std::size_t, BikeErrorV1 *) {
    *static_cast<AntiWheelieState *>(state) = AntiWheelieState{};
    return 0;
}
int antiwheelie_act(void *state, const BikeObservationV1 *obs_v1,
                    const BikeDemandV1 *demand, BikeControlV1 *out,
                    BikeErrorV1 *) {
    auto *st = static_cast<AntiWheelieState *>(state);
    out->struct_size = sizeof(BikeControlV1);
    out->rider_enabled = 1;
    out->use_saddle = 1;
    const double req = (demand != nullptr && demand->present) ? demand->torque_nm : 120.0;
    double limit_nm = 200.0;
    if (obs_v1 != nullptr && obs_v1->valid &&
        obs_v1->sensor_schema_version == BIKE_SENSOR_SCHEMA_SPATIAL_6AXIS &&
        obs_v1->struct_size >= sizeof(BikeObservationSpatialV1)) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) standard-layout prefix extension at offset 0
        const auto *obs = reinterpret_cast<const BikeObservationSpatialV1 *>(obs_v1);
        const double pitch_rate_up = -obs->angular_rate_body_rad_s[1];
        st->max_pitch_rate_up = std::max(st->max_pitch_rate_up, pitch_rate_up);
        if (pitch_rate_up > 0.15) {
            st->limit_triggered = true;
            limit_nm = 2.0;
        }
    }
    st->last_limit_nm = limit_nm;
    out->present_mask = BIKE_CONTROL_MOTOR_TORQUE | BIKE_CONTROL_MOTOR_LIMIT;
    out->motor_torque_nm = req;
    out->motor_limit_nm = limit_nm;
    return 0;
}
void antiwheelie_destroy(void *state) {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) C ABI host destroys opaque handle created in create()
    delete static_cast<AntiWheelieState *>(state);
}

[[gnu::noinline]] double run_3d_wheelie_episode(bool engage_antiwheelie,
                                                bool &out_limit_triggered) {
    auto rig = build_imu_rig(
        make_spatial_config(),
        {.v0_mps = 2.5,
         .drive_mode = "crank_effort",
         .motor_clutch = true,
         .grade = 0.32,
         .rider_variant = "lumped",
         .rear_teeth = 50,
         .front_teeth = 28});
    settle_rig(*rig, {.start_x_m = 10.0, .v0_mps = 2.5});

    const mjModel *m = rig->stepper->model();
    const auto radii = rig->stepper->tire_writer().radii();
    runtime::TruthGeometry truth_geom;
    truth_geom.front_radius = radii[0];
    truth_geom.rear_radius = radii[1];
    truth_geom.spatial = std::make_shared<const runtime::SpatialTruthGeometry>(
        runtime::SpatialTruthGeometry{
            .root_free_qpos = static_cast<std::size_t>(rig->root_free_qpos),
            .root_free_dof = static_cast<std::size_t>(rig->root_free_dof),
            .front_crown_radius_m = 0.0355,
            .rear_crown_radius_m = 0.0355,
            .terrain = make_test_callbacks(*rig->terrain),
        });

    res::SensorConfig sc;
    sc.sample_period_s = 0.005;
    sc.delivery.latency_s = 0.005;
    sc.delivery.maximum_age_s = 0.020;
    sc.delivery.imu_enabled = true;
    auto tape = res::make_sensor_tape(sc, "42", 400, true);
    runtime::SensorPipeline sensors(sc.delivery, std::move(tape));

    const auto probe0 =
        rig->physical->probe_step_inputs(runtime::RideControl{}, 0.0, 0.0);
    runtime::PhysicalSampleData s0;
    s0.time_s = 0.0;
    s0.channels.emplace_back("sensors", runtime::Wire(probe0.sensors));
    sensors.reset(runtime::raw_sensor_observation(s0, true));

    const BikePolicyV1 abi{
        .abi_version = BIKE_POLICY_ABI_VERSION,
        .struct_size = sizeof(BikePolicyV1),
        .required_capabilities = 0,
        .create = antiwheelie_create,
        .reset = antiwheelie_reset,
        .act = antiwheelie_act,
        .destroy = antiwheelie_destroy,
        .sensor_schema_version = BIKE_SENSOR_SCHEMA_SPATIAL_6AXIS,
    };
    res::validate_policy_abi(&abi);
    void *policy_inst = nullptr;
    BikeErrorV1 err{};
    bt::require(abi.create("{}", 2, &policy_inst, &err) == 0, "policy create");
    bt::require(abi.reset(policy_inst, "42", 2, &err) == 0, "policy reset");

    runtime::WheelieTracker tracker(0.02, 5.0, 0.01, 0.035);
    double max_rel_pitch = 0.0;
    runtime::RideControl ctrl{};
    ctrl.human_torque_nm = 120.0;
    ctrl.motor_torque_nm = 140.0;

    // 1.0 s = 800 steps at dt=0.00125 s, control/sensor period = 4 steps (0.005 s).
    for (int step = 0; step < 800; ++step) {
        if (step % 4 == 0) {
            const auto obs =
                sensors.read(static_cast<double>(step) * 0.00125);
            if (engage_antiwheelie) {
                const auto abi_obs =
                    res::policy_observation_to_spatial_abi(obs);
                const BikeDemandV1 demand{.present = 1, .torque_nm = 140.0};
                BikeControlV1 out{};
                out.struct_size = sizeof(out);
                out.rider_enabled = 1;
                out.use_saddle = 1;
                bt::require(abi.act(policy_inst, &abi_obs.v1, &demand, &out,
                                    &err) == 0,
                            "policy act");
                ctrl = res::policy_control_from_abi(out);
                ctrl.human_torque_nm = 120.0;
            } else {
                ctrl.human_torque_nm = 120.0;
                ctrl.motor_torque_nm = 140.0;
                ctrl.motor_limit_nm.reset();
            }
        }
        const auto raw =
            rig->physical->advance_physics(0.0, 0.0, std::nullopt, ctrl);
        runtime::PhysicalSampleData ps;
        ps.time_s = raw.time_s;
        ps.end_time_s = raw.end_time_s;
        ps.qpos = raw.qpos;
        ps.qvel = raw.qvel;
        ps.channels = raw.channels;
        if ((step + 1) % 4 == 0) {
            sensors.push(runtime::raw_sensor_observation(ps, true));
        }
        const auto truth = runtime::research_truth(ps, truth_geom);
        max_rel_pitch = std::max(max_rel_pitch, truth.relative_pitch_rad);
        tracker.update(truth, ps.dt_s());
        if (rig->physical->crash().has_value()) {
            break;
        }
        (void)m;
    }

    out_limit_triggered =
        static_cast<AntiWheelieState *>(policy_inst)->limit_triggered;
    abi.destroy(policy_inst);
    return max_rel_pitch;
}

[[gnu::noinline]] void test_closed_loop_antiwheelie() {
    bool uncapped_trigger = false;
    const double uncapped_pitch =
        run_3d_wheelie_episode(/*engage_antiwheelie=*/false, uncapped_trigger);
    bool capped_trigger = false;
    const double capped_pitch =
        run_3d_wheelie_episode(/*engage_antiwheelie=*/true, capped_trigger);

    bt::require(uncapped_pitch > 0.035,
                "uncapped 3D torque pitches nose up > 0.035 rad, got " +
                    std::to_string(uncapped_pitch));
    bt::require(capped_trigger,
                "BikeObservationSpatialV1 policy triggered motor_limit_nm");
    bt::require(capped_pitch < uncapped_pitch,
                "BikeControlV1 motor_limit_nm reduces 3D wheelie pitch (" +
                    std::to_string(capped_pitch) + " < " +
                    std::to_string(uncapped_pitch) + ")");
}

[[gnu::noinline]] void test_native_research_runtime_3d() {
    // Also exercise NativeResearchRuntime directly in 3D (native_reference
    // RuntimeBootstrap with SpatialTruthGeometry + SensorPipeline).
    const auto sc = make_spatial_config();
    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args("none");
    req->include_rider = false;
    req->physics = make_ride_physics(4.0, "crank_effort", true);
    req->physics->tires.backend = "native_reference";
    req->physics->tires.surface_mode = "configured";
    cfg::post_init(*req->physics);
    req->spatial = sc;
    const auto tire_cfg = make_compliant_3d_tire_config(sc.tire_preset);
    req->bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.0;
    req->bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.0;
    const double ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.0;
    const auto track = trn::flat(60.0);
    auto ext = std::make_unique<trn::ExtrudedTrack3d>(
        track, sc.corridor_half_width_m, ground_z_m, 0.25);
    const trn::Grid3dSpec gs = trn::make_grid3d_spec(ext->bounds(), 0.25, 0.25);
    const auto grid = std::make_unique<trn::Grid3d>(trn::sample_to_grid3d(*ext, gs));
    auto hf = std::make_unique<trn::HeightField3dBuild>(
        trn::build_heightfield_3d(*grid, ground_z_m));
    req->field = hf->spec;
    req->crank_joint = true;
    auto compiled = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    trn::populate_heightfield_3d(*compiled, *grid, *hf);

    auto rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, track));
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
    const int r_qpos = joint_qposadr(*m, "root_free");
    const int r_dof = joint_dofadr(*m, "root_free");
    {
        const auto qp = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qv = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qp[static_cast<std::size_t>(r_qpos)] = 10.0;
        qp[static_cast<std::size_t>(r_qpos) + 2U] = -ground_z_m + 0.002;
        qv[static_cast<std::size_t>(r_dof)] = 4.0;
    }
    stepper->mutate([&] {
        stepper->drive().reset();
        stepper->drive().restart_clock();
        (void)stepper->drive().take_model_dirty();
    });
    stepper->note_posvel_stale();
    stepper->forward();
    auto phys = runtime::make_physical_step(*stepper, *rt_cfg);
    const auto [e0, s0] = phys->measure_energy_and_scale();
    auto st = std::make_unique<runtime::StepState>(phys->state());
    st->generation = 1;
    st->energy.initial_energy_j = e0;
    st->energy.energy_scale_j = s0;
    st->energy.mechanical_energy_j = e0;
    phys->restore(*st);
    auto ride_boot = std::make_unique<runtime::RuntimeBootstrap>(
        init::capture_initialized(*stepper, *phys, *rt_cfg));
    auto ride_rt = std::make_unique<runtime::NativeRideRuntime>(std::move(*ride_boot));

    res::SensorConfig s_cfg;
    s_cfg.sample_period_s = 0.005;
    s_cfg.delivery.latency_s = 0.0;
    s_cfg.delivery.maximum_age_s = 0.02;
    auto tape = res::make_sensor_tape(s_cfg, "7", 1 + 40 / 4, true);

    const auto probe0 = phys->probe_step_inputs(runtime::RideControl{}, 0.0, 0.0);
    runtime::PhysicalSampleData raw_init_data;
    raw_init_data.time_s = 0.0;
    raw_init_data.channels.emplace_back("sensors", runtime::Wire(probe0.sensors));
    const auto raw_init = runtime::raw_sensor_observation(raw_init_data, true);
    auto startup_state = res::startup_sensor_state(s_cfg, tape, raw_init);

    const auto terrain_keep = std::make_shared<trn::Terrain3dSource>(*ext);
    auto rc = std::make_unique<runtime::ResearchConfig>();
    rc->timestep_s = rt_cfg->timestep_s;
    rc->track_length_m = track.length_m;
    rc->start_position_m = 10.0;
    rc->maximum_energy_residual_ratio = 0.05;
    rc->wheelie_persistence_s = 0.02;
    rc->control_steps = 4;
    rc->delay_steps = 0;
    rc->max_steps = 40;
    rc->sensor_steps = 4;
    rc->record_decimation = 1;
    rc->capture_mode = runtime::ResearchCaptureMode::legacy;
    rc->stop_on_model_violation = false;
    rc->sensors = s_cfg.delivery;
    rc->geometry.front_radius = tire_cfg.front.outer_radius_m;
    rc->geometry.rear_radius = tire_cfg.rear.outer_radius_m;
    rc->geometry.root_x_qpos = static_cast<std::size_t>(r_qpos);
    rc->geometry.root_x_dof = static_cast<std::size_t>(r_dof);
    rc->geometry.root_pitch_qpos = static_cast<std::size_t>(r_qpos) + 4U;
    rc->geometry.root_pitch_dof = static_cast<std::size_t>(r_dof) + 4U;
    rc->geometry.spatial = std::make_shared<const runtime::SpatialTruthGeometry>(
        runtime::SpatialTruthGeometry{
            .root_free_qpos = static_cast<std::size_t>(r_qpos),
            .root_free_dof = static_cast<std::size_t>(r_dof),
            .front_crown_radius_m = tire_cfg.front.crown_radius_m,
            .rear_crown_radius_m = tire_cfg.rear.crown_radius_m,
            .terrain = make_test_callbacks(*terrain_keep),
            .terrain_keepalive = terrain_keep,
        });
    rc->seed_decimal = "7";
    rc->raw_initial = raw_init;

    auto research_rt = std::make_unique<runtime::NativeResearchRuntime>(
        std::move(ride_rt), std::move(*rc), std::move(tape), std::move(startup_state));
    runtime::RideControl cmd{};
    cmd.motor_torque_nm = 60.0;
    cmd.motor_limit_nm = 25.0;
    research_rt->begin_control(cmd, 0.0, 0.0);
    const auto tr = research_rt->advance_control(std::nullopt);
    if (!tr.has_value()) {
        bt::require(false, "NativeResearchRuntime 3D control transition returned");
        return;
    }
    bt::require(tr->observation.angular_rate_body_rad_s.has_value(),
                "NativeResearchRuntime 3D observation carries angular_rate_body_rad_s");
    if (!tr->truth.has_value()) {
        bt::require(false, "NativeResearchRuntime 3D transition carries WheelieTruth");
        return;
    }
    bt::require(std::isfinite(tr->truth->relative_pitch_rad),
                "NativeResearchRuntime 3D transition carries finite relative_pitch_rad");
    research_rt->acknowledge_control();
}
} // namespace

int main() {
    return bt::run([] {
        test_stationary_and_physical_oracle_imu();
        test_rng_shield_and_sensor_ring_buffer();
        test_c_abi_spatial_v1();
        test_3d_wheelie_truth();
        test_closed_loop_antiwheelie();
        test_native_research_runtime_3d();
    });
}
