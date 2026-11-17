// test_spatial_finalize.cpp — S3b spatial SetupOwner, TOML [spatial] request
// loading, 3D moving-start finalize, and presentation/FlatColumns tests.
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
#include "runtime/config.hpp"
#include "runtime/presentation.hpp"
#include "runtime/project_config.hpp"
#include "runtime/samples.hpp"
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
#include "terrain/source3d.hpp"
#include "writer_config.hpp"
#include "writers/compliant_3d_tire.hpp"
#include "writers/drivetrain.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
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

class TempDir {
public:
    TempDir()
        : path(std::filesystem::temp_directory_path() /
               ("bike-s3b-" +
                std::to_string(
                    std::chrono::steady_clock::now().time_since_epoch().count()))) {
        if (!std::filesystem::create_directory(path))
            throw std::runtime_error("cannot create temporary test directory");
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
    std::filesystem::path path;
};

void write_text_file(const std::filesystem::path& file, std::string_view text) {
    std::ofstream out(file, std::ios::binary);
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.close();
}

sp::SpatialConfig make_spatial_config(sp::SpatialStartMode mode = sp::SpatialStartMode::moving) {
    sp::SpatialConfig out{};
    out.tire_backend = sp::SpatialTireBackend::compliant_3d;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = mode;
    out.corridor_half_width_m = 1.5;
    out.capsize_roll_rad = 1.0;
    out.steer.range_deg = 45.0;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.010;
    out.steer.max_torque_nm = 35.0;
    if (mode == sp::SpatialStartMode::standstill) {
        out.start_support = sp::SpatialStartSupportSpec{
            .release_speed_mps = 2.0,
            .lateral_stiffness_n_per_m = 2000.0,
            .lateral_damping_ns_per_m = 100.0,
            .roll_stiffness_nm_per_rad = 2000.0,
            .roll_damping_nms_per_rad = 100.0,
        };
    }
    return out;
}

cfg::SimulationPhysicsConfig make_spatial_physics(double speed_mps, bool motor_clutch) {
    auto phys = std::make_unique<cfg::SimulationPhysicsConfig>(cfg::default_physics());
    phys->physics_mode = "physical";
    phys->drive_mode = "crank_effort";
    phys->pitch_assist = false;
    phys->seated_climb.enabled = false;
    phys->articulated.pedal_attachment = "flat";
    phys->articulated.saddle_attachment = "flat";
    phys->articulated.grip_attachment = "spring";
    phys->drive.transmission_model = "ideal_mid_drive";
    phys->drive.motor_clutch = motor_clutch;
    phys->drive.human_torque_nm = 25.0;
    phys->drive.shifting.enabled = true;
    phys->drive.shifting.target_cadence_min_rpm = 75.0;
    phys->drive.shifting.target_cadence_max_rpm = 85.0;
    phys->tires.backend = "compliant_2d";
    phys->tires.surface_mode = "track";
    phys->initial_speed_mps = speed_mps;
    cfg::post_init(*phys);
    return *phys;
}

int joint_qposadr(const mjModel& model, const char* name) {
    const int jid = model_access::resolve_id(&model, mjOBJ_JOINT, name);
    return model_access::readonly_buffer(model.jnt_qposadr, model.njnt, "jnt_qposadr")
        [static_cast<std::size_t>(jid)];
}

int joint_dofadr(const mjModel& model, const char* name) {
    const int jid = model_access::resolve_id(&model, mjOBJ_JOINT, name);
    return model_access::readonly_buffer(model.jnt_dofadr, model.njnt, "jnt_dofadr")
        [static_cast<std::size_t>(jid)];
}

struct Vec3Pair {
    std::array<double, 3> a{};
    std::array<double, 3> b{};
};

double dot3(const Vec3Pair& p) {
    return p.a[0] * p.b[0] + p.a[1] * p.b[1] + p.a[2] * p.b[2];
}

std::array<double, 3> cross3(const Vec3Pair& p) {
    return {p.a[1] * p.b[2] - p.a[2] * p.b[1],
            p.a[2] * p.b[0] - p.a[0] * p.b[2],
            p.a[0] * p.b[1] - p.a[1] * p.b[0]};
}

std::array<double, 3> normalize3(const std::array<double, 3>& v) {
    const double len = std::sqrt(dot3({.a = v, .b = v}));
    bt::require(len > 1e-12, "nonzero 3D vector for normalization");
    return {v[0] / len, v[1] / len, v[2] / len};
}

struct FlatQuery {
    const runtime::FlatColumns* cols = nullptr;
    std::string_view key;
};

double flat_value(FlatQuery query) {
    for (const auto& [name, val] : *query.cols) {
        if (name == query.key) return val;
    }
    bt::require(false, std::string("missing flat column: ") + std::string(query.key));
    return 0.;
}

struct SpatialTestRig {
    trn::TrackSpec track{};
    std::unique_ptr<trn::Terrain3dSource> terrain{};
    std::unique_ptr<runtime::RuntimeConfig> rt_cfg{};
    std::unique_ptr<Stepper> stepper{};
    std::unique_ptr<runtime::PhysicalStep> physical{};
    std::unique_ptr<cfg::SimulationPhysicsConfig> physics{};
    int root_free_qpos = 0;
    int root_free_dof = 0;
};

// NOLINTBEGIN(modernize-make-unique) avoid frame bloat from forwarding helpers
std::unique_ptr<SpatialTestRig> build_finalize_rig(
    const trn::TrackSpec& track,
    const std::optional<trn::Terrain3dOverlaySpec>& overlay,
    double speed_mps,
    bool motor_clutch) {
    auto rig = std::make_unique<SpatialTestRig>();
    rig->track = track;
    rig->physics = std::make_unique<cfg::SimulationPhysicsConfig>(
        make_spatial_physics(speed_mps, motor_clutch));

    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->include_rider = true;
    req->physics = *rig->physics;
    req->spatial = make_spatial_config(sp::SpatialStartMode::moving);

    const auto tire_cfg = make_compliant_3d_tire_config(req->spatial->tire_preset);
    req->bike.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.;
    req->bike.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.;
    const double ground_z_m = bike::geometry::compute_ground_z(req->bike) / 1000.;

    std::unique_ptr<trn::Grid3d> hfield_grid;
    trn::HeightField3dBuild hf{};
    if (overlay.has_value()) {
        trn::Terrain3dOverlaySpec ov = *overlay;
        if (ov.datum_z_offset_m == 0.) ov.datum_z_offset_m = ground_z_m;
        hfield_grid = std::make_unique<trn::Grid3d>(trn::build_overlay_grid3d(track, ov));
        hf = trn::build_heightfield_3d(*hfield_grid, ground_z_m);
        rig->terrain = std::make_unique<trn::Terrain3dSource>(*hfield_grid);
    } else {
        const trn::ExtrudedTrack3d ext(track, req->spatial->corridor_half_width_m, ground_z_m, 0.25);
        const trn::Grid3dSpec gs = trn::make_grid3d_spec(ext.bounds(), 0.25, 0.25);
        hfield_grid = std::make_unique<trn::Grid3d>(trn::sample_to_grid3d(ext, gs));
        hf = trn::build_heightfield_3d(*hfield_grid, ground_z_m);
        rig->terrain = std::make_unique<trn::Terrain3dSource>(ext);
    }
    req->field = hf.spec;
    req->crank_joint = true;

    auto compiled = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    trn::populate_heightfield_3d(*compiled, *hfield_grid, hf);
    rig->rt_cfg = std::make_unique<runtime::RuntimeConfig>(
        runtime::project_runtime(*req, *compiled, track));
    const auto image = init::capture_model_image(*compiled);

    auto writer_cfg = std::make_unique<nativecfg::WriterConfig>(
        nativecfg::decode(rig->rt_cfg->writer_config));
    writer_cfg->tire.reset();
    writer_cfg->cruise.reset();
    writer_cfg->compliant_3d_tire = make_compliant_3d_tire_config(
        req->spatial->tire_preset, make_terrain3d_callbacks(*rig->terrain));
    if (writer_cfg->resistance.has_value())
        writer_cfg->resistance->allow_spatial = true;

    rig->stepper = std::make_unique<Stepper>(image.mjb, *writer_cfg);
    const mjModel& m = *rig->stepper->model();
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
    step_cfg.intent = rider::decode_intent_config(rig->rt_cfg->rider_intent, "config.rider_intent");
    step_cfg.reaction = rider::decode_reaction_config(rig->rt_cfg->rider_reaction, "config.rider_reaction");
    step_cfg.launch = rider::decode_launch_config(rig->rt_cfg->rider_launch, "config.rider_launch");
    step_cfg.surface_labels = rider::decode_surface_labels(rig->rt_cfg->surface_labels, "config.surface_labels");
    step_cfg.spatial = req->spatial;

    const auto& drv = rig->stepper->drive().config();
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

    std::vector<std::array<double, 2>> vertices = trn::profile(track, 0.25);
    for (auto& pt : vertices) pt[1] += ground_z_m;
    rig->physical = std::make_unique<runtime::PhysicalStep>(
        *rig->stepper, std::move(step_cfg), std::move(vertices), brake,
        std::nullopt, std::nullopt);
    return rig;
}
// NOLINTEND(modernize-make-unique)

struct SpatialPoseSeed {
    double x_m = 0.;
    double y_m = 0.;
    double yaw_rad = 0.;
    double pitch_rad = 0.;
};

void seat_spatial_bike_on_terrain(const SpatialTestRig& rig, SpatialPoseSeed seed) {
    const mjModel* m = rig.stepper->model();
    const mjData* d = rig.stepper->data();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    std::ranges::fill(qpos, 0.);
    std::ranges::fill(qvel, 0.);

    const auto qa = static_cast<std::size_t>(rig.root_free_qpos);
    const sp::Quat q_att = sp::quat_from_euler_zyx(0., seed.pitch_rad, seed.yaw_rad);
    const auto surf = rig.terrain->sample(seed.x_m, seed.y_m);
    const double base_z = surf.status == trn::QueryStatus3d::ok ? surf.height_m : 0.;
    qpos[qa + 0U] = seed.x_m;
    qpos[qa + 1U] = seed.y_m;
    qpos[qa + 2U] = base_z;
    qpos[qa + 3U] = q_att[0];
    qpos[qa + 4U] = q_att[1];
    qpos[qa + 5U] = q_att[2];
    qpos[qa + 6U] = q_att[3];
    rig.stepper->note_posvel_stale();
    rig.stepper->forward();

    const int fg = model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact");
    const int rg = model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact");
    const auto xpos = model_access::readonly_buffer(d->geom_xpos, 3 * m->ngeom, "geom_xpos");
    const auto tire_cfg = make_compliant_3d_tire_config(sp::SpatialTirePreset::mtb_2p8_1p2bar);
    const std::array<double, 2> outer_radii{tire_cfg.front.outer_radius_m, tire_cfg.rear.outer_radius_m};
    const std::array<int, 2> geoms{fg, rg};

    // Place both wheels into ~5 mm normal deflection along surface normal.
    double max_shift_z = -1e9;
    for (std::size_t side = 0; side < 2; ++side) {
        const auto g = static_cast<std::size_t>(geoms[side]);
        const trn::Vec3d center{xpos[3 * g], xpos[3 * g + 1], xpos[3 * g + 2]};
        const auto q = rig.terrain->closest_point(center, 1.0);
        bt::require(q.status == trn::QueryStatus3d::ok, "terrain closest_point ok for seeding");
        const double current_delta = outer_radii[side] - q.signed_distance_m;
        const double target_delta = 0.005;
        const double dz = (current_delta - target_delta) / std::max(0.2, q.normal_world[2]);
        max_shift_z = std::max(max_shift_z, dz);
    }
    qpos[qa + 2U] += max_shift_z;
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
}

struct ExpectedFinalizeCase {
    double v0 = 0.;
    double expected_yaw_rad = 0.;
    double expected_pitch_rad = 0.;
    std::string_view label;
};

void verify_finalize_case(const SpatialTestRig& rig, ExpectedFinalizeCase expected) {
    const double v0 = expected.v0;
    const double expected_yaw_rad = expected.expected_yaw_rad;
    const double expected_pitch_rad = expected.expected_pitch_rad;
    const std::string_view label = expected.label;
    const mjModel* m = rig.stepper->model();
    const mjData* d = rig.stepper->data();
    const auto tire_cfg = make_compliant_3d_tire_config(sp::SpatialTirePreset::mtb_2p8_1p2bar);
    mdl::BikeSpecs bike_specs{};
    bike_specs.front_wheel_radius = tire_cfg.front.outer_radius_m * 1000.;
    bike_specs.rear_wheel_radius = tire_cfg.rear.outer_radius_m * 1000.;
    init::SetupContext ctx{
        .stepper = *rig.stepper,
        .physical = *rig.physical,
        .physics = *rig.physics,
        .start_x_m = 10.0,
        .ground_z_m = bike::geometry::compute_ground_z(bike_specs) / 1000.,
        .frame_body_id = model_access::resolve_id(m, mjOBJ_BODY, "frame"),
        .front_geom_id = model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact"),
        .rear_geom_id = model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact"),
    };
    init::EquilibriumResult eq{};
    eq.steps = 1;
    eq.residual_qacc = 0.0;
    ctx.equilibrium = eq;

    init::finalize_initial_state(ctx, *rig.rt_cfg);
    bt::require(ctx.completed, std::string(label) + ": finalize marks context completed");

    const auto qpos = model_access::readonly_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
    const auto qa = static_cast<std::size_t>(rig.root_free_qpos);
    const auto da = static_cast<std::size_t>(rig.root_free_dof);
    bt::require(qvel[da + 3U] == 0. && qvel[da + 4U] == 0. && qvel[da + 5U] == 0.,
                std::string(label) + ": root_free angular velocity zeroed");

    const TireWriterView tire_view = rig.stepper->tire_writer();
    const auto& snaps = tire_view.snapshot_storage();
    const auto xmat = model_access::readonly_buffer(d->xmat, 9 * m->nbody, "xmat");
    const auto bodies = model_access::readonly_buffer(m->geom_bodyid, m->ngeom, "geom_bodyid");
    const std::array<int, 2> geoms{ctx.front_geom_id, ctx.rear_geom_id};
    for (std::size_t side = 0; side < 2; ++side) {
        bt::require(!snaps[side].patches.empty() && snaps[side].patches.front().normal_load_n > 1.0,
                    std::string(label) + ": settled tire has loaded patch");
        const auto& patch = snaps[side].patches.front();
        const int body = bodies[static_cast<std::size_t>(geoms[side])];
        const std::size_t off = static_cast<std::size_t>(body) * 9U;
        const std::array<double, 3> a_w = normalize3({xmat[off + 1U], xmat[off + 4U], xmat[off + 7U]});
        const std::array<double, 3> n = normalize3(patch.normal);
        const std::array<double, 3> t_x = normalize3(cross3({.a = a_w, .b = n}));
        const std::array<double, 3> t_y = normalize3(cross3({.a = n, .b = t_x}));
        const std::array<double, 3> v_c = mdl::point_velocity(*m, *d, body, patch.point_m);
        const auto xpos = model_access::readonly_buffer(d->geom_xpos, 3 * m->ngeom, "geom_xpos");
        const auto gidx = static_cast<std::size_t>(geoms[side]);
        const std::array<double, 3> p_hub{xpos[3 * gidx], xpos[3 * gidx + 1U], xpos[3 * gidx + 2U]};
        const std::array<double, 3> v_hub = mdl::point_velocity(*m, *d, body, p_hub);
        const bool banked = std::abs(dot3({.a = a_w, .b = n})) > 1e-6;
        const double lat_tol = banked ? 5e-9 : 1e-10;
        const double norm_tol = banked ? 5e-8 : 1e-10;

        const double v_c_ty = dot3({.a = v_c, .b = t_y});
        const double v_c_n = dot3({.a = v_c, .b = n});
        const double v_hub_ty = dot3({.a = v_hub, .b = t_y});
        const double v_hub_n = dot3({.a = v_hub, .b = n});
        const std::array<double, 3> r_vec{
            patch.point_m[0] - p_hub[0], patch.point_m[1] - p_hub[1], patch.point_m[2] - p_hub[2]};
        const double r_tx = dot3({.a = r_vec, .b = t_x});
        const auto diag = [&] {
            std::ostringstream msg;
            msg << " (side=" << side
                << " v_c.t_y=" << v_c_ty
                << " v_hub.t_y=" << v_hub_ty
                << " v_c.n=" << v_c_n
                << " v_hub.n=" << v_hub_n
                << " r_vec.t_x=" << r_tx
                << " n=[" << n[0] << "," << n[1] << "," << n[2] << "])";
            return msg.str();
        };

        bt::require(std::abs(dot3({.a = v_c, .b = t_x})) <= 1e-10,
                    std::string(label) + ": zero longitudinal tire slip |v_c . t_x| <= 1e-10");
        bt::require(std::abs(v_hub_ty) <= 1e-10,
                    std::string(label) + ": zero hub lateral velocity |v_hub . t_y| <= 1e-10");
        bt::require(std::abs(v_c_ty) <= lat_tol,
                    std::string(label) + ": zero lateral tire slip |v_c . t_y| within closest-point floor" + diag());
        bt::require(std::abs(v_hub_n) <= 1e-10,
                    std::string(label) + ": zero hub normal velocity |v_hub . n| <= 1e-10");
        bt::require(std::abs(v_c_n) <= norm_tol,
                    std::string(label) + ": zero normal tire velocity |v_c . n| within closest-point floor" + diag());
    }

    // Verify cassette cog selection and crank/shaft/pedal rates.
    const auto drive_state = rig.stepper->drive().state();
    const int rear_teeth = drive_state.shifting.rear_teeth;
    const int front_teeth = rig.physics->drive.gearing.front_teeth;
    const double ratio = static_cast<double>(front_teeth) / static_cast<double>(rear_teeth);
    const double rear_omega = qvel[static_cast<std::size_t>(joint_dofadr(*m, "rear_wheel_spin"))];
    const double expected_crank_rate = rear_omega / ratio;
    const double cadence_rpm = std::abs(expected_crank_rate) * 60.0 / (2.0 * std::numbers::pi);
    bt::require(cadence_rpm >= rig.physics->drive.shifting.target_cadence_min_rpm - 1e-9 &&
                    cadence_rpm <= rig.physics->drive.shifting.target_cadence_max_rpm + 1e-9,
                std::string(label) + ": initial cassette cog matches target cadence band");

    const double crank_rate = qvel[static_cast<std::size_t>(joint_dofadr(*m, "crank_spin"))];
    const double pedal_f = qvel[static_cast<std::size_t>(joint_dofadr(*m, "pedal_front_spin"))];
    const double pedal_r = qvel[static_cast<std::size_t>(joint_dofadr(*m, "pedal_rear_spin"))];
    bt::require(std::abs(crank_rate - expected_crank_rate) <= 1e-12,
                std::string(label) + ": crank_spin matches rear_omega / ratio");
    bt::require(std::abs(pedal_f + expected_crank_rate) <= 1e-12 &&
                    std::abs(pedal_r + expected_crank_rate) <= 1e-12,
                std::string(label) + ": pedal_front/rear_spin match -crank_rate");
    if (mj_name2id(m, mjOBJ_JOINT, "drive_shaft_spin") >= 0) {
        const double shaft_rate = qvel[static_cast<std::size_t>(joint_dofadr(*m, "drive_shaft_spin"))];
        bt::require(std::abs(shaft_rate - expected_crank_rate) <= 1e-12,
                    std::string(label) + ": drive_shaft_spin matches crank_spin");
    }

    // Verify PresentationState & FlatColumns reads.
    runtime::PresentationState pres;
    rig.physical->initialize_presentation(pres);
    const double expected_vx = v0 * std::cos(expected_pitch_rad) * std::cos(expected_yaw_rad);
    const double expected_vy = v0 * std::cos(expected_pitch_rad) * std::sin(expected_yaw_rad);
    const double expected_horiz_speed =
        std::cos(expected_yaw_rad) * expected_vx + std::sin(expected_yaw_rad) * expected_vy;
    bt::require(std::abs(pres.position_m - qpos[qa]) <= 1e-12,
                std::string(label) + ": presentation position_m matches root_free X");
    bt::require(std::abs(pres.speed_mps - expected_horiz_speed) <= 1e-10,
                std::string(label) + ": presentation speed_mps matches horizontal forward speed");
    bt::require(std::abs(pres.pitch_rad - expected_pitch_rad) <= 1e-10,
                std::string(label) + ": presentation pitch_rad matches euler_zyx pitch");
    const auto wire = pres.as_wire();
    bt::require(!wire.empty(), std::string(label) + ": PresentationState::as_wire succeeds");

    runtime::PhysicalSampleData sample;
    sample.interval_id = 0;
    sample.time_s = 0.;
    sample.end_time_s = rig.rt_cfg->timestep_s;
    sample.qpos.assign(qpos.begin(), qpos.end());
    sample.qvel.assign(qvel.begin(), qvel.end());
    sample.channels.emplace_back("mass", runtime::WireObject{
        {"mass_kg", runtime::Wire(20.0)},
        {"com_m", runtime::WireArray{runtime::Wire(0.), runtime::Wire(0.), runtime::Wire(0.)}}});
    sample.channels.emplace_back("energy", runtime::WireObject{
        {"mechanical_energy_j", runtime::Wire(100.0)},
        {"residual_j", runtime::Wire(0.0)}});

    const runtime::ColumnLayout layout{
        .root_x_qpos = qa,
        .root_x_dof = da,
        .root_pitch_qpos = qa + 4U};
    const auto cols = sample.flat_columns(layout);
    bt::require(std::abs(flat_value({.cols = &cols, .key = "x_m"}) - qpos[qa]) <= 1e-12,
                std::string(label) + ": FlatColumns x_m matches root_free X");
    bt::require(std::abs(flat_value({.cols = &cols, .key = "speed_mps"}) - expected_horiz_speed) <= 1e-10,
                std::string(label) + ": FlatColumns speed_mps matches horizontal forward speed");
    bt::require(std::abs(flat_value({.cols = &cols, .key = "pitch_rad"}) - expected_pitch_rad) <= 1e-10,
                std::string(label) + ": FlatColumns pitch_rad matches euler_zyx pitch");
}

[[gnu::noinline]] void test_toml_request_spatial_and_planar() {
    // 1. Planar fixture remains 100% untouched.
    const auto planar_req = init::read_setup_request(bt::fixture("requests/welded_flat.toml"));
    bt::require(!planar_req.spatial.has_value() && !planar_req.model.spatial.has_value(),
                "planar request leaves spatial disengaged");

    // 2. Combined physics TOML with [spatial] section parses cleanly.
    const TempDir dir;
    std::string phys_toml = bt::text(bt::fixture("config/viewer_physics_welded.toml"));
    phys_toml +=
        "\n[spatial]\n"
        "tire_backend = \"compliant_3d\"\n"
        "tire_preset = \"2p8_1p2bar\"\n"
        "start_mode = \"moving\"\n"
        "corridor_half_width_m = 1.5\n"
        "capsize_roll_rad = 1.0\n"
        "\n[spatial.steer]\n"
        "range_deg = 45.0\n"
        "control_period_s = 0.005\n"
        "reaction_delay_s = 0.015\n"
        "max_torque_nm = 35.0\n";
    write_text_file(dir.path / "physics_spatial.toml", phys_toml);
    std::filesystem::copy_file(bt::fixture("tracks/flat.toml"), dir.path / "flat.toml");
    write_text_file(dir.path / "request.toml",
        "request_schema = 1\n"
        "physics_file = 'physics_spatial.toml'\n"
        "track_file = 'flat.toml'\n"
        "model_mode = 'ride'\n"
        "rider_variant = 'lumped'\n"
        "speed_kph = 18.0\n"
        "seed_decimal = '0'\n"
        "use_equilibrium_cache = false\n");

    const auto spatial_req = init::read_setup_request(dir.path / "request.toml");
    bt::require(spatial_req.spatial.has_value() && spatial_req.model.spatial.has_value(),
                "combined physics TOML populates SetupRequest spatial config");
    if (spatial_req.spatial.has_value()) {
        bt::require(spatial_req.spatial->tire_preset == sp::SpatialTirePreset::mtb_2p8_1p2bar,
                    "parsed spatial tire preset");
    }
    bt::require(std::abs(spatial_req.model.bike.front_wheel_radius - 379.0) < 1e-9 &&
                    std::abs(spatial_req.model.bike.rear_wheel_radius - 360.0) < 1e-9,
                "2.8-inch preset syncs 379/360 mm wheel radii");
}

[[gnu::noinline]] void test_setup_owner_none_and_lumped() {
    const auto build_req = [&](std::string_view variant, std::optional<trn::Terrain3dOverlaySpec> ov) {
        init::SetupRequest req;
        req.track = trn::flat(40.0);
        req.model.mode = "ride";
        req.model.rider = mdl::rider_from_args(variant);
        req.model.physics = make_spatial_physics(5.0, false);
        req.spatial = make_spatial_config(sp::SpatialStartMode::moving);
        req.terrain_overlay = std::move(ov);
        return req;
    };

    // 1. none + default ExtrudedTrack3d
    {
        auto owner = init::make_setup(build_req("none", std::nullopt));
        const auto meta = owner->metadata();
        const auto& obj = bike::io::as_object(meta, "metadata");
        const auto* inputs = bike::io::find_member(obj, "inputs");
        bt::require(inputs != nullptr &&
                        bike::io::find_member(bike::io::as_object(*inputs, "inputs"), "spatial") != nullptr,
                    "spatial none SetupOwner metadata pins spatial inputs");
    }

    // 2. lumped + cross-slope Terrain3dOverlaySpec
    {
        trn::Terrain3dOverlaySpec ov;
        ov.half_width_m = 1.5;
        ov.dx_m = 0.25;
        ov.dy_m = 0.25;
        ov.lead_in_m = 2.0;
        ov.lead_out_m = 2.0;
        ov.cross_slopes.push_back(trn::CrossSlopeSegment3d{
            .start_m = 6.0, .end_m = 30.0, .transition_m = 2.0,
            .bank_rad = 5.0 * std::numbers::pi / 180.0});
        auto owner = init::make_setup(build_req("lumped", ov));
        const auto meta = owner->metadata();
        bt::require(bike::io::find_member(bike::io::as_object(meta, "metadata"), "model_sha256") != nullptr,
                    "spatial lumped SetupOwner with Grid3d overlay compiles and projects cleanly");
    }

    // 3. Reject terrain_overlay without spatial config.
    {
        init::SetupRequest bad;
        bad.track = trn::flat(40.0);
        bad.model.mode = "ride";
        bad.model.rider = mdl::rider_from_args("none");
        bad.model.physics = make_spatial_physics(5.0, false);
        bad.model.field = trn::HeightFieldSpec::for_track(bad.track);
        bad.terrain_overlay = trn::Terrain3dOverlaySpec{};
        bt::invalid([&] { (void)init::make_setup(std::move(bad)); });
    }
}

[[gnu::noinline]] void test_finalize_moving_speed_floor() {
    auto rig = build_finalize_rig(trn::flat(40.0), std::nullopt, 2.49, false);
    seat_spatial_bike_on_terrain(*rig, {.x_m = 10.0, .y_m = 0.0, .yaw_rad = 0.0, .pitch_rad = 0.0});
    const mjModel* m = rig->stepper->model();
    init::SetupContext ctx{
        .stepper = *rig->stepper,
        .physical = *rig->physical,
        .physics = *rig->physics,
        .start_x_m = 10.0,
        .ground_z_m = bike::geometry::compute_ground_z(mdl::BuildRequest{}.bike) / 1000.,
        .frame_body_id = model_access::resolve_id(m, mjOBJ_BODY, "frame"),
        .front_geom_id = model_access::resolve_id(m, mjOBJ_GEOM, "geom_front_contact"),
        .rear_geom_id = model_access::resolve_id(m, mjOBJ_GEOM, "geom_rear_contact"),
    };
    init::EquilibriumResult eq{};
    eq.steps = 1;
    eq.residual_qacc = 0.0;
    ctx.equilibrium = eq;
    bt::invalid([&] { init::finalize_initial_state(ctx, *rig->rt_cfg); });
}

[[gnu::noinline]] void test_finalize_yaw45_and_uphill_cross_slope() {
    // Case 1: Flat ExtrudedTrack3d at yaw = +45 deg, v0 = 5.0 m/s.
    {
        const auto rig = build_finalize_rig(trn::flat(60.0), std::nullopt, 5.0, false);
        const double yaw = std::numbers::pi / 4.0;
        seat_spatial_bike_on_terrain(*rig, {.x_m = 12.0, .y_m = 0.0, .yaw_rad = yaw, .pitch_rad = 0.0});
        verify_finalize_case(*rig, {.v0 = 5.0, .expected_yaw_rad = yaw, .expected_pitch_rad = 0.0, .label = "flat_yaw45"});
    }

    // Case 2: 15% uphill + 5 deg cross-slope Grid3d with motor_clutch = true, v0 = 4.8 m/s.
    {
        trn::TrackSpec climb = trn::flat(60.0);
        climb.grade_profile = trn::GradeProfile{{{0.0, 0.15}, {60.0, 0.15}}};
        trn::Terrain3dOverlaySpec ov;
        ov.half_width_m = 1.5;
        ov.dx_m = 0.25;
        ov.dy_m = 0.25;
        ov.lead_in_m = 2.0;
        ov.lead_out_m = 2.0;
        ov.cross_slopes.push_back(trn::CrossSlopeSegment3d{
            .start_m = 4.0, .end_m = 52.0, .transition_m = 2.0,
            .bank_rad = 5.0 * std::numbers::pi / 180.0});
        const auto rig = build_finalize_rig(climb, ov, 4.8, true);
        const double pitch = -std::atan(0.15);
        seat_spatial_bike_on_terrain(*rig, {.x_m = 15.0, .y_m = 0.0, .yaw_rad = 0.0, .pitch_rad = pitch});
        verify_finalize_case(*rig, {.v0 = 4.8, .expected_yaw_rad = 0.0, .expected_pitch_rad = pitch, .label = "uphill15_bank5"});
    }

    // Case 3: Planar FlatColumns path remains unchanged (nq == nv).
    {
        runtime::PhysicalSampleData planar;
        planar.qpos = {3.25, 0.01, -0.14};
        planar.qvel = {4.50, 0.02, 0.03};
        planar.channels.emplace_back("mass", runtime::WireObject{
            {"mass_kg", runtime::Wire(20.0)},
            {"com_m", runtime::WireArray{runtime::Wire(0.), runtime::Wire(0.), runtime::Wire(0.)}}});
        planar.channels.emplace_back("energy", runtime::WireObject{
            {"mechanical_energy_j", runtime::Wire(10.0)},
            {"residual_j", runtime::Wire(0.0)}});
        const runtime::ColumnLayout layout{
            .root_x_qpos = 0U, .root_x_dof = 0U, .root_pitch_qpos = 2U};
        const auto cols = planar.flat_columns(layout);
        bt::require(flat_value({.cols = &cols, .key = "x_m"}) == 3.25 &&
                        flat_value({.cols = &cols, .key = "speed_mps"}) == 4.50 &&
                        flat_value({.cols = &cols, .key = "pitch_rad"}) == -0.14,
                    "planar FlatColumns preserves scalar root indexing");
    }
}

} // namespace
NATIVE_DIAG_POP

int main() {
    return bt::run([] {
        test_toml_request_spatial_and_planar();
        test_setup_owner_none_and_lumped();
        test_finalize_moving_speed_floor();
        test_finalize_yaw45_and_uphill_cross_slope();
    });
}
