// test_distributed_3d_tire.cpp — unit, 3-stage culling, steady-state
// cornering/trail, 100 mm SquareEdge + 5 deg cross-slope Grid3d energy
// closure, and SetupOwner wiring tests for Distributed3dTireWriter.
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "engaged.hpp"
#include "initialization/equilibrium.hpp"
#include "initialization/setup_owner.hpp"
#include "model/builder.hpp"
#include "spatial/types.hpp"
#include "stepper.hpp"
#include "support/model_fixture.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/grid3d.hpp"
#include "terrain/profile.hpp"
#include "terrain/source3d.hpp"
#include "writer_config.hpp"
#include "writers/distributed_3d_tire.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <vector>

NATIVE_DIAG_PUSH
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace {

bike::terrain::TerrainSample3d test_sample_cb(
    const void *c, double x,
    double y) noexcept { // NOLINT(bugprone-easily-swappable-parameters)
    return static_cast<const bike::terrain::Terrain3dQueryView *>(c)->sample(
        x, y);
}

bike::terrain::TorusClosestPoint3d test_closest_cb(
    const void *c, const bike::terrain::Vec3d &center,
    double search_radius_m) noexcept {
    return static_cast<const bike::terrain::Terrain3dQueryView *>(c)
        ->closest_point(center, search_radius_m);
}

Compliant3dTerrainCallbacks make_test_terrain_callbacks(
    const bike::terrain::Terrain3dQueryView &view) noexcept {
    return Compliant3dTerrainCallbacks{
        .ctx = &view,
        .sample_fn = &test_sample_cb,
        .closest_point_fn = &test_closest_cb,
    };
}

constexpr std::string_view kSpatialBikeXml = R"(<mujoco>
  <compiler angle="degree" coordinate="local" inertiafromgeom="auto"/>
  <option gravity="0 0 -9.81" timestep="0.00125" integrator="Euler"/>
  <default>
    <joint damping="0" stiffness="0" frictionloss="0" armature="0"/>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="frame" pos="0 0 0.5">
      <freejoint name="root_free"/>
      <inertial pos="0.35 0 0.35" mass="80" diaginertia="9.0 11.0 3.0"/>
      <body name="rear_wheel" pos="0 0 -0.140">
        <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="2.5" diaginertia="0.065 0.13 0.065"/>
        <geom name="geom_rear_contact" type="sphere" size="0.360"/>
      </body>
      <body name="steer" pos="1.08 0 -0.15">
        <joint name="steer_joint" type="hinge" axis="-0.3090169943749474 0 0.9510565162951535"/>
        <inertial pos="-0.05 0 0.25" mass="4.0" diaginertia="0.06 0.06 0.01"/>
        <body name="front_wheel" pos="0.02 0 0.029">
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="2.5" diaginertia="0.070 0.14 0.070"/>
          <geom name="geom_front_contact" type="sphere" size="0.379"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>)";

constexpr std::string_view kNonZeroCollisionXml = R"(<mujoco>
  <compiler angle="degree" coordinate="local" inertiafromgeom="auto"/>
  <worldbody>
    <body name="frame" pos="0 0 0.5">
      <freejoint name="root_free"/>
      <geom type="box" size="0.3 0.05 0.1" mass="10" contype="0" conaffinity="0"/>
      <body name="front_wheel" pos="0.5 0 -0.1">
        <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom name="geom_front_contact" type="sphere" size="0.379" mass="2" contype="1" conaffinity="1"/>
      </body>
      <body name="rear_wheel" pos="-0.5 0 -0.1">
        <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom name="geom_rear_contact" type="sphere" size="0.360" mass="2" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>)";

struct BikeIds {
    int front_geom = -1;
    int rear_geom = -1;
    int front_body = -1;
    int rear_body = -1;
    int front_spin_dof = -1;
    int rear_spin_dof = -1;
    int steer_qpos = -1;
};

BikeIds resolve_ids(const mjModel *m) {
    BikeIds ids{};
    ids.front_geom = mj_name2id(m, mjOBJ_GEOM, "geom_front_contact");
    ids.rear_geom = mj_name2id(m, mjOBJ_GEOM, "geom_rear_contact");
    model_access::require_id(ids.front_geom, m->ngeom, "geom_front_contact");
    model_access::require_id(ids.rear_geom, m->ngeom, "geom_rear_contact");
    const auto gbody = model_access::readonly_buffer(m->geom_bodyid, m->ngeom);
    ids.front_body = gbody[static_cast<std::size_t>(ids.front_geom)];
    ids.rear_body = gbody[static_cast<std::size_t>(ids.rear_geom)];

    const auto jdofs = model_access::readonly_buffer(m->jnt_dofadr, m->njnt);
    const auto jqpos = model_access::readonly_buffer(m->jnt_qposadr, m->njnt);
    const int f_jnt = mj_name2id(m, mjOBJ_JOINT, "front_wheel_spin");
    const int r_jnt = mj_name2id(m, mjOBJ_JOINT, "rear_wheel_spin");
    const int s_jnt = mj_name2id(m, mjOBJ_JOINT, "steer_joint");
    model_access::require_id(f_jnt, m->njnt, "front_wheel_spin");
    model_access::require_id(r_jnt, m->njnt, "rear_wheel_spin");
    model_access::require_id(s_jnt, m->njnt, "steer_joint");
    ids.front_spin_dof = jdofs[static_cast<std::size_t>(f_jnt)];
    ids.rear_spin_dof = jdofs[static_cast<std::size_t>(r_jnt)];
    ids.steer_qpos = jqpos[static_cast<std::size_t>(s_jnt)];
    return ids;
}

std::unique_ptr<bike::terrain::Terrain3dSource>
make_plane_terrain(double slope_x, double slope_y, double z0,
                   rider::SurfaceLabel label) {
    const bike::terrain::Grid3dSpec spec{
        .nx = 201,
        .ny = 41,
        .x0_m = -10.0,
        .y0_m = -4.0,
        .dx_m = 0.2,
        .dy_m = 0.2,
    };
    std::vector<double> z(spec.nx * spec.ny, 0.0);
    for (std::size_t ix = 0; ix < spec.nx; ++ix) {
        const double x = spec.x0_m + static_cast<double>(ix) * spec.dx_m;
        for (std::size_t iy = 0; iy < spec.ny; ++iy) {
            const double y = spec.y0_m + static_cast<double>(iy) * spec.dy_m;
            z[ix * spec.ny + iy] = z0 + slope_x * x + slope_y * y;
        }
    }
    std::vector<rider::SurfaceLabel> surf((spec.nx - 1) * (spec.ny - 1), label);
    return std::make_unique<bike::terrain::Terrain3dSource>(
        bike::terrain::Grid3d(spec, std::move(z), std::move(surf)));
}

std::unique_ptr<bike::terrain::Terrain3dSource>
make_step_cross_slope_terrain() {
    bike::terrain::TrackSpec track = bike::terrain::flat(18.0);
    track.obstacles.push_back(bike::terrain::Obstacle{
        .shape = bike::terrain::SquareEdge{
            .start_m = 5.0,
            .height_m = 0.100,
            .ledge_length_m = 4.0,
        },
    });
    bike::terrain::Terrain3dOverlaySpec ov;
    ov.half_width_m = 1.5;
    ov.dx_m = 0.005;
    ov.dy_m = 0.020;
    ov.lead_in_m = 1.0;
    ov.lead_out_m = 1.0;
    ov.cross_slopes.push_back(bike::terrain::CrossSlopeSegment3d{
        .start_m = 0.5,
        .end_m = 15.0,
        .transition_m = 0.5,
        .bank_rad = 5.0 * std::numbers::pi / 180.0,
    });
    return std::make_unique<bike::terrain::Terrain3dSource>(
        bike::terrain::build_overlay_grid3d(track, ov));
}

std::span<mjtNum> qpos_mut(bt::ModelFixture &fixture) {
    return model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq);
}

std::span<mjtNum> qvel_mut(bt::ModelFixture &fixture) {
    return model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv);
}

struct PoseZyx {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
};

void set_zyx_euler_pose(bt::ModelFixture &fixture, const PoseZyx &p) {
    const double cr = std::cos(0.5 * p.roll_rad);
    const double sr = std::sin(0.5 * p.roll_rad);
    const double cp = std::cos(0.5 * p.pitch_rad);
    const double sp = std::sin(0.5 * p.pitch_rad);
    const double cy = std::cos(0.5 * p.yaw_rad);
    const double sy = std::sin(0.5 * p.yaw_rad);
    const double qw = cy * cp * cr + sy * sp * sr;
    const double qx = cy * cp * sr - sy * sp * cr;
    const double qy = cy * sp * cr + sy * cp * sr;
    const double qz = sy * cp * cr - cy * sp * sr;

    const auto qpos = qpos_mut(fixture);
    qpos[0] = p.x;
    qpos[1] = p.y;
    qpos[2] = p.z;
    qpos[3] = qw;
    qpos[4] = qx;
    qpos[5] = qy;
    qpos[6] = qz;
}

double dot_vecs(std::span<const double> a, std::span<const double> b) {
    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

double compute_total_mechanical_energy(const mjModel *m, const mjData *d,
                                       std::vector<double> &mul_buf) {
    const auto nv_u = static_cast<std::size_t>(m->nv);
    mul_buf.assign(nv_u, 0.0);
    engine::mul_m(m, d, mul_buf.data(), d->qvel);
    const auto qvel = model_access::readonly_buffer(d->qvel, m->nv);
    const double ke = 0.5 * dot_vecs(qvel, mul_buf);

    const auto mass = model_access::readonly_buffer(m->body_mass, m->nbody);
    const auto xipos = model_access::readonly_buffer(d->xipos, 3 * m->nbody);
    const auto grav =
        model_access::readonly_buffer(std::data(m->opt.gravity), 3);
    double pe = 0.0;
    for (int i = 0; i < m->nbody; ++i) {
        const auto iu = static_cast<std::size_t>(i);
        const double bmass = mass[iu];
        for (std::size_t k = 0; k < 3; ++k) {
            pe -= bmass * xipos[3U * iu + k] * grav[k];
        }
    }
    return ke + pe;
}

[[gnu::noinline]] void test_presets_config() {
    const auto c28 = make_distributed_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar);
    bt::require(c28.surface_mode == "track", "preset surface_mode is track");
    bt::require(c28.front.station_count == 128 &&
                    c28.rear.station_count == 128,
                "2.8 station_count == 128");
    bt::require(c28.front.pressure_bar == 1.20 &&
                    c28.rear.pressure_bar == 1.20,
                "2.8 preset pressure 1.20 bar");
    bt::require(c28.front.outer_radius_m == 0.3790 &&
                    c28.rear.outer_radius_m == 0.3600,
                "2.8 outer radii");
    bt::require(c28.front.rim_radius_m == 0.3200 &&
                    c28.rear.rim_radius_m == 0.3000,
                "2.8 rim radii");
    bt::require(c28.front.crown_radius_m == 0.0355 &&
                    c28.rear.crown_radius_m == 0.0355,
                "2.8 crown radii");
    bt::require(std::abs(c28.front.major_radius_m() - 0.3435) <= 1e-15 &&
                    std::abs(c28.rear.major_radius_m() - 0.3245) <= 1e-15,
                "2.8 major radii");
    bt::require(std::abs(c28.front.rim_strike_deflection_m() - 0.0530) <=
                        1e-15 &&
                    std::abs(c28.rear.rim_strike_deflection_m() - 0.0540) <=
                        1e-15,
                "2.8 rim-strike deflections");
    bt::require(c28.front.kappa_p_n_m_rad == 201000.0 &&
                    c28.rear.kappa_p_n_m_rad == 195600.0 &&
                    c28.front.delta_0_m == 0.00050 &&
                    c28.front.kappa_c_n_m_rad == 15000.0 &&
                    c28.rear.kappa_c_n_m_rad == 15000.0 &&
                    c28.front.delta_carcass_m == 0.0200 &&
                    c28.front.kappa_rim_n_m_rad == 1400000.0 &&
                    c28.front.delta_rim_0_m == 0.0020 &&
                    c28.front.loss_factor == 0.0550 &&
                    c28.front.kappa_brush_x_n_m_rad == 200000.0 &&
                    c28.rear.kappa_brush_x_n_m_rad == 169000.0 &&
                    c28.front.kappa_brush_y_n_m_rad == 185300.0 &&
                    c28.rear.kappa_brush_y_n_m_rad == 156600.0 &&
                    c28.front.c_kappa_ref_per_fz == 15.0 &&
                    c28.front.mu_cap == 1.10 &&
                    c28.front.valid_load_range_n[1] == 3500.0,
                "2.8 distributed stiffness, hysteresis, and brush constants");

    const auto c24 = make_distributed_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p4_standard);
    bt::require(c24.front.station_count == 128 &&
                    c24.rear.station_count == 128,
                "2.4 station_count == 128");
    bt::require(c24.front.pressure_bar == 1.50 &&
                    c24.rear.pressure_bar == 1.70,
                "2.4 preset pressures");
    bt::require(c24.front.outer_radius_m == 0.3720 &&
                    c24.rear.outer_radius_m == 0.3520,
                "2.4 outer radii");
    bt::require(c24.front.crown_radius_m == 0.0300 &&
                    c24.rear.crown_radius_m == 0.0300,
                "2.4 crown radii");
    bt::require(std::abs(c24.front.major_radius_m() - 0.3420) <= 1e-15 &&
                    std::abs(c24.rear.major_radius_m() - 0.3220) <= 1e-15,
                "2.4 major radii");
    bt::require(std::abs(c24.front.rim_strike_deflection_m() - 0.0460) <=
                        1e-15 &&
                    std::abs(c24.rear.rim_strike_deflection_m() - 0.0460) <=
                        1e-15,
                "2.4 rim-strike deflections");
    bt::require(c24.front.kappa_p_n_m_rad == 197600.0 &&
                    c24.rear.kappa_p_n_m_rad == 212200.0 &&
                    c24.front.kappa_c_n_m_rad == 8000.0 &&
                    c24.rear.kappa_c_n_m_rad == 4000.0 &&
                    c24.front.delta_carcass_m == 0.0180 &&
                    c24.front.loss_factor == 0.0662 &&
                    c24.rear.loss_factor == 0.0690 &&
                    c24.front.kappa_brush_x_n_m_rad == 198000.0 &&
                    c24.rear.kappa_brush_x_n_m_rad == 199700.0 &&
                    c24.front.kappa_brush_y_n_m_rad == 153700.0 &&
                    c24.rear.kappa_brush_y_n_m_rad == 143500.0 &&
                    c24.front.valid_load_range_n[1] == 3000.0,
                "2.4 distributed stiffness, hysteresis, and brush constants");
}

[[gnu::noinline]] void test_validation_probe_and_state_wire() {
    auto terrain =
        make_plane_terrain(0.0, 0.0, 0.0, rider::SurfaceLabel::hardpack);
    const auto view = terrain->view();
    const auto cb = make_test_terrain_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    engine::forward(fixture.model.get(), fixture.data.get());

    bt::invalid([&] {
        (void)std::make_unique<Distributed3dTireWriter>(
            nullptr, fixture.data.get(),
            make_distributed_3d_tire_config(
                bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb));
    });
    bt::invalid([&] {
        const auto bad_cfg = make_distributed_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar);
        (void)std::make_unique<Distributed3dTireWriter>(
            fixture.model.get(), fixture.data.get(), bad_cfg);
    });
    bt::invalid([&] {
        auto bad_cfg = make_distributed_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
        bad_cfg.front.station_count = 16;
        (void)std::make_unique<Distributed3dTireWriter>(
            fixture.model.get(), fixture.data.get(), bad_cfg);
    });
    bt::invalid([&] {
        auto bad_cfg = make_distributed_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
        bad_cfg.surface_mode = "invalid_mode";
        (void)std::make_unique<Distributed3dTireWriter>(
            fixture.model.get(), fixture.data.get(), bad_cfg);
    });
    {
        bt::ModelFixture bad_coll(kNonZeroCollisionXml);
        bt::invalid([&] {
            (void)std::make_unique<Distributed3dTireWriter>(
                bad_coll.model.get(), bad_coll.data.get(),
                make_distributed_3d_tire_config(
                    bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb));
        });
    }

    const auto cfg = make_distributed_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    auto writer_ptr = std::make_unique<Distributed3dTireWriter>(
        fixture.model.get(), fixture.data.get(), cfg);
    Distributed3dTireWriter &writer = *writer_ptr;
    bt::require(writer.radii()[0] == 0.3790 && writer.radii()[1] == 0.3600,
                "radii() returns {R_o_front, R_o_rear}");
    bt::require(!writer.snapshots_committed() && !writer.probe_committed(),
                "uncommitted before first compute");
    bt::require(writer.state_names().size() == 2304,
                "2 * 128 * 9 = 2304 canonical state columns");

    std::vector<double> out(static_cast<std::size_t>(fixture.model->nv), 0.0);
    set_zyx_euler_pose(fixture, {.z = 0.490});
    engine::forward(fixture.model.get(), fixture.data.get());
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.00125, out) ==
            CoreStatus::ok,
        "valid try_compute_into returns ok");
    bt::require(writer.snapshots_committed(), "committed after valid compute");
    bt::require(writer.snapshot_storage()[0].backend == "distributed_3d" &&
                    writer.snapshot_storage()[1].backend == "distributed_3d",
                "snapshot backend is distributed_3d");
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.00125, out) ==
            CoreStatus::invalid_input,
        "double-advance at same timestamp returns invalid_input");
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.0, out) ==
            CoreStatus::invalid_input,
        "non-positive dt returns invalid_input");

    const auto saved_state = writer.state();
    const double saved_t0 = writer.snapshot_storage()[0].time_s;
    fixture.data->time = 0.0025;
    std::vector<double> probe_out(static_cast<std::size_t>(fixture.model->nv),
                                  0.0);
    writer.probe_compute_into(fixture.data.get(), 0.00125, probe_out);
    bt::require(writer.probe_committed(), "probe committed");
    bt::require(writer.state().size() == saved_state.size(),
                "probe leaves state size intact");
    for (std::size_t i = 0; i < saved_state.size(); ++i) {
        const bool both_nan =
            std::isnan(saved_state[i]) && std::isnan(writer.state()[i]);
        bt::require(both_nan || saved_state[i] == writer.state()[i],
                    "probe leaves live state bitwise identical");
    }
    bt::require(writer.snapshot_storage()[0].time_s == saved_t0,
                "probe leaves live snapshot timestamp intact");

    set_zyx_euler_pose(fixture, {.x = 500.0, .z = 0.490});
    engine::forward(fixture.model.get(), fixture.data.get());
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.00125, out) ==
            CoreStatus::invalid_geometry,
        "outside terrain returns CoreStatus::invalid_geometry");
    bt::require(writer.snapshot_storage()[0].time_s == saved_t0,
                "failed tick preserves committed snapshots");

    const auto &names = writer.state_names();
    std::vector<double> bad_row = saved_state;
    bad_row[0] = 2.0;
    bt::invalid([&] { writer.prepare_state(names, bad_row); });
    writer.prepare_state(names, saved_state);
    writer.commit_prepared_state();
    writer.reset();
    bt::require(!writer.snapshots_committed(), "reset clears committed flag");
}

[[gnu::noinline]] void test_3stage_culling_zero_missed_contacts() {
    auto terrain = make_step_cross_slope_terrain();
    const auto view = terrain->view();
    const auto cb = make_test_terrain_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    const BikeIds ids = resolve_ids(fixture.model.get());
    const auto cfg = make_distributed_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    Distributed3dTireWriter writer(fixture.model.get(), fixture.data.get(), cfg);
    std::vector<double> out(static_cast<std::size_t>(fixture.model->nv), 0.0);

    // Sweep poses across the 100 mm SquareEdge (x=5.0), 5 deg cross-slope,
    // high roll camber (20 deg), and corridor boundary (within 18 mm of y_max).
    const std::vector<PoseZyx> poses{
        {.x = 4.92, .y = 0.0, .z = 0.490, .roll_rad = 0.10, .yaw_rad = 0.05},
        {.x = 5.00, .y = 0.2, .z = 0.560, .roll_rad = -0.25, .yaw_rad = 0.08},
        {.x = 4.97, .y = 1.482, .z = 0.620, .roll_rad = 0.0, .yaw_rad = 0.0},
        {.x = 4.97, .y = 1.25, .z = 0.560, .roll_rad = 0.35, .yaw_rad = 0.0},
        {.x = 5.10, .y = 0.0, .z = 0.580, .roll_rad = 0.0, .pitch_rad = -0.15},
    };
    const double dt = 0.00125;
    double time = 0.0;
    const double two_pi = 2.0 * std::numbers::pi;

    for (const auto &pose : poses) {
        writer.reset();
        time += dt;
        fixture.data->time = time;
        set_zyx_euler_pose(fixture, pose);
        std::ranges::fill(qvel_mut(fixture), 0.0);
        engine::forward(fixture.model.get(), fixture.data.get());

        bt::require(
            writer.try_compute_into(fixture.data.get(), dt, out) ==
                CoreStatus::ok,
            "culling pose compute ok");

        // Brute-force check all 128 stations on both front and rear wheels.
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const auto xmat = model_access::readonly_buffer(
            fixture.data->xmat, 9 * fixture.model->nbody);
        for (std::size_t s = 0; s < 2; ++s) {
            const auto &p = (s == 0) ? cfg.front : cfg.rear;
            const int geom = (s == 0) ? ids.front_geom : ids.rear_geom;
            const int body = (s == 0) ? ids.front_body : ids.rear_body;
            const std::size_t g3 = 3U * static_cast<std::size_t>(geom);
            const std::size_t b9 = 9U * static_cast<std::size_t>(body);
            const double r_major = p.major_radius_m();
            const double r_search = p.crown_radius_m + 0.005;

            int brute_active = 0;
            for (int i = 0; i < p.station_count; ++i) {
                const double th =
                    two_pi * static_cast<double>(i) /
                    static_cast<double>(p.station_count);
                const std::array<double, 3> loc{
                    r_major * std::cos(th), 0.0, r_major * std::sin(th)};
                const std::array<double, 3> world{
                    xpos[g3] + xmat[b9 + 0] * loc[0] + xmat[b9 + 1] * loc[1] +
                        xmat[b9 + 2] * loc[2],
                    xpos[g3 + 1] + xmat[b9 + 3] * loc[0] +
                        xmat[b9 + 4] * loc[1] + xmat[b9 + 5] * loc[2],
                    xpos[g3 + 2] + xmat[b9 + 6] * loc[0] +
                        xmat[b9 + 7] * loc[1] + xmat[b9 + 8] * loc[2],
                };
                const auto hit = view.closest_point(world, r_search);
                if (hit.status == bike::terrain::QueryStatus3d::ok &&
                    (p.crown_radius_m - hit.signed_distance_m) > 0.0) {
                    ++brute_active;
                }
            }
            const int writer_active =
                writer.diagnostic_storage()[s].active_station_count;
            bt::require(writer_active == brute_active,
                        "3-stage culling matches brute-force active station "
                        "count (0 missed contacts)");
        }
        bt::require(
            std::abs(writer.stored_energy(fixture.data.get()) -
                     writer.elastic_energy_j()) <= 1e-12,
            "stored_energy matches compute_into at static pose");
    }
}

[[gnu::noinline]] void test_cornering_stiffness_and_pneumatic_trail() {
    struct SlopeProbe {
        bike::spatial::SpatialTirePreset preset =
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar;
        rider::SurfaceLabel surf_label = rider::SurfaceLabel::hardpack;
        double alpha_rad = 0.0;
        double kappa = 0.0;
        bool measure_lateral = false;
        double target_load_n = 466.0;
    };
    struct SlopeOutcome {
        double ratio = 0.0;
        double fy_total = 0.0;
        double mz_about_hub = 0.0;
        double fx_total = 0.0;
        double fn_total = 0.0;
    };
    const auto eval_slope = [](const SlopeProbe &probe) -> SlopeOutcome {
        auto terrain = make_plane_terrain(0.0, 0.0, 0.0, probe.surf_label);
        const auto view = terrain->view();
        const auto cb = make_test_terrain_callbacks(view);
        bt::ModelFixture fixture(kSpatialBikeXml);
        const BikeIds ids = resolve_ids(fixture.model.get());
        auto cfg = make_distributed_3d_tire_config(probe.preset, cb);
        // Test pure steady-state lateral/longitudinal force without radial hysteresis bias.
        cfg.front.loss_factor = 0.0;
        cfg.rear.loss_factor = 0.0;
        Distributed3dTireWriter writer(fixture.model.get(), fixture.data.get(),
                                       cfg);
        std::vector<double> out(static_cast<std::size_t>(fixture.model->nv),
                                0.0);

        const double dt = 0.00125;
        const double v_x = 5.0;
        // Bisect deflection giving F_z ~ target_load_n on the front wheel.
        const double r_front = cfg.front.outer_radius_m;
        double d_lo = 0.0005;
        double d_hi = 0.0350;
        double delta = 0.010;
        for (int it = 0; it < 30; ++it) {
            delta = 0.5 * (d_lo + d_hi);
            writer.reset();
            set_zyx_euler_pose(fixture, {.z = 0.500, .pitch_rad = +0.35});
            std::ranges::fill(qvel_mut(fixture), 0.0);
            engine::forward(fixture.model.get(), fixture.data.get());
            const auto xpos = model_access::readonly_buffer(
                fixture.data->geom_xpos, 3 * fixture.model->ngeom);
            const double front_hub_z =
                xpos[3U * static_cast<std::size_t>(ids.front_geom) + 2];
            qpos_mut(fixture)[2] += (r_front - delta) - front_hub_z;
            engine::forward(fixture.model.get(), fixture.data.get());
            fixture.data->time = static_cast<double>(it + 1) * dt;
            writer.compute_into(fixture.data.get(), dt, out);
            const double fz = writer.diagnostic_storage()[0].normal_load_n;
            if (std::abs(fz - probe.target_load_n) < 0.2) break;
            if (fz < probe.target_load_n) {
                d_lo = delta;
            } else {
                d_hi = delta;
            }
        }
        writer.reset();

        const double r_eff = r_front - delta;
        const double v_y = -v_x * std::tan(probe.alpha_rad);
        const double omega = (1.0 + probe.kappa) * v_x / r_eff;

        set_zyx_euler_pose(fixture, {.z = 0.500, .pitch_rad = +0.35});
        engine::forward(fixture.model.get(), fixture.data.get());
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const double front_hub_z =
            xpos[3U * static_cast<std::size_t>(ids.front_geom) + 2];
        qpos_mut(fixture)[2] += (r_front - delta) - front_hub_z;

        const auto qvel = qvel_mut(fixture);
        std::ranges::fill(qvel, 0.0);
        qvel[0] = v_x;
        qvel[1] = v_y;
        qvel[static_cast<std::size_t>(ids.front_spin_dof)] = omega;
        engine::forward(fixture.model.get(), fixture.data.get());

        // Roll forward kinematically for 0.35 s and average over the last 80 steps.
        const int f_jnt = mj_name2id(fixture.model.get(), mjOBJ_JOINT,
                                     "front_wheel_spin");
        const auto f_qpos = static_cast<std::size_t>(
            model_access::readonly_buffer(
                fixture.model->jnt_qposadr,
                fixture.model->njnt)[static_cast<std::size_t>(f_jnt)]);
        const double t0 = 1.0;
        double fx_tot = 0.0;
        double fy_tot = 0.0;
        double fz_tot = 0.0;
        double mz_tot = 0.0;
        int avg_count = 0;
        for (int step = 1; step <= 280; ++step) {
            const double t = t0 + static_cast<double>(step) * dt;
            fixture.data->time = t;
            qpos_mut(fixture)[f_qpos] += omega * dt;
            engine::forward(fixture.model.get(), fixture.data.get());
            writer.compute_into(fixture.data.get(), dt, out);
            if (step > 200) {
                const auto xpos_step = model_access::readonly_buffer(
                    fixture.data->geom_xpos, 3 * fixture.model->ngeom);
                const double hx =
                    xpos_step[3U * static_cast<std::size_t>(ids.front_geom) + 0];
                const double hy =
                    xpos_step[3U * static_cast<std::size_t>(ids.front_geom) + 1];
                const auto &snap = writer.snapshot_storage()[0];
                for (const auto &patch : snap.patches) {
                    const auto &f = engaged(patch.native_world_force_n);
                    fx_tot += f[0];
                    fy_tot += f[1];
                    fz_tot += f[2];
                    const double rx = patch.point_m[0] - hx;
                    const double ry = patch.point_m[1] - hy;
                    mz_tot += rx * f[1] - ry * f[0];
                }
                ++avg_count;
            }
        }
        const double inv_n = 1.0 / static_cast<double>(avg_count);
        fx_tot *= inv_n;
        fy_tot *= inv_n;
        fz_tot *= inv_n;
        mz_tot *= inv_n;
        double ratio = 0.0;
        if (probe.measure_lateral) {
            ratio = fy_tot / (fz_tot * std::tan(probe.alpha_rad));
        } else if (std::abs(probe.kappa) > 0.0) {
            ratio = fx_tot / (fz_tot * probe.kappa);
        }
        return {
            .ratio = ratio,
            .fy_total = fy_tot,
            .mz_about_hub = mz_tot,
            .fx_total = fx_tot,
            .fn_total = fz_tot,
        };
    };

    const auto corner_28 =
        eval_slope({.preset = bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar,
                    .surf_label = rider::SurfaceLabel::hardpack,
                    .alpha_rad = 0.5 * std::numbers::pi / 180.0,
                    .measure_lateral = true,
                    .target_load_n = 466.0});
    bt::require(std::abs(corner_28.ratio - 13.90) / 13.90 <= 0.02,
                "steady-state 2.8 C_alpha/F_z matches 13.90 rad^-1 within 2%");
    bt::require(corner_28.mz_about_hub / corner_28.fy_total < 0.0,
                "steady cornering exhibits positive restoring pneumatic trail "
                "(Mz/Fy < 0)");

    const auto free_roll =
        eval_slope({.preset = bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar,
                    .surf_label = rider::SurfaceLabel::asphalt,
                    .alpha_rad = 0.0,
                    .kappa = 0.0,
                    .measure_lateral = false,
                    .target_load_n = 466.0});
    bt::require(std::abs(free_roll.fx_total) / free_roll.fn_total < 5e-4,
                "free-rolling bristle scrub drag vanishes on flat ground");
}

[[gnu::noinline]] void test_square_edge_multi_support_and_energy_closure() {
    auto terrain = make_step_cross_slope_terrain();
    const auto view = terrain->view();
    const auto cb = make_test_terrain_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    const BikeIds ids = resolve_ids(fixture.model.get());
    const auto cfg = make_distributed_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    Distributed3dTireWriter writer(fixture.model.get(), fixture.data.get(), cfg);

    const std::size_t nv_u = static_cast<std::size_t>(fixture.model->nv);
    std::vector<double> qfrc_tire(nv_u, 0.0);
    std::vector<double> v_start(nv_u, 0.0);
    std::vector<double> v_mid(nv_u, 0.0);
    std::vector<double> mul_buf(nv_u, 0.0);

    // Start before the 100 mm SquareEdge (at x=5.0) with 5 deg cross-slope and roll across it.
    const double bank = 5.0 * std::numbers::pi / 180.0;
    const auto start_smp = view.sample(2.5, 0.0);
    bt::require(start_smp.status == bike::terrain::QueryStatus3d::ok,
                "start terrain sample ok");
    set_zyx_euler_pose(fixture,
                       {.x = 2.5,
                        .y = 0.0,
                        .z = start_smp.height_m + 0.505,
                        .roll_rad = bank,
                        .yaw_rad = 0.0});
    const auto qvel_init = qvel_mut(fixture);
    std::ranges::fill(qvel_init, 0.0);
    const double v0 = 4.5;
    qvel_init[0] = v0;
    qvel_init[static_cast<std::size_t>(ids.front_spin_dof)] =
        v0 / cfg.front.outer_radius_m;
    qvel_init[static_cast<std::size_t>(ids.rear_spin_dof)] =
        v0 / cfg.rear.outer_radius_m;
    engine::forward(fixture.model.get(), fixture.data.get());

    const double e_initial =
        compute_total_mechanical_energy(fixture.model.get(), fixture.data.get(),
                                        mul_buf) +
        writer.stored_energy(fixture.data.get());
    const double scale = std::max(std::abs(e_initial), 1.0);

    const double dt = 0.00125;
    double loss_total = 0.0;
    double max_rel_residual = 0.0;
    bool saw_multi_support = false;

    for (int step = 0; step < 960; ++step) {
        const CoreStatus status =
            writer.try_compute_into(fixture.data.get(), dt, qfrc_tire);
        bt::require(status == CoreStatus::ok,
                    "step impact try_compute_into ok");
        if (writer.diagnostic_storage()[0].multi_support ||
            writer.diagnostic_storage()[1].multi_support) {
            saw_multi_support = true;
        }

        const auto qfrc_app = model_access::mutable_buffer(
            fixture.data->qfrc_applied, fixture.model->nv);
        std::ranges::copy(qfrc_tire, qfrc_app.begin());
        const auto qvel = model_access::readonly_buffer(fixture.data->qvel,
                                                        fixture.model->nv);
        std::ranges::copy(qvel, v_start.begin());

        engine::step(fixture.model.get(), fixture.data.get());

        const auto qvel_end = model_access::readonly_buffer(fixture.data->qvel,
                                                            fixture.model->nv);
        for (std::size_t i = 0; i < nv_u; ++i) {
            v_mid[i] = 0.5 * (v_start[i] + qvel_end[i]);
        }
        loss_total += writer.step_loss_j(v_mid, dt);

        engine::forward(fixture.model.get(), fixture.data.get());
        const double e_curr =
            compute_total_mechanical_energy(fixture.model.get(),
                                            fixture.data.get(), mul_buf) +
            writer.stored_energy(fixture.data.get());
        const double rel_residual =
            std::abs(e_curr - e_initial + loss_total) / scale;
        max_rel_residual = std::max(max_rel_residual, rel_residual);
    }

    bt::require(saw_multi_support,
                "100 mm SquareEdge step impact triggers multi_support == true");
    bt::require(loss_total > 1.0,
                "SquareEdge + cross-slope impact dissipates real energy");
    bt::require(max_rel_residual <= 0.01,
                "100 mm SquareEdge + 5 deg cross-slope energy closes within 1%");
}

[[gnu::noinline]] void test_setup_owner_distributed_3d() {
    auto phys = std::make_unique<bike::configuration::SimulationPhysicsConfig>(
        bike::configuration::default_physics());
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
    bike::configuration::post_init(*phys);

    bike::spatial::SpatialConfig sp{};
    sp.tire_backend = bike::spatial::SpatialTireBackend::distributed_3d;
    sp.tire_preset = bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar;
    sp.start_mode = bike::spatial::SpatialStartMode::moving;
    sp.corridor_half_width_m = 1.5;
    sp.capsize_roll_rad = 1.0;
    sp.steer.range_deg = 45.0;
    sp.steer.control_period_s = 0.005;
    sp.steer.reaction_delay_s = 0.010;
    sp.steer.max_torque_nm = 35.0;

    bike::initialization::SetupRequest req;
    req.track = bike::terrain::flat(40.0);
    req.start_x_m = 10.0;
    req.model.mode = "ride";
    req.model.include_rider = false;
    req.model.rider = bike::model::rider_from_args("none");
    req.model.physics = std::move(*phys);
    req.spatial = sp;
    req.model.field = bike::terrain::HeightFieldSpec::for_track(req.track);

    const auto owner = bike::initialization::make_setup(std::move(req));
    const auto eq = bike::initialization::settle(*owner);
    bt::require(eq.steps > 0 && std::isfinite(eq.residual_qacc) &&
                    eq.residual_qacc <= 0.05,
                "SetupOwner with distributed_3d settles cleanly");
    const auto boot = owner->seal_bootstrap();
    bt::require(boot.config.spatial.has_value() &&
                    boot.config.spatial->tire_backend ==
                        bike::spatial::SpatialTireBackend::distributed_3d,
                "sealed bootstrap preserves distributed_3d backend");
    bt::require(boot.state.tire.names.size() == 2304 &&
                    boot.state.tire.row.size() == 2304,
                "SetupOwner wires 2304-column Distributed3dTireWriter state");
}

} // namespace

// cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
int main() {
    return bt::run([] {
        test_presets_config();
        test_validation_probe_and_state_wire();
        test_3stage_culling_zero_missed_contacts();
        test_cornering_stiffness_and_pneumatic_trail();
        test_square_edge_multi_support_and_energy_closure();
        test_setup_owner_distributed_3d();
    });
}

NATIVE_DIAG_POP
