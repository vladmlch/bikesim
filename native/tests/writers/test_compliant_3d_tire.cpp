// test_compliant_3d_tire.cpp — unit, passivity, cornering, camber, and 2.0 s
// 3D energy-closure contract tests for Compliant3dTireWriter.
#include "engaged.hpp"
#include "support/model_fixture.hpp"
#include "terrain/source3d.hpp"
#include "writers/compliant_3d_tire.hpp"

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

namespace {

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
    // q = q_z(yaw) * q_y(pitch) * q_x(roll)
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

std::array<double, 3> wheel_axis_world(const mjModel *m, const mjData *d,
                                       int body_id) {
    const auto xmat = model_access::readonly_buffer(d->xmat, 9 * m->nbody);
    const std::size_t b9 = 9U * static_cast<std::size_t>(body_id);
    return {xmat[b9 + 1], xmat[b9 + 4], xmat[b9 + 7]};
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
    const auto c28 = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar);
    bt::require(c28.surface_mode == "track", "preset surface_mode is track");
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
    bt::require(c28.front.k_pneumatic_n_m == 53290.0 &&
                    c28.rear.k_pneumatic_n_m == 53290.0 &&
                    c28.front.k_carcass_n_m == 18650.0 &&
                    c28.rear.k_carcass_n_m == 18790.0 &&
                    c28.front.loss_factor == 0.0550 &&
                    c28.front.c_alpha_per_fz == 13.90 &&
                    c28.front.k_brush_x_n_m == 83600.0 &&
                    c28.front.k_brush_y_n_m == 36300.0 &&
                    c28.front.mu_cap == 1.10 &&
                    c28.front.valid_load_range_n[1] == 3500.0,
                "2.8 stiffness, hysteresis, and brush constants");

    const auto c24 = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p4_standard);
    bt::require(c24.front.pressure_bar == 1.50 &&
                    c24.rear.pressure_bar == 1.70,
                "2.4 preset pressures");
    bt::require(c24.front.outer_radius_m == 0.3720 &&
                    c24.rear.outer_radius_m == 0.3520,
                "2.4 outer radii");
    bt::require(c24.front.crown_radius_m == 0.0300 &&
                    c24.rear.crown_radius_m == 0.0300,
                "2.4 crown radii");
    bt::require(std::abs(c24.front.rim_strike_deflection_m() - 0.0460) <=
                        1e-15 &&
                    std::abs(c24.rear.rim_strike_deflection_m() - 0.0460) <=
                        1e-15,
                "2.4 rim-strike deflections");
    bt::require(c24.front.k_pneumatic_n_m == 54150.0 &&
                    c24.rear.k_pneumatic_n_m == 61070.0 &&
                    c24.front.k_carcass_n_m == 8170.0 &&
                    c24.rear.k_carcass_n_m == 3380.0 &&
                    c24.front.loss_factor == 0.0662 &&
                    c24.rear.loss_factor == 0.0690 &&
                    c24.front.c_alpha_per_fz == 11.65 &&
                    c24.rear.c_alpha_per_fz == 10.78 &&
                    c24.front.k_brush_y_n_m == 30800.0 &&
                    c24.rear.k_brush_y_n_m == 28500.0 &&
                    c24.front.valid_load_range_n[1] == 3000.0,
                "2.4 stiffness, hysteresis, and brush constants");
}

[[gnu::noinline]] void test_validation_probe_and_state() {
    auto terrain =
        make_plane_terrain(0.0, 0.0, 0.0, rider::SurfaceLabel::hardpack);
    const auto view = terrain->view();
    const auto cb = make_terrain3d_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    engine::forward(fixture.model.get(), fixture.data.get());

    bt::invalid([&] {
        (void)std::make_unique<Compliant3dTireWriter>(
            nullptr, fixture.data.get(),
            make_compliant_3d_tire_config(
                bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb));
    });
    bt::invalid([&] {
        const auto bad_cfg = make_compliant_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar);
        (void)std::make_unique<Compliant3dTireWriter>(
            fixture.model.get(), fixture.data.get(), bad_cfg);
    });
    bt::invalid([&] {
        auto bad_cfg = make_compliant_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
        bad_cfg.surface_mode = "invalid_mode";
        (void)std::make_unique<Compliant3dTireWriter>(
            fixture.model.get(), fixture.data.get(), bad_cfg);
    });
    bt::invalid([&] {
        auto bad_cfg = make_compliant_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
        bad_cfg.front.loss_factor = 1.0;
        (void)std::make_unique<Compliant3dTireWriter>(
            fixture.model.get(), fixture.data.get(), bad_cfg);
    });
    {
        bt::ModelFixture bad_coll(kNonZeroCollisionXml);
        bt::invalid([&] {
            (void)std::make_unique<Compliant3dTireWriter>(
                bad_coll.model.get(), bad_coll.data.get(),
                make_compliant_3d_tire_config(
                    bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb));
        });
    }

    const auto cfg = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    auto writer_ptr = std::make_unique<Compliant3dTireWriter>(
        fixture.model.get(), fixture.data.get(), cfg);
    Compliant3dTireWriter &writer = *writer_ptr;
    bt::require(writer.radii()[0] == 0.3790 && writer.radii()[1] == 0.3600,
                "radii() returns {R_o_front, R_o_rear}");
    bt::require(!writer.snapshots_committed() && !writer.probe_committed(),
                "uncommitted before first compute");
    bt::require(Compliant3dTireWriter::state_names().size() == 28,
                "28 canonical state columns");

    std::vector<double> out(static_cast<std::size_t>(fixture.model->nv), 0.0);
    set_zyx_euler_pose(fixture, {.z = 0.495});
    engine::forward(fixture.model.get(), fixture.data.get());
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.00125, out) ==
            CoreStatus::ok,
        "valid try_compute_into returns ok");
    bt::require(writer.snapshots_committed(), "committed after valid compute");
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.00125, out) ==
            CoreStatus::invalid_input,
        "double-advance at same timestamp returns invalid_input");
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.0, out) ==
            CoreStatus::invalid_input,
        "non-positive dt returns invalid_input");

    // Probe does not alter live state or snapshots.
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

    // Outside terrain bounds returns CoreStatus::invalid_geometry without corrupting state.
    set_zyx_euler_pose(fixture, {.x = 500.0, .z = 0.495});
    engine::forward(fixture.model.get(), fixture.data.get());
    bt::require(
        writer.try_compute_into(fixture.data.get(), 0.00125, out) ==
            CoreStatus::invalid_geometry,
        "outside terrain returns CoreStatus::invalid_geometry");
    bt::require(writer.snapshot_storage()[0].time_s == saved_t0,
                "failed tick preserves committed snapshots");

    // Two-phase prepare_state / commit_prepared_state roundtrip.
    const auto &names = Compliant3dTireWriter::state_names();
    std::vector<double> bad_row = saved_state;
    bad_row[0] = std::numeric_limits<double>::infinity();
    bt::invalid([&] { writer.prepare_state(names, bad_row); });
    writer.prepare_state(names, saved_state);
    writer.commit_prepared_state();
    writer.reset();
    bt::require(!writer.snapshots_committed(), "reset clears committed flag");
}

[[gnu::noinline]] void test_radial_c1_liftoff_potential_and_hysteresis() {
    auto terrain =
        make_plane_terrain(0.0, 0.0, 0.0, rider::SurfaceLabel::asphalt);
    const auto view = terrain->view();
    const auto cb = make_terrain3d_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    const BikeIds ids = resolve_ids(fixture.model.get());
    const auto cfg = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    Compliant3dTireWriter writer(fixture.model.get(), fixture.data.get(), cfg);
    std::vector<double> out(static_cast<std::size_t>(fixture.model->nv), 0.0);

    // Position rear wheel over z = 0; lift front wheel above ground by pitching nose up.
    // Note: in MJCF, rear_wheel is at frame + (0, 0, -0.140), radius 0.360 -> hub z = z_frame - 0.140.
    // At z_frame = 0.500 - delta, rear hub z = 0.360 - delta -> rear delta = delta.
    // Pitching nose up (pitch_rad < 0 in Z-up where +Y pitch is nose down) lifts front wheel airborne.
    const double dt = 0.00125;
    double time = 0.0;
    const std::array<double, 10> deltas = {-0.005, 0.0,    1e-7,   1e-5,
                                           1e-4,   5e-4,   0.005,  0.025,
                                           0.053,  0.058};
    for (const double delta : deltas) {
        writer.reset();
        time += dt;
        fixture.data->time = time;
        set_zyx_euler_pose(fixture, {.z = 0.500 - delta, .pitch_rad = -0.35});
        // Set zero velocities first to measure pure elastic F_el and U_rad.
        std::ranges::fill(qvel_mut(fixture), 0.0);
        engine::forward(fixture.model.get(), fixture.data.get());

        // Adjust z so rear hub z is exact (pitching about frame origin moves rear_wheel slightly).
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const double rear_hub_z =
            xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2];
        const double z_corr = (0.360 - delta) - rear_hub_z;
        qpos_mut(fixture)[2] += z_corr;
        engine::forward(fixture.model.get(), fixture.data.get());

        writer.compute_into(fixture.data.get(), dt, out);
        const auto &rear_d = writer.diagnostic_storage()[1];
        const auto &front_d = writer.diagnostic_storage()[0];
        bt::require(front_d.normal_load_n == 0.0 && front_d.penetration_m < 0.0,
                    "front wheel stays airborne");
        bt::require(std::abs(rear_d.penetration_m - delta) <= 1e-12,
                    "rear penetration matches imposed delta");

        if (delta < 0.0) {
            bt::require(rear_d.normal_load_n == 0.0,
                        "zero normal load when delta < 0");
            bt::require(rear_d.radial_energy_j == 0.0,
                        "zero radial energy when delta < 0");
            bt::require(writer.stored_energy(fixture.data.get()) == 0.0,
                        "stored_energy == 0 when airborne");
            bt::require(!writer.snapshot_storage()[1].geometric_contact,
                        "geometric_contact false when delta < 0");
        } else if (delta == 0.0) {
            bt::require(rear_d.normal_load_n <= 1e-18,
                        "C1 normal load vanishes at delta == 0");
            bt::require(rear_d.radial_energy_j <= 1e-30,
                        "radial energy vanishes at delta == 0");
        } else {
            bt::require(
                std::abs(writer.stored_energy(fixture.data.get()) -
                         writer.elastic_energy_j()) <= 1e-12,
                "stored_energy matches elastic_energy_j at fixed pose");
        }

        // Check fast liftoff at delta = 1e-7 m with delta_dot = -2.0 m/s (no viscous stick).
        if (delta == 1e-7) {
            writer.reset();
            time += dt;
            fixture.data->time = time;
            qvel_mut(fixture)[2] = 2.0; // +vz => delta_dot = -2.0 m/s
            engine::forward(fixture.model.get(), fixture.data.get());
            writer.compute_into(fixture.data.get(), dt, out);
            const double fn_lift =
                writer.diagnostic_storage()[1].normal_load_n;
            bt::require(fn_lift >= 0.0 && fn_lift < 1e-3,
                        "C1 liftoff at delta=1e-7 has zero viscous stick");
            bt::require(writer.radial_dissipation_power_w() >= 0.0,
                        "radial dissipation >= 0 during rebound");
        }
    }

    // Verify dU_rad / d_delta == F_el(delta) across Taylor (< 1e-3 * x0) and log1p branches.
    const std::array<double, 6> fd_deltas = {1e-7, 1e-5, 5e-4, 0.005, 0.030,
                                             0.058};
    for (const double delta : fd_deltas) {
        const double h = 1e-3 * delta;
        const auto eval_at = [&](double d_val) -> std::pair<double, double> {
            writer.reset();
            time += dt;
            fixture.data->time = time;
            std::ranges::fill(qvel_mut(fixture), 0.0);
            set_zyx_euler_pose(fixture, {.z = 0.5, .pitch_rad = -0.35});
            engine::forward(fixture.model.get(), fixture.data.get());
            const auto xpos = model_access::readonly_buffer(
                fixture.data->geom_xpos, 3 * fixture.model->ngeom);
            const double rear_hub_z =
                xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2];
            qpos_mut(fixture)[2] += (0.360 - d_val) - rear_hub_z;
            engine::forward(fixture.model.get(), fixture.data.get());
            writer.compute_into(fixture.data.get(), dt, out);
            return {writer.diagnostic_storage()[1].normal_load_n,
                    writer.diagnostic_storage()[1].radial_energy_j};
        };
        const auto [f_mid, u_mid] = eval_at(delta);
        const auto [f_p1, u_p1] = eval_at(delta + h);
        const auto [f_m1, u_m1] = eval_at(delta - h);
        const auto [f_p2, u_p2] = eval_at(delta + 2.0 * h);
        const auto [f_m2, u_m2] = eval_at(delta - 2.0 * h);
        (void)u_mid;
        (void)f_p1;
        (void)f_m1;
        (void)f_p2;
        (void)f_m2;
        const double du_ddelta =
            (8.0 * (u_p1 - u_m1) - (u_p2 - u_m2)) / (12.0 * h);
        const double rel_err = std::abs(du_ddelta - f_mid) /
                               std::max(1e-12, std::abs(f_mid));
        bt::require(rel_err < 1e-5,
                    "dU_rad/d_delta equals elastic normal force F_el");
    }

    // Verify hysteresis dissipation >= 0 in both compression (+0.5 m/s) and rebound (-0.5 m/s).
    for (const double vz : {-0.5, +0.5}) {
        writer.reset();
        time += dt;
        fixture.data->time = time;
        std::ranges::fill(qvel_mut(fixture), 0.0);
        set_zyx_euler_pose(fixture, {.z = 0.5, .pitch_rad = -0.35});
        engine::forward(fixture.model.get(), fixture.data.get());
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const double rear_hub_z =
            xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2];
        qpos_mut(fixture)[2] += (0.360 - 0.015) - rear_hub_z;
        qvel_mut(fixture)[2] = vz;
        engine::forward(fixture.model.get(), fixture.data.get());
        writer.compute_into(fixture.data.get(), dt, out);
        bt::require(writer.radial_dissipation_power_w() > 0.0,
                    "hysteresis dissipation strictly positive for vz != 0");
    }
}

[[gnu::noinline]] void test_cornering_and_longitudinal_stiffness() {
    struct SlopeProbe {
        rider::SurfaceLabel surf_label = rider::SurfaceLabel::hardpack;
        double alpha_rad = 0.0;
        double kappa = 0.0;
        bool measure_lateral = false;
    };
    const auto eval_slope = [](const SlopeProbe &probe) -> double {
        auto terrain = make_plane_terrain(0.0, 0.0, 0.0, probe.surf_label);
        const auto view = terrain->view();
        const auto cb = make_terrain3d_callbacks(view);
        bt::ModelFixture fixture(kSpatialBikeXml);
        const BikeIds ids = resolve_ids(fixture.model.get());
        const auto cfg = make_compliant_3d_tire_config(
            bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
        Compliant3dTireWriter writer(fixture.model.get(), fixture.data.get(),
                                     cfg);
        std::vector<double> out(static_cast<std::size_t>(fixture.model->nv),
                                0.0);

        const double dt = 0.00125;
        const double v_x = 6.0;
        const double delta = 0.008;
        const double r_eff = 0.360 - delta;
        const double v_y = -v_x * std::tan(probe.alpha_rad);
        const double omega = (1.0 + probe.kappa) * v_x / r_eff;

        set_zyx_euler_pose(fixture, {.z = 0.500, .pitch_rad = -0.35});
        engine::forward(fixture.model.get(), fixture.data.get());
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const double rear_hub_z =
            xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2];
        qpos_mut(fixture)[2] += (0.360 - delta) - rear_hub_z;

        const auto qvel = qvel_mut(fixture);
        std::ranges::fill(qvel, 0.0);
        qvel[0] = v_x;
        qvel[1] = v_y;
        qvel[static_cast<std::size_t>(ids.rear_spin_dof)] = omega;
        engine::forward(fixture.model.get(), fixture.data.get());

        for (int step = 1; step <= 260; ++step) {
            fixture.data->time = static_cast<double>(step) * dt;
            writer.compute_into(fixture.data.get(), dt, out);
        }
        const auto &patch = writer.snapshot_storage()[1].patches.at(0);
        const auto &f_w = engaged(patch.native_world_force_n);
        const double fn = patch.normal_load_n;
        if (probe.measure_lateral) {
            return f_w[1] / (fn * std::tan(probe.alpha_rad));
        }
        return f_w[0] / (fn * probe.kappa);
    };

    const double c_alpha_1deg =
        eval_slope({.surf_label = rider::SurfaceLabel::hardpack,
                    .alpha_rad = 1.0 * std::numbers::pi / 180.0,
                    .measure_lateral = true});
    bt::require(std::abs(c_alpha_1deg - 13.90) / 13.90 <= 0.02,
                "steady-state C_alpha/F_z matches 13.90 rad^-1 within 2%");

    const double c_kappa_asphalt =
        eval_slope({.surf_label = rider::SurfaceLabel::asphalt, .kappa = 0.005});
    bt::require(std::abs(c_kappa_asphalt - 15.0) / 15.0 <= 0.02,
                "asphalt C_kappa/F_z matches 15.0 within 2%");

    const double c_kappa_scree =
        eval_slope({.surf_label = rider::SurfaceLabel::scree, .kappa = 0.005});
    bt::require(std::abs(c_kappa_scree - 4.5) / 4.5 <= 0.02,
                "scree C_kappa/F_z matches 4.5 within 2%");
}

[[gnu::noinline]] void test_stribeck_saturation_and_discrete_passivity() {
    auto terrain =
        make_plane_terrain(0.0, 0.0, 0.0, rider::SurfaceLabel::loose);
    const auto view = terrain->view();
    const auto cb = make_terrain3d_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    const BikeIds ids = resolve_ids(fixture.model.get());
    const auto cfg = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    Compliant3dTireWriter writer(fixture.model.get(), fixture.data.get(), cfg);
    std::vector<double> out(static_cast<std::size_t>(fixture.model->nv), 0.0);

    const double dt = 0.00125;
    double time = 0.0;
    const double delta = 0.015;

    // 1. Combined slip saturation + Stribeck decay on loose (mu_peak=0.55, mu_slide=0.45, V_str=4.5).
    for (const double v_slip : {0.2, 4.5, 15.0}) {
        writer.reset();
        set_zyx_euler_pose(fixture, {.z = 0.5, .pitch_rad = -0.35});
        engine::forward(fixture.model.get(), fixture.data.get());
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const double rear_hub_z =
            xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2];
        qpos_mut(fixture)[2] += (0.360 - delta) - rear_hub_z;
        const auto qvel = qvel_mut(fixture);
        std::ranges::fill(qvel, 0.0);
        qvel[0] = v_slip * 0.6;
        qvel[1] = v_slip * 0.8;
        engine::forward(fixture.model.get(), fixture.data.get());

        for (int k = 0; k < 80; ++k) {
            time += dt;
            fixture.data->time = time;
            writer.compute_into(fixture.data.get(), dt, out);
        }
        const auto &patch = writer.snapshot_storage()[1].patches.at(0);
        const auto &f_w = engaged(patch.native_world_force_n);
        const double ft = std::hypot(f_w[0], f_w[1]);
        const double expected_mu =
            0.45 + (0.55 - 0.45) * std::exp(-v_slip / 4.5);
        bt::require(
            std::abs(writer.diagnostic_storage()[1].friction_coefficient -
                     expected_mu) <= 1e-12,
            "Stribeck mu matches surface law");
        bt::require(ft <= expected_mu * patch.normal_load_n + 1e-11,
                    "tangential force obeys friction circle bound");
        if (v_slip >= 4.5) {
            bt::require(
                std::abs(ft - expected_mu * patch.normal_load_n) <= 1e-9,
                "high slip saturates exactly onto Stribeck friction circle");
        }
    }

    // 2. Exact discrete passivity identity under yaw rotation (exercises beta guard & parallel transport).
    writer.reset();
    double prev_shear_energy = 0.0;
    for (int k = 0; k < 20; ++k) {
        time += dt;
        fixture.data->time = time;
        const double yaw = 0.03 * static_cast<double>(k);
        set_zyx_euler_pose(fixture, {.z = 0.5, .pitch_rad = -0.35, .yaw_rad = yaw});
        engine::forward(fixture.model.get(), fixture.data.get());
        const auto xpos = model_access::readonly_buffer(
            fixture.data->geom_xpos, 3 * fixture.model->ngeom);
        const double rear_hub_z =
            xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2];
        qpos_mut(fixture)[2] += (0.360 - delta) - rear_hub_z;
        const auto qvel_m = qvel_mut(fixture);
        std::ranges::fill(qvel_m, 0.0);
        qvel_m[0] = 3.0;
        qvel_m[1] = 1.8;
        qvel_m[static_cast<std::size_t>(ids.rear_spin_dof)] = 5.0;
        engine::forward(fixture.model.get(), fixture.data.get());

        writer.compute_into(fixture.data.get(), dt, out);
        const auto &patch = writer.snapshot_storage()[1].patches.at(0);
        const auto &diag = writer.diagnostic_storage()[1];
        const auto &f_w = engaged(patch.native_world_force_n);
        const double fn = patch.normal_load_n;
        const std::array<double, 3> ft_w = {
            f_w[0] - fn * patch.normal[0],
            f_w[1] - fn * patch.normal[1],
            f_w[2] - fn * patch.normal[2],
        };

        // Contact-point velocity v_c = J_p(p_c) * qvel.
        std::vector<mjtNum> jp(3U * static_cast<std::size_t>(fixture.model->nv),
                               0.0);
        mj_jac(fixture.model.get(), fixture.data.get(), jp.data(), nullptr,
               patch.point_m.data(), ids.rear_body);
        std::array<double, 3> vc{};
        const auto qvel = model_access::readonly_buffer(fixture.data->qvel,
                                                        fixture.model->nv);
        for (int r = 0; r < 3; ++r) {
            const auto off =
                static_cast<std::size_t>(r * fixture.model->nv);
            vc[static_cast<std::size_t>(r)] = dot_vecs(
                std::span{jp}.subspan(off,
                                      static_cast<std::size_t>(fixture.model->nv)),
                qvel);
        }
        const double work_extracted =
            -(ft_w[0] * vc[0] + ft_w[1] * vc[1] + ft_w[2] * vc[2]) * dt;
        const double rhs = (diag.shear_energy_j - prev_shear_energy) +
                           diag.branch_release_loss_j + diag.brush_loss_j;
        bt::require(diag.branch_release_loss_j >= 0.0 &&
                        diag.brush_loss_j >= 0.0,
                    "discrete brush dissipation components non-negative");
        bt::require(std::abs(work_extracted - rhs) <= 1e-11,
                    "discrete passivity identity holds to machine precision");
        prev_shear_energy = diag.shear_energy_j;
    }
}

[[gnu::noinline]] void test_camber_zero_spin_torque_and_envelope_rate() {
    auto terrain =
        make_plane_terrain(0.08, -0.05, 0.0, rider::SurfaceLabel::asphalt);
    const auto view = terrain->view();
    const auto cb = make_terrain3d_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    const BikeIds ids = resolve_ids(fixture.model.get());
    const auto cfg = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    Compliant3dTireWriter writer(fixture.model.get(), fixture.data.get(), cfg);
    std::vector<double> out(static_cast<std::size_t>(fixture.model->nv), 0.0);

    // Place cambered, yawed, steered bike with pure lateral slip + normal rate (zero longitudinal slip).
    // Terrain at (1.0, 0.3) is z = 0.08*1.0 - 0.05*0.3 = 0.065 m; z = 0.500 places cambered rear wheel at ~18 mm sag.
    set_zyx_euler_pose(fixture, {.x = 1.0,
                                 .y = 0.3,
                                 .z = 0.500,
                                 .roll_rad = 25.0 * std::numbers::pi / 180.0,
                                 .yaw_rad = 30.0 * std::numbers::pi / 180.0});
    qpos_mut(fixture)[static_cast<std::size_t>(ids.steer_qpos)] =
        15.0 * std::numbers::pi / 180.0;
    engine::forward(fixture.model.get(), fixture.data.get());

    // Set contact-point velocity of front and rear wheels to pure lateral + normal (zero along tx).
    const auto xpos = model_access::readonly_buffer(fixture.data->geom_xpos,
                                                    3 * fixture.model->ngeom);
    const auto rear_hub = std::array<double, 3>{
        xpos[3U * static_cast<std::size_t>(ids.rear_geom)],
        xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 1],
        xpos[3U * static_cast<std::size_t>(ids.rear_geom) + 2]};
    const auto a_rear =
        wheel_axis_world(fixture.model.get(), fixture.data.get(), ids.rear_body);
    const double n_len = std::sqrt(0.08 * 0.08 + 0.05 * 0.05 + 1.0);
    const std::array<double, 3> n_plane = {-0.08 / n_len, 0.05 / n_len,
                                           1.0 / n_len};
    // tx = normalize(a x n), ty = n x tx.
    const std::array<double, 3> tx_raw = {
        a_rear[1] * n_plane[2] - a_rear[2] * n_plane[1],
        a_rear[2] * n_plane[0] - a_rear[0] * n_plane[2],
        a_rear[0] * n_plane[1] - a_rear[1] * n_plane[0],
    };
    const double tx_norm = std::sqrt(
        tx_raw[0] * tx_raw[0] + tx_raw[1] * tx_raw[1] + tx_raw[2] * tx_raw[2]);
    const std::array<double, 3> tx = {
        tx_raw[0] / tx_norm, tx_raw[1] / tx_norm, tx_raw[2] / tx_norm};
    const std::array<double, 3> ty = {
        n_plane[1] * tx[2] - n_plane[2] * tx[1],
        n_plane[2] * tx[0] - n_plane[0] * tx[2],
        n_plane[0] * tx[1] - n_plane[1] * tx[0],
    };

    // Check lateral torus contact offset relative to hub: (p_c - p_hub) . ty == R_major * sin(gamma_rel).
    const double sin_gamma_rel = a_rear[0] * n_plane[0] +
                                 a_rear[1] * n_plane[1] +
                                 a_rear[2] * n_plane[2];
    const double dt = 0.00125;
    fixture.data->time = dt;
    const auto qvel_cam = qvel_mut(fixture);
    std::ranges::fill(qvel_cam, 0.0);
    qvel_cam[0] = 1.5 * ty[0] - 0.1 * n_plane[0];
    qvel_cam[1] = 1.5 * ty[1] - 0.1 * n_plane[1];
    qvel_cam[2] = 1.5 * ty[2] - 0.1 * n_plane[2];
    engine::forward(fixture.model.get(), fixture.data.get());

    writer.compute_into(fixture.data.get(), dt, out);
    const auto &rear_snap = writer.snapshot_storage()[1];
    bt::require(rear_snap.geometric_contact && !rear_snap.patches.empty(),
                "cambered rear wheel in contact on sloped plane");
    const auto &rp = rear_snap.patches.at(0);
    const std::array<double, 3> r_c = {
        rp.point_m[0] - rear_hub[0],
        rp.point_m[1] - rear_hub[1],
        rp.point_m[2] - rear_hub[2],
    };
    const double yc = r_c[0] * ty[0] + r_c[1] * ty[1] + r_c[2] * ty[2];
    bt::require(
        std::abs(yc - cfg.rear.major_radius_m() * sin_gamma_rel) <= 1e-6,
        "torus contact shifts laterally by R_major * sin(gamma_rel)");
    const auto &rf_w = engaged(rp.native_world_force_n);
    bt::require(rp.normal_load_n > 100.0 &&
                    std::abs(rf_w[0] * ty[0] + rf_w[1] * ty[1] +
                             rf_w[2] * ty[2]) > 5.0,
                "cambered wheel develops substantial normal and lateral force");
    bt::require(std::abs(out[static_cast<std::size_t>(ids.rear_spin_dof)]) <=
                    1e-12,
                "pure normal+lateral load on cambered wheel produces zero "
                "spin torque");

    // Verify Envelope Theorem: delta_dot from jac matches FD of delta(q) at omega_spin = 50 rad/s.
    writer.reset();
    set_zyx_euler_pose(fixture, {.x = 1.0,
                                 .y = 0.3,
                                 .z = 0.500,
                                 .roll_rad = 20.0 * std::numbers::pi / 180.0,
                                 .pitch_rad = 0.05,
                                 .yaw_rad = 12.0 * std::numbers::pi / 180.0});
    const auto qvel_env = qvel_mut(fixture);
    std::ranges::fill(qvel_env, 0.0);
    qvel_env[0] = 4.0;
    qvel_env[1] = 0.7;
    qvel_env[2] = -0.25;
    qvel_env[3] = 0.3;
    qvel_env[static_cast<std::size_t>(ids.rear_spin_dof)] = 50.0;
    engine::forward(fixture.model.get(), fixture.data.get());
    fixture.data->time = 2.0 * dt;
    writer.compute_into(fixture.data.get(), dt, out);
    const double ddot_jac = -writer.diagnostic_storage()[1].normal_speed_mps;

    const double h = 1e-6;
    std::vector<double> qpos_base(
        static_cast<std::size_t>(fixture.model->nq), 0.0);
    std::ranges::copy(
        model_access::readonly_buffer(fixture.data->qpos, fixture.model->nq),
        qpos_base.begin());
    const auto delta_at_offset = [&](double eps) -> double {
        std::ranges::copy(qpos_base, qpos_mut(fixture).begin());
        engine::integrate_pos(fixture.model.get(), fixture.data->qpos,
                              fixture.data->qvel, eps);
        engine::forward(fixture.model.get(), fixture.data.get());
        writer.reset();
        fixture.data->time += dt;
        writer.compute_into(fixture.data.get(), dt, out);
        return writer.diagnostic_storage()[1].penetration_m;
    };
    const double d_p = delta_at_offset(+h);
    const double d_m = delta_at_offset(-h);
    const double ddot_fd = (d_p - d_m) / (2.0 * h);
    bt::require(std::abs(ddot_jac - ddot_fd) <= 1e-6,
                "Envelope Theorem delta_dot matches FD at omega=50 rad/s");
}

[[gnu::noinline]] void test_3d_bounce_cornering_braking_energy_closure() {
    auto terrain =
        make_plane_terrain(0.0, 0.0, 0.0, rider::SurfaceLabel::hardpack);
    const auto view = terrain->view();
    const auto cb = make_terrain3d_callbacks(view);

    bt::ModelFixture fixture(kSpatialBikeXml);
    const BikeIds ids = resolve_ids(fixture.model.get());
    const auto cfg = make_compliant_3d_tire_config(
        bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar, cb);
    Compliant3dTireWriter writer(fixture.model.get(), fixture.data.get(), cfg);

    const std::size_t nv_u = static_cast<std::size_t>(fixture.model->nv);
    std::vector<double> qfrc_tire(nv_u, 0.0);
    std::vector<double> v_start(nv_u, 0.0);
    std::vector<double> v_mid(nv_u, 0.0);
    std::vector<double> mul_buf(nv_u, 0.0);

    // 8 deg roll lean, 6 deg yaw, ~15 mm drop above static sag, 6.0 m/s forward, 15% braking slip.
    const double roll = 8.0 * std::numbers::pi / 180.0;
    const double yaw = 6.0 * std::numbers::pi / 180.0;
    set_zyx_euler_pose(fixture, {.z = 0.515, .roll_rad = roll, .yaw_rad = yaw});
    const auto qvel_init = qvel_mut(fixture);
    std::ranges::fill(qvel_init, 0.0);
    const double v0 = 6.0;
    qvel_init[0] = v0 * std::cos(yaw);
    qvel_init[1] = v0 * std::sin(yaw);
    qvel_init[static_cast<std::size_t>(ids.front_spin_dof)] =
        0.85 * v0 / cfg.front.outer_radius_m;
    qvel_init[static_cast<std::size_t>(ids.rear_spin_dof)] =
        0.85 * v0 / cfg.rear.outer_radius_m;
    engine::forward(fixture.model.get(), fixture.data.get());

    const double e_initial =
        compute_total_mechanical_energy(fixture.model.get(), fixture.data.get(),
                                        mul_buf) +
        writer.stored_energy(fixture.data.get());
    const double scale = std::max(std::abs(e_initial), 1.0);

    const double dt = 0.00125;
    double loss_total = 0.0;
    double max_rel_residual = 0.0;
    for (int step = 0; step < 1600; ++step) {
        const CoreStatus status =
            writer.try_compute_into(fixture.data.get(), dt, qfrc_tire);
        bt::require(status == CoreStatus::ok,
                    "transient step try_compute_into ok");
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

    bt::require(loss_total > 1.0,
                "bounce+cornering+braking transient dissipates real energy");
    bt::require(max_rel_residual <= 0.005,
                "2.0 s 3D energy ledger closes within 0.5%");
}

} // namespace

// cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
int main() {
    return bt::run([] {
        test_presets_config();
        test_validation_probe_and_state();
        test_radial_c1_liftoff_potential_and_hysteresis();
        test_cornering_and_longitudinal_stiffness();
        test_stribeck_saturation_and_discrete_passivity();
        test_camber_zero_spin_torque_and_envelope_rate();
        test_3d_bounce_cornering_braking_energy_closure();
    });
}
