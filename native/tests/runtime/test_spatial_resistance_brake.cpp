// test_spatial_resistance_brake.cpp — 3D ResistanceWriter and unified
// StaticBrake ceiling contract tests on a 3D freejoint bike model.
#include "support/model_fixture.hpp"
#include "runtime/static_brake.hpp"
#include "writers/resistance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>

namespace {

constexpr std::string_view kSpatialBikeXml = R"(<mujoco>
  <compiler angle="degree" coordinate="local" inertiafromgeom="auto"/>
  <worldbody>
    <body name="frame" pos="0 0 1">
      <freejoint name="root_free"/>
      <geom type="box" size=".4 .05 .1" mass="12"/>
      <body name="fork" pos=".55 0 0">
        <joint name="steer_joint" type="hinge" axis="-0.3 0 0.9539392014169457"
               damping="0" stiffness="0" limited="true" range="-60 60"/>
        <geom type="capsule" fromto="0 0 0 .1 0 -.4" size=".03" mass="2"/>
        <body name="front_wheel" pos=".1 0 -.45">
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <geom name="geom_front_contact" type="sphere" size=".379" mass="2.5"/>
        </body>
      </body>
      <body name="rear_wheel" pos="-.55 0 -.45">
        <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom name="geom_rear_contact" type="sphere" size=".360" mass="3.0"/>
      </body>
    </body>
  </worldbody>
</mujoco>)";

struct RollYaw {
    double roll_rad = 0.0;
    double yaw_rad = 0.0;
};

struct WheelPoseState {
    int front_qpos = 0;
    int rear_qpos = 0;
    int front_dof = 0;
    int rear_dof = 0;
    double theta_f = 0.0;
    double theta_r = 0.0;
    double omega_f = 0.0;
    double omega_r = 0.0;
};

int body_id(const mjModel *model, const char *name) {
    const int id = mj_name2id(model, mjOBJ_BODY, name);
    model_access::require_id(id, model->nbody, name);
    return id;
}

int qpos_addr(const mjModel *model, const char *joint_name) {
    const int id = mj_name2id(model, mjOBJ_JOINT, joint_name);
    model_access::require_id(id, model->njnt, joint_name);
    const auto addrs = model_access::readonly_buffer(
        model->jnt_qposadr, model->njnt, "jnt_qposadr");
    const int adr = addrs[static_cast<std::size_t>(id)];
    model_access::require_id(adr, model->nq, joint_name);
    return adr;
}

std::array<double, 4> quat_from_roll_yaw(const RollYaw &angles) {
    const double cr = std::cos(0.5 * angles.roll_rad);
    const double sr = std::sin(0.5 * angles.roll_rad);
    const double cy = std::cos(0.5 * angles.yaw_rad);
    const double sy = std::sin(0.5 * angles.yaw_rad);
    // q_z(yaw) * q_x(roll) in MuJoCo (w, x, y, z) convention.
    return {cy * cr, cy * sr, sy * sr, sy * cr};
}

double dot(std::span<const double> a, std::span<const double> b) {
    bt::require(a.size() == b.size(), "dot vector lengths match");
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) sum += a[i] * b[i];
    return sum;
}

std::array<double, 3> cross3(const std::array<double, 3> &a,
                             const std::array<double, 3> &b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

nativecfg::ResistanceConfig make_resistance_config(bool allow_spatial) {
    return {
        .crr = 0.015,
        .rolling_taper_rad_s = 0.2,
        .rho_kg_m3 = 1.225,
        .cda_m2 = 0.45,
        .wind_world_mps = {0.0, 0.0, 0.0},
        .point_body_m = {0.0, 0.0, 0.0},
        .frame_body = "frame",
        .front_wheel_body = "front_wheel",
        .rear_wheel_body = "rear_wheel",
        .allow_spatial = allow_spatial,
    };
}

void test_resistance_planar_rejects_spatial_vectors_and_lateral_vy() {
    bt::ModelFixture fixture(kSpatialBikeXml);
    engine::forward(fixture.model.get(), fixture.data.get());

    auto bad_wind = make_resistance_config(false);
    bad_wind.wind_world_mps = {0.0, 1.0, 0.0};
    bt::invalid([&] { ResistanceWriter const rejected(fixture.model.get(), bad_wind); });

    auto bad_point = make_resistance_config(false);
    bad_point.point_body_m = {0.0, -0.1, 0.0};
    bt::invalid([&] { ResistanceWriter const rejected(fixture.model.get(), bad_point); });

    const ResistanceWriter planar(fixture.model.get(), make_resistance_config(false));
    bt::require(!planar.allow_spatial(), "planar config keeps allow_spatial false");

    const auto qvel = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv, "qvel");
    std::ranges::fill(qvel, 0.0);
    qvel[0] = 5.0;
    qvel[1] = 2.0; // lateral world velocity vy != 0
    engine::forward(fixture.model.get(), fixture.data.get());

    const std::array<double, 1> zero_load{0.0};
    const std::array<bool, 1> working{true};
    const TireSideInput idle{
        .patch_loads = zero_load, .patch_working = working, .effective_radius_m = 0.36};

    bt::invalid([&] { (void)planar.components(fixture.data.get(), idle, idle); });
    std::array<ForceComponentView, ResistanceWriter::kComponentCount> views{};
    bt::require(planar.try_components_into(fixture.data.get(), idle, idle, views) ==
                    CoreStatus::invalid_input,
                "planar try_components_into rejects nonzero lateral vy");
}

void test_resistance_spatial_rejects_nonfinite_inputs() {
    bt::ModelFixture fixture(kSpatialBikeXml);
    engine::forward(fixture.model.get(), fixture.data.get());

    auto nonfinite_wind = make_resistance_config(true);
    nonfinite_wind.wind_world_mps[1] = std::numeric_limits<double>::quiet_NaN();
    bt::invalid([&] { ResistanceWriter const rejected(fixture.model.get(), nonfinite_wind); });

    auto nonfinite_point = make_resistance_config(true);
    nonfinite_point.point_body_m[1] = std::numeric_limits<double>::infinity();
    bt::invalid([&] { ResistanceWriter const rejected(fixture.model.get(), nonfinite_point); });

    auto negative_cda = make_resistance_config(true);
    negative_cda.cda_m2 = -0.1;
    bt::invalid([&] { ResistanceWriter const rejected(fixture.model.get(), negative_cda); });
}

void test_resistance_spatial_3d_drag_at_yaw_60_deg() {
    bt::ModelFixture fixture(kSpatialBikeXml);
    auto cfg = make_resistance_config(true);
    cfg.crr = 0.0;
    cfg.point_body_m = {0.15, 0.05, 0.25};
    const ResistanceWriter writer(fixture.model.get(), cfg);
    bt::require(writer.allow_spatial(), "spatial config sets allow_spatial true");

    const double yaw_rad = std::numbers::pi / 3.0; // 60 deg
    const auto quat = quat_from_roll_yaw({.roll_rad = 0.0, .yaw_rad = yaw_rad});
    const auto qpos = model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq, "qpos");
    std::ranges::fill(qpos, 0.0);
    qpos[2] = 1.0;
    qpos[3] = quat[0];
    qpos[4] = quat[1];
    qpos[5] = quat[2];
    qpos[6] = quat[3];

    const std::array<double, 3> vel_world{8.0, -3.5, 1.2};
    const auto qvel = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv, "qvel");
    std::ranges::fill(qvel, 0.0);
    qvel[0] = vel_world[0];
    qvel[1] = vel_world[1];
    qvel[2] = vel_world[2];
    engine::forward(fixture.model.get(), fixture.data.get());

    const std::array<double, 1> zero_load{0.0};
    const std::array<bool, 1> working{true};
    const TireSideInput idle{
        .patch_loads = zero_load, .patch_working = working, .effective_radius_m = 0.36};

    const auto comps = writer.components(fixture.data.get(), idle, idle);
    std::array<ForceComponentView, ResistanceWriter::kComponentCount> views{};
    bt::require(writer.try_components_into(fixture.data.get(), idle, idle, views) ==
                    CoreStatus::ok,
                "spatial try_components_into succeeds on 3D oblique velocity");
    bt::require(views[1].kind == ForceKind::resistance, "aero kind is resistance");
    bt::require(std::ranges::equal(comps[1].second, views[1].values),
                "components and try_components_into match bitwise");

    const double speed = std::sqrt(vel_world[0] * vel_world[0] +
                                   vel_world[1] * vel_world[1] +
                                   vel_world[2] * vel_world[2]);
    const double factor = -0.5 * cfg.rho_kg_m3 * cfg.cda_m2 * speed;
    const std::array<double, 3> expected_force{
        factor * vel_world[0], factor * vel_world[1], factor * vel_world[2]};

    const auto &aero = comps[1].second;
    for (std::size_t i = 0; i < 3; ++i) {
        bt::require(std::abs(aero[i] - expected_force[i]) <= 1e-12,
                    "3D aerodynamic translational force matches -0.5*rho*CdA*||v||*v");
    }

    // With qvel[3..5] = 0, J_p^T * force induces body-frame torque point_body_m x (R^T * force).
    const int frame_id = body_id(fixture.model.get(), "frame");
    const auto xmat = model_access::readonly_buffer(
        fixture.data->xmat, 9 * fixture.model->nbody, "xmat");
    const std::size_t base = 9U * static_cast<std::size_t>(frame_id);
    const std::array<double, 3> force_body{
        xmat[base + 0] * expected_force[0] + xmat[base + 3] * expected_force[1] +
            xmat[base + 6] * expected_force[2],
        xmat[base + 1] * expected_force[0] + xmat[base + 4] * expected_force[1] +
            xmat[base + 7] * expected_force[2],
        xmat[base + 2] * expected_force[0] + xmat[base + 5] * expected_force[1] +
            xmat[base + 8] * expected_force[2]};
    const auto expected_torque_body = cross3(cfg.point_body_m, force_body);
    for (std::size_t i = 0; i < 3; ++i) {
        bt::require(std::abs(aero[3 + i] - expected_torque_body[i]) <= 1e-12,
                    "3D aerodynamic moment matches point_body_m x R^T F");
    }

    const double power = dot(aero, qvel);
    const double expected_power = expected_force[0] * vel_world[0] +
                                  expected_force[1] * vel_world[1] +
                                  expected_force[2] * vel_world[2];
    bt::require(power < 0.0, "3D aerodynamic drag power is strictly dissipative");
    bt::require(std::abs(power - expected_power) <= 1e-11,
                "3D aerodynamic generalized power equals F . v");

    // Verify 3D lateral wind also works when allow_spatial is enabled.
    auto wind_cfg = cfg;
    wind_cfg.wind_world_mps = {1.0, -3.5, 0.2};
    const ResistanceWriter wind_writer(fixture.model.get(), wind_cfg);
    bt::require(wind_writer.try_components_into(fixture.data.get(), idle, idle, views) ==
                    CoreStatus::ok,
                "spatial resistance allows nonzero 3D wind_world_mps[1]");
    const std::array<double, 3> rel{
        vel_world[0] - wind_cfg.wind_world_mps[0],
        vel_world[1] - wind_cfg.wind_world_mps[1],
        vel_world[2] - wind_cfg.wind_world_mps[2]};
    const double rel_norm = std::sqrt(rel[0] * rel[0] + rel[1] * rel[1] + rel[2] * rel[2]);
    const double wind_factor = -0.5 * wind_cfg.rho_kg_m3 * wind_cfg.cda_m2 * rel_norm;
    for (std::size_t i = 0; i < 3; ++i) {
        bt::require(std::abs(views[1].values[i] - wind_factor * rel[i]) <= 1e-12,
                    "3D aerodynamic drag uses relative = v - wind_world_mps");
    }
}

void test_resistance_spatial_3d_rolling_at_roll_25_deg() {
    bt::ModelFixture fixture(kSpatialBikeXml);
    auto cfg = make_resistance_config(true);
    cfg.rho_kg_m3 = 0.0;
    const ResistanceWriter writer(fixture.model.get(), cfg);

    const double roll_rad = 25.0 * (std::numbers::pi / 180.0);
    const double yaw_rad = std::numbers::pi / 6.0;
    const auto quat = quat_from_roll_yaw({.roll_rad = roll_rad, .yaw_rad = yaw_rad});
    const auto qpos = model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq, "qpos");
    std::ranges::fill(qpos, 0.0);
    qpos[2] = 1.0;
    qpos[3] = quat[0];
    qpos[4] = quat[1];
    qpos[5] = quat[2];
    qpos[6] = quat[3];
    qpos[static_cast<std::size_t>(qpos_addr(fixture.model.get(), "steer_joint"))] =
        15.0 * (std::numbers::pi / 180.0);

    const int front_dof = fixture.dof("front_wheel_spin");
    const int rear_dof = fixture.dof("rear_wheel_spin");
    const double omega_f = 12.0;
    const double omega_r = -8.0;
    const auto qvel = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv, "qvel");
    std::ranges::fill(qvel, 0.0);
    qvel[0] = 4.0;
    qvel[1] = 1.5;
    qvel[3] = 0.35; // nonzero roll rate to test 3D wheel axis projection
    qvel[static_cast<std::size_t>(front_dof)] = omega_f;
    qvel[static_cast<std::size_t>(rear_dof)] = omega_r;
    engine::forward(fixture.model.get(), fixture.data.get());

    const std::array<double, 1> load_f{350.0};
    const std::array<double, 1> load_r{420.0};
    const std::array<bool, 1> working{true};
    const TireSideInput front_side{
        .patch_loads = load_f, .patch_working = working, .effective_radius_m = 0.379};
    const TireSideInput rear_side{
        .patch_loads = load_r, .patch_working = working, .effective_radius_m = 0.360};

    std::array<ForceComponentView, ResistanceWriter::kComponentCount> views{};
    bt::require(writer.try_components_into(fixture.data.get(), front_side, rear_side, views) ==
                    CoreStatus::ok,
                "3D rolling resistance succeeds at 25 deg roll and 15 deg steer");

    const auto &rolling = views[0].values;
    const double rolling_power = dot(rolling, qvel);
    bt::require(rolling_power < 0.0, "3D rolling resistance power is strictly dissipative");
    bt::require(rolling[static_cast<std::size_t>(front_dof)] < 0.0,
                "front rolling moment opposes positive spin");
    bt::require(rolling[static_cast<std::size_t>(rear_dof)] > 0.0,
                "rear rolling moment opposes negative spin");

    // Also isolate pure wheel spin to verify exact tanh rolling moment magnitude.
    qvel[0] = 0.0;
    qvel[1] = 0.0;
    qvel[3] = 0.0;
    engine::forward(fixture.model.get(), fixture.data.get());
    bt::require(writer.try_components_into(fixture.data.get(), front_side, rear_side, views) ==
                    CoreStatus::ok,
                "3D rolling resistance succeeds for pure wheel spin at 25 deg roll");

    const double expected_f =
        -cfg.crr * load_f[0] * front_side.effective_radius_m *
        std::tanh(omega_f / cfg.rolling_taper_rad_s);
    const double expected_r =
        -cfg.crr * load_r[0] * rear_side.effective_radius_m *
        std::tanh(omega_r / cfg.rolling_taper_rad_s);
    bt::require(std::abs(rolling[static_cast<std::size_t>(front_dof)] - expected_f) <= 1e-12,
                "front 3D rolling moment matches exact formula at 25 deg roll + 15 deg steer");
    bt::require(std::abs(rolling[static_cast<std::size_t>(rear_dof)] - expected_r) <= 1e-12,
                "rear 3D rolling moment matches exact formula at 25 deg roll");
}

void set_wheel_state(const bt::ModelFixture &fixture, const WheelPoseState &state) {
    const auto qp = model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq, "qpos");
    const auto qv = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv, "qvel");
    std::ranges::fill(qp, 0.0);
    std::ranges::fill(qv, 0.0);
    qp[2] = 1.0;
    qp[3] = 1.0;
    qp[static_cast<std::size_t>(state.front_qpos)] = state.theta_f;
    qp[static_cast<std::size_t>(state.rear_qpos)] = state.theta_r;
    qv[static_cast<std::size_t>(state.front_dof)] = state.omega_f;
    qv[static_cast<std::size_t>(state.rear_dof)] = state.omega_r;
}

void test_static_brake_unified_ceiling_and_legacy() {
    bt::ModelFixture fixture(kSpatialBikeXml);
    const int front_dof = fixture.dof("front_wheel_spin");
    const int rear_dof = fixture.dof("rear_wheel_spin");
    const int front_qpos = qpos_addr(fixture.model.get(), "front_wheel_spin");
    const int rear_qpos = qpos_addr(fixture.model.get(), "rear_wheel_spin");
    const double ceiling_nm = 200.0;
    const std::array<double, 4> demands{0.0, 0.25, 0.5, 1.0};

    // 1. unified_ceiling = true with hold.torque_nm > ceiling_nm (250 > 200) and
    //    hold.torque_nm < ceiling_nm (120 < 200) across latched standstill and slipping.
    for (const double hold_torque_nm : {250.0, 120.0}) {
        runtime::StaticBrake brake(
            front_dof, rear_dof, ceiling_nm,
            runtime::BrakeHoldSpec{
                .torque_nm = hold_torque_nm,
                .stiffness_nm_rad = 1000.0,
                .damping_nms_rad = 5.0,
                .engage_radps = 0.5,
                .release_radps = 1.0,
                .unified_ceiling = true,
            });
        bt::require(brake.hold_spec().unified_ceiling,
                    "BrakeHoldSpec::unified_ceiling propagates");

        const double effective_hold_max = std::min(hold_torque_nm, ceiling_nm);
        for (const double u : demands) {
            // Latched at standstill: test both unsaturated windup (0.04 rad -> 40 N*m)
            // and saturated windup (0.5 rad -> 500 N*m raw).
            for (const double windup_rad : {0.04, 0.5}) {
                brake.reset();
                set_wheel_state(fixture, {.front_qpos = front_qpos,
                                          .rear_qpos = rear_qpos,
                                          .front_dof = front_dof,
                                          .rear_dof = rear_dof});
                (void)brake.hold_wheel_torques(fixture.model.get(), fixture.data.get(),
                                               1.0, 1.0, true);

                set_wheel_state(fixture, {.front_qpos = front_qpos,
                                          .rear_qpos = rear_qpos,
                                          .front_dof = front_dof,
                                          .rear_dof = rear_dof,
                                          .theta_f = windup_rad,
                                          .theta_r = -windup_rad});
                const auto saved_state = brake.state();
                const auto hold_torques = brake.hold_wheel_torques(
                    fixture.model.get(), fixture.data.get(), u, u, true);
                brake.restore(saved_state);
                const auto qfrc = brake.hold_qfrc(
                    fixture.model.get(), fixture.data.get(), u, u, true);

                bt::require(qfrc[static_cast<std::size_t>(front_dof)] == hold_torques[0],
                            "hold_qfrc matches hold_wheel_torques[0] bitwise");
                bt::require(qfrc[static_cast<std::size_t>(rear_dof)] == hold_torques[1],
                            "hold_qfrc matches hold_wheel_torques[1] bitwise");

                (void)brake.apply_remaining_budget(
                    fixture.model.get(), u, u, hold_torques[0], hold_torques[1]);
                const auto friction = model_access::readonly_buffer(
                    fixture.model->dof_frictionloss, fixture.model->nv, "dof_frictionloss");
                const double fl_f = friction[static_cast<std::size_t>(front_dof)];
                const double fl_r = friction[static_cast<std::size_t>(rear_dof)];

                bt::require(std::abs(hold_torques[0]) <= effective_hold_max * u + 1e-12,
                            "front unified hold torque respects min(hold, ceiling)*u");
                bt::require(std::abs(hold_torques[1]) <= effective_hold_max * u + 1e-12,
                            "rear unified hold torque respects min(hold, ceiling)*u");
                bt::require(std::abs(hold_torques[0]) + fl_f <= ceiling_nm * u + 1e-12,
                            "front |tau_hold| + dof_frictionloss <= ceiling_nm * u");
                bt::require(std::abs(hold_torques[1]) + fl_r <= ceiling_nm * u + 1e-12,
                            "rear |tau_hold| + dof_frictionloss <= ceiling_nm * u");

                // Unchanged repeat call returns false; span overload agrees.
                bt::require(!brake.apply_remaining_budget(
                                fixture.model.get(), u, u, qfrc),
                            "unchanged apply_remaining_budget span call returns false");
            }

            // Slipping / unlatched above release speed (omega = 2.0 > 1.0).
            set_wheel_state(fixture, {.front_qpos = front_qpos,
                                      .rear_qpos = rear_qpos,
                                      .front_dof = front_dof,
                                      .rear_dof = rear_dof,
                                      .theta_f = 0.2,
                                      .theta_r = 0.2,
                                      .omega_f = 2.0,
                                      .omega_r = -2.0});
            const auto unlatched = brake.hold_wheel_torques(
                fixture.model.get(), fixture.data.get(), u, u, true);
            bt::require(unlatched[0] == 0.0 && unlatched[1] == 0.0,
                        "slipping wheel unlatches hold torque to zero");
            (void)brake.apply_remaining_budget(
                fixture.model.get(), u, u, unlatched[0], unlatched[1]);
            const auto friction = model_access::readonly_buffer(
                fixture.model->dof_frictionloss, fixture.model->nv, "dof_frictionloss");
            bt::require(std::abs(unlatched[0]) +
                                friction[static_cast<std::size_t>(front_dof)] <=
                            ceiling_nm * u + 1e-12,
                        "front unlatched frictionloss stays within ceiling_nm * u");
            bt::require(std::abs(friction[static_cast<std::size_t>(front_dof)] -
                                 ceiling_nm * u) <= 1e-12,
                        "front unlatched gets full ceiling_nm * u frictionloss");
            bt::require(std::abs(friction[static_cast<std::size_t>(rear_dof)] -
                                 ceiling_nm * u) <= 1e-12,
                        "rear unlatched gets full ceiling_nm * u frictionloss");
        }
    }

    // 2. Reseat saturation threshold uses min(hold.torque_nm, ceiling_nm) * u.
    {
        runtime::StaticBrake brake(
            front_dof, rear_dof, ceiling_nm,
            runtime::BrakeHoldSpec{
                .torque_nm = 250.0,
                .stiffness_nm_rad = 1000.0,
                .damping_nms_rad = 5.0,
                .engage_radps = 0.5,
                .release_radps = 1.0,
                .unified_ceiling = true,
            });
        set_wheel_state(fixture, {.front_qpos = front_qpos,
                                  .rear_qpos = rear_qpos,
                                  .front_dof = front_dof,
                                  .rear_dof = rear_dof});
        (void)brake.hold_wheel_torques(fixture.model.get(), fixture.data.get(), 0.5, 0.5, true);
        // Spring demand 110 N*m exceeds unified cap 100 N*m (200 * 0.5) even though < 250 * 0.5.
        set_wheel_state(fixture, {.front_qpos = front_qpos,
                                  .rear_qpos = rear_qpos,
                                  .front_dof = front_dof,
                                  .rear_dof = rear_dof,
                                  .theta_f = 0.11,
                                  .theta_r = 0.11});
        const auto reseated = brake.hold_wheel_torques(
            fixture.model.get(), fixture.data.get(), 0.5, 0.5, true, true);
        bt::require(reseated[0] == 0.0 && reseated[1] == 0.0,
                    "unified_ceiling reseats at min(hold.torque_nm, ceiling_nm)*u");
    }

    // 3. apply_remaining_budget validation rejects non-finite inputs cleanly without writing.
    {
        runtime::StaticBrake const brake(
            front_dof, rear_dof, ceiling_nm,
            runtime::BrakeHoldSpec{.torque_nm = 200.0, .unified_ceiling = true});
        bt::require(brake.apply_remaining_budget(fixture.model.get(), 1.0, 1.0, 50.0, 75.0),
                    "initial apply_remaining_budget change reports true");
        bt::invalid([&] {
            (void)brake.apply_remaining_budget(
                fixture.model.get(), 1.0, 1.0, std::numeric_limits<double>::quiet_NaN(), 0.0);
        });
        bt::invalid([&] {
            (void)brake.apply_remaining_budget(
                fixture.model.get(), 1.0, std::numeric_limits<double>::infinity(), 0.0, 0.0);
        });
        const auto friction = model_access::readonly_buffer(
            fixture.model->dof_frictionloss, fixture.model->nv, "dof_frictionloss");
        bt::require(friction[static_cast<std::size_t>(front_dof)] == 150.0 &&
                        friction[static_cast<std::size_t>(rear_dof)] == 125.0,
                    "invalid apply_remaining_budget leaves prior frictionloss untouched");
    }

    // 4. unified_ceiling = false preserves legacy apply + hold behavior identically.
    {
        runtime::StaticBrake legacy(
            front_dof, rear_dof, ceiling_nm,
            runtime::BrakeHoldSpec{
                .torque_nm = 200.0,
                .stiffness_nm_rad = 1000.0,
                .damping_nms_rad = 5.0,
                .engage_radps = 0.5,
                .release_radps = 1.0,
                .unified_ceiling = false,
            });
        for (const double u : demands) {
            legacy.reset();
            set_wheel_state(fixture, {.front_qpos = front_qpos,
                                      .rear_qpos = rear_qpos,
                                      .front_dof = front_dof,
                                      .rear_dof = rear_dof});
            (void)legacy.hold_qfrc(fixture.model.get(), fixture.data.get(), u, u, true);
            set_wheel_state(fixture, {.front_qpos = front_qpos,
                                      .rear_qpos = rear_qpos,
                                      .front_dof = front_dof,
                                      .rear_dof = rear_dof,
                                      .theta_f = 0.5,
                                      .theta_r = 0.5});
            const auto hold = legacy.hold_qfrc(
                fixture.model.get(), fixture.data.get(), u, u, true);
            (void)legacy.apply(fixture.model.get(), u, u);
            const auto friction = model_access::readonly_buffer(
                fixture.model->dof_frictionloss, fixture.model->nv, "dof_frictionloss");
            bt::require(friction[static_cast<std::size_t>(front_dof)] == ceiling_nm * u,
                        "legacy apply sets full ceiling_nm * u on front");
            bt::require(friction[static_cast<std::size_t>(rear_dof)] == ceiling_nm * u,
                        "legacy apply sets full ceiling_nm * u on rear");
            bt::require(std::abs(hold[static_cast<std::size_t>(front_dof)]) == 200.0 * u,
                        "legacy hold keeps independent 200 * u cap");
        }
    }
}

} // namespace

// cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
int main() {
    return bt::run([] {
        test_resistance_planar_rejects_spatial_vectors_and_lateral_vy();
        test_resistance_spatial_rejects_nonfinite_inputs();
        test_resistance_spatial_3d_drag_at_yaw_60_deg();
        test_resistance_spatial_3d_rolling_at_roll_25_deg();
        test_static_brake_unified_ceiling_and_legacy();
    });
}
