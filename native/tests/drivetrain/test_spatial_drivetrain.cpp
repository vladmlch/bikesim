#include "support/fixture_test.hpp"
#include "diag.hpp"
#include "drivetrain/chain.hpp"
#include "drivetrain/transmission.hpp"
#include "engine_call.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model_access.hpp"
#include "writers/drivetrain.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace {

constexpr double kPi = std::numbers::pi;

double deg_to_rad(double deg) noexcept {
    return deg * (kPi / 180.0);
}

std::array<double, 4> quat_from_zyx_deg(double roll_deg, double pitch_deg,
                                        double yaw_deg) noexcept {
    const double r = 0.5 * deg_to_rad(roll_deg);
    const double p = 0.5 * deg_to_rad(pitch_deg);
    const double y = 0.5 * deg_to_rad(yaw_deg);
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);
    return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
}

std::string build_spatial_xml(bool motor_clutch, bool crank_y_axis = true) {
    const char *driver = motor_clutch ? "drive_shaft_spin" : "crank_spin";
    const char *crank_axis = crank_y_axis ? "0 1 0" : "1 0 0";
    std::string xml = R"(
<mujoco model="spatial_drivetrain_test">
  <compiler angle="degree" coordinate="local" inertiafromgeom="auto" autolimits="true"/>
  <option timestep="0.00125" gravity="0 0 0"/>
  <worldbody>
    <body name="frame" pos="0 0 1">
      <freejoint name="root_free"/>
      <geom name="geom_frame" type="box" size="0.4 0.1 0.2" mass="12.0" contype="0" conaffinity="0"/>
      <body name="crank" pos="0 0 0">
        <joint name="crank_spin" type="hinge" axis=")";
    xml += crank_axis;
    xml += R"("/>
        <geom name="geom_crank" type="cylinder" fromto="0 -0.075 0 0 0.075 0" size="0.02" mass="0.5" contype="0" conaffinity="0"/>
        <body name="pedal_front" pos="0.175 -0.1 0">
          <joint name="pedal_front_spin" type="hinge" axis="0 1 0"/>
          <geom name="geom_pedal_front" type="box" size="0.04 0.04 0.01" mass="0.2" contype="0" conaffinity="0"/>
        </body>
        <body name="pedal_rear" pos="-0.175 0.1 0">
          <joint name="pedal_rear_spin" type="hinge" axis="0 1 0"/>
          <geom name="geom_pedal_rear" type="box" size="0.04 0.04 0.01" mass="0.2" contype="0" conaffinity="0"/>
        </body>
      </body>
      <body name="rear_wheel" pos="-0.5 0 0">
        <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom name="geom_rear_wheel" type="cylinder" fromto="0 -0.03 0 0 0.03 0" size="0.35" mass="1.8" contype="0" conaffinity="0"/>
      </body>
      <body name="front_wheel" pos="0.7 0 0">
        <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
        <geom name="geom_front_wheel" type="cylinder" fromto="0 -0.03 0 0 0.03 0" size="0.35" mass="1.6" contype="0" conaffinity="0"/>
      </body>
)";
    if (motor_clutch) {
        xml += R"(
      <body name="drive_shaft" pos="0 0 0">
        <joint name="drive_shaft_spin" type="hinge" axis="0 1 0"/>
        <geom name="geom_drive_shaft" type="cylinder" fromto="0 -0.02 0 0 0.02 0" size="0.05" mass="0.4" contype="0" conaffinity="0"/>
      </body>
)";
    }
    xml += R"(
    </body>
  </worldbody>
  <tendon>
    <fixed name="ideal_mid_drive_freehub" limited="true" range="-1e12 0" margin="0" solreflimit="0.0025 1">
      <joint joint=")";
    xml += driver;
    xml += R"(" coef="0.6666666666666666"/>
      <joint joint="rear_wheel_spin" coef="-1"/>
    </fixed>
)";
    if (motor_clutch) {
        xml += R"(
    <fixed name="crank_clutch" limited="true" range="-1e12 0" margin="0" solreflimit="0.0025 1">
      <joint joint="crank_spin" coef="1"/>
      <joint joint="drive_shaft_spin" coef="-1"/>
    </fixed>
)";
    }
    xml += R"(
  </tendon>
  <actuator>
    <motor name="mid_drive" joint=")";
    xml += driver;
    xml += R"(" gear="1" ctrllimited="true" ctrlrange="0 85"/>
    <motor name="human_crank" joint="crank_spin" gear="1"/>
  </actuator>
</mujoco>
)";
    return xml;
}

drivetrain::DriveConfig make_drive_config(bool motor_clutch,
                                          std::string_view transmission_model) {
    drivetrain::DriveConfig c{};
    c.drive_mode = "crank_effort";
    c.transmission_model = std::string(transmission_model);
    c.human_torque_nm = 0.0;
    c.torque_ripple = 0.0;
    c.crank_phase_rad = 0.0;
    c.rotor_inertia_kgm2 = 0.2;
    c.motor_clutch = motor_clutch;
    c.policies.gearing = {.front_teeth = 34, .rear_teeth = 51, .chain_pitch_m = 0.0127};
    c.policies.pedaling = {
        .enabled = true,
        .coast_above_rpm = 120.0,
        .resume_below_rpm = 105.0,
        .stop_time_s = 0.35,
        .coast_cadence_tau_s = 0.05,
        .mash_cadence_rpm = 0.0,
        .mash_torque_nm = 0.0,
        .effort_slew_nm_s = 0.0,
        .rollback_brake = false,
        .rollback_engage_mps = 0.25,
        .rollback_release_mps = 0.1,
        .rollback_demand = 0.0,
        .reposition_on_stall = false,
        .reposition_min_effort_nm = 20.0,
        .reposition_stall_window_s = 1.0,
        .reposition_stall_progress_rad = 0.5,
        .reposition_cooldown_s = 1.0,
        .reposition_back_rate_rad_s = 3.0,
        .reposition_damping_nms_rad = 12.0,
        .reposition_timeout_s = 3.0,
        .reposition_phase_tolerance_rad = 0.05,
        .reposition_noop_rad = 0.12,
    };
    c.policies.shifting = {
        .enabled = false,
        .cassette = {51, 45, 39, 33, 28, 24, 21, 18, 16, 14, 12, 10},
        .target_cadence_min_rpm = 65.0,
        .target_cadence_max_rpm = 95.0,
        .shift_cooldown_s = 0.4,
        .shift_cut_duration_s = 0.2,
        .torque_factor_larger_cog = 0.8,
        .torque_factor_smaller_cog = 0.3,
        .cadence_smoothing_tau_s = 0.05,
        .upshift_slip_limit_mps = 0.5,
        .upshift_slip_mode = "magnitude",
    };
    c.policies.assist = {
        .gain = 2.0,
        .max_torque = 85.0,
        .max_power = 600.0,
        .tau = 0.04,
        .slew = 500.0,
        .engage_torque_nm = 1.0,
        .release_hold_s = 0.3,
        .cutoff_mps = 7.0,
        .taper_width_mps = 0.5,
        .torque_curve = std::nullopt,
        .profile = std::nullopt,
        .mode = "turbo",
    };
    return c;
}

struct Pose {
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
};

// NOLINTNEXTLINE(misc-const-correctness) mutates d->qpos buffer through pointer member
void set_root_pose(const mjModel *m, mjData *d, const Pose &pose) {
    const int root_id = bike::model::required_id(*m, mjOBJ_JOINT, "root_free");
    const auto qpos_adr = static_cast<std::size_t>(
        model_access::readonly_buffer(m->jnt_qposadr, m->njnt)[static_cast<std::size_t>(root_id)]);
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq);
    const auto quat = quat_from_zyx_deg(pose.roll_deg, pose.pitch_deg, pose.yaw_deg);
    qpos[qpos_adr + 0] = 0.0;
    qpos[qpos_adr + 1] = 0.0;
    qpos[qpos_adr + 2] = 1.0;
    qpos[qpos_adr + 3] = quat[0];
    qpos[qpos_adr + 4] = quat[1];
    qpos[qpos_adr + 5] = quat[2];
    qpos[qpos_adr + 6] = quat[3];
}

struct SimSummary {
    bool crank_clutch_engaged = false;
    bool freehub_engaged = false;
    double crank_clutch_torque_nm = 0.0;
    double freehub_torque_nm = 0.0;
    double crank_qvel = 0.0;
    double shaft_qvel = 0.0;
    double wheel_qvel = 0.0;
    double wheel_qvel_delta = 0.0;
};

struct ClutchStepArgs {
    double human_torque_nm{};
    double motor_torque_nm{};
    int steps{};
};

struct FreewheelStepArgs {
    int warmup_steps{};
    int freewheel_steps{};
};

[[gnu::noinline]] SimSummary run_clutch_steps(const Pose &pose,
                                            const ClutchStepArgs &args) {
    auto model = bike::model::compile_mjcf(build_spatial_xml(/*motor_clutch=*/true));
    drivetrain::OwnedData const data(engine::make_data(model.get()));
    set_root_pose(model.get(), data.get(), pose);
    const int crank_jid = bike::model::required_id(*model, mjOBJ_JOINT, "crank_spin");
    const int shaft_jid = bike::model::required_id(*model, mjOBJ_JOINT, "drive_shaft_spin");
    const int wheel_jid = bike::model::required_id(*model, mjOBJ_JOINT, "rear_wheel_spin");
    const auto dof_adr = model_access::readonly_buffer(model->jnt_dofadr, model->njnt);
    const auto crank_dof = static_cast<std::size_t>(dof_adr[static_cast<std::size_t>(crank_jid)]);
    const auto shaft_dof = static_cast<std::size_t>(dof_adr[static_cast<std::size_t>(shaft_jid)]);
    const auto wheel_dof = static_cast<std::size_t>(dof_adr[static_cast<std::size_t>(wheel_jid)]);
    engine::forward(model.get(), data.get());

    auto writer = std::make_unique<drivetrain::DrivetrainWriter>(
        model.get(), data.get(), make_drive_config(/*motor_clutch=*/true, "ideal_mid_drive"));
    writer->reset();

    drivetrain::RideControl control;
    control.human_torque_nm = args.human_torque_nm;
    control.motor_torque_nm = args.motor_torque_nm;
    control.rider_enabled = true;
    const double dt = model->opt.timestep;

    for (int i = 0; i < args.steps; ++i) {
        engine::forward(model.get(), data.get());
        (void)writer->components(control, dt, /*speed=*/0.0, /*braking=*/false,
                                 /*active=*/true, /*advance=*/true, /*sensed=*/0.0,
                                 /*pedaling=*/std::nullopt, /*contact=*/true,
                                 /*slip=*/std::nullopt);
        engine::step(model.get(), data.get());
        (void)writer->settle();
    }

    const auto &diag = writer->diagnostics(false);
    const auto qvel = model_access::readonly_buffer(data->qvel, model->nv);
    return {
        .crank_clutch_engaged = std::get<bool>(diag.at("crank_clutch_engaged")),
        .freehub_engaged = std::get<bool>(diag.at("freehub_engaged")),
        .crank_clutch_torque_nm = std::get<double>(diag.at("crank_clutch_torque_nm")),
        .freehub_torque_nm = std::get<double>(diag.at("freehub_torque_nm")),
        .crank_qvel = qvel[crank_dof],
        .shaft_qvel = qvel[shaft_dof],
        .wheel_qvel = qvel[wheel_dof],
        .wheel_qvel_delta = qvel[wheel_dof],
    };
}

[[gnu::noinline]] SimSummary run_freewheel_steps(const Pose &pose,
                                               const FreewheelStepArgs &args) {
    auto model = bike::model::compile_mjcf(build_spatial_xml(/*motor_clutch=*/true));
    drivetrain::OwnedData const data(engine::make_data(model.get()));
    set_root_pose(model.get(), data.get(), pose);
    const int crank_jid = bike::model::required_id(*model, mjOBJ_JOINT, "crank_spin");
    const int shaft_jid = bike::model::required_id(*model, mjOBJ_JOINT, "drive_shaft_spin");
    const int wheel_jid = bike::model::required_id(*model, mjOBJ_JOINT, "rear_wheel_spin");
    const auto dof_adr = model_access::readonly_buffer(model->jnt_dofadr, model->njnt);
    const auto crank_dof = static_cast<std::size_t>(dof_adr[static_cast<std::size_t>(crank_jid)]);
    const auto shaft_dof = static_cast<std::size_t>(dof_adr[static_cast<std::size_t>(shaft_jid)]);
    const auto wheel_dof = static_cast<std::size_t>(dof_adr[static_cast<std::size_t>(wheel_jid)]);
    engine::forward(model.get(), data.get());

    auto writer = std::make_unique<drivetrain::DrivetrainWriter>(
        model.get(), data.get(), make_drive_config(/*motor_clutch=*/true, "ideal_mid_drive"));
    writer->reset();

    const double dt = model->opt.timestep;
    drivetrain::RideControl warmup_control;
    warmup_control.human_torque_nm = 25.0;
    warmup_control.motor_torque_nm = 30.0;
    warmup_control.rider_enabled = true;
    for (int i = 0; i < args.warmup_steps; ++i) {
        engine::forward(model.get(), data.get());
        (void)writer->components(warmup_control, dt, /*speed=*/0.0, /*braking=*/false,
                                 /*active=*/true, /*advance=*/true, /*sensed=*/0.0,
                                 /*pedaling=*/std::nullopt, /*contact=*/true,
                                 /*slip=*/std::nullopt);
        engine::step(model.get(), data.get());
        (void)writer->settle();
    }

    const double wheel_before = model_access::readonly_buffer(data->qvel, model->nv)[wheel_dof];
    model_access::mutable_buffer(data->qvel, model->nv)[crank_dof] = -2.0;

    drivetrain::RideControl coast_control;
    coast_control.human_torque_nm = 0.0;
    coast_control.motor_torque_nm = 30.0;
    coast_control.rider_enabled = true;
    for (int i = 0; i < args.freewheel_steps; ++i) {
        engine::forward(model.get(), data.get());
        (void)writer->components(coast_control, dt, /*speed=*/0.0, /*braking=*/false,
                                 /*active=*/true, /*advance=*/true, /*sensed=*/0.0,
                                 /*pedaling=*/std::nullopt, /*contact=*/true,
                                 /*slip=*/std::nullopt);
        engine::step(model.get(), data.get());
        (void)writer->settle();
    }

    const auto &diag = writer->diagnostics(false);
    const auto qvel = model_access::readonly_buffer(data->qvel, model->nv);
    return {
        .crank_clutch_engaged = std::get<bool>(diag.at("crank_clutch_engaged")),
        .freehub_engaged = std::get<bool>(diag.at("freehub_engaged")),
        .crank_clutch_torque_nm = std::get<double>(diag.at("crank_clutch_torque_nm")),
        .freehub_torque_nm = std::get<double>(diag.at("freehub_torque_nm")),
        .crank_qvel = qvel[crank_dof],
        .shaft_qvel = qvel[shaft_dof],
        .wheel_qvel = qvel[wheel_dof],
        .wheel_qvel_delta = qvel[wheel_dof] - wheel_before,
    };
}

[[gnu::noinline]] void test_relative_sagittal_angle_across_3d_poses() {
    for (const bool clutch : {false, true}) {
        auto model = bike::model::compile_mjcf(build_spatial_xml(clutch));
        bt::require(model->nq == model->nv + 1, "spatial model has freejoint nq == nv + 1");
        drivetrain::OwnedData const data(engine::make_data(model.get()));

        const int frame_id = bike::model::required_id(*model, mjOBJ_BODY, "frame");
        const int crank_id = bike::model::required_id(*model, mjOBJ_BODY, "crank");
        const int crank_jid = bike::model::required_id(*model, mjOBJ_JOINT, "crank_spin");
        const auto crank_qpos = static_cast<std::size_t>(
            model_access::readonly_buffer(model->jnt_qposadr, model->njnt)
                [static_cast<std::size_t>(crank_jid)]);

        constexpr std::array<Pose, 5> poses{{
            {.roll_deg = 0.0, .pitch_deg = 0.0, .yaw_deg = 0.0},
            {.roll_deg = 0.0, .pitch_deg = 0.0, .yaw_deg = 90.0},
            {.roll_deg = 0.0, .pitch_deg = 0.0, .yaw_deg = 150.0},
            {.roll_deg = 25.0, .pitch_deg = 15.0, .yaw_deg = 90.0},
            {.roll_deg = 25.0, .pitch_deg = 15.0, .yaw_deg = 150.0},
        }};
        constexpr std::array<double, 6> crank_angles{{-2.3, -0.8, 0.0, 0.7, 1.9, 2.8}};

        for (const auto &pose : poses) {
            for (const double alpha : crank_angles) {
                engine::reset_data(model.get(), data.get());
                set_root_pose(model.get(), data.get(), pose);
                model_access::mutable_buffer(data->qpos, model->nq)[crank_qpos] = alpha;
                engine::forward(model.get(), data.get());

                const double recovered = drivetrain::relative_sagittal_angle(
                    model.get(), data.get(), frame_id, crank_id);
                bt::require(std::abs(recovered - alpha) <= 1e-13,
                            "relative_sagittal_angle recovers exact crank_spin angle in 3D");

                auto writer = std::make_unique<drivetrain::DrivetrainWriter>(
                    model.get(), data.get(), make_drive_config(clutch, "ideal_mid_drive"));
                writer->reset();
                const auto snap = writer->state();
                if (!snap.angles.has_value()) {
                    bt::require(false, "DrivetrainWriter reset publishes crank angle in 3D");
                    return;
                }
                const drivetrain::DriveAngles &angles = *snap.angles;
                bt::require(angles.count == 1,
                            "DrivetrainWriter reset publishes single crank angle in 3D");
                bt::require(std::abs(angles.values[0] - alpha) <= 1e-13,
                            "DrivetrainWriter::angle recovers exact crank_spin angle in 3D");
            }
        }

        // Verify that the legacy world-frame atan2(-xmat[6], xmat[0]) breaks at yaw = 90 deg.
        engine::reset_data(model.get(), data.get());
        set_root_pose(model.get(), data.get(), {.roll_deg = 25.0, .pitch_deg = 15.0, .yaw_deg = 90.0});
        const double alpha = 0.7;
        model_access::mutable_buffer(data->qpos, model->nq)[crank_qpos] = alpha;
        engine::forward(model.get(), data.get());
        const auto xmat = model_access::readonly_buffer(data->xmat, 9 * model->nbody);
        const auto crank_rot = xmat.subspan(9 * static_cast<std::size_t>(crank_id), 9);
        const double world_naive = std::atan2(-crank_rot[6], crank_rot[0]);
        bt::require(std::abs(world_naive - alpha) > 0.1,
                    "world-frame atan2 fails at yaw = 90 deg where relative_sagittal_angle succeeds");
    }
}

[[gnu::noinline]] void test_motor_clutch_freewheel_and_engagement_3d_invariance() {
    constexpr std::array<Pose, 3> poses{{
        {.roll_deg = 0.0, .pitch_deg = 0.0, .yaw_deg = 0.0},
        {.roll_deg = 0.0, .pitch_deg = 0.0, .yaw_deg = 90.0},
        {.roll_deg = 25.0, .pitch_deg = 15.0, .yaw_deg = 150.0},
    }};

    // 1. Engagement: positive human torque drives crank -> clutch engages -> freehub drives rear wheel.
    const auto eng_ref = run_clutch_steps(
        poses[0], {.human_torque_nm = 25.0, .motor_torque_nm = 0.0, .steps = 10});
    bt::require(eng_ref.crank_clutch_engaged, "clutch engages under forward pedaling");
    bt::require(eng_ref.crank_clutch_torque_nm > 1.0, "clutch transmits positive torque");
    bt::require(eng_ref.freehub_engaged, "freehub engages under forward drive");
    bt::require(eng_ref.wheel_qvel > 0.0, "rear wheel accelerates forward");

    for (std::size_t i = 1; i < poses.size(); ++i) {
        const auto eng = run_clutch_steps(
            poses[i], {.human_torque_nm = 25.0, .motor_torque_nm = 0.0, .steps = 10});
        bt::require(eng.crank_clutch_engaged == eng_ref.crank_clutch_engaged,
                    "clutch engagement invariant under 3D orientation");
        bt::require(eng.freehub_engaged == eng_ref.freehub_engaged,
                    "freehub engagement invariant under 3D orientation");
        bt::require(std::abs(eng.crank_clutch_torque_nm - eng_ref.crank_clutch_torque_nm) <= 1e-10,
                    "clutch torque invariant under 3D orientation");
        bt::require(std::abs(eng.freehub_torque_nm - eng_ref.freehub_torque_nm) <= 1e-10,
                    "freehub torque invariant under 3D orientation");
        bt::require(std::abs(eng.crank_qvel - eng_ref.crank_qvel) <= 1e-10,
                    "crank velocity invariant under 3D orientation");
        bt::require(std::abs(eng.wheel_qvel - eng_ref.wheel_qvel) <= 1e-10,
                    "wheel velocity invariant under 3D orientation");
    }

    // 2. Freewheel: after assist run-on arms, motor drives drive_shaft forward while crank spins backward; clutch stays open.
    const auto fw_ref = run_freewheel_steps(
        poses[0], {.warmup_steps = 15, .freewheel_steps = 10});
    bt::require(!fw_ref.crank_clutch_engaged, "clutch freewheels when motor overruns crank");
    bt::require(fw_ref.crank_clutch_torque_nm <= 1e-10, "open clutch transmits zero torque");
    bt::require(fw_ref.freehub_engaged, "freehub still engages to drive rear wheel");
    bt::require(fw_ref.freehub_torque_nm > 1.0, "freehub transmits positive motor torque");
    bt::require(fw_ref.shaft_qvel > 0.0 && fw_ref.wheel_qvel_delta > 0.0,
                "motor accelerates drive shaft and rear wheel while crank freewheels");
    bt::require(fw_ref.crank_qvel < -1.9,
                "freewheeling crank is not dragged forward by motor");

    for (std::size_t i = 1; i < poses.size(); ++i) {
        const auto fw = run_freewheel_steps(
            poses[i], {.warmup_steps = 15, .freewheel_steps = 10});
        bt::require(fw.crank_clutch_engaged == fw_ref.crank_clutch_engaged,
                    "clutch freewheel state invariant under 3D orientation");
        bt::require(fw.freehub_engaged == fw_ref.freehub_engaged,
                    "freehub engagement during freewheel invariant under 3D orientation");
        bt::require(std::abs(fw.crank_clutch_torque_nm - fw_ref.crank_clutch_torque_nm) <= 1e-10,
                    "freewheel clutch torque invariant under 3D orientation");
        bt::require(std::abs(fw.freehub_torque_nm - fw_ref.freehub_torque_nm) <= 1e-10,
                    "freewheel hub torque invariant under 3D orientation");
        bt::require(std::abs(fw.crank_qvel - fw_ref.crank_qvel) <= 1e-10,
                    "freewheeling crank velocity invariant under 3D orientation");
        bt::require(std::abs(fw.shaft_qvel - fw_ref.shaft_qvel) <= 1e-10,
                    "drive shaft velocity invariant under 3D orientation");
        bt::require(std::abs(fw.wheel_qvel - fw_ref.wheel_qvel) <= 1e-10,
                    "wheel velocity during freewheel invariant under 3D orientation");
    }
}

[[gnu::noinline]] void test_spatial_fail_closed_contracts() {
    auto model = bike::model::compile_mjcf(build_spatial_xml(/*motor_clutch=*/true));
    drivetrain::OwnedData const data(engine::make_data(model.get()));
    engine::forward(model.get(), data.get());

    // 1. Geometric transmission on nq != nv still fails closed.
    bt::invalid([&] {
        (void)std::make_unique<drivetrain::DrivetrainWriter>(
            model.get(), data.get(),
            make_drive_config(/*motor_clutch=*/true, "geometric_ideal_mid_drive"));
    });
    bt::invalid([&] {
        (void)std::make_unique<drivetrain::Transmission>(
            model.get(),
            drivetrain::GearingConfig{
                .front_teeth = 34, .rear_teeth = 51, .chain_pitch_m = 0.0127},
            /*geometric=*/true, "ideal_mid_drive_freehub",
            "drive_shaft_spin", "rear_wheel_spin", "crank");
    });

    // 2. Non-{0, 1, 0} crank hinge axis is still rejected by topology::joint.
    auto bad_axis_model = bike::model::compile_mjcf(
        build_spatial_xml(/*motor_clutch=*/false, /*crank_y_axis=*/false));
    drivetrain::OwnedData const bad_axis_data(engine::make_data(bad_axis_model.get()));
    engine::forward(bad_axis_model.get(), bad_axis_data.get());
    bt::invalid([&] {
        (void)std::make_unique<drivetrain::DrivetrainWriter>(
            bad_axis_model.get(), bad_axis_data.get(),
            make_drive_config(/*motor_clutch=*/false, "ideal_mid_drive"));
    });

    // 3. relative_sagittal_angle rejects world body IDs and misaligned local +Y axes.
    const int frame_id = bike::model::required_id(*model, mjOBJ_BODY, "frame");
    const int crank_id = bike::model::required_id(*model, mjOBJ_BODY, "crank");
    bt::invalid([&] {
        (void)drivetrain::relative_sagittal_angle(model.get(), data.get(), 0, crank_id);
    });
    bt::invalid([&] {
        (void)drivetrain::relative_sagittal_angle(model.get(), data.get(), frame_id, 0);
    });
    const int bad_frame_id = bike::model::required_id(*bad_axis_model, mjOBJ_BODY, "frame");
    const int bad_crank_id = bike::model::required_id(*bad_axis_model, mjOBJ_BODY, "crank");
    const int bad_crank_jid = bike::model::required_id(*bad_axis_model, mjOBJ_JOINT, "crank_spin");
    const auto bad_qpos = static_cast<std::size_t>(
        model_access::readonly_buffer(bad_axis_model->jnt_qposadr, bad_axis_model->njnt)
            [static_cast<std::size_t>(bad_crank_jid)]);
    model_access::mutable_buffer(bad_axis_data->qpos, bad_axis_model->nq)[bad_qpos] = 0.5;
    engine::forward(bad_axis_model.get(), bad_axis_data.get());
    bt::invalid([&] {
        (void)drivetrain::relative_sagittal_angle(
            bad_axis_model.get(), bad_axis_data.get(), bad_frame_id, bad_crank_id);
    });
}

} // namespace

int main() {
    return bt::run([] {
        test_relative_sagittal_angle_across_3d_poses();
        test_motor_clutch_freewheel_and_engagement_3d_invariance();
        test_spatial_fail_closed_contracts();
    });
}
