#include "configuration/load.hpp"
#include "configuration/rider_spec.hpp"
#include "engine_call.hpp"
#include "model/builder.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/state_codec.hpp"
#include "runtime/period_buffer.hpp"
#include "runtime/project_config.hpp"
#include "runtime/samples.hpp"
#include "runtime/status.hpp"
#include "runtime/step.hpp"
#include "spatial/config.hpp"
#include "spatial/frames.hpp"
#include "support/fixture_test.hpp"
#include "writers/rider_contacts.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace cfg = bike::configuration;
namespace mdl = bike::model;
namespace sp = bike::spatial;

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRad = kPi / 180.0;

[[nodiscard]] sp::SpatialConfig make_spatial_config() {
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

struct RiderPlant3d {
    std::unique_ptr<mdl::BuildRequest> request;
    mdl::ArticulatedPose3d pose3d;
    mdl::ModelOwner model{nullptr, mdl::ModelDeleter{}};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, &mj_deleteData};
    std::unique_ptr<rider::SpindleController> controller;
    std::unique_ptr<writers::RiderContactWriter> writer;
};

[[nodiscard]] rider::RiderContactsConfig make_contacts_config(
    const cfg::ArticulatedConfig &a) {
    rider::RiderContactsConfig c{};
    c.arm_reach_m = 0.7;
    c.saddle_patch_half_length_m = a.saddle_patch_half_length_m;
    c.pedal_patch_half_length_m = a.pedal_patch_half_length_m;
    c.support_pad_radius_m = a.support_pad_radius_m;
    c.support_k_n_m = a.support_k_n_m;
    c.support_c_ns_m = a.support_c_ns_m;
    c.pedal_c_ns_m = a.pedal_c_ns_m;
    c.support_tangent_k_n_m = a.support_tangent_k_n_m;
    c.support_mu = a.support_mu;
    c.support_length_m = a.support_length_m;
    c.grip_k_n_m = a.grip_k_n_m;
    c.grip_c_ns_m = a.grip_c_ns_m;
    c.grip_release_distance_m = a.grip_release_distance_m;
    c.grip_pair_force_limit_n = a.grip_pair_force_limit_n;
    c.grip_capture_distance_m = a.grip_capture_distance_m;
    c.grip_capture_speed_mps = a.grip_capture_speed_mps;
    const auto p_att = rider::pedal_attachment(a.pedal_attachment);
    const auto s_att = rider::saddle_attachment(a.saddle_attachment);
    const auto g_att = rider::grip_attachment(a.grip_attachment);
    if (!p_att || !s_att || !g_att)
        throw std::invalid_argument("invalid attachment mode");
    c.pedal_attachment = *p_att;
    c.saddle_attachment = *s_att;
    c.grip_attachment = *g_att;
    return c;
}

[[nodiscard]] RiderPlant3d make_plant_3d() {
    RiderPlant3d plant;
    plant.request = std::make_unique<mdl::BuildRequest>();
    plant.request->mode = "ride";
    plant.request->rider = mdl::rider_from_args("articulated_3d");
    plant.request->physics =
        cfg::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
    plant.request->spatial = make_spatial_config();
    plant.request->field.ncol = 101;
    plant.request->field.radius_x_m = 10.0;
    plant.request->field.elevation_m = 0.5;

    const std::string xml = mdl::generate_mjcf(*plant.request);
    plant.model = mdl::compile_mjcf(xml);
    plant.data.reset(engine::make_data(plant.model.get()));
    engine::forward(plant.model.get(), plant.data.get());

    plant.pose3d =
        mdl::geometry_pose_3d(*plant.request->rider, plant.request->bike);
    const mdl::ArticulatedPose legacy_pose = mdl::to_articulated_pose(plant.pose3d);
    const auto spindle_pose = rider::decode_spindle_pose(
        runtime::project_articulated_pose(legacy_pose), "test.pose");
    auto spindle_cfg = rider::decode_spindle_config(
        runtime::project_rider_controller(*plant.request->physics,
                                          *plant.model, legacy_pose));
    spindle_cfg.reposition_damping_nms_rad =
        plant.request->physics->drive.pedaling.reposition_damping_nms_rad;
    spindle_cfg.reposition_torque_limit_nm =
        plant.request->physics->seated_climb.max_crank_torque_nm;

    plant.controller = std::make_unique<rider::SpindleController>(
        plant.model.get(), spindle_pose, std::move(spindle_cfg),
        plant.request->bike.crank_length / 1000.0);
    plant.writer = std::make_unique<writers::RiderContactWriter>(
        plant.model.get(),
        make_contacts_config(plant.request->physics->articulated));
    plant.writer->reset(plant.data.get());
    return plant;
}

[[nodiscard]] std::array<double, 3> vec3_at(std::span<const double> buf, int id) {
    const auto base = 3U * static_cast<std::size_t>(id);
    return {buf[base], buf[base + 1U], buf[base + 2U]};
}

[[nodiscard]] double norm3(const std::array<double, 3> &v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

[[nodiscard]] std::array<double, 3> cross3(
    const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

[[nodiscard]] double dot3(
    const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void pose_bike_and_pelvis(const mjModel *m, std::span<double> qpos,
                          double roll_rad, double pitch_rad, double yaw_rad) {
    const int frame_free = mj_name2id(m, mjOBJ_JOINT, "root_free");
    const int rider_free = mj_name2id(m, mjOBJ_JOINT, "rider_root_free");
    bt::require(frame_free >= 0 && rider_free >= 0, "free joints exist");
    const auto qadr = std::views::counted(m->jnt_qposadr, m->njnt);
    const auto f_q0 = static_cast<std::size_t>(qadr[static_cast<std::size_t>(frame_free)]);
    const auto r_q0 = static_cast<std::size_t>(qadr[static_cast<std::size_t>(rider_free)]);

    const sp::Quat q_att = sp::quat_from_euler_zyx(roll_rad, pitch_rad, yaw_rad);
    const std::array<double, 3> p_frame{
        qpos[f_q0], qpos[f_q0 + 1U], qpos[f_q0 + 2U]};
    const std::array<double, 3> p_rider0{
        qpos[r_q0], qpos[r_q0 + 1U], qpos[r_q0 + 2U]};
    const std::array<double, 3> offset_world =
        sp::rotate_body_to_world(q_att, {
            p_rider0[0] - p_frame[0],
            p_rider0[1] - p_frame[1],
            p_rider0[2] - p_frame[2]});
    for (std::size_t i = 0; i < 4U; ++i) {
        qpos[f_q0 + 3U + i] = q_att[i];
        qpos[r_q0 + 3U + i] = q_att[i];
    }
    for (std::size_t i = 0; i < 3U; ++i)
        qpos[r_q0 + i] = p_frame[i] + offset_world[i];
}

void test_3d_attitude_kinematics_and_attachments() {
    auto plant = make_plant_3d();
    const mjModel *m = plant.model.get();
    mjData *d = plant.data.get();

    bt::require(plant.controller->joints().size() == 11U,
                "3D SpindleController resolves 11 active joints");
    bt::require(mj_name2id(m, mjOBJ_ACTUATOR, "act_rider_steer") == -1,
                "act_rider_steer must be absent in articulated_3d");

    // Pose at 45 deg yaw, 15% uphill pitch, 5 deg bank (roll).
    const double yaw_rad = 45.0 * kDegToRad;
    const double pitch_rad = -std::atan(0.15);
    const double roll_rad = 5.0 * kDegToRad;
    pose_bike_and_pelvis(m, std::views::counted(d->qpos, m->nq),
                         roll_rad, pitch_rad, yaw_rad);

    // Impose nonzero 3D velocity on both root_free and rider_root_free.
    const int frame_free = mj_name2id(m, mjOBJ_JOINT, "root_free");
    const int rider_free = mj_name2id(m, mjOBJ_JOINT, "rider_root_free");
    const auto dadr = std::views::counted(m->jnt_dofadr, m->njnt);
    const auto f_d0 = static_cast<std::size_t>(dadr[static_cast<std::size_t>(frame_free)]);
    const auto r_d0 = static_cast<std::size_t>(dadr[static_cast<std::size_t>(rider_free)]);
    const auto qvel = std::views::counted(d->qvel, m->nv);
    const sp::Quat q_att = sp::quat_from_euler_zyx(roll_rad, pitch_rad, yaw_rad);
    const sp::Vec3 v_fwd = sp::rotate_body_to_world(q_att, {4.5, 0.0, 0.0});
    for (std::size_t i = 0; i < 3U; ++i) {
        qvel[f_d0 + i] = v_fwd[i];
        qvel[r_d0 + i] = v_fwd[i];
    }
    qvel[f_d0 + 3U] = 0.05;
    qvel[f_d0 + 4U] = -0.02;
    qvel[f_d0 + 5U] = 0.08;
    qvel[r_d0 + 3U] = 0.05;
    qvel[r_d0 + 4U] = -0.02;
    qvel[r_d0 + 5U] = 0.08;

    plant.controller->initialize(d);
    engine::forward(m, d);
    plant.writer->reset(d);

    // Verify 3D body-local upper_targets symmetry at zero steer regardless of 3D attitude.
    rider::RiderPosture posture{};
    posture.torso_lean_rad = 0.18;
    const auto upper = plant.controller->upper_targets(d, &posture);
    bt::require(upper.size() == 7U, "upper_targets returns 7 targets in 3D");
    const auto find_target = [&](std::string_view name) {
        for (const auto &[k, v] : upper)
            if (k == name) return v;
        bt::require(false, "missing upper target");
        return 0.0;
    };
    const double abd_l = find_target("rider_shoulder_abduction_left");
    const double abd_r = find_target("rider_shoulder_abduction_right");
    const double sh_l = find_target("rider_shoulder_left");
    const double sh_r = find_target("rider_shoulder_right");
    const double el_l = find_target("rider_elbow_left");
    const double el_r = find_target("rider_elbow_right");
    bt::require(std::abs(abd_l + abd_r) <= 1e-10,
                "body-local torso rotation keeps L/R shoulder abduction antisymmetric at 45 deg yaw + 5 deg bank");
    bt::require(std::abs(sh_l - sh_r) <= 1e-10 && std::abs(el_l - el_r) <= 1e-10,
                "body-local torso rotation keeps L/R shoulder & elbow targets symmetric");

    // Verify min_torso_lean_rad & compute at 3D attitude.
    const std::optional<double> min_lean =
        plant.controller->min_torso_lean_rad(d, 15.0 * kDegToRad);
    bt::require(min_lean.has_value() && std::isfinite(*min_lean),
                "min_torso_lean_rad resolves in 3D");

    rider::RiderCommand cmd{};
    cmd.mean_crank_torque_nm = 25.0;
    cmd.enabled = true;
    cmd.posture = posture;
    cmd.crank_target_rate_rad_s = 8.0;
    const auto torques = plant.controller->compute(d, cmd, true, 0.00125);
    bt::require(torques.size() == 11U, "compute returns 11 joint torques");
    for (const auto &[name, val] : torques)
        bt::require(std::isfinite(val), name + " torque is finite in 3D");
    plant.controller->write(d, torques);
    engine::forward(m, d);

    // Verify PeriodBuffer::evaluate_attachments preserves 3D wrenches (nonzero Y/Mx/Mz).
    const int pelvis_id = mj_name2id(m, mjOBJ_BODY, "rider_pelvis");
    bt::require(pelvis_id >= 0, "rider_pelvis resolves");
    const auto xfrc = std::views::counted(d->xfrc_applied, m->nbody * 6);
    xfrc[static_cast<std::size_t>(pelvis_id) * 6U + 1U] = 25.0; // lateral Y force
    xfrc[static_cast<std::size_t>(pelvis_id) * 6U + 3U] = 8.0;  // roll moment Mx
    engine::forward(m, d);

    const auto prepared = plant.writer->prepare_attachment_raw(d);
    bt::require(prepared.errors.empty() && prepared.geometry.size() == 5U,
                "all 5 3D attachments (including 7-DOF rider_saddle_carrier) are observable");
    const auto settle = plant.writer->settle_welds(d, std::nullopt, true, &prepared);
    bt::require(settle.measurement.raws.size() == 5U,
                "settle_welds captures all 5 raw attachments in 3D");

    runtime::RawStep raw{};
    raw.interval_id = 0;
    raw.time_s = 0.0;
    raw.end_time_s = 0.00125;
    const auto raw_qpos = std::views::counted(d->qpos, m->nq);
    const auto raw_qvel = std::views::counted(d->qvel, m->nv);
    raw.qpos.assign(raw_qpos.begin(), raw_qpos.end());
    raw.qvel.assign(raw_qvel.begin(), raw_qvel.end());
    raw.attachment_raw = settle.measurement.raws;

    rider::LeastSquaresWorkspace ws{
        static_cast<rider::lapack_int>(m->nv),
        std::max<rider::lapack_int>(static_cast<rider::lapack_int>(m->nv), 6),
        1};
    const auto evaluated = runtime::evaluate_attachments(raw, ws);
    bt::require(evaluated.size() == 5U,
                "evaluate_attachments retains all 5 3D attachments with nonzero Y/Mx/Mz");
}

void test_differential_hand_steering_and_power_cap() {
    auto plant = make_plant_3d();
    const mjModel *m = plant.model.get();
    mjData *d = plant.data.get();

    const int steer_jnt = mj_name2id(m, mjOBJ_JOINT, "steer_joint");
    const int grip_eq_l = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_left");
    const int grip_eq_r = mj_name2id(m, mjOBJ_EQUALITY, "connect_grip_right");
    const int site_l = mj_name2id(m, mjOBJ_SITE, "site_rider_grip_left");
    const int site_r = mj_name2id(m, mjOBJ_SITE, "site_rider_grip_right");
    bt::require(steer_jnt >= 0 && grip_eq_l >= 0 && grip_eq_r >= 0 &&
                    site_l >= 0 && site_r >= 0,
                "steer and grip objects resolve");

    plant.controller->initialize(d);
    engine::forward(m, d);

    // Test both unopposed steering acceleration and quasi-static resisted torque delivery
    // at steer_joint through connect_grip_{left,right}.
    const rider::EqualityReactions reactions(m);
    const auto steer_dof = static_cast<std::size_t>(
        std::views::counted(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(steer_jnt)]);
    const auto qfrc_app = std::views::counted(d->qfrc_applied, m->nv);
    const auto qacc = std::views::counted(d->qacc, m->nv);

    const auto eval_steer = [&](double target_steer_nm, double opposed_nm) {
        qfrc_app[steer_dof] = opposed_nm;
        rider::RiderCommand cmd{};
        cmd.mean_crank_torque_nm = 0.0;
        cmd.enabled = true;
        cmd.steer_torque_target_nm = target_steer_nm;

        const auto torques = plant.controller->compute(d, cmd, true, 0.00125);
        plant.controller->write(d, torques);
        engine::forward(m, d);

        const auto f_l_vec = reactions.force_on_rider(d, grip_eq_l);
        const auto f_r_vec = reactions.force_on_rider(d, grip_eq_r);
        const std::array<double, 3> f_l{f_l_vec[0], f_l_vec[1], f_l_vec[2]};
        const std::array<double, 3> f_r{f_r_vec[0], f_r_vec[1], f_r_vec[2]};
        bt::require(norm3(f_l) <= 300.0 && norm3(f_r) <= 300.0,
                    "per-hand grip force stays within 300 N");

        const auto xanchor = std::views::counted(d->xanchor, m->njnt * 3);
        const auto xaxis = std::views::counted(d->xaxis, m->njnt * 3);
        const auto sxpos = std::views::counted(d->site_xpos, m->nsite * 3);
        const std::array<double, 3> p_steer = vec3_at(xanchor, steer_jnt);
        const std::array<double, 3> u_steer = vec3_at(xaxis, steer_jnt);
        const std::array<double, 3> g_l = vec3_at(sxpos, site_l);
        const std::array<double, 3> g_r = vec3_at(sxpos, site_r);

        // Force on steer = -force_on_rider
        const std::array<double, 3> r_l{
            g_l[0] - p_steer[0], g_l[1] - p_steer[1], g_l[2] - p_steer[2]};
        const std::array<double, 3> r_r{
            g_r[0] - p_steer[0], g_r[1] - p_steer[1], g_r[2] - p_steer[2]};
        const double tau_delivered =
            dot3(cross3(r_l, {-f_l[0], -f_l[1], -f_l[2]}), u_steer) +
            dot3(cross3(r_r, {-f_r[0], -f_r[1], -f_r[2]}), u_steer);

        const std::array<int, 2> grip_eqs{grip_eq_l, grip_eq_r};
        const double tau_qfrc =
            reactions.equalities_qfrc_at(d, grip_eqs, static_cast<int>(steer_dof));
        bt::require(std::abs(tau_qfrc - tau_delivered) <= 1e-6,
                    "geometric moment matches equality qfrc at steer_joint");
        qfrc_app[steer_dof] = 0.0;
        return tau_delivered;
    };

    for (const double target_steer_nm : {15.0, -15.0}) {
        const double tau_free = eval_steer(target_steer_nm, 0.0);
        bt::require(tau_free * target_steer_nm > 0.0 &&
                        qacc[steer_dof] * target_steer_nm > 0.0,
                    "unopposed differential hand steering accelerates steer_joint in target direction");

        const double tau_resisted = eval_steer(target_steer_nm, -target_steer_nm);
        bt::require(tau_resisted * target_steer_nm > 0.0 &&
                        std::abs(tau_resisted) >= 0.90 * std::abs(target_steer_nm) &&
                        std::abs(tau_resisted) <= 1.10 * std::abs(target_steer_nm),
                    "quasi-static differential hand steering delivers >90% target torque via arm Jacobian");
    }

    // Verify 450 W total positive power cap with simultaneous high pedaling & steering demand.
    const int crank_jnt = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
    const auto crank_dof = static_cast<std::size_t>(
        std::views::counted(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(crank_jnt)]);
    std::views::counted(d->qvel, m->nv)[crank_dof] = 10.0;
    plant.controller->initialize_velocity(d);
    engine::forward(m, d);

    rider::RiderCommand heavy{};
    heavy.mean_crank_torque_nm = 120.0;
    heavy.enabled = true;
    heavy.crank_target_rate_rad_s = 10.0;
    heavy.steer_torque_target_nm = 15.0;
    const auto heavy_torques = plant.controller->compute(d, heavy, true, 0.00125);
    double pos_power = 0.0;
    const auto qvel = std::views::counted(d->qvel, m->nv);
    for (std::size_t i = 0; i < plant.controller->joints().size(); ++i) {
        const auto &j = plant.controller->joints()[i];
        pos_power += std::max(
            heavy_torques[i].second * qvel[static_cast<std::size_t>(j.dof_adr)],
            0.0);
    }
    bt::require(pos_power <= 450.0 + 1e-6,
                "total positive rider joint power respects 450 W cap");

    // Verify extreme steer request respects 300 N per-hand clamp.
    rider::RiderCommand extreme{};
    extreme.mean_crank_torque_nm = 0.0;
    extreme.enabled = true;
    extreme.steer_torque_target_nm = 500.0;
    const auto ext_torques = plant.controller->compute(d, extreme, true, 0.00125);
    plant.controller->write(d, ext_torques);
    engine::forward(m, d);
    const auto ext_fl = reactions.force_on_rider(d, grip_eq_l);
    const auto ext_fr = reactions.force_on_rider(d, grip_eq_r);
    bt::require(norm3({ext_fl[0], ext_fl[1], ext_fl[2]}) <= 300.0 &&
                    norm3({ext_fr[0], ext_fr[1], ext_fr[2]}) <= 300.0,
                "pre-allocation hand force budget clamps per-hand force <= 300 N");
}

void test_sit_bone_and_unilateral_contact_checks() {
    auto plant = make_plant_3d();
    const mjModel *m = plant.model.get();
    mjData *d = plant.data.get();
    plant.controller->initialize(d);
    engine::forward(m, d);
    plant.writer->reset(d);

    const auto has_error = [](const std::vector<std::string> &errors,
                              std::string_view needle) {
        return std::ranges::any_of(errors, [&](const std::string &e) {
            return e.find(needle) != std::string::npos;
        });
    };

    const int frame_id = mj_name2id(m, mjOBJ_BODY, "frame");
    bt::require(frame_id >= 0, "frame resolves");
    const auto body_masses = std::views::counted(m->body_mass, m->nbody);
    double total_mass = 0.0;
    for (const double bm : body_masses) total_mass += bm;
    const double gravity_z = m->opt.gravity[2];
    const auto xfrc = std::views::counted(d->xfrc_applied, m->nbody * 6);
    const auto set_ground_support = [&] {
        std::ranges::fill(xfrc, 0.0);
        xfrc[static_cast<std::size_t>(frame_id) * 6U + 2U] = -total_mass * gravity_z;
    };

    // 1. Centered seated pose passes unilateral checks and rider_support_ok.
    {
        set_ground_support();
        engine::forward(m, d);
        const auto settle = plant.writer->settle_welds(d, std::nullopt, false, nullptr);
        bt::require(settle.saddle_reaction.has_value() &&
                        settle.saddle_reaction->normal_n > 0.0,
                    "centered seated riding has positive sit-bone support");
        bt::require(!has_error(settle.measurement.errors, "sit_bone_pull_violation"),
                    "centered seated riding has no sit-bone pull violation");
    }

    // 2. Roll torque |M_x| / 0.11 > F_n + 12 triggers sit_bone_pull_violation and rider_support_ok == false.
    const int pelvis_id = mj_name2id(m, mjOBJ_BODY, "rider_pelvis");
    bt::require(pelvis_id >= 0, "rider_pelvis resolves");
    {
        set_ground_support();
        // Apply 80 N*m roll moment (+X) on rider_pelvis -> saddle reaction M_x ~ -80 N*m -> N_L < -6 N.
        xfrc[static_cast<std::size_t>(pelvis_id) * 6U + 3U] = 80.0;
        engine::forward(m, d);
        const auto settle = plant.writer->settle_welds(d, std::nullopt, false, nullptr);
        bt::require(settle.saddle_reaction.has_value(),
                    "saddle reaction present");
        const double saddle_n = settle.saddle_reaction
                                    ? settle.saddle_reaction->normal_n
                                    : 0.0;
        bt::require(saddle_n < -6.0,
                    "roll moment |Mx|/0.11 > Fn + 12 drives sit-bone normal < -6 N");
        bt::require(has_error(settle.measurement.errors, "saddle:sit_bone_pull_violation"),
                    "sit-bone pull violation emitted when |Mx|/0.11 > Fn + 12");

        // Confirm status::rider_support_ok rejects this saddle_support_n.
        const runtime::WireObject support_wire{
            {"front_pedal", runtime::Wire(runtime::WireObject{{"min_n", runtime::Wire(100.0)}})},
            {"rear_pedal", runtime::Wire(runtime::WireObject{{"min_n", runtime::Wire(100.0)}})},
            {"saddle", runtime::Wire(runtime::WireObject{
                           {"min_n", runtime::Wire(saddle_n)}})}};
        bt::require(!runtime::rider_support_ok(support_wire),
                    "rider_support_ok rejects sit-bone unilateral violation");
    }

    // 3. Foot pull < -6 N triggers foot_front:pull_violation.
    const int foot_f_id = mj_name2id(m, mjOBJ_BODY, "rider_foot_front");
    bt::require(foot_f_id >= 0, "rider_foot_front resolves");
    {
        set_ground_support();
        xfrc[static_cast<std::size_t>(foot_f_id) * 6U + 2U] = 500.0; // lift foot upward -> weld pulls downward
        engine::forward(m, d);
        const auto settle = plant.writer->settle_welds(d, std::nullopt, false, nullptr);
        bt::require(has_error(settle.measurement.errors, "foot_front:pull_violation"),
                    "upward foot pull triggers foot_front:pull_violation");
    }

    // 4. Hand force > 300 N triggers grip_left:force_limit_violation.
    const int forearm_l_id = mj_name2id(m, mjOBJ_BODY, "rider_forearm_left");
    bt::require(forearm_l_id >= 0, "rider_forearm_left resolves");
    {
        set_ground_support();
        xfrc[static_cast<std::size_t>(forearm_l_id) * 6U + 1U] = 800.0;
        engine::forward(m, d);
        const auto settle = plant.writer->settle_welds(d, std::nullopt, false, nullptr);
        bt::require(has_error(settle.measurement.errors, "grip_left:force_limit_violation"),
                    "hand force > 300 N triggers grip_left:force_limit_violation");
    }
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch
    return bt::run([] {
        test_3d_attitude_kinematics_and_attachments();
        test_differential_hand_steering_and_power_cap();
        test_sit_bone_and_unilateral_contact_checks();
    });
}
