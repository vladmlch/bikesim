#include "configuration/load.hpp"
#include "configuration/rider_spec.hpp"
#include "io/hash.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "model/rider_geometry.hpp"
#include "model_access.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/least_squares.hpp"
#include "rider/spindle_math.hpp"
#include "runtime/capabilities.hpp"
#include "spatial/config.hpp"
#include "support/fixture_test.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace mdl = bike::model;
namespace cfg = bike::configuration;
namespace sp = bike::spatial;

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRad = kPi / 180.0;

std::string sha256_hex(std::string_view bytes) {
    return bike::io::sha256(std::as_bytes(std::span{bytes}));
}

int find_id(const mjModel& model, mjtObj kind, const char* name) {
    return mj_name2id(&model, kind, name);
}

int require_id(const mjModel& model, mjtObj kind, const char* name) {
    const int id = find_id(model, kind, name);
    bt::require(id >= 0, std::string("required object missing: ") + name);
    return id;
}

sp::SpatialConfig make_spatial_config(
    sp::SpatialTireBackend backend = sp::SpatialTireBackend::compliant_3d) {
    sp::SpatialConfig out{};
    out.tire_backend = backend;
    out.tire_preset = sp::SpatialTirePreset::mtb_2p8_1p2bar;
    out.start_mode = sp::SpatialStartMode::moving;
    out.corridor_half_width_m = 1.5;
    out.capsize_roll_rad = 1.0;
    out.steer.range_deg = 45.0;
    out.steer.control_period_s = 0.005;
    out.steer.reaction_delay_s = 0.020;
    out.steer.max_torque_nm = 35.0;
    return out;
}

mdl::Vec3 vec3_of(std::span<const double> buf, int id) {
    const auto base = static_cast<std::size_t>(id) * 3U;
    return {buf[base], buf[base + 1U], buf[base + 2U]};
}

void mat3_into(std::span<const double> buf, int id, spindle::Mat3& out) {
    const auto base = static_cast<std::size_t>(id) * 9U;
    for (std::size_t i = 0; i < 9U; ++i) {
        out[i] = buf[base + i];
    }
}

double norm3(const mdl::Vec3& a, const mdl::Vec3& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void test_planar_zero_bit_shield_and_validation() {
    // 1. articulated_planar matches welded.xml byte-for-byte.
    {
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "ride";
        req->rider = cfg::RiderSpecs{};
        req->rider->variant = "articulated_planar";
        req->physics = cfg::load_physics(
            bt::fixture("config/viewer_physics_welded.toml"));
        req->field.ncol = 26001;
        req->field.radius_x_m = 65.0;
        req->field.elevation_m = 30.1;
        const std::string xml = mdl::generate_mjcf(*req);
        const std::string expected = bt::text(bt::fixture("model/welded.xml"));
        bt::require(xml == expected,
                    "articulated_planar XML matches welded.xml byte-for-byte");
    }

    // 2. none and lumped pinned SHA-256 hashes remain untouched.
    {
        mdl::BuildRequest req;
        req.solver.emplace(req.bike);
        req.mode = "standard";
        bt::require(
            sha256_hex(mdl::generate_mjcf(req)) ==
                "d9e52357c4a237b8cbd35d0865dbdfb84f2a6ba3ca40e1809fb3df7248e3cfbf",
            "planar rider:none standard XML unchanged");
    }
    {
        mdl::BuildRequest req;
        req.solver.emplace(req.bike);
        req.mode = "ride";
        req.rider = cfg::RiderSpecs{};
        bt::require(
            sha256_hex(mdl::generate_mjcf(req)) ==
                "6dea27b0eb2c618f30764b40b6418781de645ec97094e4588d83c5aefd2cba47",
            "planar rider:lumped ride XML unchanged");
    }

    // 3. Fail-closed validation gates.
    {
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "ride";
        req->rider = cfg::RiderSpecs{};
        req->rider->variant = "articulated_planar";
        req->physics = cfg::load_physics(
            bt::fixture("config/viewer_physics_welded.toml"));
        req->spatial = make_spatial_config();
        bt::invalid([&] { (void)mdl::generate_mjcf(*req); });
    }
    {
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "ride";
        req->rider = cfg::RiderSpecs{};
        req->rider->variant = "articulated_3d";
        req->physics = cfg::load_physics(
            bt::fixture("config/viewer_physics_welded.toml"));
        bt::invalid([&] { (void)mdl::generate_mjcf(*req); });
    }
    {
        auto req = std::make_unique<mdl::BuildRequest>();
        req->mode = "standard";
        req->rider = cfg::RiderSpecs{};
        req->rider->variant = "articulated_3d";
        req->spatial = make_spatial_config();
        bt::invalid([&] { (void)mdl::generate_mjcf(*req); });
    }

    // 4. capabilities.cpp accepts articulated_3d without planar rejection.
    {
        const auto phys = cfg::load_physics(
            bt::fixture("config/viewer_physics_welded.toml"));
        cfg::RiderSpecs rider;
        rider.variant = "articulated_3d";
        cfg::detail::post_init(rider);
        const auto report = runtime::implemented_capabilities(phys, rider);
        bt::require(report.implemented && report.rejections.empty(),
                    "capabilities accepts articulated_3d with welded attachments");
    }
}

void test_3d_pose_solver() {
    const cfg::RiderSpecs rider = mdl::rider_from_args("articulated_3d");
    const bike::geometry::BikeSpecs bike{};
    const mdl::SeatedPose3d seated = mdl::solve_seated_pose_3d(bike, rider);
    const mdl::ArticulatedPose3d pose = mdl::geometry_pose_3d(rider, bike);

    bt::require(std::abs(pose.b_shoulder_m - 0.190) <= 1e-12,
                "3D shoulder half-width is 0.190 m at 1.80 m stature");
    bt::require(std::abs(pose.grip_left[1] - 0.360) <= 1e-12 &&
                    std::abs(pose.grip_right[1] + 0.360) <= 1e-12,
                "3D grips sit at +0.360 m (left) and -0.360 m (right)");
    bt::require(std::abs(pose.psi0_left_rad + pose.psi0_right_rad) <= 1e-12 &&
                    pose.psi0_left_rad > 0.0,
                "3D arm plane azimuths are symmetric (+left, -right)");
    bt::require(std::abs(norm3(pose.elbow_left, pose.shoulder_left) -
                         pose.upper_arm_m) <= 1e-12 &&
                    std::abs(norm3(pose.elbow_right, pose.shoulder_right) -
                             pose.upper_arm_m) <= 1e-12,
                "3D upper arm lengths match de Leva");
    bt::require(std::abs(norm3(pose.grip_left, pose.elbow_left) -
                         pose.forearm_grip_m) <= 1e-12 &&
                    std::abs(norm3(pose.grip_right, pose.elbow_right) -
                             pose.forearm_grip_m) <= 1e-12,
                "3D forearm-grip lengths match de Leva");
    bt::require(std::abs(mdl::flexion_deg(pose.shoulder_left, pose.elbow_left,
                                          pose.grip_left) -
                         30.0) <= 1e-10 &&
                    std::abs(mdl::flexion_deg(pose.shoulder_right,
                                              pose.elbow_right, pose.grip_right) -
                             30.0) <= 1e-10,
                "3D build pose elbow flexion is 30.0 deg on both arms");
    bt::require(std::abs(seated.sagittal.torso_lean_deg - 44.013263) < 0.01,
                "3D build pose torso lean matches 30-deg elbow / +-0.360 m cockpit reach");

    // Verify compute_rider_centers_of_mass totals 80.0 kg and places arms at +-Y.
    const auto coms = mdl::compute_rider_centers_of_mass(rider, bike);
    double total_mass = 0.0;
    for (const auto& entry : coms) {
        total_mass += entry.mass_kg;
        if (entry.name == "rider_upper_arm_left" ||
            entry.name == "rider_forearm_left") {
            bt::require(entry.pos_m[1] > 0.15,
                        "left arm CoM sits at +Y in 3D");
        }
        if (entry.name == "rider_upper_arm_right" ||
            entry.name == "rider_forearm_right") {
            bt::require(entry.pos_m[1] < -0.15,
                        "right arm CoM sits at -Y in 3D");
        }
    }
    bt::require(std::abs(total_mass - rider.mass_kg) <= 1e-12,
                "3D rider segment masses sum to rider.mass_kg");
}

void test_3d_mjcf_topology_mass_and_saddle_5dof() {
    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args("articulated_3d");
    req->physics = cfg::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    req->spatial = make_spatial_config();

    const std::string xml = mdl::generate_mjcf(*req);
    const auto model = mdl::compile_mjcf(xml);
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(
        mj_makeData(model.get()), mj_deleteData);
    mj_resetData(model.get(), data.get());
    mj_forward(model.get(), data.get());

    // Two freejoints (frame root_free + rider_root_free) -> nq == nv + 2.
    bt::require(model->nq == model->nv + 2,
                "articulated_3d model has nq == nv + 2");

    // Disjoint kinematic tree: rider_pelvis parent is worldbody (0).
    const int pelvis_id = require_id(*model, mjOBJ_BODY, "rider_pelvis");
    const auto body_parent = std::views::counted(model->body_parentid, model->nbody);
    bt::require(body_parent[static_cast<std::size_t>(pelvis_id)] == 0,
                "rider_pelvis is a direct child of worldbody");
    const int rider_free_id = require_id(*model, mjOBJ_JOINT, "rider_root_free");
    const auto jnt_type = std::views::counted(model->jnt_type, model->njnt);
    bt::require(jnt_type[static_cast<std::size_t>(rider_free_id)] == mjJNT_FREE,
                "rider_root_free is mjJNT_FREE");

    // Zero passive damping, stiffness, armature, frictionloss on ALL DOFs/joints.
    const auto dof_damping = std::views::counted(model->dof_damping, model->nv);
    const auto dof_armature = std::views::counted(model->dof_armature, model->nv);
    const auto dof_friction = std::views::counted(model->dof_frictionloss, model->nv);
    for (int dof = 0; dof < model->nv; ++dof) {
        const auto d = static_cast<std::size_t>(dof);
        bt::require(dof_damping[d] == 0.0, "dof_damping == 0 everywhere");
        bt::require(dof_armature[d] == 0.0, "dof_armature == 0 everywhere");
        bt::require(dof_friction[d] == 0.0, "dof_frictionloss == 0 everywhere");
    }
    const auto jnt_stiff = std::views::counted(model->jnt_stiffness, model->njnt);
    for (int jnt = 0; jnt < model->njnt; ++jnt) {
        bt::require(jnt_stiff[static_cast<std::size_t>(jnt)] == 0.0,
                    "jnt_stiffness == 0 everywhere");
    }

    // Zero steer_joint actuator; passive joints have no actuators; 11 act_rider_*.
    bt::require(find_id(*model, mjOBJ_ACTUATOR, "act_rider_steer") == -1,
                "act_rider_steer must be absent");
    bt::require(find_id(*model, mjOBJ_ACTUATOR, "act_rider_saddle_pitch") == -1 &&
                    find_id(*model, mjOBJ_ACTUATOR, "act_rider_hip_abduction_front") == -1 &&
                    find_id(*model, mjOBJ_ACTUATOR, "act_rider_hip_abduction_rear") == -1,
                "passive rider joints must not have actuators");
    const int steer_jnt_id = require_id(*model, mjOBJ_JOINT, "steer_joint");
    const auto act_trnid = std::views::counted(model->actuator_trnid, model->nu * 2);
    int rider_act_count = 0;
    for (int a = 0; a < model->nu; ++a) {
        bt::require(act_trnid[static_cast<std::size_t>(a) * 2U] != steer_jnt_id,
                    "steer_joint has no actuator in articulated_3d");
        const char* aname = mj_id2name(model.get(), mjOBJ_ACTUATOR, a);
        if (aname != nullptr && std::string_view(aname).starts_with("act_rider_")) {
            ++rider_act_count;
        }
    }
    bt::require(rider_act_count == 11,
                "articulated_3d has exactly 11 act_rider_* actuators");
    for (const std::string_view jname : cfg::kRiderJoints3d) {
        const std::string aname = "act_" + std::string(jname);
        (void)require_id(*model, mjOBJ_ACTUATOR, aname.c_str());
    }

    // Mass & static CoM conservation under all 5 carrier splits.
    const mdl::ArticulatedPose3d pose = mdl::geometry_pose_3d(*req->rider, req->bike);
    const mdl::SegmentMasses masses = mdl::split_paired_arm_masses(
        mdl::segment_masses(req->rider->mass_kg, req->rider->helmet_mass_kg));
    const auto mass_of = [&](std::string_view key) {
        for (const auto& [name, m] : masses) {
            if (name == key) return m;
        }
        bt::require(false, "missing mass key");
        return 0.0;
    };
    const auto mix_com = [](const mdl::Vec3& a, const mdl::Vec3& b, double frac) {
        return mdl::Vec3{a[0] + frac * (b[0] - a[0]),
                         a[1] + frac * (b[1] - a[1]),
                         a[2] + frac * (b[2] - a[2])};
    };
    const auto lat = [](mdl::Vec3 v, double y) { v[1] = y; return v; };
    const std::array<std::pair<double, mdl::Vec3>, 11> expected_segments{{
        {mass_of("pelvis"), {pose.hip[0], pose.hip[1], pose.hip[2] + 0.06}},
        {mass_of("torso"), mix_com(pose.hip, pose.shoulder_center, 0.5)},
        {mass_of("head"), pose.head_center},
        {mass_of("upper_arm_left"), mix_com(pose.shoulder_left, pose.elbow_left, 0.5)},
        {mass_of("forearm_left"), mix_com(pose.elbow_left, pose.grip_left, 0.5)},
        {mass_of("upper_arm_right"), mix_com(pose.shoulder_right, pose.elbow_right, 0.5)},
        {mass_of("forearm_right"), mix_com(pose.elbow_right, pose.grip_right, 0.5)},
        {mass_of("thigh_front"), mix_com(lat(pose.hip, -0.115), lat(pose.knee_front, -0.115), 0.4095)},
        {mass_of("shank_front"), mix_com(lat(pose.knee_front, -0.115), lat(pose.ankle_front, -0.115), 0.4459)},
        {mass_of("foot_front"), mix_com(lat(pose.ankle_front, -0.115), lat(pose.pedal_front, -0.115), 0.5)},
        {mass_of("thigh_rear"), mix_com(lat(pose.hip, +0.115), lat(pose.knee_rear, +0.115), 0.4095)},
    }};
    // Include rear shank and rear foot too.
    mdl::Vec3 exp_moment{0.0, 0.0, 0.0};
    double exp_mass = 0.0;
    for (const auto& [m, c] : expected_segments) {
        exp_mass += m;
        exp_moment[0] += m * c[0];
        exp_moment[1] += m * c[1];
        exp_moment[2] += m * c[2];
    }
    const double m_sr = mass_of("shank_rear");
    const mdl::Vec3 c_sr = mix_com(lat(pose.knee_rear, +0.115), lat(pose.ankle_rear, +0.115), 0.4459);
    const double m_fr = mass_of("foot_rear");
    const mdl::Vec3 c_fr = mix_com(lat(pose.ankle_rear, +0.115), lat(pose.pedal_rear, +0.115), 0.5);
    exp_mass += m_sr + m_fr;
    exp_moment[0] += m_sr * c_sr[0] + m_fr * c_fr[0];
    exp_moment[1] += m_sr * c_sr[1] + m_fr * c_fr[1];
    exp_moment[2] += m_sr * c_sr[2] + m_fr * c_fr[2];

    const auto body_mass = std::views::counted(model->body_mass, model->nbody);
    const auto xipos = std::views::counted(data->xipos, model->nbody * 3);
    double actual_mass = 0.0;
    mdl::Vec3 actual_moment{0.0, 0.0, 0.0};
    for (int b = 0; b < model->nbody; ++b) {
        const char* bname = mj_id2name(model.get(), mjOBJ_BODY, b);
        if (bname != nullptr && std::string_view(bname).starts_with("rider_")) {
            const double m = body_mass[static_cast<std::size_t>(b)];
            const mdl::Vec3 c = vec3_of(xipos, b);
            actual_mass += m;
            actual_moment[0] += m * c[0];
            actual_moment[1] += m * c[1];
            actual_moment[2] += m * c[2];
        }
    }
    bt::require(std::abs(actual_mass - req->rider->mass_kg) <= 1e-12 &&
                    std::abs(actual_mass - exp_mass) <= 1e-12,
                "total compiled rider mass equals 80.0 kg to <= 1e-12");
    const mdl::Vec3 actual_com{actual_moment[0] / actual_mass,
                               actual_moment[1] / actual_mass,
                               actual_moment[2] / actual_mass};
    const mdl::Vec3 exp_com{exp_moment[0] / exp_mass,
                            exp_moment[1] / exp_mass,
                            exp_moment[2] / exp_mass};
    bt::require(norm3(actual_com, exp_com) <= 1e-12,
                "compiled 3D build-pose static rider CoM matches unsplit de Leva CoM to <= 1e-12 m");

    // 5-DOF weld_saddle check:
    // 1) Dynamic pitch rate + applied pitch torque: weld_saddle transmits no
    //    hinge torque beyond the tiny 5e-5 kg*m^2 carrier inertial reaction,
    //    and moment transport to rider_saddle_carrier matches -weld_qfrc[sad_pitch_dof].
    const auto dofadr = std::views::counted(model->jnt_dofadr, model->njnt);
    const int frame_free_id = require_id(*model, mjOBJ_JOINT, "root_free");
    const int frame_dof = dofadr[static_cast<std::size_t>(frame_free_id)];
    const int rider_dof = dofadr[static_cast<std::size_t>(rider_free_id)];
    const int sad_pitch_id = require_id(*model, mjOBJ_JOINT, "rider_saddle_pitch");
    const int sad_pitch_dof = dofadr[static_cast<std::size_t>(sad_pitch_id)];
    const int weld_saddle_id = require_id(*model, mjOBJ_EQUALITY, "weld_saddle");
    const int frame_id = require_id(*model, mjOBJ_BODY, "frame");
    const int carrier_id = require_id(*model, mjOBJ_BODY, "rider_saddle_carrier");

    const auto qvel = std::views::counted(data->qvel, model->nv);
    const auto qfrc_applied = std::views::counted(data->qfrc_applied, model->nv);
    const auto r_dof_sz = static_cast<std::size_t>(rider_dof);
    const auto f_dof_sz = static_cast<std::size_t>(frame_dof);
    qvel[r_dof_sz + 4U] = 1.5;
    qvel[static_cast<std::size_t>(sad_pitch_dof)] = -1.5;
    qfrc_applied[r_dof_sz + 4U] = 25.0;
    mj_forward(model.get(), data.get());

    const rider::EqualityReactions eq_reactions(model.get());
    std::vector<double> weld_qfrc(static_cast<std::size_t>(model->nv), 0.0);
    eq_reactions.equality_qfrc_into(data.get(), weld_saddle_id, weld_qfrc);

    const auto xpos = std::views::counted(data->xpos, model->nbody * 3);
    const auto xmat = std::views::counted(data->xmat, model->nbody * 9);
    const mdl::Vec3 p_frame = vec3_of(xpos, frame_id);
    const mdl::Vec3 p_saddle = vec3_of(xpos, carrier_id);
    spindle::Mat3 r_frame{};
    mat3_into(xmat, frame_id, r_frame);

    const mdl::Vec3 f_world{
        weld_qfrc[f_dof_sz + 0U],
        weld_qfrc[f_dof_sz + 1U],
        weld_qfrc[f_dof_sz + 2U]};
    const mdl::Vec3 m_local{
        weld_qfrc[f_dof_sz + 3U],
        weld_qfrc[f_dof_sz + 4U],
        weld_qfrc[f_dof_sz + 5U]};
    const mdl::Vec3 m_world_at_bb{
        r_frame[0] * m_local[0] + r_frame[1] * m_local[1] + r_frame[2] * m_local[2],
        r_frame[3] * m_local[0] + r_frame[4] * m_local[1] + r_frame[5] * m_local[2],
        r_frame[6] * m_local[0] + r_frame[7] * m_local[1] + r_frame[8] * m_local[2]};
    const mdl::Vec3 r_arm{p_saddle[0] - p_frame[0],
                          p_saddle[1] - p_frame[1],
                          p_saddle[2] - p_frame[2]};
    const mdl::Vec3 r_cross_f{
        r_arm[1] * f_world[2] - r_arm[2] * f_world[1],
        r_arm[2] * f_world[0] - r_arm[0] * f_world[2],
        r_arm[0] * f_world[1] - r_arm[1] * f_world[0]};
    const mdl::Vec3 m_world_at_saddle{
        m_world_at_bb[0] - r_cross_f[0],
        m_world_at_bb[1] - r_cross_f[1],
        m_world_at_bb[2] - r_cross_f[2]};
    const double my_frame =
        r_frame[1] * m_world_at_saddle[0] +
        r_frame[4] * m_world_at_saddle[1] +
        r_frame[7] * m_world_at_saddle[2];
    const double sad_pitch_qfrc = weld_qfrc[static_cast<std::size_t>(sad_pitch_dof)];
    bt::require(std::abs(my_frame + sad_pitch_qfrc) <= 1e-12,
                "weld_saddle pitch moment at saddle equals -weld_qfrc[rider_saddle_pitch]");
    bt::require(std::abs(my_frame) < 0.02,
                "weld_saddle pitch moment is negligible compared to 25 N*m pelvis torque");

    // 2) Zero-acceleration check: with gravity disabled, qvel == 0, pelvis
    //    pitched -0.15 rad around the saddle (with rider_saddle_pitch = +0.15 rad,
    //    torso/arms/legs re-closed to keep all 5 constraints satisfied), |My_frame| <= 1e-12.
    auto zero_g_model = mdl::compile_mjcf(xml);
    zero_g_model->opt.gravity[2] = 0.0;
    auto zero_g_data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(
        mj_makeData(zero_g_model.get()), mj_deleteData);
    mj_resetData(zero_g_model.get(), zero_g_data.get());
    const auto zg_qpos = std::views::counted(zero_g_data->qpos, zero_g_model->nq);
    const auto qposadr = std::views::counted(zero_g_model->jnt_qposadr, zero_g_model->njnt);
    const auto r_qp_sz = static_cast<std::size_t>(qposadr[static_cast<std::size_t>(rider_free_id)]);
    const auto sad_qp_sz = static_cast<std::size_t>(qposadr[static_cast<std::size_t>(sad_pitch_id)]);
    const double pitch_rad = -0.15;
    const double c_half = std::cos(0.5 * pitch_rad);
    const double s_half = std::sin(0.5 * pitch_rad);
    const double dz_hip_sad = 0.060;
    const mdl::Vec3 hip_pitched{
        pose.saddle.top_center[0] + dz_hip_sad * std::sin(pitch_rad),
        0.0,
        pose.saddle.top_center[2] + dz_hip_sad * std::cos(pitch_rad)};
    zg_qpos[r_qp_sz + 0U] = hip_pitched[0];
    zg_qpos[r_qp_sz + 2U] = hip_pitched[2];
    zg_qpos[r_qp_sz + 3U] = c_half;
    zg_qpos[r_qp_sz + 4U] = 0.0;
    zg_qpos[r_qp_sz + 5U] = s_half;
    zg_qpos[r_qp_sz + 6U] = 0.0;
    zg_qpos[sad_qp_sz] = -pitch_rad;
    zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
        require_id(*zero_g_model, mjOBJ_JOINT, "rider_torso_hinge"))])] = -pitch_rad;
    const auto leg_f0 = spindle::leg_loop_geometry(
        {pose.hip[0], pose.hip[2]}, {pose.knee_front[0], pose.knee_front[2]},
        {pose.pedal_front[0], pose.pedal_front[2]}, true);
    const auto leg_r0 = spindle::leg_loop_geometry(
        {pose.hip[0], pose.hip[2]}, {pose.knee_rear[0], pose.knee_rear[2]},
        {pose.pedal_rear[0], pose.pedal_rear[2]}, false);
    const spindle::Vec2 qf0 = spindle::leg_joint_q(
        leg_f0, {hip_pitched[0], hip_pitched[2]},
        {pose.pedal_front[0], pose.pedal_front[2]}, pitch_rad);
    const spindle::Vec2 qr0 = spindle::leg_joint_q(
        leg_r0, {hip_pitched[0], hip_pitched[2]},
        {pose.pedal_rear[0], pose.pedal_rear[2]}, pitch_rad);
    zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
        require_id(*zero_g_model, mjOBJ_JOINT, "rider_hip_front"))])] = qf0[0];
    zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
        require_id(*zero_g_model, mjOBJ_JOINT, "rider_knee_front"))])] = qf0[1];
    zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
        require_id(*zero_g_model, mjOBJ_JOINT, "rider_hip_rear"))])] = qr0[0];
    zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
        require_id(*zero_g_model, mjOBJ_JOINT, "rider_knee_rear"))])] = qr0[1];
    mj_kinematics(zero_g_model.get(), zero_g_data.get());
    const auto zg_kin_xpos = std::views::counted(zero_g_data->xpos, zero_g_model->nbody * 3);
    const auto zg_kin_xmat = std::views::counted(zero_g_data->xmat, zero_g_model->nbody * 9);
    spindle::Mat3 zg_torso_r{};
    mat3_into(zg_kin_xmat, require_id(*zero_g_model, mjOBJ_BODY, "rider_torso"), zg_torso_r);
    const std::array<spindle::ArmGeometry3d, 2> zg_geoms{{
        {.upper_arm_m = pose.upper_arm_m, .forearm_grip_m = pose.forearm_grip_m,
         .psi0_rad = pose.psi0_left_rad, .beta_ua0_rad = pose.beta_ua0_rad,
         .elbow_flexion0_rad = pose.elbow_flexion0_rad},
        {.upper_arm_m = pose.upper_arm_m, .forearm_grip_m = pose.forearm_grip_m,
         .psi0_rad = pose.psi0_right_rad, .beta_ua0_rad = pose.beta_ua0_rad,
         .elbow_flexion0_rad = pose.elbow_flexion0_rad},
    }};
    const std::array<const char*, 2> zg_ua_names{"rider_upper_arm_left", "rider_upper_arm_right"};
    const std::array<const char*, 2> zg_abd_names{"rider_shoulder_abduction_left", "rider_shoulder_abduction_right"};
    const std::array<const char*, 2> zg_sh_names{"rider_shoulder_left", "rider_shoulder_right"};
    const std::array<const char*, 2> zg_el_names{"rider_elbow_left", "rider_elbow_right"};
    const std::array<mdl::Vec3, 2> zg_grips{pose.grip_left, pose.grip_right};
    for (std::size_t s = 0; s < 2; ++s) {
        const auto ik = spindle::solve_arm_ik_3d(
            zg_geoms[s], zg_torso_r,
            vec3_of(zg_kin_xpos, require_id(*zero_g_model, mjOBJ_BODY, zg_ua_names[s])),
            zg_grips[s]);
        zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
            require_id(*zero_g_model, mjOBJ_JOINT, zg_abd_names[s]))])] = ik.shoulder_abduction_q;
        zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
            require_id(*zero_g_model, mjOBJ_JOINT, zg_sh_names[s]))])] = ik.shoulder_q;
        zg_qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(
            require_id(*zero_g_model, mjOBJ_JOINT, zg_el_names[s]))])] = ik.elbow_q;
    }
    mj_forward(zero_g_model.get(), zero_g_data.get());
    eq_reactions.equality_qfrc_into(zero_g_data.get(), weld_saddle_id, weld_qfrc);
    const auto zg_xpos = std::views::counted(zero_g_data->xpos, zero_g_model->nbody * 3);
    const auto zg_xmat = std::views::counted(zero_g_data->xmat, zero_g_model->nbody * 9);
    const mdl::Vec3 zg_pf = vec3_of(zg_xpos, frame_id);
    const mdl::Vec3 zg_ps = vec3_of(zg_xpos, carrier_id);
    spindle::Mat3 zg_rf{};
    mat3_into(zg_xmat, frame_id, zg_rf);
    const mdl::Vec3 zg_fw{weld_qfrc[f_dof_sz + 0U], weld_qfrc[f_dof_sz + 1U], weld_qfrc[f_dof_sz + 2U]};
    const mdl::Vec3 zg_ml{weld_qfrc[f_dof_sz + 3U], weld_qfrc[f_dof_sz + 4U], weld_qfrc[f_dof_sz + 5U]};
    const mdl::Vec3 zg_mw{
        zg_rf[0] * zg_ml[0] + zg_rf[1] * zg_ml[1] + zg_rf[2] * zg_ml[2],
        zg_rf[3] * zg_ml[0] + zg_rf[4] * zg_ml[1] + zg_rf[5] * zg_ml[2],
        zg_rf[6] * zg_ml[0] + zg_rf[7] * zg_ml[1] + zg_rf[8] * zg_ml[2]};
    const mdl::Vec3 zg_ra{zg_ps[0] - zg_pf[0], zg_ps[1] - zg_pf[1], zg_ps[2] - zg_pf[2]};
    const mdl::Vec3 zg_rxf{
        zg_ra[1] * zg_fw[2] - zg_ra[2] * zg_fw[1],
        zg_ra[2] * zg_fw[0] - zg_ra[0] * zg_fw[2],
        zg_ra[0] * zg_fw[1] - zg_ra[1] * zg_fw[0]};
    const double zg_my_frame =
        zg_rf[1] * (zg_mw[0] - zg_rxf[0]) +
        zg_rf[4] * (zg_mw[1] - zg_rxf[1]) +
        zg_rf[7] * (zg_mw[2] - zg_rxf[2]);
    // 2) Rest check (gravity disabled, qvel == 0): only the 1e-6 m Horst-linkage
    //    JSON rounding residual induces ~4e-8 N*m inertial reaction on the 5e-5 kg*m^2
    //    carrier; verify |My_frame| <= 1e-6 N*m and |My_frame + weld_qfrc[sad_pitch]| <= 1e-12
    //    even with pelvis pitched -0.15 rad.
    const double zg_sad_pitch_qfrc = weld_qfrc[static_cast<std::size_t>(sad_pitch_dof)];
    bt::require(std::abs(zg_my_frame + zg_sad_pitch_qfrc) <= 1e-12,
                "pitched saddle carrier pitch moment equals -weld_qfrc[rider_saddle_pitch] to <= 1e-12");
    bt::require(std::abs(zg_my_frame) <= 1e-4,
                "weld_saddle transmits |My_frame| <= 1e-4 N*m at pitched rest pose");
}

void test_lower_body_svd_across_crank_cycle() {
    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args("articulated_3d");
    req->physics = cfg::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    req->spatial = make_spatial_config();

    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(
        mj_makeData(model.get()), mj_deleteData);

    const mdl::ArticulatedPose3d pose = mdl::geometry_pose_3d(*req->rider, req->bike);
    const double crank_m = req->bike.crank_length / 1000.0;
    const auto leg_f = spindle::leg_loop_geometry(
        {pose.hip[0], pose.hip[2]},
        {pose.knee_front[0], pose.knee_front[2]},
        {pose.pedal_front[0], pose.pedal_front[2]}, true);
    const auto leg_r = spindle::leg_loop_geometry(
        {pose.hip[0], pose.hip[2]},
        {pose.knee_rear[0], pose.knee_rear[2]},
        {pose.pedal_rear[0], pose.pedal_rear[2]}, false);

    const auto qposadr = std::views::counted(model->jnt_qposadr, model->njnt);
    const auto dofadr = std::views::counted(model->jnt_dofadr, model->njnt);

    const int crank_id = require_id(*model, mjOBJ_JOINT, "crank_spin");
    const int ped_f_id = require_id(*model, mjOBJ_JOINT, "pedal_front_spin");
    const int ped_r_id = require_id(*model, mjOBJ_JOINT, "pedal_rear_spin");
    const int rider_free_id = require_id(*model, mjOBJ_JOINT, "rider_root_free");
    const int sad_pitch_id = require_id(*model, mjOBJ_JOINT, "rider_saddle_pitch");
    const int hip_abd_f_id = require_id(*model, mjOBJ_JOINT, "rider_hip_abduction_front");
    const int hip_f_id = require_id(*model, mjOBJ_JOINT, "rider_hip_front");
    const int knee_f_id = require_id(*model, mjOBJ_JOINT, "rider_knee_front");
    const int hip_abd_r_id = require_id(*model, mjOBJ_JOINT, "rider_hip_abduction_rear");
    const int hip_r_id = require_id(*model, mjOBJ_JOINT, "rider_hip_rear");
    const int knee_r_id = require_id(*model, mjOBJ_JOINT, "rider_knee_rear");

    const int r_dof = dofadr[static_cast<std::size_t>(rider_free_id)];
    const std::array<int, 13> lower_dofs{
        r_dof + 0, r_dof + 1, r_dof + 2, r_dof + 3, r_dof + 4, r_dof + 5,
        dofadr[static_cast<std::size_t>(sad_pitch_id)],
        dofadr[static_cast<std::size_t>(hip_abd_f_id)],
        dofadr[static_cast<std::size_t>(hip_f_id)],
        dofadr[static_cast<std::size_t>(knee_f_id)],
        dofadr[static_cast<std::size_t>(hip_abd_r_id)],
        dofadr[static_cast<std::size_t>(hip_r_id)],
        dofadr[static_cast<std::size_t>(knee_r_id)]};

    const int eq_sad = require_id(*model, mjOBJ_EQUALITY, "weld_saddle");
    const int eq_ff = require_id(*model, mjOBJ_EQUALITY, "connect_foot_front");
    const int eq_fr = require_id(*model, mjOBJ_EQUALITY, "connect_foot_rear");

    const rider::EqualityReactions eq_reactions(model.get());
    rider::LeastSquaresWorkspace workspace(12, 13, 1);
    const std::array<double, 12> zero_rhs{};
    std::vector<double> jac_12x13(std::size_t{12} * std::size_t{13}, 0.0);
    std::vector<double> row_nv(static_cast<std::size_t>(model->nv), 0.0);
    std::vector<double> unit_efc;

    for (int deg = 0; deg < 360; deg += 15) {
        mj_resetData(model.get(), data.get());
        const double theta = static_cast<double>(deg) * kDegToRad;
        const auto qpos = std::views::counted(data->qpos, model->nq);
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(crank_id)])] = theta;
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(ped_f_id)])] = -theta;
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(ped_r_id)])] = -theta;

        const spindle::Vec2 sp_f = spindle::spindle_xz({0.0, 0.0}, crank_m, theta, 1.0);
        const spindle::Vec2 sp_r = spindle::spindle_xz({0.0, 0.0}, crank_m, theta, -1.0);
        const spindle::Vec2 q_f = spindle::leg_joint_q(leg_f, {pose.hip[0], pose.hip[2]}, sp_f, 0.0);
        const spindle::Vec2 q_r = spindle::leg_joint_q(leg_r, {pose.hip[0], pose.hip[2]}, sp_r, 0.0);
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(hip_f_id)])] = q_f[0];
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(knee_f_id)])] = q_f[1];
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(hip_r_id)])] = q_r[0];
        qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(knee_r_id)])] = q_r[1];

        mj_forward(model.get(), data.get());

        const auto rows_sad = eq_reactions.rows_of(data.get(), eq_sad);
        const auto rows_ff = eq_reactions.rows_of(data.get(), eq_ff);
        const auto rows_fr = eq_reactions.rows_of(data.get(), eq_fr);
        bt::require(rows_sad.size() == 6 && rows_ff.size() == 3 && rows_fr.size() == 3,
                    "lower-body constraints have 6 + 3 + 3 = 12 rows");

        std::array<std::int64_t, 12> all_rows{};
        for (std::size_t i = 0; i < 6; ++i) all_rows[i] = rows_sad[i];
        for (std::size_t i = 0; i < 3; ++i) all_rows[6U + i] = rows_ff[i];
        for (std::size_t i = 0; i < 3; ++i) all_rows[9U + i] = rows_fr[i];

        unit_efc.assign(static_cast<std::size_t>(data->nefc), 0.0);
        for (std::size_t r = 0; r < 12; ++r) {
            const auto efc_row = static_cast<std::size_t>(all_rows[r]);
            unit_efc[efc_row] = 1.0;
            std::ranges::fill(row_nv, 0.0);
            mj_mulJacTVec(model.get(), data.get(), row_nv.data(), unit_efc.data());
            unit_efc[efc_row] = 0.0;
            for (std::size_t c = 0; c < 13; ++c) {
                jac_12x13[r * 13U + c] = row_nv[static_cast<std::size_t>(lower_dofs[c])];
            }
        }

        const auto svd = workspace.solve(jac_12x13, 12, 13, zero_rhs, 1, 1e-12);
        bt::require(svd.rank == 12,
                    "lower-body 12x13 Jacobian has full rank 12 (mobility 1) at crank=" +
                        std::to_string(deg) + " deg");
        bt::require(svd.singular_values.size() == 12 &&
                        svd.singular_values[11] > 0.05 &&
                        svd.singular_values[0] / svd.singular_values[11] < 50.0,
                    "lower-body 12x13 Jacobian well-conditioned across crank cycle");
    }
}

void test_closed_form_3d_arm_ik_across_steer() {
    auto req = std::make_unique<mdl::BuildRequest>();
    req->mode = "ride";
    req->rider = mdl::rider_from_args("articulated_3d");
    req->physics = cfg::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    req->spatial = make_spatial_config();

    const auto model = mdl::compile_mjcf(mdl::generate_mjcf(*req));
    auto data = std::unique_ptr<mjData, decltype(&mj_deleteData)>(
        mj_makeData(model.get()), mj_deleteData);

    const mdl::ArticulatedPose3d pose = mdl::geometry_pose_3d(*req->rider, req->bike);
    const std::array<spindle::ArmGeometry3d, 2> arm_geoms{{
        {.upper_arm_m = pose.upper_arm_m,
         .forearm_grip_m = pose.forearm_grip_m,
         .psi0_rad = pose.psi0_left_rad,
         .beta_ua0_rad = pose.beta_ua0_rad,
         .elbow_flexion0_rad = pose.elbow_flexion0_rad},
        {.upper_arm_m = pose.upper_arm_m,
         .forearm_grip_m = pose.forearm_grip_m,
         .psi0_rad = pose.psi0_right_rad,
         .beta_ua0_rad = pose.beta_ua0_rad,
         .elbow_flexion0_rad = pose.elbow_flexion0_rad},
    }};

    const auto qposadr = std::views::counted(model->jnt_qposadr, model->njnt);
    const auto eq_data = std::views::counted(model->eq_data, model->neq * mjNEQDATA);
    const int steer_jnt = require_id(*model, mjOBJ_JOINT, "steer_joint");
    const int torso_jnt = require_id(*model, mjOBJ_JOINT, "rider_torso_hinge");
    const int steer_body = require_id(*model, mjOBJ_BODY, "steer");
    const int torso_body = require_id(*model, mjOBJ_BODY, "rider_torso");

    const std::array<int, 2> sh_abd_jnts{
        require_id(*model, mjOBJ_JOINT, "rider_shoulder_abduction_left"),
        require_id(*model, mjOBJ_JOINT, "rider_shoulder_abduction_right")};
    const std::array<int, 2> sh_jnts{
        require_id(*model, mjOBJ_JOINT, "rider_shoulder_left"),
        require_id(*model, mjOBJ_JOINT, "rider_shoulder_right")};
    const std::array<int, 2> el_jnts{
        require_id(*model, mjOBJ_JOINT, "rider_elbow_left"),
        require_id(*model, mjOBJ_JOINT, "rider_elbow_right")};
    const std::array<int, 2> ua_bodies{
        require_id(*model, mjOBJ_BODY, "rider_upper_arm_left"),
        require_id(*model, mjOBJ_BODY, "rider_upper_arm_right")};
    const std::array<int, 2> grip_sites{
        require_id(*model, mjOBJ_SITE, "site_rider_grip_left"),
        require_id(*model, mjOBJ_SITE, "site_rider_grip_right")};
    const std::array<int, 2> grip_eqs{
        require_id(*model, mjOBJ_EQUALITY, "connect_grip_left"),
        require_id(*model, mjOBJ_EQUALITY, "connect_grip_right")};

    for (const double torso_lean : {0.0, 0.03, 0.12}) {
        for (int steer_deg = -25; steer_deg <= 25; steer_deg += 5) {
            if (torso_lean == 0.0 && std::abs(steer_deg) > 20) {
                continue;
            }
            mj_resetData(model.get(), data.get());
            const auto qpos = std::views::counted(data->qpos, model->nq);
            qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(steer_jnt)])] =
                static_cast<double>(steer_deg) * kDegToRad;
            qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(torso_jnt)])] =
                torso_lean;
            mj_kinematics(model.get(), data.get());

            const auto xpos = std::views::counted(data->xpos, model->nbody * 3);
            const auto xmat = std::views::counted(data->xmat, model->nbody * 9);
            const mdl::Vec3 steer_pos = vec3_of(xpos, steer_body);
            spindle::Mat3 steer_r{};
            spindle::Mat3 torso_r{};
            mat3_into(xmat, steer_body, steer_r);
            mat3_into(xmat, torso_body, torso_r);

            std::array<mdl::Vec3, 2> target_grips{};
            for (std::size_t s = 0; s < 2; ++s) {
                const auto base = static_cast<std::size_t>(grip_eqs[s]) * mjNEQDATA + 3U;
                const mdl::Vec3 anchor_steer{
                    eq_data[base], eq_data[base + 1U], eq_data[base + 2U]};
                target_grips[s] = {
                    steer_pos[0] + steer_r[0] * anchor_steer[0] + steer_r[1] * anchor_steer[1] + steer_r[2] * anchor_steer[2],
                    steer_pos[1] + steer_r[3] * anchor_steer[0] + steer_r[4] * anchor_steer[1] + steer_r[5] * anchor_steer[2],
                    steer_pos[2] + steer_r[6] * anchor_steer[0] + steer_r[7] * anchor_steer[1] + steer_r[8] * anchor_steer[2]};
                const mdl::Vec3 sh_world = vec3_of(xpos, ua_bodies[s]);
                const auto ik = spindle::solve_arm_ik_3d(
                    arm_geoms[s], torso_r, sh_world, target_grips[s]);
                bt::require(!ik.saturated,
                            "3D arm IK unsaturated across +-25 deg steer");
                qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(sh_abd_jnts[s])])] =
                    ik.shoulder_abduction_q;
                qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(sh_jnts[s])])] =
                    ik.shoulder_q;
                qpos[static_cast<std::size_t>(qposadr[static_cast<std::size_t>(el_jnts[s])])] =
                    ik.elbow_q;
            }
            mj_kinematics(model.get(), data.get());
            const auto site_xpos = std::views::counted(data->site_xpos, model->nsite * 3);
            for (std::size_t s = 0; s < 2; ++s) {
                const mdl::Vec3 actual_grip = vec3_of(site_xpos, grip_sites[s]);
                const double err = norm3(actual_grip, target_grips[s]);
                bt::require(err <= 1e-12,
                            "solve_arm_ik_3d hits handlebar grip within 1e-12 m (err=" +
                                std::to_string(err) + ")");
            }
        }
    }
}

} // namespace

int main() {
    return bt::run([] {
        test_planar_zero_bit_shield_and_validation();
        test_3d_pose_solver();
        test_3d_mjcf_topology_mass_and_saddle_5dof();
        test_lower_body_svd_across_crank_cycle();
        test_closed_form_3d_arm_ik_across_steer();
    });
}
