// rider/spindle_controller.cpp — ArticulatedRiderController's spindle
// branch, ported expression-for-expression from rider_control.py. Only the
// supported welded path exists here; see the header for scope.
//
// Deviations the plan records on purpose:
//  * kinematic_state (a write-only mirror for the legacy allocator path)
//    and the legacy-only fields (pd_split, allocator warm starts,
//    _active_recovery, PedalRecovery.observe/goal) are not carried — the
//    capability gate rejects configurations that would reach them.
//  * Python's ArithmeticError for a violated effort bound is
//    std::runtime_error here; KeyError for missing last_terms rows is
//    std::out_of_range.
#include "spindle_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"

namespace rider {
namespace {

using spindle::Vec2;
using Vec3 = std::array<double, 3>;

constexpr std::array<std::string_view, 11> kReferenceJoints{
    "rider_torso_hinge",
    "rider_shoulder_left", "rider_elbow_left",
    "rider_shoulder_right", "rider_elbow_right",
    "rider_hip_front", "rider_knee_front", "rider_ankle_front",
    "rider_hip_rear", "rider_knee_rear", "rider_ankle_rear"};
constexpr std::array<std::string_view, 2> kSides{"front", "rear"};
constexpr std::array<std::string_view, 2> kArmSides{"left", "right"};

// Leg arrays are keyed by (front, rear) and arm arrays by (left, right);
// both vocabularies index the first/second slot of the same side pair.
[[nodiscard]] std::size_t side_index(std::string_view side) {
    return (side == "front" || side == "left") ? 0U : 1U;
}

[[nodiscard]] std::span<const mjtNum, 3>
row3(const mjtNum *base, mjtSize width, int row, const char *field) {
    const auto span = model_access::readonly_buffer(base, width, field);
    model_access::require_id(row, width / 3, field);
    return span.subspan(static_cast<std::size_t>(row) * 3, 3).first<3>();
}

[[nodiscard]] std::span<const mjtNum, 9>
row9(const mjtNum *base, mjtSize width, int row, const char *field) {
    const auto span = model_access::readonly_buffer(base, width, field);
    model_access::require_id(row, width / 9, field);
    return span.subspan(static_cast<std::size_t>(row) * 9, 9).first<9>();
}

[[nodiscard]] Vec3 body_pos(const mjModel *m, const mjData *d, int body) {
    const auto p = row3(d->xpos, m->nbody * 3, body, "xpos");
    return {p[0], p[1], p[2]};
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
[[nodiscard]] std::array<double, 9> body_mat(const mjModel *m,
                                             const mjData *d, int body) {
    const auto r = row9(d->xmat, m->nbody * 9, body, "xmat");
    std::array<double, 9> out{};
    std::ranges::copy(r, out.begin());
    return out;
}
NATIVE_DIAG_POP

[[nodiscard]] Vec3 site_pos(const mjModel *m, const mjData *d, int site) {
    const auto p = row3(d->site_xpos, m->nsite * 3, site, "site_xpos");
    return {p[0], p[1], p[2]};
}

[[nodiscard]] Vec3 joint_anchor(const mjModel *m, const mjData *d,
                                int joint) {
    const auto p = row3(d->xanchor, m->njnt * 3, joint, "xanchor");
    return {p[0], p[1], p[2]};
}

// (R^T v) restricted to the planar (x, z) coordinates.
[[nodiscard]] Vec2 mat_t_vec_xz(const std::array<double, 9> &r,
                                const Vec3 &v) {
    return {r[0] * v[0] + r[3] * v[1] + r[6] * v[2],
            r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
}

[[nodiscard]] Vec3 mat_vec(const std::array<double, 9> &r, const Vec3 &v) {
    return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
            r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

// r = lhs @ rhs (row-major). Mat3-sized returns are the designed
// interface; the flag's useful half (by-value parameters) stays enabled.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
[[nodiscard]] std::array<double, 9> mat_mul(const std::array<double, 9> &lhs,
                                            const std::array<double, 9> &rhs) {
    std::array<double, 9> out{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            out[i * 3 + j] = lhs[i * 3] * rhs[j] +
                             lhs[i * 3 + 1] * rhs[3 + j] +
                             lhs[i * 3 + 2] * rhs[6 + j];
    return out;
}
NATIVE_DIAG_POP

[[nodiscard]] Vec3 vec_sub(const Vec3 &a, const Vec3 &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

[[nodiscard]] Vec3 vec_add(const Vec3 &a, const Vec3 &b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

// np.linalg.norm on a float64 3-vector is sqrt(dot(v,v)) — a left-to-right
// multiply-add, not std::hypot's scaled algorithm.
[[nodiscard]] double norm3(const Vec3 &v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

[[nodiscard]] Vec2 xz(const std::array<double, 3> &v) {
    return {v[0], v[2]};
}

// np.isclose(a, b, rtol=..., atol=...): |a-b| <= atol + rtol*|b|.
[[nodiscard]] bool isclose(double a, double b, double rtol, double atol) {
    return std::abs(a - b) <= atol + rtol * std::abs(b);
}

// Ordered-dict helpers mirroring Python semantics: d[k]=v updates in
// place or appends at the end; d[k] raises KeyError (std::out_of_range)
// when absent; d.get(k, fallback) returns the default.
template <typename T>
[[nodiscard]] T *named_find(NamedEntries<T> &entries,
                            const std::string &name) {
    for (auto &[key, value] : entries)
        if (key == name)
            return &value;
    return nullptr;
}

template <typename T>
[[nodiscard]] const T *named_find(const NamedEntries<T> &entries,
                                  const std::string &name) {
    for (const auto &[key, value] : entries)
        if (key == name)
            return &value;
    return nullptr;
}

template <typename T>
void named_set(NamedEntries<T> &entries, std::string name, T value) {
    if (T *found = named_find(entries, name)) {
        *found = std::move(value);
        return;
    }
    entries.emplace_back(std::move(name), std::move(value));
}

template <typename T>
[[nodiscard]] T &named_at(NamedEntries<T> &entries,
                          const std::string &name) {
    if (T *found = named_find(entries, name))
        return *found;
    throw std::out_of_range(name);
}

template <typename T>
[[nodiscard]] const T &named_at(const NamedEntries<T> &entries,
                                const std::string &name) {
    if (const T *found = named_find(entries, name))
        return *found;
    throw std::out_of_range(name);
}

template <typename T>
[[nodiscard]] T named_get(const NamedEntries<T> &entries,
                          const std::string &name, const T &fallback) {
    const T *found = named_find(entries, name);
    return found == nullptr ? fallback : *found;
}

// validate_planar_support_model (support_geometry.py:266-286): the pedal
// geoms' ancestor joints must all be scalar-planar (hinge about +/-Y or
// slide with zero Y component) and each geom a proper planar box.
void validate_planar_support(const mjModel *m, const mjData *d,
                             std::span<const int> geoms) {
    const auto geom_body =
        model_access::readonly_buffer(m->geom_bodyid, m->ngeom);
    const auto parents =
        model_access::readonly_buffer(m->body_parentid, m->nbody);
    const auto joint_bodies =
        model_access::readonly_buffer(m->jnt_bodyid, m->njnt);
    const auto joint_types =
        model_access::readonly_buffer(m->jnt_type, m->njnt);
    const auto geom_types =
        model_access::readonly_buffer(m->geom_type, m->ngeom);
    std::vector<int> ancestors;
    for (const int geom : geoms) {
        model_access::require_id(geom, m->ngeom, "pedal geom");
        int body = geom_body[static_cast<std::size_t>(geom)];
        while (body != 0) {
            ancestors.push_back(body);
            body = parents[static_cast<std::size_t>(body)];
        }
    }
    const auto axis =
        model_access::readonly_buffer(d->xaxis, m->njnt * 3, "xaxis");
    for (int joint = 0; joint < m->njnt; ++joint) {
        if (std::ranges::find(ancestors,
                              joint_bodies[static_cast<std::size_t>(
                                  joint)]) == ancestors.end())
            continue;
        const auto base = static_cast<std::size_t>(joint) * 3;
        const int type = joint_types[static_cast<std::size_t>(joint)];
        const bool planar =
            (type == mjJNT_HINGE &&
             std::abs(std::abs(axis[base]) - 0.) <= 1e-9 &&
             std::abs(std::abs(axis[base + 1]) - 1.) <= 1e-9 &&
             std::abs(std::abs(axis[base + 2]) - 0.) <= 1e-9) ||
            (type == mjJNT_SLIDE && std::abs(axis[base + 1]) <= 1e-9);
        if (!planar)
            throw std::invalid_argument(
                "rider supports require scalar planar joint topology");
    }
    for (const int geom : geoms) {
        if (geom_types[static_cast<std::size_t>(geom)] != mjGEOM_BOX)
            throw std::invalid_argument(
                "rider support geometry must be a box");
        const auto origin =
            row3(d->geom_xpos, m->ngeom * 3, geom, "geom_xpos");
        const auto r =
            row9(d->geom_xmat, m->ngeom * 9, geom, "geom_xmat");
        const auto half =
            row3(m->geom_size, m->ngeom * 3, geom, "geom_size");
        for (const double v : origin)
            if (!std::isfinite(v))
                throw std::invalid_argument("invalid box geometry");
        for (const double v : r)
            if (!std::isfinite(v))
                throw std::invalid_argument("invalid box geometry");
        for (const double h : half)
            if (!std::isfinite(h) || h <= 0.)
                throw std::invalid_argument(
                    "box dimensions must be positive");
        const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) -
                           r[1] * (r[3] * r[8] - r[5] * r[6]) +
                           r[2] * (r[3] * r[7] - r[4] * r[6]);
        bool orthonormal = std::abs(det - 1.) <= 1e-9;
        for (std::size_t i = 0; i < 3 && orthonormal; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                const double gram =
                    r[i] * r[j] + r[3 + i] * r[3 + j] + r[6 + i] * r[6 + j];
                if (std::abs(gram - (i == j ? 1. : 0.)) > 1e-9)
                    orthonormal = false;
            }
        const bool planar_column = std::abs(r[1]) <= 1e-9 &&
                                   std::abs(r[4] - 1.) <= 1e-9 &&
                                   std::abs(r[7]) <= 1e-9;
        if (!orthonormal || !planar_column)
            throw std::invalid_argument(
                "support box needs a planar proper rotation");
    }
}

[[nodiscard]] int require_joint(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_JOINT, name);
    if (id < 0)
        throw std::invalid_argument(std::string("model has no '") + name +
                                    "'");
    return id;
}

} // namespace

// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror lean_limit.py
double lean_limit_rad(
    const SpindlePose &pose,
    const std::map<std::string, spindle::JointEnvelope> &envelopes,
    double crank_m, double tdc_phase_rad, double max_rad,
    double tolerance) {
    const double maximum =
        validation::nonnegative(max_rad, "lean search maximum");
    const double tol =
        validation::positive(tolerance, "lean search tolerance");
    const double radius =
        validation::positive(crank_m, "lean crank length");
    const double phase =
        validation::finite(tdc_phase_rad, "top dead centre phase");
    const Vec2 hip = xz(pose.hip);
    const Vec2 trunk{xz(pose.shoulder)[0] - hip[0],
                     xz(pose.shoulder)[1] - hip[1]};
    const Vec2 upper{xz(pose.elbow)[0] - xz(pose.shoulder)[0],
                     xz(pose.elbow)[1] - xz(pose.shoulder)[1]};
    const Vec2 lower{xz(pose.grip)[0] - xz(pose.elbow)[0],
                     xz(pose.grip)[1] - xz(pose.elbow)[1]};
    // np.linalg.norm(2-vec) is sqrt(x*x+z*z), left-to-right — not hypot.
    const double arm_upper =
        std::sqrt(upper[0] * upper[0] + upper[1] * upper[1]);
    const double arm_lower =
        std::sqrt(lower[0] * lower[0] + lower[1] * lower[1]);
    const double upper0 = std::atan2(upper[1], upper[0]);
    const double lower0 = std::atan2(lower[1], lower[0]);
    const int branch = std::sin(lower0 - upper0) >= 0. ? 1 : -1;
    const Vec2 crank{(pose.pedal_front[0] + pose.pedal_rear[0]) / 2.,
                     (pose.pedal_front[2] + pose.pedal_rear[2]) / 2.};
    std::array<double, 2> hips_at_tdc{};
    for (const auto side : kSides) {
        const auto s = side_index(side);
        const auto geometry = spindle::leg_loop_geometry(
            xz(pose.hip), xz(s == 0 ? pose.knee_front : pose.knee_rear),
            xz(s == 0 ? pose.pedal_front : pose.pedal_rear), s == 0);
        const double tdc_phase = phase + (s == 0 ? 0. : std::numbers::pi);
        const Vec2 spindle_pos = spindle::spindle_xz(
            crank, radius, tdc_phase, geometry.side_sign);
        hips_at_tdc[s] =
            spindle::leg_joint_q(geometry, hip, spindle_pos, 0.)[0];
    }
    const auto within = [&envelopes](const std::string &name, double q) {
        const auto entry = envelopes.find(name);
        if (entry == envelopes.end())
            throw std::out_of_range(name);
        const Vec2 range = spindle::joint_q_range(entry->second);
        return range[0] - 1e-12 <= q && q <= range[1] + 1e-12;
    };
    const Vec2 grip_xz = xz(pose.grip);
    const auto feasible = [&](double lean) {
        const double c = std::cos(lean), s = std::sin(lean);
        const Vec2 shoulder{hip[0] + c * trunk[0] + s * trunk[1],
                            hip[1] - s * trunk[0] + c * trunk[1]};
        const Vec2 goal{grip_xz[0] - shoulder[0],
                        grip_xz[1] - shoulder[1]};
        const Vec2 arm_goal{c * goal[0] - s * goal[1],
                            s * goal[0] + c * goal[1]};
        const auto ik =
            spindle::two_link_ik(arm_goal, arm_upper, arm_lower, branch);
        if (ik.saturated || !within("rider_torso_hinge", lean))
            return false;
        for (const auto arm_side : kArmSides) {
            if (!within("rider_shoulder_" + std::string(arm_side),
                        upper0 - ik.angles[0]) ||
                !within("rider_elbow_" + std::string(arm_side),
                        (lower0 - upper0) - ik.angles[1]))
                return false;
        }
        return within("rider_hip_front", hips_at_tdc[0] - lean) &&
               within("rider_hip_rear", hips_at_tdc[1] - lean);
    };
    if (!feasible(0.))
        throw std::invalid_argument(
            "neutral seated posture lies outside reachable joint ROM");
    if (feasible(maximum))
        return maximum;
    double low = 0., high = maximum;
    while (high - low > tol) {
        const double middle = (low + high) / 2;
        if (feasible(middle))
            low = middle;
        else
            high = middle;
    }
    return low;
}
// NOLINTEND(bugprone-easily-swappable-parameters)

SpindleController::SpindleController(const mjModel *model,
                                     const SpindlePose &pose,
                                     SpindleConfig config,
                                     double crank_length_m)
    : model_(model), pose_(pose), config_(std::move(config)),
      crank_length_m_(
          validation::positive(crank_length_m, "crank length")),
      target_data_(nullptr, &mj_deleteData),
      previous_target_data_(nullptr, &mj_deleteData),
      coasting_target_data_(nullptr, &mj_deleteData) {
    if (model == nullptr)
        throw std::invalid_argument("spindle controller needs a model");
    target_data_.reset(engine::make_data(model));
    previous_target_data_.reset(engine::make_data(model));
    coasting_target_data_.reset(engine::make_data(model));
    const auto qpos_adrs =
        model_access::readonly_buffer(model->jnt_qposadr, model->njnt);
    const auto dof_adrs =
        model_access::readonly_buffer(model->jnt_dofadr, model->njnt);
    const auto joint_limited =
        model_access::readonly_buffer(model->jnt_limited, model->njnt);
    const auto joint_range = model_access::readonly_buffer(
        model->jnt_range, model->njnt * 2);
    for (const auto name : kReferenceJoints) {
        const int joint =
            mj_name2id(model, mjOBJ_JOINT, std::string(name).c_str());
        if (joint < 0) {
            locked_joints_.emplace_back(name);
            continue;
        }
        const auto j = static_cast<std::size_t>(joint);
        const int actuator = mj_name2id(
            model, mjOBJ_ACTUATOR,
            ("act_" + std::string(name)).c_str());
        if (actuator < 0)
            throw std::invalid_argument(
                "missing articulated joint/actuator " + std::string(name));
        joints_.push_back(JointEntry{.name = std::string(name),
                                     .qpos_adr = qpos_adrs[j],
                                     .dof_adr = dof_adrs[j],
                                     .actuator_id = actuator});
        if (joint_limited[j]) {
            const auto base = static_cast<std::size_t>(joint) * 2;
            joint_ranges_.emplace(
                std::string(name),
                Vec2{joint_range[base], joint_range[base + 1]});
        }
    }
    // The strength file and the envelope file must agree on the
    // q->anatomical mapping, or a curve could silently bound the wrong
    // anatomical direction.
    for (const auto &[name, coordinate] : config_.strength_coordinates) {
        const auto found = config_.envelopes.find(name);
        if (found != config_.envelopes.end() &&
            (found->second.direction != coordinate.direction ||
             !isclose(found->second.neutral_anatomical_rad,
                      coordinate.neutral_anatomical_rad, 1e-05, 1e-08)))
            throw std::invalid_argument(
                name + ": strength coordinate disagrees with the joint "
                "envelope");
    }
    // The wire tables replace Python's loaders, which required every
    // resolved joint to appear (`present=`); enforce the same coverage.
    if (config_.has_strength)
        for (const auto &joint : joints_)
            if (!config_.strength.contains(joint.name) ||
                !config_.strength_coordinates.contains(joint.name))
                throw std::invalid_argument(
                    joint.name + ": missing strength curves");
    if (!config_.envelopes.empty())
        for (const auto &joint : joints_)
            if (!config_.envelopes.contains(joint.name))
                throw std::invalid_argument(
                    joint.name + ": missing joint envelope");
    // joint_ranges insertion order in Python is self.joints order — the
    // aligned vectors must keep it so `sum(stored)` accumulates in the
    // same sequence as the oracle.
    for (const auto &joint : joints_) {
        const auto range = joint_ranges_.find(joint.name);
        if (range == joint_ranges_.end())
            continue;
        envelope_qpos_adrs_.push_back(joint.qpos_adr);
        envelope_dof_adrs_.push_back(joint.dof_adr);
        envelope_lower_.push_back(range->second[0]);
        envelope_upper_.push_back(range->second[1]);
    }
    reset_activation();
    pelvis_body_ = mj_name2id(model, mjOBJ_BODY, "rider_pelvis");
    torso_joint_ = mj_name2id(model, mjOBJ_JOINT, "rider_torso_hinge");
    for (const auto side : kSides) {
        const auto s = side_index(side);
        const auto stem = std::string(side);
        feet_bodies_[s] = mj_name2id(
            model, mjOBJ_BODY, ("rider_foot_" + stem).c_str());
        sole_sites_[s] = mj_name2id(
            model, mjOBJ_SITE, ("site_rider_sole_" + stem).c_str());
        pedal_geoms_[s] = mj_name2id(
            model, mjOBJ_GEOM, ("geom_pedal_" + stem).c_str());
        pedal_sites_[s] = mj_name2id(
            model, mjOBJ_SITE, ("site_pedal_" + stem).c_str());
        pedal_bodies_[s] = mj_name2id(
            model, mjOBJ_BODY, ("pedal_" + stem).c_str());
        hip_joints_[s] = mj_name2id(
            model, mjOBJ_JOINT, ("rider_hip_" + stem).c_str());
        const int pedal_spin = mj_name2id(
            model, mjOBJ_JOINT, ("pedal_" + stem + "_spin").c_str());
        pedal_spin_dofs_[s] =
            pedal_spin < 0
                ? -1
                : dof_adrs[static_cast<std::size_t>(pedal_spin)];
    }
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        const auto stem = std::string(side);
        upper_arm_bodies_[s] =
            mj_name2id(model, mjOBJ_BODY,
                       ("rider_upper_arm_" + stem).c_str());
        forearm_bodies_[s] =
            mj_name2id(model, mjOBJ_BODY,
                       ("rider_forearm_" + stem).c_str());
        grip_sites_[s] = mj_name2id(model, mjOBJ_SITE,
                                    ("site_rider_grip_" + stem).c_str());
    }
    frame_body_ = mj_name2id(model, mjOBJ_BODY, "frame");
    torso_body_ = mj_name2id(model, mjOBJ_BODY, "rider_torso");
    crank_body_ = mj_name2id(model, mjOBJ_BODY, "crank");
    frame_pitch_dof_ = dof_adrs[static_cast<std::size_t>(
        require_joint(model, "root_pitch"))];
    rider_pitch_dof_ = dof_adrs[static_cast<std::size_t>(
        require_joint(model, "rider_root_pitch"))];
    crank_joint_ = require_joint(model, "crank_spin");
    crank_spin_dof_ =
        dof_adrs[static_cast<std::size_t>(crank_joint_)];
    crank_spin_qpos_ =
        qpos_adrs[static_cast<std::size_t>(crank_joint_)];
    const std::array<std::pair<const char *, int>, 22> interface{{
        {"rider_pelvis", pelvis_body_},       {"crank", crank_body_},
        {"rider_torso", torso_body_},         {"frame", frame_body_},
        {"rider_foot_front", feet_bodies_[0]},
        {"rider_foot_rear", feet_bodies_[1]},
        {"site_rider_sole_front", sole_sites_[0]},
        {"site_rider_sole_rear", sole_sites_[1]},
        {"site_pedal_front", pedal_sites_[0]},
        {"site_pedal_rear", pedal_sites_[1]},
        {"pedal_front", pedal_bodies_[0]},
        {"pedal_rear", pedal_bodies_[1]},
        {"rider_hip_front", hip_joints_[0]},
        {"rider_hip_rear", hip_joints_[1]},
        {"rider_upper_arm_left", upper_arm_bodies_[0]},
        {"rider_upper_arm_right", upper_arm_bodies_[1]},
        {"rider_forearm_left", forearm_bodies_[0]},
        {"rider_forearm_right", forearm_bodies_[1]},
        {"site_rider_grip_left", grip_sites_[0]},
        {"site_rider_grip_right", grip_sites_[1]},
        {"pedal_front_spin", pedal_spin_dofs_[0]},
        {"pedal_rear_spin", pedal_spin_dofs_[1]},
    }};
    for (const auto &[name, id] : interface)
        if (id < 0)
            throw std::invalid_argument(
                std::string("missing rider interface '") + name + "'");
    engine::kinematics(model, target_data_.get());
    // Welded grip ('connect' equality): aim the arm IK at the connect
    // datum — the steer point each grip site occupies at qpos0.
    steer_body_ = mj_name2id(model, mjOBJ_BODY, "steer");
    for (const auto side : kArmSides) {
        const int eq = mj_name2id(
            model, mjOBJ_EQUALITY,
            ("connect_grip_" + std::string(side)).c_str());
        if (eq < 0)
            throw std::invalid_argument(
                "missing compiled grip connect equality");
        const auto base = static_cast<std::size_t>(eq) * mjNEQDATA;
        const auto eq_data = model_access::readonly_buffer(
            model->eq_data, model->neq * mjNEQDATA, "eq_data");
        std::ranges::copy(eq_data.subspan(base + 3, 3).first<3>(),
                          weld_grip_offset_[side_index(side)].begin());
    }
    validate_planar_support(model, target_data_.get(),
                            std::span{pedal_geoms_});
    for (const auto side : kSides) {
        const auto s = side_index(side);
        leg_geometry_[s] = spindle::leg_loop_geometry(
            xz(pose_.hip), xz(s == 0 ? pose_.knee_front : pose_.knee_rear),
            xz(s == 0 ? pose_.pedal_front : pose_.pedal_rear), s == 0);
        for (const auto joint :
             std::array<std::string_view, 3>{"hip", "knee", "ankle"}) {
            const auto full = std::string("rider_") +
                              std::string(joint) + "_" + std::string(side);
            const auto entry = std::ranges::find_if(
                joints_,
                [&](const JointEntry &j) { return j.name == full; });
            if (entry != joints_.end())
                leg_joint_index_[s].push_back(
                    static_cast<std::size_t>(entry - joints_.begin()));
        }
    }
    const Vec3 trunk = vec_sub(pose_.shoulder, pose_.hip);
    const Vec3 upper = vec_sub(pose_.elbow, pose_.shoulder);
    const Vec3 lower = vec_sub(pose_.grip, pose_.elbow);
    arm_upper_m_ = norm3(upper);
    arm_lower_m_ = norm3(lower);
    arm_upper_angle0_ = std::atan2(upper[2], upper[0]);
    arm_lower_angle0_ = std::atan2(lower[2], lower[0]);
    arm_branch_ =
        std::sin(arm_lower_angle0_ - arm_upper_angle0_) >= 0. ? 1 : -1;
    {
        const Vec2 goal{xz(vec_sub(pose_.grip, pose_.hip))};
        const double reach =
            (arm_upper_m_ + arm_lower_m_) * config_.arm_reach_fraction;
        const auto first =
            spindle::two_link_ik(goal, norm3(trunk), reach, -1);
        const auto second =
            spindle::two_link_ik(goal, norm3(trunk), reach, 1);
        const auto &neutral =
            std::sin(first.angles[0]) >= std::sin(second.angles[0])
                ? first
                : second;
        neutral_torso_saturated_ = neutral.saturated;
        neutral_torso_q_ = 0.;
    }
    if (!config_.envelopes.empty())
        lean_limit_rad_ =
            rider::lean_limit_rad(pose_, config_.envelopes,
                                  crank_length_m_, -std::numbers::pi / 2.);
    for (const auto &joint : joints_) {
        joint_torques_nm_.emplace_back(joint.name, 0.);
        joint_capacity_nm_.emplace_back(joint.name, 0.);
    }
    for (int body = 0; body < model->nbody; ++body) {
        const char *name = mj_id2name(model, mjOBJ_BODY, body);
        if (name != nullptr &&
            std::string_view(name).starts_with("rider_"))
            rider_bodies_.push_back(body);
    }
    const auto mass = model_access::readonly_buffer(
        model->body_mass, model->nbody, "body_mass");
    for (const int body : rider_bodies_)
        rider_mass_ += mass[static_cast<std::size_t>(body)];
}

SpindleController::~SpindleController() = default;

void SpindleController::reset_activation() {
    active_state_.assign(joints_.size(), 0.);
    activation_time_s_ = std::nullopt;
    effort_diagnostics_ = {};
    sole_goal_diagnostics_.clear();
    pedal_recovery_ = {{"front", PedalRecoveryState{}},
                       {"rear", PedalRecoveryState{}}};
}

spindle::Vec2 SpindleController::leg_targets(const mjData *data,
                                             const char *side) {
    const auto s = side_index(side);
    const auto pelvis_r = body_mat(model_, data, pelvis_body_);
    const double pitch = std::atan2(pelvis_r[2], pelvis_r[0]);
    const Vec2 hip = xz(joint_anchor(model_, data, hip_joints_[s]));
    const Vec3 spindle_world = site_pos(model_, data, pedal_sites_[s]);
    const auto key = std::string(side);
    named_set(sole_targets_, key, spindle_world);
    named_set(sole_goal_diagnostics_, key,
              SoleGoalDiagnostics{.spindle = true,
                                  .saturated = false,
                                  .limiting_reasons = {}});
    // Python chains ik_reach_limited[side] = saturated_ik[side] = False.
    named_set(saturated_ik_, key, false);
    named_set(ik_reach_limited_, key, false);
    return spindle::leg_joint_q(leg_geometry_[s], hip,
                                xz(spindle_world), pitch);
}

std::vector<std::pair<std::string, double>>
SpindleController::upper_targets(const mjData *data,
                                 const RiderPosture *posture) {
    const auto pelvis_r = body_mat(model_, data, pelvis_body_);
    const auto frame_r = body_mat(model_, data, frame_body_);
    const auto steer_r = body_mat(model_, data, steer_body_);
    const Vec3 steer_pos = body_pos(model_, data, steer_body_);
    std::array<Vec3, 2> grips{};
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        grips[s] =
            vec_add(steer_pos, mat_vec(steer_r, weld_grip_offset_[s]));
    }
    double torso_q = neutral_torso_q_ +
                     (posture == nullptr ? 0. : posture->torso_lean_rad);
    const double frame_pitch = std::atan2(frame_r[2], frame_r[0]);
    const double pelvis_pitch = std::atan2(pelvis_r[2], pelvis_r[0]);
    torso_q += spindle::wrap_angle(frame_pitch - pelvis_pitch);
    const auto torso_r_actual = body_mat(model_, data, torso_body_);
    std::vector<std::pair<std::string, double>> targets;
    targets.emplace_back("rider_torso_hinge", torso_q);
    bool arm_saturated = false;
    // Welded hands close the pelvis-torso-arm chain: arm goals solved
    // about the actual torso would always be "already met" and give the
    // trunk no support. Solve about the shoulder the *target* torso angle
    // would place; a sagging trunk then meets elbow stiffness.
    const auto torso_entry = std::ranges::find_if(
        joints_, [](const JointEntry &j) {
            return j.name == "rider_torso_hinge";
        });
    double delta = torso_q -
        model_access::readonly_buffer(data->qpos, model_->nq,
                                      "qpos")[static_cast<std::size_t>(
            torso_entry->qpos_adr)];
    delta = std::atan2(std::sin(delta), std::cos(delta));
    const double cd = std::cos(delta), sd = std::sin(delta);
    const std::array<double, 9> rot{cd, 0., sd, 0., 1., 0., -sd, 0., cd};
    const Vec3 hinge = joint_anchor(model_, data, torso_joint_);
    const auto torso_r = mat_mul(rot, torso_r_actual);
    std::array<Vec3, 2> shoulders{};
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        shoulders[s] =
            vec_add(hinge,
                    mat_vec(rot,
                            vec_sub(body_pos(model_, data,
                                             upper_arm_bodies_[s]),
                                    hinge)));
    }
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        const Vec3 diff = vec_sub(grips[s], shoulders[s]);
        const Vec2 arm_target = mat_t_vec_xz(torso_r, diff);
        const auto ik = spindle::two_link_ik(
            arm_target, arm_upper_m_, arm_lower_m_, arm_branch_);
        arm_saturated = arm_saturated || ik.saturated;
        const auto stem = std::string(side);
        targets.emplace_back("rider_shoulder_" + stem,
                             arm_upper_angle0_ - ik.angles[0]);
        targets.emplace_back("rider_elbow_" + stem,
                             (arm_lower_angle0_ - arm_upper_angle0_) -
                                 ik.angles[1]);
    }
    // saturated_ik.update(torso=..., arms=...) — insertion order.
    named_set(saturated_ik_, "torso", neutral_torso_saturated_);
    named_set(saturated_ik_, "arms", arm_saturated);
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    for (auto &[name, target] : targets) {
        const auto entry = std::ranges::find_if(
            joints_, [&](const JointEntry &j) { return j.name == name; });
        const double current =
            qpos[static_cast<std::size_t>(entry->qpos_adr)];
        target = current + std::atan2(std::sin(target - current),
                                      std::cos(target - current));
        const auto range = joint_ranges_.find(name);
        if (range != joint_ranges_.end()) {
            const auto [lo, hi] = range->second;
            const double limited =
                target > hi ? hi : target < lo ? lo : target;
            const char *key =
                name == "rider_torso_hinge" ? "torso" : "arms";
            // Python's |= on an existing dict key — named_at is the
            // KeyError-equivalent read.
            bool &flag = named_at(saturated_ik_, key);
            flag = flag || limited != target;
            target = limited;
        }
    }
    return targets;
}

void SpindleController::predict_target_state(const mjData *data,
                                             mjData *scratch,
                                             bool reverse) const {
    const auto qpos = model_access::mutable_buffer(
        scratch->qpos, model_->nq, "qpos");
    std::ranges::copy(
        model_access::readonly_buffer(data->qpos, model_->nq, "qpos"),
        qpos.begin());
    const auto qvel = model_access::mutable_buffer(
        scratch->qvel, model_->nv, "qvel");
    std::ranges::fill(qvel, 0.);
    const double rate = model_access::readonly_buffer(
        data->qvel, model_->nv,
        "qvel")[static_cast<std::size_t>(crank_spin_dof_)];
    qvel[static_cast<std::size_t>(crank_spin_dof_)] = rate;
    for (const int dof : pedal_spin_dofs_)
        qvel[static_cast<std::size_t>(dof)] = -rate;
    engine::integrate_pos(
        model_, scratch->qpos, scratch->qvel,
        reverse ? -target_difference_s_ : target_difference_s_);
    engine::kinematics(model_, scratch);
}

void SpindleController::initialize_velocity(mjData *data) {
    predict_target_state(data, target_data_.get(), false);
    predict_target_state(data, previous_target_data_.get(), true);
    for (const auto side : kSides) {
        const auto s = side_index(side);
        const Vec2 next =
            leg_targets(target_data_.get(), s == 0 ? "front" : "rear");
        const Vec2 previous = leg_targets(
            previous_target_data_.get(), s == 0 ? "front" : "rear");
        // zip() truncates to the shorter iterable: only the first
        // min(leg joints, 2) leg DOFs receive a matched rate.
        const auto count =
            std::min(leg_joint_index_[s].size(), static_cast<std::size_t>(2));
        const auto qvel = model_access::mutable_buffer(
            data->qvel, model_->nv, "qvel");
        for (std::size_t k = 0; k < count; ++k) {
            const double velocity =
                spindle::wrap_angle(next[k] - previous[k]) /
                (2. * target_difference_s_);
            qvel[static_cast<std::size_t>(
                joints_[leg_joint_index_[s][k]].dof_adr)] = velocity;
        }
    }
}

void SpindleController::initialize(mjData *data) {
    engine::forward(model_, data);
    reset_activation();
    const auto qpos = model_access::mutable_buffer(
        data->qpos, model_->nq, "qpos");
    // Pose the torso first, then solve the arms about that actual shoulder.
    auto upper = upper_targets(data, nullptr);
    const auto torso_entry = std::ranges::find_if(
        joints_, [](const JointEntry &j) {
            return j.name == "rider_torso_hinge";
        });
    qpos[static_cast<std::size_t>(torso_entry->qpos_adr)] =
        upper.front().second;
    engine::forward(model_, data);
    upper = upper_targets(data, nullptr);
    for (const auto &[name, value] : upper) {
        const auto entry = std::ranges::find_if(
            joints_, [&](const JointEntry &j) { return j.name == name; });
        qpos[static_cast<std::size_t>(entry->qpos_adr)] = value;
    }
    engine::forward(model_, data);
    for (const auto side : kSides) {
        const auto s = side_index(side);
        const Vec2 targets =
            leg_targets(data, s == 0 ? "front" : "rear");
        if (named_at(ik_reach_limited_, std::string(side)))
            throw std::invalid_argument(
                std::string("initial ") + std::string(side) +
                " pedal is unreachable");
        const auto count =
            std::min(leg_joint_index_[s].size(), static_cast<std::size_t>(2));
        for (std::size_t k = 0; k < count; ++k)
            qpos[static_cast<std::size_t>(
                joints_[leg_joint_index_[s][k]].qpos_adr)] = targets[k];
    }
    engine::forward(model_, data);
}

std::pair<std::vector<double>, double>
SpindleController::envelope_forces(const mjData *data) const {
    std::vector<double> force(static_cast<std::size_t>(model_->nv), 0.);
    if (envelope_qpos_adrs_.empty())
        return {force, 0.};
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    std::vector<double> q(envelope_qpos_adrs_.size());
    for (std::size_t i = 0; i < q.size(); ++i)
        q[i] = qpos[static_cast<std::size_t>(envelope_qpos_adrs_[i])];
    const auto rows = spindle::soft_edge_response(
        q, envelope_lower_, envelope_upper_,
        config_.joint_envelope_soft_k_nm_rad,
        config_.joint_envelope_soft_margin_rad);
    double stored = 0.;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        force[static_cast<std::size_t>(envelope_dof_adrs_[i])] =
            rows[i].torque;
        stored += rows[i].stored_j;
    }
    return {force, stored};
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror rider_control.py
double SpindleController::strength_capacity(const std::string &name,
                                            double angle_rad,
                                            double velocity_rad_s,
                                            double torque_nm) const {
    if (!config_.has_strength || torque_nm == 0.)
        return std::numeric_limits<double>::infinity();
    const int direction = torque_nm > 0. ? 1 : -1;
    const auto found = config_.strength.find(name);
    if (found == config_.strength.end())
        throw std::out_of_range(name);
    const spindle::TorqueCurve &curve =
        direction > 0 ? found->second.positive : found->second.negative;
    const double angle =
        std::min(std::max(angle_rad, curve.angles_rad.front()),
                 curve.angles_rad.back());
    return spindle::directional_capacity(curve, angle, velocity_rad_s,
                                         direction);
}
// NOLINTEND(bugprone-easily-swappable-parameters)

double SpindleController::anatomical_joint_angle(const std::string &name,
                                                 double qpos_value) const {
    const auto found = config_.strength_coordinates.find(name);
    if (found == config_.strength_coordinates.end())
        throw std::out_of_range(name);
    return spindle::anatomical_angle(found->second, qpos_value);
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror rider_control.py
std::pair<std::vector<double>, std::vector<std::string>>
SpindleController::strength_limited(std::span<const double> torques,
                                    std::span<const double> qpos,
                                    std::span<const double> qvel) const {
    std::vector<double> clipped(torques.begin(), torques.end());
    if (!config_.has_strength)
        return {clipped, {}};
    if (clipped.size() != joints_.size())
        throw std::invalid_argument("strength limit vector mismatch");
    std::vector<std::string> limited;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto &joint = joints_[i];
        const double capacity = strength_capacity(
            joint.name,
            anatomical_joint_angle(
                joint.name, qpos[static_cast<std::size_t>(joint.qpos_adr)]),
            qvel[static_cast<std::size_t>(joint.dof_adr)], clipped[i]);
        if (std::abs(clipped[i]) > capacity) {
            clipped[i] = std::copysign(capacity, clipped[i]);
            limited.push_back(joint.name);
        }
    }
    return {clipped, limited};
}
// NOLINTEND(bugprone-easily-swappable-parameters)

std::vector<std::string> SpindleController::strength_violations(
    const NamedEntries<double> &active_torques,
    std::span<const double> qpos, std::span<const double> qvel) const {
    if (!config_.has_strength)
        return {};
    std::vector<std::string> violated;
    for (const auto &joint : joints_) {
        const double torque =
            named_get(active_torques, joint.name, 0.);
        const double capacity = strength_capacity(
            joint.name,
            anatomical_joint_angle(
                joint.name, qpos[static_cast<std::size_t>(joint.qpos_adr)]),
            qvel[static_cast<std::size_t>(joint.dof_adr)], torque);
        if (std::abs(torque) > capacity * (1. + 1e-6) + 1e-9)
            violated.push_back(joint.name);
    }
    return violated;
}

double SpindleController::directional_limit(const std::string &name,
                                            const mjData *data,
                                            double sign) const {
    const auto entry = std::ranges::find_if(
        joints_, [&](const JointEntry &j) { return j.name == name; });
    if (entry == joints_.end())
        throw std::out_of_range(name);
    if (!config_.has_strength)
        return config_.joint_limit_nm;
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    return strength_capacity(
        name,
        anatomical_joint_angle(
            name, qpos[static_cast<std::size_t>(entry->qpos_adr)]),
        qvel[static_cast<std::size_t>(entry->dof_adr)], sign);
}

std::vector<double>
SpindleController::limit_torques(std::span<const double> torques,
                                 const mjData *data) const {
    if (torques.size() != joints_.size())
        throw std::invalid_argument("effort state shape mismatch");
    const std::size_t n = joints_.size();
    std::vector<double> limited(n), speeds(n);
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    for (std::size_t i = 0; i < n; ++i) {
        const auto &joint = joints_[i];
        const double torque = validation::finite(
            torques[i], (joint.name + " requested torque").c_str());
        const double speed = validation::finite(
            qvel[static_cast<std::size_t>(joint.dof_adr)],
            (joint.name + " speed").c_str());
        const double cap =
            directional_limit(joint.name, data, torque >= 0. ? 1. : -1.);
        double value = std::min(std::max(torque, -cap), cap);
        if (std::abs(speed) >= config_.joint_speed_limit_rad_s &&
            value * speed > 0.)
            value = 0.;
        limited[i] = value;
        speeds[i] = speed;
    }
    // No configured whole-body ceiling means only the per-joint ceiling.
    const double total = config_.active_positive_power_limit_w.value_or(
        config_.joint_power_limit_w * static_cast<double>(n));
    return spindle::scale_to_power_budget(limited, speeds,
                                          config_.joint_power_limit_w,
                                          total);
}

std::vector<double>
SpindleController::finalize_effort(mjData *data,
                                   std::span<const double> torques,
                                   bool advance, double dt_s,
                                   bool steady_state) {
    const std::size_t n = joints_.size();
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    std::vector<double> speeds(n), passive(n), target(torques.begin(),
                                                      torques.end());
    for (std::size_t i = 0; i < n; ++i) {
        speeds[i] = qvel[static_cast<std::size_t>(joints_[i].dof_adr)];
        passive[i] = -config_.passive_damping_nms_rad() * speeds[i];
    }
    const bool enabled = config_.activation_tau_s > 0. ||
                         config_.active_positive_power_limit_w.has_value() ||
                         config_.has_strength;
    std::vector<double> excitation(target), result(target);
    std::vector<std::string> limited_names;
    if (enabled) {
        if (advance && !steady_state && config_.activation_tau_s > 0. &&
            activation_time_s_.has_value() &&
            data->time <= *activation_time_s_)
            throw std::invalid_argument(
                "rider activation advances once per timestamp");
        if (allocation_diagnostics_.present) {
            excitation = allocation_diagnostics_.solution_excitation_nm;
        } else {
            // Standalone callers must still obey bounded excitation.
            const double gain =
                steady_state || config_.activation_tau_s == 0.
                    ? 1.
                    : -std::expm1(-dt_s / config_.activation_tau_s);
            for (std::size_t i = 0; i < n; ++i)
                excitation[i] =
                    (target[i] - (1. - gain) * active_state_[i]) / gain;
            excitation = spindle::bounded_effort(
                speeds, excitation, config_.joint_limit_nm,
                config_.joint_speed_limit_rad_s,
                config_.joint_power_limit_w);
            excitation = strength_limited(excitation, qpos, qvel).first;
        }
        const std::vector<double> activated =
            steady_state || !command_enabled_
                ? excitation
                : spindle::activation_step(active_state_, excitation, dt_s,
                                           config_.activation_tau_s);
        result = limit_torques(activated, data);
        limited_names = strength_limited(activated, qpos, qvel).second;
        if (config_.active_positive_power_limit_w.has_value())
            result = spindle::limit_positive_power(
                result, speeds, *config_.active_positive_power_limit_w);
        for (std::size_t i = 0; i < n; ++i)
            if (!isclose(result[i], target[i], 1e-9, 1e-7))
                throw std::runtime_error(
                    "allocated rider torque is outside final actuator "
                    "limits");
        if (advance) {
            active_state_ = result;
            activation_time_s_ =
                steady_state || config_.activation_tau_s == 0.
                    ? std::nullopt
                    : std::optional<double>(data->time);
        }
    }
    double positive_power = 0., passive_power = 0.;
    for (std::size_t i = 0; i < n; ++i) {
        positive_power += std::max(result[i] * speeds[i], 0.);
        passive_power += passive[i] * speeds[i];
    }
    bool saturated = false;
    for (std::size_t i = 0; i < n && !saturated; ++i)
        saturated = !isclose(result[i], excitation[i], 1e-10, 1e-10);
    effort_diagnostics_ = {};
    effort_diagnostics_.present = true;
    for (std::size_t i = 0; i < n; ++i) {
        effort_diagnostics_.active_request_nm.emplace_back(joints_[i].name,
                                                         excitation[i]);
        effort_diagnostics_.active_delivered_nm.emplace_back(
            joints_[i].name, result[i]);
    }
    effort_diagnostics_.positive_power_w = positive_power;
    effort_diagnostics_.passive_power_w = passive_power;
    effort_diagnostics_.activation_saturated = saturated;
    effort_diagnostics_.strength_limited = limited_names;
    effort_diagnostics_.budget_exceeded = false;
    effort_diagnostics_.positive_power_limit_w =
        config_.active_positive_power_limit_w;
    effort_diagnostics_.observation = "incoming_request";
    for (std::size_t i = 0; i < n; ++i) {
        auto &terms = named_at(last_terms_, joints_[i].name);
        terms.active_request_nm = excitation[i];
        terms.active_delivered_nm = result[i];
        terms.passive_damping_nm = passive[i];
        terms.command_nm = result[i];
    }
    return result;
}

NamedEntries<double>
SpindleController::compute(mjData *data, const RiderCommand &command,
                           bool advance, std::optional<double> dt_s,
                           bool steady_state) {
    const double dt = dt_s.value_or(model_->opt.timestep);
    const std::size_t n = joints_.size();
    command_enabled_ = command.enabled && enabled_;
    saturated_ik_.clear();
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    const auto bias = model_access::readonly_buffer(
        data->qfrc_bias, model_->nv, "qfrc_bias");
    std::vector<double> requested(n, 0.);
    double total = 0.;
    std::array<spindle::Vec2, 2> jacobians{};
    std::array<bool, 2> has_jacobian{false, false};
    if (!command_enabled_) {
        if (advance)
            reset_activation();
        support_diagnostics_ = {.present = true,
                                .stance_front = false,
                                .stance_rear = false};
    } else {
        const auto frame_r = body_mat(model_, data, frame_body_);
        const auto pelvis_r = body_mat(model_, data, pelvis_body_);
        const double rel02 = frame_r[0] * pelvis_r[2] +
                             frame_r[3] * pelvis_r[5] +
                             frame_r[6] * pelvis_r[8];
        const double rel00 = frame_r[0] * pelvis_r[0] +
                             frame_r[3] * pelvis_r[3] +
                             frame_r[6] * pelvis_r[6];
        const double pelvis_pitch = std::atan2(rel02, rel00);
        const Vec3 crank_world = joint_anchor(model_, data, crank_joint_);
        std::array<Vec2, 2> spindles{};
        for (const auto side : kSides) {
            const auto s = side_index(side);
            spindles[s] = mat_t_vec_xz(
                frame_r,
                vec_sub(body_pos(model_, data, pedal_bodies_[s]),
                        crank_world));
        }
        const double phase = std::atan2(-spindles[0][1], spindles[0][0]);
        const double rate = qvel[static_cast<std::size_t>(crank_spin_dof_)];
        if (command.mean_crank_torque_nm > 0.) {
            total = spindle::spindle_torque_waveform(
                command.mean_crank_torque_nm, phase,
                config_.pedal_torque_ripple);
        } else {
            // NaN must propagate like np.clip does (both branches' NaN
            // comparisons are false, so raw survives untouched).
            const double raw =
                config_.coasting_brake_d_nm_s_rad *
                (command.crank_target_rate_rad_s - rate);
            total = raw > config_.coasting_brake_limit_nm
                        ? config_.coasting_brake_limit_nm
                    : raw < -config_.coasting_brake_limit_nm
                        ? -config_.coasting_brake_limit_nm
                        : raw;
        }
        const Vec2 crank{0., 0.};
        const Vec2 leg_targets_v = spindle::leg_crank_targets(
            total, phase, spindles[0], spindles[1], crank,
            command.mean_crank_torque_nm > 0.
                ? config_.return_foot_preload_n
                : 0.);
        const auto index_of = [&](const std::string &name) {
            return static_cast<std::size_t>(
                std::ranges::find_if(
                    joints_,
                    [&](const JointEntry &j) { return j.name == name; }) -
                joints_.begin());
        };
        for (const auto side : kSides) {
            const auto s = side_index(side);
            const Vec2 hip = mat_t_vec_xz(
                frame_r,
                vec_sub(joint_anchor(model_, data, hip_joints_[s]),
                        crank_world));
            const Vec2 jac =
                spindle::leg_loop_jacobian(leg_geometry_[s], hip, crank,
                                           crank_length_m_, phase,
                                           pelvis_pitch);
            jacobians[s] = jac;
            has_jacobian[s] = true;
            const auto capacity = [&](int joint, double sign) {
                return directional_limit(
                    std::string("rider_") +
                        (joint == 0 ? "hip_" : "knee_") +
                        std::string(side),
                    data, sign);
            };
            const Vec2 torques = spindle::split_joint_torques(
                leg_targets_v[s], jac, capacity);
            requested[index_of("rider_hip_" + std::string(side))] =
                torques[0];
            requested[index_of("rider_knee_" + std::string(side))] =
                torques[1];
        }
        const auto upper = upper_targets(data, &command.posture);
        for (const auto &[name, target] : upper) {
            const auto index = index_of(name);
            const auto &joint = joints_[index];
            // Positive hinge torque applies the restoring -y reaction
            // to the pelvis.
            const double torque =
                name == "rider_torso_hinge"
                    ? config_.joint_kp_nm_rad * pelvis_pitch +
                          config_.joint_kd_nms_rad *
                              (qvel[static_cast<std::size_t>(
                                   rider_pitch_dof_)] -
                               qvel[static_cast<std::size_t>(
                                   frame_pitch_dof_)])
                    : config_.joint_kp_nm_rad *
                              spindle::wrap_angle(
                                  target -
                                  qpos[static_cast<std::size_t>(
                                      joint.qpos_adr)]) -
                          config_.joint_kd_nms_rad *
                              qvel[static_cast<std::size_t>(joint.dof_adr)];
            requested[index] =
                torque + bias[static_cast<std::size_t>(joint.dof_adr)];
        }
        const Vec2 shares = spindle::leg_shares(phase);
        support_diagnostics_ = {.present = true,
                                .stance_front = shares[0] >= .5,
                                .stance_rear = shares[1] >= .5};
    }
    const std::vector<double> excitation = limit_torques(requested, data);
    const std::vector<double> activated =
        steady_state || !command_enabled_
            ? excitation
            : spindle::activation_step(active_state_, excitation, dt,
                                       config_.activation_tau_s);
    const std::vector<double> final = limit_torques(activated, data);
    last_terms_.clear();
    for (std::size_t i = 0; i < n; ++i) {
        const auto &name = joints_[i].name;
        const bool upper_joint = !name.starts_with("rider_hip_") &&
                                 !name.starts_with("rider_knee_");
        JointTerms terms;
        terms.requested_nm = requested[i];
        terms.command_nm = final[i];
        terms.posture_nm = upper_joint ? requested[i] : 0.;
        terms.pedaling_nm = upper_joint ? 0. : requested[i];
        terms.saturated = !isclose(final[i], requested[i], 1e-9, 1e-9);
        last_terms_.emplace_back(name, terms);
    }
    // sum() over the Python generator is a flat left-to-right fold —
    // front hip, front knee, rear hip, rear knee — so the adds must not
    // regroup per side.
    double projected = 0.;
    const auto index_of = [&](const std::string &name) {
        return static_cast<std::size_t>(
            std::ranges::find_if(
                joints_,
                [&](const JointEntry &j) { return j.name == name; }) -
            joints_.begin());
    };
    for (const auto side : kSides) {
        const auto s = side_index(side);
        if (!has_jacobian[s])
            continue;
        projected += final[index_of("rider_hip_" + std::string(side))] *
                     jacobians[s][0];
        projected += final[index_of("rider_knee_" + std::string(side))] *
                     jacobians[s][1];
    }
    allocation_diagnostics_ = {.present = true,
                               .invalid_controller = false,
                               .crank_task_nm = total,
                               .crank_task_shortfall_nm = total - projected,
                               .solution_excitation_nm = excitation};
    const std::vector<double> delivered =
        finalize_effort(data, final, advance, dt, steady_state);
    joint_torques_nm_.clear();
    joint_capacity_nm_.clear();
    NamedEntries<double> out;
    for (std::size_t i = 0; i < n; ++i) {
        joint_torques_nm_.emplace_back(joints_[i].name, delivered[i]);
        joint_capacity_nm_.emplace_back(
            joints_[i].name,
            directional_limit(joints_[i].name, data,
                              delivered[i] >= 0. ? 1. : -1.));
        out.emplace_back(joints_[i].name, delivered[i]);
    }
    return out;
}

void SpindleController::write(
    mjData *data, const NamedEntries<double> &torques) const {
    if (torques.size() != joints_.size() ||
        !std::ranges::all_of(joints_, [&](const JointEntry &j) {
            return named_find(torques, j.name) != nullptr;
        }))
        throw std::invalid_argument("incomplete rider actuator command");
    const auto ctrl = model_access::mutable_buffer(
        data->ctrl, model_->nu, "ctrl");
    for (const auto &joint : joints_) {
        const double torque =
            validation::finite(named_at(torques, joint.name),
                               "rider torque");
        // The actuator is a pure motor: ctrl is the muscle torque itself.
        // Passive damping lives on the DOF, so a disabled command removes
        // only active force; tissue damping still acts physically.
        ctrl[static_cast<std::size_t>(joint.actuator_id)] =
            command_enabled_ ? torque : 0.;
    }
}

EffortDiagnostics SpindleController::solved_effort(
    const mjData *data, std::span<const double> incoming_qpos,
    std::span<const double> incoming_velocity, double dt_s) {
    NamedEntries<double> active;
    double passive_power = 0., positive = 0.;
    const auto actuator_force = model_access::readonly_buffer(
        data->actuator_force, model_->nu, "actuator_force");
    const auto qfrc_passive = model_access::readonly_buffer(
        data->qfrc_passive, model_->nv, "qfrc_passive");
    for (const auto &joint : joints_) {
        // actuator_force is pure muscle torque now; passive tissue damping
        // is the DOF damping row of qfrc_passive at the solved state.
        const double damping =
            qfrc_passive[static_cast<std::size_t>(joint.dof_adr)];
        const double delivered =
            actuator_force[static_cast<std::size_t>(joint.actuator_id)];
        active.emplace_back(joint.name, delivered);
        positive += std::max(
            delivered * incoming_velocity[static_cast<std::size_t>(
                              joint.dof_adr)],
            0.);
        passive_power +=
            damping *
            incoming_velocity[static_cast<std::size_t>(joint.dof_adr)];
        auto &terms = named_at(last_terms_, joint.name);
        terms.solved_force_nm = delivered;
        terms.solved_active_nm = delivered;
        terms.solved_passive_nm = damping;
    }
    const auto violations = strength_violations(active, incoming_qpos,
                                                incoming_velocity);
    NamedEntries<double> joint_power;
    std::vector<std::string> power_violations, speed_violations;
    for (const auto &joint : joints_) {
        const double power = std::max(
            named_at(active, joint.name) *
                incoming_velocity[static_cast<std::size_t>(joint.dof_adr)],
            0.);
        joint_power.emplace_back(joint.name, power);
        if (power > config_.joint_power_limit_w + 1e-9)
            power_violations.push_back(joint.name);
        if (std::abs(incoming_velocity[static_cast<std::size_t>(
                joint.dof_adr)]) >
            config_.joint_speed_limit_rad_s + 1e-9)
            speed_violations.push_back(joint.name);
    }
    effort_diagnostics_.joint_positive_power_w = joint_power;
    effort_diagnostics_.joint_power_violations = power_violations;
    effort_diagnostics_.joint_speed_violations = speed_violations;
    effort_diagnostics_.active_delivered_nm = active;
    effort_diagnostics_.positive_power_w = positive;
    effort_diagnostics_.passive_power_w = passive_power;
    effort_diagnostics_.positive_work_step_j = positive * dt_s;
    effort_diagnostics_.passive_work_step_j = passive_power * dt_s;
    effort_diagnostics_.strength_violations = violations;
    effort_diagnostics_.budget_exceeded =
        config_.active_positive_power_limit_w.has_value() &&
        positive > *config_.active_positive_power_limit_w + 1e-9;
    effort_diagnostics_.observation =
        "solved_actuator_force_at_incoming_interval";
    return effort_diagnostics_;
}

SpindleController::State SpindleController::state() const {
    State out;
    out.enabled = enabled_;
    out.command_enabled = command_enabled_;
    out.active_state = active_state_;
    out.activation_time_s = activation_time_s_;
    for (const auto &joint : joints_)
        if (const auto *found = named_find(last_terms_, joint.name);
            found != nullptr)
            out.last_terms.emplace_back(joint.name, *found);
    out.saturated_ik = saturated_ik_;
    out.ik_reach_limited = ik_reach_limited_;
    out.joint_torques_nm = joint_torques_nm_;
    out.joint_capacity_nm = joint_capacity_nm_;
    out.lean_limit_rad = lean_limit_rad_;
    out.pedal_recovery = pedal_recovery_;
    out.effort_diagnostics = effort_diagnostics_;
    out.allocation_diagnostics = allocation_diagnostics_;
    out.support_diagnostics = support_diagnostics_;
    out.sole_goal_diagnostics = sole_goal_diagnostics_;
    return out;
}

void SpindleController::restore(const State &state) {
    if (state.active_state.size() != joints_.size())
        throw std::invalid_argument(
            "rider_controller.active_state: incorrect sequence width");
    enabled_ = state.enabled;
    command_enabled_ = state.command_enabled;
    active_state_ = state.active_state;
    activation_time_s_ = state.activation_time_s;
    last_terms_.clear();
    for (const auto &[name, terms] : state.last_terms)
        last_terms_.emplace_back(name, terms);
    saturated_ik_ = state.saturated_ik;
    ik_reach_limited_ = state.ik_reach_limited;
    joint_torques_nm_ = state.joint_torques_nm;
    joint_capacity_nm_ = state.joint_capacity_nm;
    lean_limit_rad_ = state.lean_limit_rad;
    pedal_recovery_ = state.pedal_recovery;
    effort_diagnostics_ = state.effort_diagnostics;
    allocation_diagnostics_ = state.allocation_diagnostics;
    support_diagnostics_ = state.support_diagnostics;
    sole_goal_diagnostics_ = state.sole_goal_diagnostics;
}

} // namespace rider
