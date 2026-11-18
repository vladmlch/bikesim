// rider/spindle_controller.cpp — ArticulatedRiderController's spindle
// spindle branch, with the non-spindle delegate in legacy_controller.cpp.
//
// Deviations the plan records on purpose:
//  * kinematic_state is an observational mirror, not an allocator input.
//    PD components, warm starts, activation and recovery are owned state;
//    non-spindle numerical work declares its own numeric identity.
//  * Python's ArithmeticError for a violated effort bound is
//    std::runtime_error here; KeyError for missing last_terms rows is
//    std::out_of_range.
#include "spindle_controller.hpp"
#include "legacy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "../cblas_abi.hpp"
#include "../diag.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "spindle_linalg.hpp"
#include "attachment_wrench.hpp"

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
constexpr std::array<std::string_view, 13> kReferenceJoints3d{
    "rider_torso_hinge",
    "rider_shoulder_abduction_left", "rider_shoulder_left", "rider_elbow_left",
    "rider_shoulder_abduction_right", "rider_shoulder_right", "rider_elbow_right",
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

[[nodiscard]] Vec3 joint_axis(const mjModel *m, const mjData *d,
                              int joint) {
    const auto a = row3(d->xaxis, m->njnt * 3, joint, "xaxis");
    return {a[0], a[1], a[2]};
}

[[nodiscard]] Vec3 cross3(const Vec3 &a, const Vec3 &b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

[[nodiscard]] double dot3(const Vec3 &a, const Vec3 &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

[[nodiscard]] double norm3_sq(const Vec3 &v) {
    return dot3(v, v);
}

[[nodiscard]] bool solve_3x3(const std::array<double, 9> &mat, bool transpose,
                             const Vec3 &rhs, Vec3 &out) {
    std::array<std::array<double, 4>, 3> aug{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j)
            aug[i][j] = transpose ? mat[3 * j + i] : mat[3 * i + j];
        aug[i][3] = rhs[i];
    }
    for (std::size_t col = 0; col < 3; ++col) {
        std::size_t pivot = col;
        double best = std::abs(aug[col][col]);
        for (std::size_t r = col + 1; r < 3; ++r) {
            const double val = std::abs(aug[r][col]);
            if (val > best) {
                best = val;
                pivot = r;
            }
        }
        if (best <= 1e-10) return false;
        if (pivot != col) std::swap(aug[col], aug[pivot]);
        const double inv = 1. / aug[col][col];
        for (std::size_t j = col; j < 4; ++j) aug[col][j] *= inv;
        for (std::size_t r = 0; r < 3; ++r) {
            if (r == col) continue;
            const double factor = aug[r][col];
            for (std::size_t j = col; j < 4; ++j)
                aug[r][j] -= factor * aug[col][j];
        }
    }
    out = {aug[0][3], aug[1][3], aug[2][3]};
    return true;
}

[[nodiscard]] Vec3 body_point_vel(const Mat3 &r, const Vec3 &p0,
                                  const std::array<double, 6> &v_free,
                                  const Vec3 &p) {
    const Vec3 omega_w = mat_vec(r, {v_free[3], v_free[4], v_free[5]});
    const Vec3 r_vec{p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]};
    const Vec3 wxr = cross3(omega_w, r_vec);
    return {v_free[0] + wxr[0], v_free[1] + wxr[1], v_free[2] + wxr[2]};
}

[[nodiscard]] Vec3 body_point_acc(const Mat3 &r, const Vec3 &p0,
                                  const std::array<double, 6> &v_free,
                                  const std::array<double, 6> &a_free,
                                  const Vec3 &p) {
    const Vec3 omega_w = mat_vec(r, {v_free[3], v_free[4], v_free[5]});
    const Vec3 alpha_w = mat_vec(r, {a_free[3], a_free[4], a_free[5]});
    const Vec3 r_vec{p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]};
    const Vec3 axr = cross3(alpha_w, r_vec);
    const Vec3 wxwxr = cross3(omega_w, cross3(omega_w, r_vec));
    return {a_free[0] + axr[0] + wxwxr[0],
            a_free[1] + axr[1] + wxwxr[1],
            a_free[2] + axr[2] + wxwxr[2]};
}

[[nodiscard]] Vec3 vec_sub(const Vec3 &a, const Vec3 &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

[[nodiscard]] Vec3 vec_add(const Vec3 &a, const Vec3 &b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
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

void validate_spatial_pedal_boxes(const mjModel *m, const mjData *d,
                                  std::span<const int> geoms) {
    const auto geom_types =
        model_access::readonly_buffer(m->geom_type, m->ngeom);
    for (const int geom : geoms) {
        model_access::require_id(geom, m->ngeom, "pedal geom");
        if (geom_types[static_cast<std::size_t>(geom)] != mjGEOM_BOX)
            throw std::invalid_argument("rider support geometry must be a box");
        const auto origin = row3(d->geom_xpos, m->ngeom * 3, geom, "geom_xpos");
        const auto r = row9(d->geom_xmat, m->ngeom * 9, geom, "geom_xmat");
        const auto half = row3(m->geom_size, m->ngeom * 3, geom, "geom_size");
        for (const double v : origin)
            if (!std::isfinite(v))
                throw std::invalid_argument("invalid box geometry");
        for (const double v : r)
            if (!std::isfinite(v))
                throw std::invalid_argument("invalid box geometry");
        for (const double h : half)
            if (!std::isfinite(h) || h <= 0.)
                throw std::invalid_argument("box dimensions must be positive");
    }
}

void synthesize_shoulder_abduction_defaults(SpindleConfig &cfg,
                                            double rider_mass_kg) {
    constexpr double kPi4 = std::numbers::pi / 4.;
    constexpr double kPi8 = std::numbers::pi / 8.;
    const double scale = rider_mass_kg > 0. ? rider_mass_kg / 80. : 1.;
    const auto curve_at = [scale](double angle) {
        const double ratio = angle / 0.85;
        return 65.0 * scale * std::exp(-0.5 * ratio * ratio);
    };
    const spindle::JointEnvelope abd_env{
        .neutral_anatomical_rad = 0.,
        .direction = 1,
        .minimum_anatomical_rad = -kPi4,
        .maximum_anatomical_rad = kPi4,
        .provenance = "3D shoulder horizontal abduction"};
    const spindle::TorqueCurve abd_curve{
        .angles_rad = {-kPi4, -kPi8, 0., kPi8, kPi4},
        .torques_nm = {curve_at(-kPi4), curve_at(-kPi8), curve_at(0.),
                       curve_at(kPi8), curve_at(kPi4)},
        .vmax_rad_s = 8.0,
        .hill_c = 0.25,
        .eccentric_ratio = 1.4,
        .source = "3D shoulder horizontal abduction (Chaffin/Stobbe)"};
    for (const char *name :
         {"rider_shoulder_abduction_left", "rider_shoulder_abduction_right"}) {
        const std::string key(name);
        if (!cfg.envelopes.empty() && !cfg.envelopes.contains(key))
            cfg.envelopes.emplace(key, abd_env);
        if (cfg.has_strength) {
            if (!cfg.strength_coordinates.contains(key))
                cfg.strength_coordinates.emplace(key, abd_env);
            if (!cfg.strength.contains(key))
                cfg.strength.emplace(
                    key, DirectionalStrength{.positive = abd_curve,
                                             .negative = abd_curve});
        }
    }
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
    // np.linalg.norm(2-vec) — the production 1-D norm mirror
    // (sqrt(v.dot(v))).
    const double arm_upper = np_norm(upper);
    const double arm_lower = np_norm(lower);
    const double upper0 = std::atan2(upper[1], upper[0]);
    const double lower0 = std::atan2(lower[1], lower[0]);
    const int branch = numeric::math_sin(lower0 - upper0) >= 0. ? 1 : -1;
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
        const double c = numeric::math_cos(lean), s = numeric::math_sin(lean);
        // shoulder = hip + rotation @ trunk through NumPy's C-order
        // (2,2) row matvec dispatch.
        const Mat2 rotation{c, s, -s, c};
        const Vec2 mv = mat_vec2(rotation, trunk);
        const Vec2 shoulder{hip[0] + mv[0], hip[1] + mv[1]};
        const Vec2 goal{grip_xz[0] - shoulder[0],
                        grip_xz[1] - shoulder[1]};
        // arm_goal = rotation.T @ goal.
        const Vec2 arm_goal = mat_t_vec2(rotation, goal);
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
    // The reposition law's gain and cap are profile-owned (2026-11-16
    // plan task 2): an assembly path that forgets to inject them fails
    // here loudly instead of damping with a zero gain.
    validation::positive(config_.reposition_damping_nms_rad,
                         "spindle reposition damping gain");
    validation::positive(config_.reposition_torque_limit_nm,
                         "spindle reposition torque limit");
    spatial_3d_ = (model->nq != model->nv) &&
                  mj_name2id(model, mjOBJ_JOINT, "root_free") >= 0;
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
    if (spatial_3d_)
        synthesize_shoulder_abduction_defaults(config_, rider_mass_);
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
    const auto add_reference_joint = [&](std::string_view name) {
        const int joint =
            mj_name2id(model, mjOBJ_JOINT, std::string(name).c_str());
        if (joint < 0) {
            locked_joints_.emplace_back(name);
            return;
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
    };
    if (spatial_3d_) {
        for (const auto name : kReferenceJoints3d) add_reference_joint(name);
    } else {
        for (const auto name : kReferenceJoints) add_reference_joint(name);
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
    resolve_model_layout(qpos_adrs, dof_adrs);
    if (spatial_3d_) {
        saddle_eq_ = mj_name2id(model, mjOBJ_EQUALITY, "weld_saddle");
        const int sad_car = mj_name2id(model, mjOBJ_BODY, "rider_saddle_carrier");
        saddle_body_ = sad_car >= 0 ? sad_car : pelvis_body_;
        saddle_site_ = mj_name2id(model, mjOBJ_SITE, "site_rider_saddle");
        saddle_eq_reactions_.emplace(model);
        saddle_lstsq_.emplace(model->nv, 6, 1);
        saddle_qfrc_.assign(static_cast<std::size_t>(model->nv), 0.);
        for (const int abd_jnt : hip_abd_joints_) {
            if (abd_jnt < 0) continue;
            const auto j = static_cast<std::size_t>(abd_jnt);
            if (!joint_limited[j]) continue;
            const auto base = j * 2;
            envelope_qpos_adrs_.push_back(qpos_adrs[j]);
            envelope_dof_adrs_.push_back(dof_adrs[j]);
            envelope_lower_.push_back(joint_range[base]);
            envelope_upper_.push_back(joint_range[base + 1]);
        }
    }
    if (config_.legacy && config_.legacy->pedal_attachment != "spindle")
        legacy_ = std::make_unique<bike::rider::LegacyController>(*this);
}

void SpindleController::resolve_model_layout(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (qpos, dof) address arrays mirror the model layout
    std::span<const int> qpos_adrs, std::span<const int> dof_adrs) {
    const mjModel *model = model_;
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
    if (spatial_3d_) {
        const int frame_free = require_joint(model, "root_free");
        const int rider_free = require_joint(model, "rider_root_free");
        frame_free_dof_ = dof_adrs[static_cast<std::size_t>(frame_free)];
        rider_free_dof_ = dof_adrs[static_cast<std::size_t>(rider_free)];
        saddle_pitch_joint_ = require_joint(model, "rider_saddle_pitch");
        saddle_pitch_dof_ =
            dof_adrs[static_cast<std::size_t>(saddle_pitch_joint_)];
        steer_joint_ = require_joint(model, "steer_joint");
        hip_abd_joints_[0] =
            require_joint(model, "rider_hip_abduction_front");
        hip_abd_joints_[1] = require_joint(model, "rider_hip_abduction_rear");
        hip_abd_dofs_[0] =
            dof_adrs[static_cast<std::size_t>(hip_abd_joints_[0])];
        hip_abd_dofs_[1] =
            dof_adrs[static_cast<std::size_t>(hip_abd_joints_[1])];
        for (const auto side : kArmSides) {
            const auto s = side_index(side);
            const auto stem = std::string(side);
            shoulder_abd_joints_[s] = require_joint(
                model, ("rider_shoulder_abduction_" + stem).c_str());
            shoulder_joints_[s] =
                require_joint(model, ("rider_shoulder_" + stem).c_str());
            elbow_joints_[s] =
                require_joint(model, ("rider_elbow_" + stem).c_str());
        }
    } else {
        frame_pitch_joint_ = require_joint(model, "root_pitch");
        rider_pitch_joint_ = require_joint(model, "rider_root_pitch");
        const int frame_x_jnt = mj_name2id(model, mjOBJ_JOINT, "root_x");
        const int frame_z_jnt = mj_name2id(model, mjOBJ_JOINT, "root_z");
        frame_root_x_dof_ =
            frame_x_jnt < 0
                ? -1
                : dof_adrs[static_cast<std::size_t>(frame_x_jnt)];
        frame_root_z_dof_ =
            frame_z_jnt < 0
                ? -1
                : dof_adrs[static_cast<std::size_t>(frame_z_jnt)];
        frame_pitch_dof_ =
            dof_adrs[static_cast<std::size_t>(frame_pitch_joint_)];
        const int rider_x_jnt = mj_name2id(model, mjOBJ_JOINT, "rider_root_x");
        const int rider_z_jnt = mj_name2id(model, mjOBJ_JOINT, "rider_root_z");
        rider_root_x_dof_ =
            rider_x_jnt < 0
                ? -1
                : dof_adrs[static_cast<std::size_t>(rider_x_jnt)];
        rider_root_z_dof_ =
            rider_z_jnt < 0
                ? -1
                : dof_adrs[static_cast<std::size_t>(rider_z_jnt)];
        rider_pitch_dof_ =
            dof_adrs[static_cast<std::size_t>(rider_pitch_joint_)];
    }
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
        if (eq < 0 && (!config_.legacy || config_.legacy->grip_attachment == "connect"))
            throw std::invalid_argument(
                "missing compiled grip connect equality");
        if (eq < 0) {
            const auto s = side_index(side);
            const auto origin = body_pos(model, target_data_.get(), steer_body_);
            const auto rotation = body_mat(model, target_data_.get(), steer_body_);
            const auto point = site_pos(model, target_data_.get(), grip_sites_[s]);
            const auto delta = vec_sub(point, origin);
            for (std::size_t j = 0; j < 3; ++j)
                weld_grip_offset_[s][j] = rotation[j]*delta[0]+rotation[3+j]*delta[1]+rotation[6+j]*delta[2];
            continue;
        }
        const auto base = static_cast<std::size_t>(eq) * mjNEQDATA;
        const auto eq_data = model_access::readonly_buffer(
            model->eq_data, model->neq * mjNEQDATA, "eq_data");
        std::ranges::copy(eq_data.subspan(base + 3, 3).first<3>(),
                          weld_grip_offset_[side_index(side)].begin());
    }
    if (spatial_3d_)
        validate_spatial_pedal_boxes(model, target_data_.get(),
                                     std::span{pedal_geoms_});
    else
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
    if (spatial_3d_) {
        std::array<Vec3, 2> sh0{};
        std::array<Vec3, 2> gr0{};
        for (const std::size_t s : {0U, 1U}) {
            sh0[s] = body_pos(model, target_data_.get(), upper_arm_bodies_[s]);
            const Vec3 el0 =
                body_pos(model, target_data_.get(), forearm_bodies_[s]);
            gr0[s] = site_pos(model, target_data_.get(), grip_sites_[s]);
            const Vec3 u0 = vec_sub(el0, sh0[s]);
            const Vec3 l0 = vec_sub(gr0[s], el0);
            const Vec3 dp0 = vec_sub(gr0[s], sh0[s]);
            arm3d_upper_m_[s] = np_norm(u0);
            arm3d_lower_m_[s] = np_norm(l0);
            arm3d_psi0_[s] = std::atan2(dp0[1], dp0[0]);
            const double cos_el0 = std::clamp(
                dot3(u0, l0) / (arm3d_upper_m_[s] * arm3d_lower_m_[s]),
                -1., 1.);
            arm3d_elbow0_[s] = std::acos(cos_el0);
            arm3d_beta_ua0_[s] =
                std::atan2(-u0[2], std::hypot(u0[0], u0[1]));
        }
        arm_upper_m_ = arm3d_upper_m_[0];
        arm_lower_m_ = arm3d_lower_m_[0];
        neutral_torso_saturated_ = false;
        neutral_torso_q_ = 0.;
        if (!config_.envelopes.empty() &&
            (!config_.legacy || config_.legacy->pedal_attachment == "spindle")) {
            const Vec2 hip_xz = xz(pose_.hip);
            const Vec2 crank_xz{
                (pose_.pedal_front[0] + pose_.pedal_rear[0]) / 2.,
                (pose_.pedal_front[2] + pose_.pedal_rear[2]) / 2.};
            std::array<double, 2> hips_at_tdc{};
            for (const std::size_t s : {0U, 1U}) {
                const double tdc_phase =
                    -std::numbers::pi / 2. + (s == 0 ? 0. : std::numbers::pi);
                const Vec2 spindle_pos = spindle::spindle_xz(
                    crank_xz, crank_length_m_, tdc_phase,
                    leg_geometry_[s].side_sign);
                hips_at_tdc[s] = spindle::leg_joint_q(
                    leg_geometry_[s], hip_xz, spindle_pos, 0.)[0];
            }
            const auto within = [&](const std::string &name, double q) {
                const auto entry = config_.envelopes.find(name);
                if (entry == config_.envelopes.end())
                    throw std::out_of_range(name);
                const Vec2 range = spindle::joint_q_range(entry->second);
                return range[0] - 1e-12 <= q && q <= range[1] + 1e-12;
            };
            const Vec3 hinge =
                joint_anchor(model, target_data_.get(), torso_joint_);
            const std::array<Vec3, 2> sh_local{
                vec_sub(sh0[0], hinge), vec_sub(sh0[1], hinge)};
            const auto feasible_3d = [&](double lean) {
                if (!within("rider_torso_hinge", lean)) return false;
                const double c = numeric::math_cos(lean);
                const double sn = numeric::math_sin(lean);
                const Mat3 torso_r{c, 0., sn, 0., 1., 0., -sn, 0., c};
                for (const auto side : kArmSides) {
                    const auto s = side_index(side);
                    const Vec3 sh_w = vec_add(hinge, mat_vec(torso_r, sh_local[s]));
                    const auto ik = solve_arm_ik_3d(s, torso_r, sh_w, gr0[s]);
                    if (ik.saturated) return false;
                    const auto stem = std::string(side);
                    if (!within("rider_shoulder_abduction_" + stem, ik.q_abd) ||
                        !within("rider_shoulder_" + stem, ik.q_sh) ||
                        !within("rider_elbow_" + stem, ik.q_el))
                        return false;
                }
                return within("rider_hip_front", hips_at_tdc[0] - lean) &&
                       within("rider_hip_rear", hips_at_tdc[1] - lean);
            };
            if (!feasible_3d(0.))
                throw std::invalid_argument(
                    "neutral seated posture lies outside reachable joint ROM");
            if (feasible_3d(0.8)) {
                lean_limit_rad_ = 0.8;
            } else {
                double low = 0., high = 0.8;
                while (high - low > 1e-3) {
                    const double middle = 0.5 * (low + high);
                    if (feasible_3d(middle)) low = middle;
                    else high = middle;
                }
                lean_limit_rad_ = low;
            }
        }
    } else {
        const Vec3 trunk = vec_sub(pose_.shoulder, pose_.hip);
        const Vec3 upper = vec_sub(pose_.elbow, pose_.shoulder);
        const Vec3 lower = vec_sub(pose_.grip, pose_.elbow);
        arm_upper_m_ = np_norm(upper);
        arm_lower_m_ = np_norm(lower);
        arm_upper_angle0_ = std::atan2(upper[2], upper[0]);
        arm_lower_angle0_ = std::atan2(lower[2], lower[0]);
        arm_branch_ =
            numeric::math_sin(arm_lower_angle0_ - arm_upper_angle0_) >= 0. ? 1 : -1;
        {
            const Vec2 goal{xz(vec_sub(pose_.grip, pose_.hip))};
            const double reach =
                (arm_upper_m_ + arm_lower_m_) * config_.arm_reach_fraction;
            const auto first =
                spindle::two_link_ik(goal, np_norm(trunk), reach, -1);
            const auto second =
                spindle::two_link_ik(goal, np_norm(trunk), reach, 1);
            const auto &neutral =
                numeric::math_sin(first.angles[0]) >= numeric::math_sin(second.angles[0])
                    ? first
                    : second;
            neutral_torso_saturated_ = neutral.saturated;
            neutral_torso_q_ = config_.legacy && config_.legacy->pedal_attachment != "spindle"
                ? std::atan2(trunk[2],trunk[0])-neutral.angles[0] : 0.;
        }
        if (!config_.envelopes.empty() && (!config_.legacy || config_.legacy->pedal_attachment == "spindle"))
            lean_limit_rad_ =
                rider::lean_limit_rad(pose_, config_.envelopes,
                                      crank_length_m_, -std::numbers::pi / 2.);
    }
    for (const auto &joint : joints_) {
        joint_torques_nm_.emplace_back(joint.name, 0.);
        joint_capacity_nm_.emplace_back(joint.name, 0.);
    }
}

SpindleController::~SpindleController() = default;

void SpindleController::reset_activation() {
    allocation_memory_ = {};
    coast_hold_nm_ = 0.;
    effort_diagnostics_ = {};
    sole_goal_diagnostics_.clear();
    pedal_recovery_ = {{"front", PedalRecoveryState{}},
                       {"rear", PedalRecoveryState{}}};
    has_prev_press_3d_ = false;
    prev_press_time_3d_ = -1.;
    prev_f_passive_3d_ = {0., 0.};
    prev_max_press_3d_ = 0.;
}

spindle::Vec2 SpindleController::leg_targets(const mjData *data,
                                             const char *side) {
    const auto s = side_index(side);
    const auto frame_r = body_mat(model_, data, frame_body_);
    const auto pelvis_r = body_mat(model_, data, pelvis_body_);
    const Mat3 relative_r = mat_t_mul(frame_r, pelvis_r);
    const double pitch = std::atan2(relative_r[2], relative_r[0]);
    const Vec3 crank_world = joint_anchor(model_, data, crank_joint_);
    const Vec2 hip = mat_t_vec_xz(
        frame_r,
        vec_sub(joint_anchor(model_, data, hip_joints_[s]), crank_world));
    const Vec3 spindle_world = site_pos(model_, data, pedal_sites_[s]);
    const Vec2 spindle_local =
        mat_t_vec_xz(frame_r, vec_sub(spindle_world, crank_world));
    const auto key = std::string(side);
    named_set(sole_targets_, key, spindle_world);
    named_set(sole_goal_diagnostics_, key,
              SoleGoalDiagnostics{.spindle = true,
                                  .saturated = false,
                                  .limiting_reasons = {},
                                  .geometry = std::nullopt});
    // Python chains ik_reach_limited[side] = saturated_ik[side] = False.
    named_set(saturated_ik_, key, false);
    named_set(ik_reach_limited_, key, false);
    return spindle::leg_joint_q(leg_geometry_[s], hip,
                                spindle_local, pitch);
}

std::size_t
SpindleController::joint_index(const std::string &name) const {
    const auto found = std::ranges::find_if(
        joints_, [&](const JointEntry &j) { return j.name == name; });
    if (found == joints_.end())
        throw std::out_of_range(name);
    return static_cast<std::size_t>(found - joints_.begin());
}

SpindleController::ArmIk3dResult SpindleController::solve_arm_ik_3d(
    std::size_t side, const Mat3 &torso_r, const Vec3 &shoulder_world,
    const Vec3 &grip_world) const {
    const spindle::ArmGeometry3d geom{
        .upper_arm_m = arm3d_upper_m_[side],
        .forearm_grip_m = arm3d_lower_m_[side],
        .psi0_rad = arm3d_psi0_[side],
        .beta_ua0_rad = arm3d_beta_ua0_[side],
        .elbow_flexion0_rad = arm3d_elbow0_[side]};
    const spindle::ArmIkResult3d res =
        spindle::solve_arm_ik_3d(geom, torso_r, shoulder_world, grip_world);
    return {
        .q_abd = res.shoulder_abduction_q,
        .q_sh = res.shoulder_q,
        .q_el = res.elbow_q,
        .saturated = res.saturated,
    };
}

void SpindleController::arm_grip_jacobian_3d(
    const mjData *data, std::size_t side, std::array<double, 9> &out) const {
    const Vec3 g = site_pos(model_, data, grip_sites_[side]);
    const Vec3 p_sh = body_pos(model_, data, upper_arm_bodies_[side]);
    const Vec3 p_el = body_pos(model_, data, forearm_bodies_[side]);
    const Vec3 u0 = joint_axis(model_, data, shoulder_abd_joints_[side]);
    const Vec3 u1 = joint_axis(model_, data, shoulder_joints_[side]);
    const Vec3 u2 = joint_axis(model_, data, elbow_joints_[side]);
    const Vec3 c0 = cross3(u0, vec_sub(g, p_sh));
    const Vec3 c1 = cross3(u1, vec_sub(g, p_sh));
    const Vec3 c2 = cross3(u2, vec_sub(g, p_el));
    for (std::size_t r = 0; r < 3; ++r) {
        out[3 * r + 0] = c0[r];
        out[3 * r + 1] = c1[r];
        out[3 * r + 2] = c2[r];
    }
}

double SpindleController::ik_target_elbow_flexion_rad(
    const mjData *data, double torso_lean_rad) const {
    const auto pelvis_r = body_mat(model_, data, pelvis_body_);
    const auto frame_r = body_mat(model_, data, frame_body_);
    const auto steer_r = body_mat(model_, data, steer_body_);
    const Vec3 steer_pos = body_pos(model_, data, steer_body_);
    std::array<Vec3, 2> grips{};
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        if (config_.legacy && config_.legacy->grip_attachment == "spring") {
            Vec3 local = pose_.grip;
            local[1] += s == 0 ? -.10 : .10;
            grips[s] = vec_add(body_pos(model_, data, frame_body_),
                               mat_vec(frame_r, local));
        } else {
            grips[s] =
                vec_add(steer_pos, mat_vec(steer_r, weld_grip_offset_[s]));
        }
    }
    double frame_minus_pelvis_pitch = 0.;
    if (spatial_3d_) {
        const Mat3 relative_r = mat_t_mul(frame_r, pelvis_r);
        frame_minus_pelvis_pitch = -std::atan2(relative_r[2], relative_r[0]);
    } else {
        const double frame_pitch = std::atan2(frame_r[2], frame_r[0]);
        const double pelvis_pitch = std::atan2(pelvis_r[2], pelvis_r[0]);
        frame_minus_pelvis_pitch = spindle::wrap_angle(frame_pitch - pelvis_pitch);
    }
    const double torso_q =
        neutral_torso_q_ + torso_lean_rad + frame_minus_pelvis_pitch;
    const auto torso_r_actual = body_mat(model_, data, torso_body_);
    double delta = torso_q -
        model_access::readonly_buffer(data->qpos, model_->nq,
                                      "qpos")[static_cast<std::size_t>(
            joints_[joint_index("rider_torso_hinge")].qpos_adr)];
    delta = std::atan2(numeric::math_sin(delta), numeric::math_cos(delta));
    if (config_.legacy && config_.legacy->grip_attachment == "spring")
        delta = 0.;
    const double cd = numeric::math_cos(delta), sd = numeric::math_sin(delta);
    const std::array<double, 9> rot{cd, 0., sd, 0., 1., 0., -sd, 0., cd};
    const Vec3 hinge = joint_anchor(model_, data, torso_joint_);
    const auto torso_r =
        spatial_3d_ ? mat_mul(torso_r_actual, rot) : mat_mul(rot, torso_r_actual);
    double min_flexion = std::numeric_limits<double>::infinity();
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        if (spatial_3d_) {
            const Vec3 sh_local = mat_t_vec(
                torso_r_actual,
                vec_sub(body_pos(model_, data, upper_arm_bodies_[s]), hinge));
            const Vec3 shoulder = vec_add(hinge, mat_vec(torso_r, sh_local));
            const auto ik = solve_arm_ik_3d(s, torso_r, shoulder, grips[s]);
            min_flexion = std::min(min_flexion, arm3d_elbow0_[s] - ik.q_el);
        } else {
            const Vec3 shoulder = vec_add(
                hinge,
                mat_vec(rot,
                        vec_sub(body_pos(model_, data, upper_arm_bodies_[s]),
                                hinge)));
            const Vec3 diff = vec_sub(grips[s], shoulder);
            const Vec2 arm_target = mat_t_vec_xz(torso_r, diff);
            const auto ik = spindle::two_link_ik(
                arm_target, arm_upper_m_, arm_lower_m_, arm_branch_);
            min_flexion = std::min(min_flexion, std::abs(ik.angles[1]));
        }
    }
    return min_flexion;
}

std::optional<double> SpindleController::min_torso_lean_rad(
    const mjData *data, double min_elbow_flexion_rad) const {
    if (!std::isfinite(min_elbow_flexion_rad) || min_elbow_flexion_rad <= 0.)
        return std::nullopt;
    if (config_.legacy && config_.legacy->grip_attachment == "spring")
        return std::nullopt;
    double lo = -.8;
    double hi = lean_limit_rad_.value_or(.8);
    if (ik_target_elbow_flexion_rad(data, lo) >= min_elbow_flexion_rad)
        return lo;
    if (ik_target_elbow_flexion_rad(data, hi) < min_elbow_flexion_rad)
        return hi;
    for (int iter = 0; iter < 21; ++iter) {
        const double mid = .5 * (lo + hi);
        if (ik_target_elbow_flexion_rad(data, mid) >= min_elbow_flexion_rad)
            hi = mid;
        else
            lo = mid;
    }
    return hi;
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
        if (config_.legacy && config_.legacy->grip_attachment == "spring") {
            Vec3 local = pose_.grip;
            local[1] += s == 0 ? -.10 : .10; // rider_segments.ARM_LATERAL_OFFSET_M
            grips[s] = vec_add(body_pos(model_,data,frame_body_),mat_vec(frame_r,local));
        } else grips[s] =
                vec_add(steer_pos, mat_vec(steer_r, weld_grip_offset_[s]));
    }
    double torso_q = neutral_torso_q_ +
                     (posture == nullptr ? 0. : posture->torso_lean_rad);
    if (spatial_3d_) {
        const Mat3 relative_r = mat_t_mul(frame_r, pelvis_r);
        torso_q -= std::atan2(relative_r[2], relative_r[0]);
    } else {
        const double frame_pitch = std::atan2(frame_r[2], frame_r[0]);
        const double pelvis_pitch = std::atan2(pelvis_r[2], pelvis_r[0]);
        torso_q += spindle::wrap_angle(frame_pitch - pelvis_pitch);
    }
    const auto torso_r_actual = body_mat(model_, data, torso_body_);
    std::vector<std::pair<std::string, double>> targets;
    targets.emplace_back("rider_torso_hinge", torso_q);
    bool arm_saturated = false;
    // Welded hands close the pelvis-torso-arm chain: arm goals solved
    // about the actual torso would always be "already met" and give the
    // trunk no support. Solve about the shoulder the *target* torso angle
    // would place; a sagging trunk then meets elbow stiffness.
    double delta = torso_q -
        model_access::readonly_buffer(data->qpos, model_->nq,
                                      "qpos")[static_cast<std::size_t>(
            joints_[joint_index("rider_torso_hinge")].qpos_adr)];
    delta = std::atan2(numeric::math_sin(delta), numeric::math_cos(delta));
    if (config_.legacy && config_.legacy->grip_attachment == "spring") delta = 0.;
    const double cd = numeric::math_cos(delta), sd = numeric::math_sin(delta);
    const std::array<double, 9> rot{cd, 0., sd, 0., 1., 0., -sd, 0., cd};
    const Vec3 hinge = joint_anchor(model_, data, torso_joint_);
    const auto torso_r =
        spatial_3d_ ? mat_mul(torso_r_actual, rot) : mat_mul(rot, torso_r_actual);
    std::array<Vec3, 2> shoulders{};
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        if (spatial_3d_) {
            const Vec3 sh_local = mat_t_vec(
                torso_r_actual,
                vec_sub(body_pos(model_, data, upper_arm_bodies_[s]), hinge));
            shoulders[s] = vec_add(hinge, mat_vec(torso_r, sh_local));
        } else {
            shoulders[s] =
                vec_add(hinge,
                        mat_vec(rot,
                                vec_sub(body_pos(model_, data,
                                                 upper_arm_bodies_[s]),
                                        hinge)));
        }
    }
    for (const auto side : kArmSides) {
        const auto s = side_index(side);
        const auto stem = std::string(side);
        if (spatial_3d_) {
            const auto ik = solve_arm_ik_3d(s, torso_r, shoulders[s], grips[s]);
            arm_saturated = arm_saturated || ik.saturated;
            targets.emplace_back("rider_shoulder_abduction_" + stem, ik.q_abd);
            targets.emplace_back("rider_shoulder_" + stem, ik.q_sh);
            targets.emplace_back("rider_elbow_" + stem, ik.q_el);
        } else {
            const Vec3 diff = vec_sub(grips[s], shoulders[s]);
            const Vec2 arm_target = mat_t_vec_xz(torso_r, diff);
            const auto ik = spindle::two_link_ik(
                arm_target, arm_upper_m_, arm_lower_m_, arm_branch_);
            arm_saturated = arm_saturated || ik.saturated;
            targets.emplace_back("rider_shoulder_" + stem,
                                 arm_upper_angle0_ - ik.angles[0]);
            targets.emplace_back("rider_elbow_" + stem,
                                 (arm_lower_angle0_ - arm_upper_angle0_) -
                                     ik.angles[1]);
        }
    }
    // saturated_ik.update(torso=..., arms=...) — insertion order.
    named_set(saturated_ik_, "torso", neutral_torso_saturated_);
    named_set(saturated_ik_, "arms", arm_saturated);
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    for (auto &[name, target] : targets) {
        const double current = qpos[static_cast<std::size_t>(
            joints_[joint_index(name)].qpos_adr)];
        target = current + std::atan2(numeric::math_sin(target - current),
                                      numeric::math_cos(target - current));
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
    if (legacy_) { legacy_->initialize_velocity(*data); return; }
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
    if (legacy_) { legacy_->initialize(*data); return; }
    engine::forward(model_, data);
    reset_activation();
    const auto qpos = model_access::mutable_buffer(
        data->qpos, model_->nq, "qpos");
    // Pose the torso first, then solve the arms about that actual shoulder.
    auto upper = upper_targets(data, nullptr);
    qpos[static_cast<std::size_t>(
        joints_[joint_index("rider_torso_hinge")].qpos_adr)] =
        upper.front().second;
    engine::forward(model_, data);
    upper = upper_targets(data, nullptr);
    for (const auto &[name, value] : upper) {
        qpos[static_cast<std::size_t>(
            joints_[joint_index(name)].qpos_adr)] = value;
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
    // qpos[qa]/qvel[dof] indexing — a wrong width is the oracle's
    // IndexError, not an out-of-bounds read.
    if (qpos.size() != static_cast<std::size_t>(model_->nq) ||
        qvel.size() != static_cast<std::size_t>(model_->nv))
        throw std::invalid_argument("strength state width mismatch");
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
    if (qpos.size() != static_cast<std::size_t>(model_->nq) ||
        qvel.size() != static_cast<std::size_t>(model_->nv))
        throw std::invalid_argument("strength state width mismatch");
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
    const JointEntry &entry = joints_[joint_index(name)];
    if (!config_.has_strength)
        return config_.joint_limit_nm;
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    return strength_capacity(
        name,
        anatomical_joint_angle(
            name, qpos[static_cast<std::size_t>(entry.qpos_adr)]),
        qvel[static_cast<std::size_t>(entry.dof_adr)], sign);
}

std::vector<double>
SpindleController::limit_torques(
    std::span<const double> torques, const mjData *data,
    std::span<const char> filled,
    std::span<const double> priority_torques) const {
    const std::size_t n = joints_.size();
    if (torques.size() != n || (!filled.empty() && filled.size() != n) ||
        (!priority_torques.empty() && priority_torques.size() != n))
        throw std::invalid_argument("effort state shape mismatch");
    std::vector<double> limited(n), speeds(n);
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
    const auto clip_one = [&](std::size_t i, double raw_torque,
                              double speed) {
        const auto &joint = joints_[i];
        const double cap = directional_limit(joint.name, data,
                                             raw_torque >= 0. ? 1. : -1.);
        double value = std::min(std::max(raw_torque, -cap), cap);
        if (std::abs(speed) >= config_.joint_speed_limit_rad_s &&
            value * speed > 0.)
            value = 0.;
        return value;
    };
    for (std::size_t i = 0; i < n; ++i) {
        const auto &joint = joints_[i];
        if (!filled.empty() && !filled[i])
            throw std::out_of_range(joint.name);
        const double torque = validation::finite(
            torques[i], (joint.name + " requested torque").c_str());
        const double speed = validation::finite(
            qvel[static_cast<std::size_t>(joint.dof_adr)],
            (joint.name + " speed").c_str());
        limited[i] = clip_one(i, torque, speed);
        speeds[i] = speed;
    }
    // No configured whole-body ceiling means only the per-joint ceiling.
    const double total = config_.active_positive_power_limit_w.value_or(
        config_.joint_power_limit_w * static_cast<double>(n));
    if (priority_torques.empty()) {
        return spindle::scale_to_power_budget(limited, speeds,
                                              config_.joint_power_limit_w,
                                              total);
    }
    // «Действует» 11 (2026-11-08): rising-leg unloading and TDC push-over
    // get the positive power budget first (per joint and in total); only
    // the drive (pushing) torque is scaled into the remaining budget.
    std::vector<double> prio(n);
    for (std::size_t i = 0; i < n; ++i) {
        prio[i] = clip_one(i, priority_torques[i], speeds[i]);
        if (config_.joint_power_limit_w >= 0. &&
            prio[i] * speeds[i] > config_.joint_power_limit_w) {
            prio[i] = std::copysign(
                config_.joint_power_limit_w / std::abs(speeds[i]), prio[i]);
        }
        if (config_.joint_power_limit_w >= 0. &&
            limited[i] * speeds[i] > config_.joint_power_limit_w) {
            limited[i] = std::copysign(
                config_.joint_power_limit_w / std::abs(speeds[i]),
                limited[i]);
        }
    }
    const auto sum_pos_power = [&](std::span<const double> t) {
        std::vector<double> terms(n);
        for (std::size_t i = 0; i < n; ++i)
            terms[i] = std::max(t[i] * speeds[i], 0.);
        return numeric::numpy_sum(terms);
    };
    const double req_power = sum_pos_power(limited);
    if (req_power <= total || total < 0.)
        return limited;
    const double prio_power = sum_pos_power(prio);
    if (prio_power >= total) {
        return spindle::scale_to_power_budget(prio, speeds,
                                              config_.joint_power_limit_w,
                                              total);
    }
    double lo = 0., hi = 1.;
    std::vector<double> candidate(n);
    for (int iter = 0; iter < 60; ++iter) {
        const double mid = .5 * (lo + hi);
        for (std::size_t i = 0; i < n; ++i) {
            const double p_prio = std::max(prio[i] * speeds[i], 0.);
            const double p_lim = std::max(limited[i] * speeds[i], 0.);
            candidate[i] = p_lim > p_prio
                               ? prio[i] + mid * (limited[i] - prio[i])
                               : limited[i];
        }
        if (sum_pos_power(candidate) <= total)
            lo = mid;
        else
            hi = mid;
    }
    for (std::size_t i = 0; i < n; ++i) {
        const double p_prio = std::max(prio[i] * speeds[i], 0.);
        const double p_lim = std::max(limited[i] * speeds[i], 0.);
        limited[i] = p_lim > p_prio
                         ? prio[i] + lo * (limited[i] - prio[i])
                         : limited[i];
    }
    return limited;
}

std::vector<double>
SpindleController::finalize_effort(mjData *data,
                                   std::span<const double> torques) {
    const std::size_t n = joints_.size();
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    std::vector<double> speeds(n), target(torques.begin(), torques.end());
    for (std::size_t i = 0; i < n; ++i)
        speeds[i] = qvel[static_cast<std::size_t>(joints_[i].dof_adr)];
    const bool enabled = config_.active_positive_power_limit_w.has_value() ||
                         config_.has_strength;
    std::vector<double> excitation(target), result(target);
    std::vector<std::string> limited_names;
    if (enabled) {
        if (allocation_diagnostics_.present) {
            excitation = allocation_diagnostics_.solution_excitation_nm;
        } else {
            // Standalone callers must still obey bounded excitation.
            excitation = spindle::bounded_effort(
                speeds, excitation, config_.joint_limit_nm,
                config_.joint_speed_limit_rad_s,
                config_.joint_power_limit_w);
            excitation = strength_limited(excitation, qpos, qvel).first;
        }
        result = limit_torques(excitation, data);
        limited_names = strength_limited(excitation, qpos, qvel).second;
        if (config_.active_positive_power_limit_w.has_value())
            result = spindle::limit_positive_power(
                result, speeds, *config_.active_positive_power_limit_w);
        for (std::size_t i = 0; i < n; ++i)
            if (!isclose(result[i], target[i], 1e-9, 1e-7))
                throw std::runtime_error(
                    "allocated rider torque is outside final actuator "
                    "limits");
    }
    // rider_effort.py:70-71 — positive power is np.max(...).sum(), i.e.
    // numpy's pairwise reduction over all joints, not a sequential fold.
    std::vector<double> positive_terms(n);
    for (std::size_t i = 0; i < n; ++i)
        positive_terms[i] = std::max(result[i] * speeds[i], 0.);
    const double positive_power =
        numeric::numpy_sum(positive_terms);
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
    effort_diagnostics_.activation_saturated = saturated;
    effort_diagnostics_.strength_limited = limited_names;
    effort_diagnostics_.budget_exceeded = false;
    // F1: the published bound is the effective delivered-power budget — the
    // declared monitor bound when re-banded, else the capability cap — so the
    // (limit, budget_exceeded) diagnostic pair stays coherent.
    effort_diagnostics_.positive_power_limit_w =
        config_.delivered_positive_budget_w();
    effort_diagnostics_.observation = "incoming_request";
    for (std::size_t i = 0; i < n; ++i) {
        auto &terms = named_at(last_terms_, joints_[i].name);
        terms.active_request_nm = excitation[i];
        terms.active_delivered_nm = result[i];
        terms.command_nm = result[i];
    }
    return result;
}

std::array<double, 2>
SpindleController::leg_weights_at_pedal(mjData *data) const {
    // «Действует» 9 (2026-11-07): each leg rests on the pedal with its own
    // weight — the pedal-normal share of the leg's own gravity. qfrc_bias
    // mixes gravity with the velocity terms, so the gravity share is
    // recovered exactly as -Σ_b m_b·(g·J^com_b[:,dof]) over the rider
    // bodies; γ maps the hip/knee gravity torques onto the pedal normal.
    const Mat3 frame_r = body_mat(model_, data, frame_body_);
    const Vec3 crank_world = joint_anchor(model_, data, crank_joint_);
    const auto gravity = model_access::readonly_buffer(
        std::data(model_->opt.gravity), 3, "opt.gravity");
    const auto mass = model_access::readonly_buffer(
        model_->body_mass, model_->nbody, "body_mass");
    const auto width = static_cast<std::size_t>(model_->nv);
    std::array<Vec2, 2> gammas{};
    std::array<std::array<std::size_t, 2>, 2> leg_dofs{};
    for (const std::size_t s : {0U, 1U}) {
        const Vec2 hip = mat_t_vec_xz(
            frame_r,
            vec_sub(joint_anchor(model_, data, hip_joints_[s]),
                    crank_world));
        const Vec2 pedal = mat_t_vec_xz(
            frame_r,
            vec_sub(body_pos(model_, data, pedal_bodies_[s]),
                    crank_world));
        gammas[s] =
            spindle::leg_vertical_gamma(leg_geometry_[s], hip, pedal);
        for (std::size_t i = 0; i < 2; ++i)
            leg_dofs[s][i] = static_cast<std::size_t>(
                joints_[leg_joint_index_[s][i]].dof_adr);
    }
    std::array<std::array<double, 2>, 2> leg_gravity{};
    std::vector<mjtNum> jac(3 * width);
    for (const int body : rider_bodies_) {
        mj_jacBodyCom(model_, data, jac.data(), nullptr, body);
        const double m = mass[static_cast<std::size_t>(body)];
        for (const std::size_t s : {0U, 1U})
            for (std::size_t i = 0; i < 2; ++i) {
                const auto dof = leg_dofs[s][i];
                leg_gravity[s][i] -=
                    m * (gravity[0] * jac[dof] +
                         gravity[1] * jac[width + dof] +
                         gravity[2] * jac[2 * width + dof]);
            }
    }
    return {gammas[0][0] * leg_gravity[0][0] +
                gammas[0][1] * leg_gravity[0][1],
            gammas[1][0] * leg_gravity[1][0] +
                gammas[1][1] * leg_gravity[1][1]};
}

NamedEntries<double>
SpindleController::compute(mjData *data, const RiderCommand &command,
                           bool advance, std::optional<double> dt_s,
                           bool steady_state,
                           std::optional<std::array<bool,4>> support_available,
                           bool pedal_recovery,
                           std::optional<double> weld_crank_torque_nm) {
    if (legacy_) {
        auto previous = state();
        try {
            return legacy_->compute(*data, command, advance,
                dt_s.value_or(model_->opt.timestep), steady_state,
                support_available.value_or(std::array<bool,4>{}), pedal_recovery);
        } catch (...) {
            commit_state(std::move(previous));
            throw;
        }
    }
    const std::size_t n = joints_.size();
    command_enabled_ = command.enabled && enabled_;
    saturated_ik_.clear();
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model_->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    const auto bias = model_access::readonly_buffer(
        data->qfrc_bias, model_->nv, "qfrc_bias");
    // Python's `requested` dict: every joint present at 0. when the
    // command is disabled, only the driven hip/knee/upper names when
    // enabled. `requested_filled` tracks which dense slots the oracle's
    // dict would actually carry so _limit_torques can re-raise the
    // missing-key KeyError instead of silently clipping an implicit 0.
    std::vector<double> requested(n, 0.);
    std::vector<double> priority_requested(n, 0.);
    std::vector<char> requested_filled(n, command_enabled_ ? 0 : 1);
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
        // relative_R = frame_R.T @ pelvis_R through the shared
        // Trans-NoTrans matmul mirror; only lanes (0,2) and (0,0)
        // feed atan2.
        const Mat3 relative_r = mat_t_mul(frame_r, pelvis_r);
        const double rel02 = relative_r[2];
        const double rel00 = relative_r[0];
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
        const Vec2 crank{0., 0.};
        // _leg_feedforward (rider_control.py:758-813): inverse dynamics of
        // the weld-imposed leg motion. The feet are pinned to the pedals,
        // so the measured crank motion imposes the leg joints' trajectory
        // through the closed loop: q_ddot = J*phi_ddot + (dJ/dphi)*phi_dot^2.
        // The weld reads the difference between applied and required joint
        // torque, so the actuators must themselves supply the bias and
        // inertia of the imposed motion; only the residual task circulates
        // through the leg-crank-leg loop. The acceleration term is held at
        // zero upstream: every solved-proximity estimate re-injects the
        // weld equalities' own ringing. The same Jacobian rows feed the
        // split below and the crank-task shortfall projection.
        constexpr double kJacobianDelta = 1e-3;
        constexpr double kPedalPressMarginN = 25.;
        std::array<Vec2, 2> djacs{};
        std::array<Vec2, 2> gammas{};
        std::array<Vec2, 2> gammas_x{};
        for (const auto side : kSides) {
            const auto s = side_index(side);
            const Vec2 hip = mat_t_vec_xz(
                frame_r,
                vec_sub(joint_anchor(model_, data, hip_joints_[s]),
                        crank_world));
            jacobians[s] =
                spindle::leg_loop_jacobian(leg_geometry_[s], hip, crank,
                                           crank_length_m_, phase,
                                           pelvis_pitch);
            has_jacobian[s] = true;
            const Vec2 plus =
                spindle::leg_loop_jacobian(leg_geometry_[s], hip, crank,
                                           crank_length_m_,
                                           phase + kJacobianDelta,
                                           pelvis_pitch);
            const Vec2 minus =
                spindle::leg_loop_jacobian(leg_geometry_[s], hip, crank,
                                           crank_length_m_,
                                           phase - kJacobianDelta,
                                           pelvis_pitch);
            djacs[s] = {(plus[0] - minus[0]) / (2. * kJacobianDelta),
                        (plus[1] - minus[1]) / (2. * kJacobianDelta)};
            gammas[s] = spindle::leg_vertical_gamma(
                leg_geometry_[s], hip, spindles[s]);
            gammas_x[s] = spindle::leg_horizontal_gamma(
                leg_geometry_[s], hip, spindles[s]);
        }
        // crank_split.leg_shares' downstroke fractions, hoisted so the
        // descending-leg rule below and the stance diagnostics read the
        // same values.
        const Vec2 shares = spindle::leg_shares(phase);
        // «Действует» 9 (2026-11-07): each leg rests on the pedal with its own
        // weight; on the rising stroke muscles unload everything above the
        // leg's own weight at the pedal (including BDC reversal inertia), and
        // around TDC muscles push the foot over so pedal press stays >= 25 N.
        std::array<std::array<std::size_t, 2>, 2> leg_dofs{};
        for (const auto side : kSides) {
            const auto s = side_index(side);
            for (std::size_t i = 0; i < 2; ++i)
                leg_dofs[s][i] = static_cast<std::size_t>(
                    joints_[leg_joint_index_[s][i]].dof_adr);
        }
        const Vec3 frame_pos = body_pos(model_, data, frame_body_);
        const Vec3 pelvis_pos = body_pos(model_, data, pelvis_body_);
        const Vec3 frame_y{frame_r[1], frame_r[4], frame_r[7]};
        const Vec3 frame_up{frame_r[2], frame_r[5], frame_r[8]};
        std::array<double, 6> f_vel6{};
        std::array<double, 6> p_vel6{};
        double pelvis_rel_pitch_rate_3d = 0.;
        if (spatial_3d_) {
            for (std::size_t k = 0; k < 6; ++k) {
                f_vel6[k] = qvel[static_cast<std::size_t>(frame_free_dof_) + k];
                p_vel6[k] = qvel[static_cast<std::size_t>(rider_free_dof_) + k];
            }
            const Vec3 omega_w_frame =
                mat_vec(frame_r, {f_vel6[3], f_vel6[4], f_vel6[5]});
            const Vec3 omega_w_pelvis =
                mat_vec(pelvis_r, {p_vel6[3], p_vel6[4], p_vel6[5]});
            pelvis_rel_pitch_rate_3d =
                dot3(frame_y, vec_sub(omega_w_pelvis, omega_w_frame));
        }
        std::array<std::size_t, 5> upper_slots{};
        std::array<std::size_t, 7> upper_slots_3d{};
        if (spatial_3d_) {
            upper_slots_3d = {
                joint_index("rider_torso_hinge"),
                joint_index("rider_shoulder_abduction_left"),
                joint_index("rider_shoulder_left"),
                joint_index("rider_elbow_left"),
                joint_index("rider_shoulder_abduction_right"),
                joint_index("rider_shoulder_right"),
                joint_index("rider_elbow_right")};
        } else {
            upper_slots = {
                joint_index("rider_torso_hinge"),
                joint_index("rider_shoulder_left"),
                joint_index("rider_elbow_left"),
                joint_index("rider_shoulder_right"),
                joint_index("rider_elbow_right")};
        }
        const auto upper = upper_targets(data, &command.posture);
        for (const auto &[name, target] : upper) {
            const auto index = joint_index(name);
            const auto &joint = joints_[index];
            // Positive hinge torque applies the restoring -y reaction
            // to the pelvis.
            const double rel_pitch_rate =
                spatial_3d_
                    ? pelvis_rel_pitch_rate_3d
                    : (qvel[static_cast<std::size_t>(rider_pitch_dof_)] -
                       qvel[static_cast<std::size_t>(frame_pitch_dof_)]);
            const double torque =
                name == "rider_torso_hinge"
                    ? config_.joint_kp_nm_rad * pelvis_pitch +
                          config_.joint_kd_nms_rad * rel_pitch_rate
                    : config_.joint_kp_nm_rad *
                              spindle::wrap_angle(
                                  target -
                                  qpos[static_cast<std::size_t>(
                                      joint.qpos_adr)]) -
                          config_.joint_kd_nms_rad *
                              qvel[static_cast<std::size_t>(joint.dof_adr)];
            requested[index] =
                torque + bias[static_cast<std::size_t>(joint.dof_adr)];
            priority_requested[index] = requested[index];
            requested_filled[index] = 1;
        }
        const auto usable_slot_capacity = [&](std::size_t idx, double sign) {
            const double speed =
                qvel[static_cast<std::size_t>(joints_[idx].dof_adr)];
            if (std::abs(speed) >= config_.joint_speed_limit_rad_s &&
                sign * speed > 0.)
                return 0.;
            return directional_limit(joints_[idx].name, data, sign);
        };
        std::array<std::array<double, 9>, 2> j_grip_3d{};
        std::array<bool, 2> arm3d_valid{false, false};
        std::array<Vec3, 2> arm3d_gamma{};
        std::array<std::array<std::size_t, 3>, 2> arm3d_dof{};
        std::array<Vec3, 2> arm3d_tau{};
        if (spatial_3d_) {
            constexpr double kMaxHandForceN = 285.0;
            std::array<Vec3, 2> f_posture{};
            double eta_hand = 1.0;
            for (const std::size_t s : {0U, 1U}) {
                arm_grip_jacobian_3d(data, s, j_grip_3d[s]);
                const std::size_t idx0 = upper_slots_3d[1U + 3U * s];
                const std::size_t idx1 = upper_slots_3d[2U + 3U * s];
                const std::size_t idx2 = upper_slots_3d[3U + 3U * s];
                if (!steady_state) {
                    for (const std::size_t idx : {idx0, idx1, idx2}) {
                        const double cap_p = usable_slot_capacity(idx, 1.);
                        const double cap_n = usable_slot_capacity(idx, -1.);
                        requested[idx] = std::clamp(requested[idx], -cap_n, cap_p);
                        priority_requested[idx] = requested[idx];
                    }
                }
                const auto dof_of = [&](std::size_t idx) {
                    return static_cast<std::size_t>(joints_[idx].dof_adr);
                };
                Vec3 tau_post{
                    requested[idx0] - (steady_state ? 0. : bias[dof_of(idx0)]),
                    requested[idx1] - (steady_state ? 0. : bias[dof_of(idx1)]),
                    requested[idx2] - (steady_state ? 0. : bias[dof_of(idx2)])};
                const double max_post_n = steady_state ? kMaxHandForceN : 235.0;
                if (solve_3x3(j_grip_3d[s], true, tau_post, f_posture[s])) {
                    const double f_norm = std::sqrt(norm3_sq(f_posture[s]));
                    if (f_norm > max_post_n) {
                        const double scale = max_post_n / f_norm;
                        tau_post = {tau_post[0] * scale, tau_post[1] * scale,
                                    tau_post[2] * scale};
                        requested[idx0] =
                            tau_post[0] + (steady_state ? 0. : bias[dof_of(idx0)]);
                        requested[idx1] =
                            tau_post[1] + (steady_state ? 0. : bias[dof_of(idx1)]);
                        requested[idx2] =
                            tau_post[2] + (steady_state ? 0. : bias[dof_of(idx2)]);
                        priority_requested[idx0] = requested[idx0];
                        priority_requested[idx1] = requested[idx1];
                        priority_requested[idx2] = requested[idx2];
                        f_posture[s] = {
                            f_posture[s][0] * scale,
                            f_posture[s][1] * scale,
                            f_posture[s][2] * scale};
                        if (steady_state) eta_hand = 0.;
                    }
                }
            }
            if (eta_hand > 0.) {
                const Vec3 p_steer = joint_anchor(model_, data, steer_joint_);
                const Vec3 u_steer = joint_axis(model_, data, steer_joint_);
                std::array<Vec3, 2> t_steer{};
                for (const std::size_t s : {0U, 1U}) {
                    const Vec3 g = site_pos(model_, data, grip_sites_[s]);
                    t_steer[s] = cross3(u_steer, vec_sub(g, p_steer));
                }
                const double d_steer =
                    norm3_sq(t_steer[0]) + norm3_sq(t_steer[1]);
                if (d_steer > 1e-8) {
                    const double tau_post_steer =
                        dot3(t_steer[0], f_posture[0]) +
                        dot3(t_steer[1], f_posture[1]);
                    const double scale_steer =
                        (command.steer_torque_target_nm - tau_post_steer) / d_steer;
                    std::array<Vec3, 2> f_steer_raw{};
                    for (const std::size_t s : {0U, 1U}) {
                        f_steer_raw[s] = {
                            scale_steer * t_steer[s][0],
                            scale_steer * t_steer[s][1],
                            scale_steer * t_steer[s][2]};
                        const double q2 = norm3_sq(f_steer_raw[s]);
                        if (q2 > 1e-16) {
                            const double q1 =
                                2. * dot3(f_posture[s], f_steer_raw[s]);
                            const double q0 =
                                norm3_sq(f_posture[s]) -
                                kMaxHandForceN * kMaxHandForceN;
                            const double disc =
                                std::max(0., q1 * q1 - 4. * q2 * q0);
                            const double eta_s = std::clamp(
                                (-q1 + std::sqrt(disc)) / (2. * q2), 0., 1.);
                            eta_hand = std::min(eta_hand, eta_s);
                        }
                        const auto &jmat = j_grip_3d[s];
                        for (std::size_t k = 0; k < 3; ++k) {
                            const double dtau =
                                jmat[k] * f_steer_raw[s][0] +
                                jmat[3 + k] * f_steer_raw[s][1] +
                                jmat[6 + k] * f_steer_raw[s][2];
                            const std::size_t slot =
                                upper_slots_3d[1U + 3U * s + k];
                            const double tau0 = requested[slot];
                            if (dtau > 1e-12) {
                                eta_hand = std::min(
                                    eta_hand,
                                    std::max(0., (usable_slot_capacity(slot, 1.) - tau0) / dtau));
                            } else if (dtau < -1e-12) {
                                eta_hand = std::min(
                                    eta_hand,
                                    std::max(0., (-usable_slot_capacity(slot, -1.) - tau0) / dtau));
                            }
                        }
                    }
                    for (const std::size_t s : {0U, 1U}) {
                        const Vec3 f_cmd{
                            eta_hand * f_steer_raw[s][0],
                            eta_hand * f_steer_raw[s][1],
                            eta_hand * f_steer_raw[s][2]};
                        const auto &jmat = j_grip_3d[s];
                        for (std::size_t k = 0; k < 3; ++k) {
                            const double delta_tau =
                                jmat[k] * f_cmd[0] +
                                jmat[3 + k] * f_cmd[1] +
                                jmat[6 + k] * f_cmd[2];
                            const std::size_t slot =
                                upper_slots_3d[1U + 3U * s + k];
                            requested[slot] += delta_tau;
                            priority_requested[slot] += delta_tau;
                        }
                    }
                }
            }
            for (const std::size_t s : {0U, 1U}) {
                if (solve_3x3(j_grip_3d[s], false, frame_up, arm3d_gamma[s]))
                    arm3d_valid[s] = true;
                for (std::size_t k = 0; k < 3; ++k) {
                    const std::size_t slot = upper_slots_3d[1U + 3U * s + k];
                    arm3d_dof[s][k] =
                        static_cast<std::size_t>(joints_[slot].dof_adr);
                    const double cap = usable_slot_capacity(
                        slot, requested[slot] >= 0. ? 1. : -1.);
                    arm3d_tau[s][k] =
                        std::min(std::max(requested[slot], -cap), cap);
                }
            }
        }
        std::array<bool, 2> arm_valid{false, false};
        std::array<double, 2> arm_gamma_sh{0., 0.};
        std::array<double, 2> arm_gamma_el{0., 0.};
        std::array<std::size_t, 2> arm_sh_dof{0U, 0U};
        std::array<std::size_t, 2> arm_el_dof{0U, 0U};
        std::array<double, 2> arm_tau_sh{0., 0.};
        std::array<double, 2> arm_tau_el{0., 0.};
        if (!spatial_3d_) {
            for (const auto side : kArmSides) {
                const auto s = side_index(side);
                const Vec2 sh_xz = mat_t_vec_xz(
                    frame_r,
                    vec_sub(body_pos(model_, data, upper_arm_bodies_[s]),
                            crank_world));
                const Vec2 el_xz = mat_t_vec_xz(
                    frame_r,
                    vec_sub(body_pos(model_, data, forearm_bodies_[s]),
                            crank_world));
                const Vec2 gr_xz = mat_t_vec_xz(
                    frame_r,
                    vec_sub(site_pos(model_, data, grip_sites_[s]),
                            crank_world));
                const double det =
                    -(gr_xz[1] - sh_xz[1]) * (gr_xz[0] - el_xz[0]) +
                    (gr_xz[1] - el_xz[1]) * (gr_xz[0] - sh_xz[0]);
                if (std::abs(det) <= 1e-12)
                    continue;
                arm_valid[s] = true;
                arm_gamma_sh[s] = -(gr_xz[1] - el_xz[1]) / det;
                arm_gamma_el[s] = (gr_xz[1] - sh_xz[1]) / det;
                const std::size_t sh_idx = upper_slots[1U + 2U * s];
                const std::size_t el_idx = upper_slots[2U + 2U * s];
                arm_sh_dof[s] =
                    static_cast<std::size_t>(joints_[sh_idx].dof_adr);
                arm_el_dof[s] =
                    static_cast<std::size_t>(joints_[el_idx].dof_adr);
                const double sh_cap = usable_slot_capacity(
                    sh_idx, requested[sh_idx] >= 0. ? 1. : -1.);
                const double el_cap = usable_slot_capacity(
                    el_idx, requested[el_idx] >= 0. ? 1. : -1.);
                arm_tau_sh[s] =
                    std::min(std::max(requested[sh_idx], -sh_cap), sh_cap);
                arm_tau_el[s] =
                    std::min(std::max(requested[el_idx], -el_cap), el_cap);
            }
        }
        std::array<std::array<std::size_t, 2>, 2> leg_slots{};
        std::array<std::array<double, 2>, 2> leg_cap_pos{};
        std::array<std::array<double, 2>, 2> leg_cap_neg{};
        const std::array<double, 2> f_weight = leg_weights_at_pedal(data);
        std::array<double, 2> press_per_nm{};
        std::array<Vec3, 2> hip_world{};
        std::array<Vec3, 2> delta_hip{};
        for (const auto side : kSides) {
            const auto s = side_index(side);
            leg_slots[s] = {joint_index("rider_hip_" + std::string(side)),
                            joint_index("rider_knee_" + std::string(side))};
            for (std::size_t j = 0; j < 2; ++j) {
                leg_cap_pos[s][j] = usable_slot_capacity(leg_slots[s][j], 1.);
                leg_cap_neg[s][j] = usable_slot_capacity(leg_slots[s][j], -1.);
            }
            hip_world[s] = joint_anchor(model_, data, hip_joints_[s]);
            delta_hip[s] = vec_sub(hip_world[s], crank_world);
            if (shares[s] > 0.) {
                const double cap_hip = leg_cap_pos[s][0];
                const double cap_knee = leg_cap_neg[s][1];
                const double w_hip = cap_hip * cap_hip * jacobians[s][0];
                const double w_knee = cap_knee * cap_knee * jacobians[s][1];
                const double denom =
                    jacobians[s][0] * w_hip + jacobians[s][1] * w_knee;
                if (denom >= 1e-9) {
                    const double u_hip =
                        jacobians[s][0] > 0. ? w_hip / denom : 0.;
                    const double u_knee =
                        jacobians[s][1] < 0. ? w_knee / denom : 0.;
                    press_per_nm[s] =
                        -(gammas[s][0] * u_hip + gammas[s][1] * u_knee);
                }
            }
        }
        struct PressState {
            std::array<double, 2> f_passive{0., 0.};
            double max_total_press_n = 0.;
        };
        std::vector<double> imposed(static_cast<std::size_t>(model_->nv), 0.);
        std::vector<double> inertia(static_cast<std::size_t>(model_->nv), 0.);
        const Vec3 frame_pitch_anchor =
            spatial_3d_ ? frame_pos
                        : joint_anchor(model_, data, frame_pitch_joint_);
        const Vec3 rider_pitch_anchor =
            spatial_3d_ ? pelvis_pos
                        : joint_anchor(model_, data, rider_pitch_joint_);
        std::array<double, 2> hip_abd_lever{0., 0.};
        if (spatial_3d_) {
            for (const std::size_t s : {0U, 1U}) {
                const Vec3 u_abd_local = mat_t_vec(
                    frame_r, joint_axis(model_, data, hip_abd_joints_[s]));
                const Vec3 r_hp_frame = mat_t_vec(
                    frame_r,
                    vec_sub(body_pos(model_, data, pedal_bodies_[s]),
                            hip_world[s]));
                hip_abd_lever[s] =
                    u_abd_local[2] * r_hp_frame[0] -
                    u_abd_local[0] * r_hp_frame[2];
            }
        }
        const auto compute_press_state_3d =
            [&](double c_acc, const std::array<double, 6> &f_acc6,
                const std::array<double, 6> &p_acc6,
                const std::array<double, 7> &u_acc7) -> PressState {
            std::ranges::fill(imposed, 0.);
            for (std::size_t k = 0; k < 6; ++k)
                imposed[static_cast<std::size_t>(rider_free_dof_) + k] =
                    p_acc6[k];
            const Vec3 alpha_w_frame =
                mat_vec(frame_r, {f_acc6[3], f_acc6[4], f_acc6[5]});
            const Vec3 alpha_w_pelvis =
                mat_vec(pelvis_r, {p_acc6[3], p_acc6[4], p_acc6[5]});
            const double pelvis_rel_pitch_acc =
                dot3(frame_y, vec_sub(alpha_w_pelvis, alpha_w_frame));
            imposed[static_cast<std::size_t>(saddle_pitch_dof_)] =
                -pelvis_rel_pitch_acc;
            for (std::size_t k = 0; k < 7; ++k)
                imposed[static_cast<std::size_t>(
                    joints_[upper_slots_3d[k]].dof_adr)] = u_acc7[k];
            for (const std::size_t s : {0U, 1U}) {
                const Vec3 a_hip_w = body_point_acc(
                    pelvis_r, pelvis_pos, p_vel6, p_acc6, hip_world[s]);
                const Vec3 a_frame_w = body_point_acc(
                    frame_r, frame_pos, f_vel6, f_acc6, hip_world[s]);
                const Vec3 a_rel_frame =
                    mat_t_vec(frame_r, vec_sub(a_hip_w, a_frame_w));
                imposed[leg_dofs[s][0]] =
                    jacobians[s][0] * c_acc + djacs[s][0] * rate * rate -
                    gammas_x[s][0] * a_rel_frame[0] -
                    gammas[s][0] * a_rel_frame[2] - pelvis_rel_pitch_acc;
                imposed[leg_dofs[s][1]] =
                    jacobians[s][1] * c_acc + djacs[s][1] * rate * rate -
                    gammas_x[s][1] * a_rel_frame[0] -
                    gammas[s][1] * a_rel_frame[2];
                if (std::abs(hip_abd_lever[s]) > 0.05) {
                    imposed[static_cast<std::size_t>(hip_abd_dofs_[s])] =
                        -a_rel_frame[1] / hip_abd_lever[s];
                }
            }
            engine::mul_m(model_, data, inertia.data(), imposed.data());
            double f_grips_up = 0.;
            for (const std::size_t s : {0U, 1U}) {
                if (!arm3d_valid[s]) continue;
                for (std::size_t k = 0; k < 3; ++k) {
                    const std::size_t dof = arm3d_dof[s][k];
                    f_grips_up +=
                        arm3d_gamma[s][k] *
                        (bias[dof] + inertia[dof] - arm3d_tau[s][k]);
                }
            }
            const auto rd_free = static_cast<std::size_t>(rider_free_dof_);
            const double root_rx = bias[rd_free + 0] + inertia[rd_free + 0];
            const double root_ry = bias[rd_free + 1] + inertia[rd_free + 1];
            const double root_rz = bias[rd_free + 2] + inertia[rd_free + 2];
            PressState out;
            out.max_total_press_n =
                frame_r[2] * root_rx + frame_r[5] * root_ry +
                frame_r[8] * root_rz - f_grips_up;
            for (const std::size_t s : {0U, 1U}) {
                const auto dof0 = leg_dofs[s][0];
                const auto dof1 = leg_dofs[s][1];
                out.f_passive[s] =
                    gammas[s][0] * (bias[dof0] + inertia[dof0]) +
                    gammas[s][1] * (bias[dof1] + inertia[dof1]);
            }
            return out;
        };
        const auto compute_press_state =
            [&](double c_acc, const std::array<double, 3> &f_acc,
                const std::array<double, 3> &p_acc,
                const std::array<double, 5> &u_acc) -> PressState {
            std::ranges::fill(imposed, 0.);
            if (rider_root_x_dof_ >= 0)
                imposed[static_cast<std::size_t>(rider_root_x_dof_)] =
                    p_acc[0];
            if (rider_root_z_dof_ >= 0)
                imposed[static_cast<std::size_t>(rider_root_z_dof_)] =
                    p_acc[1];
            if (rider_pitch_dof_ >= 0)
                imposed[static_cast<std::size_t>(rider_pitch_dof_)] =
                    p_acc[2];
            for (std::size_t k = 0; k < 5; ++k)
                imposed[static_cast<std::size_t>(
                    joints_[upper_slots[k]].dof_adr)] = u_acc[k];
            const double a_crank_x =
                f_acc[0] +
                (crank_world[2] - frame_pitch_anchor[2]) * f_acc[2];
            const double a_crank_z =
                f_acc[1] -
                (crank_world[0] - frame_pitch_anchor[0]) * f_acc[2];
            const double pelvis_rel_pitch_acc = p_acc[2] - f_acc[2];
            for (const std::size_t s : {0U, 1U}) {
                const double a_hip_x =
                    p_acc[0] +
                    (hip_world[s][2] - rider_pitch_anchor[2]) * p_acc[2];
                const double a_hip_z =
                    p_acc[1] -
                    (hip_world[s][0] - rider_pitch_anchor[0]) * p_acc[2];
                const Vec3 a_rel_world{
                    a_hip_x - a_crank_x - delta_hip[s][2] * f_acc[2], 0.,
                    a_hip_z - a_crank_z + delta_hip[s][0] * f_acc[2]};
                const Vec2 hip_acc_xz = mat_t_vec_xz(frame_r, a_rel_world);
                imposed[leg_dofs[s][0]] =
                    jacobians[s][0] * c_acc + djacs[s][0] * rate * rate -
                    gammas_x[s][0] * hip_acc_xz[0] -
                    gammas[s][0] * hip_acc_xz[1] - pelvis_rel_pitch_acc;
                imposed[leg_dofs[s][1]] =
                    jacobians[s][1] * c_acc + djacs[s][1] * rate * rate -
                    gammas_x[s][1] * hip_acc_xz[0] -
                    gammas[s][1] * hip_acc_xz[1];
            }
            engine::mul_m(model_, data, inertia.data(), imposed.data());
            double f_grips_up = 0.;
            for (const std::size_t s : {0U, 1U}) {
                if (!arm_valid[s])
                    continue;
                f_grips_up +=
                    arm_gamma_sh[s] *
                        (bias[arm_sh_dof[s]] + inertia[arm_sh_dof[s]] -
                         arm_tau_sh[s]) +
                    arm_gamma_el[s] *
                        (bias[arm_el_dof[s]] + inertia[arm_el_dof[s]] -
                         arm_tau_el[s]);
            }
            const double root_rx =
                rider_root_x_dof_ < 0
                    ? 0.
                    : bias[static_cast<std::size_t>(rider_root_x_dof_)] +
                          inertia[static_cast<std::size_t>(rider_root_x_dof_)];
            const double root_rz =
                rider_root_z_dof_ < 0
                    ? 0.
                    : bias[static_cast<std::size_t>(rider_root_z_dof_)] +
                          inertia[static_cast<std::size_t>(rider_root_z_dof_)];
            PressState out;
            out.max_total_press_n =
                frame_r[2] * root_rx + frame_r[8] * root_rz - f_grips_up;
            for (const std::size_t s : {0U, 1U}) {
                const auto dof0 = leg_dofs[s][0];
                const auto dof1 = leg_dofs[s][1];
                out.f_passive[s] =
                    gammas[s][0] * (bias[dof0] + inertia[dof0]) +
                    gammas[s][1] * (bias[dof1] + inertia[dof1]);
            }
            return out;
        };
        const double incoming_coast_hold_nm = coast_hold_nm_;
        // Launch part 4b: the velocity-level kinematic reference of the leg
        // joints — the velocity analog of compute_press_state's weld closure
        // (qdot = J*rate - gamma*hip_vel - pelvis_rel_pitch_rate). The
        // damping torque -c*(qvel_leg - qdot_leg_kin) is a muscle torque
        // added to requested (non-priority) below, so it passes the
        // strength, power and priority budget like every other leg torque.
        // It damps the leg relative to the motion the crank and pelvis
        // impose, never the crank rotation itself.
        std::array<Vec2, 2> qdot_leg_kin{};
        if (spatial_3d_) {
            for (const std::size_t si : {0U, 1U}) {
                const Vec3 v_hip_w = body_point_vel(
                    pelvis_r, pelvis_pos, p_vel6, hip_world[si]);
                const Vec3 v_frame_w = body_point_vel(
                    frame_r, frame_pos, f_vel6, hip_world[si]);
                const Vec2 hip_vel_xz = mat_t_vec_xz(
                    frame_r, vec_sub(v_hip_w, v_frame_w));
                qdot_leg_kin[si][0] =
                    jacobians[si][0] * rate -
                    gammas_x[si][0] * hip_vel_xz[0] -
                    gammas[si][0] * hip_vel_xz[1] - pelvis_rel_pitch_rate_3d;
                qdot_leg_kin[si][1] =
                    jacobians[si][1] * rate -
                    gammas_x[si][1] * hip_vel_xz[0] -
                    gammas[si][1] * hip_vel_xz[1];
            }
        } else {
            const auto rd = [&](int dof) {
                return dof >= 0 ? qvel[static_cast<std::size_t>(dof)] : 0.;
            };
            const std::array<double, 3> f_vel{rd(frame_root_x_dof_),
                                              rd(frame_root_z_dof_),
                                              rd(frame_pitch_dof_)};
            const std::array<double, 3> p_vel{rd(rider_root_x_dof_),
                                              rd(rider_root_z_dof_),
                                              rd(rider_pitch_dof_)};
            const double v_crank_x =
                f_vel[0] + (crank_world[2] - frame_pitch_anchor[2]) * f_vel[2];
            const double v_crank_z =
                f_vel[1] - (crank_world[0] - frame_pitch_anchor[0]) * f_vel[2];
            const double pelvis_rel_pitch_rate = p_vel[2] - f_vel[2];
            for (const std::size_t si : {0U, 1U}) {
                const double v_hip_x =
                    p_vel[0] +
                    (hip_world[si][2] - rider_pitch_anchor[2]) * p_vel[2];
                const double v_hip_z =
                    p_vel[1] -
                    (hip_world[si][0] - rider_pitch_anchor[0]) * p_vel[2];
                const Vec3 v_rel_world{
                    v_hip_x - v_crank_x - delta_hip[si][2] * f_vel[2], 0.,
                    v_hip_z - v_crank_z + delta_hip[si][0] * f_vel[2]};
                const Vec2 hip_vel_xz = mat_t_vec_xz(frame_r, v_rel_world);
                qdot_leg_kin[si][0] =
                    jacobians[si][0] * rate - gammas_x[si][0] * hip_vel_xz[0] -
                    gammas[si][0] * hip_vel_xz[1] - pelvis_rel_pitch_rate;
                qdot_leg_kin[si][1] =
                    jacobians[si][1] * rate - gammas_x[si][1] * hip_vel_xz[0] -
                    gammas[si][1] * hip_vel_xz[1];
            }
        }
        const auto populate_leg_torques =
            [&](const PressState &ps, bool commit_hold) {
            std::array<double, 2> f_base{};
            std::array<double, 2> f_floor{};
            std::array<double, 2> f_min{};
            for (const std::size_t s : {0U, 1U}) {
                const double f_upper =
                    std::max(kPedalPressMarginN, f_weight[s]);
                f_base[s] = std::min(ps.f_passive[s], f_upper);
                f_floor[s] = steady_state ? ps.f_passive[s] : f_base[s];
                f_min[s] =
                    steady_state ? ps.f_passive[s]
                                 : std::max(kPedalPressMarginN, f_base[s]);
            }
            std::array<double, 2> f_task_press_max{};
            // The resting press a foot can lose before the task pulls
            // the pedal: the <=0 press servo's f_weight target, floored
            // at zero — a leg the pose reports as unweighted offers no
            // press to relieve, so any negative task press is a pull.
            std::array<double, 2> f_rest_n{};
            double crank_press_ceiling_nm =
                std::numeric_limits<double>::infinity();
            const double saddle_reserve_n =
                (spatial_3d_ && !steady_state) ? 50.0 : 0.0;
            for (const std::size_t s : {0U, 1U}) {
                f_task_press_max[s] = std::max(
                    0., ps.max_total_press_n - saddle_reserve_n -
                            f_min[1U - s] - f_floor[s]);
                f_rest_n[s] = std::max(f_weight[s], 0.);
                if (shares[s] > 0. && press_per_nm[s] > 1e-12) {
                    crank_press_ceiling_nm = std::min(
                        crank_press_ceiling_nm,
                        f_task_press_max[s] / (press_per_nm[s] * shares[s]));
                }
            }
            double hold_nm = incoming_coast_hold_nm;
            if (command.mean_crank_torque_nm > 0.) {
                const double raw_task = spindle::spindle_torque_waveform(
                    command.mean_crank_torque_nm, phase,
                    config_.pedal_torque_ripple);
                const double task =
                    std::min(raw_task, crank_press_ceiling_nm);
                const bool at_press_limit =
                    raw_task + std::max(0., hold_nm) >=
                    crank_press_ceiling_nm;
                if (weld_crank_torque_nm.has_value() && advance
                    && rate >= command.crank_target_rate_rad_s
                    && *weld_crank_torque_nm <=
                           raw_task + config_.weld_servo_limit_nm
                    && !at_press_limit) {
                    const double held =
                        hold_nm + config_.weld_servo_gain *
                                      (task - *weld_crank_torque_nm);
                    hold_nm = held > config_.weld_servo_limit_nm
                                  ? config_.weld_servo_limit_nm
                              : held < -config_.weld_servo_limit_nm
                                  ? -config_.weld_servo_limit_nm
                                  : held;
                }
                const double max_hold =
                    std::min(task, crank_press_ceiling_nm - task);
                const double applied_hold = hold_nm > max_hold ? max_hold
                    : hold_nm < -task                          ? -task
                                                               : hold_nm;
                total = task + applied_hold;
            } else if (command.crank_reposition) {
                // Reposition backpedal: a rate damper toward the negative
                // crank-rate goal. The cap is the rider's crank torque
                // ceiling seated_climb.max_crank_torque_nm (decision 11);
                // the gain is drive.pedaling.reposition_damping_nms_rad
                // (2026-11-16). The coasting press servo does not run — it
                // would zero the very torque the manoeuvre needs — and
                // the carried hold is neither integrated nor applied, so
                // the pedaling weld servo resumes from its held value.
                const double raw =
                    config_.reposition_damping_nms_rad *
                    (command.crank_target_rate_rad_s - rate);
                total = std::clamp(raw, -config_.reposition_torque_limit_nm,
                                   config_.reposition_torque_limit_nm);
            } else {
                // Coasting is a rider who stops pushing (user ruling
                // 2026-11-10): the coast branch applies no crank torque —
                // no rate damper, no brake, no press-cancel hold. The
                // shared hold is zeroed so nothing carries into the
                // resume; the pedaling weld servo starts clean.
                hold_nm = 0.;
                total = 0.;
            }
            if (commit_hold)
                coast_hold_nm_ = hold_nm;
            for (const std::size_t s : {0U, 1U}) {
                const auto &slots = leg_slots[s];
                const auto usable_capacity = [&](int joint, double sign) {
                    const auto j = static_cast<std::size_t>(joint);
                    return sign >= 0. ? leg_cap_pos[s][j] : leg_cap_neg[s][j];
                };
                const double leg_task_nm = shares[s] * total;
                Vec2 task_torques =
                    command.mean_crank_torque_nm > 0.
                        ? spindle::downstroke_joint_torques(
                              leg_task_nm, jacobians[s], usable_capacity)
                        : spindle::split_joint_torques(
                              leg_task_nm, jacobians[s], usable_capacity);
                double f_task_press =
                    -(gammas[s][0] * task_torques[0] +
                      gammas[s][1] * task_torques[1]);
                if (f_task_press > f_task_press_max[s] &&
                    f_task_press > 1e-12) {
                    const double scale = f_task_press_max[s] / f_task_press;
                    task_torques[0] *= scale;
                    task_torques[1] *= scale;
                    f_task_press = f_task_press_max[s];
                } else if (f_task_press < -f_rest_n[s]) {
                    // The feet only press flat pedals; a pull is
                    // impossible (simulation decisions). A negative
                    // crank task may relieve the resting press — the
                    // <=0 press servo below targets f_weight — but not
                    // go past it into a pull. Near a dead spot a strong
                    // reposition demand hits this bound first; the
                    // delivered torque then grows as the crank arm
                    // leaves the vertical.
                    const double scale = -f_rest_n[s] / f_task_press;
                    task_torques[0] *= scale;
                    task_torques[1] *= scale;
                    f_task_press = -f_rest_n[s];
                }
                const double delta_f_press =
                    steady_state
                        ? 0.
                    : command.mean_crank_torque_nm <= 0.
                        // Coasting legs rest on the pedals with their
                        // own weight (user ruling 2026-11-10): the
                        // press target is exactly f_weight — the same
                        // «Действует»-9 quantity the rising-leg rule
                        // bounds. The equal presses on the two
                        // opposite pedals cancel at the crank.
                        ? f_weight[s] - ps.f_passive[s]
                        : std::max(kPedalPressMarginN - f_task_press,
                                   f_base[s]) -
                              ps.f_passive[s];
                Vec2 press_torques{0., 0.};
                if (std::abs(delta_f_press) > 1e-9) {
                    const auto rem_capacity = [&](int joint, double sign) {
                        const double lim = usable_capacity(joint, sign);
                        const double used =
                            sign *
                            task_torques[static_cast<std::size_t>(joint)];
                        return std::max(0., lim - used);
                    };
                    press_torques = spindle::split_joint_torques(
                        delta_f_press, {-gammas[s][0], -gammas[s][1]},
                        rem_capacity);
                }
                for (std::size_t i = 0; i < 2; ++i) {
                    priority_requested[slots[i]] = press_torques[i];
                    const double damp_nm =
                        -config_.leg_active_damping_nms_rad *
                        (qvel[leg_dofs[s][i]] - qdot_leg_kin[s][i]);
                    requested[slots[i]] =
                        task_torques[i] + press_torques[i] + damp_nm;
                    requested_filled[slots[i]] = 1;
                }
            }
        };
        PressState ps =
            spatial_3d_
                ? compute_press_state_3d(
                      0., {0., 0., 0., 0., 0., 0.}, {0., 0., 0., 0., 0., 0.},
                      {0., 0., 0., 0., 0., 0., 0.})
                : compute_press_state(
                      0., {0., 0., 0.}, {0., 0., 0.}, {0., 0., 0., 0., 0.});
        if (!steady_state && target_data_) {
            mj_copyData(target_data_.get(), model_, data);
            bool first_fwd = true;
            bool has_cached_eval = false;
            std::array<double, 4> cached_req{};
            std::array<double, 4> cached_prio{};
            PressState cached_P{};
            const auto current_leg_req = [&]() -> std::array<double, 4> {
                return {requested[leg_slots[0][0]],
                        requested[leg_slots[0][1]],
                        requested[leg_slots[1][0]],
                        requested[leg_slots[1][1]]};
            };
            const auto current_leg_prio = [&]() -> std::array<double, 4> {
                return {priority_requested[leg_slots[0][0]],
                        priority_requested[leg_slots[0][1]],
                        priority_requested[leg_slots[1][0]],
                        priority_requested[leg_slots[1][1]]};
            };
            const auto eval_P = [&](const PressState &ps_in) -> PressState {
                populate_leg_torques(ps_in, false);
                const auto cur_req = current_leg_req();
                const auto cur_prio = current_leg_prio();
                if (has_cached_eval && cur_req == cached_req &&
                    cur_prio == cached_prio)
                    return cached_P;
                const std::vector<double> exc = limit_torques(
                    requested, data, requested_filled, priority_requested);
                const auto ctrl_buf = model_access::mutable_buffer(
                    target_data_->ctrl, model_->nu, "ctrl");
                for (std::size_t i = 0; i < n; ++i) {
                    const auto &joint = joints_[i];
                    if (joint.actuator_id < 0)
                        continue;
                    const double speed =
                        qvel[static_cast<std::size_t>(joint.dof_adr)];
                    double delivered = exc[i];
                    if (std::abs(speed) >= config_.joint_speed_limit_rad_s &&
                        delivered * speed > 0.)
                        delivered = 0.;
                    ctrl_buf[static_cast<std::size_t>(joint.actuator_id)] =
                        command_enabled_ ? delivered : 0.;
                }
                if (first_fwd) {
                    if (target_data_->nefc > 0)
                        mj_forwardSkip(model_, target_data_.get(), mjSTAGE_VEL,
                                       1);
                    else
                        engine::forward(model_, target_data_.get());
                    first_fwd = false;
                } else {
                    mj_forwardSkip(model_, target_data_.get(), mjSTAGE_VEL, 1);
                }
                const auto qa = model_access::readonly_buffer(
                    target_data_->qacc, model_->nv, "qacc");
                const auto rd = [&](int dof) {
                    return dof >= 0 ? qa[static_cast<std::size_t>(dof)] : 0.;
                };
                cached_req = cur_req;
                cached_prio = cur_prio;
                if (spatial_3d_) {
                    std::array<double, 6> f_acc6{};
                    std::array<double, 6> p_acc6{};
                    for (std::size_t k = 0; k < 6; ++k) {
                        f_acc6[k] = qa[static_cast<std::size_t>(frame_free_dof_) + k];
                        p_acc6[k] = qa[static_cast<std::size_t>(rider_free_dof_) + k];
                    }
                    std::array<double, 7> u_acc7{};
                    for (std::size_t k = 0; k < 7; ++k)
                        u_acc7[k] = rd(joints_[upper_slots_3d[k]].dof_adr);
                    cached_P = compute_press_state_3d(
                        rd(crank_spin_dof_), f_acc6, p_acc6, u_acc7);
                } else {
                    std::array<double, 5> u_acc{};
                    for (std::size_t k = 0; k < 5; ++k)
                        u_acc[k] = rd(joints_[upper_slots[k]].dof_adr);
                    cached_P = compute_press_state(
                        rd(crank_spin_dof_),
                        {rd(frame_root_x_dof_), rd(frame_root_z_dof_),
                         rd(frame_pitch_dof_)},
                        {rd(rider_root_x_dof_), rd(rider_root_z_dof_),
                         rd(rider_pitch_dof_)},
                        u_acc);
                }
                has_cached_eval = true;
                return cached_P;
            };
            PressState P = eval_P(ps);
            for (int iter = 0; iter < 6; ++iter) {
                const double r0 = P.f_passive[0] - ps.f_passive[0];
                const double r1 = P.f_passive[1] - ps.f_passive[1];
                const double r2 = P.max_total_press_n - ps.max_total_press_n;
                const double err =
                    std::max({std::abs(r0), std::abs(r1), std::abs(r2)});
                if (err <= 1e-6)
                    break;
                constexpr double kEps = 1e-3;
                populate_leg_torques(ps, false);
                const auto base_req = current_leg_req();
                const auto base_prio = current_leg_prio();
                const auto eval_or_base =
                    [&](const PressState &ps_pert) -> PressState {
                    populate_leg_torques(ps_pert, false);
                    if (current_leg_req() == base_req &&
                        current_leg_prio() == base_prio)
                        return P;
                    return eval_P(ps_pert);
                };
                const PressState P0 = eval_or_base(
                    {.f_passive = {ps.f_passive[0] + kEps, ps.f_passive[1]},
                     .max_total_press_n = ps.max_total_press_n});
                const PressState P1 = eval_or_base(
                    {.f_passive = {ps.f_passive[0], ps.f_passive[1] + kEps},
                     .max_total_press_n = ps.max_total_press_n});
                const double J00 =
                    (P0.f_passive[0] - P.f_passive[0]) / kEps - 1.;
                const double J10 =
                    (P0.f_passive[1] - P.f_passive[1]) / kEps;
                const double J20 =
                    (P0.max_total_press_n - P.max_total_press_n) / kEps;
                const double J01 =
                    (P1.f_passive[0] - P.f_passive[0]) / kEps;
                const double J11 =
                    (P1.f_passive[1] - P.f_passive[1]) / kEps - 1.;
                const double J21 =
                    (P1.max_total_press_n - P.max_total_press_n) / kEps;
                const double det = J00 * J11 - J01 * J10;
                double dp0 = 0.;
                double dp1 = 0.;
                double dp2 = 0.;
                if (spatial_3d_ && std::abs(det) < 0.25) {
                    dp0 = 0.5 * r0;
                    dp1 = 0.5 * r1;
                    dp2 = 0.5 * r2;
                } else {
                    if (std::abs(det) < 1e-12)
                        break;
                    dp0 = -(J11 * r0 - J01 * r1) / det;
                    dp1 = -(-J10 * r0 + J00 * r1) / det;
                    dp2 = r2 + J20 * dp0 + J21 * dp1;
                    const double max_dp = std::max(
                        {std::abs(dp0), std::abs(dp1), std::abs(dp2)});
                    if (spatial_3d_ && max_dp > 150.) {
                        dp0 = 0.5 * r0;
                        dp1 = 0.5 * r1;
                        dp2 = 0.5 * r2;
                    }
                }
                double alpha = 1.;
                PressState ps_next = ps;
                PressState P_next = P;
                bool accepted = false;
                const int max_ls = spatial_3d_ ? 8 : 5;
                for (int ls = 0; ls < max_ls; ++ls) {
                    ps_next = {
                        .f_passive = {ps.f_passive[0] + alpha * dp0,
                                      ps.f_passive[1] + alpha * dp1},
                        .max_total_press_n =
                            ps.max_total_press_n + alpha * dp2};
                    if (spatial_3d_ &&
                        (ps_next.max_total_press_n < 0. ||
                         std::abs(ps_next.f_passive[0]) > 650. ||
                         std::abs(ps_next.f_passive[1]) > 650.)) {
                        alpha *= .5;
                        continue;
                    }
                    P_next = eval_or_base(ps_next);
                    const double next_err = std::max(
                        {std::abs(P_next.f_passive[0] - ps_next.f_passive[0]),
                         std::abs(P_next.f_passive[1] - ps_next.f_passive[1]),
                         std::abs(P_next.max_total_press_n -
                                  ps_next.max_total_press_n)});
                    if (next_err < err) {
                        accepted = true;
                        break;
                    }
                    alpha *= .5;
                }
                if (spatial_3d_ && !accepted)
                    break;
                ps = ps_next;
                P = P_next;
            }
        }
        if (spatial_3d_ && !steady_state) {
            if (!has_prev_press_3d_ || data->time <= prev_press_time_3d_ ||
                data->time < 0.01) {
                has_prev_press_3d_ = true;
                prev_f_passive_3d_ = ps.f_passive;
                prev_max_press_3d_ = ps.max_total_press_n;
            } else {
                constexpr double kPressEmaAlpha3d = 0.25;
                prev_f_passive_3d_[0] +=
                    kPressEmaAlpha3d * (ps.f_passive[0] - prev_f_passive_3d_[0]);
                prev_f_passive_3d_[1] +=
                    kPressEmaAlpha3d * (ps.f_passive[1] - prev_f_passive_3d_[1]);
                prev_max_press_3d_ +=
                    kPressEmaAlpha3d * (ps.max_total_press_n - prev_max_press_3d_);
                ps.f_passive = prev_f_passive_3d_;
                ps.max_total_press_n = prev_max_press_3d_;
            }
            prev_press_time_3d_ = data->time;
        }
        populate_leg_torques(ps, true);
        if (spatial_3d_ && !steady_state && command.enabled && target_data_ &&
            saddle_eq_ >= 0 && saddle_site_ >= 0 &&
            saddle_eq_reactions_.has_value() && saddle_lstsq_.has_value()) {
            const Vec3 sad_pos = site_pos(model_, data, saddle_site_);
            const Vec3 u_steer = joint_axis(model_, data, steer_joint_);
            const Vec3 u_frame = mat_t_vec(frame_r, u_steer);
            const Vec3 g_l = mat_t_vec(
                frame_r,
                vec_sub(site_pos(model_, data, grip_sites_[0]), frame_pos));
            const Vec3 g_r = mat_t_vec(
                frame_r,
                vec_sub(site_pos(model_, data, grip_sites_[1]), frame_pos));
            const std::array<double, 2> k_roll{
                g_l[1] * u_frame[2] - g_l[2] * u_frame[1],
                g_r[1] * u_frame[2] - g_r[2] * u_frame[1]};
            if (std::abs(k_roll[0]) > 0.05 && std::abs(k_roll[1]) > 0.05) {
                const auto ctrl_buf = model_access::mutable_buffer(
                    target_data_->ctrl, model_->nu, "ctrl");
                const auto live_ctrl = model_access::readonly_buffer(
                    data->ctrl, model_->nu, "ctrl");
                std::ranges::copy(live_ctrl, ctrl_buf.begin());
                for (int roll_it = 0; roll_it < 3; ++roll_it) {
                    const std::vector<double> exc_r = limit_torques(
                        requested, data, requested_filled, priority_requested);
                    for (std::size_t i = 0; i < n; ++i) {
                        const auto &joint = joints_[i];
                        if (joint.actuator_id < 0)
                            continue;
                        const double speed =
                            qvel[static_cast<std::size_t>(joint.dof_adr)];
                        double del = exc_r[i];
                        if (std::abs(speed) >= config_.joint_speed_limit_rad_s &&
                            del * speed > 0.)
                            del = 0.;
                        ctrl_buf[static_cast<std::size_t>(joint.actuator_id)] =
                            command_enabled_ ? del : 0.;
                    }
                    mj_forwardSkip(model_, target_data_.get(), mjSTAGE_VEL, 1);
                    std::ranges::fill(saddle_qfrc_, 0.);
                    saddle_eq_reactions_->equality_qfrc_into(
                        target_data_.get(), saddle_eq_, saddle_qfrc_);
                    const rider::BodyWrench bw = rider::body_wrench(
                        model_, target_data_.get(), *saddle_lstsq_,
                        saddle_qfrc_, saddle_body_, sad_pos);
                    const double sad_fn =
                        bw.wrench[0] * frame_up[0] +
                        bw.wrench[1] * frame_up[1] +
                        bw.wrench[2] * frame_up[2];
                    const double sad_mx =
                        bw.wrench[3] * frame_r[0] +
                        bw.wrench[4] * frame_r[3] +
                        bw.wrench[5] * frame_r[6];
                    const double mx_tol = 0.015 * std::max(0., sad_fn);
                    const double mx_target =
                        std::clamp(sad_mx, -mx_tol, mx_tol);
                    const double d_mx = sad_mx - mx_target;
                    if (std::abs(d_mx) <= 0.2)
                        break;
                    constexpr double kMaxRollHandN = 285.0;
                    std::array<double, 2> a_min{-1e9, -1e9};
                    std::array<double, 2> a_max{1e9, 1e9};
                    for (const std::size_t s : {0U, 1U}) {
                        const std::size_t idx0 = upper_slots_3d[1U + 3U * s];
                        const std::size_t idx1 = upper_slots_3d[2U + 3U * s];
                        const std::size_t idx2 = upper_slots_3d[3U + 3U * s];
                        const Vec3 tau_net{
                            requested[idx0] -
                                bias[static_cast<std::size_t>(
                                    joints_[idx0].dof_adr)],
                            requested[idx1] -
                                bias[static_cast<std::size_t>(
                                    joints_[idx1].dof_adr)],
                            requested[idx2] -
                                bias[static_cast<std::size_t>(
                                    joints_[idx2].dof_adr)]};
                        Vec3 f_cur{};
                        static_cast<void>(
                            solve_3x3(j_grip_3d[s], true, tau_net, f_cur));
                        const double q1 = 2. * dot3(f_cur, u_steer);
                        const double q0 =
                            norm3_sq(f_cur) - kMaxRollHandN * kMaxRollHandN;
                        if (q0 >= 0.) {
                            a_min[s] = 0.;
                            a_max[s] = 0.;
                        } else {
                            const double disc =
                                std::max(0., q1 * q1 - 4. * q0);
                            const double rdisc = std::sqrt(disc);
                            a_min[s] =
                                std::max(a_min[s], 0.5 * (-q1 - rdisc));
                            a_max[s] =
                                std::min(a_max[s], 0.5 * (-q1 + rdisc));
                        }
                        const auto &jmat = j_grip_3d[s];
                        for (std::size_t k = 0; k < 3; ++k) {
                            const double dtau_u = jmat[k] * u_steer[0] +
                                                  jmat[3 + k] * u_steer[1] +
                                                  jmat[6 + k] * u_steer[2];
                            const std::size_t slot =
                                upper_slots_3d[1U + 3U * s + k];
                            const double tau0 = requested[slot];
                            const double lo =
                                -usable_slot_capacity(slot, -1.) - tau0;
                            const double hi =
                                usable_slot_capacity(slot, 1.) - tau0;
                            if (dtau_u > 1e-12) {
                                a_min[s] = std::max(a_min[s], lo / dtau_u);
                                a_max[s] = std::min(a_max[s], hi / dtau_u);
                            } else if (dtau_u < -1e-12) {
                                a_min[s] = std::max(a_min[s], hi / dtau_u);
                                a_max[s] = std::min(a_max[s], lo / dtau_u);
                            }
                        }
                        if (a_min[s] > a_max[s]) {
                            a_min[s] = 0.;
                            a_max[s] = 0.;
                        }
                    }
                    const double req_mx = -d_mx;
                    std::array<double, 2> alpha_roll{
                        std::clamp(0.5 * req_mx / k_roll[0], a_min[0], a_max[0]),
                        std::clamp(0.5 * req_mx / k_roll[1], a_min[1], a_max[1])};
                    double rem =
                        req_mx - (k_roll[0] * alpha_roll[0] +
                                  k_roll[1] * alpha_roll[1]);
                    if (std::abs(rem) > 0.05) {
                        alpha_roll[0] = std::clamp(
                            alpha_roll[0] + rem / k_roll[0], a_min[0], a_max[0]);
                        rem = req_mx - (k_roll[0] * alpha_roll[0] +
                                        k_roll[1] * alpha_roll[1]);
                        alpha_roll[1] = std::clamp(
                            alpha_roll[1] + rem / k_roll[1], a_min[1], a_max[1]);
                    }
                    if (std::abs(alpha_roll[0]) + std::abs(alpha_roll[1]) <= 1e-6)
                        break;
                    for (const std::size_t s : {0U, 1U}) {
                        const auto &jmat = j_grip_3d[s];
                        for (std::size_t k = 0; k < 3; ++k) {
                            const double dtau =
                                alpha_roll[s] * (jmat[k] * u_steer[0] +
                                                 jmat[3 + k] * u_steer[1] +
                                                 jmat[6 + k] * u_steer[2]);
                            const std::size_t slot =
                                upper_slots_3d[1U + 3U * s + k];
                            requested[slot] += dtau;
                            priority_requested[slot] += dtau;
                        }
                    }
                }
            }
        }
        support_diagnostics_ = {.present = true,
                                .stance_front = shares[0] >= .5,
                                .stance_rear = shares[1] >= .5};
    }
    const std::vector<double> excitation =
        limit_torques(requested, data, requested_filled, priority_requested);
    const std::vector<double> &final = excitation;
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
    // sum() over the Python generator is CPython's Neumaier loop —
    // front hip, front knee, rear hip, rear knee in dict order — so the
    // terms must neither regroup per side nor accumulate naively.
    // Every item is a float product, so the compensated float branch
    // is exact (rider_control.py's jac/final values are floats).
    numeric::PythonSum projected_acc;
    for (const auto side : kSides) {
        const auto s = side_index(side);
        if (!has_jacobian[s])
            continue;
        projected_acc.add(
            final[joint_index("rider_hip_" + std::string(side))] *
            jacobians[s][0]);
        projected_acc.add(
            final[joint_index("rider_knee_" + std::string(side))] *
            jacobians[s][1]);
    }
    const double projected = projected_acc.total();
    allocation_diagnostics_ = {.present = true,
                               .invalid_controller = false,
                               .crank_task_nm = total,
                               .crank_task_shortfall_nm = total - projected,
                               .solution_excitation_nm = excitation,
                               .legacy = std::nullopt};
    const std::vector<double> delivered =
        finalize_effort(data, final);
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
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model_->nv, "qvel");
    for (const auto &joint : joints_) {
        const double torque =
            validation::finite(named_at(torques, joint.name),
                               "rider torque");
        // rider_control.py:1403-1419 — the held plan is applied verbatim
        // between ticks, but the bound side still re-derives every step:
        // directional Hill capacity is a function of the incoming joint
        // velocity, so a held command may not exceed the muscle's
        // instantaneous concentric/eccentric capacity, and the
        // accelerating-speed gate follows the same per-step rule.
        const double cap =
            directional_limit(joint.name, data, torque >= 0. ? 1. : -1.);
        double clipped = std::min(std::max(torque, -cap), cap);
        const double speed =
            qvel[static_cast<std::size_t>(joint.dof_adr)];
        if (std::abs(speed) >= config_.joint_speed_limit_rad_s &&
            clipped * speed > 0.)
            clipped = 0.;
        // The actuator is a pure motor: ctrl is the muscle torque itself.
        // A disabled command removes only active force; any engine passive
        // force still acts physically on the DOF.
        ctrl[static_cast<std::size_t>(joint.actuator_id)] =
            command_enabled_ ? clipped : 0.;
    }
}

EffortDiagnostics SpindleController::solved_effort(
    const mjData *data, std::span<const double> incoming_qpos,
    std::span<const double> incoming_velocity, double dt_s) {
    // incoming_qpos/velocity width gate — the oracle indexes
    // incoming_state[dof] directly, so a wrong width is its IndexError,
    // never an out-of-bounds span read. Runs before the joint loop (which
    // dereferences both spans) — strength_violations repeats the check.
    if (incoming_qpos.size() != static_cast<std::size_t>(model_->nq) ||
        incoming_velocity.size() != static_cast<std::size_t>(model_->nv))
        throw std::invalid_argument("solved_effort state width mismatch");
    NamedEntries<double> active;
    double positive = 0.;
    const auto actuator_force = model_access::readonly_buffer(
        data->actuator_force, model_->nu, "actuator_force");
    for (const auto &joint : joints_) {
        const double delivered =
            actuator_force[static_cast<std::size_t>(joint.actuator_id)];
        active.emplace_back(joint.name, delivered);
        positive += std::max(
            delivered * incoming_velocity[static_cast<std::size_t>(
                              joint.dof_adr)],
            0.);
        auto &terms = named_at(last_terms_, joint.name);
        terms.solved_force_nm = delivered;
        terms.solved_active_nm = delivered;
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
        // Oracle rider_effort.py joint_effort_limits reads the resolved
        // delivered_joint_budget_w (declared re-band, else the capability
        // cap) — not the raw joint_power_limit_w field.
        if (power > config_.delivered_joint_budget_w() + 1e-9)
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
    effort_diagnostics_.positive_work_step_j = positive * dt_s;
    effort_diagnostics_.strength_violations = violations;
    effort_diagnostics_.budget_present = true;
    effort_diagnostics_.observation_present = true;
    // F1: the solved-delivery verdict reads the re-banded monitor budget,
    // falling back to the capability cap when no re-band is declared.
    effort_diagnostics_.budget_exceeded =
        config_.delivered_positive_budget_w().has_value() &&
        positive > *config_.delivered_positive_budget_w() + 1e-9;
    effort_diagnostics_.observation =
        "solved_actuator_force_at_incoming_interval";
    return effort_diagnostics_;
}

SpindleController::State SpindleController::state() const {
    State out;
    out.enabled = enabled_;
    out.command_enabled = command_enabled_;
    // The weld-press hold leaves the wire only through the material
    // channel; populate it so capture/material staging sees the live value.
    if (!legacy_) out.coast_hold_nm = coast_hold_nm_;
    out.pd_split = pd_split_;
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
    if (legacy_) out.allocation_memory = allocation_memory_;
    return out;
}

void SpindleController::restore(const State &state) {
    validate_state(state);
    State staged = state;
    commit_state(std::move(staged));
}

void SpindleController::validate_state(const State &state) const {
    if (state.allocation_memory && !legacy_)
        throw std::invalid_argument("spindle controller cannot restore legacy allocation state");
    if (!legacy_ && !state.pd_split.empty())
        throw std::invalid_argument("spindle controller carries no PD warm split");
    for (const auto& [name, split] : state.pd_split) {
        (void)joint_index(name);
        if (!std::isfinite(split[0]) || !std::isfinite(split[1]))
            throw std::invalid_argument("rider PD split must be finite");
    }
    if (state.allocation_memory && state.allocation_memory->solution)
        for (double const x : *state.allocation_memory->solution)
            if (!std::isfinite(x)) throw std::invalid_argument("rider_controller allocation warm start must be finite");
    if (state.coast_hold_nm) {
        if (!std::isfinite(*state.coast_hold_nm))
            throw std::invalid_argument("rider_controller coast hold must be finite");
        // equilibrium_cache.py forbids the material key without a spindle
        // control; carrying a hold into the legacy delegate would drop it.
        if (legacy_)
            throw std::invalid_argument("non-spindle controller cannot restore a weld coast hold");
    }
}

// NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved) fields are consumed selectively rather than by a whole-object move
void SpindleController::commit_state(State &&state) noexcept {
    enabled_ = state.enabled;
    command_enabled_ = state.command_enabled;
    coast_hold_nm_ = state.coast_hold_nm.value_or(0.);
    last_terms_ = std::move(state.last_terms);
    pd_split_ = std::move(state.pd_split);
    saturated_ik_ = std::move(state.saturated_ik);
    ik_reach_limited_ = std::move(state.ik_reach_limited);
    joint_torques_nm_ = std::move(state.joint_torques_nm);
    joint_capacity_nm_ = std::move(state.joint_capacity_nm);
    lean_limit_rad_ = state.lean_limit_rad;
    pedal_recovery_ = std::move(state.pedal_recovery);
    effort_diagnostics_ = std::move(state.effort_diagnostics);
    allocation_diagnostics_ = std::move(state.allocation_diagnostics);
    support_diagnostics_ = std::move(state.support_diagnostics);
    sole_goal_diagnostics_ = std::move(state.sole_goal_diagnostics);
    allocation_memory_ = state.allocation_memory ? std::move(*state.allocation_memory) : bike::rider::AllocationMemory{};
}

} // namespace rider
