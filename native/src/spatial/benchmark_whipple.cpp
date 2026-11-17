#include "spatial/benchmark_whipple.hpp"

#include "engine_call.hpp"
#include "lapack_abi.hpp"
#include "model_access.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <limits>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace bike::spatial::whipple {

namespace {

constexpr std::size_t kNqExpected = 10;
constexpr std::size_t kNvExpected = 9;
constexpr std::size_t kNConstraints = 6;
constexpr std::size_t kKktDim = kNvExpected + kNConstraints; // 15

using Vec3 = std::array<double, 3>;
using Mat6x9 = std::array<double, kNConstraints * kNvExpected>;
using Mat9 = std::array<double, kNvExpected * kNvExpected>;
using Mat15 = std::array<double, kKktDim * kKktDim>;
using Vec6 = std::array<double, kNConstraints>;
using Vec9 = std::array<double, kNvExpected>;
using Vec15 = std::array<double, kKktDim>;
using Jac3x9 = std::array<double, 3 * kNvExpected>;

void require_positive_finite(double value, std::string_view name) {
    if (!std::isfinite(value) || !(value > 0.0)) {
        throw std::invalid_argument(std::string(name) + " must be positive and finite");
    }
}

void require_nonnegative_finite(double value, std::string_view name) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be non-negative and finite");
    }
}

void require_finite(double value, std::string_view name) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be finite");
    }
}

void validate_parameters(const WhippleParameters& p) {
    require_positive_finite(p.w_m, "w_m");
    require_nonnegative_finite(p.c_m, "c_m");
    require_finite(p.lambda_rad, "lambda_rad");
    if (!(std::cos(p.lambda_rad) > 0.0)) {
        throw std::invalid_argument("lambda_rad must satisfy cos(lambda) > 0");
    }
    require_positive_finite(p.g_mps2, "g_mps2");

    require_positive_finite(p.r_r_m, "r_r_m");
    require_positive_finite(p.m_r_kg, "m_r_kg");
    require_positive_finite(p.i_rxx_kgm2, "i_rxx_kgm2");
    require_positive_finite(p.i_ryy_kgm2, "i_ryy_kgm2");

    require_finite(p.x_b_m, "x_b_m");
    require_finite(p.z_b_m, "z_b_m");
    require_positive_finite(p.m_b_kg, "m_b_kg");
    require_positive_finite(p.i_bxx_kgm2, "i_bxx_kgm2");
    require_positive_finite(p.i_byy_kgm2, "i_byy_kgm2");
    require_positive_finite(p.i_bzz_kgm2, "i_bzz_kgm2");
    require_finite(p.i_bxz_kgm2, "i_bxz_kgm2");

    require_finite(p.x_h_m, "x_h_m");
    require_finite(p.z_h_m, "z_h_m");
    require_positive_finite(p.m_h_kg, "m_h_kg");
    require_positive_finite(p.i_hxx_kgm2, "i_hxx_kgm2");
    require_positive_finite(p.i_hyy_kgm2, "i_hyy_kgm2");
    require_positive_finite(p.i_hzz_kgm2, "i_hzz_kgm2");
    require_finite(p.i_hxz_kgm2, "i_hxz_kgm2");

    require_positive_finite(p.r_f_m, "r_f_m");
    require_positive_finite(p.m_f_kg, "m_f_kg");
    require_positive_finite(p.i_fxx_kgm2, "i_fxx_kgm2");
    require_positive_finite(p.i_fyy_kgm2, "i_fyy_kgm2");
}

[[nodiscard]] std::string format_double(double v) {
    const double clean = (v == 0.0) ? 0.0 : v;
    return std::format("{}", clean);
}

[[nodiscard]] double dot3(const Vec3& a, const Vec3& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

[[nodiscard]] Vec3 cross3(const Vec3& a, const Vec3& b) noexcept {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    };
}

[[nodiscard]] Vec3 normalize3(const Vec3& v, std::string_view label) {
    const double scale = std::max({std::abs(v[0]), std::abs(v[1]), std::abs(v[2])});
    if (!(scale > 0.0) || !std::isfinite(scale)) {
        throw std::runtime_error(std::string(label) + ": zero or non-finite vector");
    }
    const double sx = v[0] / scale;
    const double sy = v[1] / scale;
    const double sz = v[2] / scale;
    const double norm = std::sqrt(sx * sx + sy * sy + sz * sz);
    return {sx / norm, sy / norm, sz / norm};
}

// Solves a square N x N row-major system A * x = b via LAPACK dgesv.
template <std::size_t N>
void solve_square_dgesv(const std::array<double, N * N>& a_row_major,
                        std::array<double, N>& rhs_inout,
                        std::string_view context) {
    std::array<double, N * N> a_col_major{};
    for (std::size_t row = 0; row < N; ++row) {
        for (std::size_t col = 0; col < N; ++col) {
            a_col_major[col * N + row] = a_row_major[row * N + col];
        }
    }
    const auto n_int = static_cast<numeric_backend::integer>(N);
    const numeric_backend::integer nrhs = 1;
    std::array<numeric_backend::integer, N> ipiv{};
    numeric_backend::integer info = 0;
    numeric_backend::dgesv(&n_int, &nrhs, a_col_major.data(), &n_int, ipiv.data(),
                           rhs_inout.data(), &n_int, &info);
    if (info != 0) {
        throw std::runtime_error(std::string(context) + ": LAPACK dgesv failed with info=" +
                                 std::to_string(info));
    }
    for (const double val : rhs_inout) {
        if (!std::isfinite(val)) {
            throw std::runtime_error(std::string(context) + ": non-finite solution");
        }
    }
}

void sort_eigenvalues(Eigenvalues4& eigs) noexcept {
    std::ranges::sort(eigs, [](const std::complex<double>& a, const std::complex<double>& b) {
        constexpr double kTieTol = 1e-12;
        if (std::abs(a.real() - b.real()) > kTieTol) {
            return a.real() > b.real();
        }
        return a.imag() > b.imag();
    });
}

struct HouseholderResult {
    double tau = 0.0;
    double alpha = 0.0;
};

// Householder vector for length-K vector v: constructs v with v[0]=1
// and P = I - tau * v * v^T mapping v to (alpha, 0, ...)^T.
template <std::size_t K>
[[nodiscard]] HouseholderResult householder_vec(std::array<double, K>& v) noexcept {
    double max_abs = 0.0;
    for (const double x : v) {
        max_abs = std::max(max_abs, std::abs(x));
    }
    if (!(max_abs > 0.0)) {
        return {};
    }
    for (double& x : v) {
        x /= max_abs;
    }
    double norm_sq = 0.0;
    for (std::size_t i = 1; i < K; ++i) {
        norm_sq += v[i] * v[i];
    }
    if (norm_sq == 0.0) {
        return {.tau = 0.0, .alpha = v[0] * max_abs};
    }
    const double norm = std::sqrt(v[0] * v[0] + norm_sq);
    const double beta = (v[0] >= 0.0) ? -norm : norm;
    const double v0 = v[0] - beta;
    for (std::size_t i = 1; i < K; ++i) {
        v[i] /= v0;
    }
    v[0] = 1.0;
    const double scaled_norm_sq = norm_sq / (v0 * v0);
    return {.tau = 2.0 / (1.0 + scaled_norm_sq), .alpha = beta * max_abs};
}

[[nodiscard]] std::array<std::complex<double>, 2> schur_2x2_eigenvalues(const Mat2& block) noexcept {
    const double a = block[0];
    const double b = block[1];
    const double c = block[2];
    const double d = block[3];
    const double p = 0.5 * (a - d);
    const double bc_scale = std::max({std::abs(p), std::abs(b), std::abs(c)});
    if (bc_scale == 0.0) {
        return {{{a, 0.0}, {d, 0.0}}};
    }
    const double p_s = p / bc_scale;
    const double b_s = b / bc_scale;
    const double c_s = c / bc_scale;
    const double disc_s = p_s * p_s + b_s * c_s;
    if (disc_s >= 0.0) {
        const double root = std::sqrt(disc_s) * bc_scale;
        const double z = p + std::copysign(root, p);
        const double l1 = d + z;
        const double l2 = (z != 0.0) ? (d - (b * c) / z) : d;
        return {{{l1, 0.0}, {l2, 0.0}}};
    }
    const double mu = 0.5 * (a + d);
    const double im = std::sqrt(-disc_s) * bc_scale;
    return {{{mu, im}, {mu, -im}}};
}

struct BulgeWindow {
    std::size_t l = 0;
    std::size_t m = 0;
    std::size_t k = 0;
};

// Applies 3x3 or 2x2 Francis bulge reflector at step k of active window [l..m].
void apply_bulge_step(Mat4& h, const BulgeWindow& win, const Vec3& xyz) noexcept {
    constexpr std::size_t kN = 4;
    const std::size_t l = win.l;
    const std::size_t m = win.m;
    const std::size_t k = win.k;
    const std::size_t nr = std::min<std::size_t>(3, m - k + 1U);
    const std::size_t col_start = (k > l) ? (k - 1U) : l;
    const std::size_t row_end = std::min<std::size_t>(m, k + nr);

    if (nr == 3) {
        std::array<double, 3> v = xyz;
        const auto [tau, alpha] = householder_vec(v);
        if (tau == 0.0) {
            return;
        }
        for (std::size_t j = col_start; j <= m; ++j) {
            const double sum = h[k * kN + j] + v[1] * h[(k + 1U) * kN + j] +
                               v[2] * h[(k + 2U) * kN + j];
            const double t = tau * sum;
            h[k * kN + j] -= t;
            h[(k + 1U) * kN + j] -= t * v[1];
            h[(k + 2U) * kN + j] -= t * v[2];
        }
        if (k > l) {
            h[k * kN + (k - 1U)] = alpha;
            h[(k + 1U) * kN + (k - 1U)] = 0.0;
            h[(k + 2U) * kN + (k - 1U)] = 0.0;
        }
        for (std::size_t i = l; i <= row_end; ++i) {
            const double sum = h[i * kN + k] + v[1] * h[i * kN + (k + 1U)] +
                               v[2] * h[i * kN + (k + 2U)];
            const double t = tau * sum;
            h[i * kN + k] -= t;
            h[i * kN + (k + 1U)] -= t * v[1];
            h[i * kN + (k + 2U)] -= t * v[2];
        }
    } else if (nr == 2) {
        std::array<double, 2> v{xyz[0], xyz[1]};
        const auto [tau, alpha] = householder_vec(v);
        if (tau == 0.0) {
            return;
        }
        for (std::size_t j = col_start; j <= m; ++j) {
            const double sum = h[k * kN + j] + v[1] * h[(k + 1U) * kN + j];
            const double t = tau * sum;
            h[k * kN + j] -= t;
            h[(k + 1U) * kN + j] -= t * v[1];
        }
        if (k > l) {
            h[k * kN + (k - 1U)] = alpha;
            h[(k + 1U) * kN + (k - 1U)] = 0.0;
        }
        for (std::size_t i = l; i <= row_end; ++i) {
            const double sum = h[i * kN + k] + v[1] * h[i * kN + (k + 1U)];
            const double t = tau * sum;
            h[i * kN + k] -= t;
            h[i * kN + (k + 1U)] -= t * v[1];
        }
    }
}

struct VirtualFiles {
    mjVFS value{};
    VirtualFiles() { mj_defaultVFS(&value); }
    ~VirtualFiles() { mj_deleteVFS(&value); }
    VirtualFiles(const VirtualFiles&) = delete;
    VirtualFiles& operator=(const VirtualFiles&) = delete;
    VirtualFiles(VirtualFiles&&) = delete;
    VirtualFiles& operator=(VirtualFiles&&) = delete;
};

struct MountOperation {
    mjVFS* vfs = nullptr;
    const char* name = nullptr;
    const void* bytes = nullptr;
    int size = 0;
    int result = -1;
};

void mount_operation(void* raw) noexcept {
    auto* call = static_cast<MountOperation*>(raw);
    call->result = mj_addBufferVFS(call->vfs, call->name, call->bytes, call->size);
}

struct CompileOperation {
    const char* file = nullptr;
    const mjVFS* vfs = nullptr;
    char* message = nullptr;
    int capacity = 0;
    mjModel* model = nullptr;
};

void compile_operation(void* raw) noexcept {
    auto* call = static_cast<CompileOperation*>(raw);
    call->model = mj_loadXML(call->file, call->vfs, call->message, call->capacity);
}

struct JacOperation {
    const mjModel* model = nullptr;
    const mjData* data = nullptr;
    mjtNum* jacp = nullptr;
    const mjtNum* point = nullptr;
    int body = 0;
};

// NOLINTNEXTLINE(misc-const-correctness) engine::Operation requires void*
void jac_operation(void* raw) noexcept {
    const auto* call = static_cast<const JacOperation*>(raw);
    mj_jac(call->model, call->data, call->jacp, nullptr, call->point, call->body);
}

struct FullMOperation {
    const mjModel* model = nullptr;
    const mjData* data = nullptr;
    mjtNum* dst = nullptr;
};

// NOLINTNEXTLINE(misc-const-correctness) engine::Operation requires void*
void full_m_operation(void* raw) noexcept {
    const auto* call = static_cast<const FullMOperation*>(raw);
    mj_fullM(call->model, call->data, call->dst);
}

void checked_jacp(const mjModel* model,
                  const mjData* data,
                  std::span<mjtNum> jacp,
                  const Vec3& point,
                  int body_id) {
    if (jacp.size() != 3U * static_cast<std::size_t>(model->nv)) {
        throw std::invalid_argument("checked_jacp: invalid jacobian buffer size");
    }
    JacOperation op{
        .model = model,
        .data = data,
        .jacp = jacp.data(),
        .point = point.data(),
        .body = body_id,
    };
    engine::ErrorBuffer error;
    if (!engine::invoke(jac_operation, &op, error)) {
        engine::throw_failure(error);
    }
}

void checked_full_m(const mjModel* model, const mjData* data, Mat9& out_m) {
    FullMOperation op{
        .model = model,
        .data = data,
        .dst = out_m.data(),
    };
    engine::ErrorBuffer error;
    if (!engine::invoke(full_m_operation, &op, error)) {
        engine::throw_failure(error);
    }
}

struct WhippleModelIds {
    int rear_wheel_body = 0;
    int front_wheel_body = 0;
    std::size_t root_dof = 0;
    std::size_t rear_dof = 0;
    std::size_t steer_dof = 0;
    double r_rear_m = 0.0;
    double r_front_m = 0.0;
};

[[nodiscard]] WhippleModelIds resolve_whipple_ids(const mjModel* model) {
    if (model == nullptr) {
        throw std::invalid_argument("Whipple KKT: null mjModel");
    }
    if (std::cmp_not_equal(model->nq, kNqExpected) ||
        std::cmp_not_equal(model->nv, kNvExpected) ||
        model->nbody != 5 || model->njnt != 4) {
        throw std::invalid_argument(
            "Whipple KKT: expected 4-body model (nbody=5, njnt=4, nq=10, nv=9)");
    }

    WhippleModelIds ids{};
    ids.rear_wheel_body = model_access::resolve_id(model, mjOBJ_BODY, "rear_wheel", true);
    ids.front_wheel_body = model_access::resolve_id(model, mjOBJ_BODY, "front_wheel", true);

    const int root_jnt = model_access::resolve_id(model, mjOBJ_JOINT, "root_free");
    const int rear_jnt = model_access::resolve_id(model, mjOBJ_JOINT, "rear_wheel_spin");
    const int steer_jnt = model_access::resolve_id(model, mjOBJ_JOINT, "steer_joint");
    const int front_jnt = model_access::resolve_id(model, mjOBJ_JOINT, "front_wheel_spin");
    (void)front_jnt;

    const auto jnt_dofadr =
        model_access::readonly_buffer(model->jnt_dofadr, model->njnt, "mjModel.jnt_dofadr");
    const int root_dof_i = jnt_dofadr[static_cast<std::size_t>(root_jnt)];
    const int rear_dof_i = jnt_dofadr[static_cast<std::size_t>(rear_jnt)];
    const int steer_dof_i = jnt_dofadr[static_cast<std::size_t>(steer_jnt)];
    constexpr int kNvInt = static_cast<int>(kNvExpected);
    if (root_dof_i < 0 || rear_dof_i < 0 || steer_dof_i < 0 ||
        root_dof_i + 5 >= kNvInt || rear_dof_i >= kNvInt || steer_dof_i >= kNvInt) {
        throw std::invalid_argument("Whipple KKT: invalid joint DOF addresses");
    }
    ids.root_dof = static_cast<std::size_t>(root_dof_i);
    ids.rear_dof = static_cast<std::size_t>(rear_dof_i);
    ids.steer_dof = static_cast<std::size_t>(steer_dof_i);

    const int rear_geom = model_access::resolve_id(model, mjOBJ_GEOM, "geom_rear_contact");
    const int front_geom = model_access::resolve_id(model, mjOBJ_GEOM, "geom_front_contact");
    const auto geom_size =
        model_access::readonly_buffer(model->geom_size, 3 * model->ngeom, "mjModel.geom_size");
    ids.r_rear_m = geom_size[3U * static_cast<std::size_t>(rear_geom)];
    ids.r_front_m = geom_size[3U * static_cast<std::size_t>(front_geom)];
    require_positive_finite(ids.r_rear_m, "geom_rear_contact.size[0]");
    require_positive_finite(ids.r_front_m, "geom_front_contact.size[0]");

    return ids;
}

struct WheelTarget {
    int body = 0;
    double radius_m = 0.0;
};

struct WheelBasis {
    Vec3 p_hub{};
    Vec3 axis{};
    Vec3 e1{};
    Vec3 p_contact{};
    Vec3 tx{};
    Vec3 ty{};
};

[[nodiscard]] WheelBasis compute_wheel_basis(const mjModel* model,
                                             const mjData* data,
                                             const WheelTarget& wheel) {
    const auto xpos = model_access::readonly_buffer(data->xpos, 3 * model->nbody, "mjData.xpos");
    const auto xmat = model_access::readonly_buffer(data->xmat, 9 * model->nbody, "mjData.xmat");
    const std::size_t b = static_cast<std::size_t>(wheel.body);

    const Vec3 p_hub{xpos[3U * b + 0U], xpos[3U * b + 1U], xpos[3U * b + 2U]};
    const Vec3 a = normalize3(
        {xmat[9U * b + 1U], xmat[9U * b + 4U], xmat[9U * b + 7U]},
        "wheel spin axis");

    const Vec3 d_perp{a[2] * a[0], a[2] * a[1], a[2] * a[2] - 1.0};
    const Vec3 e1 = normalize3(d_perp, "wheel radial downward vector");
    const Vec3 p_contact{
        p_hub[0] + wheel.radius_m * e1[0],
        p_hub[1] + wheel.radius_m * e1[1],
        p_hub[2] + wheel.radius_m * e1[2],
    };

    const Vec3 n_world{0.0, 0.0, 1.0};
    const Vec3 tx = normalize3(cross3(a, n_world), "wheel longitudinal tangent");
    const Vec3 ty = cross3(n_world, tx);

    return {
        .p_hub = p_hub,
        .axis = a,
        .e1 = e1,
        .p_contact = p_contact,
        .tx = tx,
        .ty = ty,
    };
}

// Knife-edge wheel vertical gap g_w(q) = z_hub,w - R_w * sqrt(1 - a_z,w^2).
[[nodiscard]] std::array<double, 2> evaluate_wheel_gaps(const mjModel* model,
                                                        const mjData* data,
                                                        const WhippleModelIds& ids) {
    const auto xpos = model_access::readonly_buffer(data->xpos, 3 * model->nbody, "mjData.xpos");
    const auto xmat = model_access::readonly_buffer(data->xmat, 9 * model->nbody, "mjData.xmat");

    const auto gap_for = [&](const WheelTarget& wheel) {
        const std::size_t b = static_cast<std::size_t>(wheel.body);
        const double z_hub = xpos[3U * b + 2U];
        const Vec3 a = normalize3(
            {xmat[9U * b + 1U], xmat[9U * b + 4U], xmat[9U * b + 7U]},
            "wheel gap axis");
        const double cos_camber = std::sqrt(std::max(0.0, 1.0 - a[2] * a[2]));
        return z_hub - wheel.radius_m * cos_camber;
    };

    return {
        gap_for({.body = ids.rear_wheel_body, .radius_m = ids.r_rear_m}),
        gap_for({.body = ids.front_wheel_body, .radius_m = ids.r_front_m}),
    };
}

struct ChartCoords {
    double phi = 0.0;
    double delta = 0.0;
    double z_pert = 0.0;
    double pitch_pert = 0.0;
};

void set_chart_qpos(const mjModel* model,
                    mjData* data,
                    const WhippleModelIds& ids,
                    const ChartCoords& c) {
    const auto qpos0 = model_access::readonly_buffer(model->qpos0, model->nq, "mjModel.qpos0");
    const auto qpos = model_access::mutable_buffer(data->qpos, model->nq, "mjData.qpos");
    std::ranges::copy(qpos0, qpos.begin());

    Vec9 dq{};
    dq[ids.root_dof + 2U] = c.z_pert;
    dq[ids.root_dof + 3U] = c.phi;
    dq[ids.root_dof + 4U] = c.pitch_pert;
    dq[ids.steer_dof] = c.delta;

    engine::integrate_pos(model, qpos.data(), dq.data(), 1.0);
    engine::kinematics(model, data);
}

// Step 1 of Method A: holonomic (z, pitch) Newton manifold projection to <= 1e-14 m.
[[nodiscard]] double project_positions(const mjModel* model,
                                       mjData* data,
                                       const WhippleModelIds& ids,
                                       double phi,
                                       double delta) {
    double z_pert = 0.0;
    double pitch_pert = 0.0;
    constexpr int kMaxIter = 8;
    constexpr double kGapTol = 1e-14;
    constexpr double kFdEps = 1e-7;

    double max_gap = 0.0;
    for (int iter = 0; iter < kMaxIter; ++iter) {
        set_chart_qpos(model, data, ids, {.phi = phi, .delta = delta, .z_pert = z_pert, .pitch_pert = pitch_pert});
        const auto gaps = evaluate_wheel_gaps(model, data, ids);
        max_gap = std::max(std::abs(gaps[0]), std::abs(gaps[1]));
        if (max_gap <= kGapTol) {
            return max_gap;
        }

        set_chart_qpos(model, data, ids, {.phi = phi, .delta = delta, .z_pert = z_pert + kFdEps, .pitch_pert = pitch_pert});
        const auto g_zp = evaluate_wheel_gaps(model, data, ids);
        set_chart_qpos(model, data, ids, {.phi = phi, .delta = delta, .z_pert = z_pert - kFdEps, .pitch_pert = pitch_pert});
        const auto g_zm = evaluate_wheel_gaps(model, data, ids);

        set_chart_qpos(model, data, ids, {.phi = phi, .delta = delta, .z_pert = z_pert, .pitch_pert = pitch_pert + kFdEps});
        const auto g_pp = evaluate_wheel_gaps(model, data, ids);
        set_chart_qpos(model, data, ids, {.phi = phi, .delta = delta, .z_pert = z_pert, .pitch_pert = pitch_pert - kFdEps});
        const auto g_pm = evaluate_wheel_gaps(model, data, ids);

        const double j00 = (g_zp[0] - g_zm[0]) / (2.0 * kFdEps);
        const double j10 = (g_zp[1] - g_zm[1]) / (2.0 * kFdEps);
        const double j01 = (g_pp[0] - g_pm[0]) / (2.0 * kFdEps);
        const double j11 = (g_pp[1] - g_pm[1]) / (2.0 * kFdEps);

        const double det = j00 * j11 - j01 * j10;
        if (!(std::abs(det) > 1e-12)) {
            throw std::runtime_error("project_positions: singular (z, pitch) gap Jacobian");
        }
        const double dz = (-gaps[0] * j11 + gaps[1] * j01) / det;
        const double dpitch = (gaps[0] * j10 - gaps[1] * j00) / det;
        z_pert += dz;
        pitch_pert += dpitch;
    }

    set_chart_qpos(model, data, ids, {.phi = phi, .delta = delta, .z_pert = z_pert, .pitch_pert = pitch_pert});
    const auto final_gaps = evaluate_wheel_gaps(model, data, ids);
    max_gap = std::max(std::abs(final_gaps[0]), std::abs(final_gaps[1]));
    if (max_gap > kGapTol) {
        throw std::runtime_error("project_positions: failed to converge to 1e-14 m");
    }
    return max_gap;
}

void build_constraint_matrix(const mjModel* model,
                             const mjData* data,
                             const WhippleModelIds& ids,
                             Mat6x9& a_mat) {
    const auto fill_wheel_rows = [&](std::size_t row_offset, const WheelTarget& wheel) {
        const WheelBasis basis = compute_wheel_basis(model, data, wheel);
        Jac3x9 jp{};
        checked_jacp(model, data, jp, basis.p_contact, wheel.body);
        for (std::size_t col = 0; col < kNvExpected; ++col) {
            const Vec3 jcol{
                jp[0U * kNvExpected + col],
                jp[1U * kNvExpected + col],
                jp[2U * kNvExpected + col],
            };
            a_mat[(row_offset + 0U) * kNvExpected + col] = jcol[2];
            a_mat[(row_offset + 1U) * kNvExpected + col] = dot3(basis.tx, jcol);
            a_mat[(row_offset + 2U) * kNvExpected + col] = dot3(basis.ty, jcol);
        }
    };

    fill_wheel_rows(0U, {.body = ids.rear_wheel_body, .radius_m = ids.r_rear_m});
    fill_wheel_rows(3U, {.body = ids.front_wheel_body, .radius_m = ids.r_front_m});
}

struct VelocityTargets {
    double v_roll = 0.0;
    double v_steer = 0.0;
    double speed_mps = 0.0;
};

// Step 2 of Method A: 9x9 admissible velocity solve.
void project_velocity(const mjModel* model,
                      mjData* data,
                      const WhippleModelIds& ids,
                      const VelocityTargets& vel,
                      Mat6x9& out_a) {
    const auto qvel = model_access::mutable_buffer(data->qvel, model->nv, "mjData.qvel");
    std::ranges::fill(qvel, 0.0);
    engine::forward(model, data);

    build_constraint_matrix(model, data, ids, out_a);

    Mat9 sys{};
    for (std::size_t r = 0; r < kNConstraints; ++r) {
        for (std::size_t c = 0; c < kNvExpected; ++c) {
            sys[r * kNvExpected + c] = out_a[r * kNvExpected + c];
        }
    }
    sys[6U * kNvExpected + (ids.root_dof + 3U)] = 1.0;
    sys[7U * kNvExpected + ids.steer_dof] = 1.0;
    sys[8U * kNvExpected + ids.rear_dof] = 1.0;

    Vec9 rhs{};
    rhs[6] = vel.v_roll;
    rhs[7] = vel.v_steer;
    rhs[8] = vel.speed_mps / ids.r_rear_m;

    solve_square_dgesv<kNvExpected>(sys, rhs, "Whipple admissible velocity");
    std::ranges::copy(rhs, qvel.begin());
}

[[nodiscard]] Vec6 mat6x9_vec(const Mat6x9& a, const Vec9& v) noexcept {
    Vec6 out{};
    for (std::size_t r = 0; r < kNConstraints; ++r) {
        double sum = 0.0;
        for (std::size_t c = 0; c < kNvExpected; ++c) {
            sum += a[r * kNvExpected + c] * v[c];
        }
        out[r] = sum;
    }
    return out;
}

struct KktWorkspace {
    DataOwner primary;
    DataOwner scratch;
    Mat6x9 a_cur{};
    Mat6x9 a_tmp{};
    Mat9 m_full{};
    Mat15 kkt_mat{};
    Vec15 kkt_rhs{};
    double max_manifold_gap_m = 0.0;
};

[[nodiscard]] State4 evaluate_vector_field(const mjModel* model,
                                           const WhippleModelIds& ids,
                                           KktWorkspace& ws,
                                           double speed_mps,
                                           const State4& x) {
    const double gap = project_positions(model, ws.primary.get(), ids, x[0], x[1]);
    ws.max_manifold_gap_m = std::max(ws.max_manifold_gap_m, gap);

    project_velocity(model, ws.primary.get(), ids,
                     {.v_roll = x[2], .v_steer = x[3], .speed_mps = speed_mps}, ws.a_cur);

    // Populate qfrc_bias and qM at the exact admissible state (q, v).
    engine::forward(model, ws.primary.get());

    const auto qpos_cur =
        model_access::readonly_buffer(ws.primary->qpos, model->nq, "primary.qpos");
    const auto qvel_cur =
        model_access::readonly_buffer(ws.primary->qvel, model->nv, "primary.qvel");
    Vec9 v_cur{};
    std::ranges::copy(qvel_cur, v_cur.begin());

    // Convective constraint curvature Adot*v = (A(q (+) h*v) - A(q (-) h*v)) * v / (2*h).
    constexpr double kCurvatureH = 1e-7;
    const auto eval_av_at_step = [&](double dt) -> Vec6 {
        const auto sq = model_access::mutable_buffer(ws.scratch->qpos, model->nq, "scratch.qpos");
        const auto sv = model_access::mutable_buffer(ws.scratch->qvel, model->nv, "scratch.qvel");
        std::ranges::copy(qpos_cur, sq.begin());
        std::ranges::fill(sv, 0.0);
        engine::integrate_pos(model, sq.data(), v_cur.data(), dt);
        engine::forward(model, ws.scratch.get());
        build_constraint_matrix(model, ws.scratch.get(), ids, ws.a_tmp);
        return mat6x9_vec(ws.a_tmp, v_cur);
    };

    const Vec6 av_plus = eval_av_at_step(+kCurvatureH);
    const Vec6 av_minus = eval_av_at_step(-kCurvatureH);
    Vec6 adot_v{};
    for (std::size_t i = 0; i < kNConstraints; ++i) {
        adot_v[i] = (av_plus[i] - av_minus[i]) / (2.0 * kCurvatureH);
    }

    checked_full_m(model, ws.primary.get(), ws.m_full);
    const auto qfrc_bias =
        model_access::readonly_buffer(ws.primary->qfrc_bias, model->nv, "mjData.qfrc_bias");

    std::ranges::fill(ws.kkt_mat, 0.0);
    for (std::size_t r = 0; r < kNvExpected; ++r) {
        for (std::size_t c = 0; c < kNvExpected; ++c) {
            ws.kkt_mat[r * kKktDim + c] = ws.m_full[r * kNvExpected + c];
        }
        for (std::size_t c = 0; c < kNConstraints; ++c) {
            const double a_cr = ws.a_cur[c * kNvExpected + r];
            ws.kkt_mat[r * kKktDim + (kNvExpected + c)] = a_cr;
            ws.kkt_mat[(kNvExpected + c) * kKktDim + r] = a_cr;
        }
        ws.kkt_rhs[r] = -qfrc_bias[r];
    }
    for (std::size_t c = 0; c < kNConstraints; ++c) {
        ws.kkt_rhs[kNvExpected + c] = -adot_v[c];
    }

    solve_square_dgesv<kKktDim>(ws.kkt_mat, ws.kkt_rhs, "Whipple 15x15 KKT");

    return {
        x[2],
        x[3],
        ws.kkt_rhs[ids.root_dof + 3U],
        ws.kkt_rhs[ids.steer_dof],
    };
}

struct FdStep {
    double speed_mps = 0.0;
    double h = 0.0;
};

void finite_diff_jacobian(const mjModel* model,
                          const WhippleModelIds& ids,
                          KktWorkspace& ws,
                          const FdStep& step,
                          Mat4& out_jac) {
    for (std::size_t col = 0; col < 4; ++col) {
        State4 xp{};
        State4 xm{};
        xp[col] = +step.h;
        xm[col] = -step.h;
        const State4 fp = evaluate_vector_field(model, ids, ws, step.speed_mps, xp);
        const State4 fm = evaluate_vector_field(model, ids, ws, step.speed_mps, xm);
        for (std::size_t row = 0; row < 4; ++row) {
            out_jac[row * 4U + col] = (fp[row] - fm[row]) / (2.0 * step.h);
        }
    }
}

[[nodiscard]] double weave_real_part(const Eigenvalues4& eigs) {
    for (const auto& lam : eigs) {
        if (std::abs(lam.imag()) > 1e-6) {
            return lam.real();
        }
    }
    throw std::runtime_error("find_weave_critical_speed_mps: no complex weave mode found");
}

[[nodiscard]] double capsize_real_part(const Eigenvalues4& eigs) {
    bool found = false;
    double max_real = -std::numeric_limits<double>::infinity();
    for (const auto& lam : eigs) {
        if (std::abs(lam.imag()) <= 1e-6) {
            found = true;
            max_real = std::max(max_real, lam.real());
        }
    }
    if (!found) {
        throw std::runtime_error("find_capsize_critical_speed_mps: no real capsize mode found");
    }
    return max_real;
}

template <typename ModeFn>
[[nodiscard]] double bisect_critical_speed(const mjModel* model,
                                           SpeedBracket bracket,
                                           double tol_mps,
                                           std::string_view label,
                                           const ModeFn& mode_real) {
    require_nonnegative_finite(bracket.low_mps, "bracket.low_mps");
    require_nonnegative_finite(bracket.high_mps, "bracket.high_mps");
    require_positive_finite(tol_mps, "tol_mps");
    if (!(bracket.low_mps < bracket.high_mps)) {
        throw std::invalid_argument(std::string(label) + ": bracket.low_mps < bracket.high_mps required");
    }

    double lo = bracket.low_mps;
    double hi = bracket.high_mps;
    double f_lo = mode_real(mujoco_kkt_eigenvalues(model, lo));
    const double f_hi = mode_real(mujoco_kkt_eigenvalues(model, hi));
    if (f_lo == 0.0) {
        return lo;
    }
    if (f_hi == 0.0) {
        return hi;
    }
    if ((f_lo > 0.0) == (f_hi > 0.0)) {
        throw std::invalid_argument(std::string(label) + ": bracket does not straddle zero crossing");
    }

    constexpr int kMaxBisectIter = 100;
    for (int iter = 0; iter < kMaxBisectIter && (hi - lo) > tol_mps; ++iter) {
        const double mid = 0.5 * (lo + hi);
        if (mid == lo || mid == hi) {
            break;
        }
        const double f_mid = mode_real(mujoco_kkt_eigenvalues(model, mid));
        if (f_mid == 0.0) {
            return mid;
        }
        if ((f_mid > 0.0) == (f_lo > 0.0)) {
            lo = mid;
            f_lo = f_mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

} // namespace

void MjModelDeleter::operator()(mjModel* model) const noexcept {
    if (model != nullptr) {
        mj_deleteModel(model);
    }
}

void MjDataDeleter::operator()(mjData* data) const noexcept {
    if (data != nullptr) {
        mj_deleteData(data);
    }
}

WhippleCanonicalMatrices WhippleCanonicalMatrices::from_parameters(
    const WhippleParameters& p) {
    validate_parameters(p);

    const double i_rzz = p.i_rxx_kgm2;
    const double i_fzz = p.i_fxx_kgm2;

    // Appendix A Eqs. A1-A3: Total bicycle mass and CoM.
    const double m_t = p.m_r_kg + p.m_b_kg + p.m_h_kg + p.m_f_kg;
    const double x_t = (p.x_b_m * p.m_b_kg + p.x_h_m * p.m_h_kg + p.w_m * p.m_f_kg) / m_t;
    const double z_t =
        (-p.r_r_m * p.m_r_kg + p.z_b_m * p.m_b_kg + p.z_h_m * p.m_h_kg - p.r_f_m * p.m_f_kg) / m_t;

    // Appendix A Eqs. A4-A6: Total inertia about rear wheel contact (0, 0, 0).
    const double i_txx = p.i_rxx_kgm2 + p.i_bxx_kgm2 + p.i_hxx_kgm2 + p.i_fxx_kgm2 +
                         p.m_r_kg * p.r_r_m * p.r_r_m + p.m_b_kg * p.z_b_m * p.z_b_m +
                         p.m_h_kg * p.z_h_m * p.z_h_m + p.m_f_kg * p.r_f_m * p.r_f_m;
    const double i_txz = p.i_bxz_kgm2 + p.i_hxz_kgm2 - p.m_b_kg * p.x_b_m * p.z_b_m -
                         p.m_h_kg * p.x_h_m * p.z_h_m + p.m_f_kg * p.w_m * p.r_f_m;
    const double i_tzz = i_rzz + p.i_bzz_kgm2 + p.i_hzz_kgm2 + i_fzz +
                         p.m_b_kg * p.x_b_m * p.x_b_m + p.m_h_kg * p.x_h_m * p.x_h_m +
                         p.m_f_kg * p.w_m * p.w_m;

    // Appendix A Eqs. A7-A9: Front assembly (H + F) mass and CoM.
    const double m_a = p.m_h_kg + p.m_f_kg;
    const double x_a = (p.x_h_m * p.m_h_kg + p.w_m * p.m_f_kg) / m_a;
    const double z_a = (p.z_h_m * p.m_h_kg - p.r_f_m * p.m_f_kg) / m_a;

    // Appendix A Eqs. A10-A12: Front assembly inertia about its CoM.
    const double dx_ha = p.x_h_m - x_a;
    const double dz_ha = p.z_h_m - z_a;
    const double dx_fa = p.w_m - x_a;
    const double dz_fa = -p.r_f_m - z_a;
    const double i_axx = p.i_hxx_kgm2 + p.i_fxx_kgm2 +
                         p.m_h_kg * dz_ha * dz_ha + p.m_f_kg * dz_fa * dz_fa;
    const double i_axz = p.i_hxz_kgm2 - p.m_h_kg * dx_ha * dz_ha - p.m_f_kg * dx_fa * dz_fa;
    const double i_azz = p.i_hzz_kgm2 + i_fzz +
                         p.m_h_kg * dx_ha * dx_ha + p.m_f_kg * dx_fa * dx_fa;

    // Appendix A Eqs. A13-A16: Steer axis geometry & front assembly inertia projections.
    const double s_lam = std::sin(p.lambda_rad);
    const double c_lam = std::cos(p.lambda_rad);
    const double u_a = (x_a - p.w_m - p.c_m) * c_lam - z_a * s_lam;
    const double i_all = m_a * u_a * u_a + i_axx * s_lam * s_lam +
                         2.0 * i_axz * s_lam * c_lam + i_azz * c_lam * c_lam;
    const double i_alx = -m_a * u_a * z_a + i_axx * s_lam + i_axz * c_lam;
    const double i_alz = m_a * u_a * x_a + i_axz * s_lam + i_azz * c_lam;

    // Appendix A Eqs. A17-A27: Trail ratio, gyroscopic terms, and M, K0, K2, C1.
    const double mu_trail = (p.c_m / p.w_m) * c_lam;
    const double s_r = p.i_ryy_kgm2 / p.r_r_m;
    const double s_f = p.i_fyy_kgm2 / p.r_f_m;
    const double s_t = s_r + s_f;
    const double s_a = m_a * u_a + mu_trail * m_t * x_t;

    const double m_pp = i_txx;
    const double m_pd = i_alx + mu_trail * i_txz;
    const double m_dd = i_all + 2.0 * mu_trail * i_alz + mu_trail * mu_trail * i_tzz;

    const double k0_pp = m_t * z_t;
    const double k0_pd = -s_a;
    const double k0_dd = -s_a * s_lam;

    const double k2_pp = 0.0;
    const double k2_pd = ((s_t - m_t * z_t) / p.w_m) * c_lam;
    const double k2_dp = 0.0;
    const double k2_dd = ((s_a + s_f * s_lam) / p.w_m) * c_lam;

    const double c1_pp = 0.0;
    const double c1_pd = mu_trail * s_t + s_f * c_lam + (i_txz / p.w_m) * c_lam - mu_trail * m_t * z_t;
    const double c1_dp = -(mu_trail * s_t + s_f * c_lam);
    const double c1_dd = (i_alz / p.w_m) * c_lam + mu_trail * (s_a + (i_tzz / p.w_m) * c_lam);

    return {
        .M = {m_pp, m_pd, m_pd, m_dd},
        .C1 = {c1_pp, c1_pd, c1_dp, c1_dd},
        .K0 = {k0_pp, k0_pd, k0_pd, k0_dd},
        .K2 = {k2_pp, k2_pd, k2_dp, k2_dd},
        .g_mps2 = p.g_mps2,
    };
}

void WhippleCanonicalMatrices::compute_state_matrix(double speed_mps, Mat4& out) const {
    require_finite(speed_mps, "speed_mps");
    const double det_m = M[0] * M[3] - M[1] * M[2];
    if (!(std::abs(det_m) > 0.0) || !std::isfinite(det_m)) {
        throw std::invalid_argument("WhippleCanonicalMatrices: singular M matrix");
    }
    // M^{-1}
    const Mat2 m_inv{
        M[3] / det_m, -M[1] / det_m,
        -M[2] / det_m, M[0] / det_m,
    };
    const auto mul2 = [](const Mat2& a, const Mat2& b) noexcept -> Mat2 {
        return {
            a[0] * b[0] + a[1] * b[2], a[0] * b[1] + a[1] * b[3],
            a[2] * b[0] + a[3] * b[2], a[2] * b[1] + a[3] * b[3],
        };
    };

    const double v2 = speed_mps * speed_mps;
    const Mat2 k_tot{
        g_mps2 * K0[0] + v2 * K2[0], g_mps2 * K0[1] + v2 * K2[1],
        g_mps2 * K0[2] + v2 * K2[2], g_mps2 * K0[3] + v2 * K2[3],
    };
    const Mat2 c_tot{
        speed_mps * C1[0], speed_mps * C1[1],
        speed_mps * C1[2], speed_mps * C1[3],
    };

    const Mat2 a_k = mul2(m_inv, k_tot);
    const Mat2 a_c = mul2(m_inv, c_tot);

    out = {
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        -a_k[0], -a_k[1], -a_c[0], -a_c[1],
        -a_k[2], -a_k[3], -a_c[2], -a_c[3],
    };
}

Eigenvalues4 WhippleCanonicalMatrices::reference_eigenvalues(double speed_mps) const {
    Mat4 state_mat{};
    compute_state_matrix(speed_mps, state_mat);
    return real_4x4_eigenvalues(state_mat);
}

Eigenvalues4 real_4x4_eigenvalues(const Mat4& matrix) {
    for (const double val : matrix) {
        if (!std::isfinite(val)) {
            throw std::invalid_argument("real_4x4_eigenvalues: non-finite matrix entry");
        }
    }

    constexpr std::size_t kN = 4;
    Mat4 h = matrix;

    // 1. Reduce 4x4 matrix to upper Hessenberg form via orthogonal Householder reflections.
    for (std::size_t k = 0; k < kN - 2U; ++k) {
        if (k == 0U) {
            std::array<double, 3> v{h[1U * kN + 0U], h[2U * kN + 0U], h[3U * kN + 0U]};
            const auto [tau, alpha] = householder_vec(v);
            if (tau != 0.0) {
                for (std::size_t j = 0; j < kN; ++j) {
                    const double s = h[1U * kN + j] + v[1] * h[2U * kN + j] + v[2] * h[3U * kN + j];
                    const double t = tau * s;
                    h[1U * kN + j] -= t;
                    h[2U * kN + j] -= t * v[1];
                    h[3U * kN + j] -= t * v[2];
                }
                h[1U * kN + 0U] = alpha;
                h[2U * kN + 0U] = 0.0;
                h[3U * kN + 0U] = 0.0;
                for (std::size_t i = 0; i < kN; ++i) {
                    const double s = h[i * kN + 1U] + v[1] * h[i * kN + 2U] + v[2] * h[i * kN + 3U];
                    const double t = tau * s;
                    h[i * kN + 1U] -= t;
                    h[i * kN + 2U] -= t * v[1];
                    h[i * kN + 3U] -= t * v[2];
                }
            }
        } else {
            std::array<double, 2> v{h[2U * kN + 1U], h[3U * kN + 1U]};
            const auto [tau, alpha] = householder_vec(v);
            if (tau != 0.0) {
                for (std::size_t j = 1; j < kN; ++j) {
                    const double s = h[2U * kN + j] + v[1] * h[3U * kN + j];
                    const double t = tau * s;
                    h[2U * kN + j] -= t;
                    h[3U * kN + j] -= t * v[1];
                }
                h[2U * kN + 1U] = alpha;
                h[3U * kN + 1U] = 0.0;
                for (std::size_t i = 0; i < kN; ++i) {
                    const double s = h[i * kN + 2U] + v[1] * h[i * kN + 3U];
                    const double t = tau * s;
                    h[i * kN + 2U] -= t;
                    h[i * kN + 3U] -= t * v[1];
                }
            }
        }
    }

    // 2. Francis double-shift QR iteration on upper Hessenberg H.
    Eigenvalues4 out{};
    std::size_t out_count = 0;
    double matrix_scale = 0.0;
    for (const double val : h) {
        matrix_scale = std::max(matrix_scale, std::abs(val));
    }
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    constexpr double kSafMin = std::numeric_limits<double>::min();

    int m_signed = 3;
    int iter = 0;
    int total_iter = 0;
    constexpr int kMaxTotalIter = 400;

    while (m_signed >= 0) {
        const auto m = static_cast<std::size_t>(m_signed);
        std::size_t l = 0;
        for (std::size_t i = m; i > 0; --i) {
            double tst = std::abs(h[(i - 1U) * kN + (i - 1U)]) + std::abs(h[i * kN + i]);
            if (tst == 0.0) {
                tst = matrix_scale;
            }
            if (std::abs(h[i * kN + (i - 1U)]) <= std::max(kSafMin, kEps * tst)) {
                h[i * kN + (i - 1U)] = 0.0;
                l = i;
                break;
            }
        }

        if (l == m) {
            out[out_count++] = {h[m * kN + m], 0.0};
            --m_signed;
            iter = 0;
            continue;
        }
        if (l + 1U == m) {
            const auto pair2 = schur_2x2_eigenvalues({
                h[(m - 1U) * kN + (m - 1U)], h[(m - 1U) * kN + m],
                h[m * kN + (m - 1U)], h[m * kN + m],
            });
            out[out_count++] = pair2[0];
            out[out_count++] = pair2[1];
            m_signed -= 2;
            iter = 0;
            continue;
        }

        ++iter;
        ++total_iter;
        if (total_iter > kMaxTotalIter) {
            throw std::runtime_error("real_4x4_eigenvalues: Francis QR failed to converge");
        }

        double trace = 0.0;
        double det = 0.0;
        if (iter % 10 == 0) {
            const double s = std::abs(h[m * kN + (m - 1U)]) +
                             std::abs(h[(m - 1U) * kN + (m - 2U)]);
            const double h_mm = h[m * kN + m];
            const double shift_diag = h_mm + 0.75 * s;
            trace = 2.0 * shift_diag;
            det = shift_diag * shift_diag + 0.4375 * s * s;
        } else {
            const double a11 = h[(m - 1U) * kN + (m - 1U)];
            const double a12 = h[(m - 1U) * kN + m];
            const double a21 = h[m * kN + (m - 1U)];
            const double a22 = h[m * kN + m];
            trace = a11 + a22;
            det = a11 * a22 - a12 * a21;
        }

        double x = h[l * kN + l] * h[l * kN + l] +
                   h[l * kN + (l + 1U)] * h[(l + 1U) * kN + l] -
                   trace * h[l * kN + l] + det;
        double y = h[(l + 1U) * kN + l] *
                   (h[l * kN + l] + h[(l + 1U) * kN + (l + 1U)] - trace);
        double z = h[(l + 1U) * kN + l] * h[(l + 2U) * kN + (l + 1U)];

        for (std::size_t k = l; k < m; ++k) {
            apply_bulge_step(h, {.l = l, .m = m, .k = k}, {x, y, z});
            if (k + 1U < m) {
                x = h[(k + 1U) * kN + k];
                y = h[(k + 2U) * kN + k];
                z = (k + 3U <= m) ? h[(k + 3U) * kN + k] : 0.0;
            }
        }
    }

    sort_eigenvalues(out);
    return out;
}

std::string generate_whipple_mjcf(const WhippleParameters& p) {
    validate_parameters(p);

    const double sin_lam = std::sin(p.lambda_rad);
    const double cos_lam = std::cos(p.lambda_rad);
    const double steer_x = p.w_m + p.c_m;

    std::string xml;
    xml.reserve(2048);
    xml += "<mujoco model=\"whipple_benchmark\">\n";
    xml += "  <compiler angle=\"radian\" inertiafromgeom=\"false\"/>\n";
    xml += std::format("  <option gravity=\"0 0 {}\" timestep=\"0.0002\" integrator=\"Euler\"/>\n",
                       format_double(-p.g_mps2));
    xml += "  <default>\n";
    xml += "    <joint damping=\"0\" stiffness=\"0\" frictionloss=\"0\" armature=\"0\" limited=\"false\"/>\n";
    xml += "    <geom contype=\"0\" conaffinity=\"0\"/>\n";
    xml += "  </default>\n";
    xml += "  <worldbody>\n";
    xml += std::format("    <body name=\"frame\" pos=\"0 0 {}\">\n", format_double(p.r_r_m));
    xml += "      <freejoint name=\"root_free\"/>\n";
    xml += std::format(
        "      <inertial mass=\"{}\" pos=\"{} 0 {}\" fullinertia=\"{} {} {} 0 {} 0\"/>\n",
        format_double(p.m_b_kg),
        format_double(p.x_b_m),
        format_double(-p.z_b_m - p.r_r_m),
        format_double(p.i_bxx_kgm2),
        format_double(p.i_byy_kgm2),
        format_double(p.i_bzz_kgm2),
        format_double(-p.i_bxz_kgm2));
    xml += "      <body name=\"rear_wheel\" pos=\"0 0 0\">\n";
    xml += "        <joint name=\"rear_wheel_spin\" type=\"hinge\" axis=\"0 1 0\"/>\n";
    xml += std::format(
        "        <inertial mass=\"{}\" pos=\"0 0 0\" fullinertia=\"{} {} {} 0 0 0\"/>\n",
        format_double(p.m_r_kg),
        format_double(p.i_rxx_kgm2),
        format_double(p.i_ryy_kgm2),
        format_double(p.i_rxx_kgm2));
    xml += std::format(
        "        <geom name=\"geom_rear_contact\" type=\"sphere\" size=\"{}\"/>\n",
        format_double(p.r_r_m));
    xml += "      </body>\n";
    xml += std::format("      <body name=\"steer\" pos=\"{} 0 {}\">\n",
                       format_double(steer_x), format_double(-p.r_r_m));
    xml += std::format(
        "        <joint name=\"steer_joint\" type=\"hinge\" axis=\"{} 0 {}\"/>\n",
        format_double(-sin_lam), format_double(cos_lam));
    xml += std::format(
        "        <inertial mass=\"{}\" pos=\"{} 0 {}\" fullinertia=\"{} {} {} 0 {} 0\"/>\n",
        format_double(p.m_h_kg),
        format_double(p.x_h_m - steer_x),
        format_double(-p.z_h_m),
        format_double(p.i_hxx_kgm2),
        format_double(p.i_hyy_kgm2),
        format_double(p.i_hzz_kgm2),
        format_double(-p.i_hxz_kgm2));
    xml += std::format("        <body name=\"front_wheel\" pos=\"{} 0 {}\">\n",
                       format_double(-p.c_m), format_double(p.r_f_m));
    xml += "          <joint name=\"front_wheel_spin\" type=\"hinge\" axis=\"0 1 0\"/>\n";
    xml += std::format(
        "          <inertial mass=\"{}\" pos=\"0 0 0\" fullinertia=\"{} {} {} 0 0 0\"/>\n",
        format_double(p.m_f_kg),
        format_double(p.i_fxx_kgm2),
        format_double(p.i_fyy_kgm2),
        format_double(p.i_fxx_kgm2));
    xml += std::format(
        "          <geom name=\"geom_front_contact\" type=\"sphere\" size=\"{}\"/>\n",
        format_double(p.r_f_m));
    xml += "        </body>\n";
    xml += "      </body>\n";
    xml += "    </body>\n";
    xml += "  </worldbody>\n";
    xml += "</mujoco>\n";
    return xml;
}

ModelOwner compile_whipple_mjcf(std::string_view xml) {
    if (xml.empty() || xml.find('\0') != std::string_view::npos) {
        throw std::invalid_argument("compile_whipple_mjcf: XML must be nonempty without NUL");
    }
    if (xml.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("compile_whipple_mjcf: XML exceeds VFS buffer limit");
    }
    engine::require_supported_callbacks();
    engine::require_native_time_policy();

    auto files = std::make_unique<VirtualFiles>();
    constexpr const char* kFilename = "whipple_benchmark.xml";
    engine::ErrorBuffer error;
    MountOperation mount{
        .vfs = &files->value,
        .name = kFilename,
        .bytes = xml.data(),
        .size = static_cast<int>(xml.size()),
    };
    if (!engine::invoke(mount_operation, &mount, error)) {
        engine::throw_failure(error);
    }
    if (mount.result != 0) {
        throw std::runtime_error("compile_whipple_mjcf: failed to mount XML in VFS");
    }

    auto message = std::make_unique<std::array<char, 4096>>();
    CompileOperation compile{
        .file = kFilename,
        .vfs = &files->value,
        .message = message->data(),
        .capacity = static_cast<int>(message->size()),
    };
    if (!engine::invoke(compile_operation, &compile, error)) {
        ModelOwner const cleanup(compile.model);
        engine::throw_failure(error);
    }
    ModelOwner result(compile.model);
    if (!result) {
        throw std::invalid_argument(std::string("compile_whipple_mjcf: ") + message->data());
    }
    if (result->nplugin != 0) {
        throw std::invalid_argument("compile_whipple_mjcf: unsupported MuJoCo plugin model");
    }
    return result;
}

KktLinearizationResult mujoco_kkt_linearize(const mjModel* model, double speed_mps) {
    require_finite(speed_mps, "speed_mps");
    const WhippleModelIds ids = resolve_whipple_ids(model);

    auto ws = std::make_unique<KktWorkspace>();
    ws->primary = DataOwner(engine::make_data(model));
    ws->scratch = DataOwner(engine::make_data(model));
    if (!ws->primary || !ws->scratch) {
        throw std::runtime_error("mujoco_kkt_linearize: failed to allocate mjData");
    }

    const State4 fixed_point = evaluate_vector_field(model, ids, *ws, speed_mps, State4{});

    constexpr double kH1 = 1e-5;
    constexpr double kH2 = 5e-6;
    Mat4 j1{};
    Mat4 j2{};
    finite_diff_jacobian(model, ids, *ws, {.speed_mps = speed_mps, .h = kH1}, j1);
    finite_diff_jacobian(model, ids, *ws, {.speed_mps = speed_mps, .h = kH2}, j2);

    Mat4 j_rich{};
    for (std::size_t i = 0; i < j_rich.size(); ++i) {
        j_rich[i] = (4.0 * j2[i] - j1[i]) / 3.0;
    }

    return {
        .fixed_point_residual = fixed_point,
        .jacobian_rich = j_rich,
        .eigenvalues = real_4x4_eigenvalues(j_rich),
        .max_manifold_gap_m = ws->max_manifold_gap_m,
    };
}

Eigenvalues4 mujoco_kkt_eigenvalues(const mjModel* model, double speed_mps) {
    return mujoco_kkt_linearize(model, speed_mps).eigenvalues;
}

double find_weave_critical_speed_mps(const mjModel* model,
                                     SpeedBracket bracket,
                                     double tol_mps) {
    return bisect_critical_speed(
        model, bracket, tol_mps, "find_weave_critical_speed_mps", weave_real_part);
}

double find_capsize_critical_speed_mps(const mjModel* model,
                                       SpeedBracket bracket,
                                       double tol_mps) {
    return bisect_critical_speed(
        model, bracket, tol_mps, "find_capsize_critical_speed_mps", capsize_real_part);
}

} // namespace bike::spatial::whipple
