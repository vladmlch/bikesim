// writers/resistance.cpp — expression-for-expression port of
// physical_resistance.py's compute path; comments cite file:line at the
// time of porting.
//
// Bitwise contract, vector reductions: `a @ b` / `A @ x` / `np.linalg.norm`
// in CPython-numpy dispatch to BLAS — on this platform numpy links Apple
// Accelerate, whose ddot/dgemv use a vectorized multi-accumulator order a
// sequential C++ loop cannot reproduce. The writer calls the SAME legacy
// CBLAS entry points numpy resolves to, declared here at the ABI because
// <Accelerate/Accelerate.h> marks them deprecated under -Werror (and its
// vecLib headers do not parse under gcc -fsyntax-only). Probes verified
// each call reproduces the numpy expression bitwise (2000 trials each):
//   a @ b            == cblas_ddot(n, a, 1, b, 1)
//   A(3,nv) @ x      == cblas_dgemv(RowMajor, NoTrans, 3, nv, 1, A, nv, ...)
//   A.T @ x (F-view) == cblas_dgemv(RowMajor, Trans,    3, nv, 1, A, nv, ...)
// `sum()` over floats is CPython 3.12+'s Neumaier compensated summation
// (Python/bltinmodule.c builtin_sum_impl, isfinite-guarded) — replicated
// below; elementwise ops keep IEEE per-element semantics under any SIMD
// numpy may use, so scalar loops stay bitwise-correct.
#include "resistance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <string>
#include <utility>

extern "C" {
// Legacy CBLAS ABI (still exported by Accelerate.framework; enum values are
// the stable classic CBLAS constants).
double cblas_ddot(int N, const double* X, int incX, const double* Y,
                  int incY);
void cblas_dgemv(int Order, int TransA, int M, int N, double alpha,
                 const double* A, int lda, const double* X, int incX,
                 double beta, double* Y, int incY);
}

namespace {

constexpr int kCblasRowMajor = 101;
constexpr int kCblasNoTrans = 111;
constexpr int kCblasTrans = 112;

// CPython builtin sum() on floats (3.12+): Neumaier compensated summation.
// The compensation term updates only while the running total stays finite;
// f_result itself always takes the naive step. An empty/fully-filtered
// iterable yields 0 — same value this returns (int 0 vs 0.0 compare equal
// in the `Fn > 0` gate).
double py_sum(std::span<const double> loads,
              std::span<const bool> working) {
    double f_result = 0.0;
    double c_sum = 0.0;
    for (std::size_t i = 0; i < loads.size(); ++i) {
        if (!working[i])
            continue;
        const double x = loads[i];
        const double t = f_result + x;
        if (std::isfinite(t)) {
            if (std::abs(f_result) >= std::abs(x))
                c_sum += (f_result - t) + x;
            else
                c_sum += (x - t) + f_result;
        }
        f_result = t;
    }
    return f_result + c_sum;
}

// physical_mapping.py:6-11 — resolve_id.
int resolve_id(const mjModel* m, mjtObj kind, const char* name) {
    const int result = mj_name2id(m, kind, name);
    if (result < 0)
        throw std::invalid_argument("model has no '" + std::string(name) +
                                    "'");
    return result;
}

// external_resistance.py:21-23 — _rolling_moment (validation-free core).
// math.tanh is libm tanh, same as std::tanh here.
double rolling_moment(double crr, double load, double radius, double speed,
                      double taper) {
    return -crr * load * radius * std::tanh(speed / taper);
}

// physical_mapping.py:24-37 — point_jacobian_into's checks + the jacp-only
// mj_jac call (the rotational half is skipped in the source too).
void point_jacobian_into(const mjModel* m, const mjData* d, int body_id,
                         const std::array<double, 3>& point, mjtNum* jp) {
    if (!std::ranges::all_of(
            point, [](double v) { return std::isfinite(v); }))
        throw std::invalid_argument("invalid world point");
    if (!(0 < body_id && body_id < m->nbody))
        throw std::invalid_argument(
            "point Jacobian requires a physical body");
    mj_jac(m, d, jp, nullptr, point.data(), body_id);
}

}  // namespace

ResistanceWriter::ResistanceWriter(const mjModel* m,
                                   nativecfg::ResistanceConfig config)
    : m_(m), cfg_(std::move(config)),
      nv_(static_cast<int>(m->nv)) {
    // ResistanceConfig.__post_init__ (physical_config.py:275-284): scalars
    // finite + bounds, both 3-vectors finite and planar ([1] == 0).
    if (!std::isfinite(cfg_.crr) || cfg_.crr < 0.0)
        throw std::invalid_argument("invalid crr");
    if (!std::isfinite(cfg_.rho_kg_m3) || cfg_.rho_kg_m3 < 0.0)
        throw std::invalid_argument("invalid rho_kg_m3");
    if (!std::isfinite(cfg_.cda_m2) || cfg_.cda_m2 < 0.0)
        throw std::invalid_argument("invalid cda_m2");
    if (!std::isfinite(cfg_.rolling_taper_rad_s) ||
        cfg_.rolling_taper_rad_s <= 0.0)
        throw std::invalid_argument("invalid rolling taper");
    for (const double v : cfg_.wind_world_mps)
        if (!std::isfinite(v))
            throw std::invalid_argument(
                "invalid shape or non-finite wind_world_mps");
    for (const double v : cfg_.point_body_m)
        if (!std::isfinite(v))
            throw std::invalid_argument(
                "invalid shape or non-finite point_body_m");
    if (cfg_.wind_world_mps[1] != 0.0 || cfg_.point_body_m[1] != 0.0)
        throw std::invalid_argument(
            "resistance configuration must be planar");
    // physical_resistance.py:13-15.
    frame_ = resolve_id(m, mjOBJ_BODY, cfg_.frame_body.c_str());
    front_wheel_ = resolve_id(m, mjOBJ_BODY, cfg_.front_wheel_body.c_str());
    rear_wheel_ = resolve_id(m, mjOBJ_BODY, cfg_.rear_wheel_body.c_str());
    jr_.resize(static_cast<std::size_t>(3) * static_cast<std::size_t>(nv_));
    jp_.resize(static_cast<std::size_t>(3) * static_cast<std::size_t>(nv_));
}

std::vector<ResistanceWriter::Component> ResistanceWriter::components(
        const mjData* d, const TireSideInput& front,
        const TireSideInput& rear) const {
    if (front.patch_loads.size() != front.patch_working.size() ||
        rear.patch_loads.size() != rear.patch_working.size())
        throw std::invalid_argument(
            "resistance_components: patch_loads and patch_working must have "
            "equal length");

    const std::span<const mjtNum> xpos =
        std::views::counted(d->xpos, 3 * m_->nbody);
    const std::span<const mjtNum> xmat =
        std::views::counted(d->xmat, 9 * m_->nbody);
    const std::span<const mjtNum> qvel = std::views::counted(d->qvel, m_->nv);
    // Bounded element reads (and element addresses) — -Wunsafe-buffer-usage
    // rejects raw indexing/arithmetic on the mjData pointers.
    const auto at = [](std::span<const mjtNum> s, int i) -> const mjtNum& {
        return s[static_cast<std::size_t>(i)];
    };

    std::vector<double> rolling(static_cast<std::size_t>(nv_), 0.0);
    // self.wheels = {'front': ..., 'rear': ...} — iteration order matters:
    // both sides accumulate into the same vector.
    const std::pair<const TireSideInput*, int> sides[2] = {
        {&front, front_wheel_}, {&rear, rear_wheel_}};
    for (const auto& [snap_p, body] : sides) {
        const TireSideInput& snap = *snap_p;
        // Fn = sum(p.normal_load_n for p in snapshot.patches
        //          if p.working_surface)
        const double Fn = py_sum(snap.patch_loads, snap.patch_working);
        // axis = data.xmat[body].reshape(3, 3)[:, 1] — the body's world
        // y-axis, column 1 of the row-major 3x3 block.
        const std::array<double, 3> axis = {
            at(xmat, 9 * body + 1), at(xmat, 9 * body + 4),
            at(xmat, 9 * body + 7)};
        mj_jac(m_, d, nullptr, jr_.data(), &at(xpos, 3 * body), body);
        // row = self._jr.T @ axis (F-order operand → gemv-trans path)
        std::vector<double> row(static_cast<std::size_t>(nv_));
        cblas_dgemv(kCblasRowMajor, kCblasTrans, 3, nv_, 1.0, jr_.data(), nv_,
                    axis.data(), 1, 0.0, row.data(), 1);
        // omega = float(row @ data.qvel)
        const double omega =
            cblas_ddot(nv_, row.data(), 1, qvel.data(), 1);
        const double radius = snap.effective_radius_m;
        if (Fn > 0.0 && radius > 0.0) {
            const double moment =
                rolling_moment(cfg_.crr, Fn, radius, omega,
                               cfg_.rolling_taper_rad_s);
            // rolling += moment * row — two separately-rounded elementwise
            // ops in numpy (temp product, then in-place add).
            for (int i = 0; i < nv_; ++i)
                rolling[static_cast<std::size_t>(i)] +=
                    moment * row[static_cast<std::size_t>(i)];
        }
    }

    // point = data.xpos[self.frame] + data.xmat[self.frame].reshape(3, 3)
    //         @ cfg.point_body_m
    std::array<double, 3> rotp;
    cblas_dgemv(kCblasRowMajor, kCblasNoTrans, 3, 3, 1.0,
                &at(xmat, 9 * frame_), 3,
                cfg_.point_body_m.data(), 1, 0.0, rotp.data(), 1);
    std::array<double, 3> point;
    for (int i = 0; i < 3; ++i)
        point[static_cast<std::size_t>(i)] =
            at(xpos, 3 * frame_ + i) + rotp[static_cast<std::size_t>(i)];
    point_jacobian_into(m_, d, frame_, point, jp_.data());
    // velocity = self._jp @ data.qvel
    std::array<double, 3> velocity;
    cblas_dgemv(kCblasRowMajor, kCblasNoTrans, 3, nv_, 1.0, jp_.data(), nv_,
                qvel.data(), 1, 0.0, velocity.data(), 1);
    // relative = velocity - np.asarray(cfg.wind_world_mps)
    std::array<double, 3> relative;
    for (int i = 0; i < 3; ++i)
        relative[static_cast<std::size_t>(i)] =
            velocity[static_cast<std::size_t>(i)] -
            cfg_.wind_world_mps[static_cast<std::size_t>(i)];

    // external_resistance.py:37-42 — _drag_force: planar gate, then
    // -.5*rho*cda*norm(v)*v. np.linalg.norm's fast path on a real 1-D
    // vector is `x.dot(x)` → cblas_ddot (numpy/linalg/_linalg.py:2767),
    // not a sequential fold — same-libcall contract applies here too.
    // np.sqrt is libm sqrt.
    if (relative[1] != 0.0)
        throw std::invalid_argument("drag velocity must be planar X-Z");
    const double norm = std::sqrt(
        cblas_ddot(3, relative.data(), 1, relative.data(), 1));
    const double t =
        ((-0.5 * cfg_.rho_kg_m3) * cfg_.cda_m2) * norm;
    std::array<double, 3> force;
    for (int i = 0; i < 3; ++i)
        force[static_cast<std::size_t>(i)] =
            t * relative[static_cast<std::size_t>(i)];

    // 'aerodynamic': self._jp.T @ force (same gemv-trans path as row above)
    std::vector<double> aerodynamic(static_cast<std::size_t>(nv_));
    cblas_dgemv(kCblasRowMajor, kCblasTrans, 3, nv_, 1.0, jp_.data(), nv_,
                force.data(), 1, 0.0, aerodynamic.data(), 1);
    return {{"road_rolling", std::move(rolling)},
            {"aerodynamic", std::move(aerodynamic)}};
}
