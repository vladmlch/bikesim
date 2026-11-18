// writers/resistance.cpp — expression-for-expression port of
// physical_resistance.py's compute path; comments cite file:line at the
// time of porting.
//
// Bitwise contract, vector reductions: `a @ b` / `A @ x` / `np.linalg.norm`
// in CPython-numpy dispatch to BLAS — on this platform numpy links Apple
// Accelerate, whose ddot/dgemv use a vectorized multi-accumulator order a
// sequential C++ loop cannot reproduce. The writer calls the SAME legacy
// CBLAS entry points numpy resolves to, declared once at the ABI in
// cblas_abi.hpp because <Accelerate/Accelerate.h> marks them deprecated
// under -Werror (and its vecLib headers do not parse under gcc
// -fsyntax-only). Probes verified each call reproduces the numpy
// expression bitwise (2000 trials each):
//   a @ b            == cblas_ddot(n, a, 1, b, 1)
//   A(3,nv) @ x      == cblas_dgemv(RowMajor, NoTrans, 3, nv, 1, A, nv, ...)
//   A.T @ x (F-view) == cblas_dgemv(RowMajor, Trans,    3, nv, 1, A, nv, ...)
// `sum()` over floats is CPython 3.13's Neumaier compensated summation
// (Python/bltinmodule.c builtin_sum_impl, PyFloat_CheckExact fast path)
// — replicated below; elementwise ops keep IEEE per-element semantics
// under any SIMD numpy may use, so scalar loops stay bitwise-correct.
// The ported sites feed Python floats only, so no PyLong-addend naive
// steps are reachable; the isfinite guard on `c` is a local equivalent
// of the unconditional upstream update (a poisoned compensation is
// unreachable once t is non-finite — see numeric_sum.hpp).
#include "resistance.hpp"
#include "../cblas_abi.hpp"
#include "../config_validation.hpp"
#include "../model_access.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
    // CPython 3.13 builtin sum() on float items: Neumaier compensated
    // summation. The compensation term updates only while the running
    // total stays finite — a local guard, observationally equivalent to
    // CPython's unconditional update (numeric_sum.hpp has the proof);
    // f_result itself always takes the naive step. An empty/fully-filtered
    // iterable yields 0 — same value this returns (int 0 vs 0.0 compare equal
    // in the `Fn > 0` gate).
    double py_sum(std::span<const double> loads,
                  std::span<const bool> working) noexcept {
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

    template<typename T>
    [[nodiscard]] bool try_readonly_buffer(const T *data, mjtSize count,
                                           std::span<const T> &out) noexcept {
        if (count < 0 || (data == nullptr && count > 0))
            return false;
        out = std::views::counted(data, static_cast<std::ptrdiff_t>(count));
        return true;
    }

    // physical_mapping.py:6-11 — resolve_id. Site contract: BODY lookups
    // additionally reject the world body (id 0 can never host a force);
    // other kinds keep the plain name check — routed through the shared
    // resolver with the physical gate explicit.
    int resolve_id(const mjModel *m, mjtObj kind, const char *name) {
        return model_access::resolve_id(m, kind, name, kind == mjOBJ_BODY);
    }

    // external_resistance.py:21-23 — _rolling_moment (validation-free core).
    // math.tanh is libm tanh, same as std::tanh here.
    double rolling_moment_raw(double crr, double load, double radius,
                              double speed, double taper) noexcept {
        return -crr * load * radius * std::tanh(speed / taper);
    }

    // Non-throwing twin of model_access::point_jacobian_into with the
    // JacobianBody::physical_strict body gate (0 < body_id < nbody).
    [[nodiscard]] bool try_point_jacobian_into(
            const mjModel *m, const mjData *d, int body_id,
            const std::array<double, 3> &point, std::span<mjtNum> jp,
            const char *&err_msg) noexcept {
        for (const double v: point) {
            if (!std::isfinite(v)) {
                err_msg = "invalid world point";
                return false;
            }
        }
        if (!(0 < body_id && body_id < m->nbody)) {
            err_msg = "point Jacobian requires a physical body";
            return false;
        }
        if (m->nv < 0) {
            err_msg = "point Jacobian needs a nonnegative model width";
            return false;
        }
        const auto nv_u = static_cast<std::size_t>(m->nv);
        const auto k_xyz = static_cast<std::size_t>(model_access::kXYZ);
        if (jp.size() / k_xyz != nv_u || jp.size() % k_xyz != 0) {
            err_msg = "point Jacobian destination must be 3*nv elements";
            return false;
        }
        mj_jac(m, d, jp.data(), nullptr, point.data(), body_id);
        return true;
    }
} // namespace

ResistanceWriter::ResistanceWriter(const mjModel *m,
                                   nativecfg::ResistanceConfig config)
    : m_(m), cfg_(std::move(config)),
      nv_(static_cast<int>(m->nv)) {
    if (!cfg_.allow_spatial)
        nativecfg::validate(cfg_);
    // ResistanceConfig.__post_init__ (physical_config.py:275-284): scalars
    // finite + bounds, both 3-vectors finite and planar ([1] == 0 unless
    // allow_spatial is enabled for 3D motion).
    if (!std::isfinite(cfg_.crr) || cfg_.crr < 0.0)
        throw std::invalid_argument("invalid crr");
    if (!std::isfinite(cfg_.rho_kg_m3) || cfg_.rho_kg_m3 < 0.0)
        throw std::invalid_argument("invalid rho_kg_m3");
    if (!std::isfinite(cfg_.cda_m2) || cfg_.cda_m2 < 0.0)
        throw std::invalid_argument("invalid cda_m2");
    if (!std::isfinite(cfg_.rolling_taper_rad_s) ||
        cfg_.rolling_taper_rad_s <= 0.0)
        throw std::invalid_argument("invalid rolling taper");
    for (const double v: cfg_.wind_world_mps)
        if (!std::isfinite(v))
            throw std::invalid_argument(
                "invalid shape or non-finite wind_world_mps");
    for (const double v: cfg_.point_body_m)
        if (!std::isfinite(v))
            throw std::invalid_argument(
                "invalid shape or non-finite point_body_m");
    if (!cfg_.allow_spatial &&
        (cfg_.wind_world_mps[1] != 0.0 || cfg_.point_body_m[1] != 0.0))
        throw std::invalid_argument(
            "resistance configuration must be planar");
    // physical_resistance.py:13-15.
    frame_ = resolve_id(m, mjOBJ_BODY, cfg_.frame_body.c_str());
    front_wheel_ = resolve_id(m, mjOBJ_BODY, cfg_.front_wheel_body.c_str());
    rear_wheel_ = resolve_id(m, mjOBJ_BODY, cfg_.rear_wheel_body.c_str());
    jr_.resize(static_cast<std::size_t>(model_access::kXYZ) *
               static_cast<std::size_t>(nv_));
    jp_.resize(static_cast<std::size_t>(model_access::kXYZ) *
               static_cast<std::size_t>(nv_));
    // R1: the former per-call rolling/aerodynamic/per-side row vectors
    // are construction-sized members — compute fills this capacity.
    rolling_.resize(static_cast<std::size_t>(nv_));
    aerodynamic_.resize(static_cast<std::size_t>(nv_));
    row_.resize(static_cast<std::size_t>(nv_));
}

// Caller precondition (raw boundary): `d` is the live mjData of the model
// this writer was built on, driven by the single owning Stepper — widths
// are validated, model/data pairing is not derivable from them.
void ResistanceWriter::components_into(const mjData *d,
                                       const TireSideInput &front,
                                       const TireSideInput &rear,
                                       std::span<ForceComponentView> out) const {
    if (d == nullptr)
        throw std::invalid_argument(
            "resistance compute needs a live mjData");
    if (out.size() != kComponentCount)
        throw std::invalid_argument("resistance component view count");
    const char *err_msg = nullptr;
    bool overflow_err = false;
    const CoreStatus status =
        compute_core(d, front, rear, err_msg, overflow_err);
    if (status != CoreStatus::ok) {
        if (overflow_err)
            throw std::overflow_error(
                err_msg != nullptr ? err_msg : "resistance overflow");
        throw std::invalid_argument(
            err_msg != nullptr ? err_msg : "resistance compute failed");
    }
    export_views(out);
}

// The warm-core twin of components_into: runs compute_core directly with
// zero throw, zero try/catch, and zero heap allocation so it is proved
// noexcept and BIKE_NONBLOCKING.
CoreStatus ResistanceWriter::try_components_into(
        const mjData *d, const TireSideInput &front,
        const TireSideInput &rear,
        std::span<ForceComponentView> out) const noexcept {
    if (d == nullptr || out.size() != kComponentCount)
        return CoreStatus::invalid_input;
    const char *err_msg = nullptr;
    bool overflow_err = false;
    const CoreStatus status =
        compute_core(d, front, rear, err_msg, overflow_err);
    if (status != CoreStatus::ok)
        return status;
    export_views(out);
    return CoreStatus::ok;
}

std::span<const std::string_view>
ResistanceWriter::component_names() noexcept {
    static constexpr std::array<std::string_view, kComponentCount> names{
        "road_rolling", "aerodynamic"
    };
    return names;
}

void ResistanceWriter::export_views(
        std::span<ForceComponentView> out) const noexcept {
    out[0] = {.kind = ForceKind::resistance, .values = rolling_};
    out[1] = {.kind = ForceKind::resistance, .values = aerodynamic_};
}

// Python-boxed surface: the typed compute plus the named-component copy —
// identical values and insertion order to the original components().
std::vector<ResistanceWriter::Component> ResistanceWriter::components(
    const mjData *d, const TireSideInput &front,
    const TireSideInput &rear) const {
    std::array<ForceComponentView, kComponentCount> views;
    components_into(d, front, rear, views);
    return {
        {"road_rolling",
         std::vector<double>(views[0].values.begin(),
                             views[0].values.end())},
        {"aerodynamic",
         std::vector<double>(views[1].values.begin(),
                             views[1].values.end())}
    };
}

// Non-throwing arithmetic kernel shared by components_into and
// try_components_into — preserves exact binary64 operation order and
// reports all domain/overflow rejections via CoreStatus + err_msg /
// overflow_err.
CoreStatus ResistanceWriter::compute_core(const mjData *d,
                                          const TireSideInput &front,
                                          const TireSideInput &rear,
                                          const char *&err_msg,
                                          bool &overflow_err) const noexcept {
    err_msg = nullptr;
    overflow_err = false;
    if (d == nullptr) {
        err_msg = "resistance compute needs a live mjData";
        return CoreStatus::invalid_input;
    }
    if (front.patch_loads.size() != front.patch_working.size() ||
        rear.patch_loads.size() != rear.patch_working.size()) {
        err_msg =
            "resistance_components: patch_loads and patch_working must have "
            "equal length";
        return CoreStatus::invalid_input;
    }

    std::span<const mjtNum> xpos{};
    std::span<const mjtNum> xmat{};
    std::span<const mjtNum> qvel{};
    if (!try_readonly_buffer(d->xpos, model_access::kXYZ * m_->nbody, xpos) ||
        !try_readonly_buffer(
            d->xmat, model_access::kRotationElements * m_->nbody, xmat) ||
        !try_readonly_buffer(d->qvel, m_->nv, qvel)) {
        err_msg = "invalid mjData buffer extents";
        return CoreStatus::invalid_input;
    }
    // Bounded element reads (and element addresses) — -Wunsafe-buffer-usage
    // rejects raw indexing/arithmetic on the mjData pointers.
    const auto at = [](std::span<const mjtNum> s, int i) noexcept
        -> const mjtNum & {
        return s[static_cast<std::size_t>(i)];
    };

    std::ranges::fill(rolling_, 0.0);
    // self.wheels = {'front': ..., 'rear': ...} — iteration order matters:
    // both sides accumulate into the same vector.
    const std::array<std::pair<const TireSideInput *, int>, 2> sides{
        {
            {&front, front_wheel_}, {&rear, rear_wheel_}
        }
    };
    for (const auto &[snap_p, body]: sides) {
        const TireSideInput &snap = *snap_p;
        // Fn = sum(p.normal_load_n for p in snapshot.patches
        //          if p.working_surface)
        const double Fn = py_sum(snap.patch_loads, snap.patch_working);
        if (!std::isfinite(Fn)) {
            err_msg = "ExternalResistanceApplier.normal_load";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
        if (!(0 < body && body < m_->nbody) ||
            jr_.size() != static_cast<std::size_t>(model_access::kXYZ) *
                              static_cast<std::size_t>(nv_)) {
            err_msg = "point Jacobian requires a physical body";
            return CoreStatus::invalid_input;
        }
        // axis = data.xmat[body].reshape(3, 3)[:, 1] — the body's world
        // y-axis, column 1 of the row-major 3x3 block.
        const std::array<double, model_access::kXYZ> axis = {
            at(xmat, model_access::kRotationElements * body + 1),
            at(xmat, model_access::kRotationElements * body + 4),
            at(xmat, model_access::kRotationElements * body + 7)
        };
        mj_jac(m_, d, nullptr, jr_.data(), &at(xpos, model_access::kXYZ * body),
               body);
        // row = self._jr.T @ axis (F-order operand → gemv-trans path);
        // row_ is the shared per-side member — beta = 0 overwrites it.
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                    model_access::kXYZ, nv_, 1.0, jr_.data(), nv_,
                    axis.data(), 1, 0.0, row_.data(), 1);
        // omega = float(row @ data.qvel)
        const double omega =
                blas::ddot(nv_, row_.data(), 1, qvel.data(), 1);
        const double radius = snap.effective_radius_m;
        if (Fn > 0.0 && radius > 0.0) {
            const double moment =
                    rolling_moment_raw(cfg_.crr, Fn, radius, omega,
                                       cfg_.rolling_taper_rad_s);
            if (!std::isfinite(moment)) {
                err_msg = "rolling_moment.torque";
                overflow_err = true;
                return CoreStatus::invalid_input;
            }
            // rolling += moment * row — two separately-rounded elementwise
            // ops in numpy (temp product, then in-place add).
            for (int i = 0; i < nv_; ++i)
                rolling_[static_cast<std::size_t>(i)] +=
                        moment * row_[static_cast<std::size_t>(i)];
        }
    }

    // point = data.xpos[self.frame] + data.xmat[self.frame].reshape(3, 3)
    //         @ cfg.point_body_m
    std::array<double, model_access::kXYZ> rotp{};
    blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                model_access::kXYZ, model_access::kXYZ, 1.0,
                &at(xmat, model_access::kRotationElements * frame_),
                model_access::kXYZ,
                cfg_.point_body_m.data(), 1, 0.0, rotp.data(), 1);
    std::array<double, model_access::kXYZ> point{};
    for (int i = 0; i < model_access::kXYZ; ++i)
        point[static_cast<std::size_t>(i)] =
                at(xpos, model_access::kXYZ * frame_ + i) +
                rotp[static_cast<std::size_t>(i)];
    for (const double v: point) {
        if (!std::isfinite(v)) {
            err_msg = "ExternalResistanceApplier.point";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    if (!try_point_jacobian_into(m_, d, frame_, point, jp_, err_msg))
        return CoreStatus::invalid_input;
    // velocity = self._jp @ data.qvel
    std::array<double, model_access::kXYZ> velocity{};
    blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                model_access::kXYZ, nv_, 1.0, jp_.data(), nv_,
                qvel.data(), 1, 0.0, velocity.data(), 1);
    // relative = velocity - np.asarray(cfg.wind_world_mps)
    std::array<double, model_access::kXYZ> relative{};
    for (int i = 0; i < model_access::kXYZ; ++i)
        relative[static_cast<std::size_t>(i)] =
                velocity[static_cast<std::size_t>(i)] -
                cfg_.wind_world_mps[static_cast<std::size_t>(i)];

    // external_resistance.py:37-42 — _drag_force: planar gate, then
    // -.5*rho*cda*norm(v)*v. np.linalg.norm's fast path on a real 1-D
    // vector is `x.dot(x)` → cblas_ddot (numpy/linalg/_linalg.py:2767),
    // not a sequential fold — same-libcall contract applies here too.
    // np.sqrt is libm sqrt. (Distinct from the CPython vector_norm — see
    // numeric_norm.hpp; this site stays on the BLAS oracle.)
    if (!cfg_.allow_spatial && relative[1] != 0.0) {
        err_msg = "drag velocity must be planar X-Z";
        return CoreStatus::invalid_input;
    }
    const double norm = std::sqrt(
        blas::ddot(model_access::kXYZ, relative.data(), 1, relative.data(),
                   1));
    if (!std::isfinite(norm)) {
        err_msg = "ExternalResistanceApplier.velocity_norm";
        overflow_err = true;
        return CoreStatus::invalid_input;
    }
    const double t =
            ((-0.5 * cfg_.rho_kg_m3) * cfg_.cda_m2) * norm;
    std::array<double, model_access::kXYZ> force{};
    for (int i = 0; i < model_access::kXYZ; ++i)
        force[static_cast<std::size_t>(i)] =
                t * relative[static_cast<std::size_t>(i)];
    for (const double v: force) {
        if (!std::isfinite(v)) {
            err_msg = "ExternalResistanceApplier.drag_force";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }

    // 'aerodynamic': self._jp.T @ force (same gemv-trans path as row_
    // above); beta = 0 fully overwrites the member like the fresh vector.
    blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                model_access::kXYZ, nv_, 1.0, jp_.data(), nv_,
                force.data(), 1, 0.0, aerodynamic_.data(), 1);
    for (const double v: rolling_) {
        if (!std::isfinite(v)) {
            err_msg = "ExternalResistanceApplier.rolling";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    for (const double v: aerodynamic_) {
        if (!std::isfinite(v)) {
            err_msg = "ExternalResistanceApplier.aerodynamic";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    return CoreStatus::ok;
}
