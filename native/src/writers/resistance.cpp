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
#include "../config_validation.hpp"
#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/resistance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <new>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
    // physical_mapping.py:6-11 — resolve_id. Site contract: BODY lookups
    // additionally reject the world body (id 0 can never host a force);
    // other kinds keep the plain name check — routed through the shared
    // resolver with the physical gate explicit.
    int resolve_id(const mjModel *m, mjtObj kind, const char *name) {
        return model_access::resolve_id(m, kind, name, kind == mjOBJ_BODY);
    }

    // physical_mapping.py:24-37 — point_jacobian_into's checks + the jacp-only
    // mj_jac call (the rotational half is skipped in the source too). The
    // shared checked boundary keeps this site's recorded contract verbatim —
    // Python's single `0 < body_id < nbody` gate → JacobianBody::
    // physical_strict — while replacing the raw destination pointer with a
    // span that gets the exact 3*nv check.
    void point_jacobian_into(const mjModel *m, const mjData *d, int body_id,
                             const std::array<double, 3> &point,
                             std::span<mjtNum> jp) {
        model_access::point_jacobian_into(
            m, d, body_id, point, jp,
            model_access::JacobianBody::physical_strict, "point Jacobian");
    }

    [[noreturn]] void throw_resistance_error(
            bike::physics_core::ResDiagField field) {
        using bike::physics_core::ResDiagField;
        switch (field) {
            case ResDiagField::normal_load:
                throw std::overflow_error(
                    "ExternalResistanceApplier.normal_load must be finite");
            case ResDiagField::rolling_moment:
                throw std::overflow_error(
                    "rolling_moment.torque must be finite");
            case ResDiagField::point:
                throw std::overflow_error(
                    "ExternalResistanceApplier.point must be finite");
            case ResDiagField::nonplanar_velocity:
                throw std::invalid_argument(
                    "drag velocity must be planar X-Z");
            case ResDiagField::velocity_norm:
                throw std::overflow_error(
                    "ExternalResistanceApplier.velocity_norm must be finite");
            case ResDiagField::drag_force:
                throw std::overflow_error(
                    "ExternalResistanceApplier.drag_force must be finite");
            case ResDiagField::rolling:
                throw std::overflow_error(
                    "ExternalResistanceApplier.rolling must be finite");
            case ResDiagField::aerodynamic:
                throw std::overflow_error(
                    "ExternalResistanceApplier.aerodynamic must be finite");
            case ResDiagField::invalid_nv:
            case ResDiagField::none:
                throw std::invalid_argument(
                    "invalid resistance evaluation state");
        }
    }
} // namespace

ResistanceWriter::ResistanceWriter(const mjModel *m,
                                   nativecfg::ResistanceConfig config)
    : m_(m), cfg_(std::move(config)),
      nv_(static_cast<int>(m->nv)) {
    nativecfg::validate(cfg_);
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
    for (const double v: cfg_.wind_world_mps)
        if (!std::isfinite(v))
            throw std::invalid_argument(
                "invalid shape or non-finite wind_world_mps");
    for (const double v: cfg_.point_body_m)
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
    compute_validated(d, front, rear);
    export_views(out);
}

// The warm-core twin of components_into: the same precondition gates
// (plus the per-side patch-shape gate the kernel also enforces) report
// CoreStatus::invalid_input instead of throwing, and residual kernel
// rejections are caught and mapped so no exception escapes — noexcept is
// proved by the catch-all. The kernel's own throw construction stays on
// the failure path only; a valid tick allocates nothing.
CoreStatus ResistanceWriter::try_components_into(
        const mjData *d, const TireSideInput &front,
        const TireSideInput &rear,
        std::span<ForceComponentView> out) const noexcept {
    if (d == nullptr || out.size() != kComponentCount ||
        front.patch_loads.size() != front.patch_working.size() ||
        rear.patch_loads.size() != rear.patch_working.size())
        return CoreStatus::invalid_input;
    try {
        compute_validated(d, front, rear);
    } catch (const std::bad_alloc &) {
        return CoreStatus::engine_failure;
    } catch (const engine::EngineFailure &) {
        // EngineFailure derives std::exception — it must be caught before
        // the generic mapping classifies an engine error as bad input.
        return CoreStatus::engine_failure;
    } catch (const std::exception &) {
        return CoreStatus::invalid_input;
    } catch (...) {
        return CoreStatus::engine_failure;
    }
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

// Extracted unchanged from the original components() body — including the
// patch-shape gate and the derived() overflow gates, whose exception
// types are part of the observable contract.
void ResistanceWriter::compute_validated(const mjData *d,
                                         const TireSideInput &front,
                                         const TireSideInput &rear) const {
    if (front.patch_loads.size() != front.patch_working.size() ||
        rear.patch_loads.size() != rear.patch_working.size())
        throw std::invalid_argument(
            "resistance_components: patch_loads and patch_working must have "
            "equal length");

    const std::span<const mjtNum> xpos =
            std::views::counted(d->xpos, model_access::kXYZ * m_->nbody);
    const std::span<const mjtNum> xmat = std::views::counted(
        d->xmat, model_access::kRotationElements * m_->nbody);
    const std::span<const mjtNum> qvel = std::views::counted(d->qvel, m_->nv);
    // Bounded element reads (and element addresses) — -Wunsafe-buffer-usage
    // rejects raw indexing/arithmetic on the mjData pointers.
    const auto at = [](std::span<const mjtNum> s, int i) -> const mjtNum & {
        return s[static_cast<std::size_t>(i)];
    };

    const bike::physics_core::ResistanceParams<double> params{
        .crr = cfg_.crr,
        .rolling_taper_rad_s = cfg_.rolling_taper_rad_s,
        .rho_kg_m3 = cfg_.rho_kg_m3,
        .cda_m2 = cfg_.cda_m2,
        .wind_world_mps = {cfg_.wind_world_mps[0], cfg_.wind_world_mps[1], cfg_.wind_world_mps[2]},
        .point_body_m = {cfg_.point_body_m[0], cfg_.point_body_m[1], cfg_.point_body_m[2]},
    };

    auto wheel_jr_fn = [&](bike::physics_core::i32 side,
                           const double *wheel_xpos,
                           double *out_jr,
                           bike::physics_core::ErrCode &) noexcept
        -> bike::physics_core::Status {
        const int body = (side == 0) ? front_wheel_ : rear_wheel_;
        mj_jac(m_, d, nullptr, out_jr, wheel_xpos, body);
        return bike::physics_core::Status::ok;
    };

    bool jac_invalid = false;
    auto frame_jp_fn = [&](const double *pt_ptr,
                           double *out_jp,
                           bike::physics_core::ErrCode &local_err) noexcept
        -> bike::physics_core::Status {
        (void)out_jp;
        const auto pt_span = std::views::counted(pt_ptr, model_access::kXYZ);
        const std::array<double, model_access::kXYZ> pt_arr{
            pt_span[0], pt_span[1], pt_span[2]};
        try {
            point_jacobian_into(m_, d, frame_, pt_arr, jp_);
        } catch (const std::invalid_argument &) {
            jac_invalid = true;
            local_err = bike::physics_core::ErrCode::out_of_bounds_chain;
            return bike::physics_core::Status::invalid_argument;
        } catch (...) {
            jac_invalid = true;
            local_err = bike::physics_core::ErrCode::out_of_bounds_chain;
            return bike::physics_core::Status::invalid_argument;
        }
        return bike::physics_core::Status::ok;
    };

    auto err = bike::physics_core::ErrCode::none;
    auto diag = bike::physics_core::ResDiagField::none;
    const auto st = bike::physics_core::eval_resistance<
        double, bike::physics_core::CpuExactToolkit>(
        nv_, &params, qvel.data(),
        front.patch_loads.data(), front.patch_working.data(),
        static_cast<bike::physics_core::i32>(front.patch_loads.size()),
        front.effective_radius_m,
        &at(xpos, model_access::kXYZ * front_wheel_),
        &at(xmat, model_access::kRotationElements * front_wheel_),
        rear.patch_loads.data(), rear.patch_working.data(),
        static_cast<bike::physics_core::i32>(rear.patch_loads.size()),
        rear.effective_radius_m,
        &at(xpos, model_access::kXYZ * rear_wheel_),
        &at(xmat, model_access::kRotationElements * rear_wheel_),
        &at(xpos, model_access::kXYZ * frame_),
        &at(xmat, model_access::kRotationElements * frame_),
        jr_.data(), jp_.data(), row_.data(),
        rolling_.data(), aerodynamic_.data(),
        wheel_jr_fn, frame_jp_fn, err, diag);
    if (st != bike::physics_core::Status::ok) {
        if (jac_invalid) {
            point_jacobian_into(m_, d, frame_, std::array<double, model_access::kXYZ>{}, jp_);
            throw std::invalid_argument("point Jacobian: invalid input");
        }
        throw_resistance_error(diag);
    }
}
