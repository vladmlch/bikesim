// native/src/physics_core/tyre_tabulated.hpp — Tri-target (Host C++20/23, CUDA,
// Apple Metal MSL) tabulated radial force-deflection tyre kernel matching
// contactlaw::TabulatedRadial with 0-bit CPU FP64 parity.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel header: MSL has no std::span/std::array and device buffers are address-space-qualified raw pointers. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kMaxRadialTableKnots = 64;

template <class Real>
struct TabulatedRadialParams {
    i32 knot_count = 0;
    Real deflection_m[kMaxRadialTableKnots]{};
    Real force_n[kMaxRadialTableKnots]{};
    Real slopes[kMaxRadialTableKnots - 1]{};
    Real integral[kMaxRadialTableKnots]{};
};

template <class Real, class TK, class DeflPtr, class ForcePtr>
BIKE_HD Status init_tabulated_radial_params(
    i32 knot_count,
    DeflPtr deflection_m,
    ForcePtr force_n,
    BIKE_REF(TabulatedRadialParams<Real>) out,
    BIKE_REF(ErrCode) err
) noexcept {
    out = TabulatedRadialParams<Real>{};
    if (knot_count < 2) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    if (knot_count > kMaxRadialTableKnots) {
        err = ErrCode::outside_domain;
        return Status::out_of_bounds;
    }
    if (deflection_m == nullptr || force_n == nullptr) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(deflection_m[0]) || !TK::isfinite(force_n[0]) ||
        deflection_m[0] != static_cast<Real>(0) ||
        force_n[0] != static_cast<Real>(0)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    out.knot_count = knot_count;
    out.deflection_m[0] = deflection_m[0];
    out.force_n[0] = force_n[0];
    out.integral[0] = static_cast<Real>(0);

    for (i32 i = 1; i < knot_count; ++i) {
        const Real x_cur = deflection_m[i];
        const Real f_cur = force_n[i];
        const Real x_prev = deflection_m[i - 1];
        const Real f_prev = force_n[i - 1];
        if (!TK::isfinite(x_cur) || !TK::isfinite(f_cur) ||
            x_cur <= x_prev || f_cur < f_prev) {
            err = ErrCode::outside_domain;
            return Status::invalid_argument;
        }
        const Real dx = x_cur - x_prev;
        const Real slope = TK::div(f_cur - f_prev, dx);
        const Real integ =
            out.integral[i - 1] +
            BIKE_REAL(Real, 0.5) * (f_prev + f_cur) * dx;
        if (!TK::isfinite(slope) || !TK::isfinite(integ)) {
            err = ErrCode::overflow_nonfinite;
            return Status::overflow;
        }
        out.deflection_m[i] = x_cur;
        out.force_n[i] = f_cur;
        out.slopes[i - 1] = slope;
        out.integral[i] = integ;
    }

    if (!(out.force_n[knot_count - 1] > static_cast<Real>(0))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    class TK,
    class DeflPtr,
    class ForcePtr,
    class SlopePtr,
    class IntegPtr>
BIKE_HD Status tabulated_radial_elastic_arrays(
    i32 knot_count,
    DeflPtr deflection_m,
    ForcePtr force_n,
    SlopePtr slopes,
    IntegPtr integral,
    Real penetration_m,
    BIKE_REF(Real) out_force,
    BIKE_REF(Real) out_energy,
    BIKE_REF(ErrCode) err
) noexcept {
    out_force = static_cast<Real>(0);
    out_energy = static_cast<Real>(0);
    if (!TK::isfinite(penetration_m)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (penetration_m <= static_cast<Real>(0)) {
        err = ErrCode::none;
        return Status::ok;
    }
    if (knot_count < 2 || deflection_m == nullptr || force_n == nullptr ||
        slopes == nullptr || integral == nullptr) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    if (penetration_m > deflection_m[knot_count - 1]) {
        err = ErrCode::outside_domain;
        return Status::out_of_bounds;
    }

    // Matches std::ranges::upper_bound on deflection_m[0..knot_count).
    i32 first = 0;
    i32 len = knot_count;
    while (len > 0) {
        const i32 step = len / 2;
        const i32 mid = first + step;
        if (!(penetration_m < deflection_m[mid])) {
            first = mid + 1;
            len -= step + 1;
        } else {
            len = step;
        }
    }

    const i32 max_seg = knot_count - 2;
    const i32 raw_idx = first - 1;
    const i32 idx = raw_idx < 0 ? 0 : (raw_idx > max_seg ? max_seg : raw_idx);
    const Real dx = penetration_m - deflection_m[idx];
    const Real force = force_n[idx] + slopes[idx] * dx;
    const Real energy =
        integral[idx] + force_n[idx] * dx +
        BIKE_REAL(Real, 0.5) * slopes[idx] * dx * dx;
    if (!TK::isfinite(force) || !TK::isfinite(energy)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_force = force;
    out_energy = energy;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK, class TablePtr>
BIKE_HD Status tabulated_radial_elastic(
    TablePtr table,
    Real penetration_m,
    BIKE_REF(Real) out_force,
    BIKE_REF(Real) out_energy,
    BIKE_REF(ErrCode) err
) noexcept {
    if (table == nullptr || table->knot_count < 2 ||
        table->knot_count > kMaxRadialTableKnots) {
        out_force = static_cast<Real>(0);
        out_energy = static_cast<Real>(0);
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    return tabulated_radial_elastic_arrays<Real, TK>(
        table->knot_count,
        table->deflection_m,
        table->force_n,
        table->slopes,
        table->integral,
        penetration_m,
        out_force,
        out_energy,
        err);
}

template <
    class Real,
    class TK,
    class DeflPtr,
    class ForcePtr,
    class SlopePtr,
    class IntegPtr>
BIKE_HD Status tabulated_radial_contact_arrays(
    i32 knot_count,
    DeflPtr deflection_m,
    ForcePtr force_n,
    SlopePtr slopes,
    IntegPtr integral,
    Real penetration_m,
    Real rate_mps,
    Real loss_factor,
    Real hysteresis_rate_eps_mps,
    BIKE_REF(Real) out_force,
    BIKE_REF(Real) out_energy,
    BIKE_REF(ErrCode) err
) noexcept {
    out_force = static_cast<Real>(0);
    out_energy = static_cast<Real>(0);
    if (!TK::isfinite(rate_mps)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    if (!TK::isfinite(loss_factor) || loss_factor < static_cast<Real>(0) ||
        !TK::isfinite(hysteresis_rate_eps_mps) ||
        hysteresis_rate_eps_mps <= static_cast<Real>(0)) {
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    Real spring = static_cast<Real>(0);
    Real energy = static_cast<Real>(0);
    const Status st = tabulated_radial_elastic_arrays<Real, TK>(
        knot_count, deflection_m, force_n, slopes, integral,
        penetration_m, spring, energy, err);
    if (st != Status::ok) {
        return st;
    }
    if (penetration_m <= static_cast<Real>(0)) {
        err = ErrCode::none;
        return Status::ok;
    }
    const Real total =
        spring +
        loss_factor * spring *
            TK::tanh(TK::div(rate_mps, hysteresis_rate_eps_mps));
    if (!TK::isfinite(total)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    out_force = TK::max(static_cast<Real>(0), total);
    out_energy = energy;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK, class TablePtr>
BIKE_HD Status tabulated_radial_contact(
    TablePtr table,
    Real penetration_m,
    Real rate_mps,
    Real loss_factor,
    Real hysteresis_rate_eps_mps,
    BIKE_REF(Real) out_force,
    BIKE_REF(Real) out_energy,
    BIKE_REF(ErrCode) err
) noexcept {
    if (table == nullptr || table->knot_count < 2 ||
        table->knot_count > kMaxRadialTableKnots) {
        out_force = static_cast<Real>(0);
        out_energy = static_cast<Real>(0);
        err = ErrCode::missing_material;
        return Status::invalid_argument;
    }
    return tabulated_radial_contact_arrays<Real, TK>(
        table->knot_count,
        table->deflection_m,
        table->force_n,
        table->slopes,
        table->integral,
        penetration_m,
        rate_mps,
        loss_factor,
        hysteresis_rate_eps_mps,
        out_force,
        out_energy,
        err);
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
