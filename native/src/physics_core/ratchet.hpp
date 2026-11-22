// native/src/physics_core/ratchet.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// ideal one-way dual ratchet bounds, 12-tendon cassette range filler, and synchronized 2*pi re-basing.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

BIKE_CONSTEXPR i32 kCassetteCogCount = 12;

template <class Real>
struct RatchetBoundState {
    Real boundary = BIKE_REAL(Real, 0.0);
    Real ratio = BIKE_REAL(Real, 1.0);
    u8 has_boundary = 0;
    u8 pad[3]{};
};

namespace detail {

struct Df32 {
    float hi = 0.0f;
    float lo = 0.0f;
};

BIKE_HD Df32 df32_quick_two_sum(float a, float b) noexcept {
#if defined(__CUDA_ARCH__)
    const float s = __fadd_rn(a, b);
    const float e = __fsub_rn(b, __fsub_rn(s, a));
#else
    const float s = a + b;
    const float e = b - (s - a);
#endif
    Df32 out{};
    out.hi = s;
    out.lo = e;
    return out;
}

BIKE_HD Df32 df32_two_sum(float a, float b) noexcept {
#if defined(__CUDA_ARCH__)
    const float s = __fadd_rn(a, b);
    const float bb = __fsub_rn(s, a);
    const float e = __fadd_rn(__fsub_rn(a, __fsub_rn(s, bb)), __fsub_rn(b, bb));
#else
    const float s = a + b;
    const float bb = s - a;
    const float e = (a - (s - bb)) + (b - bb);
#endif
    Df32 out{};
    out.hi = s;
    out.lo = e;
    return out;
}

BIKE_HD Df32 df32_two_diff(float a, float b) noexcept {
#if defined(__CUDA_ARCH__)
    const float s = __fsub_rn(a, b);
    const float bb = __fsub_rn(s, a);
    const float e = __fsub_rn(__fsub_rn(a, __fsub_rn(s, bb)), __fadd_rn(b, bb));
#else
    const float s = a - b;
    const float bb = s - a;
    const float e = (a - (s - bb)) - (b + bb);
#endif
    Df32 out{};
    out.hi = s;
    out.lo = e;
    return out;
}

BIKE_HD Df32 df32_two_prod(float a, float b) noexcept {
#if defined(__CUDA_ARCH__)
    const float p = __fmul_rn(a, b);
    const float e = __fmaf_rn(a, b, -p);
#else
    // Veltkamp split (s = 12 for IEEE-754 binary32: 2^12 + 1 = 4097.0f); FMA-contraction invariant.
    const float kSplit = 4097.0f;
    const float ca = kSplit * a;
    const float a_hi = ca - (ca - a);
    const float a_lo = a - a_hi;
    const float cb = kSplit * b;
    const float b_hi = cb - (cb - b);
    const float b_lo = b - b_hi;
    const float p = a * b;
    const float e = ((a_hi * b_hi - p) + a_hi * b_lo + a_lo * b_hi) + a_lo * b_lo;
#endif
    Df32 out{};
    out.hi = p;
    out.lo = e;
    return out;
}

BIKE_HD Df32 df32_add(Df32 x, Df32 y) noexcept {
    const Df32 s = df32_two_sum(x.hi, y.hi);
    const Df32 t = df32_two_sum(x.lo, y.lo);
#if defined(__CUDA_ARCH__)
    const Df32 v = df32_two_sum(s.hi, __fadd_rn(s.lo, t.hi));
    return df32_two_sum(v.hi, __fadd_rn(t.lo, v.lo));
#else
    const Df32 v = df32_two_sum(s.hi, s.lo + t.hi);
    return df32_two_sum(v.hi, t.lo + v.lo);
#endif
}

BIKE_HD Df32 df32_neg(Df32 x) noexcept {
    Df32 out{};
    out.hi = -x.hi;
    out.lo = -x.lo;
    return out;
}

BIKE_HD Df32 df32_sub(Df32 x, Df32 y) noexcept {
    return df32_add(x, df32_neg(y));
}

BIKE_HD float ratchet_phi_impl(float ratio, float q_driver, float q_driven) noexcept {
    Df32 driven_df{};
    driven_df.hi = q_driven;
    const Df32 phi = df32_sub(df32_two_prod(ratio, q_driver), driven_df);
    return phi.hi + phi.lo;
}

BIKE_HD float ratchet_shift_next_bound(
    float old_ratio,
    float new_ratio,
    float q_driver,
    float q_driven,
    bool has_boundary,
    float boundary
) noexcept {
    Df32 driven_df{};
    driven_df.hi = q_driven;
    const Df32 phi_old = df32_sub(df32_two_prod(old_ratio, q_driver), driven_df);
    const Df32 phi_new = df32_sub(df32_two_prod(new_ratio, q_driver), driven_df);
    if (!has_boundary) {
        return phi_new.hi + phi_new.lo;
    }
    Df32 bound_df{};
    bound_df.hi = boundary;
    const Df32 gap = df32_sub(bound_df, phi_old);
    const Df32 next = df32_add(phi_new, gap);
    return next.hi + next.lo;
}

BIKE_HD float rebase_freehub_bound(
    float r,
    float q_ds_old,
    float q_rw_old,
    float q_ds_new,
    float q_rw_new,
    float b_old
) noexcept {
    Df32 rw_old_df{};
    rw_old_df.hi = q_rw_old;
    Df32 rw_new_df{};
    rw_new_df.hi = q_rw_new;
    Df32 b_old_df{};
    b_old_df.hi = b_old;

    const Df32 phi_old = df32_sub(df32_two_prod(r, q_ds_old), rw_old_df);
    const Df32 gap = df32_sub(b_old_df, phi_old);
    const Df32 phi_new = df32_sub(df32_two_prod(r, q_ds_new), rw_new_df);
    const Df32 b_new = df32_add(phi_new, gap);
    return b_new.hi + b_new.lo;
}

BIKE_HD float rebase_latch_bound(
    float q_old,
    float q_new,
    float latch_old
) noexcept {
    Df32 q_new_df{};
    q_new_df.hi = q_new;
    const Df32 diff = df32_two_diff(latch_old, q_old);
    const Df32 out = df32_add(q_new_df, diff);
    return out.hi + out.lo;
}

#if !defined(__METAL_VERSION__)
BIKE_HD double ratchet_phi_impl(double ratio, double q_driver, double q_driven) noexcept {
    return ratio * q_driver - q_driven;
}

BIKE_HD double ratchet_shift_next_bound(
    double old_ratio,
    double new_ratio,
    double q_driver,
    double q_driven,
    bool has_boundary,
    double boundary
) noexcept {
    const double previous = old_ratio * q_driver - q_driven;
    const double current = new_ratio * q_driver - q_driven;
    return has_boundary ? (boundary + current - previous) : current;
}

BIKE_HD double rebase_freehub_bound(
    double r,
    double q_ds_old,
    double q_rw_old,
    double q_ds_new,
    double q_rw_new,
    double b_old
) noexcept {
    const double gap = b_old - (r * q_ds_old - q_rw_old);
    return (r * q_ds_new - q_rw_new) + gap;
}

BIKE_HD double rebase_latch_bound(
    double q_old,
    double q_new,
    double latch_old
) noexcept {
    return q_new + (latch_old - q_old);
}
#endif

} // namespace detail

template <class Real, class TK>
BIKE_HD Real ratchet_phi(Real ratio, Real q_driver, Real q_driven) noexcept {
    return detail::ratchet_phi_impl(ratio, q_driver, q_driven);
}

template <class Real, class TK>
BIKE_HD Status ratchet_reset_bound(
    Real ratio,
    Real q_driver,
    Real q_driven,
    BIKE_THREAD bool &has_boundary,
    BIKE_THREAD Real &boundary,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(ratio) || ratio <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(q_driver) || !TK::isfinite(q_driven)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real phi = ratchet_phi<Real, TK>(ratio, q_driver, q_driven);
    if (!TK::isfinite(phi)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    has_boundary = true;
    boundary = phi;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status ratchet_prepare_bound(
    Real ratio,
    Real q_driver,
    Real q_driven,
    BIKE_THREAD bool &has_boundary,
    BIKE_THREAD Real &boundary,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(ratio) || ratio <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(q_driver) || !TK::isfinite(q_driven) ||
        (has_boundary && !TK::isfinite(boundary))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real phi = ratchet_phi<Real, TK>(ratio, q_driver, q_driven);
    if (!TK::isfinite(phi)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    boundary = has_boundary ? TK::min(boundary, phi) : phi;
    has_boundary = true;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status ratchet_shift_ratio_bound(
    Real old_ratio,
    Real new_ratio,
    Real q_driver,
    Real q_driven,
    BIKE_THREAD bool &has_boundary,
    BIKE_THREAD Real &boundary,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(old_ratio) || old_ratio <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(new_ratio) || new_ratio <= BIKE_REAL(Real, 0.0) ||
        !TK::isfinite(q_driver) || !TK::isfinite(q_driven) ||
        (has_boundary && !TK::isfinite(boundary))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    const Real next_boundary = detail::ratchet_shift_next_bound(
        old_ratio, new_ratio, q_driver, q_driven, has_boundary, boundary);
    if (!TK::isfinite(next_boundary)) {
        err = ErrCode::overflow_nonfinite;
        return Status::overflow;
    }
    boundary = next_boundary;
    has_boundary = true;
    return Status::ok;
}

template <class Real, class TK, class PtrIds, class PtrRange>
BIKE_HD Status fill_cassette_tendon_ranges(
    i32 active_cog,
    i32 cog_count,
    i32 ntendon,
    Real active_upper_bound,
    PtrIds cassette_tendon_ids,
    PtrRange tendon_range,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (cog_count <= 0 || active_cog < 0 || active_cog >= cog_count ||
        ntendon <= 0 || !TK::isfinite(active_upper_bound) ||
        cassette_tendon_ids == nullptr || tendon_range == nullptr) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    for (i32 i = 0; i < cog_count; ++i) {
        const i32 tid = cassette_tendon_ids[i];
        if (tid < 0 || tid >= ntendon) {
            err = ErrCode::out_of_bounds_chain;
            return Status::out_of_bounds;
        }
    }
    const Real kLower = BIKE_REAL(Real, -1e12);
    const Real kInactiveUpper = BIKE_REAL(Real, 1e12);
    for (i32 i = 0; i < cog_count; ++i) {
        const i32 tid = cassette_tendon_ids[i];
        tendon_range[2 * tid + 0] = kLower;
        tendon_range[2 * tid + 1] =
            (i == active_cog) ? active_upper_bound : kInactiveUpper;
    }
    return Status::ok;
}

template <class Real>
struct WorldRotationsRebaseState {
    Real front_wheel_qpos = BIKE_REAL(Real, 0.0);
    Real rear_wheel_qpos = BIKE_REAL(Real, 0.0);
    Real crank_qpos = BIKE_REAL(Real, 0.0);
    // Carries the freehub driver qpos: `drive_shaft_spin` when `motor_clutch=true`,
    // or `crank_spin` (with `has_clutch_boundary=0`) when `motor_clutch=false`.
    Real drive_shaft_qpos = BIKE_REAL(Real, 0.0);
    Real freehub_ratio = BIKE_REAL(Real, 1.0);
    Real freehub_boundary = BIKE_REAL(Real, 0.0);
    Real clutch_boundary = BIKE_REAL(Real, 0.0);
    Real brake_latch[2]{};
    u8 has_freehub_boundary = 0;
    u8 has_clutch_boundary = 0;
    u8 has_brake_latch[2]{};
};

template <class Real, class TK, class StatePtr>
BIKE_HD Status rebase_world_rotations_2pi(
    StatePtr state,
    BIKE_THREAD ErrCode &err
) noexcept {
    err = ErrCode::none;
    if (!TK::isfinite(state->front_wheel_qpos) ||
        !TK::isfinite(state->rear_wheel_qpos) ||
        !TK::isfinite(state->crank_qpos) ||
        !TK::isfinite(state->drive_shaft_qpos) ||
        !TK::isfinite(state->freehub_ratio) ||
        state->freehub_ratio <= BIKE_REAL(Real, 0.0) ||
        (state->has_freehub_boundary != 0 && !TK::isfinite(state->freehub_boundary)) ||
        (state->has_clutch_boundary != 0 && !TK::isfinite(state->clutch_boundary)) ||
        (state->has_brake_latch[0] != 0 && !TK::isfinite(state->brake_latch[0])) ||
        (state->has_brake_latch[1] != 0 && !TK::isfinite(state->brake_latch[1]))) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }

    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi literal
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 2*pi literal
    const Real kTwoPi = BIKE_REAL(Real, 6.28318530717958647692);

    const Real fw_old = state->front_wheel_qpos;
    const Real rw_old = state->rear_wheel_qpos;
    const Real cr_old = state->crank_qpos;
    const Real ds_old = state->drive_shaft_qpos;

    const Real shift_front = TK::floor((fw_old + kPi) / kTwoPi) * kTwoPi;
    const Real shift_rear = TK::floor((rw_old + kPi) / kTwoPi) * kTwoPi;
    const Real shift_crank = TK::floor((cr_old + kPi) / kTwoPi) * kTwoPi;
    const Real shift_shaft = TK::floor((ds_old + kPi) / kTwoPi) * kTwoPi;

    const Real fw_new = fw_old - shift_front;
    const Real rw_new = rw_old - shift_rear;
    const Real cr_new = cr_old - shift_crank;
    const Real ds_new = ds_old - shift_shaft;

    state->front_wheel_qpos = fw_new;
    state->rear_wheel_qpos = rw_new;
    state->crank_qpos = cr_new;
    state->drive_shaft_qpos = ds_new;

    if (state->has_freehub_boundary != 0) {
        state->freehub_boundary = detail::rebase_freehub_bound(
            state->freehub_ratio, ds_old, rw_old, ds_new, rw_new, state->freehub_boundary);
    }
    if (state->has_clutch_boundary != 0) {
        state->clutch_boundary = detail::rebase_freehub_bound(
            BIKE_REAL(Real, 1.0), cr_old, ds_old, cr_new, ds_new, state->clutch_boundary);
    }
    if (state->has_brake_latch[0] != 0) {
        state->brake_latch[0] = detail::rebase_latch_bound(
            fw_old, fw_new, state->brake_latch[0]);
    }
    if (state->has_brake_latch[1] != 0) {
        state->brake_latch[1] = detail::rebase_latch_bound(
            rw_old, rw_new, state->brake_latch[1]);
    }
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
