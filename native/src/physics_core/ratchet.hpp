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

template <class Real, class TK>
BIKE_HD Real ratchet_phi(Real ratio, Real q_driver, Real q_driven) noexcept {
    return ratio * q_driver - q_driven;
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
    const Real previous = ratchet_phi<Real, TK>(old_ratio, q_driver, q_driven);
    const Real current = ratchet_phi<Real, TK>(new_ratio, q_driver, q_driven);
    const Real next_boundary =
        has_boundary ? (boundary + current - previous) : current;
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

    const Real shift_front =
        TK::floor((state->front_wheel_qpos + kPi) / kTwoPi) * kTwoPi;
    const Real shift_rear =
        TK::floor((state->rear_wheel_qpos + kPi) / kTwoPi) * kTwoPi;
    const Real shift_crank =
        TK::floor((state->crank_qpos + kPi) / kTwoPi) * kTwoPi;
    const Real shift_shaft =
        TK::floor((state->drive_shaft_qpos + kPi) / kTwoPi) * kTwoPi;

    state->front_wheel_qpos -= shift_front;
    state->rear_wheel_qpos -= shift_rear;
    state->crank_qpos -= shift_crank;
    state->drive_shaft_qpos -= shift_shaft;

    if (state->has_freehub_boundary != 0) {
        state->freehub_boundary -= (state->freehub_ratio * shift_shaft - shift_rear);
    }
    if (state->has_clutch_boundary != 0) {
        state->clutch_boundary -= (shift_crank - shift_shaft);
    }
    if (state->has_brake_latch[0] != 0) {
        state->brake_latch[0] -= shift_front;
    }
    if (state->has_brake_latch[1] != 0) {
        state->brake_latch[1] -= shift_rear;
    }
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
