// native/src/physics_core/suspension.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// physical suspension force laws: ForkAirSpring, Charger3Damper, CoilShock,
// SuperDeluxeDamper (with HBO & lockout_firm), and shock end_stop.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: POD structs and positional arguments match the CUDA/MSL ABI. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real>
struct AirSpringParams {
    Real stanchion_inner_diam_mm = BIKE_REAL(Real, 0.0);
    Real total_travel_mm = BIKE_REAL(Real, 0.0);
    Real pos_chamber_length_mm = BIKE_REAL(Real, 0.0);
    Real neg_chamber_length_mm = BIKE_REAL(Real, 0.0);
    Real token_volume_cm3 = BIKE_REAL(Real, 0.0);
    i32 max_tokens = 0;
    Real gamma = BIKE_REAL(Real, 0.0);
    Real atm_pressure_pa = BIKE_REAL(Real, 0.0);
    i32 num_tokens = 0;
    Real gauge_pressure_psi = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct DamperCoreParams {
    i32 max_hsc = 0;
    i32 max_lsc = 0;
    i32 max_reb = 0;
    i32 hsc_clicks = 0;
    i32 lsc_clicks = 0;
    i32 rebound_clicks = 0;
    Real c_lsc_min = BIKE_REAL(Real, 0.0);
    Real c_lsc_max = BIKE_REAL(Real, 0.0);
    Real c_hsc_min = BIKE_REAL(Real, 0.0);
    Real c_hsc_max = BIKE_REAL(Real, 0.0);
    Real c_reb_min = BIKE_REAL(Real, 0.0);
    Real c_reb_max = BIKE_REAL(Real, 0.0);
    Real v_knee_comp = BIKE_REAL(Real, 0.0);
    Real v_knee_reb = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct Charger3Params {
    DamperCoreParams<Real> core{};
    Real total_travel_mm = BIKE_REAL(Real, 0.0);
    Real hbo_start_mm = BIKE_REAL(Real, 0.0);
    Real c_hbo_base = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct SuperDeluxeParams {
    DamperCoreParams<Real> core{};
    Real total_stroke_mm = BIKE_REAL(Real, 0.0);
    i32 max_hbo = 0;
    i32 hbo_clicks = 0;
    u8 lockout_firm = 0;
    u8 pad[3]{};
    Real hbo_start_mm = BIKE_REAL(Real, 0.0);
    Real c_hbo_min = BIKE_REAL(Real, 0.0);
    Real c_hbo_max = BIKE_REAL(Real, 0.0);
    Real lockout_preload_n = BIKE_REAL(Real, 0.0);
    Real lockout_stiffness = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct CoilParams {
    Real rate_n_m = BIKE_REAL(Real, 0.0);
    Real preload_mm = BIKE_REAL(Real, 0.0);
    Real stroke_mm = BIKE_REAL(Real, 0.0);
    Real bumper_length_mm = BIKE_REAL(Real, 0.0);
    Real bumper_peak_n = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct EndStopParams {
    Real stiffness_n_m = BIKE_REAL(Real, 0.0);
    Real damping_n_s_m = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct SuspensionParams {
    AirSpringParams<Real> air_spring{};
    Charger3Params<Real> fork_damper{};
    SuperDeluxeParams<Real> shock_damper{};
    CoilParams<Real> coil{};
    EndStopParams<Real> end_stops{};
};

enum class SuspDiagField : u8 {
    none = 0,
    air_displaced_volume,
    air_initial_pos_volume,
    air_initial_neg_volume,
    air_volume_exhausted,
    air_pos_volume,
    air_neg_volume,
    air_abs_pressure,
    air_volumes_nonpositive,
    air_pos_pressure,
    air_neg_pressure,
    air_axial_force,
    charger3_zero_hbo_denom,
    charger3_damper_force,
    coil_spring_force,
    coil_bumper_force,
    sd_nonfinite_state,
    sd_zero_hbo_denom,
    sd_base_force,
    sd_hbo_force,
    sd_total_force,
    end_stop_invalid_params,
    end_stop_raw_force,
    telemetry_force_or_energy,
    invalid_damper_clicks_denom
};

template <class Real>
struct SuspensionResult {
    Real fork_spring = BIKE_REAL(Real, 0.0);
    Real fork_damper = BIKE_REAL(Real, 0.0);
    Real shock_spring = BIKE_REAL(Real, 0.0);
    Real shock_bumper = BIKE_REAL(Real, 0.0);
    Real shock_damper = BIKE_REAL(Real, 0.0);
    Real shock_top_out = BIKE_REAL(Real, 0.0);
    Real shock_upper_stop = BIKE_REAL(Real, 0.0);
    Real shock_hbo = BIKE_REAL(Real, 0.0);
    Real shock_total = BIKE_REAL(Real, 0.0);
    Real top_out_energy = BIKE_REAL(Real, 0.0);
    Real upper_energy = BIKE_REAL(Real, 0.0);
    Real bumper_energy = BIKE_REAL(Real, 0.0);
    Real coil_energy = BIKE_REAL(Real, 0.0);
    Real err_v_disp_m3 = BIKE_REAL(Real, 0.0);
    Real err_v_pos0_m3 = BIKE_REAL(Real, 0.0);
    SuspDiagField diag_field = SuspDiagField::none;
};

namespace detail {

BIKE_HD i32 clamp_i32(i32 v, i32 lo, i32 hi) noexcept {
    return v < lo ? lo : (v > hi ? hi : v);
}

template <class Real, class TK, class AirPtr>
BIKE_HD Real air_piston_area_m2(AirPtr air) noexcept {
    // NOLINTNEXTLINE(modernize-use-std-numbers) MSL has no <numbers>; exact FP64/FP32 pi literal
    const Real kPi = BIKE_REAL(Real, 3.14159265358979323846);
    const Real r_m = (air->stanchion_inner_diam_mm / BIKE_REAL(Real, 2.0)) /
                     BIKE_REAL(Real, 1000.0);
    return kPi * TK::pow2(r_m);
}

template <class Real, class TK, class CorePtr>
BIKE_HD Status calc_damper_coeffs(
    CorePtr c,
    BIKE_THREAD Real &c_lsc,
    BIKE_THREAD Real &c_hsc,
    BIKE_THREAD Real &c_reb,
    BIKE_THREAD ErrCode &err,
    BIKE_THREAD SuspDiagField &diag
) noexcept {
    if (c->max_lsc <= 0 || c->max_hsc <= 0 || c->max_reb <= 0) {
        err = ErrCode::outside_domain;
        diag = SuspDiagField::invalid_damper_clicks_denom;
        return Status::invalid_argument;
    }
    const Real frac_lsc =
        static_cast<Real>(clamp_i32(c->lsc_clicks, 0, c->max_lsc)) /
        static_cast<Real>(c->max_lsc);
    const Real frac_hsc =
        static_cast<Real>(clamp_i32(c->hsc_clicks, 0, c->max_hsc)) /
        static_cast<Real>(c->max_hsc);
    const Real frac_reb =
        static_cast<Real>(clamp_i32(c->rebound_clicks, 0, c->max_reb)) /
        static_cast<Real>(c->max_reb);
    c_lsc = c->c_lsc_min +
            (c->c_lsc_max - c->c_lsc_min) * TK::pow_py(frac_lsc, BIKE_REAL(Real, 1.25));
    c_hsc = c->c_hsc_min +
            (c->c_hsc_max - c->c_hsc_min) * TK::pow_py(frac_hsc, BIKE_REAL(Real, 1.15));
    c_reb = c->c_reb_min +
            (c->c_reb_max - c->c_reb_min) * TK::pow_py(frac_reb, BIKE_REAL(Real, 1.20));
    return Status::ok;
}

template <class Real, class TK, class CorePtr>
BIKE_HD Real base_damping_force(
    CorePtr c,
    Real v,
    Real c_lsc,
    Real c_hsc,
    Real c_reb
) noexcept {
    if (v >= BIKE_REAL(Real, 0.0)) {
        if (v <= c->v_knee_comp) {
            return c_lsc * v *
                   (BIKE_REAL(Real, 1.0) + BIKE_REAL(Real, 0.35) * (v / c->v_knee_comp));
        }
        const Real f_knee = c_lsc * c->v_knee_comp * BIKE_REAL(Real, 1.35);
        const Real v_excess = v - c->v_knee_comp;
        return f_knee +
               c_hsc * TK::pow_py(v_excess, BIKE_REAL(Real, 0.88)) *
                   TK::pow_py(c->v_knee_comp, BIKE_REAL(Real, 0.12));
    }
    const Real v_abs = -v;
    if (v_abs <= c->v_knee_reb) {
        return -c_reb * v_abs *
               (BIKE_REAL(Real, 1.0) + BIKE_REAL(Real, 0.25) * (v_abs / c->v_knee_reb));
    }
    const Real f_knee = c_reb * c->v_knee_reb * BIKE_REAL(Real, 1.25);
    const Real v_excess = v_abs - c->v_knee_reb;
    const Real c_hs_reb = c_reb * BIKE_REAL(Real, 0.45);
    return -(f_knee +
             c_hs_reb * TK::pow_py(v_excess, BIKE_REAL(Real, 0.90)) *
                 TK::pow_py(c->v_knee_reb, BIKE_REAL(Real, 0.10)));
}

template <class Real, class TK>
BIKE_HD Status eval_end_stop(
    Real q,
    Real v,
    Real lo,
    Real hi,
    Real k,
    Real c,
    Real upper_boundary_force_n,
    Real upper_boundary_energy_j,
    BIKE_THREAD Real &f_out,
    BIKE_THREAD Real &e_out,
    BIKE_THREAD ErrCode &err,
    BIKE_THREAD SuspDiagField &diag
) noexcept {
    if (!TK::isfinite(q) || !TK::isfinite(v) || !TK::isfinite(lo) ||
        !TK::isfinite(hi) || !TK::isfinite(k) || !TK::isfinite(c) ||
        !TK::isfinite(upper_boundary_force_n) ||
        !TK::isfinite(upper_boundary_energy_j) || hi <= lo ||
        k < BIKE_REAL(Real, 0.0) || c < BIKE_REAL(Real, 0.0) ||
        upper_boundary_force_n < BIKE_REAL(Real, 0.0) ||
        upper_boundary_energy_j < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        diag = SuspDiagField::end_stop_invalid_params;
        return Status::invalid_argument;
    }

    const Real low_depth = TK::max(lo - q, BIKE_REAL(Real, 0.0));
    if (low_depth > BIKE_REAL(Real, 0.0)) {
        const Real raw_force = k * low_depth - c * v;
        if (!TK::isfinite(raw_force)) {
            err = ErrCode::overflow_nonfinite;
            diag = SuspDiagField::end_stop_raw_force;
            return Status::overflow;
        }
        f_out = TK::max(BIKE_REAL(Real, 0.0), raw_force);
        e_out = BIKE_REAL(Real, 0.5) * k * TK::pow2(low_depth);
        return Status::ok;
    }

    const Real high_depth = TK::max(q - hi, BIKE_REAL(Real, 0.0));
    if (high_depth > BIKE_REAL(Real, 0.0) ||
        (q == hi && upper_boundary_force_n > BIKE_REAL(Real, 0.0))) {
        const Real elastic_force = upper_boundary_force_n + k * high_depth;
        const Real raw_force = elastic_force + c * v;
        if (!TK::isfinite(raw_force)) {
            err = ErrCode::overflow_nonfinite;
            diag = SuspDiagField::end_stop_raw_force;
            return Status::overflow;
        }
        f_out = -TK::max(BIKE_REAL(Real, 0.0), raw_force);
        e_out = upper_boundary_energy_j +
                upper_boundary_force_n * high_depth +
                BIKE_REAL(Real, 0.5) * k * TK::pow2(high_depth);
        return Status::ok;
    }

    f_out = BIKE_REAL(Real, 0.0);
    e_out = BIKE_REAL(Real, 0.0);
    return Status::ok;
}

} // namespace detail

template <class Real, class TK, class AirPtr>
BIKE_HD Status eval_air_axial_force(
    AirPtr air,
    Real travel_mm,
    BIKE_THREAD Real &out_force,
    BIKE_THREAD Real &out_v_disp_m3,
    BIKE_THREAD Real &out_v_pos0_m3,
    BIKE_THREAD ErrCode &err,
    BIKE_THREAD SuspDiagField &diag
) noexcept {
    const Real kPsiToPa = BIKE_REAL(Real, 6894.757293168);
    const i32 tokens = detail::clamp_i32(air->num_tokens, 0, air->max_tokens);
    const Real travel_m =
        TK::max(BIKE_REAL(Real, 0.0), TK::min(air->total_travel_mm, travel_mm)) /
        BIKE_REAL(Real, 1000.0);
    const Real area = detail::air_piston_area_m2<Real, TK>(air);
    const Real v_pos_0 =
        area * (air->pos_chamber_length_mm / BIKE_REAL(Real, 1000.0)) -
        static_cast<Real>(tokens) * (air->token_volume_cm3 * BIKE_REAL(Real, 1e-6));
    const Real v_neg_0 =
        area * (air->neg_chamber_length_mm / BIKE_REAL(Real, 1000.0));
    const Real v_disp = area * travel_m;
    out_v_disp_m3 = v_disp;
    out_v_pos0_m3 = v_pos_0;

    if (!TK::isfinite(v_disp)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_displaced_volume;
        return Status::overflow;
    }
    if (!TK::isfinite(v_pos_0)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_initial_pos_volume;
        return Status::overflow;
    }
    if (!TK::isfinite(v_neg_0)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_initial_neg_volume;
        return Status::overflow;
    }
    if (v_pos_0 <= BIKE_REAL(Real, 0.0) || v_neg_0 <= BIKE_REAL(Real, 0.0) ||
        v_disp >= v_pos_0) {
        err = ErrCode::outside_domain;
        diag = SuspDiagField::air_volume_exhausted;
        return Status::invalid_argument;
    }

    const Real v_pos = v_pos_0 - v_disp;
    const Real v_neg = v_neg_0 + v_disp;
    if (!TK::isfinite(v_pos)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_pos_volume;
        return Status::overflow;
    }
    if (!TK::isfinite(v_neg)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_neg_volume;
        return Status::overflow;
    }
    const Real p_abs_0 = air->gauge_pressure_psi * kPsiToPa + air->atm_pressure_pa;
    if (!TK::isfinite(p_abs_0)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_abs_pressure;
        return Status::overflow;
    }
    if (v_pos <= BIKE_REAL(Real, 0.0) || v_neg <= BIKE_REAL(Real, 0.0)) {
        err = ErrCode::outside_domain;
        diag = SuspDiagField::air_volumes_nonpositive;
        return Status::invalid_argument;
    }

    const Real p_pos = p_abs_0 * TK::pow_py(v_pos_0 / v_pos, air->gamma);
    const Real p_neg = p_abs_0 * TK::pow_py(v_neg_0 / v_neg, air->gamma);
    if (!TK::isfinite(p_pos)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_pos_pressure;
        return Status::overflow;
    }
    if (!TK::isfinite(p_neg)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_neg_pressure;
        return Status::overflow;
    }
    const Real axial_force = area * (p_pos - p_neg);
    if (!TK::isfinite(axial_force)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::air_axial_force;
        return Status::overflow;
    }
    out_force = TK::max(BIKE_REAL(Real, 0.0), axial_force);
    return Status::ok;
}

template <class Real, class TK, class Charger3Ptr>
BIKE_HD Status eval_charger3_damping_force(
    Charger3Ptr d,
    Real velocity_mps,
    Real travel_mm,
    BIKE_THREAD Real &out_force,
    BIKE_THREAD ErrCode &err,
    BIKE_THREAD SuspDiagField &diag
) noexcept {
    Real c_lsc = BIKE_REAL(Real, 0.0);
    Real c_hsc = BIKE_REAL(Real, 0.0);
    Real c_reb = BIKE_REAL(Real, 0.0);
    const Status cst = detail::calc_damper_coeffs<Real, TK>(
        &d->core, c_lsc, c_hsc, c_reb, err, diag);
    if (cst != Status::ok) {
        return cst;
    }
    const Real v = velocity_mps;
    Real f_damp = detail::base_damping_force<Real, TK>(
        &d->core, v, c_lsc, c_hsc, c_reb);
    if (travel_mm > d->hbo_start_mm && v > BIKE_REAL(Real, 0.0)) {
        if (d->total_travel_mm == d->hbo_start_mm) {
            err = ErrCode::outside_domain;
            diag = SuspDiagField::charger3_zero_hbo_denom;
            return Status::invalid_argument;
        }
        const Real hbo_prog = TK::max(
            BIKE_REAL(Real, 0.0),
            TK::min(
                BIKE_REAL(Real, 1.0),
                (travel_mm - d->hbo_start_mm) / (d->total_travel_mm - d->hbo_start_mm)));
        f_damp += d->c_hbo_base * TK::pow2(hbo_prog) * v;
    }
    if (!TK::isfinite(f_damp)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::charger3_damper_force;
        return Status::overflow;
    }
    out_force = f_damp;
    return Status::ok;
}

template <class Real, class TK, class SdPtr>
BIKE_HD Status eval_sd_damping_components(
    SdPtr d,
    Real velocity_mps,
    Real stroke_mm,
    BIKE_THREAD Real &out_base_n,
    BIKE_THREAD Real &out_hbo_n,
    BIKE_THREAD ErrCode &err,
    BIKE_THREAD SuspDiagField &diag
) noexcept {
    const Real v = velocity_mps;
    if (!TK::isfinite(v) || !TK::isfinite(stroke_mm)) {
        err = ErrCode::outside_domain;
        diag = SuspDiagField::sd_nonfinite_state;
        return Status::invalid_argument;
    }
    if (d->max_hbo <= 0) {
        err = ErrCode::outside_domain;
        diag = SuspDiagField::invalid_damper_clicks_denom;
        return Status::invalid_argument;
    }
    Real c_lsc = BIKE_REAL(Real, 0.0);
    Real c_hsc = BIKE_REAL(Real, 0.0);
    Real c_reb = BIKE_REAL(Real, 0.0);
    const Status cst = detail::calc_damper_coeffs<Real, TK>(
        &d->core, c_lsc, c_hsc, c_reb, err, diag);
    if (cst != Status::ok) {
        return cst;
    }
    const Real frac_hbo =
        static_cast<Real>(detail::clamp_i32(d->hbo_clicks, 0, d->max_hbo)) /
        static_cast<Real>(d->max_hbo);
    const Real c_hbo =
        d->c_hbo_min +
        (d->c_hbo_max - d->c_hbo_min) * TK::pow_py(frac_hbo, BIKE_REAL(Real, 1.30));

    Real f_damp = BIKE_REAL(Real, 0.0);
    if (d->lockout_firm != 0 && v > BIKE_REAL(Real, 0.0)) {
        const Real knee = BIKE_REAL(Real, 0.03);
        const Real low = d->lockout_stiffness;
        const Real high = BIKE_REAL(Real, 1.8) * c_hsc;
        f_damp = (v < knee) ? (low * v) : (low * knee + high * (v - knee));
    } else {
        f_damp = detail::base_damping_force<Real, TK>(
            &d->core, v, c_lsc, c_hsc, c_reb);
    }

    Real hbo = BIKE_REAL(Real, 0.0);
    if (stroke_mm > d->hbo_start_mm && v > BIKE_REAL(Real, 0.0)) {
        if (d->total_stroke_mm == d->hbo_start_mm) {
            err = ErrCode::outside_domain;
            diag = SuspDiagField::sd_zero_hbo_denom;
            return Status::invalid_argument;
        }
        const Real fraction = TK::min(
            BIKE_REAL(Real, 1.0),
            (stroke_mm - d->hbo_start_mm) / (d->total_stroke_mm - d->hbo_start_mm));
        hbo = c_hbo * TK::pow2(fraction) * v;
    }

    if (!TK::isfinite(f_damp)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::sd_base_force;
        return Status::overflow;
    }
    if (!TK::isfinite(hbo)) {
        err = ErrCode::overflow_nonfinite;
        diag = SuspDiagField::sd_hbo_force;
        return Status::overflow;
    }
    out_base_n = f_damp;
    out_hbo_n = hbo;
    return Status::ok;
}

template <class Real, class TK, class ParamsPtr>
BIKE_HD Status eval_suspension(
    ParamsPtr cfg,
    Real fork_qpos_m,
    Real fork_qvel_mps,
    Real shock_qpos_m,
    Real shock_qvel_mps,
    BIKE_THREAD SuspensionResult<Real> &res,
    BIKE_THREAD ErrCode &err
) noexcept {
    res = SuspensionResult<Real>{};
    err = ErrCode::none;

    const Real travel_mm = fork_qpos_m * BIKE_REAL(Real, 1000.0);
    Real fork_spring = BIKE_REAL(Real, 0.0);
    Status st = eval_air_axial_force<Real, TK>(
        &cfg->air_spring, travel_mm, fork_spring,
        res.err_v_disp_m3, res.err_v_pos0_m3, err, res.diag_field);
    if (st != Status::ok) {
        return st;
    }

    Real fork_damper = BIKE_REAL(Real, 0.0);
    st = eval_charger3_damping_force<Real, TK>(
        &cfg->fork_damper, fork_qvel_mps, travel_mm,
        fork_damper, err, res.diag_field);
    if (st != Status::ok) {
        return st;
    }
    const Real fork_total = fork_spring + fork_damper;

    const Real stroke_mm = shock_qpos_m * BIKE_REAL(Real, 1000.0);
    const Real compression_mm =
        TK::max(BIKE_REAL(Real, 0.0), stroke_mm + cfg->coil.preload_mm);
    const Real shock_spring =
        cfg->coil.rate_n_m * compression_mm / BIKE_REAL(Real, 1000.0);
    if (!TK::isfinite(shock_spring)) {
        err = ErrCode::overflow_nonfinite;
        res.diag_field = SuspDiagField::coil_spring_force;
        return Status::overflow;
    }

    const Real stroke_limit_mm = cfg->coil.stroke_mm;
    Real shock_bumper = BIKE_REAL(Real, 0.0);
    if (stroke_mm <= stroke_limit_mm) {
        const Real excess_mm = TK::max(
            BIKE_REAL(Real, 0.0),
            stroke_mm - (cfg->coil.stroke_mm - cfg->coil.bumper_length_mm));
        shock_bumper =
            cfg->coil.bumper_peak_n *
            TK::pow2(excess_mm / cfg->coil.bumper_length_mm);
        if (!TK::isfinite(shock_bumper)) {
            err = ErrCode::overflow_nonfinite;
            res.diag_field = SuspDiagField::coil_bumper_force;
            return Status::overflow;
        }
    }

    Real sd_base = BIKE_REAL(Real, 0.0);
    Real sd_hbo = BIKE_REAL(Real, 0.0);
    st = eval_sd_damping_components<Real, TK>(
        &cfg->shock_damper, shock_qvel_mps, stroke_mm,
        sd_base, sd_hbo, err, res.diag_field);
    if (st != Status::ok) {
        return st;
    }
    const Real shock_damper = sd_base + sd_hbo;
    if (!TK::isfinite(shock_damper)) {
        err = ErrCode::overflow_nonfinite;
        res.diag_field = SuspDiagField::sd_total_force;
        return Status::overflow;
    }

    Real top_out_force = BIKE_REAL(Real, 0.0);
    Real top_out_energy = BIKE_REAL(Real, 0.0);
    Real upper_force = BIKE_REAL(Real, 0.0);
    Real upper_energy = BIKE_REAL(Real, 0.0);
    Real bumper_energy = BIKE_REAL(Real, 0.0);

    const Real stroke_m = stroke_mm / BIKE_REAL(Real, 1000.0);
    const Real limit_m = stroke_limit_mm / BIKE_REAL(Real, 1000.0);
    if (stroke_m < BIKE_REAL(Real, 0.0)) {
        st = detail::eval_end_stop<Real, TK>(
            stroke_m, shock_qvel_mps,
            BIKE_REAL(Real, 0.0), limit_m,
            cfg->end_stops.stiffness_n_m, cfg->end_stops.damping_n_s_m,
            BIKE_REAL(Real, 0.0), BIKE_REAL(Real, 0.0),
            top_out_force, top_out_energy, err, res.diag_field);
        if (st != Status::ok) {
            return st;
        }
    }
    const Real bumper_length_m = cfg->coil.bumper_length_mm / BIKE_REAL(Real, 1000.0);
    const Real bumper_peak_n = cfg->coil.bumper_peak_n;
    const Real bumper_full_energy =
        bumper_peak_n * bumper_length_m / BIKE_REAL(Real, 3.0);
    if (stroke_m <= limit_m) {
        const Real bumper_depth_m =
            TK::max(BIKE_REAL(Real, 0.0), stroke_m - (limit_m - bumper_length_m));
        bumper_energy =
            bumper_peak_n * TK::pow3(bumper_depth_m) /
            (BIKE_REAL(Real, 3.0) * TK::pow2(bumper_length_m));
    } else {
        st = detail::eval_end_stop<Real, TK>(
            stroke_m, shock_qvel_mps,
            BIKE_REAL(Real, 0.0), limit_m,
            cfg->end_stops.stiffness_n_m, cfg->end_stops.damping_n_s_m,
            bumper_peak_n, bumper_full_energy,
            upper_force, upper_energy, err, res.diag_field);
        if (st != Status::ok) {
            return st;
        }
    }

    const Real coil_compression_m =
        TK::max(BIKE_REAL(Real, 0.0),
                (stroke_mm + cfg->coil.preload_mm) / BIKE_REAL(Real, 1000.0));
    const Real coil_energy =
        BIKE_REAL(Real, 0.5) * cfg->coil.rate_n_m * TK::pow2(coil_compression_m);

    const Real shock_top_out = -top_out_force;
    const Real shock_upper_stop = -upper_force;
    const Real shock_total = shock_spring + shock_bumper + shock_damper +
                             shock_top_out + shock_upper_stop;

    if (!TK::isfinite(fork_total) || !TK::isfinite(shock_total) ||
        !TK::isfinite(top_out_force) || !TK::isfinite(top_out_energy) ||
        !TK::isfinite(upper_force) || !TK::isfinite(upper_energy) ||
        !TK::isfinite(bumper_energy) || !TK::isfinite(coil_energy)) {
        err = ErrCode::overflow_nonfinite;
        res.diag_field = SuspDiagField::telemetry_force_or_energy;
        return Status::overflow;
    }

    res.fork_spring = fork_spring;
    res.fork_damper = fork_damper;
    res.shock_spring = shock_spring;
    res.shock_bumper = shock_bumper;
    res.shock_damper = shock_damper;
    res.shock_top_out = shock_top_out;
    res.shock_upper_stop = shock_upper_stop;
    res.shock_hbo = sd_hbo;
    res.shock_total = shock_total;
    res.top_out_energy = top_out_energy;
    res.upper_energy = upper_energy;
    res.bumper_energy = bumper_energy;
    res.coil_energy = coil_energy;
    res.diag_field = SuspDiagField::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
