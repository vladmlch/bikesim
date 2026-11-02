// native/src/gpu/suspension_kernel.hpp — Allocation-free C++/CUDA GPU suspension
// force kernels ported from native/src/writers/suspension.cpp.
#pragma once

#include "common.hpp"

#if !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
#include <numbers>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI; positional float signatures mirror the CUDA/MSL kernels. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::gpu {

BIKE_CONSTEXPR float kPsiToPa = 6894.757293168f;
#if !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
BIKE_CONSTEXPR float kPi = std::numbers::pi_v<float>;
#else
BIKE_CONSTEXPR float kPi = 3.14159265358979323846f;
#endif

struct GpuAirSpringParams {
    float stanchion_inner_diam_mm = 24.0f;
    float pos_chamber_length_mm = 280.0f;
    float neg_chamber_length_mm = 55.0f;
    float wheel_travel_mm = 180.0f;
    float total_travel_mm = 180.0f;
    float gamma = 1.40f;
    float atm_pressure_pa = 101325.0f;
    float token_volume_cm3 = 8.0f;
    int max_tokens = 4;
    float gauge_pressure_psi = 85.2f;
    int num_tokens = 2;
    float sag_fraction = 0.20f;
};

struct GpuDamperParams {
    float c_lsc_min = 450.0f;
    float c_lsc_max = 1800.0f;
    float c_hsc_min = 250.0f;
    float c_hsc_max = 900.0f;
    float c_reb_min = 600.0f;
    float c_reb_max = 2400.0f;
    float v_knee = 0.5f;
    int lsc_clicks = 7;
    int hsc_clicks = 2;
    int rebound_clicks = 9;
    int max_lsc = 14;
    int max_hsc = 4;
    int max_reb = 17;
};

struct GpuForkDamperParams {
    GpuDamperParams core{};
    float hbo_engagement_mm = 36.0f;
    float hbo_c_max = 3500.0f;
    float hbo_exponent = 2.0f;
    float total_travel_mm = 180.0f;
};

struct GpuCoilShockParams {
    float stroke_mm = 65.0f;
    float bumper_thickness_mm = 10.0f;
    float bumper_stiffness_n_m = 50000.0f;
    float bumper_quadratic_n_m2 = 7.0e7f;
    float spring_rate_n_mm = 114.6f;
    float preload_mm = 0.0f;
    float sag_fraction = 0.30f;
};

struct GpuSuspensionParams {
    GpuAirSpringParams fork_air{};
    GpuForkDamperParams fork_damper{};
    GpuCoilShockParams shock_coil{};
    GpuDamperParams shock_damper{};
    float fork_top_out_stiffness_n_m = 150000.0f;
    float fork_top_out_damping_ns_m = 1200.0f;
    float shock_top_out_stiffness_n_m = 250000.0f;
    float shock_top_out_damping_ns_m = 2000.0f;
    float fork_preload_n = 0.0f;
    float shock_preload_n = 0.0f;
};

struct GpuSuspensionOutput {
    float fork_spring_n = 0.0f;
    float fork_damper_n = 0.0f;
    float shock_coil_n = 0.0f;
    float shock_damper_n = 0.0f;
    float shock_bumper_n = 0.0f;
    float shock_top_out_n = 0.0f;
    float total_fork_force_n = 0.0f;
    float total_shock_force_n = 0.0f;
};

// Polytropic dual-chamber air spring axial force (ported from writers/suspension.cpp::air_axial_force).
BIKE_HD float gpu_air_axial_force(const BIKE_THREAD GpuAirSpringParams &air, float travel_mm) noexcept {
    const int tokens = air.num_tokens < 0
        ? 0
        : (air.num_tokens > air.max_tokens ? air.max_tokens : air.num_tokens);
    const float max_travel_mm = air.total_travel_mm > 0.0f ? air.total_travel_mm : air.wheel_travel_mm;
    const float clamped_travel_mm = gpu_clamp(travel_mm, 0.0f, max_travel_mm);
    const float travel_m = clamped_travel_mm * 1e-3f;

    const float r_m = (air.stanchion_inner_diam_mm * 0.5f) * 1e-3f;
    const float area = kPi * r_m * r_m;
    const float token_vol_m3 = air.token_volume_cm3 * 1e-6f;
    const float v_pos_0 = area * (air.pos_chamber_length_mm * 1e-3f) -
                          static_cast<float>(tokens) * token_vol_m3;
    const float v_neg_0 = area * (air.neg_chamber_length_mm * 1e-3f);
    if (v_pos_0 <= 0.0f || v_neg_0 <= 0.0f) {
        return 0.0f;
    }

    const float v_disp = area * travel_m;
    const float v_pos = gpu_max(v_pos_0 - v_disp, 1e-9f);
    const float v_neg = gpu_max(v_neg_0 + v_disp, 1e-9f);

    const float p_abs_0 = air.gauge_pressure_psi * kPsiToPa + air.atm_pressure_pa;
    const float p_pos = p_abs_0 * gpu_pow(v_pos_0 / v_pos, air.gamma);
    const float p_neg = p_abs_0 * gpu_pow(v_neg_0 / v_neg, air.gamma);

    return gpu_max(0.0f, area * (p_pos - p_neg));
}

// Base digressive LSC/HSC compression and rebound damper (ported from writers/suspension.cpp::base_damping).
BIKE_HD float gpu_shock_damper_force(const BIKE_THREAD GpuDamperParams &d, float stroke_vel_mps) noexcept {
    const int max_lsc = d.max_lsc > 0 ? d.max_lsc : 1;
    const int max_hsc = d.max_hsc > 0 ? d.max_hsc : 1;
    const int max_reb = d.max_reb > 0 ? d.max_reb : 1;

    const float frac_lsc = gpu_clamp(static_cast<float>(d.lsc_clicks), 0.0f, static_cast<float>(max_lsc)) /
                           static_cast<float>(max_lsc);
    const float frac_hsc = gpu_clamp(static_cast<float>(d.hsc_clicks), 0.0f, static_cast<float>(max_hsc)) /
                           static_cast<float>(max_hsc);
    const float frac_reb = gpu_clamp(static_cast<float>(d.rebound_clicks), 0.0f, static_cast<float>(max_reb)) /
                           static_cast<float>(max_reb);

    const float c_lsc = d.c_lsc_min + (d.c_lsc_max - d.c_lsc_min) * gpu_pow(frac_lsc, 1.25f);
    const float c_hsc = d.c_hsc_min + (d.c_hsc_max - d.c_hsc_min) * gpu_pow(frac_hsc, 1.15f);
    const float c_reb = d.c_reb_min + (d.c_reb_max - d.c_reb_min) * gpu_pow(frac_reb, 1.20f);

    const float v_knee = gpu_max(d.v_knee, 1e-6f);
    const float v = stroke_vel_mps;

    if (v >= 0.0f) {
        if (v <= v_knee) {
            return c_lsc * v * (1.0f + 0.35f * (v / v_knee));
        }
        const float f_knee = c_lsc * v_knee * 1.35f;
        const float v_excess = v - v_knee;
        return f_knee + c_hsc * gpu_pow(v_excess, 0.88f) * gpu_pow(v_knee, 0.12f);
    }

    const float v_abs = -v;
    if (v_abs <= v_knee) {
        return -c_reb * v_abs * (1.0f + 0.25f * (v_abs / v_knee));
    }
    const float f_knee = c_reb * v_knee * 1.25f;
    const float v_excess = v_abs - v_knee;
    const float c_hs_reb = c_reb * 0.45f;
    return -(f_knee + c_hs_reb * gpu_pow(v_excess, 0.90f) * gpu_pow(v_knee, 0.10f));
}

// Charger3 fork damper with position-sensitive hydraulic bottom-out (HBO)
// (ported from writers/suspension.cpp::charger3_damping_force).
BIKE_HD float gpu_fork_damper_force(
    const BIKE_THREAD GpuForkDamperParams &d,
    float compression_vel_mps,
    float travel_mm
) noexcept {
    float f_damp = gpu_shock_damper_force(d.core, compression_vel_mps);
    const float hbo_start_mm = (d.hbo_engagement_mm <= 0.5f * d.total_travel_mm)
        ? (d.total_travel_mm - d.hbo_engagement_mm)
        : d.hbo_engagement_mm;

    if (travel_mm > hbo_start_mm && compression_vel_mps > 0.0f) {
        const float hbo_zone_mm = d.total_travel_mm - hbo_start_mm;
        if (hbo_zone_mm > 1e-6f) {
            const float hbo_prog = gpu_clamp((travel_mm - hbo_start_mm) / hbo_zone_mm, 0.0f, 1.0f);
            f_damp += d.hbo_c_max * gpu_pow(hbo_prog, d.hbo_exponent) * compression_vel_mps;
        }
    }
    return f_damp;
}

// Linear coil spring with preload (ported from writers/suspension.cpp::coil_spring_force).
BIKE_HD float gpu_shock_coil_force(const BIKE_THREAD GpuCoilShockParams &coil, float stroke_mm) noexcept {
    const float compression_mm = gpu_max(0.0f, stroke_mm + coil.preload_mm);
    return coil.spring_rate_n_mm * compression_mm;
}

// Elastomer bottom-out bumper with linear + quadratic stiffness
// (ported from writers/suspension.cpp::coil_bumper_force).
BIKE_HD float gpu_shock_bumper_force(
    const BIKE_THREAD GpuCoilShockParams &coil,
    float stroke_mm,
    [[maybe_unused]] float stroke_vel_mps
) noexcept {
    const float engage_mm = coil.stroke_mm - coil.bumper_thickness_mm;
    if (stroke_mm <= engage_mm) {
        return 0.0f;
    }
    const float pen_m = (stroke_mm - engage_mm) * 1e-3f;
    return coil.bumper_stiffness_n_m * pen_m + coil.bumper_quadratic_n_m2 * pen_m * pen_m;
}

// Unilateral top-out stop when travel_m < 0 (ported from writers/suspension.cpp::end_stop).
BIKE_HD float gpu_top_out_force(float travel_m, float vel_mps, float k_n_m, float c_ns_m) noexcept {
    if (travel_m >= 0.0f) {
        return 0.0f;
    }
    const float low_depth_m = -travel_m;
    return gpu_max(0.0f, k_n_m * low_depth_m - c_ns_m * vel_mps);
}

// Initializes static sag preload forces from fork air and shock coil parameters.
BIKE_HD void gpu_init_suspension_preloads(BIKE_THREAD GpuSuspensionParams &p) noexcept {
    const float fork_sag_mm = p.fork_air.wheel_travel_mm * p.fork_air.sag_fraction;
    p.fork_preload_n = gpu_air_axial_force(p.fork_air, fork_sag_mm);

    const float shock_sag_mm = p.shock_coil.stroke_mm * p.shock_coil.sag_fraction;
    p.shock_preload_n = gpu_shock_coil_force(p.shock_coil, shock_sag_mm);
}

// Evaluates all fork and rear shock force components at the given slide joint states.
BIKE_HD GpuSuspensionOutput gpu_compute_suspension(
    const BIKE_THREAD GpuSuspensionParams &p,
    float fork_qpos_m,
    float fork_qvel_mps,
    float shock_qpos_m,
    float shock_qvel_mps
) noexcept {
    const float fork_travel_mm = fork_qpos_m * 1000.0f;
    const float shock_stroke_mm = shock_qpos_m * 1000.0f;

    GpuSuspensionOutput out{};
    out.fork_spring_n = gpu_air_axial_force(p.fork_air, fork_travel_mm);
    out.fork_damper_n = gpu_fork_damper_force(p.fork_damper, fork_qvel_mps, fork_travel_mm);
    const float fork_top_out_n = gpu_top_out_force(
        fork_qpos_m,
        fork_qvel_mps,
        p.fork_top_out_stiffness_n_m,
        p.fork_top_out_damping_ns_m
    );

    out.shock_coil_n = gpu_shock_coil_force(p.shock_coil, shock_stroke_mm);
    out.shock_damper_n = gpu_shock_damper_force(p.shock_damper, shock_qvel_mps);
    out.shock_bumper_n = gpu_shock_bumper_force(p.shock_coil, shock_stroke_mm, shock_qvel_mps);
    out.shock_top_out_n = gpu_top_out_force(
        shock_qpos_m,
        shock_qvel_mps,
        p.shock_top_out_stiffness_n_m,
        p.shock_top_out_damping_ns_m
    );

    // Compression-positive joint coordinates take negated compression-resisting
    // forces and positive top-out restoration forces (matching writers/suspension.cpp).
    out.total_fork_force_n = -(out.fork_spring_n + out.fork_damper_n) + fork_top_out_n;
    out.total_shock_force_n = -(out.shock_coil_n + out.shock_damper_n + out.shock_bumper_n) + out.shock_top_out_n;
    return out;
}

// Computes suspension forces for one world and accumulates them into qfrc.
BIKE_HD GpuSuspensionOutput gpu_write_suspension_qfrc(
    const BIKE_THREAD GpuSuspensionParams &p,
    const BIKE_THREAD GpuModelIndices &idx,
    const BIKE_THREAD float *qpos,
    const BIKE_THREAD float *qvel,
    BIKE_THREAD float *qfrc
) noexcept {
    const float fork_qpos_m = (idx.fork_travel_qpos >= 0) ? qpos[idx.fork_travel_qpos] : 0.0f;
    const float fork_qvel_mps = (idx.fork_travel_dof >= 0) ? qvel[idx.fork_travel_dof] : 0.0f;
    const float shock_qpos_m = (idx.shock_stroke_qpos >= 0) ? qpos[idx.shock_stroke_qpos] : 0.0f;
    const float shock_qvel_mps = (idx.shock_stroke_dof >= 0) ? qvel[idx.shock_stroke_dof] : 0.0f;

    const GpuSuspensionOutput out = gpu_compute_suspension(
        p,
        fork_qpos_m,
        fork_qvel_mps,
        shock_qpos_m,
        shock_qvel_mps
    );

    if (idx.fork_travel_dof >= 0) {
        qfrc[idx.fork_travel_dof] += out.total_fork_force_n;
    }
    if (idx.shock_stroke_dof >= 0) {
        qfrc[idx.shock_stroke_dof] += out.total_shock_force_n;
    }
    return out;
}

} // namespace bike::gpu

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
