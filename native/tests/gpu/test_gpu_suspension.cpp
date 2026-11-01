// native/tests/gpu/test_gpu_suspension.cpp — Unit tests for the GPU suspension kernel
// against double-precision reference formulas from native/src/writers/suspension.cpp.
#include "../../src/gpu/suspension_kernel.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace {

using namespace bike::gpu;

// Double-precision reference from native/src/writers/suspension.cpp::air_axial_force
double ref_air_axial_force(const GpuAirSpringParams &air, double travel_mm) {
    constexpr double kPsiToPa = 6894.757293168;
    const int tokens = std::max(0, std::min(air.max_tokens, air.num_tokens));
    const double travel_m =
        std::max(0.0, std::min(static_cast<double>(air.total_travel_mm), travel_mm)) / 1000.0;
    const double r_m = (static_cast<double>(air.stanchion_inner_diam_mm) / 2.0) / 1000.0;
    const double area = std::numbers::pi * std::pow(r_m, 2.0);
    const double token_vol_m3 = static_cast<double>(air.token_volume_cm3) * 1e-6;
    const double v_pos_0 =
        area * (static_cast<double>(air.pos_chamber_length_mm) / 1000.0) -
        static_cast<double>(tokens) * token_vol_m3;
    const double v_neg_0 = area * (static_cast<double>(air.neg_chamber_length_mm) / 1000.0);
    const double v_disp = area * travel_m;
    const double v_pos = v_pos_0 - v_disp;
    const double v_neg = v_neg_0 + v_disp;
    const double p_abs_0 =
        static_cast<double>(air.gauge_pressure_psi) * kPsiToPa +
        static_cast<double>(air.atm_pressure_pa);
    const double p_pos = p_abs_0 * std::pow(v_pos_0 / v_pos, static_cast<double>(air.gamma));
    const double p_neg = p_abs_0 * std::pow(v_neg_0 / v_neg, static_cast<double>(air.gamma));
    return std::max(0.0, area * (p_pos - p_neg));
}

// Double-precision reference from native/src/writers/suspension.cpp::base_damping
double ref_base_damping(const GpuDamperParams &c, double v) {
    const double frac_lsc =
        std::clamp(c.lsc_clicks, 0, c.max_lsc) / static_cast<double>(c.max_lsc);
    const double frac_hsc =
        std::clamp(c.hsc_clicks, 0, c.max_hsc) / static_cast<double>(c.max_hsc);
    const double frac_reb =
        std::clamp(c.rebound_clicks, 0, c.max_reb) / static_cast<double>(c.max_reb);
    const double c_lsc =
        c.c_lsc_min + (c.c_lsc_max - c.c_lsc_min) * std::pow(frac_lsc, 1.25);
    const double c_hsc =
        c.c_hsc_min + (c.c_hsc_max - c.c_hsc_min) * std::pow(frac_hsc, 1.15);
    const double c_reb =
        c.c_reb_min + (c.c_reb_max - c.c_reb_min) * std::pow(frac_reb, 1.20);
    const double v_knee = static_cast<double>(c.v_knee);

    if (v >= 0.0) {
        if (v <= v_knee) {
            return c_lsc * v * (1.0 + 0.35 * (v / v_knee));
        }
        const double f_knee = c_lsc * v_knee * 1.35;
        const double v_excess = v - v_knee;
        return f_knee + c_hsc * std::pow(v_excess, 0.88) * std::pow(v_knee, 0.12);
    }
    const double v_abs = -v;
    if (v_abs <= v_knee) {
        return -c_reb * v_abs * (1.0 + 0.25 * (v_abs / v_knee));
    }
    const double f_knee = c_reb * v_knee * 1.25;
    const double v_excess = v_abs - v_knee;
    const double c_hs_reb = c_reb * 0.45;
    return -(f_knee + c_hs_reb * std::pow(v_excess, 0.90) * std::pow(v_knee, 0.10));
}

void test_default_params_and_preloads() {
    GpuSuspensionParams p{};
    assert(p.fork_air.stanchion_inner_diam_mm == 24.0f);
    assert(p.fork_air.pos_chamber_length_mm == 280.0f);
    assert(p.fork_air.neg_chamber_length_mm == 55.0f);
    assert(p.fork_air.total_travel_mm == 180.0f);
    assert(p.fork_air.gamma == 1.40f);
    assert(p.fork_air.atm_pressure_pa == 101325.0f);
    assert(p.fork_air.token_volume_cm3 == 8.0f);
    assert(p.fork_air.max_tokens == 4);
    assert(p.fork_air.gauge_pressure_psi == 85.2f);
    assert(p.fork_air.num_tokens == 2);
    assert(p.shock_coil.stroke_mm == 65.0f);
    assert(p.shock_coil.bumper_thickness_mm == 10.0f);
    assert(p.shock_coil.spring_rate_n_mm == 114.6f);

    gpu_init_suspension_preloads(p);
    assert(p.fork_preload_n > 200.0f && p.fork_preload_n < 1500.0f);
    assert(p.shock_preload_n > 1000.0f && p.shock_preload_n < 4000.0f);

    const float expected_shock_preload =
        p.shock_coil.spring_rate_n_mm * (p.shock_coil.stroke_mm * p.shock_coil.sag_fraction);
    assert(std::fabs(p.shock_preload_n - expected_shock_preload) < 1e-3f);
}

void test_air_spring_curve_vs_reference() {
    for (int tokens = 0; tokens <= 4; ++tokens) {
        GpuAirSpringParams air{};
        air.num_tokens = tokens;

        float prev_force = -1.0f;
        for (int travel_int = 0; travel_int <= 170; travel_int += 5) {
            const float travel_mm = static_cast<float>(travel_int);
            const float gpu_f = gpu_air_axial_force(air, travel_mm);
            const double ref_f = ref_air_axial_force(air, static_cast<double>(travel_mm));

            assert(gpu_isfinite(gpu_f));
            assert(gpu_f >= prev_force);
            prev_force = gpu_f;

            if (ref_f > 1e-6) {
                const double rel_err = std::fabs(static_cast<double>(gpu_f) - ref_f) / ref_f;
                assert(rel_err < 1e-5);
            } else {
                assert( std::fabs(static_cast<double>(gpu_f) - ref_f) < 1e-5 );
            }
        }
    }

    // Verify progressive token scaling near bottom-out (170 mm): more tokens -> higher force
    GpuAirSpringParams air0{};
    air0.num_tokens = 0;
    GpuAirSpringParams air4{};
    air4.num_tokens = 4;
    const float f0 = gpu_air_axial_force(air0, 170.0f);
    const float f4 = gpu_air_axial_force(air4, 170.0f);
    assert(f4 > f0 * 1.3f);
}

void test_damping_and_hbo() {
    GpuForkDamperParams fd{};
    const float velocities[] = {-2.0f, -0.8f, -0.5f, -0.2f, 0.0f, 0.2f, 0.5f, 0.8f, 2.0f};

    float prev_f = -1e9f;
    for (float v : velocities) {
        const float f_mid = gpu_fork_damper_force(fd, v, 90.0f); // Before HBO zone (hbo_start = 144 mm)
        const double ref_f = ref_base_damping(fd.core, static_cast<double>(v));
        assert(gpu_isfinite(f_mid));
        assert(f_mid > prev_f);
        prev_f = f_mid;

        if (std::fabs(ref_f) > 1e-6) {
            const double rel_err = std::fabs(static_cast<double>(f_mid) - ref_f) / std::fabs(ref_f);
            assert(rel_err < 1e-5);
        } else {
            assert(std::fabs(f_mid) < 1e-6f);
        }
    }

    // Test HBO engagement in last 36 mm of travel (travel > 144 mm) for positive compression velocity
    const float f_before_hbo = gpu_fork_damper_force(fd, 1.2f, 140.0f);
    const float f_mid_hbo = gpu_fork_damper_force(fd, 1.2f, 162.0f); // 50% into HBO zone
    const float f_full_hbo = gpu_fork_damper_force(fd, 1.2f, 180.0f); // 100% into HBO zone
    assert(f_mid_hbo > f_before_hbo);
    assert(f_full_hbo > f_mid_hbo);
    const float expected_full_hbo_add = fd.hbo_c_max * 1.2f;
    assert(std::fabs((f_full_hbo - f_before_hbo) - expected_full_hbo_add) < 1e-2f);

    // HBO must not engage on rebound (v < 0) even deep in travel
    const float f_reb_early = gpu_fork_damper_force(fd, -1.0f, 90.0f);
    const float f_reb_deep = gpu_fork_damper_force(fd, -1.0f, 175.0f);
    assert(std::fabs(f_reb_early - f_reb_deep) < 1e-5f);
}

void test_coil_bumper_and_top_out() {
    GpuCoilShockParams coil{};
    coil.preload_mm = 2.0f;

    // Linear coil force
    const float f_zero = gpu_shock_coil_force(coil, 0.0f);
    const float f_mid = gpu_shock_coil_force(coil, 30.0f);
    assert(std::fabs(f_zero - coil.spring_rate_n_mm * 2.0f) < 1e-3f);
    assert(std::fabs(f_mid - coil.spring_rate_n_mm * 32.0f) < 1e-3f);

    // Bumper engages after stroke_mm - bumper_thickness_mm = 55 mm
    assert(gpu_shock_bumper_force(coil, 50.0f, 0.5f) == 0.0f);
    assert(gpu_shock_bumper_force(coil, 55.0f, 0.5f) == 0.0f);
    const float b_60 = gpu_shock_bumper_force(coil, 60.0f, 0.5f); // 5 mm penetration
    const float b_65 = gpu_shock_bumper_force(coil, 65.0f, 0.5f); // 10 mm penetration
    assert(b_60 > 0.0f);
    assert(b_65 > b_60 * 3.0f); // Quadratic growth

    // Top-out spring/damper when qpos < 0
    assert(gpu_top_out_force(0.01f, -0.5f, 250000.0f, 2000.0f) == 0.0f);
    assert(gpu_top_out_force(0.0f, -0.5f, 250000.0f, 2000.0f) == 0.0f);
    const float f_top_out = gpu_top_out_force(-0.004f, -0.2f, 250000.0f, 2000.0f);
    // k * 0.004 - c * (-0.2) = 1000 + 400 = 1400 N
    assert(std::fabs(f_top_out - 1400.0f) < 1e-3f);
}

void test_write_suspension_qfrc() {
    GpuSuspensionParams p{};
    gpu_init_suspension_preloads(p);
    GpuModelIndices idx{};
    assert(idx.fork_travel_dof == 6);
    assert(idx.shock_stroke_dof == 13);

    float qpos[kMaxNq] = {0.0f};
    float qvel[kMaxNv] = {0.0f};
    float qfrc[kMaxNv] = {0.0f};

    qpos[idx.fork_travel_qpos] = 0.085f;  // 85 mm fork travel
    qvel[idx.fork_travel_dof] = 0.6f;     // compressing at 0.6 m/s
    qpos[idx.shock_stroke_qpos] = 0.058f; // 58 mm shock stroke (inside bumper)
    qvel[idx.shock_stroke_dof] = 0.3f;    // compressing at 0.3 m/s

    const GpuSuspensionOutput out = gpu_write_suspension_qfrc(p, idx, qpos, qvel, qfrc);

    assert(out.fork_spring_n > 0.0f);
    assert(out.fork_damper_n > 0.0f);
    assert(out.shock_coil_n > 0.0f);
    assert(out.shock_damper_n > 0.0f);
    assert(out.shock_bumper_n > 0.0f);
    assert(out.shock_top_out_n == 0.0f);

    // Generalized forces oppose compression on compression-positive DOFs 6 and 13
    assert(qfrc[6] == out.total_fork_force_n);
    assert(qfrc[13] == out.total_shock_force_n);
    assert(qfrc[6] < 0.0f);
    assert(qfrc[13] < 0.0f);

    // All other DOFs remain untouched
    for (int i = 0; i < kMaxNv; ++i) {
        if (i != 6 && i != 13) {
            assert(qfrc[i] == 0.0f);
        }
    }

    // Verify top-out restoration when qpos < 0
    float qfrc_top_out[kMaxNv] = {0.0f};
    qpos[idx.shock_stroke_qpos] = -0.005f;
    qvel[idx.shock_stroke_dof] = -0.1f;
    const GpuSuspensionOutput out_top = gpu_write_suspension_qfrc(p, idx, qpos, qvel, qfrc_top_out);
    assert(out_top.shock_top_out_n > 0.0f);
    assert(qfrc_top_out[13] > 0.0f);
}

} // namespace

int main() {
    test_default_params_and_preloads();
    test_air_spring_curve_vs_reference();
    test_damping_and_hbo();
    test_coil_bumper_and_top_out();
    test_write_suspension_qfrc();
    std::printf("All GPU suspension kernel tests passed!\n");
    return 0;
}
