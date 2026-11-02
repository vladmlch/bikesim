// native/tests/gpu/test_gpu_suspension.cpp — Unit tests for the GPU suspension kernel
// against double-precision reference formulas from native/src/writers/suspension.cpp.
#include "../../src/gpu/suspension_kernel.hpp"
#include "../../src/platform/argv.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using namespace bike::gpu;

using TestFunction = void (*)();

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition)
        throw std::runtime_error(std::string(message));
}

// Numeric helper used instead of printf-style messages. Templated on the
// arithmetic type so float comparisons keep their original float semantics
// instead of silently promoting to double.
template <class T>
void require_near(T actual, T expected, T tol, std::string_view what) {
    if (!(std::abs(actual - expected) <= tol))
        throw std::runtime_error(std::string(what) +
                                 ": actual=" + std::to_string(actual) +
                                 " expected=" + std::to_string(expected) +
                                 " tol=" + std::to_string(tol));
}

// GpuModelIndices addresses are int for the kernel ABI (a disabled slot is
// -1); container subscripts take size_t, so the explicit conversion lives in
// one place.
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

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
        static_cast<double>(c.c_lsc_min) +
        (static_cast<double>(c.c_lsc_max) - static_cast<double>(c.c_lsc_min)) *
            std::pow(frac_lsc, 1.25);
    const double c_hsc =
        static_cast<double>(c.c_hsc_min) +
        (static_cast<double>(c.c_hsc_max) - static_cast<double>(c.c_hsc_min)) *
            std::pow(frac_hsc, 1.15);
    const double c_reb =
        static_cast<double>(c.c_reb_min) +
        (static_cast<double>(c.c_reb_max) - static_cast<double>(c.c_reb_min)) *
            std::pow(frac_reb, 1.20);
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
    require(p.fork_air.stanchion_inner_diam_mm == 24.0f,
            "default stanchion inner diameter is 24 mm");
    require(p.fork_air.pos_chamber_length_mm == 280.0f,
            "default positive chamber length is 280 mm");
    require(p.fork_air.neg_chamber_length_mm == 55.0f,
            "default negative chamber length is 55 mm");
    require(p.fork_air.total_travel_mm == 180.0f,
            "default total travel is 180 mm");
    require(p.fork_air.gamma == 1.40f, "default polytropic exponent is 1.40");
    require(p.fork_air.atm_pressure_pa == 101325.0f,
            "default atmospheric pressure is 101325 Pa");
    require(p.fork_air.token_volume_cm3 == 8.0f,
            "default token volume is 8 cm3");
    require(p.fork_air.max_tokens == 4, "default max tokens is 4");
    require(p.fork_air.gauge_pressure_psi == 85.2f,
            "default gauge pressure is 85.2 psi");
    require(p.fork_air.num_tokens == 2, "default token count is 2");
    require(p.shock_coil.stroke_mm == 65.0f,
            "default shock stroke is 65 mm");
    require(p.shock_coil.bumper_thickness_mm == 10.0f,
            "default bumper thickness is 10 mm");
    require(p.shock_coil.spring_rate_n_mm == 114.6f,
            "default coil spring rate is 114.6 N/mm");

    gpu_init_suspension_preloads(p);
    require(p.fork_preload_n > 200.0f && p.fork_preload_n < 1500.0f,
            "fork preload lands in the plausible 200..1500 N window");
    require(p.shock_preload_n > 1000.0f && p.shock_preload_n < 4000.0f,
            "shock preload lands in the plausible 1000..4000 N window");

    const float expected_shock_preload =
        p.shock_coil.spring_rate_n_mm * (p.shock_coil.stroke_mm * p.shock_coil.sag_fraction);
    require_near(p.shock_preload_n, expected_shock_preload, 1e-3f,
                 "shock preload equals rate * stroke * sag fraction");
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

            require(gpu_isfinite(gpu_f), "air spring force is finite");
            require(gpu_f >= prev_force, "air spring force is non-decreasing in travel");
            prev_force = gpu_f;

            if (ref_f > 1e-6) {
                const double rel_err = std::fabs(static_cast<double>(gpu_f) - ref_f) / ref_f;
                require_near(rel_err, 0.0, 1e-5,
                             "air spring force tracks the double reference relatively");
            } else {
                require_near(static_cast<double>(gpu_f), ref_f, 1e-5,
                             "air spring force tracks the double reference absolutely");
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
    require(f4 > f0 * 1.3f, "four tokens beat zero tokens by >30% near bottom-out");
}

void test_damping_and_hbo() {
    const GpuForkDamperParams fd{};
    constexpr std::array<float, 9> velocities{
        {-2.0f, -0.8f, -0.5f, -0.2f, 0.0f, 0.2f, 0.5f, 0.8f, 2.0f}};

    float prev_f = -1e9f;
    for (const float v : velocities) {
        const float f_mid = gpu_fork_damper_force(fd, v, 90.0f); // Before HBO zone (hbo_start = 144 mm)
        const double ref_f = ref_base_damping(fd.core, static_cast<double>(v));
        require(gpu_isfinite(f_mid), "fork damper force is finite");
        require(f_mid > prev_f, "fork damper force is strictly increasing in velocity");
        prev_f = f_mid;

        if (std::fabs(ref_f) > 1e-6) {
            const double rel_err = std::fabs(static_cast<double>(f_mid) - ref_f) / std::fabs(ref_f);
            require_near(rel_err, 0.0, 1e-5,
                         "fork damper force tracks the double reference relatively");
        } else {
            require_near(f_mid, 0.0f, 1e-6f,
                         "fork damper force at zero velocity stays near zero");
        }
    }

    // Test HBO engagement in last 36 mm of travel (travel > 144 mm) for positive compression velocity
    const float f_before_hbo = gpu_fork_damper_force(fd, 1.2f, 140.0f);
    const float f_mid_hbo = gpu_fork_damper_force(fd, 1.2f, 162.0f); // 50% into HBO zone
    const float f_full_hbo = gpu_fork_damper_force(fd, 1.2f, 180.0f); // 100% into HBO zone
    require(f_mid_hbo > f_before_hbo, "HBO raises the force inside the zone");
    require(f_full_hbo > f_mid_hbo, "HBO keeps growing through the zone");
    const float expected_full_hbo_add = fd.hbo_c_max * 1.2f;
    require_near(f_full_hbo - f_before_hbo, expected_full_hbo_add, 1e-2f,
                 "full HBO adds hbo_c_max * velocity");

    // HBO must not engage on rebound (v < 0) even deep in travel
    const float f_reb_early = gpu_fork_damper_force(fd, -1.0f, 90.0f);
    const float f_reb_deep = gpu_fork_damper_force(fd, -1.0f, 175.0f);
    require_near(f_reb_early, f_reb_deep, 1e-5f,
                 "rebound force ignores travel depth (no HBO)");
}

void test_coil_bumper_and_top_out() {
    GpuCoilShockParams coil{};
    coil.preload_mm = 2.0f;

    // Linear coil force
    const float f_zero = gpu_shock_coil_force(coil, 0.0f);
    const float f_mid = gpu_shock_coil_force(coil, 30.0f);
    require_near(f_zero, coil.spring_rate_n_mm * 2.0f, 1e-3f,
                 "coil force at zero stroke equals rate * preload");
    require_near(f_mid, coil.spring_rate_n_mm * 32.0f, 1e-3f,
                 "coil force adds rate * stroke linearly");

    // Bumper engages after stroke_mm - bumper_thickness_mm = 55 mm
    require(gpu_shock_bumper_force(coil, 50.0f, 0.5f) == 0.0f,
            "bumper is silent before engagement");
    require(gpu_shock_bumper_force(coil, 55.0f, 0.5f) == 0.0f,
            "bumper is silent at the engagement boundary");
    const float b_60 = gpu_shock_bumper_force(coil, 60.0f, 0.5f); // 5 mm penetration
    const float b_65 = gpu_shock_bumper_force(coil, 65.0f, 0.5f); // 10 mm penetration
    require(b_60 > 0.0f, "bumper pushes back past engagement");
    require(b_65 > b_60 * 3.0f, "bumper grows quadratically"); // Quadratic growth

    // Top-out spring/damper when qpos < 0
    require(gpu_top_out_force(0.01f, -0.5f, 250000.0f, 2000.0f) == 0.0f,
            "top-out is silent at positive travel");
    require(gpu_top_out_force(0.0f, -0.5f, 250000.0f, 2000.0f) == 0.0f,
            "top-out is silent at zero travel");
    const float f_top_out = gpu_top_out_force(-0.004f, -0.2f, 250000.0f, 2000.0f);
    // k * 0.004 - c * (-0.2) = 1000 + 400 = 1400 N
    require_near(f_top_out, 1400.0f, 1e-3f,
                 "top-out returns k*depth - c*vel = 1400 N");
}

void test_write_suspension_qfrc() {
    GpuSuspensionParams p{};
    gpu_init_suspension_preloads(p);
    const GpuModelIndices idx{};
    require(idx.fork_travel_dof == 6, "fork travel DOF address is 6");
    require(idx.shock_stroke_dof == 13, "shock stroke DOF address is 13");

    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    std::array<float, kMaxNv> qfrc{};

    qpos[to_index(idx.fork_travel_qpos)] = 0.085f;  // 85 mm fork travel
    qvel[to_index(idx.fork_travel_dof)] = 0.6f;     // compressing at 0.6 m/s
    qpos[to_index(idx.shock_stroke_qpos)] = 0.058f; // 58 mm shock stroke (inside bumper)
    qvel[to_index(idx.shock_stroke_dof)] = 0.3f;    // compressing at 0.3 m/s

    const GpuSuspensionOutput out =
        gpu_write_suspension_qfrc(p, idx, qpos.data(), qvel.data(), qfrc.data());

    require(out.fork_spring_n > 0.0f, "fork spring force is positive");
    require(out.fork_damper_n > 0.0f, "fork damper force is positive");
    require(out.shock_coil_n > 0.0f, "shock coil force is positive");
    require(out.shock_damper_n > 0.0f, "shock damper force is positive");
    require(out.shock_bumper_n > 0.0f, "shock bumper force is positive");
    require(out.shock_top_out_n == 0.0f, "shock top-out stays silent");

    // Generalized forces oppose compression on compression-positive DOFs 6 and 13
    require(qfrc[6] == out.total_fork_force_n, "qfrc[6] holds the fork total");
    require(qfrc[13] == out.total_shock_force_n, "qfrc[13] holds the shock total");
    require(qfrc[6] < 0.0f, "fork generalized force opposes compression");
    require(qfrc[13] < 0.0f, "shock generalized force opposes compression");

    // All other DOFs remain untouched
    for (std::size_t i = 0; i < qfrc.size(); ++i) {
        if (i != 6 && i != 13) {
            require(qfrc[i] == 0.0f, "untouched DOFs stay zero");
        }
    }

    // Verify top-out restoration when qpos < 0
    std::array<float, kMaxNv> qfrc_top_out{};
    qpos[to_index(idx.shock_stroke_qpos)] = -0.005f;
    qvel[to_index(idx.shock_stroke_dof)] = -0.1f;
    const GpuSuspensionOutput out_top =
        gpu_write_suspension_qfrc(p, idx, qpos.data(), qvel.data(), qfrc_top_out.data());
    require(out_top.shock_top_out_n > 0.0f, "top-out engages below qpos 0");
    require(qfrc_top_out[13] > 0.0f, "top-out restores the shock DOF positively");
}

void test_suspension_qfrc_accumulation_and_disabled_dofs() {
    GpuSuspensionParams p{};
    gpu_init_suspension_preloads(p);
    const GpuModelIndices idx{};

    std::array<float, kMaxNq> qpos{};
    std::array<float, kMaxNv> qvel{};
    qpos[to_index(idx.fork_travel_qpos)] = 0.050f;
    qvel[to_index(idx.fork_travel_dof)] = 0.25f;
    qpos[to_index(idx.shock_stroke_qpos)] = 0.030f;
    qvel[to_index(idx.shock_stroke_dof)] = 0.15f;

    // 1. Verify accumulation (+=) into non-zero pre-existing qfrc values
    std::array<float, kMaxNv> qfrc{};
    std::array<float, kMaxNv> qfrc_expected{};
    for (std::size_t i = 0; i < qfrc.size(); ++i) {
        qfrc[i] = 100.0f + static_cast<float>(i) * 7.5f;
        qfrc_expected[i] = qfrc[i];
    }

    const GpuSuspensionOutput out1 =
        gpu_write_suspension_qfrc(p, idx, qpos.data(), qvel.data(), qfrc.data());
    require(std::fabs(out1.total_fork_force_n) > 1.0f,
            "fork total force is non-trivial");
    require(std::fabs(out1.total_shock_force_n) > 1.0f,
            "shock total force is non-trivial");

    qfrc_expected[to_index(idx.fork_travel_dof)] += out1.total_fork_force_n;
    qfrc_expected[to_index(idx.shock_stroke_dof)] += out1.total_shock_force_n;

    for (std::size_t i = 0; i < qfrc.size(); ++i) {
        require_near(qfrc[i], qfrc_expected[i], 1e-5f,
                     "first accumulation adds into every expected slot");
    }

    // Secondary accumulation call to confirm successive += accumulation
    const GpuSuspensionOutput out2 =
        gpu_write_suspension_qfrc(p, idx, qpos.data(), qvel.data(), qfrc.data());
    qfrc_expected[to_index(idx.fork_travel_dof)] += out2.total_fork_force_n;
    qfrc_expected[to_index(idx.shock_stroke_dof)] += out2.total_shock_force_n;

    for (std::size_t i = 0; i < qfrc.size(); ++i) {
        require_near(qfrc[i], qfrc_expected[i], 1e-5f,
                     "second accumulation adds again into every expected slot");
    }

    // 2. Verify disabled fork DOF (fork_travel_dof = -1) safely skips fork write
    {
        GpuModelIndices idx_fork_disabled = idx;
        idx_fork_disabled.fork_travel_dof = -1;

        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            qfrc[i] = 50.0f + static_cast<float>(i) * 3.0f;
            qfrc_expected[i] = qfrc[i];
        }

        const GpuSuspensionOutput out_fd = gpu_write_suspension_qfrc(
            p, idx_fork_disabled, qpos.data(), qvel.data(), qfrc.data()
        );

        // Fork DOF (6) must NOT be written
        require(qfrc[to_index(idx.fork_travel_dof)] ==
                    qfrc_expected[to_index(idx.fork_travel_dof)],
                "disabled fork DOF is not written");
        // Shock DOF (13) must be accumulated
        qfrc_expected[to_index(idx.shock_stroke_dof)] += out_fd.total_shock_force_n;
        require_near(qfrc[to_index(idx.shock_stroke_dof)],
                     qfrc_expected[to_index(idx.shock_stroke_dof)], 1e-5f,
                     "shock DOF still accumulates with fork disabled");

        // All other DOFs in [0, kMaxNv) remain untouched
        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            require_near(qfrc[i], qfrc_expected[i], 1e-5f,
                         "only the shock DOF moves with fork disabled");
        }
    }

    // 3. Verify disabled shock DOF (shock_stroke_dof = -1) safely skips shock write
    {
        GpuModelIndices idx_shock_disabled = idx;
        idx_shock_disabled.shock_stroke_dof = -1;

        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            qfrc[i] = 75.0f - static_cast<float>(i) * 2.0f;
            qfrc_expected[i] = qfrc[i];
        }

        const GpuSuspensionOutput out_sd = gpu_write_suspension_qfrc(
            p, idx_shock_disabled, qpos.data(), qvel.data(), qfrc.data()
        );

        // Shock DOF (13) must NOT be written
        require(qfrc[to_index(idx.shock_stroke_dof)] ==
                    qfrc_expected[to_index(idx.shock_stroke_dof)],
                "disabled shock DOF is not written");
        // Fork DOF (6) must be accumulated
        qfrc_expected[to_index(idx.fork_travel_dof)] += out_sd.total_fork_force_n;
        require_near(qfrc[to_index(idx.fork_travel_dof)],
                     qfrc_expected[to_index(idx.fork_travel_dof)], 1e-5f,
                     "fork DOF still accumulates with shock disabled");

        // All other DOFs in [0, kMaxNv) remain untouched
        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            require_near(qfrc[i], qfrc_expected[i], 1e-5f,
                         "only the fork DOF moves with shock disabled");
        }
    }

    // 4. Verify both DOFs disabled skips all writes without clobbering any DOF
    {
        GpuModelIndices idx_both_disabled = idx;
        idx_both_disabled.fork_travel_dof = -1;
        idx_both_disabled.shock_stroke_dof = -1;

        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            qfrc[i] = 200.0f + static_cast<float>(i);
            qfrc_expected[i] = qfrc[i];
        }

        gpu_write_suspension_qfrc(
            p, idx_both_disabled, qpos.data(), qvel.data(), qfrc.data());

        for (std::size_t i = 0; i < qfrc.size(); ++i) {
            require(qfrc[i] == qfrc_expected[i],
                    "with both DOFs disabled no slot is clobbered");
        }
    }

    // 5. Verify negative qpos indices are handled safely without out-of-bounds reads
    {
        GpuModelIndices idx_neg_qpos = idx;
        idx_neg_qpos.fork_travel_qpos = -1;
        idx_neg_qpos.fork_travel_dof = -1;
        idx_neg_qpos.shock_stroke_qpos = -1;
        idx_neg_qpos.shock_stroke_dof = -1;

        qfrc.fill(123.0f);

        gpu_write_suspension_qfrc(
            p, idx_neg_qpos, qpos.data(), qvel.data(), qfrc.data());

        for (const float force : qfrc) {
            require(force == 123.0f,
                    "negative indices leave every qfrc slot untouched");
        }
    }
}

constexpr std::array<TestCase, 6> cases{{
    {.name = "default_params_and_preloads",
     .run = test_default_params_and_preloads},
    {.name = "air_spring_curve_vs_reference",
     .run = test_air_spring_curve_vs_reference},
    {.name = "damping_and_hbo", .run = test_damping_and_hbo},
    {.name = "coil_bumper_and_top_out", .run = test_coil_bumper_and_top_out},
    {.name = "write_suspension_qfrc", .run = test_write_suspension_qfrc},
    {.name = "suspension_qfrc_accumulation_and_disabled_dofs",
     .run = test_suspension_qfrc_accumulation_and_disabled_dofs},
}};

int run_case(const TestCase &test_case) {
    try {
        test_case.run();
        std::cout << "PASS " << test_case.name << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
        return 1;
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        const std::span<char *> arguments =
            bike::platform::main_arguments(argc, argv);
        if (arguments.size() == 2 &&
            std::string_view(arguments[1]) == "--list") {
            for (const TestCase &test_case: cases)
                std::cout << test_case.name << '\n';
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_suspension [case-name|--list]\n";
            return 2;
        }

        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
        bool found = requested == "all";
        int status = 0;
        for (const TestCase &test_case: cases) {
            if (requested == "all" || requested == test_case.name) {
                found = true;
                if (run_case(test_case) != 0)
                    status = 1;
            }
        }
        if (!found) {
            std::cerr << "unknown gpu suspension case: " << requested << '\n';
            return 2;
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
