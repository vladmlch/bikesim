#include "physics_core/common.hpp"
#include "physics_core/cpu_exact_toolkit.hpp"
#include "physics_core/exact_math.hpp"
#include "physics_core/gpu_math.hpp"
#include "physics_core/gpu_toolkit.hpp"
#include "physics_core/tyre_types.hpp"
#include "tyre/profile.hpp"

#if defined(__APPLE__)
#include "test_physics_core_metal_helper.hpp"
#endif

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using bike::physics_core::CpuExactToolkit;
using bike::physics_core::GpuToolkit;

void require(bool cond, std::string_view msg) {
    if (!cond) {
        throw std::runtime_error(std::string(msg));
    }
}

std::uint32_t float_ulp_diff(float a, float b) {
    if (std::isnan(a) || std::isnan(b)) {
        return std::numeric_limits<std::uint32_t>::max();
    }
    if (a == b) {
        return 0u;
    }
    const auto ua = std::bit_cast<std::uint32_t>(a);
    const auto ub = std::bit_cast<std::uint32_t>(b);
    const auto map_ordered = [](std::uint32_t u) -> std::uint32_t {
        return (u & 0x80000000u) != 0u ? (0x80000000u - u) : (u + 0x80000000u);
    };
    const std::uint32_t oa = map_ordered(ua);
    const std::uint32_t ob = map_ordered(ub);
    return oa > ob ? (oa - ob) : (ob - oa);
}

void require_bits_f64(double actual, double expected, std::string_view what) {
    if (std::bit_cast<std::uint64_t>(actual) != std::bit_cast<std::uint64_t>(expected)) {
        throw std::runtime_error(std::string(what) + ": 64-bit mismatch");
    }
}

void require_bits_f32(float actual, float expected, std::string_view what) {
    if (std::bit_cast<std::uint32_t>(actual) != std::bit_cast<std::uint32_t>(expected)) {
        throw std::runtime_error(std::string(what) + ": 32-bit mismatch");
    }
}

void test_common_and_types() {
    using namespace bike::physics_core;
    require(Lit<double>(1e-12) == 1e-12, "Lit<double>");
    require(Lit<float>(1e-12f) == 1e-12f, "Lit<float>");
    require(BIKE_REAL(double, 1e-12) == 1e-12, "BIKE_REAL<double>");
    require(BIKE_REAL(float, 1e-12) == 1e-12f, "BIKE_REAL<float>");
    Result r{};
    require(r.ok(), "default Result ok");
    r.status = Status::invalid_argument;
    r.err = ErrCode::degenerate_contact;
    require(!r.ok(), "error Result !ok");
    require(kJntSlide == 2 && kJntHinge == 3, "MuJoCo joint kind constants");
    const TyreModel<float> tm{};
    require(tm.backend == TyreBackendKind::compliant_2d, "default TyreModel backend");
    require(tm.distinct_normal_cosine == 0.0f, "default TyreModel distinct_normal_cosine");
    require(tm.surface_map.has_surface_map == 0, "default SurfaceMapView has_surface_map");
    require(tm.wheels[0].chain.njnt == 0, "default GpuWheelChain njnt");
    const CompliantWheelStatePod<float> cw{};
    require(cw.segment == -1 && cw.has_segment == 0, "default CompliantWheelStatePod segment");
    const StationStatePod<float> st{};
    require(st.active == 0, "default StationStatePod active");
}

void test_expf_portable() {
    // Special values
    require_bits_f32(GpuToolkit::exp(0.0f), 1.0f, "exp(+0)");
    require_bits_f32(GpuToolkit::exp(-0.0f), 1.0f, "exp(-0)");
    require(std::isnan(GpuToolkit::exp(std::numeric_limits<float>::quiet_NaN())), "exp(NaN)");
    const float inf_pos = std::numeric_limits<float>::infinity();
    require(std::isinf(GpuToolkit::exp(inf_pos)) && GpuToolkit::exp(inf_pos) > 0.0f, "exp(+Inf)");
    require_bits_f32(GpuToolkit::exp(-inf_pos), 0.0f, "exp(-Inf)");
    require(std::isinf(GpuToolkit::exp(89.0f)) && GpuToolkit::exp(89.0f) > 0.0f, "exp(89)");
    require(std::isinf(GpuToolkit::exp(90.0f)) && GpuToolkit::exp(90.0f) > 0.0f, "exp(90)");
    require_bits_f32(GpuToolkit::exp(-104.5f), 0.0f, "exp(-104.5)");
    require_bits_f32(GpuToolkit::exp(-105.0f), 0.0f, "exp(-105)");

    // Fine grid [-20.0, +20.0] step 0.005: <= 2 ULP vs float(CpuExactToolkit::exp(double(x)))
    for (int i = -4000; i <= 4000; ++i) {
        const float x = static_cast<float>(i) * 0.005f;
        const float g = GpuToolkit::exp(x);
        const float ref = static_cast<float>(CpuExactToolkit::exp(static_cast<double>(x)));
        const std::uint32_t ulp = float_ulp_diff(g, ref);
        if (ulp > 2u) {
            throw std::runtime_error("expf_portable ULP > 2 at x=" + std::to_string(x) +
                                     " ulp=" + std::to_string(ulp));
        }
    }

    // Wide normal range [-87.0f, +88.0f]
    for (int i = -348; i <= 352; ++i) {
        const float x = static_cast<float>(i) * 0.25f;
        const float g = GpuToolkit::exp(x);
        const float ref = static_cast<float>(CpuExactToolkit::exp(static_cast<double>(x)));
        const std::uint32_t ulp = float_ulp_diff(g, ref);
        if (ulp > 2u) {
            throw std::runtime_error("expf_portable wide ULP > 2 at x=" + std::to_string(x) +
                                     " ulp=" + std::to_string(ulp));
        }
    }
}

void test_tanhf_portable() {
    require_bits_f32(GpuToolkit::tanh(0.0f), 0.0f, "tanh(+0)");
    require_bits_f32(GpuToolkit::tanh(-0.0f), -0.0f, "tanh(-0)");
    require(std::isnan(GpuToolkit::tanh(std::numeric_limits<float>::quiet_NaN())), "tanh(NaN)");
    const float inf_pos = std::numeric_limits<float>::infinity();
    require_bits_f32(GpuToolkit::tanh(inf_pos), 1.0f, "tanh(+Inf)");
    require_bits_f32(GpuToolkit::tanh(-inf_pos), -1.0f, "tanh(-Inf)");
    require_bits_f32(GpuToolkit::tanh(10.0f), 1.0f, "tanh(10)");
    require_bits_f32(GpuToolkit::tanh(-10.0f), -1.0f, "tanh(-10)");

    // Taylor region |x| < 1e-4
    const std::array<float, 8> tiny_vals = {
        -9.9e-5f, -5.0e-5f, -1.0e-6f, -1.0e-8f,
        1.0e-8f, 1.0e-6f, 5.0e-5f, 9.9e-5f
    };
    for (const float x : tiny_vals) {
        const float g = GpuToolkit::tanh(x);
        const float ref = static_cast<float>(CpuExactToolkit::tanh(static_cast<double>(x)));
        require(float_ulp_diff(g, ref) <= 2u, "tanh tiny ULP <= 2");
    }

    // Fine grid [-20.0, +20.0] step 0.005
    for (int i = -4000; i <= 4000; ++i) {
        const float x = static_cast<float>(i) * 0.005f;
        const float g = GpuToolkit::tanh(x);
        const double c = CpuExactToolkit::tanh(static_cast<double>(x));
        const float ref = static_cast<float>(c);
        const std::uint32_t ulp = float_ulp_diff(g, ref);
        if (ulp > 2u) {
            throw std::runtime_error("tanhf_portable ULP > 2 at x=" + std::to_string(x) +
                                     " ulp=" + std::to_string(ulp));
        }
        if (!(std::abs(static_cast<double>(g) - c) <= 3e-7)) {
            throw std::runtime_error("tanhf_portable abs err > 3e-7 at x=" + std::to_string(x));
        }
    }

    // Dense sweep [-1.5, +1.5] step 5e-5 (covers [ln(2)/2, 1.5*ln(2)] z=1 range-reduction window)
    for (int i = -30000; i <= 30000; ++i) {
        const float x = static_cast<float>(i) * 5.0e-5f;
        const float g = GpuToolkit::tanh(x);
        const double c = CpuExactToolkit::tanh(static_cast<double>(x));
        const float ref = static_cast<float>(c);
        const std::uint32_t ulp = float_ulp_diff(g, ref);
        if (ulp > 2u) {
            throw std::runtime_error("tanhf_portable dense ULP > 2 at x=" + std::to_string(x) +
                                     " ulp=" + std::to_string(ulp));
        }
        if (!(std::abs(static_cast<double>(g) - c) <= 3e-7)) {
            throw std::runtime_error("tanhf_portable dense abs err > 3e-7 at x=" + std::to_string(x));
        }
    }

    // Explicit regression points from t3-rev z=1 cancellation boundary
    for (const float x : {0.22045f, -0.17328729f}) {
        const float g = GpuToolkit::tanh(x);
        const float ref = static_cast<float>(CpuExactToolkit::tanh(static_cast<double>(x)));
        require(float_ulp_diff(g, ref) <= 2u, "tanhf_portable z=1 regression point ULP <= 2");
    }
}

void test_hypot2f_portable() {
    struct Pair {
        float a;
        float b;
    };
    const std::array<Pair, 14> finite_pairs = {{
        {.a = 0.0f, .b = 0.0f},
        {.a = -0.0f, .b = 0.0f},
        {.a = 3.0f, .b = 4.0f},
        {.a = -3.0f, .b = -4.0f},
        {.a = 1.0e-30f, .b = 1.0e-30f},
        {.a = 3.0e-30f, .b = 4.0e-30f},
        {.a = 1.0e30f, .b = 1.0e30f},
        {.a = 3.0e30f, .b = 4.0e30f},
        {.a = 1.0e20f, .b = 1.0e-20f},
        {.a = 0.0f, .b = -5.5f},
        {.a = 1.2345e-3f, .b = 9.8765e-3f},
        {.a = 123.456f, .b = 0.00789f},
        {.a = 3.4e38f, .b = 0.0f},
        {.a = 1.0e-38f, .b = 1.0e-38f}
    }};
    for (const Pair p : finite_pairs) {
        const float g = GpuToolkit::hypot2(p.a, p.b);
        const float ref = static_cast<float>(
            CpuExactToolkit::hypot2(static_cast<double>(p.a), static_cast<double>(p.b)));
        require(std::isfinite(g), "hypot2 finite pair must remain finite");
        require(float_ulp_diff(g, ref) <= 2u, "hypot2 finite pair ULP <= 2");
    }

    const float inf_pos = std::numeric_limits<float>::infinity();
    const float qnan = std::numeric_limits<float>::quiet_NaN();
    require(std::isinf(GpuToolkit::hypot2(inf_pos, 1.0f)), "hypot2(Inf, 1)");
    require(std::isinf(GpuToolkit::hypot2(-inf_pos, qnan)), "hypot2(-Inf, NaN)");
    require(std::isinf(GpuToolkit::hypot2(qnan, inf_pos)), "hypot2(NaN, Inf)");
    require(std::isnan(GpuToolkit::hypot2(qnan, 1.0f)), "hypot2(NaN, 1)");
    require(std::isnan(GpuToolkit::hypot2(1.0f, qnan)), "hypot2(1, NaN)");
}

void test_interp_segment_unfused() {
    // Normal segment
    const float g_norm = GpuToolkit::interp_segment(2.0f, 1.5f, 1.0f, 10.0f, 2.0f, 12.0f);
    require_bits_f32(g_norm, 11.0f, "interp_segment normal");

    // Constant segment y0 == y1 (must return y0 even when x = +Inf, s = 0)
    const float inf_pos = std::numeric_limits<float>::infinity();
    const float g_const = GpuToolkit::interp_segment(0.0f, inf_pos, 0.0f, 7.25f, 1.0f, 7.25f);
    require_bits_f32(g_const, 7.25f, "interp_segment constant y0 == y1");

    // Steep-slope right-endpoint NaN retry:
    // y0 = -Inf, s * (x - x0) = +Inf -> y0 + s * (x - x0) = NaN -> retry y1 + s * (x - x1) = 42.0f
    const float g_retry = GpuToolkit::interp_segment(
        1.0e38f, 10.0f, 0.0f, -inf_pos, 10.0f, 42.0f);
    require_bits_f32(g_retry, 42.0f, "interp_segment right-endpoint retry");
}

void test_cpu_exact_toolkit_parity() {
    const std::array<double, 4> a = {1.125, -2.5, 3.75, 0.0625};
    const std::array<double, 4> b = {-0.5, 4.25, -1.25, 8.0};

    require_bits_f64(
        CpuExactToolkit::dot2(a.data(), b.data()),
        blas::ddot(2, a.data(), 1, b.data(), 1),
        "CpuExactToolkit::dot2");
    require_bits_f64(
        CpuExactToolkit::dot3(a.data(), b.data()),
        blas::ddot(3, a.data(), 1, b.data(), 1),
        "CpuExactToolkit::dot3");
    require_bits_f64(
        CpuExactToolkit::dot(4, a.data(), b.data()),
        blas::ddot(4, a.data(), 1, b.data(), 1),
        "CpuExactToolkit::dot");

    require_bits_f64(
        CpuExactToolkit::norm2_ddot(a.data()),
        std::sqrt(blas::ddot(2, a.data(), 1, a.data(), 1)),
        "CpuExactToolkit::norm2_ddot");
    require_bits_f64(
        CpuExactToolkit::norm3_ddot(a.data()),
        std::sqrt(blas::ddot(3, a.data(), 1, a.data(), 1)),
        "CpuExactToolkit::norm3_ddot");

    require_bits_f64(
        CpuExactToolkit::einsum2(-0.0, 0.0, 0.0, -0.0),
        0.0 + (-0.0) * 0.0 + 0.0 * (-0.0),
        "CpuExactToolkit::einsum2 signed zero");

    // pow2 vs pyfloat::pow(x, 2.0) — verify 0-bit match even where pow(x,2.0) != x*x
    const std::array<double, 6> pow_inputs = {0.0, -0.0, 0.123456789, 1.875, 12.345, -3.7};
    for (const double x : pow_inputs) {
        require_bits_f64(
            CpuExactToolkit::pow2(x),
            pyfloat::pow(x, 2.0),
            "CpuExactToolkit::pow2");
        require_bits_f64(
            CpuExactToolkit::pow_py(x, 2.0),
            pyfloat::pow(x, 2.0),
            "CpuExactToolkit::pow_py");
    }

    // hypot2 vs biketyre::py_hypot
    require_bits_f64(
        CpuExactToolkit::hypot2(1.23456789012345, -9.87654321098765),
        biketyre::py_hypot(1.23456789012345, -9.87654321098765),
        "CpuExactToolkit::hypot2");

    // interp_segment vs numeric::np_interp_segment
    require_bits_f64(
        CpuExactToolkit::interp_segment(0.3141592653589793, 2.75, 2.0, 1.5, 3.0, 1.8141592653589793),
        numeric::np_interp_segment(0.3141592653589793, 2.75, 2.0, 1.5, 3.0, 1.8141592653589793),
        "CpuExactToolkit::interp_segment");

    // clamp -0.0 preservation
    require_bits_f64(CpuExactToolkit::clamp(-0.0, 0.0, 1.0), -0.0, "CpuExactToolkit::clamp(-0.0)");
    require_bits_f32(GpuToolkit::clamp(-0.0f, 0.0f, 1.0f), -0.0f, "GpuToolkit::clamp(-0.0f)");

    // gemv_n3, gemv_t3, gemv_n2, ring_lift
    const std::array<double, 12> mat_3x4 = {
        1.0, -2.0, 3.5, 0.25,
        -0.5, 4.0, 1.25, -3.0,
        2.0, 0.75, -1.5, 5.0
    };
    const std::array<double, 4> x4 = {0.5, -1.5, 2.25, -0.75};
    const std::array<double, 3> x3 = {1.25, -0.5, 2.0};
    std::array<double, 3> y_tk3{};
    std::array<double, 3> y_blas3{};
    CpuExactToolkit::gemv_n3(4, mat_3x4.data(), x4.data(), y_tk3.data());
    blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, 4, 1.0, mat_3x4.data(), 4, x4.data(), 1, 0.0, y_blas3.data(), 1);
    for (std::size_t i = 0; i < 3; ++i) {
        require_bits_f64(y_tk3[i], y_blas3[i], "CpuExactToolkit::gemv_n3");
    }

    std::array<double, 4> y_tk4{};
    std::array<double, 4> y_blas4{};
    CpuExactToolkit::gemv_t3(4, mat_3x4.data(), x3.data(), y_tk4.data());
    blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, 4, 1.0, mat_3x4.data(), 4, x3.data(), 1, 0.0, y_blas4.data(), 1);
    for (std::size_t i = 0; i < 4; ++i) {
        require_bits_f64(y_tk4[i], y_blas4[i], "CpuExactToolkit::gemv_t3");
    }

    const std::array<double, 6> mat_3x2 = {1.0, 2.0, -3.0, 4.0, 0.5, -1.5};
    const std::array<double, 2> x2 = {2.5, -0.75};
    std::array<double, 3> yn2_tk{};
    std::array<double, 3> yn2_blas{};
    CpuExactToolkit::gemv_n2(3, mat_3x2.data(), x2.data(), yn2_tk.data());
    blas::dgemv(blas::Order::row_major, blas::Transpose::no, 3, 2, 1.0, mat_3x2.data(), 2, x2.data(), 1, 0.0, yn2_blas.data(), 1);
    for (std::size_t i = 0; i < 3; ++i) {
        require_bits_f64(yn2_tk[i], yn2_blas[i], "CpuExactToolkit::gemv_n2");
    }

    const std::array<double, 6> L = {0.35, 0.0, 0.1, -0.25, 0.0, 0.3};
    const std::array<double, 9> R = {
        0.9, 0.0, 0.43588989,
        0.0, 1.0, 0.0,
        -0.43588989, 0.0, 0.9
    };
    std::array<double, 6> out_tk{};
    std::array<double, 6> out_blas{};
    CpuExactToolkit::ring_lift(2, L.data(), R.data(), out_tk.data());
    blas::dgemm(blas::Order::row_major, blas::Transpose::no, blas::Transpose::yes,
                2, 3, 3, 1.0, L.data(), 3, R.data(), 3, 0.0, out_blas.data(), 3);
    for (std::size_t i = 0; i < 6; ++i) {
        require_bits_f64(out_tk[i], out_blas[i], "CpuExactToolkit::ring_lift");
    }
}

void test_gpu_toolkit_linalg_vs_cpu() {
    const std::array<float, 12> mat_3x4_f = {
        1.0f, -2.0f, 3.5f, 0.25f,
        -0.5f, 4.0f, 1.25f, -3.0f,
        2.0f, 0.75f, -1.5f, 5.0f
    };
    const std::array<float, 4> x4_f = {0.5f, -1.5f, 2.25f, -0.75f};
    const std::array<float, 3> x3_f = {1.25f, -0.5f, 2.0f};
    std::array<double, 12> mat_3x4_d{};
    std::array<double, 4> x4_d{};
    std::array<double, 3> x3_d{};
    for (std::size_t i = 0; i < 12; ++i) {
        mat_3x4_d[i] = static_cast<double>(mat_3x4_f[i]);
    }
    for (std::size_t i = 0; i < 4; ++i) {
        x4_d[i] = static_cast<double>(x4_f[i]);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        x3_d[i] = static_cast<double>(x3_f[i]);
    }

    require(std::abs(static_cast<double>(GpuToolkit::dot2(x4_f.data(), mat_3x4_f.data())) -
                     CpuExactToolkit::dot2(x4_d.data(), mat_3x4_d.data())) <= 1e-6,
            "GpuToolkit::dot2 vs CpuExactToolkit");
    require(std::abs(static_cast<double>(GpuToolkit::dot3(x3_f.data(), mat_3x4_f.data())) -
                     CpuExactToolkit::dot3(x3_d.data(), mat_3x4_d.data())) <= 1e-6,
            "GpuToolkit::dot3 vs CpuExactToolkit");
    require(std::abs(static_cast<double>(GpuToolkit::dot(4, x4_f.data(), mat_3x4_f.data())) -
                     CpuExactToolkit::dot(4, x4_d.data(), mat_3x4_d.data())) <= 1e-6,
            "GpuToolkit::dot vs CpuExactToolkit");
    require(std::abs(static_cast<double>(GpuToolkit::norm2_ddot(x4_f.data())) -
                     CpuExactToolkit::norm2_ddot(x4_d.data())) <= 1e-6,
            "GpuToolkit::norm2_ddot vs CpuExactToolkit");
    require(std::abs(static_cast<double>(GpuToolkit::norm3_ddot(x3_f.data())) -
                     CpuExactToolkit::norm3_ddot(x3_d.data())) <= 1e-6,
            "GpuToolkit::norm3_ddot vs CpuExactToolkit");
    require(std::abs(static_cast<double>(GpuToolkit::einsum2(x3_f[0], x3_f[1], x4_f[0], x4_f[1])) -
                     CpuExactToolkit::einsum2(x3_d[0], x3_d[1], x4_d[0], x4_d[1])) <= 1e-6,
            "GpuToolkit::einsum2 vs CpuExactToolkit");

    const std::array<float, 6> mat_3x2_f = {1.0f, 2.0f, -3.0f, 4.0f, 0.5f, -1.5f};
    const std::array<float, 2> x2_f = {2.5f, -0.75f};
    std::array<double, 6> mat_3x2_d{};
    std::array<double, 2> x2_d{};
    for (std::size_t i = 0; i < 6; ++i) {
        mat_3x2_d[i] = static_cast<double>(mat_3x2_f[i]);
    }
    for (std::size_t i = 0; i < 2; ++i) {
        x2_d[i] = static_cast<double>(x2_f[i]);
    }
    std::array<float, 3> yn2_f{};
    std::array<double, 3> yn2_d{};
    GpuToolkit::gemv_n2(3, mat_3x2_f.data(), x2_f.data(), yn2_f.data());
    CpuExactToolkit::gemv_n2(3, mat_3x2_d.data(), x2_d.data(), yn2_d.data());
    for (std::size_t i = 0; i < 3; ++i) {
        require(std::abs(static_cast<double>(yn2_f[i]) - yn2_d[i]) <= 1e-6,
                "GpuToolkit::gemv_n2 vs CpuExactToolkit");
    }

    std::array<float, 3> yn3_f{};
    std::array<double, 3> yn3_d{};
    GpuToolkit::gemv_n3(4, mat_3x4_f.data(), x4_f.data(), yn3_f.data());
    CpuExactToolkit::gemv_n3(4, mat_3x4_d.data(), x4_d.data(), yn3_d.data());
    for (std::size_t i = 0; i < 3; ++i) {
        require(std::abs(static_cast<double>(yn3_f[i]) - yn3_d[i]) <= 1e-6,
                "GpuToolkit::gemv_n3 vs CpuExactToolkit");
    }

    std::array<float, 4> yt3_f{};
    std::array<double, 4> yt3_d{};
    GpuToolkit::gemv_t3(4, mat_3x4_f.data(), x3_f.data(), yt3_f.data());
    CpuExactToolkit::gemv_t3(4, mat_3x4_d.data(), x3_d.data(), yt3_d.data());
    for (std::size_t i = 0; i < 4; ++i) {
        require(std::abs(static_cast<double>(yt3_f[i]) - yt3_d[i]) <= 1e-6,
                "GpuToolkit::gemv_t3 vs CpuExactToolkit");
    }

    const std::array<float, 6> L_f = {0.35f, 0.0f, 0.1f, -0.25f, 0.0f, 0.3f};
    const std::array<float, 9> R_f = {
        0.9f, 0.0f, 0.43588989f,
        0.0f, 1.0f, 0.0f,
        -0.43588989f, 0.0f, 0.9f
    };
    std::array<double, 6> L_d{};
    std::array<double, 9> R_d{};
    for (std::size_t i = 0; i < 6; ++i) {
        L_d[i] = static_cast<double>(L_f[i]);
    }
    for (std::size_t i = 0; i < 9; ++i) {
        R_d[i] = static_cast<double>(R_f[i]);
    }
    std::array<float, 6> out_f{};
    std::array<double, 6> out_d{};
    GpuToolkit::ring_lift(2, L_f.data(), R_f.data(), out_f.data());
    CpuExactToolkit::ring_lift(2, L_d.data(), R_d.data(), out_d.data());
    for (std::size_t i = 0; i < 6; ++i) {
        require(std::abs(static_cast<double>(out_f[i]) - out_d[i]) <= 1e-6,
                "GpuToolkit::ring_lift vs CpuExactToolkit");
    }
}

#if defined(__APPLE__)
void test_live_apple_metal_parity() {
    namespace tm = bike::physics_core::test_metal;
    constexpr std::size_t kN = 1024u;
    tm::MetalParityRequest req;
    req.physics_core_dir = BIKE_PHYSICS_CORE_SRC_DIR;
    req.exp_inputs.resize(kN);
    req.tanh_inputs.resize(kN);
    req.hypot_a.resize(kN);
    req.hypot_b.resize(kN);
    req.interp_s.resize(kN);
    req.interp_x.resize(kN);
    req.interp_x0.resize(kN);
    req.interp_y0.resize(kN);
    req.interp_x1.resize(kN);
    req.interp_y1.resize(kN);
    req.dot3_a.resize(3u * kN);
    req.dot3_b.resize(3u * kN);
    req.ring_local.resize(3u * kN);
    req.ring_rmat = {
        0.92106099f, 0.0f, 0.38941834f,
        0.0f, 1.0f, 0.0f,
        -0.38941834f, 0.0f, 0.92106099f
    };

    for (std::size_t i = 0; i < kN; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(kN - 1u);
        req.exp_inputs[i] = -20.0f + 40.0f * u;
        req.tanh_inputs[i] = -12.0f + 24.0f * u;
        req.hypot_a[i] = (i % 2u == 0u ? 1.0f : -1.0f) * (1e-20f + 1e20f * u);
        req.hypot_b[i] = (i % 3u == 0u ? 1.0f : -1.0f) * (1e-20f + 0.5e20f * (1.0f - u));
        req.interp_x0[i] = static_cast<float>(i) * 0.1f;
        req.interp_x1[i] = req.interp_x0[i] + 0.1f;
        req.interp_y0[i] = -5.0f + 10.0f * u;
        req.interp_y1[i] = req.interp_y0[i] + (i % 17u == 0u ? 0.0f : (0.5f - u));
        req.interp_s[i] = (req.interp_y1[i] - req.interp_y0[i]) / (req.interp_x1[i] - req.interp_x0[i]);
        req.interp_x[i] = req.interp_x0[i] + 0.037f;

        req.dot3_a[3u * i + 0u] = -1.5f + 3.0f * u;
        req.dot3_a[3u * i + 1u] = 0.25f * static_cast<float>(i % 7u);
        req.dot3_a[3u * i + 2u] = 2.0f - 4.0f * u;
        req.dot3_b[3u * i + 0u] = 0.75f - 1.5f * u;
        req.dot3_b[3u * i + 1u] = -0.5f + static_cast<float>(i % 5u);
        req.dot3_b[3u * i + 2u] = 1.25f + 0.5f * u;

        const float theta = 6.283185307179586f * u;
        req.ring_local[3u * i + 0u] = 0.35f * std::cos(theta);
        req.ring_local[3u * i + 1u] = 0.0f;
        req.ring_local[3u * i + 2u] = 0.35f * std::sin(theta);
    }
    req.tanh_inputs[0] = 0.22045f;
    req.tanh_inputs[1] = -0.17328729f;

    const tm::MetalParityOutput gpu = tm::run_metal_parity_kernel(req);
    require(gpu.device_available, "Metal device must be available on Apple Silicon");
    if (!gpu.error_message.empty()) {
        throw std::runtime_error(gpu.error_message);
    }

    require_bits_f32(gpu.bike_real_1e_12, BIKE_REAL(float, 1e-12), "Metal BIKE_REAL(float, 1e-12)");
    require(gpu.sizeof_tyre_model == sizeof(bike::physics_core::TyreModel<float>),
            "Metal sizeof(TyreModel<float>) mismatch");
    require(gpu.sizeof_compliant_state == sizeof(bike::physics_core::CompliantWheelStatePod<float>),
            "Metal sizeof(CompliantWheelStatePod<float>) mismatch");
    require(gpu.sizeof_station_state == sizeof(bike::physics_core::StationStatePod<float>),
            "Metal sizeof(StationStatePod<float>) mismatch");

    for (std::size_t i = 0; i < kN; ++i) {
        const float h_exp = GpuToolkit::exp(req.exp_inputs[i]);
        const float h_tanh = GpuToolkit::tanh(req.tanh_inputs[i]);
        const float h_hypot = GpuToolkit::hypot2(req.hypot_a[i], req.hypot_b[i]);
        const float h_interp = GpuToolkit::interp_segment(
            req.interp_s[i], req.interp_x[i],
            req.interp_x0[i], req.interp_y0[i],
            req.interp_x1[i], req.interp_y1[i]);
        const float h_dot3 = GpuToolkit::dot3(&req.dot3_a[3u * i], &req.dot3_b[3u * i]);

        require_bits_f32(gpu.exp_out[i], h_exp, "Metal vs Host GpuToolkit::exp");
        require_bits_f32(gpu.tanh_out[i], h_tanh, "Metal vs Host GpuToolkit::tanh");
        require_bits_f32(gpu.hypot_out[i], h_hypot, "Metal vs Host GpuToolkit::hypot2");
        require_bits_f32(gpu.interp_out[i], h_interp, "Metal vs Host GpuToolkit::interp_segment");
        require_bits_f32(gpu.dot3_out[i], h_dot3, "Metal vs Host GpuToolkit::dot3");

        std::array<float, 3> h_ring{};
        GpuToolkit::ring_lift(1, &req.ring_local[3u * i], req.ring_rmat.data(), h_ring.data());
        for (std::size_t k = 0; k < 3u; ++k) {
            require_bits_f32(gpu.ring_out[3u * i + k], h_ring[k], "Metal vs Host GpuToolkit::ring_lift");
        }
    }
    std::cout << "[test_physics_core_math] Apple Metal MTLMathModeSafe parity PASS on "
              << gpu.device_name << " (1024 x 6 ops, 0 bit diffs)\n";
}
#endif

} // namespace

int main() {
    try {
        test_common_and_types();
        test_expf_portable();
        test_tanhf_portable();
        test_hypot2f_portable();
        test_interp_segment_unfused();
        test_cpu_exact_toolkit_parity();
        test_gpu_toolkit_linalg_vs_cpu();
#if defined(__APPLE__)
        test_live_apple_metal_parity();
#endif
        std::cout << "[test_physics_core_math] ALL PASS\n";
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "[test_physics_core_math] FAIL: " << ex.what() << '\n';
        return 1;
    }
}
