// numeric_norm.hpp — CPython 3.14.3 Modules/mathmodule.c vector_norm,
// extracted verbatim in operation order and shared by the ports whose
// oracle is `math.hypot` (NOT the numpy `np.linalg.norm` → BLAS contract
// — those call sites keep their own sqrt(ddot(v,v)) form by design).
//
// The kernel is DoubleLength compensated arithmetic (Ogita/Rump/Oishi):
// lossless power-of-two scaling, exact squaring via fma, compensated
// summation, then a square-root differential correction. Explicit fma
// calls are confined to the error-free products, so the port is
// unaffected by -ffp-contract=off. This is also not the platform libm
// hypot — it was verified to differ on this build, which is why the
// vector_norm lowering exists at all.
//
// Use `python_vector_norm` only where the ported expression is CPython's
// hypot()/dist(). Sites whose contract is a different reduction —
// np.linalg.norm's BLAS path, np.einsum's seeded accumulation, or
// builtin sum()'s Neumaier loop — intentionally keep their separate,
// named implementations elsewhere.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace numeric {
    namespace detail {
        // DoubleLength (mathmodule.c): an exact (hi, lo) split of an
        // arithmetic result.
        struct DLen {
            double hi, lo;
        };

        // dl_fast_sum — Algorithm 1.1 compensated sum (|a| >= |b| is the
        // caller's invariant, guaranteed by the csum >= 1.0 seed below).
        inline DLen dl_fast_sum(double a, double b) noexcept {
            const double x = a + b;
            const double y = (a - x) + b;
            return {.hi = x, .lo = y};
        }

        // dl_mul — Algorithm 3.5 error-free product via fma (the reliable
        // FMA path; CPython's UNRELIABLE_FMA Veltkamp fallback does not
        // apply to this toolchain).
        inline DLen dl_mul(double x, double y) noexcept {
            const double z = x * y;
            const double zz = std::fma(x, y, -z);
            return {.hi = z, .lo = zz};
        }

        // vector_norm(n, vec, max, found_nan): `vec` holds the absolute
        // values and may be rescaled in place on the subnormal path,
        // exactly like CPython's buffer.
        // NOLINTNEXTLINE(misc-no-recursion) subnormal-rescale recursion mirrors CPython mathmodule.c
        inline double vector_norm(std::span<double> vec, double mx,
                                  bool found_nan) noexcept {
            if (std::isinf(mx))
                return mx;
            if (found_nan)
                return std::numeric_limits<double>::quiet_NaN();
            if (mx == 0.0 || vec.size() <= 1)
                return mx;
            int max_e = 0;
            (void) std::frexp(mx, &max_e);
            if (max_e < -1023) {
                // When max_e < -1023, ldexp(1.0, -max_e) would overflow;
                // rescale subnormals to normals and recurse verbatim.
                for (double &v: vec)
                    v /= std::numeric_limits<double>::min();
                return std::numeric_limits<double>::min() *
                       vector_norm(
                           vec, mx / std::numeric_limits<double>::min(),
                           found_nan);
            }
            const double scale = std::ldexp(1.0, -max_e);
            double csum = 1.0, frac1 = 0.0, frac2 = 0.0;
            for (const double v: vec) {
                const double x = v * scale; // lossless scaling
                const DLen pr = dl_mul(x, x); // lossless squaring
                const DLen sm = dl_fast_sum(csum, pr.hi); // lossless addition
                csum = sm.hi;
                frac1 += pr.lo; // lossy addition
                frac2 += sm.lo; // lossy addition
            }
            double h = std::sqrt(csum - 1.0 + (frac1 + frac2));
            const DLen pr = dl_mul(-h, h);
            const DLen sm = dl_fast_sum(csum, pr.hi);
            csum = sm.hi;
            frac1 += pr.lo;
            frac2 += sm.lo;
            const double x = csum - 1.0 + (frac1 + frac2);
            h += x / (2.0 * h); // differential correction
            return h / scale;
        }
    } // namespace detail

    // math_hypot_impl (mathmodule.c) over a span: coordinates are |x|
    // into a private buffer — the small-buffer threshold mirrors
    // CPython's NUM_STACK_ELEMS so the common 2/3-element calls never
    // reach the heap. The empty span is hypot() == 0.0 by the
    // max == 0.0 || n <= 1 early return.
    [[nodiscard]] inline double python_vector_norm(
        std::span<const double> values) {
        constexpr std::size_t kStackElems = 16;
        std::array<double, kStackElems> on_stack{};
        std::vector<double> heap;
        std::span<double> coordinates;
        if (values.size() <= kStackElems) {
            // span-from-array then subspan keeps the bound inside the
            // container type — the (pointer, size) ctor is rejected by
            // -Wunsafe-buffer-usage-in-container.
            coordinates =
                    std::span<double>(on_stack).subspan(0, values.size());
        } else {
            heap.resize(values.size());
            coordinates = heap;
        }
        double mx = 0.0;
        bool found_nan = false;
        for (std::size_t i = 0; i < values.size(); ++i) {
            const double x = std::fabs(values[i]);
            coordinates[i] = x;
            found_nan = found_nan || std::isnan(x);
            if (x > mx)
                mx = x;
        }
        return detail::vector_norm(coordinates, mx, found_nan);
    }
} // namespace numeric
