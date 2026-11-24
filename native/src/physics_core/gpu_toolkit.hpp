// native/src/physics_core/gpu_toolkit.hpp — Tri-target FP32 numerical toolkit
// deterministic across Host C++, NVIDIA CUDA (--fmad=false), and Apple Metal (MTLMathModeSafe).
#pragma once

#include "common.hpp"
#include "gpu_math.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel toolkit: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

struct GpuToolkit {
    using Real = float;

    BIKE_HD static float div(float a, float b) noexcept {
        return pf_div(a, b);
    }

    BIKE_HD static float sqrt(float x) noexcept {
        return pf_sqrt(x);
    }

    BIKE_HD static float abs(float x) noexcept {
        return pf_abs(x);
    }

    BIKE_HD static bool isfinite(float x) noexcept {
        return pf_isfinite(x);
    }

    BIKE_HD static bool isnan(float x) noexcept {
        return pf_isnan(x);
    }

    BIKE_HD static float exp(float x) noexcept {
        return expf_portable(x);
    }

    BIKE_HD static float log(float x) noexcept {
        return logf_portable(x);
    }

    BIKE_HD static float pow_py(float a, float b) noexcept {
        return powf_portable(a, b);
    }

    BIKE_HD static float tanh(float x) noexcept {
        return tanhf_portable(x);
    }

    BIKE_HD static float floor(float x) noexcept {
        return pf_floor(x);
    }

    BIKE_HD static float sq(float x) noexcept {
        return x * x;
    }

    BIKE_HD static float pow2(float x) noexcept {
        return x * x;
    }

    BIKE_HD static float pow3(float x) noexcept {
        return (x * x) * x;
    }

    BIKE_HD static float hypot2(float a, float b) noexcept {
        return hypot2f_portable(a, b);
    }

    BIKE_HD static float interp_segment(
        float s, float x, float x0, float y0, float x1, float y1
    ) noexcept {
        return interp_segment_unfused(s, x, x0, y0, x1, y1);
    }

    BIKE_HD static float min(float a, float b) noexcept {
        return pf_min(a, b);
    }

    BIKE_HD static float max(float a, float b) noexcept {
        return pf_max(a, b);
    }

    BIKE_HD static float clamp(float x, float lo, float hi) noexcept {
        return pf_clamp(x, lo, hi);
    }

    template <class PtrA, class PtrB>
    BIKE_HD static float dot2(PtrA a, PtrB b) noexcept {
        return 0.0f + a[0] * b[0] + a[1] * b[1];
    }

    template <class PtrA, class PtrB>
    BIKE_HD static float dot3(PtrA a, PtrB b) noexcept {
        return 0.0f + a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    template <class PtrA, class PtrB>
    BIKE_HD static float dot(int n, PtrA a, PtrB b) noexcept {
        float s = 0.0f;
        for (int i = 0; i < n; ++i) {
            s += a[i] * b[i];
        }
        return s;
    }

    template <class PtrVals, class PtrMask>
    BIKE_HD static float masked_sum(int n, PtrVals vals, PtrMask mask) noexcept {
        float s = 0.0f;
        for (int i = 0; i < n; ++i) {
            if (mask[i] != 0) {
                s += vals[i];
            }
        }
        return s;
    }

    template <class PtrA>
    BIKE_HD static float norm2_ddot(PtrA a) noexcept {
        return pf_sqrt(dot2(a, a));
    }

    template <class PtrA>
    BIKE_HD static float norm3_ddot(PtrA a) noexcept {
        return pf_sqrt(dot3(a, a));
    }

    BIKE_HD static float einsum2(float a0, float a1, float b0, float b1) noexcept {
        return 0.0f + a0 * b0 + a1 * b1;
    }

    template <class PtrA, class PtrX, class PtrY>
    BIKE_HD static void gemv_n3(int nv, PtrA A, PtrX x, PtrY y) noexcept {
        for (int r = 0; r < 3; ++r) {
            float acc = 0.0f;
            const int base = r * nv;
            for (int c = 0; c < nv; ++c) {
                acc += A[base + c] * x[c];
            }
            y[r] = acc;
        }
    }

    template <class PtrA, class PtrX, class PtrY>
    BIKE_HD static void gemv_t3(int nv, PtrA A, PtrX x, PtrY y) noexcept {
        for (int c = 0; c < nv; ++c) {
            float acc = 0.0f;
            for (int r = 0; r < 3; ++r) {
                acc += A[r * nv + c] * x[r];
            }
            y[c] = acc;
        }
    }

    template <class PtrA, class PtrX, class PtrY>
    BIKE_HD static void gemv_n2(int rows, PtrA A, PtrX x, PtrY y) noexcept {
        for (int r = 0; r < rows; ++r) {
            const int base = r * 2;
            y[r] = 0.0f + A[base] * x[0] + A[base + 1] * x[1];
        }
    }

    template <class PtrL, class PtrR, class PtrOut>
    BIKE_HD static void ring_lift(int n, PtrL L, PtrR R, PtrOut out) noexcept {
        for (int i = 0; i < n; ++i) {
            const int l_base = i * 3;
            const float lx = L[l_base + 0];
            const float ly = L[l_base + 1];
            const float lz = L[l_base + 2];
            for (int r = 0; r < 3; ++r) {
                const int r_base = r * 3;
                out[l_base + r] =
                    0.0f + lx * R[r_base + 0] + ly * R[r_base + 1] + lz * R[r_base + 2];
            }
        }
    }
};

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
