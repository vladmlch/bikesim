// native/src/physics_core/common.hpp — Tri-target (Host C++20/23, CUDA, Apple Metal MSL)
// portability macros, fixed-width types, literal helpers, and status/error contracts.
#pragma once

#if defined(__METAL_VERSION__)
#include <metal_stdlib>
#ifndef noexcept
#define noexcept
#endif
#ifndef BIKE_HD
#define BIKE_HD inline
#endif
#ifndef BIKE_INLINE
#define BIKE_INLINE inline
#endif
#ifndef BIKE_THREAD
#define BIKE_THREAD thread
#endif
#ifndef BIKE_DEVICE
#define BIKE_DEVICE device
#endif
#ifndef BIKE_CONSTANT
#define BIKE_CONSTANT constant
#endif
#ifndef BIKE_GLOBAL
#define BIKE_GLOBAL kernel
#endif
#ifndef BIKE_CONSTEXPR
#define BIKE_CONSTEXPR constant constexpr
#endif
#elif defined(__CUDACC__)
#include <cstdint>
#ifndef BIKE_HD
#define BIKE_HD __host__ __device__ __forceinline__
#endif
#ifndef BIKE_INLINE
#define BIKE_INLINE __forceinline__
#endif
#ifndef BIKE_THREAD
#define BIKE_THREAD
#endif
#ifndef BIKE_DEVICE
#define BIKE_DEVICE
#endif
#ifndef BIKE_CONSTANT
#define BIKE_CONSTANT
#endif
#ifndef BIKE_GLOBAL
#define BIKE_GLOBAL __global__
#endif
#ifndef BIKE_CONSTEXPR
#define BIKE_CONSTEXPR inline constexpr
#endif
#else
#include <cstdint>
#ifndef BIKE_HD
#define BIKE_HD inline
#endif
#ifndef BIKE_INLINE
#define BIKE_INLINE inline
#endif
#ifndef BIKE_THREAD
#define BIKE_THREAD
#endif
#ifndef BIKE_DEVICE
#define BIKE_DEVICE
#endif
#ifndef BIKE_CONSTANT
#define BIKE_CONSTANT
#endif
#ifndef BIKE_GLOBAL
#define BIKE_GLOBAL
#endif
#ifndef BIKE_CONSTEXPR
#define BIKE_CONSTEXPR inline constexpr
#endif
#endif

#ifndef BIKE_GPU_SHARED_CODE_BEGIN
#if defined(__clang__) && !defined(__METAL_VERSION__) && !defined(__CUDA_ARCH__)
#define BIKE_GPU_SHARED_CODE_BEGIN \
    _Pragma("clang diagnostic push") \
    _Pragma("clang diagnostic ignored \"-Wunsafe-buffer-usage\"")
#define BIKE_GPU_SHARED_CODE_END _Pragma("clang diagnostic pop")
#else
#define BIKE_GPU_SHARED_CODE_BEGIN
#define BIKE_GPU_SHARED_CODE_END
#endif
#endif

#ifndef BIKE_REAL
#if defined(__METAL_VERSION__)
#define BIKE_REAL(Real, dec_lit) static_cast<Real>(dec_lit##f)
#else
#define BIKE_REAL(Real, dec_lit) static_cast<Real>(dec_lit)
#endif
#endif

#ifndef BIKE_REF
#if defined(__METAL_VERSION__)
// NOLINTNEXTLINE(bugprone-macro-parentheses) type-qualifier macro cannot parenthesize T in T&
#define BIKE_REF(T) thread T &
// NOLINTNEXTLINE(bugprone-macro-parentheses) type-qualifier macro cannot parenthesize T in const T&
#define BIKE_CREF(T) const thread T &
#else
// NOLINTNEXTLINE(bugprone-macro-parentheses) type-qualifier macro cannot parenthesize T in T&
#define BIKE_REF(T) T &
// NOLINTNEXTLINE(bugprone-macro-parentheses) type-qualifier macro cannot parenthesize T in const T&
#define BIKE_CREF(T) const T &
#endif
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL header: MSL has no std::span/std::array and address-space-qualified raw pointers are the device ABI. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

#if defined(__METAL_VERSION__)
using u8 = uchar;
using u16 = ushort;
using i32 = int;
using u32 = uint;
#else
using u8 = std::uint8_t;
using u16 = std::uint16_t;
using i32 = std::int32_t;
using u32 = std::uint32_t;
#endif

#if !defined(__METAL_VERSION__)
template <class Real>
BIKE_HD constexpr Real Lit(double v) noexcept {
    return static_cast<Real>(v);
}
template <class Real>
BIKE_HD constexpr Real Lit(float v) noexcept {
    return static_cast<Real>(v);
}
#else
template <class Real>
BIKE_HD constexpr Real Lit(float v) noexcept {
    return static_cast<Real>(v);
}
#endif

BIKE_CONSTEXPR int kMaxNq = 32;
BIKE_CONSTEXPR int kMaxNv = 32;
BIKE_CONSTEXPR int kMaxNu = 16;
BIKE_CONSTEXPR int kMaxNbody = 32;
BIKE_CONSTEXPR int kMaxNjnt = 32;
BIKE_CONSTEXPR int kMaxChainJoints = 8;
BIKE_CONSTEXPR int kMaxGpuTyreStations = 128;
BIKE_CONSTEXPR int kMaxHingeKnots = 8;
BIKE_CONSTEXPR int kMaxSurfaceSections = 16;

enum class Status : u8 {
    ok = 0,
    invalid_argument,
    overflow,
    out_of_bounds,
    logic_error
};

// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference/device ABI layout
enum class ErrCode : u16 {
    none = 0,
    nonfinite_center,
    invalid_radius,
    invalid_prev_seg,
    outside_domain,
    solid_road,
    degenerate_contact,
    nonunit_normal,
    logic_no_closest,
    overflow_nonfinite,
    overflow_passivity,
    missing_material,
    bad_dt,
    bad_time,
    out_of_bounds_chain,
    out_of_bounds_station,
    nonfinite_tangent,
    invalid_station_quad
};

struct Result {
    Status status = Status::ok;
    ErrCode err = ErrCode::none;
    BIKE_HD constexpr bool ok() const noexcept { return status == Status::ok; }
};

struct NoOpIo {};

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
