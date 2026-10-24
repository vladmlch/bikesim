#pragma once

// Internal, engine-independent arithmetic shared by the spatial tyre kernels.
#include "spatial/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace bike::pneumatic3d::detail {
using bike::spatial::Vec3;
inline constexpr std::size_t max_fan_elements = 256U * 15U;
inline constexpr double two_pi = 2. * std::numbers::pi;
inline double dot(const Vec3& a, const Vec3& b) noexcept {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
inline Vec3 add(const Vec3& a, const Vec3& b) noexcept {
    return {a[0]+b[0], a[1]+b[1], a[2]+b[2]};
}
inline Vec3 sub(const Vec3& a, const Vec3& b) noexcept {
    return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}
inline Vec3 scale(const Vec3& a, double s) noexcept {
    return {a[0]*s, a[1]*s, a[2]*s};
}
inline Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0]};
}
inline double norm(const Vec3& a) noexcept { return std::sqrt(dot(a,a)); }
inline bool finite(const Vec3& a) noexcept {
    return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}
inline Vec3 unit(const Vec3& a) noexcept {
    const double n = norm(a);
    return n > 0. ? scale(a, 1./n) : Vec3{};
}
inline double wrapped(double a) noexcept {
    double r = std::fmod(a, two_pi);
    if (r < 0.) r += two_pi;
    return r == two_pi ? 0. : r;
}
} // namespace bike::pneumatic3d::detail
