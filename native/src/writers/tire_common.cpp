// writers/tire_common.cpp — shared tyre-writer helpers extracted from
// writers/tire.cpp: surface_mu/surface_at moved character-for-character
// (the diff that deletes them from tire.cpp is the parity proof), and
// effective_friction assembled from tire.cpp's inlined tire_forces.py:
// 56-61 block under the plan's signature.
#include "tire_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace {
    [[nodiscard]] bool
    same_surface_spec(const nativecfg::SurfaceSpec &a,
                      const nativecfg::SurfaceSpec &b) noexcept {
        return a.name == b.name &&
               a.mu_peak == b.mu_peak &&
               a.mu_slide == b.mu_slide &&
               a.slip_stiffness_per_load == b.slip_stiffness_per_load &&
               a.stribeck_speed_mps == b.stribeck_speed_mps &&
               a.crr_delta == b.crr_delta &&
               a.load_sensitivity == b.load_sensitivity;
    }

    [[nodiscard]] const nativecfg::SurfaceSpec &
    pure_surface_at(const nativecfg::SurfaceMap &map, double x_m) noexcept {
        if (!map.sections.empty() && map.sections.front().start_m == 0.0 &&
            x_m < 0.0) {
            return map.sections.front().surface;
        }
        for (const auto &sec : map.sections)
            if (sec.start_m <= x_m && x_m < sec.end_m)
                return sec.surface;
        return map.surface;
    }

    struct TransitionBoundary {
        double x_b = 0.0;
        double width_m = 0.0;
        const nativecfg::SurfaceSpec *left = nullptr;
        const nativecfg::SurfaceSpec *right = nullptr;
    };

    // Visits active (blend_m > 0) transition boundaries in ascending x_b order
    // with zero heap allocations. `fn(boundary)` returns true to stop early.
    template <class Fn>
    void for_each_transition(const nativecfg::SurfaceMap &map, const Fn &fn) {
        const std::size_t n = map.sections.size();
        if (n == 0)
            return;
        if (map.sections[0].start_m > 0.0 && map.sections[0].blend_m > 0.0) {
            if (fn(TransitionBoundary{.x_b = map.sections[0].start_m,
                                      .width_m = map.sections[0].blend_m,
                                      .left = &map.surface,
                                      .right = &map.sections[0].surface})) {
                return;
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            const auto &sec = map.sections[i];
            if (i + 1 < n && sec.end_m == map.sections[i + 1].start_m) {
                if (sec.blend_m > 0.0 &&
                    fn(TransitionBoundary{
                            .x_b = sec.end_m,
                            .width_m = sec.blend_m,
                            .left = &sec.surface,
                            .right = &map.sections[i + 1].surface})) {
                    return;
                }
            } else {
                if (sec.blend_m > 0.0 &&
                    fn(TransitionBoundary{.x_b = sec.end_m,
                                          .width_m = sec.blend_m,
                                          .left = &sec.surface,
                                          .right = &map.surface})) {
                    return;
                }
                if (i + 1 < n && map.sections[i + 1].blend_m > 0.0 &&
                    fn(TransitionBoundary{
                            .x_b = map.sections[i + 1].start_m,
                            .width_m = map.sections[i + 1].blend_m,
                            .left = &map.surface,
                            .right = &map.sections[i + 1].surface})) {
                    return;
                }
            }
        }
    }

    [[nodiscard]] tirecommon::SurfaceSample
    blend_specs(const TransitionBoundary &tr, double x_m) noexcept {
        const nativecfg::SurfaceSpec &left = *tr.left;
        const nativecfg::SurfaceSpec &right = *tr.right;
        const nativecfg::SurfaceSpec &canonical = x_m < tr.x_b ? left : right;
        if (same_surface_spec(left, right))
            return tirecommon::sample_from_spec(canonical);
        const double t = std::clamp(0.5 + (x_m - tr.x_b) / tr.width_m, 0.0, 1.0);
        const double s = t * t * (3.0 - 2.0 * t);
        const double w_l = 1.0 - s;
        return {
            .name = canonical.name,
            .mu_peak = w_l * left.mu_peak + s * right.mu_peak,
            .mu_slide = w_l * left.mu_slide + s * right.mu_slide,
            .slip_stiffness_per_load = w_l * left.slip_stiffness_per_load +
                                       s * right.slip_stiffness_per_load,
            .stribeck_speed_mps = w_l * left.stribeck_speed_mps +
                                  s * right.stribeck_speed_mps,
            .crr_delta = w_l * left.crr_delta + s * right.crr_delta,
            .load_sensitivity =
                    w_l * left.load_sensitivity + s * right.load_sensitivity,
        };
    }
} // namespace

namespace tirecommon {
    // surface.py:72-88 — SurfaceSpec.mu: mu_slide + (peak-slide)*exp(-|v|/str).
    double surface_mu(const nativecfg::SurfaceSpec &s, double slip) {
        const double decay =
                std::exp(-std::abs(slip) / s.stribeck_speed_mps);
        return s.mu_slide + (s.mu_peak - s.mu_slide) * decay;
    }

    // surface.py:194-206 — SurfaceMap.at: finite query, first matching
    // half-open section wins, else the default material.
    const nativecfg::SurfaceSpec &
    surface_at(const nativecfg::SurfaceMap &map, double x_m) {
        if (!std::isfinite(x_m))
            throw std::invalid_argument("material query must be finite");
        for (const auto &sec: map.sections)
            if (sec.start_m <= x_m && x_m < sec.end_m)
                return sec.surface;
        return map.surface;
    }

    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror the call site's Python order
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    std::pair<double, std::string_view>
    effective_friction(double tire_mu, const nativecfg::SurfaceMap *map,
                       double contact_x, double slip) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        if (map == nullptr)
            return {tire_mu, "configured"};
        const nativecfg::SurfaceSpec &surf = surface_at(*map, contact_x);
        return {std::min(tire_mu, surface_mu(surf, slip)), surf.name};
    }

    SurfaceSample sample_from_spec(const nativecfg::SurfaceSpec &s) noexcept {
        return {
            .name = s.name,
            .mu_peak = s.mu_peak,
            .mu_slide = s.mu_slide,
            .slip_stiffness_per_load = s.slip_stiffness_per_load,
            .stribeck_speed_mps = s.stribeck_speed_mps,
            .crr_delta = s.crr_delta,
            .load_sensitivity = s.load_sensitivity,
        };
    }

    SurfaceSample eval_surface(const nativecfg::SurfaceMap &map, double x_m) {
        if (!std::isfinite(x_m))
            throw std::invalid_argument("material query must be finite");
        SurfaceSample blended{};
        bool matched = false;
        for_each_transition(
                map, [&](const TransitionBoundary &tr) noexcept {
                    const double half = 0.5 * tr.width_m;
                    if (tr.x_b - half < x_m && x_m < tr.x_b + half) {
                        blended = blend_specs(tr, x_m);
                        matched = true;
                        return true;
                    }
                    return false;
                });
        if (matched)
            return blended;
        return sample_from_spec(pure_surface_at(map, x_m));
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    const nativecfg::SurfaceSpec *
    uniform_surface_for_span(const nativecfg::SurfaceMap &map, double x_min_m,
                             double x_max_m) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        if (!std::isfinite(x_min_m) || !std::isfinite(x_max_m) ||
            x_min_m > x_max_m) {
            throw std::invalid_argument(
                    "uniform_surface_for_span requires finite x_min_m <= x_max_m");
        }
        const std::size_t n = map.sections.size();
        if (n == 0)
            return &map.surface;

        bool touches_transition = false;
        for_each_transition(
                map, [&](const TransitionBoundary &tr) noexcept {
                    const double half = 0.5 * tr.width_m;
                    if (x_min_m <= tr.x_b + half && x_max_m >= tr.x_b - half) {
                        touches_transition = true;
                        return true;
                    }
                    return false;
                });
        if (touches_transition)
            return nullptr;

        if (map.sections[0].start_m == 0.0) {
            if (x_max_m < map.sections[0].end_m)
                return &map.sections[0].surface;
        } else {
            if (x_max_m < map.sections[0].start_m)
                return &map.surface;
            if (map.sections[0].start_m <= x_min_m &&
                x_max_m < map.sections[0].end_m) {
                return &map.sections[0].surface;
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (i >= 1 && map.sections[i].start_m <= x_min_m &&
                x_max_m < map.sections[i].end_m) {
                return &map.sections[i].surface;
            }
            if (i + 1 < n &&
                map.sections[i].end_m < map.sections[i + 1].start_m &&
                map.sections[i].end_m <= x_min_m &&
                x_max_m < map.sections[i + 1].start_m) {
                return &map.surface;
            }
        }
        if (map.sections[n - 1].end_m <= x_min_m)
            return &map.surface;
        return nullptr;
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    double load_sensitivity_factor(double wheel_normal_load_n,
                                   double k_z) noexcept {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        if (wheel_normal_load_n <= kSurfaceReferenceLoadN || k_z <= 0.0)
            return 1.0;
        const double u = wheel_normal_load_n / kSurfaceReferenceLoadN - 1.0;
        if (u <= 0.0)
            return 1.0;
        const double g = (u * u) / (u + kSurfaceLoadSmoothDelta);
        return 1.0 / (1.0 + k_z * g);
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters)
    double sample_mu(const SurfaceSample &s, double sliding_speed_mps,
                     double wheel_normal_load_n) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        if (!std::isfinite(sliding_speed_mps))
            throw std::invalid_argument(
                    "invalid shape or non-finite SurfaceSample.sliding_speed_mps");
        if (!std::isfinite(wheel_normal_load_n) || wheel_normal_load_n < 0.0)
            throw std::invalid_argument(
                    "SurfaceSample.wheel_normal_load_n must be finite and nonnegative");
        const double decay =
                std::exp(-std::abs(sliding_speed_mps) / s.stribeck_speed_mps);
        const double mu_str = s.mu_slide + (s.mu_peak - s.mu_slide) * decay;
        const double value =
                (wheel_normal_load_n <= kSurfaceReferenceLoadN ||
                 s.load_sensitivity <= 0.0)
                        ? mu_str
                        : load_sensitivity_factor(wheel_normal_load_n,
                                                  s.load_sensitivity) *
                                  mu_str;
        if (!std::isfinite(value))
            throw std::overflow_error(
                    "SurfaceSample.mu: non-finite derived value");
        return value;
    }
} // namespace tirecommon
