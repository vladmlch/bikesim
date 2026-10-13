// writers/tire_common.cpp — shared tyre-writer helpers extracted from
// writers/tire.cpp: surface_mu/surface_at moved character-for-character
// (the diff that deletes them from tire.cpp is the parity proof), and
// effective_friction assembled from tire.cpp's inlined tire_forces.py:
// 56-61 block under the plan's signature.
#include "tire_common.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
} // namespace tirecommon
