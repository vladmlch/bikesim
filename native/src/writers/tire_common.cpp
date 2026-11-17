// writers/tire_common.cpp — shared tyre-writer helpers extracted from
// writers/tire.cpp: surface_mu/surface_at moved character-for-character
// (the diff that deletes them from tire.cpp is the parity proof), and
// effective_friction assembled from tire.cpp's inlined tire_forces.py:
// 56-61 block under the plan's signature.
#include "tire_common.hpp"

#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_contact.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace tirecommon {
    // surface.py:72-88 — SurfaceSpec.mu: mu_slide + (peak-slide)*exp(-|v|/str).
    double surface_mu(const nativecfg::SurfaceSpec &s, double slip) {
        return bike::physics_core::surface_mu<
            double, bike::physics_core::CpuExactToolkit>(
                to_surface_spec_pod(s), slip);
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
        bike::physics_core::SurfaceMapView<double> view{};
        if (fill_surface_map_view(map, view)) {
            double mu = 0.0;
            bike::physics_core::i32 section_idx = -1;
            auto err = bike::physics_core::ErrCode::none;
            const auto status = bike::physics_core::effective_friction<
                double, bike::physics_core::CpuExactToolkit>(
                    tire_mu, &view, contact_x, slip, mu, section_idx, err);
            if (status != bike::physics_core::Status::ok)
                throw std::invalid_argument("material query must be finite");
            if (section_idx >= 0)
                return {mu,
                        map->sections[static_cast<std::size_t>(section_idx)]
                            .surface.name};
            return {mu, map->surface.name};
        }
        const nativecfg::SurfaceSpec &surf = surface_at(*map, contact_x);
        return {std::min(tire_mu, surface_mu(surf, slip)), surf.name};
    }
} // namespace tirecommon
