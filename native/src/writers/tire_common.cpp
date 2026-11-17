// writers/tire_common.cpp — shared tyre-writer helpers extracted from
// writers/tire.cpp: surface_mu/surface_at moved character-for-character
// (the diff that deletes them from tire.cpp is the parity proof), and
// effective_friction assembled from tire.cpp's inlined tire_forces.py:
// 56-61 block under the plan's signature.
#include "tire_common.hpp"

#include "../contact/laws.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_contact.hpp"
#include "../physics_core/tyre_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>

namespace tirecommon {
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror tire.py brush step
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    BrushResult brush_step(double xi, double u, double v_roll, double fn,
                           double k, double mu, double length, double dt) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const auto [new_xi, force, loss] =
                contactlaw::brush_step(xi, u, v_roll, fn, k, mu, length, dt);
        return {.xi = new_xi, .force = force, .loss = loss};
    }

    bike::physics_core::SurfaceSpecPod<double>
    to_surface_spec_pod(const nativecfg::SurfaceSpec &s) noexcept {
        return {
            .mu_peak = s.mu_peak,
            .mu_slide = s.mu_slide,
            .slip_stiffness_per_load = s.slip_stiffness_per_load,
            .stribeck_speed_mps = s.stribeck_speed_mps,
            .surface_id = 0
        };
    }

    bool fill_surface_map_view(
        const nativecfg::SurfaceMap *map,
        bike::physics_core::SurfaceMapView<double> &out
    ) noexcept {
        out = {};
        if (map == nullptr ||
            map->sections.size() >
                static_cast<std::size_t>(bike::physics_core::kMaxSurfaceSections)) {
            return false;
        }
        out.default_surface = to_surface_spec_pod(map->surface);
        out.section_count = static_cast<bike::physics_core::i32>(
            map->sections.size());
        out.has_surface_map = 1;
        const std::span<bike::physics_core::SurfaceSectionView<double>,
                        bike::physics_core::kMaxSurfaceSections>
            dst_sections(out.sections);
        for (std::size_t i = 0; i < map->sections.size(); ++i) {
            dst_sections[i].start_m = map->sections[i].start_m;
            dst_sections[i].end_m = map->sections[i].end_m;
            dst_sections[i].surface = to_surface_spec_pod(map->sections[i].surface);
        }
        return true;
    }
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
