#include "terrain/generator3d.hpp"

#include "random/numpy_compat.hpp"
#include "terrain/profile.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace bike::terrain {
namespace {

[[nodiscard]] double quintic_smoothstep(double t) noexcept {
    const double c = std::clamp(t, 0., 1.);
    return c * c * c * (c * (c * 6. - 15.) + 10.);
}

[[nodiscard]] double cross_slope_tan_at(double x_m, const CrossSlopeSegment3d& seg) noexcept {
    if (x_m <= seg.start_m || x_m >= seg.end_m || seg.bank_rad == 0.) {
        return 0.;
    }
    const double slope = std::tan(seg.bank_rad);
    if (!(seg.transition_m > 0.)) {
        return slope;
    }
    const double up = quintic_smoothstep((x_m - seg.start_m) / seg.transition_m);
    const double down = quintic_smoothstep((seg.end_m - x_m) / seg.transition_m);
    return slope * std::min(up, down);
}

[[nodiscard]] double lead_window(double x_m, const Terrain3dOverlaySpec& ov, double length_m) noexcept {
    // Keep 2 extra dx spacings (+ ULP guard) of exact zero inside lead-in/out
    // so 4-point bicubic stencils stay strictly flat across [0, lead_in_m]
    // and [length_m - lead_out_m, length_m].
    const double halo = 2. * ov.dx_m + 1e-12 * std::max(1., length_m);
    const double in_end = ov.lead_in_m + halo;
    const double out_start = length_m - ov.lead_out_m - halo;
    if (x_m <= in_end || x_m >= out_start) {
        return 0.;
    }
    constexpr double kFadeM = 1.0;
    const double w_in = quintic_smoothstep((x_m - in_end) / kFadeM);
    const double w_out = quintic_smoothstep((out_start - x_m) / kFadeM);
    return std::min(w_in, w_out);
}

} // namespace

void validate_overlay_spec(const TrackSpec& base_track, const Terrain3dOverlaySpec& overlay) {
    base_track.validate();
    if (!std::isfinite(overlay.half_width_m) || !(overlay.half_width_m >= 0.25) ||
        !std::isfinite(overlay.dx_m) || !(overlay.dx_m > 0.) ||
        !std::isfinite(overlay.dy_m) || !(overlay.dy_m > 0.) ||
        !std::isfinite(overlay.datum_z_offset_m) ||
        !std::isfinite(overlay.lead_in_m) || !(overlay.lead_in_m >= 0.) ||
        !std::isfinite(overlay.lead_out_m) || !(overlay.lead_out_m >= 0.) ||
        !std::isfinite(overlay.lateral_roughness_amplitude_m) ||
        !(overlay.lateral_roughness_amplitude_m >= 0.)) {
        throw std::invalid_argument("Terrain3dOverlaySpec: invalid dimensions or nonfinite parameter");
    }
    const double ny_real = std::nearbyint((2. * overlay.half_width_m) / overlay.dy_m) + 1.;
    if (!std::isfinite(ny_real) || ny_real < 5. || ny_real > 10000. ||
        std::abs((ny_real - 1.) * overlay.dy_m - 2. * overlay.half_width_m) > 1e-9) {
        throw std::invalid_argument(
            "Terrain3dOverlaySpec.dy_m: 2 * half_width_m must be an exact multiple of dy_m with 5 <= ny <= 10000");
    }
    for (const CrossSlopeSegment3d& seg : overlay.cross_slopes) {
        if (!std::isfinite(seg.start_m) || !std::isfinite(seg.end_m) || !(seg.end_m > seg.start_m) ||
            !std::isfinite(seg.transition_m) || !(seg.transition_m >= 0.) ||
            !std::isfinite(seg.bank_rad) || std::abs(seg.bank_rad) >= std::numbers::pi / 4.) {
            throw std::invalid_argument("Terrain3dOverlaySpec.cross_slopes: invalid segment or |bank| >= 45 deg");
        }
    }
    if (overlay.lateral_roughness_amplitude_m > 0.) {
        (void)bike::random::SeedSequence(overlay.seed);
    }
}

Grid3d build_overlay_grid3d(const TrackSpec& base_track, const Terrain3dOverlaySpec& overlay) {
    validate_overlay_spec(base_track, overlay);

    const std::vector<Point> base_pts = profile(base_track, overlay.dx_m);
    const std::size_t nx = base_pts.size();
    const auto ny = static_cast<std::size_t>(std::nearbyint((2. * overlay.half_width_m) / overlay.dy_m)) + 1U;

    std::vector<double> elevations(nx * ny, 0.);
    std::vector<double> roughness(nx * ny, 0.);
    if (overlay.lateral_roughness_amplitude_m > 0.) {
        const bike::random::SeedSequence seq(overlay.seed);
        bike::random::NumpyRng rng(seq);
        for (double& r : roughness) {
            r = overlay.lateral_roughness_amplitude_m * rng.normal01();
        }
    }

    for (std::size_t ix = 0; ix < nx; ++ix) {
        const double x = base_pts[ix][0];
        const double base_z = base_pts[ix][1] + overlay.datum_z_offset_m;
        const double fade = lead_window(x, overlay, base_track.length_m);
        double slope_y = 0.;
        if (fade > 0.) {
            for (const CrossSlopeSegment3d& seg : overlay.cross_slopes) {
                slope_y += cross_slope_tan_at(x, seg);
            }
        }
        for (std::size_t iy = 0; iy < ny; ++iy) {
            const double y = -overlay.half_width_m + static_cast<double>(iy) * overlay.dy_m;
            const double lat = slope_y * y + roughness[ix * ny + iy];
            elevations[ix * ny + iy] = fade == 0. ? base_z : (base_z + fade * lat);
        }
    }

    const SurfaceMap smap = base_track.surface_map();
    std::vector<rider::SurfaceLabel> cell_surfaces((nx - 1) * (ny - 1), rider::SurfaceLabel::hardpack);
    for (std::size_t ix = 0; ix + 1 < nx; ++ix) {
        const double x_mid = 0.5 * (base_pts[ix][0] + base_pts[ix + 1][0]);
        const std::string& sname = smap.at(x_mid).name;
        const auto lbl = rider::surface_label(sname);
        if (!lbl.has_value()) {
            throw std::invalid_argument("build_overlay_grid3d: unknown surface '" + sname + "'");
        }
        for (std::size_t iy = 0; iy + 1 < ny; ++iy) {
            cell_surfaces[ix * (ny - 1) + iy] = *lbl;
        }
    }

    return Grid3d(
        Grid3dSpec{
            .nx = nx,
            .ny = ny,
            .x0_m = base_pts.front()[0],
            .y0_m = -overlay.half_width_m,
            .dx_m = overlay.dx_m,
            .dy_m = overlay.dy_m,
        },
        std::move(elevations), std::move(cell_surfaces));
}

} // namespace bike::terrain
