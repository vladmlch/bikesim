#include "terrain/heightfield3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bike::terrain {
namespace {

void validate_grid_spec_shape(const Grid3dSpec& spec) {
    if (spec.nx < 4 || spec.ny < 4 || spec.nx > 500000 || spec.ny > 10000) {
        throw std::invalid_argument(
            "sample_to_grid3d: nx and ny must be >= 4 and within bounded limits");
    }
    if (!std::isfinite(spec.x0_m) || !std::isfinite(spec.y0_m) ||
        !std::isfinite(spec.dx_m) || !std::isfinite(spec.dy_m) ||
        !(spec.dx_m > 0.) || !(spec.dy_m > 0.)) {
        throw std::invalid_argument(
            "sample_to_grid3d: finite origin and positive grid spacings required");
    }
}

template <class Source>
Grid3d sample_source_to_grid3d(const Source& source, const Grid3dSpec& spec) {
    validate_grid_spec_shape(spec);

    const std::size_t n_nodes = spec.nx * spec.ny;
    const std::size_t n_cells = (spec.nx - 1U) * (spec.ny - 1U);
    std::vector<double> elevations(n_nodes, 0.);
    for (std::size_t ix = 0; ix < spec.nx; ++ix) {
        const double x = spec.x0_m + static_cast<double>(ix) * spec.dx_m;
        for (std::size_t iy = 0; iy < spec.ny; ++iy) {
            const double y = spec.y0_m + static_cast<double>(iy) * spec.dy_m;
            const TerrainSample3d sample = source.sample(x, y);
            if (sample.status != QueryStatus3d::ok) {
                throw std::invalid_argument(
                    "sample_to_grid3d: sample coordinate falls outside terrain bounds");
            }
            elevations[ix * spec.ny + iy] = sample.height_m;
        }
    }

    std::vector<::rider::SurfaceLabel> cell_surfaces(n_cells, ::rider::SurfaceLabel::hardpack);
    for (std::size_t ix = 0; ix + 1U < spec.nx; ++ix) {
        const double x_mid = spec.x0_m + (static_cast<double>(ix) + 0.5) * spec.dx_m;
        for (std::size_t iy = 0; iy + 1U < spec.ny; ++iy) {
            const double y_mid = spec.y0_m + (static_cast<double>(iy) + 0.5) * spec.dy_m;
            const TerrainSample3d sample = source.sample(x_mid, y_mid);
            if (sample.status != QueryStatus3d::ok) {
                throw std::invalid_argument(
                    "sample_to_grid3d: cell midpoint falls outside terrain bounds");
            }
            cell_surfaces[ix * (spec.ny - 1U) + iy] = sample.surface;
        }
    }

    return {spec, std::move(elevations), std::move(cell_surfaces)};
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (span_m, step_m, max_nodes) internal axis helper
std::pair<std::size_t, double> axis_count_and_step(double origin_m, double max_m,
                                                   double requested_step_m,
                                                   std::size_t max_nodes,
                                                   const char* axis_label) {
    if (!std::isfinite(requested_step_m) || !(requested_step_m > 0.)) {
        throw std::invalid_argument(std::string("make_grid3d_spec: positive ") +
                                    axis_label + " step required");
    }
    const double span = max_m - origin_m;
    if (!std::isfinite(span) || !(span > 0.)) {
        throw std::invalid_argument(std::string("make_grid3d_spec: positive ") +
                                    axis_label + " span required");
    }
    const double steps_real = std::nearbyint(span / requested_step_m);
    if (!std::isfinite(steps_real) || steps_real < 3. ||
        steps_real > static_cast<double>(max_nodes - 1U) ||
        std::abs(steps_real * requested_step_m - span) > 1e-9 * std::max(1., span)) {
        throw std::invalid_argument(
            std::string("make_grid3d_spec: ") + axis_label +
            " span must be an exact multiple (>= 3) of step within bounded node limits");
    }
    const auto count = static_cast<std::size_t>(steps_real) + 1U;
    double step = requested_step_m;
    while (origin_m + static_cast<double>(count - 1U) * step > max_m) {
        step = std::nextafter(step, 0.);
    }
    return {count, step};
}

} // namespace

Grid3dSpec make_grid3d_spec(const TerrainBounds3d& bounds, double dx_m, double dy_m) {
    if (!std::isfinite(bounds.x_min_m) || !std::isfinite(bounds.x_max_m) ||
        !std::isfinite(bounds.y_min_m) || !std::isfinite(bounds.y_max_m)) {
        throw std::invalid_argument("make_grid3d_spec: finite horizontal bounds required");
    }
    const auto [nx, dx] = axis_count_and_step(bounds.x_min_m, bounds.x_max_m, dx_m, 500000U, "X");
    const auto [ny, dy] = axis_count_and_step(bounds.y_min_m, bounds.y_max_m, dy_m, 10000U, "Y");
    return Grid3dSpec{
        .nx = nx,
        .ny = ny,
        .x0_m = bounds.x_min_m,
        .y0_m = bounds.y_min_m,
        .dx_m = dx,
        .dy_m = dy,
    };
}

Grid3d sample_to_grid3d(Terrain3dQueryView view, const Grid3dSpec& spec) {
    if (!view.valid()) {
        throw std::invalid_argument("sample_to_grid3d: invalid Terrain3dQueryView");
    }
    return sample_source_to_grid3d(view, spec);
}

Grid3d sample_to_grid3d(const ExtrudedTrack3d& extruded, const Grid3dSpec& spec) {
    return sample_source_to_grid3d(extruded, spec);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (ground_z_m, catch_clearance_m) mirrors HeightFieldSpec::catch_plane_z_m
HeightField3dBuild build_heightfield_3d(const Grid3d& grid, double ground_z_m,
                                        double catch_clearance_m) {
    if (!std::isfinite(ground_z_m)) {
        throw std::invalid_argument("build_heightfield_3d: ground_z_m must be finite");
    }
    if (!std::isfinite(catch_clearance_m) || !(catch_clearance_m > 0.)) {
        throw std::invalid_argument("build_heightfield_3d: catch_clearance_m must be positive");
    }
    const Grid3dSpec& gs = grid.spec();
    const TerrainBounds3d& bounds = grid.bounds();
    if (gs.nx < 2U || gs.ny < 2U ||
        gs.nx > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        gs.ny > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("build_heightfield_3d: grid dimensions exceed native domain");
    }
    if (!std::isfinite(bounds.x_min_m) || !std::isfinite(bounds.x_max_m) ||
        !std::isfinite(bounds.y_min_m) || !std::isfinite(bounds.y_max_m) ||
        !std::isfinite(bounds.z_min_m) || !std::isfinite(bounds.z_max_m)) {
        throw std::invalid_argument("build_heightfield_3d: finite terrain bounds required");
    }

    HeightFieldSpec spec{};
    spec.nrow = static_cast<int>(gs.ny);
    spec.ncol = static_cast<int>(gs.nx);
    spec.radius_x_m = 0.5 * (bounds.x_max_m - bounds.x_min_m);
    spec.radius_y_m = 0.5 * (bounds.y_max_m - bounds.y_min_m);
    spec.base_m = 0.5;

    const double min_datum_bounds = -bounds.z_min_m + 1.0;
    const double min_datum_catch = ground_z_m - bounds.z_min_m + 1.0;
    const double raw_datum = std::max({2.8, min_datum_bounds, min_datum_catch});
    spec.datum_z_m = std::ceil(raw_datum * 10.) / 10.;
    const double raw_elev = std::max(3.4, spec.datum_z_m + bounds.z_max_m + 1.0);
    spec.elevation_m = std::ceil(raw_elev * 10.) / 10.;
    spec.validate();

    const double catch_plane_z_m = spec.catch_plane_z_m(ground_z_m, catch_clearance_m);
    if (!std::isfinite(catch_plane_z_m) || !(catch_plane_z_m < bounds.z_min_m)) {
        throw std::invalid_argument(
            "build_heightfield_3d: catch_plane_z_m is not strictly below bounds.z_min_m");
    }
    if (!spec.fits({bounds.z_min_m, bounds.z_max_m})) {
        throw std::invalid_argument(
            "build_heightfield_3d: terrain elevation extent leaves heightfield envelope");
    }

    return HeightField3dBuild{
        .spec = spec,
        .bounds = bounds,
        .ground_z_m = ground_z_m,
        .catch_plane_z_m = catch_plane_z_m,
    };
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (ground_z_m, catch_clearance_m) mirrors HeightFieldSpec::catch_plane_z_m
HeightField3dBuild build_heightfield_3d(Terrain3dQueryView view, const Grid3dSpec& spec,
                                        double ground_z_m, double catch_clearance_m) {
    const Grid3d grid = sample_to_grid3d(view, spec);
    return build_heightfield_3d(grid, ground_z_m, catch_clearance_m);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (ground_z_m, catch_clearance_m) mirrors HeightFieldSpec::catch_plane_z_m
HeightField3dBuild build_heightfield_3d(const ExtrudedTrack3d& extruded, const Grid3dSpec& spec,
                                        double ground_z_m, double catch_clearance_m) {
    const Grid3d grid = sample_to_grid3d(extruded, spec);
    return build_heightfield_3d(grid, ground_z_m, catch_clearance_m);
}

// NOLINTNEXTLINE(misc-const-correctness) writes through model.hfield_data; non-const mjModel& expresses logical mutation
void populate_heightfield_3d(mjModel& model, const Grid3d& grid,
                             const HeightField3dBuild& hf) {
    hf.spec.validate();
    const Grid3dSpec& gs = grid.spec();
    const TerrainBounds3d& bounds = grid.bounds();
    if (std::cmp_not_equal(gs.nx, hf.spec.ncol) ||
        std::cmp_not_equal(gs.ny, hf.spec.nrow)) {
        throw std::invalid_argument(
            "populate_heightfield_3d: grid dimensions do not match HeightFieldSpec (ncol=nx, nrow=ny)");
    }
    if (!std::isfinite(hf.catch_plane_z_m) || !(hf.catch_plane_z_m < bounds.z_min_m) ||
        !(hf.catch_plane_z_m < hf.bounds.z_min_m)) {
        throw std::invalid_argument(
            "populate_heightfield_3d: catch_plane_z_m must be strictly below terrain z_min_m");
    }
    if (!hf.spec.fits({bounds.z_min_m, bounds.z_max_m})) {
        throw std::invalid_argument(
            "populate_heightfield_3d: grid elevation extent leaves heightfield envelope");
    }

    const int id = mj_name2id(&model, mjOBJ_HFIELD, "road");
    if (id < 0) {
        throw std::invalid_argument("populate_heightfield_3d: model has no 'road' hfield");
    }
    const auto rows = std::views::counted(model.hfield_nrow, model.nhfield);
    const auto columns = std::views::counted(model.hfield_ncol, model.nhfield);
    const auto addresses = std::views::counted(model.hfield_adr, model.nhfield);
    const auto index = static_cast<std::size_t>(id);
    if (rows[index] != hf.spec.nrow || columns[index] != hf.spec.ncol) {
        throw std::invalid_argument(
            "populate_heightfield_3d: compiled road dimensions differ from HeightField3dBuild");
    }

    const std::size_t total = static_cast<std::size_t>(hf.spec.nrow) *
                              static_cast<std::size_t>(hf.spec.ncol);
    const int offset = addresses[index];
    if (offset < 0 || std::cmp_greater(offset, model.nhfielddata) ||
        std::cmp_greater(total, model.nhfielddata - offset)) {
        throw std::invalid_argument(
            "populate_heightfield_3d: compiled road address is outside heightfield storage");
    }

    const auto target = std::views::counted(model.hfield_data, model.nhfielddata)
                            .subspan(static_cast<std::size_t>(offset), total);

    const std::size_t nx = gs.nx;
    const std::size_t ny = gs.ny;
    for (std::size_t iy = 0; iy < ny; ++iy) {
        const std::size_t row_base = iy * nx;
        for (std::size_t ix = 0; ix < nx; ++ix) {
            const double z = grid.node_z(ix, iy);
            const double norm = (z + hf.spec.datum_z_m) / hf.spec.elevation_m;
            if (!std::isfinite(norm) || norm < 0. || norm > 1.) {
                throw std::invalid_argument(
                    "populate_heightfield_3d: normalized road elevation is outside [0, 1]");
            }
            // Transpose Grid3d (ix * ny + iy) -> MuJoCo hfield (row * ncol + col = iy * nx + ix).
            target[row_base + ix] = static_cast<float>(norm);
        }
    }
}

} // namespace bike::terrain
