#include "terrain_fixture.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace spatial_tests {
namespace {

bike::terrain::Grid3d build(std::uint64_t nx, std::uint64_t ny, double dx, double dy,
                            std::vector<double> elevation, std::string name) {
    using namespace bike::terrain;
    Grid3dDescriptor descriptor{.nx = nx, .ny = ny, .x0_m = 0.,
        .y0_m = -.5 * static_cast<double>(ny - 1) * dy, .dx_m = dx, .dy_m = dy, .dtype = 0};
    Grid3dMetadata metadata{.generator_version = "analytic-fixture/v1",
        .resolved_spec = bike::io::Value{std::move(name)}, .seed_decimal = "0",
        .units = "m", .height_datum_m = 0., .provenance = bike::io::Value{std::string("analytic; no simulation capture")}};
    auto grid = Grid3d::create(descriptor, std::move(elevation),
        std::vector<std::uint32_t>(static_cast<std::size_t>((nx - 1) * (ny - 1)), 1),
        {{.id = 1, .name = "analytic", .mu_peak = .8, .mu_slide = .6,
          .slip_stiffness_per_load = 20., .stribeck_speed_mps = .1, .description = "synthetic fixture"}},
        std::move(metadata));
    if (!grid) throw std::invalid_argument(grid.error());
    return std::move(*grid);
}

void dimensions(std::uint64_t nx, std::uint64_t ny, double dx, double dy) {
    if (nx < 4 || ny < 4 || !std::isfinite(dx) || !std::isfinite(dy) || dx <= 0. || dy <= 0. ||
        nx > std::numeric_limits<std::size_t>::max() / sizeof(double) / ny)
        throw std::invalid_argument("analytic terrain fixture: invalid dimensions");
}

} // namespace

bike::terrain::Grid3d plane_grid(double a, double b, double c,
    std::uint64_t nx, std::uint64_t ny, double dx, double dy) {
    dimensions(nx, ny, dx, dy);
    std::vector<double> values(static_cast<std::size_t>(nx * ny));
    const double y0 = -.5 * static_cast<double>(ny - 1) * dy;
    for (std::uint64_t iy = 0; iy < ny; ++iy)
        for (std::uint64_t ix = 0; ix < nx; ++ix)
            values[static_cast<std::size_t>(iy * nx + ix)] =
                a * (static_cast<double>(ix) * dx) + b * (y0 + static_cast<double>(iy) * dy) + c;
    return build(nx, ny, dx, dy, std::move(values), "z=a*x+b*y+c");
}

bike::terrain::Grid3d polynomial_grid(std::span<const double> c,
    std::uint64_t nx, std::uint64_t ny, double dx, double dy) {
    dimensions(nx, ny, dx, dy);
    if (c.size() != 10) throw std::invalid_argument("polynomial terrain fixture: expected 10 coefficients");
    std::vector<double> values(static_cast<std::size_t>(nx * ny));
    const double y0 = -.5 * static_cast<double>(ny - 1) * dy;
    for (std::uint64_t iy = 0; iy < ny; ++iy) {
        const double y = y0 + static_cast<double>(iy) * dy;
        for (std::uint64_t ix = 0; ix < nx; ++ix) {
            const double x = static_cast<double>(ix) * dx;
            values[static_cast<std::size_t>(iy * nx + ix)] = c[0] + c[1] * x + c[2] * y +
                c[3] * x * x + c[4] * x * y + c[5] * y * y + c[6] * x * x * x +
                c[7] * x * x * y + c[8] * x * y * y + c[9] * y * y * y;
        }
    }
    return build(nx, ny, dx, dy, std::move(values), "analytic cubic polynomial");
}

bike::terrain::Grid3d asymmetric_4x4() {
    using namespace bike::terrain;
    std::vector<double> elevation(16);
    for (std::size_t iy = 0; iy < 4; ++iy)
        for (std::size_t ix = 0; ix < 4; ++ix)
            elevation[iy * 4 + ix] = .1 * static_cast<double>(ix) + .37 * static_cast<double>(iy) +
                .01 * static_cast<double>(ix * iy);
    std::vector<std::uint32_t> material(9);
    for (std::size_t i = 0; i < material.size(); ++i) material[i] = i % 2 == 0 ? 1 : 7;
    auto grid = Grid3d::create({.nx = 4, .ny = 4, .x0_m = 0., .y0_m = 0., .dx_m = 1., .dy_m = 1., .dtype = 0},
        std::move(elevation), std::move(material),
        {{.id = 1, .name = "hard", .mu_peak = .9, .mu_slide = .7, .slip_stiffness_per_load = 20., .stribeck_speed_mps = .1, .description = "analytic"},
         {.id = 7, .name = "soft", .mu_peak = .6, .mu_slide = .4, .slip_stiffness_per_load = 10., .stribeck_speed_mps = .2, .description = {}}},
        {.generator_version = "asymmetric-4x4/v1", .resolved_spec = bike::io::Value{std::string("0.1*ix+0.37*iy+0.01*ix*iy")},
         .seed_decimal = "0", .units = "m", .height_datum_m = 0., .provenance = bike::io::Value{std::string("analytic")}});
    if (!grid) throw std::invalid_argument(grid.error());
    return std::move(*grid);
}

bike::terrain::Grid3d bump_grid(double height, double sigma,
    std::uint64_t nx, std::uint64_t ny, double dx, double dy) {
    dimensions(nx, ny, dx, dy);
    if (!std::isfinite(height) || !std::isfinite(sigma) || sigma <= 0.)
        throw std::invalid_argument("bump terrain fixture: invalid height/sigma");
    std::vector<double> values(static_cast<std::size_t>(nx * ny));
    const double cx = .5 * static_cast<double>(nx - 1) * dx;
    const double y0 = -.5 * static_cast<double>(ny - 1) * dy;
    for (std::uint64_t iy = 0; iy < ny; ++iy) {
        const double y = y0 + static_cast<double>(iy) * dy;
        for (std::uint64_t ix = 0; ix < nx; ++ix) {
            const double x = static_cast<double>(ix) * dx - cx;
            values[static_cast<std::size_t>(iy * nx + ix)] = height * std::exp(-.5 * (x * x + y * y) / (sigma * sigma));
        }
    }
    return build(nx, ny, dx, dy, std::move(values), "Gaussian bump on z=0 plane");
}

std::vector<double> polynomial_ray_roots_highprecision(const bike::terrain::Poly6& polynomial) {
    // Independent Durand-Kerner calculation in long double; never calls the
    // production Bernstein isolator or uses a captured/generated expected hit.
    using Complex = std::complex<long double>;
    unsigned degree = 6;
    while (degree != 0 && polynomial.a[degree] == 0.) --degree;
    if (degree == 0) return polynomial.a[0] == 0. ? std::vector<double>{0.} : std::vector<double>{};
    std::array<long double, 7> a{};
    long double radius = 1.;
    for (unsigned i = 0; i <= degree; ++i) {
        a[i] = static_cast<long double>(polynomial.a[i]) / polynomial.a[degree];
        if (i < degree) radius = std::max(radius, 1. + std::abs(a[i]));
    }
    std::array<Complex, 6> roots{};
    for (unsigned i = 0; i < degree; ++i) {
        const long double angle = 2. * std::numbers::pi_v<long double> * (static_cast<long double>(i) + .5L) / degree;
        roots[i] = std::polar(radius, angle);
    }
    for (unsigned iteration = 0; iteration < 200; ++iteration) {
        const auto previous = roots;
        for (unsigned i = 0; i < degree; ++i) {
            Complex value = a[degree], denominator = 1.;
            for (unsigned j = degree; j != 0; --j) value = value * previous[i] + a[j - 1];
            for (unsigned j = 0; j < degree; ++j) if (j != i) denominator *= previous[i] - previous[j];
            if (std::abs(denominator) == 0.)
                roots[i] += Complex{std::numeric_limits<long double>::epsilon() * (i + 1),
                                    std::numeric_limits<long double>::epsilon()};
            else roots[i] = previous[i] - value / denominator;
        }
    }
    std::vector<double> real_roots;
    for (unsigned i = 0; i < degree; ++i)
        if (std::abs(roots[i].imag()) < 1e-14L && roots[i].real() >= 0. && roots[i].real() <= 1.)
            real_roots.push_back(static_cast<double>(roots[i].real()));
    std::sort(real_roots.begin(), real_roots.end());
    real_roots.erase(std::unique(real_roots.begin(), real_roots.end(),
        [](double a0, double b0) { return std::abs(a0 - b0) < 1e-13; }), real_roots.end());
    return real_roots;
}

} // namespace spatial_tests
