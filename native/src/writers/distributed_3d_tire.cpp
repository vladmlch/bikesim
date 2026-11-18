// writers/distributed_3d_tire.cpp — 128-station Lagrangian toroidal 3D
// distributed tire writer (`distributed_3d` backend). Implements S5 spec §3
// verbatim with zero heap allocations on `try_compute_into` and zero link
// dependencies on `bike_terrain`.
#include "writers/distributed_3d_tire.hpp"

#include "cblas_abi.hpp"
#include "model_access.hpp"
#include "validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using Vec3 = std::array<double, model_access::kXYZ>;

constexpr std::array<const char *, 9> kLeaves = {
    "active", "ux", "uy", "tx.0", "tx.1", "tx.2", "ty.0", "ty.1", "ty.2",
};
constexpr std::array<const char *, 2> kSides = {"front", "rear"};
constexpr double kCos82Deg = 0.13917310096006544; // std::cos(82 * pi / 180)
constexpr double kStencilMarginM = 0.015;
constexpr double kSearchMarginM = 0.005;
constexpr std::string_view kBackendName = "distributed_3d";
constexpr std::string_view kCapabilityStatus =
    "experimental_reference_not_release_qualified";
constexpr std::string_view kCalibrationStatus = "calibrated";

struct SurfaceEntry {
    std::string_view name;
    double mu_peak;
    double mu_slide;
    double c_kappa_per_fz;
    double stribeck_speed_mps;
};

// Embedded 5-entry surface registry (S5 spec §3.1 / terrain/surface.cpp)
// so `bike_core` needs zero external `bike_terrain` link symbols.
constexpr SurfaceEntry surface_entry(rider::SurfaceLabel label) noexcept {
    switch (label) {
    case rider::SurfaceLabel::asphalt:
        return {.name = "asphalt",
                .mu_peak = 1.05,
                .mu_slide = 0.75,
                .c_kappa_per_fz = 15.0,
                .stribeck_speed_mps = 4.5};
    case rider::SurfaceLabel::hardpack:
        return {.name = "hardpack",
                .mu_peak = 0.80,
                .mu_slide = 0.60,
                .c_kappa_per_fz = 12.0,
                .stribeck_speed_mps = 4.5};
    case rider::SurfaceLabel::loose:
        return {.name = "loose",
                .mu_peak = 0.55,
                .mu_slide = 0.45,
                .c_kappa_per_fz = 7.0,
                .stribeck_speed_mps = 4.5};
    case rider::SurfaceLabel::wet:
        return {.name = "wet",
                .mu_peak = 0.50,
                .mu_slide = 0.40,
                .c_kappa_per_fz = 10.0,
                .stribeck_speed_mps = 4.5};
    case rider::SurfaceLabel::scree:
        return {.name = "scree",
                .mu_peak = 0.45,
                .mu_slide = 0.38,
                .c_kappa_per_fz = 4.5,
                .stribeck_speed_mps = 4.5};
    }
    return {.name = "hardpack",
            .mu_peak = 0.80,
            .mu_slide = 0.60,
            .c_kappa_per_fz = 12.0,
            .stribeck_speed_mps = 4.5};
}

double dot3(const Vec3 &a, const Vec3 &b) noexcept {
    return blas::ddot(model_access::kXYZ, a.data(), 1, b.data(), 1);
}

double norm3(const Vec3 &v) noexcept {
    return std::sqrt(dot3(v, v));
}

Vec3 sub3(const Vec3 &a, const Vec3 &b) noexcept {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 cross3(const Vec3 &a, const Vec3 &b) noexcept {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

template <typename T>
[[nodiscard]] bool try_readonly_buffer(const T *data, mjtSize count,
                                       std::span<const T> &out) noexcept {
    if (count < 0 || (data == nullptr && count > 0)) {
        return false;
    }
    out = std::views::counted(data, static_cast<std::ptrdiff_t>(count));
    return true;
}

[[nodiscard]] bool try_normalize_unit(const Vec3 &raw, Vec3 &out) noexcept {
    for (const double v : raw) {
        if (!std::isfinite(v)) return false;
    }
    const double len = norm3(raw);
    if (!(len > 1e-12)) return false;
    out = {raw[0] / len, raw[1] / len, raw[2] / len};
    return true;
}

void normalize_unit_axis(const Vec3 &raw, Vec3 &out, const char *label) {
    if (!try_normalize_unit(raw, out)) {
        throw std::invalid_argument(std::string(label) +
                                    " must be finite and nonzero");
    }
}

[[nodiscard]] bool try_build_road_basis(const Vec3 &a_hat, const Vec3 &n_hat,
                                        Vec3 &tx, Vec3 &ty) noexcept {
    const Vec3 tx_raw = cross3(a_hat, n_hat);
    const double tx_len = norm3(tx_raw);
    if (tx_len > 1e-8) {
        tx = {tx_raw[0] / tx_len, tx_raw[1] / tx_len, tx_raw[2] / tx_len};
    } else {
        const Vec3 ref = (std::abs(n_hat[0]) < 0.9) ? Vec3{1.0, 0.0, 0.0}
                                                    : Vec3{0.0, 1.0, 0.0};
        const double proj = dot3(ref, n_hat);
        const Vec3 ortho = {ref[0] - proj * n_hat[0],
                            ref[1] - proj * n_hat[1],
                            ref[2] - proj * n_hat[2]};
        if (!try_normalize_unit(ortho, tx)) return false;
    }
    return try_normalize_unit(cross3(n_hat, tx), ty);
}

[[nodiscard]] bool try_interval_id(double time, double dt,
                                   std::int64_t &out) noexcept {
    if (!std::isfinite(time) || time < 0.0 || !std::isfinite(dt) ||
        dt <= 0.0) {
        return false;
    }
    const double quotient = time / dt;
    if (!std::isfinite(quotient) || quotient >= 0x1p63) {
        return false;
    }
    double base = std::floor(quotient);
    const double fraction = quotient - base;
    if (fraction > 0.5 || (fraction == 0.5 && std::fmod(base, 2.0) != 0.0)) {
        base += 1.0;
    }
    if (base >= 0x1p63) {
        return false;
    }
    out = static_cast<std::int64_t>(base);
    return true;
}

struct NeumaierSum {
    void add(double x) noexcept {
        const double t = f_result + x;
        if (std::isfinite(t)) {
            if (std::abs(f_result) >= std::abs(x)) {
                c_sum += (f_result - t) + x;
            } else {
                c_sum += (x - t) + f_result;
            }
        }
        f_result = t;
    }

    [[nodiscard]] double total() const noexcept { return f_result + c_sum; }

    double f_result = 0.0;
    double c_sum = 0.0;
};

struct RampInput {
    double x = 0.0;
    double x0 = 1.0;
};

[[nodiscard]] double smooth_ramp(RampInput arg) noexcept {
    if (!(arg.x > 0.0)) return 0.0;
    return (arg.x * arg.x) / (arg.x + arg.x0);
}

[[nodiscard]] double smooth_ramp_integral(RampInput arg) noexcept {
    if (!(arg.x > 0.0)) return 0.0;
    const double u = arg.x / arg.x0;
    if (u < 1e-3) {
        const double u2 = u * u;
        const double poly =
            (1.0 / 3.0) -
            u * (1.0 / 4.0) +
            u2 * ((1.0 / 5.0) - u * (1.0 / 6.0) +
                  u2 * ((1.0 / 7.0) - u * (1.0 / 8.0)));
        return arg.x0 * arg.x0 * (u * u2) * poly;
    }
    return 0.5 * arg.x * arg.x - arg.x0 * arg.x +
           arg.x0 * arg.x0 * std::log1p(u);
}

[[nodiscard]] double station_elastic_force(const Distributed3dWheelParams &p,
                                           double weight,
                                           double delta) noexcept {
    if (!(delta > 0.0)) return 0.0;
    const double xi_rim = delta - p.rim_strike_deflection_m();
    return weight *
           (p.kappa_p_n_m_rad * smooth_ramp({.x = delta, .x0 = p.delta_0_m}) +
            p.kappa_c_n_m_rad *
                smooth_ramp({.x = delta, .x0 = p.delta_carcass_m}) +
            p.kappa_rim_n_m_rad *
                smooth_ramp({.x = xi_rim, .x0 = p.delta_rim_0_m}));
}

[[nodiscard]] double station_radial_energy(const Distributed3dWheelParams &p,
                                           double weight,
                                           double delta) noexcept {
    if (!(delta > 0.0)) return 0.0;
    const double xi_rim = delta - p.rim_strike_deflection_m();
    return weight *
           (p.kappa_p_n_m_rad *
                smooth_ramp_integral({.x = delta, .x0 = p.delta_0_m}) +
            p.kappa_c_n_m_rad *
                smooth_ramp_integral({.x = delta, .x0 = p.delta_carcass_m}) +
            p.kappa_rim_n_m_rad *
                smooth_ramp_integral({.x = xi_rim, .x0 = p.delta_rim_0_m}));
}

void validate_wheel_params(const Distributed3dWheelParams &p,
                           std::string_view side) {
    const std::string prefix = "distributed_3d." + std::string(side);
    if (p.station_count < 32 || p.station_count > 512) {
        throw std::invalid_argument(
            prefix + ".station_count must lie in [32, 512]");
    }
    validation::positive(p.pressure_bar, prefix + ".pressure_bar");
    validation::positive(p.outer_radius_m, prefix + ".outer_radius_m");
    validation::positive(p.rim_radius_m, prefix + ".rim_radius_m");
    validation::nonnegative(p.crown_radius_m, prefix + ".crown_radius_m");
    validation::nonnegative(p.compressed_casing_m,
                            prefix + ".compressed_casing_m");
    if (!(p.rim_radius_m < p.outer_radius_m)) {
        throw std::invalid_argument(prefix +
                                    ".rim_radius_m must be < outer_radius_m");
    }
    if (!(p.major_radius_m() > 0.0)) {
        throw std::invalid_argument(prefix +
                                    ".crown_radius_m must be < outer_radius_m");
    }
    if (!(p.rim_strike_deflection_m() > 0.0)) {
        throw std::invalid_argument(
            prefix + ".rim_strike_deflection_m must be positive");
    }
    validation::positive(p.kappa_p_n_m_rad, prefix + ".kappa_p_n_m_rad");
    validation::positive(p.delta_0_m, prefix + ".delta_0_m");
    validation::nonnegative(p.kappa_c_n_m_rad, prefix + ".kappa_c_n_m_rad");
    validation::positive(p.delta_carcass_m, prefix + ".delta_carcass_m");
    validation::nonnegative(p.kappa_rim_n_m_rad, prefix + ".kappa_rim_n_m_rad");
    validation::positive(p.delta_rim_0_m, prefix + ".delta_rim_0_m");
    validation::nonnegative(p.loss_factor, prefix + ".loss_factor");
    if (!(p.loss_factor < 1.0)) {
        throw std::invalid_argument(prefix + ".loss_factor must lie in [0, 1)");
    }
    validation::positive(p.hysteresis_rate_eps_mps,
                         prefix + ".hysteresis_rate_eps_mps");
    validation::positive(p.kappa_brush_x_n_m_rad,
                         prefix + ".kappa_brush_x_n_m_rad");
    validation::positive(p.kappa_brush_y_n_m_rad,
                         prefix + ".kappa_brush_y_n_m_rad");
    validation::positive(p.c_kappa_ref_per_fz, prefix + ".c_kappa_ref_per_fz");
    validation::nonnegative(p.mu_cap, prefix + ".mu_cap");
    validation::nonnegative(p.valid_load_range_n[0],
                            prefix + ".valid_load_range_n.0");
    validation::positive(p.valid_load_range_n[1],
                         prefix + ".valid_load_range_n.1");
    if (!(p.valid_load_range_n[0] < p.valid_load_range_n[1])) {
        throw std::invalid_argument(prefix +
                                    ".valid_load_range_n must be ordered");
    }
    validation::nonnegative(p.valid_deflection_range_m[0],
                            prefix + ".valid_deflection_range_m.0");
    validation::positive(p.valid_deflection_range_m[1],
                         prefix + ".valid_deflection_range_m.1");
    if (!(p.valid_deflection_range_m[0] < p.valid_deflection_range_m[1])) {
        throw std::invalid_argument(
            prefix + ".valid_deflection_range_m must be ordered");
    }
}

struct Stage1Gate {
    bool enabled = false;
    Vec3 e1{};
    double min_dot = 0.0;
};

[[nodiscard]] Stage1Gate make_stage1_gate(const Vec3 &a_hat,
                                          double r_major) noexcept {
    const Vec3 d_perp = {
        a_hat[2] * a_hat[0],
        a_hat[2] * a_hat[1],
        a_hat[2] * a_hat[2] - 1.0,
    };
    const double d_perp_len = norm3(d_perp);
    if (!(d_perp_len > 1e-4)) {
        return {.enabled = false};
    }
    return {
        .enabled = true,
        .e1 = {d_perp[0] / d_perp_len, d_perp[1] / d_perp_len,
               d_perp[2] / d_perp_len},
        .min_dot = r_major * kCos82Deg,
    };
}

enum class StationCullStatus : std::uint8_t {
    culled,
    evaluated,
    outside,
    invalid_input,
};

struct StationQuery {
    StationCullStatus status = StationCullStatus::culled;
    double delta = 0.0;
    bike::terrain::TorusClosestPoint3d hit{};
};

// Exact 3-stage resolution-independent airborne culling (S5 spec §3.3).
// Shared verbatim by `accumulate_core` and `stored_energy`.
[[nodiscard]] StationQuery query_station_3stage(
    const Compliant3dTerrainCallbacks &terrain, const Stage1Gate &gate,
    const Vec3 &p_hub, const Vec3 &c_i, double r_crown) noexcept {
    if (gate.enabled) {
        const Vec3 rel = sub3(c_i, p_hub);
        if (dot3(rel, gate.e1) < gate.min_dot) {
            return {.status = StationCullStatus::culled};
        }
    }

    const std::array<std::array<double, 2>, 5> stencil{{
        {c_i[0], c_i[1]},
        {c_i[0] - r_crown, c_i[1]},
        {c_i[0] + r_crown, c_i[1]},
        {c_i[0], c_i[1] - r_crown},
        {c_i[0], c_i[1] + r_crown},
    }};
    bool all_ok = true;
    double h_max = -std::numeric_limits<double>::infinity();
    for (const auto &pt : stencil) {
        const bike::terrain::TerrainSample3d smp =
            terrain.sample_fn(terrain.ctx, pt[0], pt[1]);
        if (smp.status == bike::terrain::QueryStatus3d::invalid_input) {
            return {.status = StationCullStatus::invalid_input};
        }
        if (smp.status != bike::terrain::QueryStatus3d::ok) {
            all_ok = false;
        } else if (std::isfinite(smp.height_m)) {
            h_max = std::max(h_max, smp.height_m);
        } else {
            return {.status = StationCullStatus::invalid_input};
        }
    }
    if (all_ok && c_i[2] > h_max + r_crown + kStencilMarginM) {
        return {.status = StationCullStatus::culled};
    }

    const double r_search = r_crown + kSearchMarginM;
    const bike::terrain::TorusClosestPoint3d hit =
        terrain.closest_point_fn(terrain.ctx, c_i, r_search);
    if (hit.status == bike::terrain::QueryStatus3d::invalid_input) {
        return {.status = StationCullStatus::invalid_input};
    }
    if (hit.status == bike::terrain::QueryStatus3d::outside) {
        return {.status = StationCullStatus::outside};
    }
    if (!std::isfinite(hit.signed_distance_m)) {
        return {.status = StationCullStatus::invalid_input};
    }
    return {
        .status = StationCullStatus::evaluated,
        .delta = r_crown - hit.signed_distance_m,
        .hit = hit,
    };
}

void lift_wheel_stations(const Vec3 &p_hub,
                         std::span<const mjtNum> xmat_wheel,
                         const std::vector<std::array<double, 3>> &local,
                         std::vector<double> &points_out) noexcept {
    const std::size_t n = local.size();
    blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                blas::Transpose::yes, static_cast<int>(n),
                model_access::kXYZ, model_access::kXYZ, 1.0, &local[0][0],
                model_access::kXYZ, xmat_wheel.data(), model_access::kXYZ, 0.0,
                points_out.data(), model_access::kXYZ);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t off = static_cast<std::size_t>(model_access::kXYZ) * i;
        for (std::size_t k = 0; k < static_cast<std::size_t>(model_access::kXYZ);
             ++k) {
            points_out[off + k] += p_hub[k];
        }
    }
}

struct PatchValues {
    double normal_load_n = 0.0;
    double tangent_force_n = 0.0;
    double slip_mps = 0.0;
};

[[nodiscard]] bool try_fill_patch(const Vec3 &point, const Vec3 &normal,
                                  PatchValues vals, const Vec3 &world_force,
                                  TirePatch &out) noexcept {
    for (const double v : point) {
        if (!std::isfinite(v)) return false;
    }
    for (const double v : normal) {
        if (!std::isfinite(v)) return false;
    }
    if (std::abs(norm3(normal) - 1.0) > 1e-6) return false;
    if (!(std::isfinite(vals.normal_load_n) &&
          std::isfinite(vals.tangent_force_n) &&
          std::isfinite(vals.slip_mps)) ||
        vals.normal_load_n < 0.0) {
        return false;
    }
    for (const double v : world_force) {
        if (!std::isfinite(v)) return false;
    }
    out.point_m = point;
    out.normal = normal;
    out.normal_load_n = vals.normal_load_n;
    out.tangent_force_n = vals.tangent_force_n;
    out.slip_mps = vals.slip_mps;
    out.working_surface = true;
    out.native_world_force_n = world_force;
    out.couple_world_nm = {0.0, 0.0, 0.0};
    return true;
}

double effective_radius(const std::vector<TirePatch> &patches,
                        const Vec3 &axis) noexcept {
    if (patches.empty()) return 0.0;
    const bool any_loaded = std::ranges::any_of(
        patches, [](const TirePatch &p) { return p.normal_load_n > 0.0; });
    NeumaierSum terms;
    NeumaierSum weights;
    for (const TirePatch &p : patches) {
        const double w = any_loaded ? p.normal_load_n : 1.0;
        const double drop = -dot3(sub3(p.point_m, axis), p.normal);
        terms.add(w * drop);
        weights.add(w);
    }
    return std::max(0.0, terms.total() / weights.total());
}

} // namespace

Distributed3dTireConfig make_distributed_3d_tire_config(
    bike::spatial::SpatialTirePreset preset,
    Compliant3dTerrainCallbacks terrain) {
    switch (preset) {
    case bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar: {
        const Distributed3dWheelParams front{
            .station_count = 128,
            .pressure_bar = 1.20,
            .outer_radius_m = 0.3790,
            .rim_radius_m = 0.3200,
            .crown_radius_m = 0.0355,
            .compressed_casing_m = 0.0060,
            .kappa_p_n_m_rad = 201000.0,
            .delta_0_m = 0.00050,
            .kappa_c_n_m_rad = 15000.0,
            .delta_carcass_m = 0.0200,
            .kappa_rim_n_m_rad = 1400000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0550,
            .hysteresis_rate_eps_mps = 0.0100,
            .kappa_brush_x_n_m_rad = 200000.0,
            .kappa_brush_y_n_m_rad = 185300.0,
            .c_kappa_ref_per_fz = 15.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3500.0},
            .valid_deflection_range_m = {0.0, 0.060},
        };
        const Distributed3dWheelParams rear{
            .station_count = 128,
            .pressure_bar = 1.20,
            .outer_radius_m = 0.3600,
            .rim_radius_m = 0.3000,
            .crown_radius_m = 0.0355,
            .compressed_casing_m = 0.0060,
            .kappa_p_n_m_rad = 195600.0,
            .delta_0_m = 0.00050,
            .kappa_c_n_m_rad = 15000.0,
            .delta_carcass_m = 0.0200,
            .kappa_rim_n_m_rad = 1400000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0550,
            .hysteresis_rate_eps_mps = 0.0100,
            .kappa_brush_x_n_m_rad = 169000.0,
            .kappa_brush_y_n_m_rad = 156600.0,
            .c_kappa_ref_per_fz = 15.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3500.0},
            .valid_deflection_range_m = {0.0, 0.060},
        };
        return Distributed3dTireConfig{
            .front = front,
            .rear = rear,
            .surface_mode = "track",
            .terrain = terrain,
            .distinct_normal_deg = 15.0,
            .configured_mu = std::nullopt,
            .configured_c_kappa_per_fz = 12.0,
        };
    }
    case bike::spatial::SpatialTirePreset::mtb_2p4_standard: {
        const Distributed3dWheelParams front{
            .station_count = 128,
            .pressure_bar = 1.50,
            .outer_radius_m = 0.3720,
            .rim_radius_m = 0.3200,
            .crown_radius_m = 0.0300,
            .compressed_casing_m = 0.0060,
            .kappa_p_n_m_rad = 197600.0,
            .delta_0_m = 0.00050,
            .kappa_c_n_m_rad = 8000.0,
            .delta_carcass_m = 0.0180,
            .kappa_rim_n_m_rad = 1400000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0662,
            .hysteresis_rate_eps_mps = 0.0100,
            .kappa_brush_x_n_m_rad = 198000.0,
            .kappa_brush_y_n_m_rad = 153700.0,
            .c_kappa_ref_per_fz = 15.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3000.0},
            .valid_deflection_range_m = {0.0, 0.060},
        };
        const Distributed3dWheelParams rear{
            .station_count = 128,
            .pressure_bar = 1.70,
            .outer_radius_m = 0.3520,
            .rim_radius_m = 0.3000,
            .crown_radius_m = 0.0300,
            .compressed_casing_m = 0.0060,
            .kappa_p_n_m_rad = 212200.0,
            .delta_0_m = 0.00050,
            .kappa_c_n_m_rad = 4000.0,
            .delta_carcass_m = 0.0180,
            .kappa_rim_n_m_rad = 1400000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0690,
            .hysteresis_rate_eps_mps = 0.0100,
            .kappa_brush_x_n_m_rad = 199700.0,
            .kappa_brush_y_n_m_rad = 143500.0,
            .c_kappa_ref_per_fz = 15.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3000.0},
            .valid_deflection_range_m = {0.0, 0.060},
        };
        return Distributed3dTireConfig{
            .front = front,
            .rear = rear,
            .surface_mode = "track",
            .terrain = terrain,
            .distinct_normal_deg = 15.0,
            .configured_mu = std::nullopt,
            .configured_c_kappa_per_fz = 12.0,
        };
    }
    }
    throw std::invalid_argument("unknown SpatialTirePreset");
}

Distributed3dTireWriter::Distributed3dTireWriter(const mjModel *m,
                                                 const mjData * /*d*/,
                                                 Distributed3dTireConfig config)
    : m_(m), cfg_(std::move(config)) {
    if (m_ == nullptr) {
        throw std::invalid_argument(
            "distributed_3d tire writer needs a live mjModel");
    }
    if (m_->nv < 0 || m_->nv > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(
            "distributed_3d tire writer needs a model width in int range");
    }
    nv_ = static_cast<int>(m_->nv);

    if (cfg_.surface_mode != "track" && cfg_.surface_mode != "configured") {
        throw std::invalid_argument(
            "distributed_3d surface_mode must be 'track' or 'configured'");
    }
    if (!cfg_.terrain.valid()) {
        throw std::invalid_argument(
            "distributed_3d tire writer requires valid Terrain3d callbacks");
    }
    validation::positive(cfg_.distinct_normal_deg,
                         "distributed_3d.distinct_normal_deg");
    if (!(cfg_.distinct_normal_deg < 180.0)) {
        throw std::invalid_argument(
            "distributed_3d.distinct_normal_deg must be < 180");
    }
    if (cfg_.configured_mu.has_value()) {
        validation::nonnegative(*cfg_.configured_mu,
                                "distributed_3d.configured_mu");
    }
    validation::positive(cfg_.configured_c_kappa_per_fz,
                         "distributed_3d.configured_c_kappa_per_fz");
    validate_wheel_params(cfg_.front, "front");
    validate_wheel_params(cfg_.rear, "rear");

    geoms_ = {
        model_access::resolve_id(m_, mjOBJ_GEOM, "geom_front_contact"),
        model_access::resolve_id(m_, mjOBJ_GEOM, "geom_rear_contact"),
    };
    const std::span<const int> body_of =
        model_access::readonly_buffer(m_->geom_bodyid, m_->ngeom);
    for (std::size_t i = 0; i < 2; ++i) {
        model_access::require_id(geoms_[i], m_->ngeom, "tire contact geom");
        bodies_[i] = body_of[static_cast<std::size_t>(geoms_[i])];
        if (bodies_[i] <= 0 || bodies_[i] >= m_->nbody) {
            throw std::invalid_argument(
                "tire contact geom must belong to a physical wheel body");
        }
    }
    radii_ = {cfg_.front.outer_radius_m, cfg_.rear.outer_radius_m};

    const std::span<const int> contype =
        model_access::readonly_buffer(m_->geom_contype, m_->ngeom);
    const std::span<const int> conaffinity =
        model_access::readonly_buffer(m_->geom_conaffinity, m_->ngeom);
    for (const int body : bodies_) {
        for (int g = 0; g < m_->ngeom; ++g) {
            const auto gu = static_cast<std::size_t>(g);
            if (body_of[gu] == body && (contype[gu] || conaffinity[gu])) {
                throw std::invalid_argument(
                    "native wheel collisions must be disabled for "
                    "distributed_3d");
            }
        }
    }

    std::size_t max_n = 0;
    std::size_t total_stations = 0;
    for (std::size_t s = 0; s < 2; ++s) {
        const Distributed3dWheelParams &p = (s == 0) ? cfg_.front : cfg_.rear;
        stations_[s] = biketyre::station_angles(p.station_count);
        const std::size_t n = stations_[s].angles.size();
        max_n = std::max(max_n, n);
        total_stations += n;
        const double r_major = p.major_radius_m();
        local_[s].resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double th = stations_[s].angles[i];
            local_[s][i] = {r_major * std::cos(th), 0.0,
                            r_major * std::sin(th)};
        }
        states_[s].resize(n);
        states_stage_[s].resize(n);
        prepared_states_[s].resize(n);
    }

    const std::size_t cols = total_stations * kLeaves.size();
    wire_cols_.resize(cols);
    wire_seen_.resize(cols);
    state_names_.reserve(cols);
    canonical_.reserve(cols);
    for (std::size_t s = 0; s < 2; ++s) {
        const std::size_t n = stations_[s].angles.size();
        for (std::size_t i = 0; i < n; ++i) {
            for (const char *leaf : kLeaves) {
                std::string name =
                    std::format("{}.s{:03d}.{}", kSides[s], i, leaf);
                canonical_.emplace(name, state_names_.size());
                state_names_.push_back(std::move(name));
            }
        }
    }

    const auto nv_u = static_cast<std::size_t>(nv_);
    const auto xyz_u = static_cast<std::size_t>(model_access::kXYZ);
    points_.resize(max_n * xyz_u);
    active_ids_.resize(max_n);
    normals_.resize(max_n * xyz_u);
    jac_center_.resize(xyz_u * nv_u);
    jr_.resize(xyz_u * nv_u);
    jac_station_.resize(xyz_u * nv_u);
    tmp_.resize(nv_u);
    qfrc_out_.resize(nv_u);
    qfrc_dissipative_.resize(nv_u);
    qfrc_diss_stage_.resize(nv_u);

    constexpr std::size_t kStringCap = 64;
    for (std::size_t s = 0; s < 2; ++s) {
        const std::size_t n = stations_[s].angles.size();
        snapshot_stage_[s].patches.reserve(n);
        snapshot_stage_[s].backend.reserve(kStringCap);
        snapshots_[s].patches.reserve(n);
        snapshots_[s].backend.reserve(kStringCap);
        probe_snapshots_[s].patches.reserve(n);
        probe_snapshots_[s].backend.reserve(kStringCap);
        diagnostics_stage_[s].surface.reserve(kStringCap);
        diagnostics_stage_[s].calibration_status.reserve(kStringCap);
        diagnostics_stage_[s].capability_status.reserve(kStringCap);
        diagnostics_[s].surface.reserve(kStringCap);
        diagnostics_[s].calibration_status.reserve(kStringCap);
        diagnostics_[s].capability_status.reserve(kStringCap);
    }
}

void Distributed3dTireWriter::reset() {
    for (auto &bank : states_) {
        std::ranges::fill(bank, StationState3d{});
    }
    for (auto &bank : prepared_states_) {
        std::ranges::fill(bank, StationState3d{});
    }
    static const TireSnapshot empty_snapshot{};
    static const TireDiagnostics empty_diagnostics{};
    for (TireSnapshot &snap : snapshots_) snap = empty_snapshot;
    for (TireDiagnostics &diag : diagnostics_) diag = empty_diagnostics;
    committed_ = false;
    elastic_energy_j_ = 0.0;
    brush_loss_step_j_ = 0.0;
    radial_dissipation_power_w_ = 0.0;
    dissipative_power_start_w_ = 0.0;
    std::ranges::fill(qfrc_dissipative_, 0.0);
    last_time_s_.reset();
}

void Distributed3dTireWriter::set_state(std::span<const std::string> names,
                                        std::span<const double> row) {
    prepare_state(names, row);
    commit_prepared_state();
}

void Distributed3dTireWriter::prepare_state(std::span<const std::string> names,
                                            std::span<const double> row) {
    std::ranges::fill(wire_seen_, static_cast<char>(0));
    for (const std::string &name : names) {
        const auto it = canonical_.find(name);
        if (it == canonical_.end()) {
            throw std::invalid_argument(
                "set_tire_state: unknown column '" + name + "'");
        }
        if (wire_seen_[it->second] != 0) {
            throw std::invalid_argument(
                "set_tire_state: duplicate column '" + name + "'");
        }
        wire_seen_[it->second] = 1;
    }
    if (names.size() != row.size()) {
        throw std::invalid_argument(
            "set_tire_state: names and row must have equal length");
    }
    for (std::size_t c = 0; c < state_names_.size(); ++c) {
        if (wire_seen_[c] == 0) {
            throw std::invalid_argument(
                "set_tire_state: missing tire-state column '" +
                state_names_[c] + "'");
        }
    }
    for (std::size_t i = 0; i < names.size(); ++i) {
        wire_cols_[canonical_.find(names[i])->second] = row[i];
    }

    std::size_t base = 0;
    for (std::size_t s = 0; s < 2; ++s) {
        const std::size_t n = stations_[s].angles.size();
        for (std::size_t i = 0; i < n; ++i) {
            const double act = wire_cols_[base + 0];
            const double ux = wire_cols_[base + 1];
            const double uy = wire_cols_[base + 2];
            const Vec3 tx = {wire_cols_[base + 3], wire_cols_[base + 4],
                             wire_cols_[base + 5]};
            const Vec3 ty = {wire_cols_[base + 6], wire_cols_[base + 7],
                             wire_cols_[base + 8]};
            const bool all_nan =
                std::isnan(ux) && std::isnan(uy) &&
                std::ranges::all_of(tx, [](double v) { return std::isnan(v); }) &&
                std::ranges::all_of(ty, [](double v) { return std::isnan(v); });
            const bool all_finite =
                std::isfinite(ux) && std::isfinite(uy) &&
                std::ranges::all_of(
                    tx, [](double v) { return std::isfinite(v); }) &&
                std::ranges::all_of(
                    ty, [](double v) { return std::isfinite(v); });

            if ((act == 0.0 || std::isnan(act)) && all_nan) {
                prepared_states_[s][i] = StationState3d{};
            } else if (act == 1.0 && all_finite) {
                if (!(norm3(tx) > 1e-6) || !(norm3(ty) > 1e-6)) {
                    throw std::invalid_argument(
                        "set_tire_state: active tx/ty basis vectors must be "
                        "nonzero");
                }
                prepared_states_[s][i] = StationState3d{
                    .active = true,
                    .ux = ux,
                    .uy = uy,
                    .tx = tx,
                    .ty = ty,
                };
            } else {
                throw std::invalid_argument(
                    "set_tire_state: station fields must be inactive "
                    "(active=0/NaN with all-NaN leaves) or active "
                    "(active=1 with finite leaves)");
            }
            base += kLeaves.size();
        }
    }
}

void Distributed3dTireWriter::commit_prepared_state() noexcept {
    states_[0].swap(prepared_states_[0]);
    states_[1].swap(prepared_states_[1]);
    last_time_s_.reset();
    std::ranges::fill(qfrc_dissipative_, 0.0);
    dissipative_power_start_w_ = 0.0;
    brush_loss_step_j_ = 0.0;
    radial_dissipation_power_w_ = 0.0;
}

std::vector<double> Distributed3dTireWriter::state() const {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> out(state_names_.size(), nan_v);
    std::size_t base = 0;
    for (std::size_t s = 0; s < 2; ++s) {
        for (const StationState3d &st : states_[s]) {
            if (!st.active) {
                out[base] = 0.0;
            } else {
                out[base + 0] = 1.0;
                out[base + 1] = st.ux;
                out[base + 2] = st.uy;
                out[base + 3] = st.tx[0];
                out[base + 4] = st.tx[1];
                out[base + 5] = st.tx[2];
                out[base + 6] = st.ty[0];
                out[base + 7] = st.ty[1];
                out[base + 8] = st.ty[2];
            }
            base += kLeaves.size();
        }
    }
    return out;
}

void Distributed3dTireWriter::compute_into(const mjData *d, double dt,
                                           std::span<double> out) {
    const char *err_msg = nullptr;
    bool overflow_err = false;
    const CoreStatus status =
        accumulate_core(d, dt, out, err_msg, overflow_err);
    if (status == CoreStatus::ok) return;
    if (overflow_err) {
        throw std::overflow_error(err_msg != nullptr ? err_msg
                                                     : "tire overflow");
    }
    throw std::invalid_argument(err_msg != nullptr ? err_msg
                                                   : "tire compute failed");
}

CoreStatus Distributed3dTireWriter::try_compute_into(
    const mjData *d, double dt, std::span<double> out) noexcept {
    const char *err_msg = nullptr;
    bool overflow_err = false;
    try {
        return accumulate_core(d, dt, out, err_msg, overflow_err);
    } catch (...) {
        return CoreStatus::engine_failure;
    }
}

std::vector<double> Distributed3dTireWriter::qfrc(const mjData *d, double dt) {
    compute_into(d, dt, qfrc_out_);
    return qfrc_out_;
}

CoreStatus Distributed3dTireWriter::accumulate_core(
    const mjData *d, double dt, std::span<double> out, const char *&err_msg,
    bool &overflow_err) {
    err_msg = nullptr;
    overflow_err = false;

    if (d == nullptr) {
        err_msg = "tire compute needs a live mjData";
        return CoreStatus::invalid_input;
    }
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty())) {
        err_msg = "tire output width";
        return CoreStatus::invalid_input;
    }
    if (!std::isfinite(dt) || !(dt > 0.0)) {
        err_msg = "invalid distributed_3d tire interval";
        return CoreStatus::invalid_input;
    }
    const double time = static_cast<double>(d->time);
    if (!std::isfinite(time) || time < 0.0) {
        err_msg = "contact snapshot needs finite nonnegative time";
        return CoreStatus::invalid_input;
    }
    if (last_time_s_.has_value() && time <= *last_time_s_) {
        err_msg = "tire state can advance only once per increasing timestamp";
        return CoreStatus::invalid_input;
    }
    std::int64_t interval_id = 0;
    if (!try_interval_id(time, dt, interval_id) || interval_id < 0) {
        err_msg = "contact snapshot needs a nonnegative interval ID";
        return CoreStatus::invalid_input;
    }

    std::span<const mjtNum> xpos{};
    std::span<const mjtNum> xmat{};
    std::span<const mjtNum> qvel{};
    if (!try_readonly_buffer(d->geom_xpos, model_access::kXYZ * m_->ngeom,
                             xpos) ||
        !try_readonly_buffer(d->xmat,
                             model_access::kRotationElements * m_->nbody,
                             xmat) ||
        !try_readonly_buffer(d->qvel, m_->nv, qvel)) {
        err_msg = "invalid mjData buffer extents";
        return CoreStatus::invalid_input;
    }

    std::ranges::fill(out, 0.0);
    std::ranges::fill(qfrc_diss_stage_, 0.0);

    const std::size_t nv = static_cast<std::size_t>(nv_);
    constexpr std::size_t kDim =
        static_cast<std::size_t>(model_access::kXYZ);
    const double normal_cosine =
        std::cos(cfg_.distinct_normal_deg * (std::numbers::pi / 180.0));
    const bool track_mode = (cfg_.surface_mode == "track");

    double energy = 0.0;
    double loss = 0.0;
    double radial_loss_power = 0.0;

    for (std::size_t s = 0; s < 2; ++s) {
        const Distributed3dWheelParams &p = (s == 0) ? cfg_.front : cfg_.rear;
        const std::size_t n = stations_[s].angles.size();
        const int body = bodies_[s];
        const std::size_t g3 = kDim * static_cast<std::size_t>(geoms_[s]);
        const std::size_t b9 =
            static_cast<std::size_t>(model_access::kRotationElements) *
            static_cast<std::size_t>(body);

        const Vec3 p_hub = {xpos[g3], xpos[g3 + 1], xpos[g3 + 2]};
        for (const double v : p_hub) {
            if (!std::isfinite(v)) {
                err_msg = "wheel axis must be a finite 3D vector";
                return CoreStatus::invalid_input;
            }
        }
        const Vec3 a_raw = {xmat[b9 + 1], xmat[b9 + 4], xmat[b9 + 7]};
        Vec3 a_hat{};
        if (!try_normalize_unit(a_raw, a_hat)) {
            err_msg = "wheel body spin axis must be finite and nonzero";
            return CoreStatus::invalid_input;
        }

        const bike::terrain::TerrainSample3d hub_sample =
            cfg_.terrain.sample_fn(cfg_.terrain.ctx, p_hub[0], p_hub[1]);
        if (hub_sample.status == bike::terrain::QueryStatus3d::invalid_input) {
            err_msg = "terrain3d sample returned invalid_input";
            return CoreStatus::invalid_input;
        }
        if (hub_sample.status != bike::terrain::QueryStatus3d::ok) {
            err_msg = "terrain3d hub sample outside terrain bounds";
            return CoreStatus::invalid_geometry;
        }

        lift_wheel_stations(p_hub, xmat.subspan(b9, 9), local_[s], points_);

        if (m_->nv < 0 || body <= 0 || body >= m_->nbody ||
            jac_center_.size() != kDim * nv || jr_.size() != kDim * nv) {
            err_msg = "point Jacobian requires a physical body";
            return CoreStatus::invalid_input;
        }
        mj_jac(m_, d, jac_center_.data(), jr_.data(), p_hub.data(), body);

        Vec3 v_hub{};
        if (nv_ > 0) {
            blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                        model_access::kXYZ, nv_, 1.0, jac_center_.data(), nv_,
                        qvel.data(), 1, 0.0, v_hub.data(), 1);
        }

        const Stage1Gate gate = make_stage1_gate(a_hat, p.major_radius_m());
        std::ranges::fill(states_stage_[s], StationState3d{});
        TireSnapshot &snap = snapshot_stage_[s];
        snap.patches.clear();

        std::size_t active_count = 0;
        double normal_total = 0.0;
        double tangent_x_total = 0.0;
        double radial_energy = 0.0;
        double shear_energy = 0.0;
        double radial_power_side = 0.0;
        double release_loss_side = 0.0;
        double brush_loss_side = 0.0;
        double max_depth = 0.0;
        double max_nonpos_delta = -std::numeric_limits<double>::infinity();
        bool any_stage3 = false;
        double weighted_slip_num = 0.0;
        double weighted_ndot_num = 0.0;
        double weighted_mu_num = 0.0;
        double deepest_delta = -1.0;
        std::string_view dominant_surface =
            track_mode ? surface_entry(hub_sample.surface).name : "configured";
        bool any_active_saturated = false;
        bool all_active_saturated = true;

        for (std::size_t sid = 0; sid < n; ++sid) {
            const Vec3 c_i = {points_[kDim * sid], points_[kDim * sid + 1],
                              points_[kDim * sid + 2]};
            const StationQuery q = query_station_3stage(
                cfg_.terrain, gate, p_hub, c_i, p.crown_radius_m);
            if (q.status == StationCullStatus::invalid_input) {
                err_msg = "terrain3d query returned invalid_input";
                return CoreStatus::invalid_input;
            }
            if (q.status == StationCullStatus::outside) {
                err_msg = "terrain3d closest_point outside terrain bounds";
                return CoreStatus::invalid_geometry;
            }
            if (q.status == StationCullStatus::culled) {
                continue;
            }
            any_stage3 = true;
            if (!(q.delta > 0.0)) {
                max_nonpos_delta = std::max(max_nonpos_delta, q.delta);
                continue;
            }

            Vec3 n_hat{};
            if (!try_normalize_unit(q.hit.normal_world, n_hat)) {
                err_msg = "terrain normal must be finite and nonzero";
                return CoreStatus::invalid_input;
            }
            const Vec3 p_c = q.hit.surface_point_world_m;
            const Vec3 r_arm = sub3(p_c, p_hub);
            for (std::size_t k = 0; k < kDim; ++k) {
                const std::size_t a = (k + 1) % kDim;
                const std::size_t b = (k + 2) % kDim;
                for (std::size_t m_idx = 0; m_idx < nv; ++m_idx) {
                    jac_station_[k * nv + m_idx] =
                        jac_center_[k * nv + m_idx] +
                        (jr_[a * nv + m_idx] * r_arm[b] -
                         jr_[b * nv + m_idx] * r_arm[a]);
                }
            }
            Vec3 v_c{};
            if (nv_ > 0) {
                blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                            model_access::kXYZ, nv_, 1.0, jac_station_.data(),
                            nv_, qvel.data(), 1, 0.0, v_c.data(), 1);
            }

            const double w_i = stations_[s].weights[sid];
            const double delta_dot = -dot3(v_c, n_hat);
            const double f_el = station_elastic_force(p, w_i, q.delta);
            const double u_rad = station_radial_energy(p, w_i, q.delta);
            const double hyst =
                1.0 + p.loss_factor *
                          std::tanh(delta_dot / p.hysteresis_rate_eps_mps);
            const double fn = std::max(0.0, f_el * hyst);
            const double rad_diss = std::max(0.0, (fn - f_el) * delta_dot);

            Vec3 tx{};
            Vec3 ty{};
            if (!try_build_road_basis(a_hat, n_hat, tx, ty)) {
                err_msg = "road tangent basis degenerate";
                return CoreStatus::invalid_input;
            }

            const double kbx = w_i * p.kappa_brush_x_n_m_rad;
            const double kby = w_i * p.kappa_brush_y_n_m_rad;
            const StationState3d &old = states_[s][sid];
            const double u_old =
                old.active ? 0.5 * (kbx * old.ux * old.ux +
                                    kby * old.uy * old.uy)
                           : 0.0;

            double ux0 = 0.0;
            double uy0 = 0.0;
            double u0_energy = 0.0;
            if (old.active) {
                const Vec3 n_prev = cross3(old.tx, old.ty);
                if (dot3(n_prev, n_hat) < normal_cosine) {
                    release_loss_side += u_old;
                } else {
                    const Vec3 w_prev = {
                        old.ux * old.tx[0] + old.uy * old.ty[0],
                        old.ux * old.tx[1] + old.uy * old.ty[1],
                        old.ux * old.tx[2] + old.uy * old.ty[2],
                    };
                    const double u_tilde_x = dot3(w_prev, tx);
                    const double u_tilde_y = dot3(w_prev, ty);
                    const double u_tilde_e =
                        0.5 * (kbx * u_tilde_x * u_tilde_x +
                               kby * u_tilde_y * u_tilde_y);
                    const double beta =
                        (u_tilde_e > u_old && u_tilde_e > 0.0)
                            ? std::sqrt(u_old / u_tilde_e)
                            : 1.0;
                    ux0 = beta * u_tilde_x;
                    uy0 = beta * u_tilde_y;
                    u0_energy = 0.5 * (kbx * ux0 * ux0 + kby * uy0 * uy0);
                    release_loss_side += std::max(0.0, u_old - u0_energy);
                }
            }

            const double vsx = dot3(v_c, tx);
            const double vsy = dot3(v_c, ty);
            const double v_s = std::hypot(vsx, vsy);
            const double v_roll = std::abs(dot3(v_hub, tx));

            double mu = 0.0;
            double c_kappa_surf = cfg_.configured_c_kappa_per_fz;
            std::string_view surf_name = "configured";
            if (track_mode) {
                const SurfaceEntry surf = surface_entry(q.hit.surface);
                const double decay = std::exp(-v_s / surf.stribeck_speed_mps);
                const double mu_surf =
                    surf.mu_slide + (surf.mu_peak - surf.mu_slide) * decay;
                mu = std::min(p.mu_cap, mu_surf);
                c_kappa_surf = surf.c_kappa_per_fz;
                surf_name = surf.name;
            } else {
                mu = cfg_.configured_mu
                         ? std::min(p.mu_cap, *cfg_.configured_mu)
                         : p.mu_cap;
            }

            double ux_new = 0.0;
            double uy_new = 0.0;
            double fx = 0.0;
            double fy = 0.0;
            double alpha_sat = 1.0;
            double brush_loss_i = 0.0;

            if (!(fn > 1e-12) || !(mu > 0.0)) {
                brush_loss_i = u0_energy;
                all_active_saturated = false;
            } else {
                const double c_soil = std::max(
                    0.0, (1.0 / c_kappa_surf) - (1.0 / p.c_kappa_ref_per_fz));
                const double rx = dt * v_roll * c_soil * (kbx / fn);
                const double ux_trial = (ux0 + dt * vsx) / (1.0 + rx);
                const double uy_trial = uy0 + dt * vsy;
                const double fx_trial = -kbx * ux_trial;
                const double fy_trial = -kby * uy_trial;
                const double ft_trial = std::hypot(fx_trial, fy_trial);
                const double f_max = mu * fn;
                alpha_sat = (ft_trial > f_max && ft_trial > 0.0)
                                ? (f_max / ft_trial)
                                : 1.0;
                if (alpha_sat < 1.0) {
                    any_active_saturated = true;
                } else {
                    all_active_saturated = false;
                }
                ux_new = alpha_sat * ux_trial;
                uy_new = alpha_sat * uy_trial;
                fx = -kbx * ux_new;
                fy = -kby * uy_new;

                const double dux = ux_new - ux0;
                const double duy = uy_new - uy0;
                const double l_implicit =
                    0.5 * (kbx * dux * dux + kby * duy * duy);
                const double l_relax = kbx * rx * ux_new * ux_new;
                const double l_proj =
                    (1.0 - alpha_sat) * alpha_sat *
                    (kbx * (1.0 + rx) * ux_trial * ux_trial +
                     kby * uy_trial * uy_trial);
                brush_loss_i = std::max(0.0, l_implicit) +
                               std::max(0.0, l_relax) +
                               std::max(0.0, l_proj);

                states_stage_[s][sid] = StationState3d{
                    .active = true,
                    .ux = ux_new,
                    .uy = uy_new,
                    .tx = tx,
                    .ty = ty,
                };
                shear_energy += 0.5 * (kbx * ux_new * ux_new +
                                       kby * uy_new * uy_new);
            }

            const Vec3 force_world = {
                fn * n_hat[0] + fx * tx[0] + fy * ty[0],
                fn * n_hat[1] + fx * tx[1] + fy * ty[1],
                fn * n_hat[2] + fx * tx[2] + fy * ty[2],
            };
            const double fn_diss = fn - f_el;
            const Vec3 f_diss_world = {
                fn_diss * n_hat[0] + fx * tx[0] + fy * ty[0],
                fn_diss * n_hat[1] + fx * tx[1] + fy * ty[1],
                fn_diss * n_hat[2] + fx * tx[2] + fy * ty[2],
            };

            if (nv_ > 0) {
                blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                            model_access::kXYZ, nv_, 1.0, jac_station_.data(),
                            nv_, force_world.data(), 1, 0.0, tmp_.data(), 1);
                for (std::size_t m_idx = 0; m_idx < nv; ++m_idx) {
                    out[m_idx] += tmp_[m_idx];
                }

                blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                            model_access::kXYZ, nv_, 1.0, jac_station_.data(),
                            nv_, f_diss_world.data(), 1, 0.0, tmp_.data(), 1);
                for (std::size_t m_idx = 0; m_idx < nv; ++m_idx) {
                    qfrc_diss_stage_[m_idx] += tmp_[m_idx];
                }
            }

            if (snap.patches.size() >= n) {
                err_msg = "tire snapshot patch capacity exceeded";
                return CoreStatus::invalid_input;
            }
            TirePatch patch{};
            if (!try_fill_patch(
                    p_c, n_hat,
                    {.normal_load_n = fn,
                     .tangent_force_n = fx,
                     .slip_mps = v_s},
                    force_world, patch)) {
                err_msg = "invalid station contact patch values";
                return CoreStatus::invalid_input;
            }
            snap.patches.push_back(patch);

            active_ids_[active_count] = static_cast<int>(sid);
            normals_[kDim * active_count] = n_hat[0];
            normals_[kDim * active_count + 1] = n_hat[1];
            normals_[kDim * active_count + 2] = n_hat[2];
            ++active_count;

            normal_total += fn;
            tangent_x_total += fx;
            radial_energy += u_rad;
            radial_power_side += rad_diss;
            brush_loss_side += brush_loss_i;
            max_depth = std::max(max_depth, q.delta);
            weighted_slip_num += fn * v_s;
            weighted_ndot_num += fn * (-delta_dot);
            weighted_mu_num += fn * mu;
            if (q.delta > deepest_delta) {
                deepest_delta = q.delta;
                dominant_surface = surf_name;
            }
        }

        // Detached stations release their stored shear energy as loss.
        for (std::size_t i = 0; i < n; ++i) {
            const StationState3d &old = states_[s][i];
            if (old.active && !states_stage_[s][i].active) {
                const double w_i = stations_[s].weights[i];
                const double kbx = w_i * p.kappa_brush_x_n_m_rad;
                const double kby = w_i * p.kappa_brush_y_n_m_rad;
                release_loss_side +=
                    0.5 * (kbx * old.ux * old.ux + kby * old.uy * old.uy);
            }
        }

        bool distinct = false;
        if (active_count > 1) {
            for (std::size_t a = 0; a < active_count && !distinct; ++a) {
                const Vec3 na = {normals_[kDim * a], normals_[kDim * a + 1],
                                 normals_[kDim * a + 2]};
                for (std::size_t b = a + 1; b < active_count; ++b) {
                    const Vec3 nb = {normals_[kDim * b], normals_[kDim * b + 1],
                                     normals_[kDim * b + 2]};
                    if (dot3(na, nb) < normal_cosine) {
                        distinct = true;
                        break;
                    }
                }
            }
        }
        bool separate = false;
        if (active_count > 1) {
            for (std::size_t j = 0; j + 1 < active_count; ++j) {
                if (active_ids_[j + 1] - active_ids_[j] > 1) {
                    separate = true;
                    break;
                }
            }
            if (active_ids_[0] == 0 &&
                active_ids_[active_count - 1] == static_cast<int>(n) - 1) {
                separate = false;
            }
        }

        snap.time_s = time;
        snap.interval_id = interval_id;
        snap.backend.assign(kBackendName.data(), kBackendName.size());
        snap.geometric_contact = (active_count > 0);
        snap.wheel_axis_m = p_hub;
        snap.effective_radius_m = effective_radius(snap.patches, p_hub);

        double signed_overlap = max_depth;
        if (active_count == 0) {
            if (any_stage3) {
                signed_overlap = max_nonpos_delta;
            } else {
                const double clearance_est =
                    p_hub[2] - hub_sample.height_m - p.outer_radius_m;
                signed_overlap =
                    (clearance_est > 0.0) ? -clearance_est : -kStencilMarginM;
            }
        }

        const double side_brush_loss =
            std::max(0.0, release_loss_side) + brush_loss_side;
        const double mean_mu_idle =
            track_mode
                ? std::min(p.mu_cap,
                           surface_entry(hub_sample.surface).mu_peak)
                : (cfg_.configured_mu
                       ? std::min(p.mu_cap, *cfg_.configured_mu)
                       : p.mu_cap);

        TireDiagnostics &diag = diagnostics_stage_[s];
        diag.multi_support = distinct || separate;
        diag.penetration_m = signed_overlap;
        diag.normal_speed_mps =
            (normal_total > 0.0) ? (weighted_ndot_num / normal_total) : 0.0;
        diag.slip_mps =
            (normal_total > 0.0) ? (weighted_slip_num / normal_total) : 0.0;
        diag.normal_load_n = normal_total;
        diag.tangent_force_n = tangent_x_total;
        diag.friction_coefficient =
            (normal_total > 0.0) ? (weighted_mu_num / normal_total)
                                 : mean_mu_idle;
        diag.surface.assign(dominant_surface.data(), dominant_surface.size());
        diag.branch_release_loss_j = std::max(0.0, release_loss_side);
        diag.brush_loss_j = brush_loss_side;
        diag.radial_energy_j = radial_energy;
        diag.shear_energy_j = shear_energy;
        diag.outside_material_load_range =
            !(p.valid_load_range_n[0] <= normal_total &&
              normal_total <= p.valid_load_range_n[1]);
        diag.supports_multiple_contacts = true;
        diag.station_count = p.station_count;
        diag.active_station_count = static_cast<int>(active_count);
        diag.nonsmooth_station_count = 0;
        diag.max_patch_count = static_cast<int>(active_count);
        diag.outside_material_deflection_range =
            !(p.valid_deflection_range_m[0] <= max_depth &&
              max_depth <= p.valid_deflection_range_m[1]);
        diag.calibration_status.assign(kCalibrationStatus.data(),
                                       kCalibrationStatus.size());
        diag.capability_status.assign(kCapabilityStatus.data(),
                                      kCapabilityStatus.size());
        diag.pressure_bar = p.pressure_bar;
        diag.patch_count = static_cast<int>(active_count);
        diag.airborne = (active_count == 0);
        diag.rim_strike_active = (max_depth > p.rim_strike_deflection_m());
        diag.fully_sliding =
            (active_count > 0 && any_active_saturated && all_active_saturated);
        diag.dissipated_power_w = radial_power_side + side_brush_loss / dt;

        radial_loss_power += radial_power_side;
        energy += radial_energy + shear_energy;
        loss += side_brush_loss;
    }

    for (const double v : out) {
        if (!std::isfinite(v)) {
            err_msg = "Distributed3dTireWriter.qfrc";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    for (const double v : qfrc_diss_stage_) {
        if (!std::isfinite(v)) {
            err_msg = "Distributed3dTireWriter.qfrc_dissipative";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    if (!std::isfinite(energy) || !std::isfinite(loss) ||
        !std::isfinite(radial_loss_power)) {
        err_msg = "Distributed3dTireWriter.energy_or_power";
        overflow_err = true;
        return CoreStatus::invalid_input;
    }
    const double diss_power_start =
        (nv_ > 0)
            ? -blas::ddot(nv_, qfrc_diss_stage_.data(), 1, qvel.data(), 1)
            : 0.0;
    if (!std::isfinite(diss_power_start)) {
        err_msg = "Distributed3dTireWriter.dissipative_power_start_w";
        overflow_err = true;
        return CoreStatus::invalid_input;
    }

    for (std::size_t s = 0; s < 2; ++s) {
        states_[s].swap(states_stage_[s]);
        snapshots_[s] = snapshot_stage_[s];
        diagnostics_[s] = diagnostics_stage_[s];
    }
    qfrc_dissipative_.swap(qfrc_diss_stage_);
    committed_ = true;
    elastic_energy_j_ = energy;
    brush_loss_step_j_ = loss;
    radial_dissipation_power_w_ = radial_loss_power;
    dissipative_power_start_w_ = diss_power_start;
    last_time_s_ = time;
    return CoreStatus::ok;
}

double Distributed3dTireWriter::stored_energy(const mjData *d) const {
    if (d == nullptr) {
        throw std::invalid_argument(
            "tire stored_energy needs a live mjData");
    }
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        d->geom_xpos, model_access::kXYZ * m_->ngeom);
    const std::span<const mjtNum> xmat = model_access::readonly_buffer(
        d->xmat, model_access::kRotationElements * m_->nbody);

    double energy = 0.0;
    for (std::size_t s = 0; s < 2; ++s) {
        const Distributed3dWheelParams &p = (s == 0) ? cfg_.front : cfg_.rear;
        const std::size_t n = stations_[s].angles.size();
        const std::size_t g3 =
            static_cast<std::size_t>(model_access::kXYZ) *
            static_cast<std::size_t>(geoms_[s]);
        const std::size_t b9 =
            static_cast<std::size_t>(model_access::kRotationElements) *
            static_cast<std::size_t>(bodies_[s]);

        const Vec3 p_hub = {xpos[g3], xpos[g3 + 1], xpos[g3 + 2]};
        const Vec3 a_raw = {xmat[b9 + 1], xmat[b9 + 4], xmat[b9 + 7]};
        Vec3 a_hat{};
        normalize_unit_axis(a_raw, a_hat, "wheel body spin axis");

        const bike::terrain::TerrainSample3d hub_sample =
            cfg_.terrain.sample_fn(cfg_.terrain.ctx, p_hub[0], p_hub[1]);
        if (hub_sample.status == bike::terrain::QueryStatus3d::invalid_input) {
            throw std::invalid_argument("terrain3d sample returned invalid_input");
        }
        if (hub_sample.status != bike::terrain::QueryStatus3d::ok) {
            throw std::invalid_argument(
                "terrain3d hub sample outside terrain bounds");
        }

        lift_wheel_stations(p_hub, xmat.subspan(b9, 9), local_[s], points_);
        const Stage1Gate gate = make_stage1_gate(a_hat, p.major_radius_m());

        double u_rad_side = 0.0;
        double u_shear_side = 0.0;
        for (std::size_t sid = 0; sid < n; ++sid) {
            const std::size_t off =
                static_cast<std::size_t>(model_access::kXYZ) * sid;
            const Vec3 c_i = {points_[off], points_[off + 1], points_[off + 2]};
            const StationQuery q = query_station_3stage(
                cfg_.terrain, gate, p_hub, c_i, p.crown_radius_m);
            if (q.status == StationCullStatus::invalid_input) {
                throw std::invalid_argument(
                    "terrain3d query returned invalid_input");
            }
            if (q.status == StationCullStatus::outside) {
                throw std::invalid_argument(
                    "terrain3d closest_point outside terrain bounds");
            }
            const double w_i = stations_[s].weights[sid];
            if (q.status == StationCullStatus::evaluated && q.delta > 0.0) {
                u_rad_side += station_radial_energy(p, w_i, q.delta);
            }
            const StationState3d &st = states_[s][sid];
            if (st.active) {
                const double kbx = w_i * p.kappa_brush_x_n_m_rad;
                const double kby = w_i * p.kappa_brush_y_n_m_rad;
                u_shear_side +=
                    0.5 * (kbx * st.ux * st.ux + kby * st.uy * st.uy);
            }
        }
        validation::derived(u_rad_side, "Distributed3dTireWriter.radial_energy");
        validation::derived(u_shear_side,
                            "Distributed3dTireWriter.shear_energy");
        energy += u_rad_side + u_shear_side;
    }
    return validation::derived(energy, "Distributed3dTireWriter.stored_energy");
}

void Distributed3dTireWriter::probe_compute_into(const mjData *d, double dt,
                                                 std::span<double> out) {
    if (d == nullptr) {
        throw std::invalid_argument("tire probe needs a live mjData");
    }
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty())) {
        throw std::invalid_argument("tire output width");
    }
    Distributed3dTireWriter probe(*this);
    probe.last_time_s_.reset();
    probe.compute_into(d, dt, out);
    probe_snapshots_ = probe.snapshots_;
    probe_committed_ = probe.committed_;
}

std::vector<double> Distributed3dTireWriter::probe_qfrc(const mjData *d,
                                                        double dt) {
    std::vector<double> out(static_cast<std::size_t>(nv_), 0.0);
    probe_compute_into(d, dt, out);
    return out;
}

double Distributed3dTireWriter::step_loss_j(std::span<const double> qvel_mid,
                                            double dt) const noexcept {
    if (!std::isfinite(dt) || !(dt > 0.0)) {
        return std::max(0.0, brush_loss_step_j_);
    }
    const double start_step =
        brush_loss_step_j_ + radial_dissipation_power_w_ * dt;
    if (qvel_mid.size() != static_cast<std::size_t>(nv_) || nv_ <= 0) {
        return std::max(0.0, start_step);
    }
    const double mid_diss_power =
        -blas::ddot(nv_, qfrc_dissipative_.data(), 1, qvel_mid.data(), 1);
    const double total =
        start_step + (mid_diss_power - dissipative_power_start_w_) * dt;
    if (!std::isfinite(total)) {
        return std::max(0.0, start_step);
    }
    return std::max(0.0, total);
}
