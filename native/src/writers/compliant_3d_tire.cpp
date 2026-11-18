// writers/compliant_3d_tire.cpp — 3D toroidal crown-circle compliant tire
// writer (`compliant_3d` backend). Implements spec §1 verbatim with zero
// heap allocations on `try_compute_into` and zero link dependencies on
// `bike_terrain`.
#include "writers/compliant_3d_tire.hpp"

#include "cblas_abi.hpp"
#include "model_access.hpp"
#include "validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
constexpr std::string_view kBackendName = "compliant_3d";

class InvalidGeometryError final : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
    InvalidGeometryError(const InvalidGeometryError &) = default;
    InvalidGeometryError &operator=(const InvalidGeometryError &) = default;
    InvalidGeometryError(InvalidGeometryError &&) = default;
    InvalidGeometryError &operator=(InvalidGeometryError &&) = default;
    ~InvalidGeometryError() noexcept override;
};

InvalidGeometryError::~InvalidGeometryError() noexcept = default;

struct SurfaceEntry {
    std::string_view name;
    double mu_peak;
    double mu_slide;
    double c_kappa_per_fz;
    double stribeck_speed_mps;
};

// Embedded 5-entry surface registry (spec §1.5 / terrain/surface.cpp:10-16)
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

[[nodiscard]] bool try_interval_id(double time, double dt, std::int64_t &out,
                                   const char *&err_msg,
                                   bool &overflow_err) noexcept {
    if (!std::isfinite(time) || time < 0.0) {
        err_msg = "invalid tire.interval_id.time";
        return false;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
        err_msg = "invalid tire.interval_id.dt";
        return false;
    }
    const double quotient = time / dt;
    if (!std::isfinite(quotient) || quotient >= 0x1p63) {
        err_msg = "tire.interval_id";
        overflow_err = true;
        return false;
    }
    double base = std::floor(quotient);
    const double fraction = quotient - base;
    if (fraction > 0.5 || (fraction == 0.5 && std::fmod(base, 2.0) != 0.0)) {
        base += 1.0;
    }
    if (base >= 0x1p63) {
        err_msg = "tire.interval_id";
        overflow_err = true;
        return false;
    }
    out = static_cast<std::int64_t>(base);
    return true;
}

[[nodiscard]] bool try_point_jacobian_into(const mjModel *m, const mjData *d,
                                           int body_id, const Vec3 &point,
                                           std::span<mjtNum> jacp,
                                           const char *&err_msg) noexcept {
    for (const double v : point) {
        if (!std::isfinite(v)) {
            err_msg = "invalid world point";
            return false;
        }
    }
    if (body_id < 0 || body_id >= m->nbody) {
        err_msg = "point Jacobian body: invalid model ID";
        return false;
    }
    if (body_id == 0) {
        err_msg = "point Jacobian requires a physical body";
        return false;
    }
    if (m->nv < 0) {
        err_msg = "point Jacobian needs a nonnegative model width";
        return false;
    }
    const auto nv_u = static_cast<std::size_t>(m->nv);
    const auto k_xyz = static_cast<std::size_t>(model_access::kXYZ);
    if (jacp.size() / k_xyz != nv_u || jacp.size() % k_xyz != 0) {
        err_msg = "point Jacobian destination must be 3*nv elements";
        return false;
    }
    mj_jac(m, d, jacp.data(), nullptr, point.data(), body_id);
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

[[nodiscard]] double elastic_normal_force(const Compliant3dWheelParams &p,
                                          double delta) noexcept {
    if (!(delta > 0.0)) return 0.0;
    const double xi_rim = delta - p.rim_strike_deflection_m();
    return p.k_pneumatic_n_m * smooth_ramp({.x = delta, .x0 = p.delta_0_m}) +
           p.k_carcass_n_m *
               smooth_ramp({.x = delta, .x0 = p.delta_carcass_m}) +
           p.k_rim_n_m * smooth_ramp({.x = xi_rim, .x0 = p.delta_rim_0_m});
}

[[nodiscard]] double radial_stored_energy(const Compliant3dWheelParams &p,
                                          double delta) noexcept {
    if (!(delta > 0.0)) return 0.0;
    const double xi_rim = delta - p.rim_strike_deflection_m();
    return p.k_pneumatic_n_m *
               smooth_ramp_integral({.x = delta, .x0 = p.delta_0_m}) +
           p.k_carcass_n_m *
               smooth_ramp_integral({.x = delta, .x0 = p.delta_carcass_m}) +
           p.k_rim_n_m *
               smooth_ramp_integral({.x = xi_rim, .x0 = p.delta_rim_0_m});
}

struct CrownContact {
    double delta = 0.0;
    Vec3 point{};
    Vec3 normal{0.0, 0.0, 1.0};
    rider::SurfaceLabel surface = rider::SurfaceLabel::hardpack;
};

void validate_wheel_params(const Compliant3dWheelParams &p,
                           std::string_view side) {
    const std::string prefix = "compliant_3d." + std::string(side);
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
    validation::positive(p.k_pneumatic_n_m, prefix + ".k_pneumatic_n_m");
    validation::positive(p.delta_0_m, prefix + ".delta_0_m");
    validation::nonnegative(p.k_carcass_n_m, prefix + ".k_carcass_n_m");
    validation::positive(p.delta_carcass_m, prefix + ".delta_carcass_m");
    validation::nonnegative(p.k_rim_n_m, prefix + ".k_rim_n_m");
    validation::positive(p.delta_rim_0_m, prefix + ".delta_rim_0_m");
    validation::nonnegative(p.loss_factor, prefix + ".loss_factor");
    if (!(p.loss_factor < 1.0)) {
        throw std::invalid_argument(prefix + ".loss_factor must lie in [0, 1)");
    }
    validation::positive(p.hysteresis_rate_eps_mps,
                         prefix + ".hysteresis_rate_eps_mps");
    validation::positive(p.c_alpha_per_fz, prefix + ".c_alpha_per_fz");
    validation::positive(p.k_brush_x_n_m, prefix + ".k_brush_x_n_m");
    validation::positive(p.k_brush_y_n_m, prefix + ".k_brush_y_n_m");
    validation::nonnegative(p.mu_cap, prefix + ".mu_cap");
    validation::nonnegative(p.valid_load_range_n[0],
                            prefix + ".valid_load_range_n.0");
    validation::positive(p.valid_load_range_n[1],
                         prefix + ".valid_load_range_n.1");
    if (!(p.valid_load_range_n[0] < p.valid_load_range_n[1])) {
        throw std::invalid_argument(prefix +
                                    ".valid_load_range_n must be ordered");
    }
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

[[nodiscard]] bool try_build_wheel_plane_basis(const Vec3 &a_hat, Vec3 &e1,
                                               Vec3 &e2) noexcept {
    const Vec3 d_perp = {
        a_hat[2] * a_hat[0],
        a_hat[2] * a_hat[1],
        a_hat[2] * a_hat[2] - 1.0,
    };
    const double d_perp_len = norm3(d_perp);
    if (d_perp_len > 1e-6) {
        e1 = {d_perp[0] / d_perp_len, d_perp[1] / d_perp_len,
              d_perp[2] / d_perp_len};
    } else {
        const Vec3 r_ref = (std::abs(a_hat[0]) < 0.9) ? Vec3{1.0, 0.0, 0.0}
                                                      : Vec3{0.0, 1.0, 0.0};
        const double proj = dot3(r_ref, a_hat);
        const Vec3 ortho = {r_ref[0] - proj * a_hat[0],
                            r_ref[1] - proj * a_hat[1],
                            r_ref[2] - proj * a_hat[2]};
        if (!try_normalize_unit(ortho, e1)) return false;
    }
    return try_normalize_unit(cross3(e1, a_hat), e2);
}

struct WheelPose3d {
    Vec3 p_hub{};
    Vec3 a_hat{};
};

struct CrownArc {
    Vec3 p_hub{};
    Vec3 e1{};
    Vec3 e2{};
    double r_major = 0.0;
    double r_search = 0.0;
};

[[nodiscard]] CoreStatus query_crown_at(
    const Compliant3dTerrainCallbacks &terrain, const CrownArc &arc,
    double theta, bike::terrain::TorusClosestPoint3d &q,
    const char *&err_msg) noexcept {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const Vec3 c = {
        arc.p_hub[0] + arc.r_major * (ct * arc.e1[0] + st * arc.e2[0]),
        arc.p_hub[1] + arc.r_major * (ct * arc.e1[1] + st * arc.e2[1]),
        arc.p_hub[2] + arc.r_major * (ct * arc.e1[2] + st * arc.e2[2]),
    };
    q = terrain.closest_point_fn(terrain.ctx, c, arc.r_search);
    if (q.status == bike::terrain::QueryStatus3d::invalid_input) {
        err_msg = "terrain3d closest_point returned invalid_input";
        return CoreStatus::invalid_input;
    }
    if (q.status == bike::terrain::QueryStatus3d::outside) {
        err_msg = "terrain3d closest_point outside terrain bounds";
        return CoreStatus::invalid_geometry;
    }
    return CoreStatus::ok;
}

[[nodiscard]] CoreStatus find_crown_contact(
    const Compliant3dTerrainCallbacks &terrain, const WheelPose3d &pose,
    const Compliant3dWheelParams &params, CrownContact &out,
    const char *&err_msg) noexcept {
    Vec3 e1{};
    Vec3 e2{};
    if (!try_build_wheel_plane_basis(pose.a_hat, e1, e2)) {
        err_msg = "wheel-plane basis must be finite and nonzero";
        return CoreStatus::invalid_input;
    }

    const double r_crown = params.crown_radius_m;
    const double r_major = params.major_radius_m();
    const double r_search = r_crown + params.rim_strike_deflection_m() + 0.03;
    const CrownArc arc{
        .p_hub = pose.p_hub,
        .e1 = e1,
        .e2 = e2,
        .r_major = r_major,
        .r_search = r_search,
    };
    constexpr double kThetaMax = 5.0 * std::numbers::pi / 12.0;

    bike::terrain::TorusClosestPoint3d q_mid{};
    if (const CoreStatus st = query_crown_at(terrain, arc, 0.0, q_mid, err_msg);
        st != CoreStatus::ok) {
        return st;
    }
    const double airborne_chord = 2.0 * r_major * std::sin(0.5 * kThetaMax);
    if (q_mid.signed_distance_m > r_crown + airborne_chord) {
        Vec3 n_hat{};
        if (!try_normalize_unit(q_mid.normal_world, n_hat)) {
            err_msg = "terrain normal must be finite and nonzero";
            return CoreStatus::invalid_input;
        }
        out.delta = r_crown - q_mid.signed_distance_m;
        out.point = q_mid.surface_point_world_m;
        out.normal = n_hat;
        out.surface = q_mid.surface;
        return CoreStatus::ok;
    }

    constexpr std::size_t kCoarseCount = 17;
    constexpr std::size_t kMidIndex = 8;
    std::array<double, kCoarseCount> ds_coarse{};
    std::array<double, kCoarseCount> theta_coarse{};
    bike::terrain::TorusClosestPoint3d q_best = q_mid;
    bike::terrain::TorusClosestPoint3d q_tmp{};
    for (std::size_t i = 0; i < kCoarseCount; ++i) {
        const double theta =
            -kThetaMax +
            static_cast<double>(i) * (2.0 * kThetaMax / 16.0);
        theta_coarse[i] = theta;
        if (i == kMidIndex) {
            ds_coarse[i] = q_mid.signed_distance_m;
        } else {
            if (const CoreStatus st =
                    query_crown_at(terrain, arc, theta, q_tmp, err_msg);
                st != CoreStatus::ok) {
                return st;
            }
            ds_coarse[i] = q_tmp.signed_distance_m;
            if (q_tmp.signed_distance_m < q_best.signed_distance_m) {
                q_best = q_tmp;
            }
        }
    }

    std::array<std::size_t, kCoarseCount> minima{};
    std::size_t n_minima = 0;
    for (std::size_t i = 0; i < kCoarseCount; ++i) {
        const bool left_ok = (i == 0) || (ds_coarse[i] <= ds_coarse[i - 1]);
        const bool right_ok =
            (i + 1 == kCoarseCount) || (ds_coarse[i] <= ds_coarse[i + 1]);
        if (left_ok && right_ok) {
            minima[n_minima++] = i;
        }
    }
    if (n_minima == 0) {
        minima[0] = kMidIndex;
        n_minima = 1;
    }

    std::ranges::sort(
        std::span{minima}.first(n_minima),
        [&](std::size_t a, std::size_t b) {
            if (ds_coarse[a] != ds_coarse[b]) {
                return ds_coarse[a] < ds_coarse[b];
            }
            const std::ptrdiff_t da =
                std::abs(static_cast<std::ptrdiff_t>(a) -
                         static_cast<std::ptrdiff_t>(kMidIndex));
            const std::ptrdiff_t db =
                std::abs(static_cast<std::ptrdiff_t>(b) -
                         static_cast<std::ptrdiff_t>(kMidIndex));
            return da < db;
        });

    const double inv_phi = 0.5 * (std::sqrt(5.0) - 1.0);
    const std::size_t n_refine = std::min<std::size_t>(n_minima, 2);
    for (std::size_t k = 0; k < n_refine; ++k) {
        const std::size_t idx = minima[k];
        const std::size_t lo = idx > 0 ? idx - 1 : 0;
        const std::size_t hi = (idx + 1 < kCoarseCount) ? idx + 1 : idx;
        double a = theta_coarse[lo];
        double b = theta_coarse[hi];
        if (!(b > a)) continue;

        double x1 = b - inv_phi * (b - a);
        double x2 = a + inv_phi * (b - a);
        bike::terrain::TorusClosestPoint3d q1{};
        bike::terrain::TorusClosestPoint3d q2{};
        if (const CoreStatus st = query_crown_at(terrain, arc, x1, q1, err_msg);
            st != CoreStatus::ok) {
            return st;
        }
        if (const CoreStatus st = query_crown_at(terrain, arc, x2, q2, err_msg);
            st != CoreStatus::ok) {
            return st;
        }
        if (q1.signed_distance_m < q_best.signed_distance_m) q_best = q1;
        if (q2.signed_distance_m < q_best.signed_distance_m) q_best = q2;

        for (int iter = 0; iter < 22; ++iter) {
            if (q1.signed_distance_m < q2.signed_distance_m) {
                b = x2;
                x2 = x1;
                q2 = q1;
                x1 = b - inv_phi * (b - a);
                if (const CoreStatus st =
                        query_crown_at(terrain, arc, x1, q1, err_msg);
                    st != CoreStatus::ok) {
                    return st;
                }
                if (q1.signed_distance_m < q_best.signed_distance_m) {
                    q_best = q1;
                }
            } else {
                a = x1;
                x1 = x2;
                q1 = q2;
                x2 = a + inv_phi * (b - a);
                if (const CoreStatus st =
                        query_crown_at(terrain, arc, x2, q2, err_msg);
                    st != CoreStatus::ok) {
                    return st;
                }
                if (q2.signed_distance_m < q_best.signed_distance_m) {
                    q_best = q2;
                }
            }
        }

        // Safeguarded secant/bisection root polish on stationarity
        // (dc/dtheta) . n == 0 so lever-arm error in tx drops from
        // O(1e-6 m) to O(1e-16 m), eliminating parasitic normal spin torque.
        const auto stationarity = [&](double theta,
                                      const Vec3 &n_raw) -> double {
            const double ct = std::cos(theta);
            const double st = std::sin(theta);
            const Vec3 tang = {
                -st * arc.e1[0] + ct * arc.e2[0],
                -st * arc.e1[1] + ct * arc.e2[1],
                -st * arc.e1[2] + ct * arc.e2[2],
            };
            return dot3(tang, n_raw);
        };
        bike::terrain::TorusClosestPoint3d qa{};
        bike::terrain::TorusClosestPoint3d qb{};
        if (const CoreStatus st = query_crown_at(terrain, arc, a, qa, err_msg);
            st != CoreStatus::ok) {
            return st;
        }
        if (const CoreStatus st = query_crown_at(terrain, arc, b, qb, err_msg);
            st != CoreStatus::ok) {
            return st;
        }
        double ga = stationarity(a, qa.normal_world);
        double gb = stationarity(b, qb.normal_world);
        if (ga < 0.0 && gb > 0.0) {
            for (int iter = 0; iter < 12; ++iter) {
                const double span = b - a;
                if (!(span > 1e-15)) break;
                double tm = a - ga * (span / (gb - ga));
                if (!(tm > a + 0.05 * span && tm < b - 0.05 * span)) {
                    tm = 0.5 * (a + b);
                }
                bike::terrain::TorusClosestPoint3d qm{};
                if (const CoreStatus st =
                        query_crown_at(terrain, arc, tm, qm, err_msg);
                    st != CoreStatus::ok) {
                    return st;
                }
                if (qm.signed_distance_m <= q_best.signed_distance_m + 1e-13) {
                    q_best = qm;
                }
                const double gm = stationarity(tm, qm.normal_world);
                if (std::abs(gm) <= 1e-15) break;
                if (gm < 0.0) {
                    a = tm;
                    ga = gm;
                } else {
                    b = tm;
                    gb = gm;
                }
            }
        }
    }

    Vec3 n_hat{};
    if (!try_normalize_unit(q_best.normal_world, n_hat)) {
        err_msg = "terrain normal must be finite and nonzero";
        return CoreStatus::invalid_input;
    }
    out.delta = r_crown - q_best.signed_distance_m;
    out.point = q_best.surface_point_world_m;
    out.normal = n_hat;
    out.surface = q_best.surface;
    return CoreStatus::ok;
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

Compliant3dTireConfig make_compliant_3d_tire_config(
    bike::spatial::SpatialTirePreset preset,
    Compliant3dTerrainCallbacks terrain) {
    switch (preset) {
    case bike::spatial::SpatialTirePreset::mtb_2p8_1p2bar: {
        const Compliant3dWheelParams front{
            .pressure_bar = 1.20,
            .outer_radius_m = 0.3790,
            .rim_radius_m = 0.3200,
            .crown_radius_m = 0.0355,
            .compressed_casing_m = 0.0060,
            .k_pneumatic_n_m = 53290.0,
            .delta_0_m = 0.00025,
            .k_carcass_n_m = 18650.0,
            .delta_carcass_m = 0.0357,
            .k_rim_n_m = 350000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0550,
            .hysteresis_rate_eps_mps = 0.0100,
            .c_alpha_per_fz = 13.90,
            .k_brush_x_n_m = 83600.0,
            .k_brush_y_n_m = 36300.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3500.0},
        };
        const Compliant3dWheelParams rear{
            .pressure_bar = 1.20,
            .outer_radius_m = 0.3600,
            .rim_radius_m = 0.3000,
            .crown_radius_m = 0.0355,
            .compressed_casing_m = 0.0060,
            .k_pneumatic_n_m = 53290.0,
            .delta_0_m = 0.00025,
            .k_carcass_n_m = 18790.0,
            .delta_carcass_m = 0.0357,
            .k_rim_n_m = 350000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0550,
            .hysteresis_rate_eps_mps = 0.0100,
            .c_alpha_per_fz = 13.90,
            .k_brush_x_n_m = 83600.0,
            .k_brush_y_n_m = 36300.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3500.0},
        };
        return Compliant3dTireConfig{
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
        const Compliant3dWheelParams front{
            .pressure_bar = 1.50,
            .outer_radius_m = 0.3720,
            .rim_radius_m = 0.3200,
            .crown_radius_m = 0.0300,
            .compressed_casing_m = 0.0060,
            .k_pneumatic_n_m = 54150.0,
            .delta_0_m = 0.00025,
            .k_carcass_n_m = 8170.0,
            .delta_carcass_m = 0.0275,
            .k_rim_n_m = 350000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0662,
            .hysteresis_rate_eps_mps = 0.0100,
            .c_alpha_per_fz = 11.65,
            .k_brush_x_n_m = 83600.0,
            .k_brush_y_n_m = 30800.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3000.0},
        };
        const Compliant3dWheelParams rear{
            .pressure_bar = 1.70,
            .outer_radius_m = 0.3520,
            .rim_radius_m = 0.3000,
            .crown_radius_m = 0.0300,
            .compressed_casing_m = 0.0060,
            .k_pneumatic_n_m = 61070.0,
            .delta_0_m = 0.00025,
            .k_carcass_n_m = 3380.0,
            .delta_carcass_m = 0.0211,
            .k_rim_n_m = 350000.0,
            .delta_rim_0_m = 0.0020,
            .loss_factor = 0.0690,
            .hysteresis_rate_eps_mps = 0.0100,
            .c_alpha_per_fz = 10.78,
            .k_brush_x_n_m = 83600.0,
            .k_brush_y_n_m = 28500.0,
            .mu_cap = 1.10,
            .valid_load_range_n = {0.0, 3000.0},
        };
        return Compliant3dTireConfig{
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

Compliant3dTireWriter::Compliant3dTireWriter(const mjModel *m,
                                             const mjData * /*d*/,
                                             Compliant3dTireConfig config)
    : m_(m), cfg_(std::move(config)) {
    if (m_ == nullptr) {
        throw std::invalid_argument(
            "compliant_3d tire writer needs a live mjModel");
    }
    if (m_->nv < 0 || m_->nv > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(
            "compliant_3d tire writer needs a model width in int range");
    }
    nv_ = static_cast<int>(m_->nv);

    if (cfg_.surface_mode != "track" && cfg_.surface_mode != "configured") {
        throw std::invalid_argument(
            "compliant_3d surface_mode must be 'track' or 'configured'");
    }
    if (!cfg_.terrain.valid()) {
        throw std::invalid_argument(
            "compliant_3d tire writer requires valid Terrain3d callbacks");
    }
    validation::positive(cfg_.distinct_normal_deg,
                         "compliant_3d.distinct_normal_deg");
    if (!(cfg_.distinct_normal_deg < 180.0)) {
        throw std::invalid_argument(
            "compliant_3d.distinct_normal_deg must be < 180");
    }
    if (cfg_.configured_mu.has_value()) {
        validation::nonnegative(*cfg_.configured_mu,
                                "compliant_3d.configured_mu");
    }
    validation::positive(cfg_.configured_c_kappa_per_fz,
                         "compliant_3d.configured_c_kappa_per_fz");
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
                    "compliant_3d");
            }
        }
    }

    const auto nv_u = static_cast<std::size_t>(nv_);
    jac_contact_.resize(static_cast<std::size_t>(model_access::kXYZ) * nv_u);
    jac_center_.resize(static_cast<std::size_t>(model_access::kXYZ) * nv_u);
    tmp_.resize(nv_u);
    qfrc_out_.resize(nv_u);
    qfrc_dissipative_.resize(nv_u);
    qfrc_diss_stage_.resize(nv_u);

    constexpr std::size_t kStringCap = 32;
    constexpr std::size_t kStatusCap = 64;
    for (std::size_t s = 0; s < 2; ++s) {
        snapshot_stage_[s].patches.reserve(kMaxSnapshotPatches);
        snapshot_stage_[s].backend.reserve(kStringCap);
        snapshots_[s].patches.reserve(kMaxSnapshotPatches);
        snapshots_[s].backend.reserve(kStringCap);
        probe_snapshots_[s].patches.reserve(kMaxSnapshotPatches);
        probe_snapshots_[s].backend.reserve(kStringCap);
        diagnostics_stage_[s].surface.reserve(kStringCap);
        diagnostics_stage_[s].calibration_status.reserve(kStatusCap);
        diagnostics_stage_[s].capability_status.reserve(kStatusCap);
        diagnostics_[s].surface.reserve(kStringCap);
        diagnostics_[s].calibration_status.reserve(kStatusCap);
        diagnostics_[s].capability_status.reserve(kStatusCap);
    }
}

void Compliant3dTireWriter::reset() {
    states_ = {};
    prepared_states_ = {};
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

const std::vector<std::string> &Compliant3dTireWriter::state_names() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> v;
        v.reserve(28);
        for (const char *side : {"front", "rear"}) {
            for (const char *field :
                 {"ux", "uy", "tx.0", "tx.1", "tx.2", "ty.0", "ty.1", "ty.2",
                  "point.0", "point.1", "point.2", "center.0", "center.1",
                  "center.2"}) {
                v.push_back(std::string(side) + "." + field);
            }
        }
        return v;
    }();
    return names;
}

void Compliant3dTireWriter::set_state(std::span<const std::string> names,
                                      std::span<const double> row) {
    prepare_state(names, row);
    commit_prepared_state();
}

void Compliant3dTireWriter::prepare_state(std::span<const std::string> names,
                                          std::span<const double> row) {
    if (names.size() != row.size()) {
        throw std::invalid_argument(
            "set_tire_state: names and row must have equal length");
    }
    for (const std::string &want : state_names()) {
        if (!std::ranges::contains(names, want)) {
            throw std::invalid_argument(
                "set_tire_state: missing tire-state column '" + want + "'");
        }
    }

    struct SideAcc {
        bool ux_seen = false;
        bool uy_seen = false;
        double ux = 0.0;
        double uy = 0.0;
        std::array<bool, 3> tx_seen{};
        std::array<bool, 3> ty_seen{};
        std::array<bool, 3> pt_seen{};
        std::array<bool, 3> ctr_seen{};
        Vec3 tx{};
        Vec3 ty{};
        Vec3 pt{};
        Vec3 ctr{};
    };
    std::array<SideAcc, 2> acc{};

    for (std::size_t i = 0; i < names.size(); ++i) {
        const std::string &name = names[i];
        const double v = row[i];
        const std::size_t d1 = name.find('.');
        const std::size_t d2 =
            d1 == std::string::npos ? std::string::npos : name.find('.', d1 + 1);
        const std::string side_s = name.substr(0, d1);
        const std::string field =
            d1 == std::string::npos
                ? ""
                : name.substr(d1 + 1, d2 == std::string::npos ? std::string::npos
                                                              : d2 - d1 - 1);
        const std::string sub =
            d2 == std::string::npos ? "" : name.substr(d2 + 1);

        if (side_s != "front" && side_s != "rear") {
            throw std::invalid_argument("set_tire_state: unknown column '" +
                                        name + "'");
        }
        const std::size_t s = (side_s == "front") ? 0 : 1;
        if (d2 != std::string::npos &&
            (sub.size() != 1 || sub[0] < '0' || sub[0] > '2' ||
             field == "ux" || field == "uy")) {
            throw std::invalid_argument("set_tire_state: malformed column '" +
                                        name + "'");
        }
        if (field == "ux") {
            if (acc[s].ux_seen) {
                throw std::invalid_argument(
                    "set_tire_state: duplicate column '" + name + "'");
            }
            acc[s].ux_seen = true;
            acc[s].ux = v;
        } else if (field == "uy") {
            if (acc[s].uy_seen) {
                throw std::invalid_argument(
                    "set_tire_state: duplicate column '" + name + "'");
            }
            acc[s].uy_seen = true;
            acc[s].uy = v;
        } else {
            std::array<bool, 3> *seen = nullptr;
            Vec3 *vec = nullptr;
            if (field == "tx") {
                seen = &acc[s].tx_seen;
                vec = &acc[s].tx;
            } else if (field == "ty") {
                seen = &acc[s].ty_seen;
                vec = &acc[s].ty;
            } else if (field == "point") {
                seen = &acc[s].pt_seen;
                vec = &acc[s].pt;
            } else if (field == "center") {
                seen = &acc[s].ctr_seen;
                vec = &acc[s].ctr;
            } else {
                throw std::invalid_argument(
                    "set_tire_state: unknown column '" + name + "'");
            }
            if (sub.empty()) {
                if (!std::isnan(v)) {
                    throw std::invalid_argument(
                        "set_tire_state: non-scalar leaf value in '" + name +
                        "'");
                }
                continue;
            }
            const auto idx = static_cast<std::size_t>(sub[0] - '0');
            if ((*seen)[idx]) {
                throw std::invalid_argument(
                    "set_tire_state: duplicate column '" + name + "'");
            }
            (*seen)[idx] = true;
            (*vec)[idx] = v;
        }
    }

    std::array<BrushState3d, 2> next{};
    for (std::size_t s = 0; s < 2; ++s) {
        const SideAcc &a = acc[s];
        bool all_nan = std::isnan(a.ux) && std::isnan(a.uy);
        bool all_finite = std::isfinite(a.ux) && std::isfinite(a.uy);
        for (std::size_t k = 0; k < 3; ++k) {
            all_nan = all_nan && std::isnan(a.tx[k]) && std::isnan(a.ty[k]) &&
                      std::isnan(a.pt[k]) && std::isnan(a.ctr[k]);
            all_finite = all_finite && std::isfinite(a.tx[k]) &&
                         std::isfinite(a.ty[k]) && std::isfinite(a.pt[k]) &&
                         std::isfinite(a.ctr[k]);
        }
        if (all_nan) {
            next[s] = BrushState3d{};
        } else if (all_finite) {
            if (!(norm3(a.tx) > 1e-6) || !(norm3(a.ty) > 1e-6)) {
                throw std::invalid_argument(
                    "set_tire_state: active tx/ty basis vectors must be "
                    "nonzero");
            }
            next[s] = BrushState3d{
                .active = true,
                .ux = a.ux,
                .uy = a.uy,
                .tx = a.tx,
                .ty = a.ty,
                .point = a.pt,
                .center = a.ctr,
            };
        } else {
            throw std::invalid_argument(
                "set_tire_state: per-side fields must be all-NaN (inactive) or "
                "all-finite (active)");
        }
    }
    prepared_states_ = next;
}

void Compliant3dTireWriter::commit_prepared_state() noexcept {
    states_ = prepared_states_;
    last_time_s_.reset();
    std::ranges::fill(qfrc_dissipative_, 0.0);
    dissipative_power_start_w_ = 0.0;
    brush_loss_step_j_ = 0.0;
    radial_dissipation_power_w_ = 0.0;
}

std::vector<double> Compliant3dTireWriter::state() const {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> out;
    out.reserve(28);
    const auto push_vec = [&out](const Vec3 &v) {
        for (const double c : v) out.push_back(c);
    };
    for (const BrushState3d &st : states_) {
        if (!st.active) {
            for (std::size_t i = 0; i < 14; ++i) out.push_back(nan_v);
        } else {
            out.push_back(st.ux);
            out.push_back(st.uy);
            push_vec(st.tx);
            push_vec(st.ty);
            push_vec(st.point);
            push_vec(st.center);
        }
    }
    return out;
}

void Compliant3dTireWriter::compute_into(const mjData *d, double dt,
                                         std::span<double> out) {
    const char *err_msg = nullptr;
    bool overflow_err = false;
    const CoreStatus status =
        accumulate_core(d, dt, out, err_msg, overflow_err);
    if (status == CoreStatus::ok) return;
    if (status == CoreStatus::invalid_geometry) {
        throw InvalidGeometryError(
            err_msg != nullptr ? err_msg
                               : "terrain3d closest_point outside terrain bounds");
    }
    if (overflow_err) {
        throw std::overflow_error(err_msg != nullptr ? err_msg
                                                     : "tire overflow");
    }
    throw std::invalid_argument(err_msg != nullptr ? err_msg
                                                   : "tire compute failed");
}

CoreStatus Compliant3dTireWriter::try_compute_into(
    const mjData *d, double dt, std::span<double> out) noexcept {
    const char *err_msg = nullptr;
    bool overflow_err = false;
    try {
        return accumulate_core(d, dt, out, err_msg, overflow_err);
    } catch (...) {
        return CoreStatus::engine_failure;
    }
}

std::vector<double> Compliant3dTireWriter::qfrc(const mjData *d, double dt) {
    compute_into(d, dt, qfrc_out_);
    return qfrc_out_;
}

CoreStatus Compliant3dTireWriter::accumulate_core(const mjData *d, double dt,
                                                  std::span<double> out,
                                                  const char *&err_msg,
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
    if (!std::isfinite(dt) || dt <= 0.0) {
        err_msg = "invalid tire interval";
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
    if (!try_interval_id(time, dt, interval_id, err_msg, overflow_err) ||
        interval_id < 0) {
        if (err_msg == nullptr) {
            err_msg = "contact snapshot needs a nonnegative interval ID";
        }
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

    const double normal_cosine =
        std::cos(cfg_.distinct_normal_deg * (std::numbers::pi / 180.0));
    const bool track_mode = (cfg_.surface_mode == "track");

    double energy = 0.0;
    double loss = 0.0;
    double radial_loss_power = 0.0;
    std::array<BrushState3d, 2> new_states{};

    for (std::size_t s = 0; s < 2; ++s) {
        const Compliant3dWheelParams &p = (s == 0) ? cfg_.front : cfg_.rear;
        const BrushState3d &prev = states_[s];
        const int geom = geoms_[s];
        const int body = bodies_[s];
        const std::size_t g3 = static_cast<std::size_t>(model_access::kXYZ) *
                               static_cast<std::size_t>(geom);
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

        CrownContact contact{};
        if (const CoreStatus st = find_crown_contact(
                cfg_.terrain, {.p_hub = p_hub, .a_hat = a_hat}, p, contact,
                err_msg);
            st != CoreStatus::ok) {
            return st;
        }
        if (!std::isfinite(contact.delta)) {
            err_msg = "tire deflection must be finite";
            return CoreStatus::invalid_input;
        }

        if (!try_point_jacobian_into(m_, d, body, contact.point, jac_contact_,
                                     err_msg) ||
            !try_point_jacobian_into(m_, d, body, p_hub, jac_center_,
                                     err_msg)) {
            return CoreStatus::invalid_input;
        }

        Vec3 v_c{};
        Vec3 v_hub{};
        if (nv_ > 0) {
            blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                        model_access::kXYZ, nv_, 1.0, jac_contact_.data(), nv_,
                        qvel.data(), 1, 0.0, v_c.data(), 1);
            blas::dgemv(blas::Order::row_major, blas::Transpose::no,
                        model_access::kXYZ, nv_, 1.0, jac_center_.data(), nv_,
                        qvel.data(), 1, 0.0, v_hub.data(), 1);
        }

        const double delta_dot = -dot3(v_c, contact.normal);
        Vec3 tx{};
        Vec3 ty{};
        if (!try_build_road_basis(a_hat, contact.normal, tx, ty)) {
            err_msg = "road tangent basis degenerate";
            return CoreStatus::invalid_input;
        }

        const double vsx = dot3(v_c, tx);
        const double vsy = dot3(v_c, ty);
        const double v_s = std::hypot(vsx, vsy);
        const double v_roll = std::abs(dot3(v_hub, tx));

        double mu = 0.0;
        double c_kappa_per_fz = cfg_.configured_c_kappa_per_fz;
        std::string_view surface_name = "configured";
        if (track_mode) {
            const SurfaceEntry surf = surface_entry(contact.surface);
            const double decay = std::exp(-v_s / surf.stribeck_speed_mps);
            const double mu_surf =
                surf.mu_slide + (surf.mu_peak - surf.mu_slide) * decay;
            mu = std::min(p.mu_cap, mu_surf);
            c_kappa_per_fz = surf.c_kappa_per_fz;
            surface_name = surf.name;
        } else {
            mu = cfg_.configured_mu
                     ? std::min(p.mu_cap, *cfg_.configured_mu)
                     : p.mu_cap;
        }

        const double kbx = p.k_brush_x_n_m;
        const double kby = p.k_brush_y_n_m;
        const double u_old =
            prev.active ? 0.5 * (kbx * prev.ux * prev.ux +
                                 kby * prev.uy * prev.uy)
                        : 0.0;

        double f_el = 0.0;
        double u_rad = 0.0;
        double fn = 0.0;
        double rad_diss_side = 0.0;
        double release_loss = 0.0;
        double brush_loss = 0.0;
        double ux_new = 0.0;
        double uy_new = 0.0;
        double fx = 0.0;
        double fy = 0.0;
        double alpha_sat = 1.0;

        if (!(contact.delta > 0.0)) {
            release_loss = u_old;
            new_states[s] = BrushState3d{};
        } else {
            f_el = elastic_normal_force(p, contact.delta);
            u_rad = radial_stored_energy(p, contact.delta);
            const double hyst =
                1.0 + p.loss_factor *
                          std::tanh(delta_dot / p.hysteresis_rate_eps_mps);
            fn = std::max(0.0, f_el * hyst);
            rad_diss_side = std::max(0.0, (fn - f_el) * delta_dot);

            double ux0 = 0.0;
            double uy0 = 0.0;
            double u0_energy = 0.0;
            if (prev.active) {
                const Vec3 n_prev = cross3(prev.tx, prev.ty);
                const bool similar = (dot3(n_prev, contact.normal) >= normal_cosine);
                const double dp = norm3(sub3(contact.point, prev.point));
                const double dhub = norm3(sub3(p_hub, prev.center));
                const double dbasis = norm3(sub3(tx, prev.tx)) +
                                      norm3(sub3(ty, prev.ty));
                const bool continuous =
                    (dp < 1e-8) ||
                    (similar &&
                     dp <= 2.0 * dhub + p.outer_radius_m * dbasis + 1e-4);
                if (!continuous) {
                    release_loss = u_old;
                } else {
                    const Vec3 w_prev = {
                        prev.ux * prev.tx[0] + prev.uy * prev.ty[0],
                        prev.ux * prev.tx[1] + prev.uy * prev.ty[1],
                        prev.ux * prev.tx[2] + prev.uy * prev.ty[2],
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
                    release_loss = std::max(0.0, u_old - u0_energy);
                }
            }

            if (!(fn > 1e-12) || !(mu > 0.0)) {
                brush_loss = u0_energy;
                new_states[s] = BrushState3d{};
            } else {
                const double sigma_x = (c_kappa_per_fz * fn) / kbx;
                const double sigma_y = (p.c_alpha_per_fz * fn) / kby;
                const double rx = dt * (v_roll / sigma_x);
                const double ry = dt * (v_roll / sigma_y);
                const double ux_trial = (ux0 + dt * vsx) / (1.0 + rx);
                const double uy_trial = (uy0 + dt * vsy) / (1.0 + ry);
                const double fx_trial = -kbx * ux_trial;
                const double fy_trial = -kby * uy_trial;
                const double ft_trial = std::hypot(fx_trial, fy_trial);
                const double f_max = mu * fn;
                alpha_sat = (ft_trial > f_max && ft_trial > 0.0)
                                ? (f_max / ft_trial)
                                : 1.0;
                ux_new = alpha_sat * ux_trial;
                uy_new = alpha_sat * uy_trial;
                fx = -kbx * ux_new;
                fy = -kby * uy_new;

                const double dux = ux_new - ux0;
                const double duy = uy_new - uy0;
                const double l_implicit =
                    0.5 * (kbx * dux * dux + kby * duy * duy);
                const double l_relax =
                    kbx * rx * ux_new * ux_new + kby * ry * uy_new * uy_new;
                const double l_proj =
                    (1.0 - alpha_sat) * alpha_sat *
                    (kbx * (1.0 + rx) * ux_trial * ux_trial +
                     kby * (1.0 + ry) * uy_trial * uy_trial);
                brush_loss = std::max(0.0, l_implicit) +
                             std::max(0.0, l_relax) +
                             std::max(0.0, l_proj);

                new_states[s] = BrushState3d{
                    .active = true,
                    .ux = ux_new,
                    .uy = uy_new,
                    .tx = tx,
                    .ty = ty,
                    .point = contact.point,
                    .center = p_hub,
                };
            }
        }

        const Vec3 force_world = {
            fn * contact.normal[0] + fx * tx[0] + fy * ty[0],
            fn * contact.normal[1] + fx * tx[1] + fy * ty[1],
            fn * contact.normal[2] + fx * tx[2] + fy * ty[2],
        };
        const double fn_diss = fn - f_el;
        const Vec3 f_diss_world = {
            fn_diss * contact.normal[0] + fx * tx[0] + fy * ty[0],
            fn_diss * contact.normal[1] + fx * tx[1] + fy * ty[1],
            fn_diss * contact.normal[2] + fx * tx[2] + fy * ty[2],
        };

        if (contact.delta > 0.0 && nv_ > 0) {
            blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                        model_access::kXYZ, nv_, 1.0, jac_contact_.data(), nv_,
                        force_world.data(), 1, 0.0, tmp_.data(), 1);
            for (int i = 0; i < nv_; ++i) {
                const auto iu = static_cast<std::size_t>(i);
                out[iu] += tmp_[iu];
            }

            blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                        model_access::kXYZ, nv_, 1.0, jac_contact_.data(), nv_,
                        f_diss_world.data(), 1, 0.0, tmp_.data(), 1);
            for (int i = 0; i < nv_; ++i) {
                const auto iu = static_cast<std::size_t>(i);
                qfrc_diss_stage_[iu] += tmp_[iu];
            }
        }

        TireSnapshot &snap = snapshot_stage_[s];
        snap.patches.clear();
        snap.time_s = time;
        snap.backend.assign(kBackendName.data(), kBackendName.size());
        snap.geometric_contact = (contact.delta > 0.0);
        snap.wheel_axis_m = p_hub;
        if (contact.delta > 0.0) {
            if (snap.patches.size() >= kMaxSnapshotPatches) {
                err_msg = "tire snapshot patch capacity exceeded";
                return CoreStatus::invalid_input;
            }
            TirePatch patch{};
            if (!try_fill_patch(
                    contact.point, contact.normal,
                    {.normal_load_n = fn,
                     .tangent_force_n = fx,
                     .slip_mps = v_s},
                    force_world, patch)) {
                err_msg = "invalid contact patch values";
                return CoreStatus::invalid_input;
            }
            snap.patches.push_back(patch);
        }
        snap.interval_id = interval_id;
        snap.effective_radius_m = effective_radius(snap.patches, p_hub);

        const double u_shear_new =
            new_states[s].active
                ? 0.5 * (kbx * ux_new * ux_new + kby * uy_new * uy_new)
                : 0.0;
        const double side_brush_loss =
            std::max(release_loss, 0.0) + brush_loss;

        radial_loss_power += rad_diss_side;
        energy += u_rad + u_shear_new;
        loss += side_brush_loss;

        TireDiagnostics &diag = diagnostics_stage_[s];
        diag.multi_support = false;
        diag.penetration_m = contact.delta;
        diag.normal_speed_mps = -delta_dot;
        diag.slip_mps = v_s;
        diag.normal_load_n = fn;
        diag.tangent_force_n = fx;
        diag.friction_coefficient = mu;
        diag.surface.assign(surface_name.data(), surface_name.size());
        diag.branch_release_loss_j = std::max(release_loss, 0.0);
        diag.brush_loss_j = brush_loss;
        diag.radial_energy_j = u_rad;
        diag.shear_energy_j = u_shear_new;
        diag.outside_material_load_range =
            (fn < p.valid_load_range_n[0] || fn > p.valid_load_range_n[1]);
        diag.pressure_bar = p.pressure_bar;
        diag.airborne = !(contact.delta > 0.0);
        diag.rim_strike_active =
            (contact.delta > p.rim_strike_deflection_m());
        diag.patch_count = (contact.delta > 0.0) ? 1 : 0;
        diag.fully_sliding = (contact.delta > 0.0 && alpha_sat < 1.0);
        diag.dissipated_power_w = rad_diss_side + side_brush_loss / dt;
    }

    for (const double v : out) {
        if (!std::isfinite(v)) {
            err_msg = "Compliant3dTireWriter.qfrc";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    for (const double v : qfrc_diss_stage_) {
        if (!std::isfinite(v)) {
            err_msg = "Compliant3dTireWriter.qfrc_dissipative";
            overflow_err = true;
            return CoreStatus::invalid_input;
        }
    }
    if (!std::isfinite(energy) || !std::isfinite(loss) ||
        !std::isfinite(radial_loss_power)) {
        err_msg = "Compliant3dTireWriter.energy_or_power";
        overflow_err = true;
        return CoreStatus::invalid_input;
    }
    const double diss_power_start =
        (nv_ > 0)
            ? -blas::ddot(nv_, qfrc_diss_stage_.data(), 1, qvel.data(), 1)
            : 0.0;
    if (!std::isfinite(diss_power_start)) {
        err_msg = "Compliant3dTireWriter.dissipative_power_start_w";
        overflow_err = true;
        return CoreStatus::invalid_input;
    }

    states_ = new_states;
    for (std::size_t s = 0; s < 2; ++s) {
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

double Compliant3dTireWriter::stored_energy(const mjData *d) const {
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
        const Compliant3dWheelParams &p = (s == 0) ? cfg_.front : cfg_.rear;
        const BrushState3d &st = states_[s];
        const std::size_t g3 = static_cast<std::size_t>(model_access::kXYZ) *
                               static_cast<std::size_t>(geoms_[s]);
        const std::size_t b9 =
            static_cast<std::size_t>(model_access::kRotationElements) *
            static_cast<std::size_t>(bodies_[s]);

        const Vec3 p_hub = {xpos[g3], xpos[g3 + 1], xpos[g3 + 2]};
        const Vec3 a_raw = {xmat[b9 + 1], xmat[b9 + 4], xmat[b9 + 7]};
        Vec3 a_hat{};
        normalize_unit_axis(a_raw, a_hat, "wheel body spin axis");

        CrownContact contact{};
        const char *err_msg = nullptr;
        const CoreStatus st_crown = find_crown_contact(
            cfg_.terrain, {.p_hub = p_hub, .a_hat = a_hat}, p, contact,
            err_msg);
        if (st_crown == CoreStatus::invalid_geometry) {
            throw InvalidGeometryError(
                err_msg != nullptr
                    ? err_msg
                    : "terrain3d closest_point outside terrain bounds");
        }
        if (st_crown != CoreStatus::ok) {
            throw std::invalid_argument(
                err_msg != nullptr ? err_msg : "invalid crown contact");
        }
        if (!std::isfinite(contact.delta)) {
            throw std::invalid_argument("tire deflection must be finite");
        }
        const double u_rad = radial_stored_energy(p, contact.delta);
        const double u_shear =
            st.active ? 0.5 * (p.k_brush_x_n_m * st.ux * st.ux +
                               p.k_brush_y_n_m * st.uy * st.uy)
                      : 0.0;
        validation::derived(u_rad, "Compliant3dTireWriter.radial_energy");
        validation::derived(u_shear, "Compliant3dTireWriter.shear_energy");
        energy += u_rad + u_shear;
    }
    return validation::derived(energy, "Compliant3dTireWriter.stored_energy");
}

void Compliant3dTireWriter::probe_compute_into(const mjData *d, double dt,
                                               std::span<double> out) {
    if (d == nullptr) {
        throw std::invalid_argument("tire probe needs a live mjData");
    }
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty())) {
        throw std::invalid_argument("tire output width");
    }
    Compliant3dTireWriter probe(*this);
    probe.last_time_s_.reset();
    probe.compute_into(d, dt, out);
    probe_snapshots_ = probe.snapshots_;
    probe_committed_ = probe.committed_;
}

std::vector<double> Compliant3dTireWriter::probe_qfrc(const mjData *d,
                                                      double dt) {
    std::vector<double> out(static_cast<std::size_t>(nv_), 0.0);
    probe_compute_into(d, dt, out);
    return out;
}

double Compliant3dTireWriter::step_loss_j(std::span<const double> qvel_mid,
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
