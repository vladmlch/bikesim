// tyre/rut_memory.cpp — Stage B 1D rut memory. See rut_memory.hpp for
// the scheme; the node update is the return-mapping of research 06 §1.2
// with the conjugate hat scatter for exact virtual work.
#include "tyre/rut_memory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "../pneumatic/carcass.hpp"   // ray_elastic_force_n
#include "../pneumatic/geometry.hpp"  // RayRing, RayHitBuffers
#include "../pneumatic/profile.hpp"   // RoadProfile

namespace biketyre {
namespace {

constexpr double kGravity = 9.81;
constexpr double kBekkerToeEpsM = 1e-4; // 0.1 mm grain-scale C1 toe regularization
constexpr std::size_t kSolveIterations = 24;

// Smoothstep S(u) = u^2(3-2u) on [0,1], clamped — the same C1 Hermite
// the surface-transition blend uses.
[[nodiscard]] double smoothstep(double u) noexcept {
    const double t = std::clamp(u, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// Section weight at x: smoothstep collars of half-width blend_m/2 on
// each edge (the same collar geometry for_each_transition walks). When
// extend_left is true (sections[0].start_m == 0.0), the section covers
// x < 0 without a left collar, matching tire_common.cpp.
[[nodiscard]] double section_weight(const nativecfg::SurfaceSection &s,
                                    double x, bool extend_left) noexcept {
    double wl = 1.0, wr = 1.0;
    if (s.blend_m > 0.0) {
        const double half = 0.5 * s.blend_m;
        wl = extend_left ? 1.0
                         : smoothstep((x - (s.start_m - half)) / s.blend_m);
        wr = 1.0 - smoothstep((x - (s.end_m - half)) / s.blend_m);
    } else {
        wl = (extend_left || x >= s.start_m) ? 1.0 : 0.0;
        wr = (x < s.end_m) ? 1.0 : 0.0;
    }
    return std::min(wl, wr);
}

[[nodiscard]] SoilLayerParams mix_params(const SoilLayerParams &a,
                                         const SoilLayerParams &b,
                                         double wa, double wb) noexcept {
    const double w = wa + wb;
    if (!(w > 0.0)) return a;
    const auto lerp = [&](double pa, double pb) { return (wa * pa + wb * pb) / w; };
    return {.k0_pa = lerp(a.k0_pa, b.k0_pa), .n = lerp(a.n, b.n),
            .zeta_o = lerp(a.zeta_o, b.zeta_o), .zeta_w = lerp(a.zeta_w, b.zeta_w),
            .b_hp_pa = lerp(a.b_hp_pa, b.b_hp_pa), .c_hp = lerp(a.c_hp, b.c_hp),
            .beta_u = lerp(a.beta_u, b.beta_u), .mu_basal = lerp(a.mu_basal, b.mu_basal),
            .c_bed_pa = lerp(a.c_bed_pa, b.c_bed_pa),
            .rho_b_kg_m3 = lerp(a.rho_b_kg_m3, b.rho_b_kg_m3),
            .alpha_tau = lerp(a.alpha_tau, b.alpha_tau),
            .alpha_n = lerp(a.alpha_n, b.alpha_n)};
}

// 16-point Gauss-Legendre on [-1,1] — deterministic, allocation-free.
// Used only inside commit_pass's per-node work integral.
struct Gauss16 {
    static constexpr std::array<double, 8> x = {
            0.09501250983763744, 0.2816035507792589, 0.4580167776572274,
            0.6178762444026438, 0.755404408355003, 0.8656312023878318,
            0.9445750230732326, 0.9894009349916499};
    static constexpr std::array<double, 8> w = {
            0.1894506104550685, 0.18260341504492358, 0.16915651939500254,
            0.14959598881657674, 0.12462897125553388, 0.09515851168249279,
            0.0622535239386479, 0.027152459411754095};
};

// Illinois-style regula falsi on a strictly monotone function with a
// sign-change bracket; deterministic fixed iteration count with a
// bisection guard so the bracket always stays live.
[[nodiscard]] double solve_bracketed(double lo, double hi, double f_lo,
                                     double f_hi, const auto &fn) noexcept {
    double x = 0.5 * (lo + hi);
    for (std::size_t it = 0; it < kSolveIterations; ++it) {
        x = hi - f_hi * (hi - lo) / (f_hi - f_lo);
        if (!(x > lo && x < hi)) x = 0.5 * (lo + hi); // degenerate guard
        const double f_x = fn(x);
        if (f_x == 0.0) return x;
        if (f_x > 0.0) { hi = x; f_hi = f_x; f_lo *= 0.5; }
        else { lo = x; f_lo = f_x; f_hi *= 0.5; }
    }
    return x;
}

// The C1 finite-depth virgin curve p_v(z) [Pa] and its slope [Pa/m]
// for a fully-resolved (thickness-blended) parameter set on a layer of
// thickness h under contact half-width b.
struct PressureLaw {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) read-only view borrows the resolved per-node params
    const SoilLayerParams &p;
    double h, b;
    [[nodiscard]] double operator()(double z) const noexcept {
        if (!(h > 0.0)) return 0.0;
        const double zz = std::max(0.0, z);
        const double zo = p.zeta_o * h;
        const double over = std::max(0.0, zz - zo);
        const double ramp = smoothstep((zz - zo) / (p.zeta_w * h));
        const double base = p.k0_pa * (std::pow((zz + kBekkerToeEpsM) / b, p.n) -
                                       std::pow(kBekkerToeEpsM / b, p.n));
        const double barrier = (ramp > 0.0 && over > 0.0)
                ? p.b_hp_pa * ramp * (std::exp(p.c_hp * over / b) - 1.0)
                : 0.0;
        return base + barrier;
    }
    [[nodiscard]] double slope(double z) const noexcept {
        if (!(h > 0.0)) return 0.0;
        const double zz = std::max(0.0, z);
        const double zo = p.zeta_o * h;
        const double over = std::max(0.0, zz - zo);
        const double u = (zz - zo) / (p.zeta_w * h);
        const double s = smoothstep(u), t = std::clamp(u, 0.0, 1.0);
        const double ds = (u > 0.0 && u < 1.0)
                ? 6.0 * t * (1.0 - t) / (p.zeta_w * h) : 0.0;
        double barrier_slope = 0.0;
        if (over > 0.0) {
            const double ex = std::exp(p.c_hp * over / b);
            const double de = (p.c_hp / b) * ex;
            barrier_slope = p.b_hp_pa *
                    ((ds != 0.0 ? ds * (ex - 1.0) : 0.0) + s * de);
        }
        const double base =
                p.k0_pa * p.n * std::pow((zz + kBekkerToeEpsM) / b, p.n - 1.0) / b;
        return base + barrier_slope;
    }
};

} // namespace

const SoilLayerParams &soil_params_for(std::string_view name) noexcept {
    // Authored Stage B constants (Spec §4.1, user decision 2026-11-23):
    // loose gravel over hardpack vs loose scree over bedrock — scree is
    // softer in the virgin branch yet stiffer at the bedrock barrier and
    // slips basally on a lower bed friction.
    static const SoilLayerParams kLoose{
        .k0_pa = 180e3, .n = 0.80, .zeta_o = 0.45, .zeta_w = 0.30,
        .b_hp_pa = 500e3, .c_hp = 4.5, .beta_u = 6.0,
        .mu_basal = 0.52, .c_bed_pa = 0.0, .rho_b_kg_m3 = 1650.0,
        .alpha_tau = 1.0, .alpha_n = 0.55};
    static const SoilLayerParams kScree{
        .k0_pa = 130e3, .n = 0.85, .zeta_o = 0.40, .zeta_w = 0.30,
        .b_hp_pa = 650e3, .c_hp = 5.0, .beta_u = 5.0,
        .mu_basal = 0.42, .c_bed_pa = 0.0, .rho_b_kg_m3 = 1750.0,
        .alpha_tau = 1.0, .alpha_n = 0.45};
    if (name == "scree") return kScree;
    return kLoose; // the deformable domain is exactly {loose, scree}
}

bool is_deformable_surface(std::string_view name) noexcept {
    return name == "loose" || name == "scree";
}

RutMemory::RutMemory(const RoadProfile &road, const nativecfg::SurfaceMap &map)
    : n_(road.size()), x0_m_(road.x0_m()), dx_m_(road.dx_m()) {
    h_s_m_.assign(n_, 0.0);
    z_max_m_.assign(n_, 0.0);
    z_p_m_.assign(n_, 0.0);
    z_e_m_.assign(n_, 0.0);
    k_u_.assign(n_, 0.0);
    force_n_.assign(n_, 0.0);
    grade_.assign(n_, 0.0);
    params_.assign(n_, SoilLayerParams{});
    // The map's default material contributes the base layer wherever
    // the sections leave weight; a configured uniform loose/scree
    // surface lands here as a whole-track layer.
    const bool base_deformable =
        is_deformable_surface(map.surface.name) &&
        map.surface.layer_thickness_m > 0.0;
    const auto x = road.x_m(), z = road.z_m();
    for (std::size_t g = 0; g < n_; ++g) {
        const std::size_t gm = g == 0 ? 0 : g - 1;
        const std::size_t gp = g + 1 < n_ ? g + 1 : n_ - 1;
        grade_[g] = std::atan((z[gp] - z[gm]) / (x[gp] - x[gm]));
        double h = 0.0, w_acc = 0.0, w_sum = 0.0;
        SoilLayerParams p{};
        for (std::size_t si = 0; si < map.sections.size(); ++si) {
            const auto &sec = map.sections[si];
            const bool extend_left = (si == 0 && sec.start_m == 0.0);
            const double w = section_weight(sec, x[g], extend_left);
            if (w <= 0.0) continue;
            w_sum += w;
            if (!is_deformable_surface(sec.surface.name) ||
                sec.layer_thickness_m <= 0.0)
                continue;
            const double wt = w * sec.layer_thickness_m;
            h += wt;
            p = mix_params(p, soil_params_for(sec.surface.name), w_acc, wt);
            w_acc += wt;
        }
        if (base_deformable) {
            // The default material fills the section-free remainder —
            // the same fallback surface_at() applies.
            const double w = std::max(0.0, 1.0 - w_sum);
            if (w > 0.0) {
                const double wt = w * map.surface.layer_thickness_m;
                h += wt;
                p = mix_params(p, soil_params_for(map.surface.name),
                               w_acc, wt);
            }
        }
        h_s_m_[g] = h;
        params_[g] = p;
        if (h > 0.0) active_ = true;
    }
    scratch_f_.assign(n_, 0.0);
    scratch_zt_.assign(n_, 0.0);
    scratch_zw_.assign(n_, 0.0);
    covered_.assign(n_, 0);
    in_loaded_.assign(n_, 0);
    // Full-grid capacity: apply_diff's noexcept push_back and the
    // warm commit's node list can then never reallocate.
    nodes_idx_.reserve(n_);
    loaded_.reserve(n_);
}

std::size_t RutMemory::node_of(double x_m) const noexcept {
    const double u = (x_m - x0_m_) / dx_m_;
    const auto g = static_cast<std::int64_t>(std::floor(u));
    return static_cast<std::size_t>(
            std::clamp<std::int64_t>(g, 0, static_cast<std::int64_t>(n_) - 2));
}

double RutMemory::pressure_virgin(std::size_t g, double z, double b_m) const noexcept {
    return PressureLaw{.p = params_[g], .h = h_s_m_[g], .b = b_m}(z);
}

double RutMemory::pressure_virgin_slope(std::size_t g, double z, double b_m) const noexcept {
    return PressureLaw{.p = params_[g], .h = h_s_m_[g], .b = b_m}.slope(z);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) node index then (z, width, z_max, z_p) mirror the nodal state tuple
double RutMemory::pressure_on_path(std::size_t g, double z, double b_m,
                                   double z_max, double z_p) const noexcept {
    if (z > z_max) return pressure_virgin(g, z, b_m);
    const PressureLaw law{.p = params_[g], .h = h_s_m_[g], .b = b_m};
    const double k_u_pa = params_[g].beta_u * law.slope(z_max); // [Pa/m]
    return k_u_pa * std::max(0.0, z - z_p);
}

void RutMemory::solve_deflections(const RayRing &ring,
                                  const nativecfg::PneumaticTyreSpec &tyre,
                                  double ds_m, RayHitBuffers &hits) const {
    const double b_m = 0.5 * tyre.casing_width_mm * 0.001;
    const double area_e = 2.0 * b_m * ds_m; // element area B*ds [m^2]
    for (std::size_t i = 0; i < ring.n(); ++i) {
        const double delta = hits.delta_m[i];
        if (!(delta > 0.0) || !std::isfinite(hits.road_x_m[i])) continue;
        const double x = hits.road_x_m[i];
        const double u = (x - x0_m_) / dx_m_;
        if (u < 0.0 || u >= static_cast<double>(n_ - 1)) continue;
        const std::size_t g = node_of(x);
        const double a = u - std::floor(u);
        const double h = (1.0 - a) * h_s_m_[g] + a * h_s_m_[g + 1];
        if (!(h > 0.0)) continue;
        const double zp = (1.0 - a) * z_p_m_[g] + a * z_p_m_[g + 1];
        const double zm = (1.0 - a) * z_max_m_[g] + a * z_max_m_[g + 1];
        const SoilLayerParams p = mix_params(params_[g], params_[g + 1], 1.0 - a, a);
        const PressureLaw law{.p = p, .h = h, .b = b_m};
        const double slope_m = law.slope(zm);
        const double ku = std::max(0.0, p.beta_u * slope_m * area_e);
        const double z_hi = std::min(delta, h);
        // Load-path soil element force at depression z: the ray
        // equilibrates its elastic force against the gathered nodal
        // (z_max, z_p) surface — the committed z_eq this produces is
        // what commit_pass reproduces on the nodes. The residual is
        // soil-minus-tire (monotone increasing in z) because
        // solve_bracketed's Illinois update assumes that ordering.
        const auto soil = [&](double zz) {
            return zz > zm ? area_e * law(zz)
                           : ku * std::max(0.0, zz - zp);
        };
        const auto residual = [&](double zz) {
            return soil(zz) - ray_elastic_force_n(delta - zz, tyre, ds_m);
        };
        const double r0 = residual(0.0), r1 = residual(z_hi);
        double z = 0.0;
        if (!(r0 < 0.0)) z = 0.0;      // soil(0) >= tire(delta): rigid
        else if (!(r1 > 0.0)) z = z_hi; // soil never catches the tire
        else z = solve_bracketed(0.0, z_hi, r0, r1, residual);
        hits.delta_m[i] = delta - z;
    }
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) parallel span rows mirror the committed ray-element buffers
void RutMemory::commit_pass(const RayRing &ring,
                            const nativecfg::PneumaticTyreSpec &tyre,
                            std::span<const double> element_force_n,
                            std::span<const double> road_x_m,
                            std::span<const double> soil_z_eq_m) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    const double b_m = 0.5 * tyre.casing_width_mm * 0.001;
    const double area_g = 2.0 * b_m * dx_m_; // node tributary area B*dx
    std::ranges::fill(scratch_f_, 0.0);
    std::ranges::fill(scratch_zt_, 0.0);
    std::ranges::fill(scratch_zw_, 0.0);
    // covered_ accumulates across the whole step (front + rear);
    // begin_step() cleared it — this pass only ORs its window in.
    nodes_idx_.clear();
    // Each contacting ray element presses soil over its whole arc
    // length — several node spacings at the 64-ray tier — so coverage
    // for the release pass is the element footprint interval, not the
    // two hat nodes the force scatter writes. Point-sampled coverage
    // would flicker as element forces evolve: a node between ray
    // footprints would read as uncovered, release its elastic depth
    // as loss, and reload it next step — a fake plastic term that
    // scales with the stored elastic energy.
    const double half_foot_m =
            0.5 * tyre.outer_radius_mm * 0.001 * ring.dtheta_rad();
    // Conjugate hat scatter: F_g = sum_i w_ig * f_i (exact virtual work).
    for (std::size_t i = 0; i < ring.n(); ++i) {
        const double f = std::max(0.0, element_force_n[i]);
        const double x = road_x_m[i];
        if (!std::isfinite(x) || !(f > 0.0)) continue;
        const double u = (x - x0_m_) / dx_m_;
        if (u < 0.0 || u >= static_cast<double>(n_ - 1)) continue;
        const std::size_t g = node_of(x);
        const double a = u - std::floor(u);
        if (!(h_s_m_[g] > 0.0) && !(h_s_m_[g + 1] > 0.0)) continue;
        for (const std::size_t gg : {g, g + 1}) {
            if (covered_[gg] == 0) {
                covered_[gg] = 1;
                nodes_idx_.push_back(static_cast<std::uint32_t>(gg));
            }
        }
        const std::size_t fa = node_of(x - half_foot_m);
        const std::size_t fb = node_of(x + half_foot_m);
        for (std::size_t gg = fa; gg <= fb; ++gg) covered_[gg] = 1;
        scratch_f_[g] += (1.0 - a) * f;
        scratch_f_[g + 1] += a * f;
    }
    // Hat gather of the ray-equilibrium depression z_eq: the committed
    // nodal total (z_p + z_e) must reproduce the soil surface the rays
    // actually equilibrated against, or the tire's effective deflection
    // and the force×displacement bookkeeping disagree by the ray/node
    // sampling gap. scratch_zt_/scratch_zw_ hold the weighted sum and
    // the hat weight so zt_g = zt/zw is the local depression estimate.
    const bool have_zeq = soil_z_eq_m.size() >= ring.n();
    if (have_zeq) {
        for (std::size_t i = 0; i < ring.n(); ++i) {
            const double z = soil_z_eq_m[i];
            const double x = road_x_m[i];
            if (!std::isfinite(x) || !(z > 0.0)) continue;
            const double u = (x - x0_m_) / dx_m_;
            if (u < 0.0 || u >= static_cast<double>(n_ - 1)) continue;
            const std::size_t g = node_of(x);
            const double a = u - std::floor(u);
            if (!(h_s_m_[g] > 0.0) && !(h_s_m_[g + 1] > 0.0)) continue;
            scratch_zt_[g] += (1.0 - a) * z;
            scratch_zt_[g + 1] += a * z;
            scratch_zw_[g] += (1.0 - a);
            scratch_zw_[g + 1] += a;
        }
    }
    // Nodal load-path update + exact ΔU/ΔD bookkeeping (research 06 §1.2).
    for (const std::uint32_t gi : nodes_idx_) {
        const std::size_t g = gi;
        if (!(h_s_m_[g] > 0.0)) continue;
        const double f_new = scratch_f_[g];
        const double z_old = z_p_m_[g] + z_e_m_[g];
        const double u_old = 0.5 * k_u_[g] * z_e_m_[g] * z_e_m_[g];
        const PressureLaw law{.p = params_[g], .h = h_s_m_[g], .b = b_m};
        double z_new = z_p_m_[g], zp_new = z_p_m_[g],
               zm_new = z_max_m_[g], ku = k_u_[g];
        // Post-restore nodes carry no stiffness: rebuild the unload
        // modulus the committed (z_max, z_p) pair implies under this
        // wheel's width. Use the SAME beta_u*slope(z_max) convention the
        // ray solve and the virgin commit use — the secant
        // F_v(z_max)/(z_max-z_p) is a different (softer) stiffness that
        // would make a restored rut reload inconsistently with a fresh
        // one and invert the multipass plastic-work ordering.
        if (zm_new > 0.0 && !(ku > 0.0))
            ku = std::max(params_[g].beta_u * law.slope(zm_new) * area_g,
                          1e-12);
        // Displacement-driven target: the nodal total depression the
        // ray solve equilibrated against, gathered back onto this node.
        double zt = -1.0;
        if (have_zeq && scratch_zw_[g] > 0.0)
            zt = std::min(scratch_zt_[g] / scratch_zw_[g], h_s_m_[g]);
        if (zt >= 0.0) {
            // Reproduce z_eq on the node: total depression = zt, the
            // (z_p, z_e) split follows the load path so the committed
            // surface stays work-conjugate with the tire's deflection.
            const double zm_old = z_max_m_[g], zp_old = z_p_m_[g];
            const double f_cap = ku * std::max(0.0, zm_old - zp_old);
            const double f_tol = 1e-12 * std::max(f_cap, 1.0);
            if (zm_old <= 0.0 || zt > zm_old || f_new > f_cap + f_tol) {
                // Virgin branch / new peak: stiffness from the reached
                // depth, elastic part from the committed force.
                zm_new = zt;
                ku = std::max(params_[g].beta_u * law.slope(zt) * area_g,
                              1e-12);
                const double ze = ku > 0.0
                        ? std::min(f_new / ku, zt) : 0.0;
                zp_new = zt - ze;
                z_new = zt;
            } else {
                // Reload/unload inside the past maximum: plastic depth
                // is frozen, elastic depth is the remaining approach.
                zp_new = zp_old;
                zm_new = zm_old;
                z_new = std::max(zt, zp_old);
            }
        } else if (f_new > 0.0) {
            // Reload-line force ceiling: K_u * (z_max - z_p). The
            // virgin/reload boundary is continuous, so an ulp of
            // reconstructed-rounding headroom keeps a repeat of the
            // exact committing force on the reload line (multipass
            // hardening stays bitwise where the physics is flat).
            const double f_cap = ku * std::max(0.0, zm_new - zp_new);
            const double f_tol = 1e-12 * std::max(f_cap, 1.0);
            if (zm_new <= 0.0 || f_new > f_cap + f_tol) {
                // Virgin branch: invert p_v(z) * A_g = f_new on
                // [0, h_s] (the barrier keeps the root below bedrock).
                const double f1 = law(h_s_m_[g]) * area_g;
                if (f_new >= f1) {
                    z_new = h_s_m_[g];
                } else {
                    z_new = solve_bracketed(
                            zm_new, h_s_m_[g],
                            law(zm_new) * area_g - f_new, f1 - f_new,
                            [&](double zz) { return law(zz) * area_g - f_new; });
                }
                zm_new = z_new;
                ku = std::max(params_[g].beta_u * law.slope(z_new) * area_g,
                              1e-12);
                zp_new = std::max(0.0, z_new - f_new / ku);
            } else {
                z_new = zp_new + f_new / ku;
            }
        }
        // Permanent quantities never heal: z_max is the preconsolidation
        // peak and z_p the rut depth, so both clamp monotone nondecreasing.
        // Without this a node under the ~20-40 commits of one patch passage
        // re-derives a smaller zp_new whenever the gathered zt/scattered
        // force fluctuate, "heals" the rut, then re-books the same plastic
        // advance on the next upswing — phantom loss that scales with the
        // stored elastic energy, not the true compaction work.
        zm_new = std::max(zm_new, z_max_m_[g]);
        zp_new = std::max(zp_new, z_p_m_[g]);
        // Keep the serialized invariants exactly: zp <= zm, ze <= zm - zp.
        zp_new = std::min(zp_new, zm_new);
        const double ze_new = std::clamp(z_new - zp_new, 0.0, zm_new - zp_new);
        const double u_new = 0.5 * ku * ze_new * ze_new;
        // Exact hysteresis dissipation (research 06 §1.2): the work the
        // tire did on this node along the committed depression path is
        // area_g * ∫ p_loadpath(z) dz — integrated exactly by 16-pt
        // Gauss-Legendre split at the pre-commit z_max kink (the
        // elastic-predictor reference). Subtracting the elastic-storage
        // change u_new-u_old leaves the hysteresis-loop area: pure
        // elastic reload/unload gives identically zero, and only the
        // virgin-curve excursion above z_max contributes positive
        // compaction loss. This replaces the F_mid*Δz trapezoid, whose
        // force-convention error leaked a fake plastic term every
        // commit and accumulated as phantom ledger loss.
        const double zc_new = zp_new + ze_new;
        const double zm_ref = z_max_m_[g], zp_ref = z_p_m_[g];
        const double lo = std::min(z_old, zc_new), hi = std::max(z_old, zc_new);
        const double mid = std::clamp(zm_ref, lo, hi);
        const auto quad = [&](double a, double b) {
            if (!(b > a)) return 0.0;
            const double c = 0.5 * (a + b), h = 0.5 * (b - a);
            double s = 0.0;
            for (std::size_t i = 0; i < 8; ++i) {
                s += Gauss16::w[i] *
                     (pressure_on_path(g, c + h * Gauss16::x[i], b_m,
                                       zm_ref, zp_ref) +
                      pressure_on_path(g, c - h * Gauss16::x[i], b_m,
                                       zm_ref, zp_ref));
            }
            return s * h;
        };
        double w_pa_m = quad(lo, mid) + quad(mid, hi);
        if (zc_new < z_old) w_pa_m = -w_pa_m;
        const double d_d =
                std::max(0.0, area_g * w_pa_m - (u_new - u_old));
        step_plastic_loss_j_ += d_d;
        plastic_loss_j_ += d_d;
        elastic_energy_j_ += u_new - u_old;
        z_max_m_[g] = zm_new;
        z_p_m_[g] = zp_new;
        z_e_m_[g] = ze_new;
        k_u_[g] = ku;
        force_n_[g] = f_new;
        if (ze_new > 0.0 && in_loaded_[g] == 0) {
            in_loaded_[g] = 1;
            loaded_.push_back(gi);
        } else if (ze_new <= 0.0) {
            in_loaded_[g] = 0; // lazy drop; finish_step compacts
        }
    }
}

void RutMemory::begin_step() noexcept {
    step_plastic_loss_j_ = 0.0;
    std::ranges::fill(covered_, 0);
}

void RutMemory::finish_step() {
    release_uncovered(covered_);
}

void RutMemory::release_uncovered(std::span<const char> covered) {
    std::size_t w = 0;
    for (const std::uint32_t gi : loaded_) {
        const std::size_t g = gi;
        if (in_loaded_[g] == 0) continue; // already dropped this step
        if (g < covered.size() && covered[g] != 0) {
            loaded_[w++] = gi; // still under a wheel this step
            continue;
        }
        if (z_e_m_[g] > 0.0) {
            // Elastic depression released with no interface work —
            // dissipated inside the layer as hysteresis (no rebound
            // partner). The elastic depth folds into the plastic rut
            // rather than zeroing: the committed surface z_p + z_e is
            // then continuous across the unload, so the next ray solve
            // sees the same depression and no phantom deflection boost
            // appears. Draining z_e to zero would lift the surface by
            // z_e — worth ~2u of tyre-side work that the dissipated u
            // never paid for (an energy-ledger leak per released node).
            const double u = 0.5 * k_u_[g] * z_e_m_[g] * z_e_m_[g];
            step_plastic_loss_j_ += u;
            plastic_loss_j_ += u;
            elastic_energy_j_ -= u;
            z_p_m_[g] = std::min(z_p_m_[g] + z_e_m_[g], z_max_m_[g]);
            z_e_m_[g] = 0.0;
            force_n_[g] = 0.0;
        }
        in_loaded_[g] = 0;
    }
    loaded_.resize(w);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (x, area, load) mirror the basal-slip law's inputs
double RutMemory::traction_mu_cap(double x_m, double area_m2,
                                  double normal_load_n) const noexcept {
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (!active_ || !(area_m2 > 0.0) || !(normal_load_n > 0.0)) return inf;
    const double u = (x_m - x0_m_) / dx_m_;
    if (u < 0.0 || u >= static_cast<double>(n_ - 1)) return inf;
    const std::size_t g = node_of(x_m);
    const double a = u - std::floor(u);
    const double h = (1.0 - a) * h_s_m_[g] + a * h_s_m_[g + 1];
    if (!(h > 0.0)) return inf;
    const SoilLayerParams p = mix_params(params_[g], params_[g + 1], 1.0 - a, a);
    const double th = (1.0 - a) * grade_[g] + a * grade_[g + 1];
    const double sigma = std::max(normal_load_n / area_m2, 1e-12);
    const double cs = std::cos(th), sn = std::sin(th);
    const double tau_max = std::max(0.0,
        p.c_bed_pa + p.mu_basal * (p.rho_b_kg_m3 * kGravity * h * cs * cs +
                                 p.alpha_n * sigma) -
        p.rho_b_kg_m3 * kGravity * h * sn * cs);
    return tau_max / (p.alpha_tau * sigma);
}

RutStateDiff RutMemory::snapshot() const {
    RutStateDiff out;
    std::size_t g = 0, count = 0;
    while (g < n_) {
        if (z_max_m_[g] == 0.0 && z_p_m_[g] == 0.0 && z_e_m_[g] == 0.0) { ++g; continue; }
        const auto begin = static_cast<std::uint32_t>(g);
        while (g < n_ && (z_max_m_[g] != 0.0 || z_p_m_[g] != 0.0 || z_e_m_[g] != 0.0)) ++g;
        out.spans.emplace_back(begin, static_cast<std::uint32_t>(g));
        count += static_cast<std::size_t>(out.spans.back().second - begin);
    }
    out.z_max_m.reserve(count); out.z_p_m.reserve(count); out.z_e_m.reserve(count);
    for (const auto &[b, e] : out.spans)
        for (std::size_t k = b; k < e; ++k) {
            out.z_max_m.push_back(z_max_m_[k]);
            out.z_p_m.push_back(z_p_m_[k]);
            out.z_e_m.push_back(z_e_m_[k]);
        }
    return out;
}

void RutMemory::restore(const RutStateDiff &diff, double b_m) {
    if (diff.z_p_m.size() != diff.z_max_m.size() ||
        diff.z_e_m.size() != diff.z_max_m.size())
        throw std::invalid_argument("rut state diff: length mismatch");
    if (!(b_m > 0.0) || !std::isfinite(b_m))
        throw std::invalid_argument("rut state diff: invalid contact width");
    std::size_t at = 0;
    std::uint32_t span_floor = 0;
    for (const auto &[b, e] : diff.spans) {
        // Spans must be ordered and disjoint so apply_diff's noexcept
        // node walk visits each grid node at most once — that is what
        // keeps its pre-reserved loaded_ push_back allocation-free.
        if (e <= b || e > n_ || b < span_floor ||
            at + (e - b) > diff.z_max_m.size())
            throw std::invalid_argument("rut state diff: malformed span");
        span_floor = e;
        for (std::size_t g = b; g < e; ++g, ++at) {
            const double zm = diff.z_max_m[at], zp = diff.z_p_m[at],
                         ze = diff.z_e_m[at];
            if (!(zm >= 0.0 && zp >= 0.0 && ze >= 0.0) ||
                !std::isfinite(zm) || !std::isfinite(zp) ||
                !std::isfinite(ze) || zp > zm || ze > zm - zp ||
                (zm > 0.0 && !(h_s_m_[g] > 0.0)) ||
                (h_s_m_[g] > 0.0 && zm > h_s_m_[g]))
                throw std::invalid_argument(
                        "rut state diff: invalid node value");
        }
    }
    if (at != diff.z_max_m.size())
        throw std::invalid_argument("rut state diff: trailing values");
    apply_diff(diff, b_m);
}

// NOLINTNEXTLINE(bugprone-exception-escape) restore() admits only ordered disjoint spans, so each node pushes into the n_-reserved loaded_ at most once — no reallocation, no throw
void RutMemory::apply_diff(const RutStateDiff &diff,
                           double b_m) noexcept {
    reset();
    const double area_g = 2.0 * b_m * dx_m_;
    std::size_t at = 0;
    for (const auto &[b, e] : diff.spans) {
        for (std::size_t g = b; g < e; ++g, ++at) {
            const double zm = diff.z_max_m[at], zp = diff.z_p_m[at],
                         ze = diff.z_e_m[at];
            z_max_m_[g] = zm; z_p_m_[g] = zp; z_e_m_[g] = ze;
            if (ze > 0.0) {
                in_loaded_[g] = 1;
                loaded_.push_back(static_cast<std::uint32_t>(g));
            }
        }
    }
    std::ranges::sort(loaded_);
    // Rebuild the unload stiffness the restored (z_max, z_p) implies
    // under the reference width. Use the SAME beta_u*slope(z_max) unload
    // modulus solve_deflections and the virgin commit apply — the
    // secant F_v(z_max)/(z_max-z_p) is a different (softer) stiffness
    // that would make a restored rut reload inconsistently with a fresh
    // one and invert the multipass plastic-work ordering.
    for (std::size_t g = 0; g < n_; ++g) {
        if (z_max_m_[g] > 0.0) {
            const PressureLaw law{.p = params_[g], .h = h_s_m_[g], .b = b_m};
            k_u_[g] = std::max(
                    params_[g].beta_u * law.slope(z_max_m_[g]) * area_g,
                    1e-12);
        }
        if (z_e_m_[g] > 0.0) {
            force_n_[g] = k_u_[g] * z_e_m_[g];
            elastic_energy_j_ += 0.5 * k_u_[g] * z_e_m_[g] * z_e_m_[g];
        }
    }
}

void RutMemory::reset() noexcept {
    std::ranges::fill(z_max_m_, 0.0);
    std::ranges::fill(z_p_m_, 0.0);
    std::ranges::fill(z_e_m_, 0.0);
    std::ranges::fill(k_u_, 0.0);
    std::ranges::fill(force_n_, 0.0);
    std::ranges::fill(in_loaded_, 0);
    loaded_.clear();
    elastic_energy_j_ = 0.0;
    step_plastic_loss_j_ = 0.0;
    plastic_loss_j_ = 0.0;
}

void RutMemory::assign(const RutMemory &src) {
    if (src.n_ != n_)
        throw std::invalid_argument("rut memory assign: grid mismatch");
    std::ranges::copy(src.z_max_m_, z_max_m_.begin());
    std::ranges::copy(src.z_p_m_, z_p_m_.begin());
    std::ranges::copy(src.z_e_m_, z_e_m_.begin());
    std::ranges::copy(src.k_u_, k_u_.begin());
    std::ranges::copy(src.force_n_, force_n_.begin());
    std::ranges::copy(src.in_loaded_, in_loaded_.begin());
    loaded_ = src.loaded_;
    elastic_energy_j_ = src.elastic_energy_j_;
    step_plastic_loss_j_ = src.step_plastic_loss_j_;
    plastic_loss_j_ = src.plastic_loss_j_;
}

} // namespace biketyre
