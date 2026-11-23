// tyre/rut_memory.hpp — Stage B deformable thin granular layer: a 1D
// world-fixed rut-memory grid aligned with the pneumatic RoadProfile,
// the C1 finite-depth Bekker-Reece pressure law with smoothstep
// bedrock hardening, linear unload/reload branches, exact per-node
// soil energy accounting and a sparse dirty-span state snapshot.
//
// Physics (docs/research/2026-11-22-tracks-and-surfaces/05,06):
//  - virgin loading p_v(z) = k0*(z/b)^n + B_hp*S((z-zo)/(zw*h_s)) *
//    (exp(c_hp*(z-zo)_+/b) - 1), zo = zeta_o*h_s — C1 at z_o, bedrock
//    stiffening keeps sinkage strictly below h_s;
//  - unload/reload is linear with k_u = beta_u*p_v'(z_max); the plastic
//    rut is z_p = z_max - p_v(z_max)/k_u;
//  - the per-ray solve equilibrates the static tyre element law against
//    the soil pressure law (ray_elastic_force_n vs B*p(z)), then the
//    committed element forces scatter onto nodes through the same hat
//    weights that interpolate node state back to rays (exact virtual
//    work map); node state follows the load path so F_g = K_u*z_e_g at
//    every committed state;
//  - per-node energy split (research 06 §1.2): ΔU = ½K_u z_e²
//    differences, ΔD = A_g·∫p_loadpath(z)dz − ΔU ≥ 0 evaluated by
//    exact Gauss–Legendre path integration split at the z_max kink —
//    no viscous term anywhere (Ban 39).
//
// Probe purity: every soil evaluation reads committed state only;
// probe calls mutate nothing but mutable construction-sized scratch.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "../config_types.hpp"

namespace biketyre {

class RoadProfile;
class RayRing;
struct RayHitBuffers;
class RutMemory;

// Sparse restore image of the deformable layer: merged half-open node
// spans plus (z_max, z_p, z_e) triples per covered node in span order.
// Empty spans == virgin soil.
struct RutStateDiff {
    std::vector<std::pair<std::uint32_t, std::uint32_t>> spans;
    std::vector<double> z_max_m, z_p_m, z_e_m;
};

// Per-surface authored soil constants (research 05: Bekker–Reece
// finite-depth law + Wong unload modulus + basal-slip ceiling).
struct SoilLayerParams {
    double k0_pa = 0.0;        // Bekker–Reece pressure scale [Pa]
    double n = 1.0;            // Bekker exponent
    double zeta_o = 0.42;      // bedrock-onset fraction of h_s
    double zeta_w = 0.35;      // smoothstep ramp fraction of h_s
    double b_hp_pa = 0.0;      // bedrock barrier scale [Pa]
    double c_hp = 0.0;         // barrier growth (dimensionless)
    double beta_u = 6.0;       // unload/reload modulus multiplier
    double mu_basal = 0.0;     // layer/bedrock friction tan(phi_bed)
    double c_bed_pa = 0.0;     // basal cohesion [Pa]
    double rho_b_kg_m3 = 0.0;  // layer bulk density [kg/m^3]
    double alpha_tau = 1.0;    // traction transmission fraction
    double alpha_n = 0.5;      // normal-load fraction reaching the bed
};

// The pneumatic model's soil handle: `state` is read by the per-ray
// solve in every path; `commit` is non-null only on advancing calls
// (probe keeps it null — nothing semantic may mutate).
struct SoilContact {
    const RutMemory *state = nullptr;
    RutMemory *commit = nullptr;
};

[[nodiscard]] const SoilLayerParams &soil_params_for(std::string_view surface_name) noexcept;
[[nodiscard]] bool is_deformable_surface(std::string_view surface_name) noexcept;

class RutMemory {
public:
    // Builds the per-node field: layer thickness and blended soil
    // params over the RoadProfile grid; `active()` is false when every
    // node resolves h_s <= 0 (the compatibility fast path).
    RutMemory(const RoadProfile &road, const nativecfg::SurfaceMap &map);

    [[nodiscard]] bool active() const noexcept { return active_; }
    [[nodiscard]] std::size_t size() const noexcept { return h_s_m_.size(); }
    // Profile-grid node count — the serialized rut.node.* slot bound a
    // full-length deformable ride can dirty (130 m at 5 mm -> 26001).
    [[nodiscard]] std::size_t node_count() const noexcept { return n_; }
    [[nodiscard]] double x0_m() const noexcept { return x0_m_; }
    [[nodiscard]] double dx_m() const noexcept { return dx_m_; }

    // Per-ray joint solve: rewrites hits.delta_m with the coupled
    // tyre/soil equilibrium deflections. Const — reads committed state
    // and writes only mutable scratch; used verbatim by both the
    // advancing and the probe path.
    void solve_deflections(const RayRing &ring,
                           const nativecfg::PneumaticTyreSpec &tyre,
                           double ds_m, RayHitBuffers &hits) const;

    // Advancing-pass commit: scatters the committed element forces to
    // nodes, advances (z_max, z_p, z_e) on the load path, folds
    // ΔU/ΔD into the energy accumulators and releases elastic
    // depression on nodes the pass no longer covers. The tyre spec
    // supplies the contact half-width the nodal law resolves against.
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) parallel span rows mirror the committed ray-element buffers
    void commit_pass(const RayRing &ring,
                     const nativecfg::PneumaticTyreSpec &tyre,
                     std::span<const double> element_force_n,
                     std::span<const double> road_x_m,
                     std::span<const double> soil_z_eq_m = {});
    // NOLINTEND(bugprone-easily-swappable-parameters)

    // Step-frame bookkeeping: begin_step() opens a coverage window the
    // two wheel passes accumulate into; finish_step() releases elastic
    // depth on loaded nodes neither wheel covered this step (the union
    // coverage is what keeps the front wheel's rut alive while the
    // rear pass commits).
    void finish_step();

    // Slope-coupled basal-slip traction ceiling (research 05 §2.2):
    // the min(local, basal) law reduced to an effective-mu cap; +inf
    // where the layer is absent.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (x, area, load) mirror the basal-slip law's inputs
    [[nodiscard]] double traction_mu_cap(double x_m, double area_m2,
                                         double normal_load_n) const noexcept;

    // Committed soil elastic energy — folded into the writer's
    // stored_energy/elastic_energy_j. The scalar accumulates per-node
    // ½K_u z_e² deltas, so rounding can leave it an ulp below zero
    // while every nodal term stays nonnegative; the reported value is
    // clamped at the physically exact bound.
    [[nodiscard]] double soil_elastic_energy_j() const noexcept {
        return elastic_energy_j_ > 0.0 ? elastic_energy_j_ : 0.0;
    }
    // Plastic soil work committed since the last begin_step() —
    // folded into the writer's step loss.
    [[nodiscard]] double step_plastic_loss_j() const noexcept {
        return step_plastic_loss_j_;
    }
    // Cumulative plastic soil work since construction/last reset —
    // diagnostics only; the ledger reads the per-step value.
    [[nodiscard]] double total_plastic_loss_j() const noexcept {
        return plastic_loss_j_;
    }
    void begin_step() noexcept;
    void clear_step_loss() noexcept { step_plastic_loss_j_ = 0.0; }

    // Sparse state I/O for reset/settle/episode restore. restore()
    // re-derives the per-node unload stiffness under the given contact
    // half-width so a diff stays tyre-width agnostic.
    [[nodiscard]] RutStateDiff snapshot() const;
    void restore(const RutStateDiff &diff, double contact_half_width_m);
    // The no-throw half of restore() for callers that already ran its
    // validation (the writer's two-phase set_state commit). Applies a
    // well-formed diff without any check or allocation.
    void apply_diff(const RutStateDiff &diff,
                    double contact_half_width_m) noexcept;
    void reset() noexcept;
    void assign(const RutMemory &source);

    // Read-only node views for state-diff probing and tests.
    [[nodiscard]] std::span<const double> z_max_m() const noexcept { return z_max_m_; }
    [[nodiscard]] std::span<const double> z_p_m() const noexcept { return z_p_m_; }
    [[nodiscard]] std::span<const double> z_e_m() const noexcept { return z_e_m_; }
    [[nodiscard]] std::span<const double> h_s_m() const noexcept { return h_s_m_; }

    // Element-level soil force law helpers (shared by the solve and
    // the inverse load path): pressure Pa → force N on one element
    // area A = width * ds.
    [[nodiscard]] double pressure_virgin(std::size_t g, double z, double b_m) const noexcept;
    [[nodiscard]] double pressure_virgin_slope(std::size_t g, double z, double b_m) const noexcept;
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) node index then (z, width, z_max, z_p) mirror the nodal state tuple
    [[nodiscard]] double pressure_on_path(std::size_t g, double z, double b_m,
                                          double z_max, double z_p) const noexcept;

private:
    [[nodiscard]] std::size_t node_of(double x_m) const noexcept;
    void release_uncovered(std::span<const char> covered);

    std::size_t n_ = 0;
    double x0_m_ = 0.0, dx_m_ = 0.0;
    bool active_ = false;
    // Per-node fields (index g over the profile grid).
    std::vector<double> h_s_m_, z_max_m_, z_p_m_, z_e_m_, k_u_, force_n_,
            grade_;
    std::vector<SoilLayerParams> params_;
    // Nodes currently holding elastic depression — a small list kept
    // by per-node membership flags so release never scans the whole
    // grid on the hot path.
    std::vector<std::uint32_t> loaded_;
    std::vector<char> in_loaded_;
    double elastic_energy_j_ = 0.0;
    double step_plastic_loss_j_ = 0.0;
    double plastic_loss_j_ = 0.0;
    // Construction-sized scratch: per-ray solve rows and the commit
    // scatter/gather rows. Mutable: probe calls write scratch only.
    mutable std::vector<double> scratch_f_, scratch_zt_,
            scratch_zw_;
    mutable std::vector<char> covered_;
    mutable std::vector<std::uint32_t> nodes_idx_;
};

} // namespace biketyre
