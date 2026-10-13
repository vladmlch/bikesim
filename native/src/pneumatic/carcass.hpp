// pneumatic/carcass.hpp — the pneumatic tyre's carcass kernel: a port
// of CarcassState, RimStrikeEvent, CarcassResult, evaluate_carcass,
// material_deflection_rate_mps, _advect_material, _contact_lengths_m
// and the rim-strike FSM (_update_rim_event) from
// sim/ride/tyre/carcass.py (lines 17-364).
//
// Each wheel's radial rays are world-fixed; carcass material moves
// through them as the wheel spins, so Maxwell state is semi-Lagrangian
// advected and the hysteresis rate carries the upwind material
// derivative (docs/RIDE.md §3.1). evaluate_carcass validates first and
// stages every result in locals — the state commit
// (delta_prev_m/elastic_prev_n/maxwell_force_n + initialized) happens
// only after all computation and the rim FSM succeed, so a rejected
// call leaves the state byte-identical.
//
// Divergence guide:
//  - pneumatic/carcass.cpp — implementation; its header comment lists
//    the NumPy lowering decisions that keep the port bitwise-identical
//    (np.tanh -> numeric::np_tanh LUT port, np.hypot -> std::hypot,
//    np.interp -> numeric::np_interp_unit_grid, np.sum -> pairwise_sum,
//    np.dot -> blas::ddot).
//  - src/bike_sim/sim/ride/tyre/carcass.py — the behavioral oracle.
//  - tests/reference/test_native_pneumatic_carcass.py — the bitwise
//    arbiter.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "../config_types.hpp" // nativecfg::PneumaticTyreSpec
#include "geometry.hpp"        // RayRing, RayHits

namespace biketyre {

    struct CarcassBuffers;

    // RimStrikeEvent (carcass.py:22-30) — one completed rim strike;
    // absorbed_energy_j is the peak elastic energy stored in the rim
    // term while the event ran.
    struct RimStrikeEvent {
        double x_m{}, speed_mps{}, peak_load_n{}, peak_rim_force_n{},
                absorbed_energy_j{};
    };

    // CarcassState (carcass.py:33-98) — per-ray material memory plus the
    // rim-strike event in progress for one wheel. The three history rows
    // are sized n_rays at construction and never resize — assign()
    // copies into the existing storage (the writer's rollback/probe
    // path in Task 11 must never reach the allocator through it).
    class CarcassState {
    public:
        // CarcassState.__post_init__ (carcass.py:53-58): at least 3
        // rays; the material rows start zeroed, the event fields clear.
        explicit CarcassState(std::int64_t n_rays);

        // reset (carcass.py:60-66): zeroes the material history and
        // clears any rim event under way (initialized -> false).
        void reset() noexcept;

        // advance_unloaded (carcass.py:68-90): advects and relaxes the
        // Maxwell memory while the tyre is fully unloaded. Returns
        // immediately when not initialized; throws std::invalid_argument
        // where the oracle raises ValueError (after the early return,
        // like the oracle).
        void advance_unloaded(double omega_radps, double dt_s,
                              double dtheta_rad, double relaxation_s);

        // Explicit semantic-state copy — the Task-11 writer's failure
        // rollback/probe boundary. Copies rows into the existing
        // storage (never reallocates); throws std::invalid_argument on
        // a ray-count mismatch instead of resizing.
        void assign(const CarcassState &source);

        // Overwrites the flags and rim-event accumulators — the
        // oracle's public dataclass fields are directly assignable,
        // and the debug binding's state-dict intake plus writer probes
        // need the same write access. Never on the warm evaluate path.
        void restore_fields(bool initialized, bool rim_strike_active,
                            double rim_event_x_m,
                            double rim_event_speed_mps,
                            double rim_event_peak_load_n,
                            double rim_event_peak_force_n,
                            double rim_event_energy_j) noexcept;

        [[nodiscard]] std::size_t n_rays() const noexcept {
            return n_rays_;
        }

        // The oracle's public dataclass fields — n_rays-wide rows.
        [[nodiscard]] std::span<double> delta_prev_m() noexcept {
            return delta_prev_m_;
        }

        [[nodiscard]] std::span<const double> delta_prev_m() const noexcept {
            return delta_prev_m_;
        }

        [[nodiscard]] std::span<double> elastic_prev_n() noexcept {
            return elastic_prev_n_;
        }

        [[nodiscard]] std::span<const double> elastic_prev_n()
                const noexcept {
            return elastic_prev_n_;
        }

        [[nodiscard]] std::span<double> maxwell_force_n() noexcept {
            return maxwell_force_n_;
        }

        [[nodiscard]] std::span<const double> maxwell_force_n()
                const noexcept {
            return maxwell_force_n_;
        }

        [[nodiscard]] bool initialized() const noexcept {
            return initialized_;
        }

        [[nodiscard]] bool rim_strike_active() const noexcept {
            return rim_strike_active_;
        }

        // Rim-event accumulators (carcass.py:47-51) — private writes,
        // read-only views for results/bindings/tests.
        [[nodiscard]] double rim_event_x_m() const noexcept {
            return rim_event_x_m_;
        }

        [[nodiscard]] double rim_event_speed_mps() const noexcept {
            return rim_event_speed_mps_;
        }

        [[nodiscard]] double rim_event_peak_load_n() const noexcept {
            return rim_event_peak_load_n_;
        }

        [[nodiscard]] double rim_event_peak_force_n() const noexcept {
            return rim_event_peak_force_n_;
        }

        [[nodiscard]] double rim_event_energy_j() const noexcept {
            return rim_event_energy_j_;
        }

    private:
        friend void evaluate_carcass_into(
                const RayRing &ring, std::span<const double> delta_m,
                std::span<const double> road_x_m,
                std::span<const double> road_z_m,
                const nativecfg::PneumaticTyreSpec &tyre,
                CarcassState &state, double dt_s, double omega_radps,
                double speed_mps, CarcassBuffers &out);

        // _clear_rim_event (carcass.py:92-98).
        void clear_rim_event() noexcept;

        // _update_rim_event (carcass.py:327-364) — the rim-strike FSM;
        // only evaluate_carcass_into drives it (the oracle's call site
        // is inside evaluate_carcass).
        std::optional<RimStrikeEvent> update_rim_event(
                bool active, std::span<const double> road_x_m,
                std::span<const double> overlap_m,
                std::span<const double> rim_force_n,
                double vertical_load_n, double energy_j,
                double speed_mps) noexcept;

        std::size_t n_rays_ = 0;
        std::vector<double> delta_prev_m_, elastic_prev_n_,
                maxwell_force_n_;
        // advect staging for advance_unloaded — sized n_rays at
        // construction, never escapes the class (not semantic state).
        std::vector<double> advect_scratch_;
        bool initialized_ = false, rim_strike_active_ = false;
        double rim_event_x_m_ = 0.0, rim_event_speed_mps_ = 0.0;
        double rim_event_peak_load_n_ = 0.0, rim_event_peak_force_n_ = 0.0,
                rim_event_energy_j_ = 0.0;
    };

    // CarcassResult (carcass.py:101-124) — owned per-step outputs.
    struct CarcassResult {
        std::vector<double> pressure_force_n, elastic_force_n,
                maxwell_force_n, hysteresis_force_n, rim_force_n,
                total_element_force_n, material_rate_mps;
        std::vector<std::pair<int, int>> contact_patches;
        std::vector<double> contact_lengths_m;
        double force_x_n = 0.0, force_z_n = 0.0, normal_load_n = 0.0,
                rim_energy_j = 0.0;
        bool rim_strike_active = false;
        std::optional<RimStrikeEvent> rim_event;
    };

    // The caller-buffer twin of CarcassResult — the warm-path target
    // the Task-11 writer keeps beside the wheel state. Every row span
    // must be ring.n() wide; contact_patches / contact_lengths_m must
    // hold at least ceil(n/2) entries (the patches_into contract); and
    // scratch must be at least 2*n wide (the two advected rows are
    // staged there while rim_overlap/energy terms are still live).
    // The kernel allocates nothing and must not alias out rows with
    // state history rows — like RayHitBuffers for intersect_into.
    struct CarcassBuffers {
        std::span<double> pressure_force_n, elastic_force_n,
                maxwell_force_n, hysteresis_force_n, rim_force_n,
                total_element_force_n, material_rate_mps;
        std::span<std::pair<int, int>> contact_patches;
        std::span<double> contact_lengths_m;
        std::span<double> scratch;
        std::size_t patch_count = 0;
        double force_x_n = 0.0, force_z_n = 0.0, normal_load_n = 0.0,
                rim_energy_j = 0.0;
        bool rim_strike_active = false;
        std::optional<RimStrikeEvent> rim_event;
    };

    // evaluate_carcass (carcass.py:195-324) into caller storage —
    // the allocation-free warm path. Validation order and the commit
    // boundary match the oracle (lines 226-234, 303-306): all checks
    // and all math run before state rows/flags are touched. Throws
    // std::invalid_argument where the oracle raises ValueError, and on
    // undersized buffers (a C++ boundary contract the oracle expresses
    // via ndarray shape).
    void evaluate_carcass_into(
            const RayRing &ring, std::span<const double> delta_m,
            std::span<const double> road_x_m,
            std::span<const double> road_z_m,
            const nativecfg::PneumaticTyreSpec &tyre,
            CarcassState &state, double dt_s, double omega_radps,
            double speed_mps, CarcassBuffers &out);

    // The allocating convenience twin — identical values, owned
    // storage. `hits` supplies delta_m / road_x_m / road_z_m (the
    // r_m/coverage/airborne fields are consumed by the caller, not the
    // kernel — matching the oracle signature).
    [[nodiscard]] CarcassResult
    evaluate_carcass(const RayRing &ring, const RayHits &hits,
                     const nativecfg::PneumaticTyreSpec &tyre,
                     CarcassState &state, double dt_s, double omega_radps,
                     double speed_mps);

    // material_deflection_rate_mps (carcass.py:127-165) — the upwind
    // material rate Dδ/Dt on the fixed ray grid, into caller storage.
    // Throws std::invalid_argument on the oracle's ValueError cases
    // (shape mismatch, non-positive dt_s/dtheta_rad).
    void material_deflection_rate_mps_into(
            std::span<const double> delta_m,
            std::span<const double> delta_prev_m, double omega_radps,
            double dt_s, double dtheta_rad, std::span<double> out);

    [[nodiscard]] std::vector<double> material_deflection_rate_mps(
            std::span<const double> delta_m,
            std::span<const double> delta_prev_m, double omega_radps,
            double dt_s, double dtheta_rad);

    // _advect_material (carcass.py:168-174) — back-traces a material
    // field by -omega*dt with 0.0 outside the ray window. `out` must be
    // values.size() wide and must not alias `values` (the oracle always
    // builds a fresh array). omega_radps == 0.0 copies verbatim.
    void advect_material_into(std::span<const double> values,
                              double omega_radps, double dt_s,
                              double dtheta_rad, std::span<double> out);

    [[nodiscard]] std::vector<double>
    advect_material(std::span<const double> values, double omega_radps,
                    double dt_s, double dtheta_rad);

} // namespace biketyre
