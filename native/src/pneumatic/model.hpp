// pneumatic/model.hpp — the whole-wheel pneumatic tyre model: a port of
// TyrePatchOutput, WheelOutputs, _PatchBrushState, PneumaticTyre,
// _unloaded_outputs, _tread_loss_crr_at_pressure and _tread_loss_force
// from sim/ride/tyre/model.py (lines 30-489), plus the pieces the model
// file imports — clamp_pressure_bar (physics/tyre.py:296-298) and the
// SurfaceMap.at section scan (terrain/surface.py:201-212 with the
// checks.py scalar() finite gate).
//
// One wheel's step composes the ported kernels: intersect() finds the
// ray/road hits, evaluate_carcass() turns per-ray deflection into
// element forces and contiguous patches, and each patch's longitudinal
// force comes from either the discretised bristle field or the lumped
// brush, plus the pressure-scaled tread rolling loss. Patch brush state
// migrates across patch-boundary drift by ray overlap.
//
// Divergence guide:
//  - pneumatic/model.cpp — implementation; its header comment lists the
//    NumPy/CPython lowering decisions that keep the port
//    bitwise-identical (np.linalg.norm/np.dot -> blas::ddot, np.sum ->
//    numeric::numpy_sum, np.sign -> numeric::np_sign, float **
//    -> pyfloat::float_pow, np.maximum -> numeric::np_maximum).
//  - src/bike_sim/sim/ride/tyre/model.py — the behavioral oracle.
//  - tests/reference/test_native_pneumatic_model.py — the bitwise
//    arbiter.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "../config_types.hpp" // nativecfg::PneumaticTyreSpec / SurfaceMap
#include "brush.hpp"           // DiscretisedBrush, BrushResult
#include "carcass.hpp"         // CarcassState, CarcassBuffers, RimStrikeEvent
#include "geometry.hpp"        // RayRing, RayHitBuffers, RoadProfile

namespace biketyre {

    // _ROLLING_LOSS_TAPER_RADPS (model.py:31) — wheel-spin magnitude at
    // which the tread rolling-loss force reaches its full value.
    inline constexpr double kRollingLossTaperRadps = 1.0;

    // PRESSURE_MIN_BAR / PRESSURE_MAX_BAR /
    // CRR_REFERENCE_PRESSURE_BAR / CRR_PRESSURE_EXPONENT
    // (physics/tyre.py:30-33, imported by model.py:13-19).
    inline constexpr double kPressureMinBar = 0.8;
    inline constexpr double kPressureMaxBar = 3.0;
    inline constexpr double kCrrReferencePressureBar = 1.5;
    inline constexpr double kCrrPressureExponent = -0.3;

    // TyrePatchOutput (model.py:34-58) — normal and tangential force
    // data for one contiguous contact patch. The three 3-vectors are
    // fixed arrays standing in for the oracle's (3,) float64 ndarrays.
    struct TyrePatchOutput {
        int ray_start = 0, ray_stop = 0;
        std::string surface_name;
        double normal_load_n = 0.0, support_n = 0.0,
                tangential_force_n = 0.0;
        std::array<double, 3> normal_force_world_n{}, tangent_world{},
                force_world_n{};
        std::array<double, 3> centroid_world_m{};
        double mean_deflection_m = 0.0, contact_length_m = 0.0;
        double slip_ratio = 0.0, transient_slip_ratio = 0.0;
        // slip_speed_mps is the signed tread slip v_x - omega * R_eff;
        // sliding_speed_mps stays its magnitude for the Stribeck
        // lookup (model.py:53-55).
        double sliding_speed_mps = 0.0, slip_speed_mps = 0.0;
        double friction_coefficient = 0.0;
        bool fully_sliding = false;
        double dissipated_power_w = 0.0;
    };

    // WheelOutputs (model.py:61-108) — last computed per-wheel tyre
    // result for force writers and telemetry. The defaults match
    // WheelOutputs.empty() (model.py:85-108) minus the spec-carried
    // fields, so a default-constructed value is already the zero-load
    // snapshot's shape.
    struct WheelOutputs {
        std::string tyre_name;
        double pressure_bar = 0.0;
        std::array<double, 3> hub_position_world_m{},
                hub_velocity_world_mps{};
        double omega_forward_radps = 0.0;
        std::array<double, 3> force_world_n{};
        double normal_load_n = 0.0, support_n = 0.0,
                mean_deflection_m = 0.0;
        double contact_length_m = 0.0, slip_ratio = 0.0,
                transient_slip_ratio = 0.0;
        bool fully_sliding = false;
        bool rim_strike_active = false;
        std::optional<RimStrikeEvent> rim_event;
        double dissipated_power_w = 0.0;
        bool coverage_event = false, airborne = true;
        // The oracle's Tuple[TyrePatchOutput, ...] — the live rows are
        // patch_slots[:patch_count]; the vector stays SIZED at the
        // model's patch bound so evaluate writes every patch in place.
        // A clear()+emplace_back idiom would destroy the elements (and
        // their string storage) each call — the bound-sized slot bank
        // keeps each element's surface_name capacity alive across
        // ticks, which is what lets reserve_patch_names() prewarm the
        // assign-into-capacity discipline for names past the inline
        // string storage.
        std::vector<TyrePatchOutput> patch_slots;
        std::size_t patch_count = 0;
        // The committed patch rows the oracle's `patches` tuple
        // carries — the live prefix of the slot bank.
        [[nodiscard]] std::span<const TyrePatchOutput> patches() const
                noexcept {
            return std::span<const TyrePatchOutput>(patch_slots)
                    .first(patch_count);
        }
    };

    // _PatchBrushState (model.py:111-118) — slip memory tied to a
    // contiguous range of fixed rays. `has_detailed` mirrors
    // `detailed is not None`; the detailed brush's n_elements is the
    // bristle row's live width — under the discretised tier it equals
    // (stop - start) for every state the migration itself created, but
    // an injected state may carry a different width and the step's
    // load-count check must still fire exactly like the oracle's.
    struct PatchBrushState {
        int start = 0, stop = 0;
        double kappa_prime = 0.0;
        bool has_detailed = false;
        double detailed_kappa_prime = 0.0;
        std::vector<double> bristle_deflection_m;
    };

    // clamp_pressure_bar (physics/tyre.py:296-298) — min(MAX, max(MIN,
    // pressure)) with Python min/max operand order: a NaN input clamps
    // to PRESSURE_MIN_BAR, matching `max(0.8, nan)` keeping the bound.
    [[nodiscard]] double clamp_pressure_bar(double pressure_bar) noexcept;

    // _tread_loss_crr_at_pressure (model.py:485-489) — reference-
    // pressure Crr shortfall scaled by the authored pressure law
    // (float ** -> pyfloat::float_pow).
    [[nodiscard]] double
    tread_loss_crr_at_pressure(const nativecfg::PneumaticTyreSpec &tyre);

    // _tread_loss_force (model.py:471-482) — optional Crr shortfall as
    // a tangential force, tapered to zero at wheel rest.
    [[nodiscard]] double tread_loss_force(double crr, double normal_load_n,
                                          double omega_forward_radps,
                                          double v_x_mps) noexcept;

    // PneumaticTyre (model.py:156-468) — the pure per-wheel model.
    //
    // Fixed-capacity layout, allocated at construction:
    //  - ring_/hits_ — the world-fixed ray geometry and its outputs;
    //  - carcass_state_ + carcass_out_ — semantic state and its
    //    evaluate_carcass_into caller buffers;
    //  - pool_/next_pool_ — the _patch_states dict as a bounded slot
    //    array (oracle insertion order = patch order), plus used_
    //    flags for the migration scan;
    //  - brushes_ — lazily built DiscretisedBrush engines indexed by
    //    element count; patch slots carry state only and are copied
    //    into/out of the right-width engine around step() so the warm
    //    path never sizes a brush at drift time. prewarm_detailed_
    //    brushes() builds every width up front for strict
    //    allocation-free contexts;
    //  - staging_/last_outputs_ — evaluate() stages the result before
    //    the O(1) swap commit so a thrown call leaves last_outputs()
    //    byte-identical, exactly like the oracle's assignment order.
    //
    // The class confines its scratch to one owning context like
    // CarcassState — concurrent use requires separate instances.
    class PneumaticTyreModel {
    public:
        // PneumaticTyre.__init__ (model.py:159-165). tier.name and
        // tier.timestep_s ride along unused in the oracle — the writer
        // consumes them — so only the three consumed tier fields are
        // taken. half_angle_deg converts with the oracle's own
        // expression (model.py:30,162 — deg * (np.pi/180)).
        PneumaticTyreModel(nativecfg::PneumaticTyreSpec tyre,
                           std::int64_t n_rays, double half_angle_deg,
                           bool discretised_brush);

        // evaluate (model.py:177-423) — the whole-wheel step. Mutates
        // the carcass state and the patch pool in the oracle's order
        // and commits last_outputs_ only on success. Throws
        // std::invalid_argument where the oracle raises ValueError;
        // a thrown call leaves every semantic byte (carcass state,
        // patch states, last_outputs) untouched, matching the oracle —
        // DiscretisedBrush.step() is the one documented partial-commit
        // boundary (its kappa_prime is committed when a later check
        // throws, exactly like the oracle's own mutation order).
        const WheelOutputs &
        evaluate(std::span<const double> hub_position_world_m,
                 std::span<const double> hub_velocity_world_mps,
                 double omega_forward_radps, const RoadProfile &road,
                 const nativecfg::SurfaceMap &surface_map, double dt_s);

        // The caller-buffer twin: identical values written into `out`
        // (which must have patch_slots.size() >= the model's patch
        // bound — the *_into undersized-buffer contract; callers
        // needing a nonallocating write also reserve each slot's
        // surface_name to the longest name their surface map can
        // carry). The oracle's `self.last_outputs = outputs` semantics
        // still apply: last_outputs() reflects `out` after the call.
        void evaluate_into(std::span<const double> hub_position_world_m,
                           std::span<const double> hub_velocity_world_mps,
                           double omega_forward_radps,
                           const RoadProfile &road,
                           const nativecfg::SurfaceMap &surface_map,
                           double dt_s, WheelOutputs &out);

        // reset (model.py:167-171) — clears carcass, bristle and
        // per-patch slip history; last_outputs() becomes
        // WheelOutputs.empty(tyre). Not noexcept: the snapshot's
        // tyre_name assignment can allocate.
        void reset();

        // set_pressure (model.py:173-175) — updates the live pressure
        // within the viewer's allowed range (the oracle's
        // with_pressure replaces only pressure_bar).
        void set_pressure(double pressure_bar);

        // Semantic-state copy — the Task-11 writer's failure-rollback
        // / probe boundary: the oracle's deepcopy of (tyre, carcass
        // state, _patch_states, last_outputs). Scratch (hits, buffers,
        // brush engines, staging) is not copied. Throws
        // std::invalid_argument on a ring/tier mismatch instead of
        // resizing.
        void assign(const PneumaticTyreModel &source);

        // The oracle's public `self.tyre` — read access; the only
        // mutation the oracle performs is the clamped set_pressure
        // write.
        [[nodiscard]] const nativecfg::PneumaticTyreSpec &
        tyre() const noexcept {
            return tyre_;
        }

        [[nodiscard]] std::size_t n_rays() const noexcept {
            return ring_.n();
        }

        // Maximum disjoint patch count — the (n_rays + 1) // 2 bound
        // the wire's patch state schema uses.
        [[nodiscard]] std::size_t max_patches() const noexcept {
            return (ring_.n() + 1) / 2;
        }

        [[nodiscard]] bool discretised_brush() const noexcept {
            return discretised_;
        }

        [[nodiscard]] const RayRing &ring() const noexcept { return ring_; }

        // The oracle's mutable carcass_state — assign()/restore_
        // fields drive the wire intake and rollback paths.
        [[nodiscard]] CarcassState &carcass_state() noexcept {
            return carcass_state_;
        }

        [[nodiscard]] const CarcassState &carcass_state() const noexcept {
            return carcass_state_;
        }

        // The oracle's _patch_states as an ordered view — insertion
        // order == the last migration's patch order, which is what the
        // dict's .items() exposes.
        [[nodiscard]] std::span<const PatchBrushState>
        patch_states() const noexcept {
            return std::span<const PatchBrushState>(pool_).first(
                    patch_count_);
        }

        // Rebuilds _patch_states from a state-dict projection — the
        // oracle's `self._patch_states[(start,stop)] = state` writes
        // with dict last-write-wins semantics on duplicate keys.
        // Bounds the slot count and each bristle row to the ring;
        // (start, stop) keys pass through unchecked exactly like the
        // oracle's dict keys (the migration's clamped overlap math
        // keeps even a bogus range memory-safe). Throws
        // std::invalid_argument on out-of-range state.
        void restore_patch_states(std::span<const PatchBrushState> states);

        // The oracle's public last_outputs — the most recent committed
        // WheelOutputs (WheelOutputs.empty(tyre) before any evaluate).
        [[nodiscard]] const WheelOutputs &last_outputs() const noexcept {
            return last_outputs_;
        }

        // Builds every DiscretisedBrush width up to n_rays — callers
        // needing a strictly allocation-free evaluate() warm path
        // invoke this once at configure time (engine creation is the
        // only lazy allocation the model can otherwise perform at
        // patch-drift time).
        void prewarm_detailed_brushes();

        // Reserves every patch slot's surface_name to `capacity` —
        // the same configure-time prewarm as
        // prewarm_detailed_brushes(): a surface name longer than the
        // string's inline storage is legal input, and without the
        // reserve its per-patch assign would heap-allocate on the
        // marked warm path. Pass the longest name the surface map can
        // carry.
        void reserve_patch_names(std::size_t capacity);

    private:
        // _states_for_patches (model.py:425-468) — reuses slip state
        // across small patch-boundary shifts, copying bristle overlap
        // by ray index. Rebuilds pool_ into next_pool_ in patch order
        // and swaps (the oracle replaces the dict).
        void migrate_patch_states(
                std::span<const std::pair<int, int>> patches);

        // The lazily-built discretised engine of exactly `width`
        // elements — patch state copies in/out around step() so no
        // engine is ever sized at drift time on the warm path after
        // prewarm_detailed_brushes().
        [[nodiscard]] DiscretisedBrush &brush_for_width(std::size_t width);

        void evaluate_impl(std::span<const double> hub,
                           std::span<const double> velocity,
                           double omega_forward_radps,
                           const RoadProfile &road,
                           const nativecfg::SurfaceMap &surface_map,
                           double dt_s, WheelOutputs &out);

        nativecfg::PneumaticTyreSpec tyre_;
        bool discretised_ = false;
        RayRing ring_;

        // Semantic state.
        CarcassState carcass_state_;
        std::vector<PatchBrushState> pool_, next_pool_;
        std::size_t patch_count_ = 0;
        WheelOutputs last_outputs_;

        // Construction-sized scratch — never escapes the class.
        struct HitsScratch {
            std::vector<double> r_m, delta_m, road_x_m, road_z_m;
        } hits_scratch_;
        struct CarcassScratch {
            std::vector<double> pressure_force_n, elastic_force_n,
                    maxwell_force_n, hysteresis_force_n, rim_force_n,
                    total_element_force_n, material_rate_mps, scratch;
            std::vector<std::pair<int, int>> contact_patches;
            std::vector<double> contact_lengths_m;
            CarcassBuffers buffers;
        } carcass_scratch_;
        std::vector<double> loss_scratch_;
        std::vector<char> used_;
        std::vector<std::unique_ptr<DiscretisedBrush>> brushes_;
        WheelOutputs staging_;
    };

} // namespace biketyre
