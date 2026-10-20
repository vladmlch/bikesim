// pneumatic/model.cpp — implementation of the whole-wheel pneumatic
// tyre model ported from sim/ride/tyre/model.py (lines 30-489).
//
// NumPy/CPython lowering decisions (the same catalogue the kernel
// ports carry — see each helper's header comment for the bitwise
// contract):
//
//   np.asarray(hub/velocity, dtype=float)   -> contiguous float64
//       spans at the API boundary; shape != (3,) and non-finite inputs
//       keep the oracle's validation order (model.py:199-206).
//   np.hypot(v0, v2)                        -> std::hypot (scalar path).
//   np.any(row)                             -> element != 0.0 scan
//       (NaN is truthy, matching `v != 0.0`).
//   np.any(row > 0.0)                       -> element > 0.0 scan.
//   np.dot(a, b) / a @ b                    -> blas::ddot — Accelerate's
//       accumulation order, not a sequential loop.
//   np.linalg.norm(v) (1-D, ord None)       -> sqrt(v.dot(v)) per
//       numpy/linalg/_linalg.py (axis is None branch) — ddot + sqrt.
//   v / scalar and v1 + s * v2              -> per-element doubles in
//       the oracle's operand order (literal +0.0 lanes stay literal).
//   np.sum(row)                             -> numeric::numpy_pairwise_
//       sum (sequential below n = 8, matching pairwise_sum).
//   np.maximum(row, 0.0)                    -> numeric::np_maximum per
//       element (the NEON fmaxnm contract — NaN wins unquieted).
//   np.sign(x)                              -> numeric::np_sign
//       (+1/-1/+0, NaN passthrough — used by the tread-loss direction).
//   float ** float                          -> pyfloat::float_pow — the
//       full float_pow special-case ladder (CPython
//       Objects/floatobject.c:708-824) around the opaque libm pow.
//   Python max(a, b) / min(a, b)            -> std::max/std::min where
//       the operand order is equivalent (documented per call site).
//   SurfaceMap.at(x)                        -> surface_at: the
//       checks.py scalar() finite gate, then the sorted non-overlapping
//       half-open section scan with default fallback
//       (terrain/surface.py:201-212).
//   self._patch_states (dict)               -> pool_/next_pool_ slot
//       arrays in patch order; migration's `key in old_states` scan,
//       `used` set, and (count, (start, stop)) max-overlap selection
//       reproduce the oracle's semantics — including the slice-clamp,
//       the length-1 broadcast fill and the broadcast ValueError when
//       a reused state's bristle row is too short for the overlap in
//       any other way (model.py:461-465).
//
// Failure atomicity: evaluate() validates first, mutates carcass state
// only through the ported kernels' own commit boundaries, stages the
// patch pool in next_pool_ (the live pool_ is untouched until the
// swap), and commits WheelOutputs into last_outputs_ by an O(1) swap
// after everything succeeds — matching the oracle's assignment order.
// The one documented partial-commit boundary is DiscretisedBrush:
// step() commits kappa_prime before a later check could throw, exactly
// like the oracle mutating state.detailed in place; the bristle copy
// back into the slot happens in both the success and exception paths.

#include "model.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "../cblas_abi.hpp"
#include "../numeric_numpy.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"

namespace biketyre {
    namespace {
        constexpr std::size_t kNoSlot =
                static_cast<std::size_t>(-1);

        bool any_nonzero(std::span<const double> row) noexcept {
            return std::ranges::any_of(
                    row, [](double v) { return v != 0.0; });
        }

        // SurfaceMap.at (terrain/surface.py:201-212) — the scalar()
        // finite gate then the half-open section scan; the oracle
        // sorts sections at construction and validation keeps them
        // non-overlapping, so a linear first-match scan is identical.
        const nativecfg::SurfaceSpec &
        surface_at(const nativecfg::SurfaceMap &map, double x_m) {
            if (!std::isfinite(x_m))
                throw std::invalid_argument(
                        "SurfaceMap.x_m: expected finite real scalar");
            for (const nativecfg::SurfaceSection &section: map.sections)
                if (section.start_m <= x_m && x_m < section.end_m)
                    return section.surface;
            return map.surface;
        }

        // _unloaded_outputs (model.py:121-153) — the zero-force result
        // shared by the airborne and no-patch paths.
        // NOLINTBEGIN(bugprone-easily-swappable-parameters) flag order mirrors _unloaded_outputs' keyword sequence
        void write_unloaded(WheelOutputs &out,
                            const nativecfg::PneumaticTyreSpec &tyre,
                            std::span<const double> hub,
                            std::span<const double> velocity,
                            double omega_forward_radps,
                            bool coverage_event, bool airborne,
                            bool rim_strike_active,
                            const std::optional<RimStrikeEvent> &
                                    rim_event) {
            // NOLINTEND(bugprone-easily-swappable-parameters)
            out.tyre_name = tyre.name;
            out.pressure_bar = tyre.pressure_bar;
            out.hub_position_world_m = {hub[0], hub[1], hub[2]};
            out.hub_velocity_world_mps = {velocity[0], velocity[1],
                                          velocity[2]};
            out.omega_forward_radps = omega_forward_radps;
            out.force_world_n = {0.0, 0.0, 0.0};
            out.normal_load_n = 0.0;
            out.support_n = 0.0;
            out.mean_deflection_m = 0.0;
            out.contact_length_m = 0.0;
            out.slip_ratio = 0.0;
            out.transient_slip_ratio = 0.0;
            out.fully_sliding = false;
            out.rim_strike_active = rim_strike_active;
            out.rim_event = rim_event;
            out.dissipated_power_w = 0.0;
            out.coverage_event = coverage_event;
            out.airborne = airborne;
            // The slot bank keeps its elements (and their string
            // capacity) — only the live count resets.
            out.patch_count = 0;
        }

        // WheelOutputs.empty (model.py:85-108) — the reset snapshot:
        // zeroed kinematics, airborne, no rim event, no patches.
        void write_empty(WheelOutputs &out,
                         const nativecfg::PneumaticTyreSpec &tyre) {
            out.tyre_name = tyre.name;
            out.pressure_bar = tyre.pressure_bar;
            out.hub_position_world_m = {0.0, 0.0, 0.0};
            out.hub_velocity_world_mps = {0.0, 0.0, 0.0};
            out.omega_forward_radps = 0.0;
            out.force_world_n = {0.0, 0.0, 0.0};
            out.normal_load_n = 0.0;
            out.support_n = 0.0;
            out.mean_deflection_m = 0.0;
            out.contact_length_m = 0.0;
            out.slip_ratio = 0.0;
            out.transient_slip_ratio = 0.0;
            out.fully_sliding = false;
            out.rim_strike_active = false;
            out.rim_event.reset();
            out.dissipated_power_w = 0.0;
            out.coverage_event = false;
            out.airborne = true;
            out.patch_count = 0;
        }
    } // namespace

    // clamp_pressure_bar (physics/tyre.py:296-298) — min(3.0,
    // max(0.8, p)). Python's two-argument max/min compare `b > a` /
    // `b < a`; std::max/std::min's `a < b` / `b < a` are the same
    // comparisons for this operand arrangement (a NaN keeps the 0.8
    // bound in both).
    double clamp_pressure_bar(double pressure_bar) noexcept {
        return std::min(kPressureMaxBar,
                        std::max(kPressureMinBar, pressure_bar));
    }

    // _tread_loss_crr_at_pressure (model.py:485-489) — tread_loss_crr
    // * (pressure_bar / 1.5) ** -0.3 with CPython's float **.
    double tread_loss_crr_at_pressure(
            const nativecfg::PneumaticTyreSpec &tyre) {
        return tyre.tread_loss_crr *
               pyfloat::float_pow(tyre.pressure_bar /
                                          kCrrReferencePressureBar,
                                  kCrrPressureExponent);
    }

    // _tread_loss_force (model.py:471-482) — early returns in the
    // oracle's order: nonpositive crr/load, undirected motion, then
    // the spin taper. np.sign keeps its NaN passthrough.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror _tread_loss_force's signature
    double tread_loss_force(double crr, double normal_load_n,
                            double omega_forward_radps,
                            double v_x_mps) noexcept {
        if (crr <= 0.0 || normal_load_n <= 0.0) return 0.0;
        double direction = numeric::np_sign(v_x_mps);
        if (direction == 0.0)
            direction = numeric::np_sign(omega_forward_radps);
        if (direction == 0.0) return 0.0;
        const double taper =
                std::min(std::fabs(omega_forward_radps) /
                                 kRollingLossTaperRadps,
                         1.0);
        return -crr * normal_load_n * taper * direction;
    }

    PneumaticTyreModel::PneumaticTyreModel(
            nativecfg::PneumaticTyreSpec tyre, std::int64_t n_rays,
            double half_angle_deg, bool discretised_brush)
        : tyre_(std::move(tyre)), discretised_(discretised_brush),
          // model.py:162 — RayRing(n_rays, half_angle_deg * _RAD),
          // _RAD = np.pi / 180.0 (model.py:30).
          ring_(n_rays, half_angle_deg * (std::numbers::pi / 180.0)),
          carcass_state_(n_rays) {
        const std::size_t n = ring_.n();
        const std::size_t bound = max_patches();

        hits_scratch_.r_m.assign(n, 0.0);
        hits_scratch_.delta_m.assign(n, 0.0);
        hits_scratch_.road_x_m.assign(n, 0.0);
        hits_scratch_.road_z_m.assign(n, 0.0);

        auto &cs = carcass_scratch_;
        cs.pressure_force_n.assign(n, 0.0);
        cs.elastic_force_n.assign(n, 0.0);
        cs.maxwell_force_n.assign(n, 0.0);
        cs.hysteresis_force_n.assign(n, 0.0);
        cs.rim_force_n.assign(n, 0.0);
        cs.total_element_force_n.assign(n, 0.0);
        cs.material_rate_mps.assign(n, 0.0);
        cs.scratch.assign(2 * n, 0.0);
        cs.contact_patches.assign(bound, {0, 0});
        cs.contact_lengths_m.assign(bound, 0.0);
        cs.buffers = CarcassBuffers{
                .pressure_force_n = cs.pressure_force_n,
                .elastic_force_n = cs.elastic_force_n,
                .maxwell_force_n = cs.maxwell_force_n,
                .hysteresis_force_n = cs.hysteresis_force_n,
                .rim_force_n = cs.rim_force_n,
                .total_element_force_n = cs.total_element_force_n,
                .material_rate_mps = cs.material_rate_mps,
                .contact_patches = cs.contact_patches,
                .contact_lengths_m = cs.contact_lengths_m,
                .scratch = cs.scratch,
                .patch_count = 0,
                .force_x_n = 0.0,
                .force_z_n = 0.0,
                .normal_load_n = 0.0,
                .rim_energy_j = 0.0,
                .rim_strike_active = false,
                .rim_event = std::nullopt};

        pool_.resize(bound);
        next_pool_.resize(bound);
        for (PatchBrushState &slot: pool_) slot.bristle_deflection_m.reserve(n);
        for (PatchBrushState &slot: next_pool_)
            slot.bristle_deflection_m.reserve(n);
        used_.assign(bound, false);
        loss_scratch_.assign(n, 0.0);
        brushes_.resize(n + 1);
        // Bound-sized slot banks: evaluate() writes patch elements in
        // place so their surface_name storage persists across calls —
        // reserve_patch_names() then prewarms that capacity for
        // callers needing a strictly allocation-free warm path.
        last_outputs_.patch_slots.resize(bound);
        staging_.patch_slots.resize(bound);
        write_empty(last_outputs_, tyre_);
    }

    void PneumaticTyreModel::reset() {
        carcass_state_.reset();
        patch_count_ = 0;
        write_empty(last_outputs_, tyre_);
    }

    void PneumaticTyreModel::set_pressure(double pressure_bar) {
        // model.py:175 — with_pressure replaces only pressure_bar;
        // last_outputs keeps its committed snapshot pressure until the
        // next evaluate, exactly like the oracle.
        tyre_.pressure_bar = clamp_pressure_bar(pressure_bar);
    }

    void PneumaticTyreModel::prewarm_detailed_brushes() {
        for (std::size_t w = 1; w <= ring_.n(); ++w)
            (void) brush_for_width(w);
    }

    void PneumaticTyreModel::reserve_patch_names(std::size_t capacity) {
        for (TyrePatchOutput &p: last_outputs_.patch_slots)
            p.surface_name.reserve(capacity);
        for (TyrePatchOutput &p: staging_.patch_slots)
            p.surface_name.reserve(capacity);
    }

    DiscretisedBrush &
    PneumaticTyreModel::brush_for_width(std::size_t width) {
        std::unique_ptr<DiscretisedBrush> &engine = brushes_[width];
        if (!engine)
            engine = std::make_unique<DiscretisedBrush>(
                    static_cast<std::int64_t>(width));
        return *engine;
    }

    void PneumaticTyreModel::migrate_patch_states(
            std::span<const std::pair<int, int>> patches) {
        // _states_for_patches (model.py:425-468). The oracle builds a
        // fresh dict and rebinds it once the loop succeeds; the port
        // rebuilds next_pool_ in the same order and swaps at the end,
        // so a rejected migration leaves the live pool untouched.
        std::ranges::fill(used_, false);
        std::size_t next = 0;
        for (const auto &[start_i, stop_i]: patches) {
            const std::int64_t start = start_i, stop = stop_i;
            // model.py:435 — exact (start, stop) key on an unused slot.
            std::size_t old = kNoSlot;
            for (std::size_t i = 0; i < patch_count_; ++i) {
                if (!used_[i] && pool_[i].start == start_i &&
                    pool_[i].stop == stop_i) {
                    old = i;
                    break;
                }
            }
            if (old == kNoSlot) {
                // model.py:437-445 — max(count, (start, stop)) over
                // positive-overlap unused slots; keys are unique so the
                // lexicographic total order needs no first-seen rule.
                std::int64_t best_count = 0;
                for (std::size_t i = 0; i < patch_count_; ++i) {
                    if (used_[i]) continue;
                    const std::int64_t count =
                            std::min<std::int64_t>(stop, pool_[i].stop) -
                            std::max<std::int64_t>(start,
                                                   pool_[i].start);
                    if (count <= 0) continue;
                    if (old == kNoSlot || count > best_count ||
                        (count == best_count &&
                         (pool_[i].start > pool_[old].start ||
                          (pool_[i].start == pool_[old].start &&
                           pool_[i].stop > pool_[old].stop)))) {
                        old = i;
                        best_count = count;
                    }
                }
            }

            PatchBrushState &ns = next_pool_[next++];
            ns.start = start_i;
            ns.stop = stop_i;
            ns.has_detailed = discretised_;
            ns.detailed_kappa_prime = 0.0;
            if (old == kNoSlot) {
                // _PatchBrushState(start, stop) + a fresh brush —
                // model.py:447-450.
                ns.kappa_prime = 0.0;
                if (discretised_)
                    ns.bristle_deflection_m.assign(
                            static_cast<std::size_t>(stop_i - start_i),
                            0.0);
                else
                    ns.bristle_deflection_m.clear();
            } else {
                // model.py:451-465 — carry kappa_prime and rebuild the
                // detailed brush at the new width, copying the bristle
                // overlap by ray index.
                used_[old] = true;
                const PatchBrushState &prev = pool_[old];
                ns.kappa_prime = prev.kappa_prime;
                if (discretised_) {
                    ns.bristle_deflection_m.assign(
                            static_cast<std::size_t>(stop_i - start_i),
                            0.0);
                    if (prev.has_detailed) {
                        ns.detailed_kappa_prime =
                                prev.detailed_kappa_prime;
                        const std::int64_t os =
                                std::max<std::int64_t>(start,
                                                       prev.start);
                        const std::int64_t oe =
                                std::min<std::int64_t>(stop, prev.stop);
                        // Python slicing clamps the previous row's
                        // window to its length; NumPy's elementwise
                        // assignment then broadcasts a length-1 RHS
                        // into the whole target and rejects every
                        // other short row (ValueError) — reproduce all
                        // three cases. got <= need holds because the
                        // unclamped span is exactly oe - os.
                        const std::size_t len =
                                prev.bristle_deflection_m.size();
                        const std::size_t rb =
                                static_cast<std::size_t>(std::min<
                                        std::int64_t>(
                                        std::max<std::int64_t>(
                                                os - prev.start, 0),
                                        static_cast<std::int64_t>(
                                                len)));
                        const std::size_t re =
                                static_cast<std::size_t>(std::min<
                                        std::int64_t>(
                                        std::max<std::int64_t>(
                                                oe - prev.start, 0),
                                        static_cast<std::int64_t>(
                                                len)));
                        const std::size_t got = re > rb ? re - rb : 0;
                        const std::size_t need =
                                static_cast<std::size_t>(oe - os);
                        const std::span<double> dst =
                                std::span<double>(
                                        ns.bristle_deflection_m)
                                        .subspan(static_cast<
                                                std::size_t>(os - start),
                                                need);
                        if (got == need)
                            std::ranges::copy(
                                    std::span<const double>(
                                            prev.bristle_deflection_m)
                                            .subspan(rb, got),
                                    dst.begin());
                        else if (got == 1)
                            std::ranges::fill(
                                    dst, prev.bristle_deflection_m
                                                    [rb]);
                        else
                            throw std::invalid_argument(
                                    "bristle overlap could not "
                                    "broadcast onto the migrated "
                                    "patch");
                    }
                } else {
                    ns.bristle_deflection_m.clear();
                }
            }
        }
        pool_.swap(next_pool_);
        patch_count_ = next;
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror PneumaticTyre.evaluate's signature
    void PneumaticTyreModel::evaluate_impl(
            std::span<const double> hub_position_world_m,
            std::span<const double> hub_velocity_world_mps,
            double omega_forward_radps, const RoadProfile &road,
            const nativecfg::SurfaceMap &surface_map, double dt_s,
            WheelOutputs &out) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const std::span<const double> &hub = hub_position_world_m;
        const std::span<const double> &velocity =
                hub_velocity_world_mps;
        // model.py:199-206 — shape, then finiteness, then spin.
        if (hub.size() != 3 || velocity.size() != 3)
            throw std::invalid_argument(
                    "hub position and velocity must be 3-vectors in "
                    "world axes");
        if (!std::isfinite(hub[0]) || !std::isfinite(hub[1]) ||
            !std::isfinite(hub[2]) || !std::isfinite(velocity[0]) ||
            !std::isfinite(velocity[1]) || !std::isfinite(velocity[2]))
            throw std::invalid_argument(
                    "hub position and velocity must be finite");
        if (!std::isfinite(omega_forward_radps))
            throw std::invalid_argument("wheel spin must be finite");
        // The *_into undersized-buffer contract, up front like the
        // carcass kernel's own out-row check (carcass.cpp): the caller
        // buffer must carry the model's patch bound as sized slots —
        // the writer's banks are construction-sized so this can never
        // fire on the warm path.
        if (out.patch_slots.size() < max_patches())
            throw std::invalid_argument(
                    "evaluate: out patch slots below the patch bound");

        // model.py:208-214 — ray/road intersection.
        RayHitBuffers hits{.r_m = hits_scratch_.r_m,
                           .delta_m = hits_scratch_.delta_m,
                           .road_x_m = hits_scratch_.road_x_m,
                           .road_z_m = hits_scratch_.road_z_m};
        intersect_into(ring_, hub[0], hub[2],
                       tyre_.outer_radius_mm / 1000.0, road, hits);

        // model.py:215-237 — the fresh-airborne early-out: advect the
        // Maxwell memory, drop every patch state, report unloaded.
        if (hits.airborne && !carcass_state_.rim_strike_active() &&
            !any_nonzero(carcass_state_.delta_prev_m()) &&
            !any_nonzero(carcass_state_.elastic_prev_n())) {
            carcass_state_.advance_unloaded(
                    omega_forward_radps, dt_s, ring_.dtheta_rad(),
                    tyre_.rate_relaxation_s);
            patch_count_ = 0;
            write_unloaded(out, tyre_, hub, velocity,
                           omega_forward_radps, hits.coverage_event,
                           true, false, std::nullopt);
            return;
        }

        // model.py:239-248 — radial carcass evaluation.
        const double speed_mps = std::hypot(velocity[0], velocity[2]);
        evaluate_carcass_into(ring_, hits.delta_m, hits.road_x_m,
                              hits.road_z_m, tyre_, carcass_state_,
                              dt_s, omega_forward_radps, speed_mps,
                              carcass_scratch_.buffers);
        const CarcassBuffers &carcass = carcass_scratch_.buffers;

        // model.py:250-263 — contacting carcass produced no patch:
        // keep the rim fields the kernel committed.
        if (carcass.patch_count == 0) {
            patch_count_ = 0;
            write_unloaded(out, tyre_, hub, velocity,
                           omega_forward_radps, hits.coverage_event,
                           hits.airborne, carcass.rim_strike_active,
                           carcass.rim_event);
            return;
        }

        // model.py:265 — slip state migrates across boundary drift.
        migrate_patch_states(
                carcass.contact_patches.first(carcass.patch_count));

        // model.py:266-274 — accumulators in declaration order.
        std::array<double, 3> total_force_world_n{0.0, 0.0, 0.0};
        double total_load_n = 0.0, total_support_n = 0.0;
        double weighted_deflection_m = 0.0, weighted_kappa = 0.0,
                weighted_kappa_prime = 0.0;
        double total_contact_length_m = 0.0, brush_loss_w = 0.0;
        out.patch_count = 0;

        // model.py:276-280 — carcass hysteresis dissipation clipped at
        // zero per element, then the pairwise sum.
        for (std::size_t i = 0; i < ring_.n(); ++i)
            loss_scratch_[i] = numeric::np_maximum(
                    carcass.hysteresis_force_n[i] *
                            carcass.material_rate_mps[i],
                    0.0);
        const double carcass_loss_w =
                numeric::numpy_sum(loss_scratch_);

        for (std::size_t k = 0; k < carcass.patch_count; ++k) {
            const auto [start_i, stop_i] = carcass.contact_patches[k];
            const double contact_length_m =
                    carcass.contact_lengths_m[k];
            const std::size_t w =
                    static_cast<std::size_t>(stop_i - start_i);
            const std::span<const double> element_force_n =
                    carcass.total_element_force_n.subspan(
                            static_cast<std::size_t>(start_i), w);
            // model.py:287.
            if (!std::ranges::any_of(element_force_n,
                                     [](double v) {
                                         return v > 0.0;
                                     }))
                continue;

            // model.py:290-298 — element forces point road-to-hub;
            // the patch normal force is their negated projection.
            const int wn = static_cast<int>(w);
            const std::size_t start_u =
                    static_cast<std::size_t>(start_i);
            const std::array<double, 3> normal_force_world_n{
                    -blas::ddot(wn, element_force_n.data(), 1,
                                ring_.ux().subspan(start_u, w).data(),
                                1),
                    0.0,
                    -blas::ddot(wn, element_force_n.data(), 1,
                                ring_.uz().subspan(start_u, w).data(),
                                1)};
            // np.linalg.norm lowers to sqrt(v.dot(v)) — ddot + sqrt.
            const double normal_load_n = std::sqrt(
                    blas::ddot(3, normal_force_world_n.data(), 1,
                               normal_force_world_n.data(), 1));
            if (normal_load_n <= 0.0) continue;
            const std::array<double, 3> normal_world{
                    normal_force_world_n[0] / normal_load_n,
                    0.0 / normal_load_n,
                    normal_force_world_n[2] / normal_load_n};
            const std::array<double, 3> tangent_world{normal_world[2],
                                                    0.0,
                                                    -normal_world[0]};

            // model.py:301-309 — force-weighted centroid and mean
            // deflection; weight_sum is the pairwise reduce.
            const double weight_sum =
                    numeric::numpy_sum(element_force_n);
            const std::array<double, 3> centroid_world_m{
                    blas::ddot(wn, element_force_n.data(), 1,
                               hits.road_x_m.subspan(start_u, w)
                                       .data(),
                               1) /
                            weight_sum,
                    hub[1],
                    blas::ddot(wn, element_force_n.data(), 1,
                               hits.road_z_m.subspan(start_u, w)
                                       .data(),
                               1) /
                            weight_sum};
            const double mean_deflection_m =
                    blas::ddot(wn, element_force_n.data(), 1,
                               hits.delta_m.subspan(start_u, w).data(),
                               1) /
                    weight_sum;
            const double half_length_m = 0.5 * contact_length_m;

            // model.py:311-318 — tread slip against the deflection-
            // reduced effective radius. max(r, 1e-6) keeps r on NaN,
            // matching Python max's `1e-6 > r` test.
            const double v_x_mps =
                    blas::ddot(3, velocity.data(), 1,
                               tangent_world.data(), 1);
            const double effective_radius_m =
                    std::max(tyre_.outer_radius_mm / 1000.0 -
                                     mean_deflection_m / 3.0,
                             1e-6);
            const double v_s_mps =
                    v_x_mps - omega_forward_radps * effective_radius_m;
            const double slip_ratio =
                    std::fabs(v_x_mps) > 1e-9
                            ? -v_s_mps / std::fabs(v_x_mps)
                            : 0.0;
            const nativecfg::SurfaceSpec &surface =
                    surface_at(surface_map, centroid_world_m[0]);

            // model.py:319-348 — patch slip state, then the brush.
            // The migration emitted one slot per contact patch in the
            // same order, so contact_patches[k] lives at pool_[k].
            PatchBrushState &state = pool_[k];
            BrushResult brush_result;
            if (discretised_) {
                // model.py:321-323 — a patch with no detailed brush
                // starts fresh, seeded by the patch-level slip.
                if (!state.has_detailed) {
                    state.has_detailed = true;
                    state.bristle_deflection_m.assign(w, 0.0);
                    state.detailed_kappa_prime = state.kappa_prime;
                }
                // Step the slot's brush state through the right-width
                // engine; copy state back on success AND on a throw
                // (the engine holds no semantic state of its own).
                DiscretisedBrush &engine = brush_for_width(
                        state.bristle_deflection_m.size());
                std::ranges::copy(state.bristle_deflection_m,
                                  engine.bristle_deflection_m()
                                          .begin());
                engine.restore_kappa_prime(state.detailed_kappa_prime);
                try {
                    brush_result = engine.step(
                            element_force_n, half_length_m, v_x_mps,
                            v_s_mps,
                            tyre_.relaxation_length_mm / 1000.0,
                            surface, dt_s);
                } catch (...) {
                    state.detailed_kappa_prime = engine.kappa_prime();
                    std::ranges::copy(
                            engine.bristle_deflection_m(),
                            state.bristle_deflection_m.begin());
                    throw;
                }
                state.detailed_kappa_prime = engine.kappa_prime();
                std::ranges::copy(engine.bristle_deflection_m(),
                                  state.bristle_deflection_m.begin());
                // model.py:333 — the patch-level slip mirrors the
                // stepped brush.
                state.kappa_prime = state.detailed_kappa_prime;
            } else {
                state.kappa_prime =
                        relax(state.kappa_prime, v_x_mps, v_s_mps,
                              tyre_.relaxation_length_mm / 1000.0,
                              dt_s);
                brush_result = lumped_brush(
                        state.kappa_prime, normal_load_n,
                        half_length_m, surface, std::nullopt,
                        std::fabs(v_s_mps));
            }

            // model.py:350-362 — tread rolling loss, the combined
            // patch force and the two clipped dissipation terms.
            const double tread_loss_force_n =
                    tread_loss_force(tread_loss_crr_at_pressure(tyre_),
                                     normal_load_n,
                                     omega_forward_radps, v_x_mps);
            const double tangential_force_n =
                    brush_result.force_n + tread_loss_force_n;
            const std::array<double, 3> patch_force_world_n{
                    normal_force_world_n[0] +
                            tangential_force_n * tangent_world[0],
                    normal_force_world_n[1] +
                            tangential_force_n * tangent_world[1],
                    normal_force_world_n[2] +
                            tangential_force_n * tangent_world[2]};
            const double support_n = normal_force_world_n[2];
            const double rolling_loss_w =
                    std::max(0.0, -tread_loss_force_n * v_x_mps);
            const double slip_loss_w =
                    std::max(0.0, -brush_result.force_n * v_s_mps);
            const double patch_loss_w = rolling_loss_w + slip_loss_w;
            brush_loss_w += patch_loss_w;

            // model.py:364-392 — the patch record, then the
            // accumulate block in the oracle's order. Write the slot's
            // fields first and publish it after — a throw mid-record
            // leaves the live prefix holding only fully-written rows.
            TyrePatchOutput &po = out.patch_slots[out.patch_count];
            po.ray_start = start_i;
            po.ray_stop = stop_i;
            po.surface_name = surface.name;
            po.normal_load_n = normal_load_n;
            po.support_n = support_n;
            po.tangential_force_n = tangential_force_n;
            po.normal_force_world_n = normal_force_world_n;
            po.tangent_world = tangent_world;
            po.force_world_n = patch_force_world_n;
            po.centroid_world_m = centroid_world_m;
            po.mean_deflection_m = mean_deflection_m;
            po.contact_length_m = contact_length_m;
            po.slip_ratio = slip_ratio;
            po.transient_slip_ratio = state.kappa_prime;
            po.sliding_speed_mps = std::fabs(v_s_mps);
            po.slip_speed_mps = v_s_mps;
            po.friction_coefficient =
                    brush_result.friction_coefficient;
            po.fully_sliding = brush_result.fully_sliding;
            po.dissipated_power_w = patch_loss_w;
            ++out.patch_count;

            total_force_world_n[0] += patch_force_world_n[0];
            total_force_world_n[1] += patch_force_world_n[1];
            total_force_world_n[2] += patch_force_world_n[2];
            total_load_n += normal_load_n;
            total_support_n += support_n;
            weighted_deflection_m += mean_deflection_m * normal_load_n;
            weighted_kappa += slip_ratio * normal_load_n;
            weighted_kappa_prime += state.kappa_prime * normal_load_n;
            total_contact_length_m += contact_length_m;
        }

        // model.py:394-399 — load-weighted aggregates.
        double mean_deflection_out = 0.0, slip_ratio_out = 0.0,
                transient_slip_ratio_out = 0.0;
        if (total_load_n > 0.0) {
            mean_deflection_out = weighted_deflection_m / total_load_n;
            slip_ratio_out = weighted_kappa / total_load_n;
            transient_slip_ratio_out =
                    weighted_kappa_prime / total_load_n;
        }

        // model.py:401-421 — the committed wheel outputs.
        out.tyre_name = tyre_.name;
        out.pressure_bar = tyre_.pressure_bar;
        out.hub_position_world_m = {hub[0], hub[1], hub[2]};
        out.hub_velocity_world_mps = {velocity[0], velocity[1],
                                      velocity[2]};
        out.omega_forward_radps = omega_forward_radps;
        out.force_world_n = total_force_world_n;
        out.normal_load_n = total_load_n;
        out.support_n = total_support_n;
        out.mean_deflection_m = mean_deflection_out;
        out.contact_length_m = total_contact_length_m;
        out.slip_ratio = slip_ratio_out;
        out.transient_slip_ratio = transient_slip_ratio_out;
        out.fully_sliding =
                std::ranges::any_of(out.patches(),
                                    [](const TyrePatchOutput &p) {
                                        return p.fully_sliding;
                                    });
        out.rim_strike_active = carcass.rim_strike_active;
        out.rim_event = carcass.rim_event;
        out.dissipated_power_w = carcass_loss_w + brush_loss_w;
        out.coverage_event = hits.coverage_event;
        out.airborne = false;
    }

    const WheelOutputs &PneumaticTyreModel::evaluate(
            std::span<const double> hub_position_world_m,
            std::span<const double> hub_velocity_world_mps,
            double omega_forward_radps, const RoadProfile &road,
            const nativecfg::SurfaceMap &surface_map, double dt_s) {
        // Stage into the scratch outputs, then the O(1) swap commit —
        // a thrown step leaves last_outputs_ holding the previous
        // complete result, matching self.last_outputs = outputs.
        evaluate_impl(hub_position_world_m, hub_velocity_world_mps,
                      omega_forward_radps, road, surface_map, dt_s,
                      staging_);
        std::swap(last_outputs_, staging_);
        return last_outputs_;
    }

    void PneumaticTyreModel::evaluate_into(
            std::span<const double> hub_position_world_m,
            std::span<const double> hub_velocity_world_mps,
            double omega_forward_radps, const RoadProfile &road,
            const nativecfg::SurfaceMap &surface_map, double dt_s,
            WheelOutputs &out) {
        if (out.patch_slots.size() < max_patches())
            throw std::invalid_argument(
                    "evaluate_into: out.patch_slots must hold the "
                    "model's patch bound");
        evaluate_impl(hub_position_world_m, hub_velocity_world_mps,
                      omega_forward_radps, road, surface_map, dt_s,
                      out);
        // self.last_outputs = outputs — the semantic-state commit
        // applies to the caller-buffer path too.
        last_outputs_ = out;
    }

    void PneumaticTyreModel::restore_patch_states(
            std::span<const PatchBrushState> states) {
        if (states.size() > pool_.size())
            throw std::invalid_argument(
                    "patch_states exceed the ring's patch bound");
        // The oracle rebuilds _patch_states as a fresh dict and rebinds
        // it — stage into next_pool_ so a rejected entry leaves the
        // live pool byte-identical.
        std::size_t next = 0;
        for (const PatchBrushState &s: states) {
            if (s.bristle_deflection_m.size() > ring_.n())
                throw std::invalid_argument(
                        "patch-state bristle row exceeds the ray ring");
            // Dict semantics: a repeated (start, stop) overwrites.
            std::size_t slot = next;
            for (std::size_t i = 0; i < next; ++i)
                if (next_pool_[i].start == s.start &&
                    next_pool_[i].stop == s.stop) {
                    slot = i;
                    break;
                }
            if (slot == next) ++next;
            PatchBrushState &d = next_pool_[slot];
            d.start = s.start;
            d.stop = s.stop;
            d.kappa_prime = s.kappa_prime;
            d.has_detailed = s.has_detailed;
            d.detailed_kappa_prime = s.detailed_kappa_prime;
            d.bristle_deflection_m.assign(
                    s.bristle_deflection_m.begin(),
                    s.bristle_deflection_m.end());
        }
        pool_.swap(next_pool_);
        patch_count_ = next;
    }

    void PneumaticTyreModel::assign(const PneumaticTyreModel &source) {
        if (ring_.n() != source.ring_.n() ||
            discretised_ != source.discretised_)
            throw std::invalid_argument(
                    "pneumatic tyre assign: ring/tier mismatch");
        // The oracle's deepcopy snapshot: tyre, carcass state, patch
        // states and last_outputs — scratch stays untouched.
        tyre_ = source.tyre_;
        carcass_state_.assign(source.carcass_state_);
        for (std::size_t i = 0; i < source.patch_count_; ++i)
            pool_[i] = source.pool_[i];
        patch_count_ = source.patch_count_;
        last_outputs_ = source.last_outputs_;
    }

} // namespace biketyre
