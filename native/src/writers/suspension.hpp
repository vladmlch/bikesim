// writers/suspension.hpp — port of SuspensionForceApplier
// (src/bike_sim/sim/ride/forces.py) plus the pure calculators it calls:
// ForkAirSpring (physics/air_spring.py), Charger3Damper /
// SuperDeluxeDamper (physics/damper.py), CoilShock (physics/coil_shock.py)
// and end_stop (physics/stops.py). FP operation order follows the Python
// source expression-for-expression — that order is the bitwise contract.
#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "../config_types.hpp"
#include "../rtsan.hpp"
#include "writer_types.hpp"

// suspension_energy's terms dict (physical_energy.py:70-89): the five
// elastic storage terms evaluated at q — the material-loss ledger reads
// them every step, independently of the last force call.
struct SuspensionStoredTerms {
    double fork_air, shock_coil, shock_bumper, shock_top_out,
           shock_upper_stop;
};

class SuspensionWriter {
public:
    // Resolves both slide joints with forces.py's
    // `_resolve_compression_joint` checks and validates the config the
    // way the component constructors do (coil spec, HBO zones, travel).
    // Throws std::invalid_argument on any violation.
    SuspensionWriter(const mjModel *m, nativecfg::SuspensionConfig config);

    using Component = std::pair<std::string, std::vector<double> >;

    // Insertion-order component counts ('shock_hbo' exists only in
    // physical mode) — construction-time constants like the Python dict.
    static constexpr std::size_t kLegacyComponentCount = 7;
    static constexpr std::size_t kPhysicalComponentCount = 8;

    // compute_qfrc_components: nv-vectors in the Python dict's insertion
    // order ('shock_hbo' last, physical mode only). The result depends
    // only on d->qpos/d->qvel at the two slide coordinates.
    [[nodiscard]] std::vector<Component> components(const mjData *d) const;

    // R1 allocation-free face. components_into fills `out` with one
    // ForceComponentView per named component — 7 legacy / 8 physical
    // views in components()' insertion order, each
    // ForceKind::suspension — over
    // persistent member storage valid until the next compute.
    // components_into is the checked convenience boundary:
    // std::invalid_argument on a null mjData or wrong view count, then
    // components()' arithmetic verbatim. try_components_into is the
    // prevalidated warm-core entry: same preconditions through
    // CoreStatus, no throw/string construction or allocation in its own
    // code (residual kernel rejections are caught and mapped — see the
    // .cpp), so it is proved noexcept.
    void components_into(const mjData *d,
                         std::span<ForceComponentView> out) const;
    [[nodiscard]] CoreStatus
    try_components_into(const mjData *d, std::span<ForceComponentView> out)
            const noexcept BIKE_NONBLOCKING;

    // The serialized component names in components()' insertion order —
    // a prefix sized to the active physics mode (7 legacy / 8 physical).
    // Stepper's boxed convenience surface serializes views under these
    // names instead of calling the allocating components().
    [[nodiscard]] std::span<const std::string_view>
    component_names() const noexcept;

    // Joint addresses and config the observation helpers read —
    // suspension_energy (physical_energy.py:73-81) reaches
    // applier.shock_qposadr / .fork_qposadr / .coil_shock.specs /
    // .physics_config.end_stops / .controller.air_spring.
    [[nodiscard]] int fork_qposadr() const noexcept { return fork_qposadr_; }
    [[nodiscard]] int fork_dofadr() const noexcept { return fork_dofadr_; }
    [[nodiscard]] int shock_qposadr() const noexcept { return shock_qposadr_; }
    [[nodiscard]] int shock_dofadr() const noexcept { return shock_dofadr_; }
    [[nodiscard]] const nativecfg::SuspensionConfig &
    config() const noexcept { return cfg_; }

    // suspension_energy (physical_energy.py:70-89): elastic terms at
    // d->qpos — no retained compute state, matching the oracle which
    // re-derives every term from the coordinates.
    [[nodiscard]] SuspensionStoredTerms stored_terms(const mjData *d) const;

private:
    nativecfg::SuspensionConfig cfg_;
    bool physical_ = false;
    mjtSize nq_ = 0, nv_ = 0;
    int fork_qposadr_ = 0, fork_dofadr_ = 0;
    int shock_qposadr_ = 0, shock_dofadr_ = 0;
    // No retained telemetry snapshot: SuspensionForceApplier's self.*_n
    // scalars and potential_energy_j (forces.py:207-221) are still
    // computed and validated inside components(), but nothing outside that
    // call ever read the stored copy — every components() call is
    // scratch-free and const in observable effect.

    // components()'s arithmetic verbatim on the member component buffers
    // — the scalar pipeline, the derived() validation, then per name the
    // zero-fill + negated dofadr write. Kept throwing so both boundaries
    // share one kernel and the original rejection types reach the caller.
    void compute_validated(const mjData *d) const;

    // Publish the insertion-ordered views over the member components.
    // noexcept: spans over construction-sized storage only.
    void export_views(std::span<ForceComponentView> out) const noexcept;

    // The per-call component nv-vectors — construction-sized (7 legacy /
    // 8 physical entries of nv each), rewritten in place every compute.
    // export_views borrows them, so they must never resize.
    mutable std::vector<std::vector<double> > components_;
};
