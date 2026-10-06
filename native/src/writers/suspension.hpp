// writers/suspension.hpp — port of SuspensionForceApplier
// (src/bike_sim/sim/ride/forces.py) plus the pure calculators it calls:
// ForkAirSpring (physics/air_spring.py), Charger3Damper /
// SuperDeluxeDamper (physics/damper.py), CoilShock (physics/coil_shock.py)
// and end_stop (physics/stops.py). FP operation order follows the Python
// source expression-for-expression — that order is the bitwise contract.
#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <utility>
#include <vector>

#include "../config_types.hpp"

class SuspensionWriter {
public:
    // Resolves both slide joints with forces.py's
    // `_resolve_compression_joint` checks and validates the config the
    // way the component constructors do (coil spec, HBO zones, travel).
    // Throws std::invalid_argument on any violation.
    SuspensionWriter(const mjModel *m, nativecfg::SuspensionConfig config);

    using Component = std::pair<std::string, std::vector<double> >;

    // compute_qfrc_components: nv-vectors in the Python dict's insertion
    // order ('shock_hbo' last, physical mode only). The result depends
    // only on d->qpos/d->qvel at the two slide coordinates.
    [[nodiscard]] std::vector<Component> components(const mjData *d) const;

private:
    // Mirrors SuspensionForceApplier's `self.*_n` scalars and
    // `potential_energy_j` (forces.py:207-221) — a snapshot kept per call
    // for a future telemetry surface; it does not feed `components()`.
    struct Telemetry {
        double fork_spring_n, fork_damper_n, fork_total_n;
        double shock_spring_n, shock_bumper_n, shock_damper_n;
        double shock_top_out_n, shock_upper_stop_n, shock_total_n;

        struct {
            double shock_coil, shock_bumper, shock_top_out, shock_upper_stop;
        } potential_energy_j;
    };

    nativecfg::SuspensionConfig cfg_;
    bool physical_;
    mjtSize nq_, nv_;
    int fork_qposadr_ = 0, fork_dofadr_ = 0;
    int shock_qposadr_ = 0, shock_dofadr_ = 0;
    mutable Telemetry last_{};
};
