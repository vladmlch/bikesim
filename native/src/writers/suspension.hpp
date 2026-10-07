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
};
