// writers/rider_forces.hpp — port of RiderForceApplier
// (src/bike_sim/sim/ride/rider_forces.py): one spring-damper per rider
// slide DOF, written into qfrc_applied and recorded by the accumulator as
// 'seated_interfaces' (physical_runtime.py:256-259). The config's offset_m is
// a projection-time snapshot of mutable pedal offsets: Python _follow_cranks
// updates set_pedal_offsets during legacy stepping (ride_sim.py:573-587).
// P3/P4 must port those updates and feed current offsets to this writer before
// claiming step-loop equivalence; P2 verifies only the projected values.
// FP operation order follows the Python source expression-for-expression —
// that order is the bitwise contract.
#pragma once

#include <mujoco/mujoco.h>

#include <vector>

#include "../config_types.hpp"

class RiderForcesWriter {
public:
    // Resolves every configured path's joint through rider_forces.py's
    // _resolve_slide checks (named joint, unlimited slide). Throws
    // std::invalid_argument on any violation, like the Python ctor.
    RiderForcesWriter(const mjModel *m, const nativecfg::RiderForcesConfig &config);

    // apply() read back as a vector: zeros(nv) with each path's computed
    // force at its dofadr — the same surface
    // acc.add('seated_interfaces', d.qfrc_applied.copy()) captures (the
    // buffer is zeroed before apply and touched by no other writer in
    // between). Pure for d.
    [[nodiscard]] std::vector<double> qfrc(const mjData *d) const;

private:
    // _JointPath.__slots__ resolved: the body's spring params, the joint's
    // addresses, and the projection-time pedal offset_m.
    struct Path {
        int qposadr = 0, dofadr = 0;
        double stiffness_n_m = 0.0, damping_ns_m = 0.0;
        double preload_deflection_m = 0.0, offset_m = 0.0;
        bool unilateral = false;
    };

    // _JointPath.force_n / .gap_m — a per-call snapshot kept for a future
    // telemetry surface (interface_loads_n, recorder), like
    // SuspensionWriter's last_; it does not feed qfrc().
    struct Telemetry {
        double force_n = 0.0, gap_m = 0.0;
    };

    std::vector<Path> paths_;
    mjtSize nq_ = 0, nv_ = 0;
    mutable std::vector<Telemetry> last_;
};
