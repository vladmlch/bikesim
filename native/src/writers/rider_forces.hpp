// writers/rider_forces.hpp — port of RiderForceApplier
// (src/bike_sim/sim/ride/rider_forces.py): one spring-damper per rider
// slide DOF, written into qfrc_applied and recorded by the accumulator as
// 'seated_interfaces' (physical_runtime.py:256-259). set_pedal_offsets is
// construction state — the config carries the resolved offset_m
// (ride_sim.py:587), there is no per-call offset input.
// FP operation order follows the Python source expression-for-expression —
// that order is the bitwise contract.
#pragma once

#include <mujoco/mujoco.h>

#include <vector>

#include "../config.hpp"

class RiderForcesWriter {
public:
    // Resolves every configured path's joint through rider_forces.py's
    // _resolve_slide checks (named joint, unlimited slide). Throws
    // std::invalid_argument on any violation, like the Python ctor.
    RiderForcesWriter(const mjModel* m, nativecfg::RiderForcesConfig config);

    // apply() read back as a vector: zeros(nv) with each path's computed
    // force at its dofadr — the same surface
    // acc.add('seated_interfaces', d.qfrc_applied.copy()) captures (the
    // buffer is zeroed before apply and touched by no other writer in
    // between). Pure for d.
    [[nodiscard]] std::vector<double> qfrc(const mjData* d) const;

private:
    // _JointPath.__slots__ resolved: the body's spring params, the joint's
    // addresses, and the construction-time pedal offset_m.
    struct Path {
        int qposadr, dofadr;
        double stiffness_n_m, damping_ns_m;
        double preload_deflection_m, offset_m;
        bool unilateral;
    };
    // _JointPath.force_n / .gap_m — a per-call snapshot kept for a future
    // telemetry surface (interface_loads_n, recorder), like
    // SuspensionWriter's last_; it does not feed qfrc().
    struct Telemetry {
        double force_n, gap_m;
    };

    std::vector<Path> paths_;
    mjtSize nq_, nv_;
    mutable std::vector<Telemetry> last_;
};
