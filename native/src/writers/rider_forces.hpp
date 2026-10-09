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

#include <span>
#include <string>
#include <vector>

#include "../config_types.hpp"
#include "../rtsan.hpp"
#include "writer_types.hpp"

class RiderForcesWriter {
public:
    // _JointPath.__slots__ resolved: the body's spring params, the joint's
    // addresses, and the projection-time pedal offset_m. `name` is the
    // SeatedPose body name — the 'seated_'+name stored-terms key
    // (physical_observations.py:89-93).
    struct Path {
        std::string name;
        int qposadr = 0, dofadr = 0;
        double stiffness_n_m = 0.0, damping_ns_m = 0.0;
        double preload_deflection_m = 0.0, offset_m = 0.0;
        bool unilateral = false;
    };

    // _JointPath.force_n / .gap_m — per-path telemetry read back by qfrc()
    // itself (each path's computed force_n is the row's dofadr value), so
    // unlike the removed brake/suspension snapshots this state is live
    // output state and must stay. It is per-instance scratch of the owning
    // Stepper context: qfrc() is const yet mutates it, which stays correct
    // only because calls arrive through the nanobind boundary while the GIL
    // is held — cross-context sharing needs external synchronization
    // outside realtime code.
    struct Telemetry {
        double force_n = 0.0, gap_m = 0.0;
    };

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

    // R1 allocation-free face. compute_into is the checked convenience
    // boundary — std::invalid_argument on a null sink, a null mjData, or a
    // wrong output width — then runs qfrc()'s arithmetic verbatim on the
    // caller-owned span. try_compute_into is the prevalidated warm-core
    // entry: identical preconditions reported through CoreStatus, and no
    // throw/string construction or allocation in its own code (residual
    // kernel rejections are caught and mapped — see the .cpp), so the
    // function is proved noexcept.
    void compute_into(const mjData *d, std::span<double> out) const;
    [[nodiscard]] CoreStatus
    try_compute_into(const mjData *d, std::span<double> out)
            const noexcept BIKE_NONBLOCKING;

    // self.active / self._paths / the live per-path telemetry — the
    // runtime's stored-terms loop reads _paths directly
    // (physical_observations.py:89-93) and the loss ledger reads
    // force_n/gap_m after apply().
    [[nodiscard]] bool active() const noexcept { return !paths_.empty(); }
    [[nodiscard]] std::span<const Path>
    paths() const noexcept { return paths_; }
    [[nodiscard]] std::span<const Telemetry>
    path_telemetry() const noexcept { return last_; }

private:
    // qfrc()'s body verbatim on caller-owned storage: zero-fill, then the
    // per-path spring-damper dofadr writes (assign, not accumulate) and
    // the live last_ telemetry updates. Kept throwing so both boundaries
    // share one kernel and the original rejection types reach the caller.
    void accumulate_validated(const mjData *d, std::span<double> out) const;

    std::vector<Path> paths_;
    mjtSize nq_ = 0, nv_ = 0;
    mutable std::vector<Telemetry> last_;
    // qfrc()'s output buffer — the former per-call zeros(nv) vector moved
    // to a construction-sized member; compute_into fills it, the returned
    // copy is the Python-boxing allocation. Never resized per tick.
    mutable std::vector<double> out_;
};
