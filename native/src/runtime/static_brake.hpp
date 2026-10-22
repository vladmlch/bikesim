// runtime/static_brake.hpp — port of StaticBrakeApplier
// (src/bike_sim/sim/ride/static_braking.py).
//
// Physical braking is a model-level static friction constraint:
// dof_frictionloss bounds on the two wheel DOFs, applied to mjModel, plus
// the solved mjCNSTR_FRICTION_DOF generalized forces read back from
// mjData. The legacy BrakeWriter (actuator-ctrl semantics) is a different
// operation and remains dedicated to legacy mode.
//
// At near-zero wheel speed a latched elastic bite models pad stiction:
// the regularized friction bound alone still creeps under a sustained
// load, but a real held brake pins the wheel angle. hold_qfrc captures
// the wheel angle while the lever is pulled and |omega| <= engage,
// applies a clipped spring-damper about it, and forgets it on release
// (lever freed or |omega| > release, the hysteresis that keeps a rolling
// wheel on the friction bound alone).
#pragma once
#include <mujoco/mujoco.h>

#include <optional>
#include <utility>
#include <vector>

namespace runtime {

// Hold parameters of StaticBrakeApplier's keyword-only constructor
// arguments — grouped so the decoder/factory pass one aggregate.
struct BrakeHoldSpec {
    double torque_nm = 0.;
    double stiffness_nm_rad = 0.;
    double damping_nms_rad = 0.;
    double engage_radps = .5;
    double release_radps = 1.;
};

class StaticBrake {
public:
    // static_braking.py:24-38 — distinct nonnegative DOF addresses, a
    // nonnegative ceiling and the bounded hold spec; the owning model's nv
    // bound is enforced per apply() like the Python class.
    StaticBrake(int front_dofadr, int rear_dofadr, double ceiling_nm,
                BrakeHoldSpec hold = {});

    // apply(model, data, front_demand, rear_demand): validate both demands
    // before either write, then
    //   dof_frictionloss[dof] = ceiling * clip(demand, 0, 1)
    // front first — the Python statement order. Returns whether either
    // bound changed (a position-stage model input; the caller's freshness
    // guard needs it).
    [[nodiscard]] bool apply(mjModel *model, double front_demand,
                             double rear_demand) const;

    // hold_qfrc(model, data, front_demand, rear_demand, advance):
    // the bite's generalized force (nv zeros when unlatched). An
    // advancing evaluation may latch or release; a probe reads the
    // committed latch without moving it — the Python ``advance``
    // contract verbatim.
    [[nodiscard]] std::vector<double> hold_qfrc(
        const mjModel *model, const mjData *data, double front_demand,
        double rear_demand, bool advance);

    // reset(): forget both captured bite angles; the next standstill
    // re-bites. The oracle's PhysicalRuntime.reset clears them for a
    // fresh episode; here the bootstrap restore does.
    void reset();

    // solved_components: per-brake generalized force of the solved
    // mjCNSTR_FRICTION_DOF constraint rows, via mj_mulJacTVec over the
    // selected multipliers. Zero vector when no row is present.
    // Returns (front_static_brake, rear_static_brake).
    [[nodiscard]] std::pair<std::vector<double>, std::vector<double>>
    solved_components(const mjModel *model, const mjData *data) const;

    [[nodiscard]] int front_dof() const { return front_; }
    [[nodiscard]] int rear_dof() const { return rear_; }
    [[nodiscard]] double ceiling_nm() const { return ceiling_nm_; }
    [[nodiscard]] const BrakeHoldSpec &hold_spec() const { return hold_; }

private:
    int front_, rear_;
    double ceiling_nm_;
    BrakeHoldSpec hold_;
    std::optional<double> latch_front_, latch_rear_;
};

} // namespace runtime
