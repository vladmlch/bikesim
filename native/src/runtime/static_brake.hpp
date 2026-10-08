// runtime/static_brake.hpp — port of StaticBrakeApplier
// (src/bike_sim/sim/ride/static_braking.py).
//
// Physical braking is a model-level static friction constraint:
// dof_frictionloss bounds on the two wheel DOFs, applied to mjModel, plus
// the solved mjCNSTR_FRICTION_DOF generalized forces read back from
// mjData. The legacy BrakeWriter (actuator-ctrl semantics) is a different
// operation and remains dedicated to legacy mode.
#pragma once
#include <mujoco/mujoco.h>

#include <utility>
#include <vector>

namespace runtime {

class StaticBrake {
public:
    // static_braking.py:7-11 — distinct nonnegative DOF addresses and a
    // nonnegative ceiling; the owning model's nv bound is enforced per
    // apply() like the Python class.
    StaticBrake(int front_dofadr, int rear_dofadr, double ceiling_nm);

    // apply(model, data, front_demand, rear_demand): validate both demands
    // before either write, then
    //   dof_frictionloss[dof] = ceiling * clip(demand, 0, 1)
    // front first — the Python statement order.
    void apply(mjModel *model, double front_demand,
               double rear_demand) const;

    // solved_components: per-brake generalized force of the solved
    // mjCNSTR_FRICTION_DOF constraint rows, via mj_mulJacTVec over the
    // selected multipliers. Zero vector when no row is present.
    // Returns (front_static_brake, rear_static_brake).
    [[nodiscard]] std::pair<std::vector<double>, std::vector<double>>
    solved_components(const mjModel *model, const mjData *data) const;

    [[nodiscard]] int front_dof() const { return front_; }
    [[nodiscard]] int rear_dof() const { return rear_; }
    [[nodiscard]] double ceiling_nm() const { return ceiling_nm_; }

private:
    int front_, rear_;
    double ceiling_nm_;
};

} // namespace runtime
