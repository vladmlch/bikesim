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
#include "../spatial/brake_state.hpp"

#include <array>
#include <cstdint>
#include <span>

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

struct SpatialBrakeResult {
    bool frictionloss_changed = false;
    std::array<double, 2> hold_torque_nm{}, friction_budget_nm{};
    // damping_heat includes pad yielding and nonnegative endpoint-integration
    // loss; release_heat records complete lever/hysteresis unlatching.
    double stored_energy_j = 0., damping_heat_j = 0., release_heat_j = 0.;
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

    // Spatial-only, allocation-free hinge bite. relative_rate is front/rear
    // wheel-minus-hub angular velocity about the axle, never world rotation.
    // qfrc is an additive nv-vector; the remainder of each total torque budget
    // is installed as frictionloss. A probe changes no committed latch/history.
    // advance accepts exactly interval_index == previous+1 and integrates angle
    // by relative_rate*dt. The outer accepted-interval transaction owns rollback.
    [[nodiscard]] SpatialBrakeResult apply_spatial(
        mjModel*, std::span<const double, 2> relative_rate_radps,
        double front_demand, double rear_demand, double dt_s,
        std::int64_t interval_index, bool advance, std::span<double> qfrc);
    [[nodiscard]] bike::spatial::BrakeState capture_spatial_brake() const noexcept;
    void restore_spatial_brake(const bike::spatial::BrakeState&);
    // Publish only a state already validated at the cold prepare boundary.
    // This final transaction operation cannot allocate or throw.
    void commit_spatial_brake(const bike::spatial::BrakeState&) noexcept;

    // reset(): forget both captured bite angles; the next standstill
    // re-bites. The oracle's PhysicalRuntime.reset clears them for a
    // fresh episode; here the bootstrap restore does.
    void reset();

    // Scoped suppression of the latched bite — the oracle's
    // StaticBrakeApplier.hold_suspended context manager: a pose solve is
    // not a standstill the latch should remember (its engage/release
    // hysteresis ratchets the wheels while the damped settle cycles run).
    // Zeroes the hold spec for the scope and leaves a forgotten latch.
    class HoldSuspend {
    public:
        explicit HoldSuspend(StaticBrake &brake) noexcept
            : brake_(&brake), saved_(brake.hold_) {
            brake_->hold_.torque_nm = 0.;
            brake_->hold_.stiffness_nm_rad = 0.;
        }
        HoldSuspend(const HoldSuspend &) = delete;
        HoldSuspend &operator=(const HoldSuspend &) = delete;
        HoldSuspend(HoldSuspend &&) = delete;
        HoldSuspend &operator=(HoldSuspend &&) = delete;
        ~HoldSuspend() { brake_->hold_ = saved_; brake_->reset(); }

    private:
        StaticBrake *brake_;
        BrakeHoldSpec saved_;
    };
    [[nodiscard]] HoldSuspend hold_suspend() noexcept { return HoldSuspend{*this}; }

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
    bike::spatial::BrakeState spatial_;
};

} // namespace runtime
