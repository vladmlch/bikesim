// runtime/static_brake.cpp — StaticBrakeApplier port; see the header for
// the contract. Expression order follows static_braking.py literally.
#include "static_brake.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <span>
#include <stdexcept>
#include <tuple>

#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../physics_core/brake_hold.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../validation.hpp"

namespace runtime {

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) dof order mirrors StaticBrakeApplier
StaticBrake::StaticBrake(int front_dofadr, int rear_dofadr,
                         double ceiling_nm, BrakeHoldSpec hold)
    : front_(front_dofadr), rear_(rear_dofadr),
      ceiling_nm_(
          validation::nonnegative(ceiling_nm, "brake torque ceiling")),
      hold_{
          .torque_nm =
              validation::nonnegative(hold.torque_nm, "brake hold torque"),
          .stiffness_nm_rad = validation::nonnegative(
              hold.stiffness_nm_rad, "brake hold stiffness"),
          .damping_nms_rad = validation::nonnegative(
              hold.damping_nms_rad, "brake hold damping"),
          .engage_radps = validation::positive(
              hold.engage_radps, "brake hold engage speed"),
          .release_radps = validation::nonnegative(
              hold.release_radps, "brake hold release speed"),
      } {
    if (std::min(front_, rear_) < 0 || front_ == rear_)
        throw std::invalid_argument(
            "brakes require two distinct valid wheel DOF addresses");
    if (hold_.release_radps < hold_.engage_radps)
        throw std::invalid_argument(
            "brake hold release must not precede engage");
}

bool StaticBrake::apply(mjModel *model, double front_demand,
                        double rear_demand) const {
    const double front =
        validation::finite(front_demand, "front brake demand");
    const double rear = validation::finite(rear_demand, "rear brake demand");
    if (std::max(front_, rear_) >= model->nv)
        throw std::invalid_argument("brake DOF is outside this model");
    const auto frictionloss = model_access::mutable_buffer(
        model->dof_frictionloss, model->nv, "dof_frictionloss");
    // Validate both before writing either. These are bounds on solved
    // static friction, not a predetermined signed torque or a velocity
    // reset.
    bool changed = false;
    auto err = bike::physics_core::ErrCode::none;
    const auto st = bike::physics_core::apply_static_brake<
        double, bike::physics_core::CpuExactToolkit>(
        ceiling_nm_, front, rear, front_, rear_,
        static_cast<bike::physics_core::i32>(model->nv),
        frictionloss.data(), changed, err);
    if (st != bike::physics_core::Status::ok)
        throw std::invalid_argument("invalid static brake state");
    return changed;
}

std::vector<double> StaticBrake::hold_qfrc(
    const mjModel *model, const mjData *data, double front_demand,
    double rear_demand, bool advance, bool reseat) {
    const double front =
        validation::finite(front_demand, "front brake demand");
    const double rear =
        validation::finite(rear_demand, "rear brake demand");
    std::vector<double> out(static_cast<std::size_t>(model->nv), 0.);
    const auto dof_jntid = model_access::readonly_buffer(
        model->dof_jntid, model->nv, "dof_jntid");
    const auto jnt_qposadr = model_access::readonly_buffer(
        model->jnt_qposadr, model->njnt, "jnt_qposadr");
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model->nv, "qvel");
    const bike::physics_core::BrakeHoldParams<double> hold_params{
        .torque_nm = hold_.torque_nm,
        .stiffness_nm_rad = hold_.stiffness_nm_rad,
        .damping_nms_rad = hold_.damping_nms_rad,
        .engage_radps = hold_.engage_radps,
        .release_radps = hold_.release_radps,
    };
    for (const auto &[dof, demand, latch] :
         {std::tuple{front_, front, &latch_front_},
          std::tuple{rear_, rear, &latch_rear_}}) {
        const std::size_t di = static_cast<std::size_t>(dof);
        const double theta = qpos[static_cast<std::size_t>(
            jnt_qposadr[static_cast<std::size_t>(dof_jntid[di])])];
        const double omega = qvel[di];
        bool has_latch = latch->has_value();
        double latch_theta = latch->value_or(0.0);
        double out_torque = 0.0;
        auto err = bike::physics_core::ErrCode::none;
        const auto st = bike::physics_core::eval_brake_hold_wheel<
            double, bike::physics_core::CpuExactToolkit>(
            &hold_params, demand, theta, omega, advance, reseat,
            has_latch, latch_theta, out_torque, err);
        if (st != bike::physics_core::Status::ok)
            throw std::invalid_argument("non-finite wheel state for brake hold");
        *latch = has_latch ? std::optional<double>(latch_theta) : std::nullopt;
        if (!has_latch) continue;
        out[di] = out_torque;
    }
    return out;
}

void StaticBrake::reset() {
    latch_front_ = std::nullopt;
    latch_rear_ = std::nullopt;
}

void StaticBrake::restore(const State &state) {
    // Both angles validate before either commits — a rejected rear must
    // not orphan an already-written front.
    if (state.latch_front.has_value())
        validation::finite(*state.latch_front, "front latch angle");
    if (state.latch_rear.has_value())
        validation::finite(*state.latch_rear, "rear latch angle");
    latch_front_ = state.latch_front;
    latch_rear_ = state.latch_rear;
}

std::pair<std::vector<double>, std::vector<double>>
StaticBrake::solved_components(const mjModel *model,
                               const mjData *data) const {
    const auto efc_type = model_access::readonly_buffer(
        data->efc_type, data->nefc, "efc_type");
    const auto efc_id = model_access::readonly_buffer(
        data->efc_id, data->nefc, "efc_id");
    const auto efc_force = model_access::readonly_buffer(
        data->efc_force, data->nefc, "efc_force");
    std::vector<double> multipliers(
        static_cast<std::size_t>(data->nefc), 0.);
    std::pair<std::vector<double>, std::vector<double>> result{
        std::vector<double>(static_cast<std::size_t>(model->nv), 0.),
        std::vector<double>(static_cast<std::size_t>(model->nv), 0.)};
    for (const auto &[dof, out] : {std::pair{front_, &result.first},
                                   std::pair{rear_, &result.second}}) {
        std::ranges::fill(multipliers, 0.);
        bool any = false;
        for (std::size_t row = 0; row < multipliers.size(); ++row)
            if (efc_type[row] == mjCNSTR_FRICTION_DOF &&
                efc_id[row] == dof) {
                multipliers[row] = efc_force[row];
                any = true;
            }
        if (any)
            // Engine helper supports either dense or sparse EFC Jacobians.
            engine::mul_jac_t_vec(model, data, out->data(),
                                  multipliers.data());
    }
    return result;
}

} // namespace runtime
