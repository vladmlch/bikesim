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
#include "../validation.hpp"

namespace runtime {

namespace {
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) front/rear DOF + loss pair
bool write_frictionloss(const mjModel *model, int front_dof, int rear_dof,
                        double front_loss, double rear_loss) {
    if (model == nullptr)
        throw std::invalid_argument("brake model must not be null");
    if (std::max(front_dof, rear_dof) >= model->nv)
        throw std::invalid_argument("brake DOF is outside this model");
    const auto frictionloss = model_access::mutable_buffer(
        model->dof_frictionloss, model->nv, "dof_frictionloss");
    // ctor guarantees 0 <= front_/rear_ and the nv bound ran above, so the
    // size_t casts below can only wrap an already-rejected index.
    const std::size_t fi = static_cast<std::size_t>(front_dof);
    const std::size_t ri = static_cast<std::size_t>(rear_dof);
    // dof_frictionloss is a position-stage model input (makeConstraint
    // builds the FRICTION_DOF rows from it) — report whether this write
    // changed a value so the caller can invalidate cached stage products.
    const bool changed =
        frictionloss[fi] != front_loss || frictionloss[ri] != rear_loss;
    frictionloss[fi] = front_loss;
    frictionloss[ri] = rear_loss;
    return changed;
}
} // namespace

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
          .unified_ceiling = hold.unified_ceiling,
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
    // Validate both before writing either. These are bounds on solved
    // static friction, not a predetermined signed torque or a velocity
    // reset.
    const double front_loss = ceiling_nm_ * std::clamp(front, 0., 1.);
    const double rear_loss = ceiling_nm_ * std::clamp(rear, 0., 1.);
    return write_frictionloss(model, front_, rear_, front_loss, rear_loss);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) paired front/rear demands and hold torques
bool StaticBrake::apply_remaining_budget(
    mjModel *model, double front_demand, double rear_demand,
    double front_hold_torque_nm, double rear_hold_torque_nm) const {
    const double front =
        validation::finite(front_demand, "front brake demand");
    const double rear = validation::finite(rear_demand, "rear brake demand");
    const double front_hold =
        validation::finite(front_hold_torque_nm, "front brake hold torque");
    const double rear_hold =
        validation::finite(rear_hold_torque_nm, "rear brake hold torque");
    const double front_loss = std::max(
        0.0, ceiling_nm_ * std::clamp(front, 0., 1.) - std::abs(front_hold));
    const double rear_loss = std::max(
        0.0, ceiling_nm_ * std::clamp(rear, 0., 1.) - std::abs(rear_hold));
    return write_frictionloss(model, front_, rear_, front_loss, rear_loss);
}

bool StaticBrake::apply_remaining_budget(
    mjModel *model, double front_demand, double rear_demand,
    std::span<const double> hold_qfrc) const {
    if (model == nullptr)
        throw std::invalid_argument("brake model must not be null");
    if (std::max(front_, rear_) >= model->nv)
        throw std::invalid_argument("brake DOF is outside this model");
    if (hold_qfrc.size() != static_cast<std::size_t>(model->nv))
        throw std::invalid_argument("hold_qfrc size does not match model->nv");
    return apply_remaining_budget(
        model, front_demand, rear_demand,
        hold_qfrc[static_cast<std::size_t>(front_)],
        hold_qfrc[static_cast<std::size_t>(rear_)]);
}

std::array<double, 2> StaticBrake::hold_wheel_torques(
    const mjModel *model, const mjData *data, double front_demand,
    double rear_demand, bool advance, bool reseat) {
    if (model == nullptr || data == nullptr)
        throw std::invalid_argument(
            "brake hold requires non-null model and data");
    if (std::max(front_, rear_) >= model->nv)
        throw std::invalid_argument("brake DOF is outside this model");
    const double front =
        validation::finite(front_demand, "front brake demand");
    const double rear =
        validation::finite(rear_demand, "rear brake demand");
    const auto dof_jntid = model_access::readonly_buffer(
        model->dof_jntid, model->nv, "dof_jntid");
    const auto jnt_qposadr = model_access::readonly_buffer(
        model->jnt_qposadr, model->njnt, "jnt_qposadr");
    const auto qpos = model_access::readonly_buffer(
        data->qpos, model->nq, "qpos");
    const auto qvel = model_access::readonly_buffer(
        data->qvel, model->nv, "qvel");
    const double max_hold_nm =
        hold_.unified_ceiling ? std::min(hold_.torque_nm, ceiling_nm_)
                              : hold_.torque_nm;
    std::array<double, 2> torques{};
    std::size_t side = 0;
    for (const auto &[dof, demand, latch] :
         {std::tuple{front_, front, &latch_front_},
          std::tuple{rear_, rear, &latch_rear_}}) {
        const double u = std::clamp(demand, 0., 1.);
        const double cap = max_hold_nm * u;
        const std::size_t di = static_cast<std::size_t>(dof);
        const double theta = qpos[static_cast<std::size_t>(
            jnt_qposadr[static_cast<std::size_t>(dof_jntid[di])])];
        const double omega = qvel[di];
        std::optional<double> latched = *latch;
        if (advance) {
            if (!latched.has_value()) {
                if (u > 0. && max_hold_nm > 0. &&
                    hold_.stiffness_nm_rad > 0. &&
                    std::abs(omega) <= hold_.engage_radps) {
                    latched = theta;
                    *latch = latched;
                }
            } else if (u <= 0. || std::abs(omega) > hold_.release_radps) {
                latched = std::nullopt;
                *latch = std::nullopt;
            } else if (reseat && std::abs(omega) <= hold_.engage_radps &&
                       std::abs(-hold_.stiffness_nm_rad * (theta - *latched) -
                                hold_.damping_nms_rad * omega) > cap) {
                // A saturated bite inside the engage band is a slipped pad:
                // re-seat the anchor at the current angle (the oracle's
                // `reseat` branch — static_braking.py).
                latched = theta;
                *latch = latched;
            }
        }
        if (latched.has_value()) {
            torques[side] = std::clamp(
                -hold_.stiffness_nm_rad * (theta - *latched) -
                    hold_.damping_nms_rad * omega,
                -cap, cap);
        }
        ++side;
    }
    return torques;
}

std::vector<double> StaticBrake::hold_qfrc(
    const mjModel *model, const mjData *data, double front_demand,
    double rear_demand, bool advance, bool reseat) {
    const auto torques = hold_wheel_torques(
        model, data, front_demand, rear_demand, advance, reseat);
    std::vector<double> out(static_cast<std::size_t>(model->nv), 0.);
    out[static_cast<std::size_t>(front_)] = torques[0];
    out[static_cast<std::size_t>(rear_)] = torques[1];
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
