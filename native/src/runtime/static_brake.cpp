// runtime/static_brake.cpp — StaticBrakeApplier port; see the header for
// the contract. Expression order follows static_braking.py literally.
#include "static_brake.hpp"

#include <algorithm>
#include <stdexcept>

#include "../engine_call.hpp"
#include "../model_access.hpp"
#include "../validation.hpp"

namespace runtime {

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) dof order mirrors StaticBrakeApplier
StaticBrake::StaticBrake(int front_dofadr, int rear_dofadr,
                         double ceiling_nm)
    : front_(front_dofadr), rear_(rear_dofadr),
      ceiling_nm_(
          validation::nonnegative(ceiling_nm, "brake torque ceiling")) {
    if (std::min(front_, rear_) < 0 || front_ == rear_)
        throw std::invalid_argument(
            "brakes require two distinct valid wheel DOF addresses");
}

void StaticBrake::apply(mjModel *model, double front_demand,
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
    // ctor guarantees 0 <= front_/rear_ and the nv bound ran above, so the
    // size_t casts below can only wrap an already-rejected index.
    frictionloss[static_cast<std::size_t>(front_)] =
        ceiling_nm_ * std::clamp(front, 0., 1.);
    frictionloss[static_cast<std::size_t>(rear_)] =
        ceiling_nm_ * std::clamp(rear, 0., 1.);
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
