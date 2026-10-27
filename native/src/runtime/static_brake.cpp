// runtime/static_brake.cpp — StaticBrakeApplier port; see the header for
// the contract. Expression order follows static_braking.py literally.
#include "static_brake.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <span>
#include <stdexcept>
#include <tuple>

#include "../engine_call.hpp"
#include "../model_access.hpp"
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
    // ctor guarantees 0 <= front_/rear_ and the nv bound ran above, so the
    // size_t casts below can only wrap an already-rejected index.
    const double front_loss = ceiling_nm_ * std::clamp(front, 0., 1.);
    const double rear_loss = ceiling_nm_ * std::clamp(rear, 0., 1.);
    const std::size_t fi = static_cast<std::size_t>(front_);
    const std::size_t ri = static_cast<std::size_t>(rear_);
    // dof_frictionloss is a position-stage model input (makeConstraint
    // builds the FRICTION_DOF rows from it) — report whether this write
    // changed a value so the caller can invalidate cached stage products.
    const bool changed =
        frictionloss[fi] != front_loss || frictionloss[ri] != rear_loss;
    frictionloss[fi] = front_loss;
    frictionloss[ri] = rear_loss;
    return changed;
}

std::vector<double> StaticBrake::hold_qfrc(
    const mjModel *model, const mjData *data, double front_demand,
    double rear_demand, bool advance) {
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
    for (const auto &[dof, demand, latch] :
         {std::tuple{front_, front, &latch_front_},
          std::tuple{rear_, rear, &latch_rear_}}) {
        const double u = std::clamp(demand, 0., 1.);
        const std::size_t di = static_cast<std::size_t>(dof);
        const double theta = qpos[static_cast<std::size_t>(
            jnt_qposadr[static_cast<std::size_t>(dof_jntid[di])])];
        const double omega = qvel[di];
        std::optional<double> latched = *latch;
        if (advance) {
            if (!latched.has_value()) {
                if (u > 0. && hold_.torque_nm > 0. &&
                    hold_.stiffness_nm_rad > 0. &&
                    std::abs(omega) <= hold_.engage_radps) {
                    latched = theta;
                    *latch = latched;
                }
            } else if (u <= 0. || std::abs(omega) > hold_.release_radps) {
                latched = std::nullopt;
                *latch = std::nullopt;
            }
        }
        if (!latched.has_value()) continue;
        const double cap = hold_.torque_nm * u;
        out[di] = std::clamp(
            -hold_.stiffness_nm_rad * (theta - *latched) -
                hold_.damping_nms_rad * omega,
            -cap, cap);
    }
    return out;
}

SpatialBrakeResult StaticBrake::apply_spatial(
    mjModel* model, std::span<const double, 2> rates, double front_demand,
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) call sites keep the declared (front, rear) parameter order
    double rear_demand, double dt_s, std::int64_t interval, bool advance,
    std::span<double> qfrc) {
    if (model == nullptr || model->nv <= std::max(front_, rear_) ||
        qfrc.size() != static_cast<std::size_t>(model->nv))
        throw std::invalid_argument("spatial.brake: model/force dimensions mismatch");
    const auto frictionloss=model_access::mutable_buffer(
        model->dof_frictionloss,model->nv,"dof_frictionloss");
    const std::array demand{
        std::clamp(validation::finite(front_demand,"spatial.brake.front_demand"),0.,1.),
        std::clamp(validation::finite(rear_demand,"spatial.brake.rear_demand"),0.,1.)};
    validation::nonnegative(dt_s,"spatial.brake.dt_s");
    for (const double rate:rates) validation::finite(rate,"spatial.brake.relative_rate_radps");
    if (advance && (dt_s == 0. || spatial_.interval_index ==
            std::numeric_limits<std::int64_t>::max() || interval != spatial_.interval_index+1))
        throw std::invalid_argument("spatial.brake.interval_index: duplicate or nonsequential advance");
    auto staged=spatial_;
    SpatialBrakeResult result;
    const std::array dofs{front_,rear_};
    for (std::size_t i=0;i<2;++i) {
        auto& w=staged.wheel[i];
        const double rate=rates[i], u=demand[i], total=ceiling_nm_*u;
        if (advance) w.unwrapped_angle_rad += rate*dt_s;
        const double theta=w.unwrapped_angle_rad;
        if (!std::isfinite(theta))
            throw std::invalid_argument("spatial.brake.unwrapped_angle_rad: overflow");
        // The switch takes effect in force probes as well, but probe history is
        // discarded. Thus releasing a lever never retains a stale hold force.
        if (w.engaged && (u==0. || std::abs(rate)>hold_.release_radps)) {
            if (advance) result.release_heat_j+=w.stored_energy_j;
            w.engaged=false;w.stored_energy_j=0.;
        }
        if (!w.engaged && u>0. && hold_.torque_nm>0. &&
            hold_.stiffness_nm_rad>0. && std::abs(rate)<=hold_.engage_radps) {
            w.engaged=true;w.latch_angle_rad=theta;w.stored_energy_j=0.;
        }
        double torque=0.;
        if (w.engaged) {
            const double cap=std::min(total,hold_.torque_nm*u);
            const double previous_energy=w.stored_energy_j;
            const double e0=theta-w.latch_angle_rad;
            const double e=std::clamp(e0,-cap/hold_.stiffness_nm_rad,
                                            cap/hold_.stiffness_nm_rad);
            // Yield the internal pad reference at its torque limit. The energy
            // of the excess elastic deflection becomes heat, never root work.
            if (advance && e!=e0) {
                w.latch_angle_rad=theta-e;
            }
            const double spring=-hold_.stiffness_nm_rad*e;
            const double damper=std::clamp(-hold_.damping_nms_rad*rate,
                                         -cap-spring,cap-spring);
            torque=spring+damper;
            w.stored_energy_j=.5*hold_.stiffness_nm_rad*e*e;
            if (advance) {
                // Endpoint work minus the retained spring potential includes
                // pad yield and the nonnegative implicit-integration loss.
                // In particular, a saturated pad dissipates torque*slip even
                // while its elastic energy remains constant.
                result.damping_heat_j+=std::max(0.,-spring*rate*dt_s-
                    (w.stored_energy_j-previous_energy));
                result.damping_heat_j+=std::max(0.,-damper*rate)*dt_s;
            }
        }
        w.previous_demand=u;
        result.hold_torque_nm[i]=torque;
        result.friction_budget_nm[i]=std::max(0.,total-std::abs(torque));
        result.stored_energy_j+=w.stored_energy_j;
        if (!std::isfinite(torque) || !std::isfinite(result.friction_budget_nm[i]) ||
            !std::isfinite(result.stored_energy_j) || !std::isfinite(result.damping_heat_j) ||
            !std::isfinite(result.release_heat_j) ||
            !std::isfinite(qfrc[static_cast<std::size_t>(dofs[i])]+torque))
            throw std::invalid_argument("spatial.brake: nonfinite staged force or energy");
    }
    // Both wheels have now passed validation: no partial model/latch write.
    for (std::size_t i=0;i<2;++i) {
        const auto dof=static_cast<std::size_t>(dofs[i]);
        result.frictionloss_changed|=frictionloss[dof]!=result.friction_budget_nm[i];
        frictionloss[dof]=result.friction_budget_nm[i];
        qfrc[dof]+=result.hold_torque_nm[i];
    }
    if (advance) { staged.interval_index=interval;staged.valid_mask=3;spatial_=staged; }
    return result;
}

bike::spatial::BrakeState StaticBrake::capture_spatial_brake() const noexcept { return spatial_; }
void StaticBrake::restore_spatial_brake(const bike::spatial::BrakeState& state) {
    // Cold validation stages a complete owned value before publishing it.
    const auto staged=bike::spatial::decode_brake(bike::spatial::encode_brake(state));
    for (const auto& w:staged.wheel) {
        // A latch can only have been created under a spec with a positive pad
        // stiffness and torque; apply_spatial divides by the stiffness while a
        // wheel is engaged, so such a state must be rejected here, cold.
        if (w.engaged && !(hold_.stiffness_nm_rad>0. && hold_.torque_nm>0.))
            throw std::invalid_argument("spatial.brake.engaged: hold spec cannot latch a pad");
        const double expected=w.engaged ? .5*hold_.stiffness_nm_rad*
            (w.unwrapped_angle_rad-w.latch_angle_rad)*
            (w.unwrapped_angle_rad-w.latch_angle_rad) : 0.;
        if (!std::isfinite(expected) || std::abs(expected-w.stored_energy_j)>
            1e-10*std::max(1.,expected))
            throw std::invalid_argument("spatial.brake.stored_energy_j: latch energy differs from hold stiffness");
    }
    commit_spatial_brake(staged);
}

void StaticBrake::commit_spatial_brake(const bike::spatial::BrakeState& state) noexcept {
    spatial_=state;
}

void StaticBrake::reset() {
    latch_front_ = std::nullopt;
    latch_rear_ = std::nullopt;
    spatial_ = bike::spatial::canonical_released_brake();
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
