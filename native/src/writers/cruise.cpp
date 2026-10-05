#include "cruise.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>

namespace {
    void finite(double value, const char *name) {
        if (!std::isfinite(value))
            throw std::invalid_argument(std::string(name) + " must be finite");
    }

    void positive(double value, const char *name) {
        finite(value, name);
        if (value <= 0.0)
            throw std::invalid_argument(std::string(name) + " must be positive");
    }

    double clamp(double value, double limit) {
        // Python keeps the first argument on ties (including signed zero).
        return std::max(-limit, std::min(limit, value));
    }
} // namespace

CruiseWriter::CruiseWriter(const mjModel *model, nativecfg::CruiseConfig config)
    : cfg_(config), nv_(model->nv),
      timestep_(model->opt.timestep), state_{.target_speed_mps = 0.0} {
    positive(cfg_.kp_nm_per_mps, "kp_nm_per_mps");
    positive(cfg_.ki_nm_per_mps_s, "ki_nm_per_mps_s");
    positive(cfg_.torque_ceiling_nm, "torque_ceiling_nm");
    set_target_speed(cfg_.target_speed_kmh);
    const int joint = mj_name2id(model, mjOBJ_JOINT, "root_x");
    if (joint < 0)
        throw std::invalid_argument("model has no joint 'root_x'");
    const std::span<const int> types =
            std::views::counted(model->jnt_type, model->njnt);
    const auto index = static_cast<std::size_t>(joint);
    if (types[index] != mjJNT_SLIDE && types[index] != mjJNT_HINGE)
        throw std::invalid_argument("root_x must be a scalar slide or hinge joint");
    const std::span<const int> dofs =
            std::views::counted(model->jnt_dofadr, model->njnt);
    root_dof_ = dofs[index];
}

double CruiseWriter::compute(const mjData *data, bool rear_in_contact,
                             bool traction_limited,
                             std::optional<bool> controller_grounded) {
    // cruise.py compute: preserve every operation and branch in source order.
    state_.engaged = controller_grounded.value_or(rear_in_contact);
    const std::span<const mjtNum> qvel = std::views::counted(data->qvel, nv_);
    const double error = state_.target_speed_mps -
                         qvel[static_cast<std::size_t>(root_dof_)];
    if (!state_.engaged) {
        state_.torque_nm = 0.0;
        return state_.torque_nm;
    }
    const double kp = cfg_.kp_nm_per_mps * state_.gain_scale;
    const double ki = cfg_.ki_nm_per_mps_s * state_.gain_scale;
    const double proportional = kp * error;
    double demand = proportional + ki * state_.integral_mps_s;
    const bool pushing_further =
            std::abs(demand) >= cfg_.torque_ceiling_nm &&
            (demand > 0.0) == (error > 0.0);
    if (!pushing_further && !traction_limited) {
        state_.integral_mps_s = clamp(state_.integral_mps_s + error * timestep_,
                                      cfg_.torque_ceiling_nm / ki);
        demand = proportional + ki * state_.integral_mps_s;
    }
    state_.torque_nm = clamp(demand, cfg_.torque_ceiling_nm);
    return state_.torque_nm;
}

void CruiseWriter::reset() {
    state_.integral_mps_s = 0.0;
    state_.torque_nm = 0.0;
    state_.engaged = false;
}

void CruiseWriter::set_target_speed(double value_kmh) {
    finite(value_kmh, "target_speed_kmh");
    if (value_kmh < 15.0 || value_kmh > 45.0)
        throw std::invalid_argument("target_speed_kmh must be in [15, 45]");
    state_.target_speed_mps = value_kmh / 3.6;
}

void CruiseWriter::validate_scale(double scale) const {
    positive(scale, "gain_scale");
    const double ki = cfg_.ki_nm_per_mps_s * scale;
    if (!std::isfinite(ki) || ki == 0.0)
        throw std::invalid_argument("gain_scale produces zero or nonfinite scaled ki");
}

double CruiseWriter::set_assist_compensation(double support_factor) {
    finite(support_factor, "support_factor");
    const double scale = 1.0 / (1.0 + std::max(0.0, support_factor));
    validate_scale(scale);
    state_.gain_scale = scale;
    return scale;
}

void CruiseWriter::set_state(const CruiseState &state) {
    finite(state.target_speed_mps, "target_speed_mps");
    finite(state.integral_mps_s, "integral_mps_s");
    finite(state.torque_nm, "torque_nm");
    if (state.target_speed_mps < 15.0 / 3.6 ||
        state.target_speed_mps > 45.0 / 3.6)
        throw std::invalid_argument("target_speed_mps must be in [15/3.6, 45/3.6]");
    validate_scale(state.gain_scale);
    state_ = state;
}
