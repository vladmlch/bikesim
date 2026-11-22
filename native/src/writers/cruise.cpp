#include "cruise.hpp"
#include "../config_validation.hpp"
#include "../model_topology.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/cruise.hpp"

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

    bike::physics_core::CruiseConfigPod<double>
    to_pod(const nativecfg::CruiseConfig &cfg) noexcept {
        return {
            .target_speed_kmh = cfg.target_speed_kmh,
            .kp_nm_per_mps = cfg.kp_nm_per_mps,
            .ki_nm_per_mps_s = cfg.ki_nm_per_mps_s,
            .torque_ceiling_nm = cfg.torque_ceiling_nm,
        };
    }

    bike::physics_core::CruiseStatePod<double>
    to_pod(const CruiseState &s) noexcept {
        return {
            .target_speed_mps = s.target_speed_mps,
            .integral_mps_s = s.integral_mps_s,
            .torque_nm = s.torque_nm,
            .gain_scale = s.gain_scale,
            .engaged = s.engaged ? bike::physics_core::u8{1}
                                 : bike::physics_core::u8{0},
        };
    }

    CruiseState from_pod(const bike::physics_core::CruiseStatePod<double> &s) noexcept {
        return {
            .target_speed_mps = s.target_speed_mps,
            .integral_mps_s = s.integral_mps_s,
            .torque_nm = s.torque_nm,
            .engaged = s.engaged != 0,
            .gain_scale = s.gain_scale,
        };
    }
} // namespace

CruiseWriter::CruiseWriter(const mjModel *model, nativecfg::CruiseConfig config)
    : cfg_(config), model_(model), nv_(model->nv),
      state_{.target_speed_mps = 0.0} {
    // The shared validator owns the config domain (positive kp/ki/ceiling,
    // finite target speed in [15, 45]) — checked before any use here.
    nativecfg::validate(cfg_);
    set_target_speed(cfg_.target_speed_kmh);
    const int joint = mj_name2id(model, mjOBJ_JOINT, "root_x");
    if (joint < 0)
        throw std::invalid_argument("model has no joint 'root_x'");
    const auto index = static_cast<std::size_t>(joint);
    topology::joint(model, joint, "CruiseWriter.root_x", mjJNT_SLIDE, {1., 0., 0.});
    const std::span<const int> dofs =
            std::views::counted(model->jnt_dofadr, model->njnt);
    root_dof_ = dofs[index];
}

double CruiseWriter::compute(const mjData *data, bool rear_in_contact,
                             bool traction_limited,
                             std::optional<bool> controller_grounded) {
    const auto cfg_pod = to_pod(cfg_);
    auto state_pod = to_pod(state_);
    const std::span<const mjtNum> qvel = std::views::counted(data->qvel, nv_);
    const double speed_mps =
            static_cast<double>(qvel[static_cast<std::size_t>(root_dof_)]);
    const double timestep = static_cast<double>(model_->opt.timestep);
    double out_torque_nm = 0.0;
    auto err = bike::physics_core::ErrCode::none;
    const auto st = bike::physics_core::cruise_step<
        double, bike::physics_core::CpuExactToolkit>(
        &cfg_pod,
        &state_pod,
        speed_mps,
        timestep,
        rear_in_contact,
        traction_limited,
        controller_grounded.has_value(),
        controller_grounded.value_or(false),
        out_torque_nm,
        err);
    if (st == bike::physics_core::Status::overflow) {
        throw std::overflow_error("CruiseWriter arithmetic overflow");
    }
    if (st != bike::physics_core::Status::ok) {
        if (err == bike::physics_core::ErrCode::bad_dt) {
            throw std::invalid_argument("CruiseWriter.timestep must be positive");
        }
        throw std::invalid_argument("CruiseWriter.ki must be positive");
    }
    state_ = from_pod(state_pod);
    return out_torque_nm;
}

void CruiseWriter::reset() {
    auto state_pod = to_pod(state_);
    bike::physics_core::cruise_reset<double>(&state_pod);
    state_ = from_pod(state_pod);
}

void CruiseWriter::set_target_speed(double value_kmh) {
    finite(value_kmh, "target_speed_kmh");
    auto state_pod = to_pod(state_);
    auto err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::cruise_set_target_speed_kmh<
            double, bike::physics_core::CpuExactToolkit>(
            &state_pod, value_kmh, err) != bike::physics_core::Status::ok) {
        throw std::invalid_argument("target_speed_kmh must be in [15, 45]");
    }
    state_ = from_pod(state_pod);
}

void CruiseWriter::validate_scale(double scale) const {
    finite(scale, "gain_scale");
    if (scale <= 0.0)
        throw std::invalid_argument("gain_scale must be positive");
    const auto cfg_pod = to_pod(cfg_);
    auto err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::cruise_validate_scale<
            double, bike::physics_core::CpuExactToolkit>(
            &cfg_pod, scale, err) != bike::physics_core::Status::ok) {
        throw std::invalid_argument("gain_scale produces zero or nonfinite scaled ki");
    }
}

double CruiseWriter::set_assist_compensation(double support_factor) {
    finite(support_factor, "support_factor");
    const auto cfg_pod = to_pod(cfg_);
    auto state_pod = to_pod(state_);
    double scale = 1.0;
    auto err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::cruise_set_assist_compensation<
            double, bike::physics_core::CpuExactToolkit>(
            &cfg_pod, &state_pod, support_factor, scale, err) !=
        bike::physics_core::Status::ok) {
        throw std::invalid_argument("gain_scale produces zero or nonfinite scaled ki");
    }
    state_ = from_pod(state_pod);
    return scale;
}

void CruiseWriter::set_state(const CruiseState &next_state) {
    finite(next_state.target_speed_mps, "target_speed_mps");
    finite(next_state.integral_mps_s, "integral_mps_s");
    finite(next_state.torque_nm, "torque_nm");
    finite(next_state.gain_scale, "gain_scale");
    const auto cfg_pod = to_pod(cfg_);
    const auto src_pod = to_pod(next_state);
    auto dst_pod = to_pod(state_);
    auto err = bike::physics_core::ErrCode::none;
    if (bike::physics_core::cruise_restore_state<
            double, bike::physics_core::CpuExactToolkit>(
            &cfg_pod, &dst_pod, &src_pod, err) !=
        bike::physics_core::Status::ok) {
        if (next_state.target_speed_mps < 15.0 / 3.6 ||
            next_state.target_speed_mps > 45.0 / 3.6) {
            throw std::invalid_argument(
                "target_speed_mps must be in [15/3.6, 45/3.6]");
        }
        validate_scale(next_state.gain_scale);
        throw std::invalid_argument("invalid CruiseState");
    }
    state_ = from_pod(dst_pod);
}
