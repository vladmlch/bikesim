#include "launch_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "../spatial/frames.hpp"

namespace runtime {
namespace {

[[nodiscard]] bool all_finite3(const std::array<double, 3> &v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

} // namespace

LaunchSupport::LaunchSupport(const LaunchSupportConfig &config)
    : config_(config) {
    if (config_.root_free_qposadr < 0 || config_.root_free_dofadr < 0) {
        throw std::invalid_argument(
            "launch_support: non-negative root_free qpos/dof addresses required");
    }
    if (!std::isfinite(config_.standstill_speed_mps) ||
        config_.standstill_speed_mps < 0.0) {
        throw std::invalid_argument(
            "launch_support: standstill_speed_mps must be finite and non-negative");
    }
    const auto &s = config_.spec;
    if (!std::isfinite(s.release_speed_mps) || !(s.release_speed_mps > 0.0) ||
        !(s.release_speed_mps > 2.0 * config_.standstill_speed_mps)) {
        throw std::invalid_argument(
            "launch_support: release_speed_mps must be > 2 * standstill_speed_mps");
    }
    if (!std::isfinite(s.lateral_stiffness_n_per_m) ||
        !(s.lateral_stiffness_n_per_m > 0.0) ||
        !std::isfinite(s.lateral_damping_ns_per_m) ||
        s.lateral_damping_ns_per_m < 0.0 ||
        !std::isfinite(s.roll_stiffness_nm_per_rad) ||
        !(s.roll_stiffness_nm_per_rad > 0.0) ||
        !std::isfinite(s.roll_damping_nms_per_rad) ||
        s.roll_damping_nms_per_rad < 0.0) {
        throw std::invalid_argument(
            "launch_support: positive stiffnesses and non-negative dampings required");
    }
    reset(config_.initially_armed);
}

void LaunchSupport::reset(bool initially_armed) noexcept {
    state_.armed = initially_armed;
    state_.reference_latched = false;
    state_.ref_pos_m = {0.0, 0.0, 0.0};
    state_.ref_yaw_rad = 0.0;
}

void LaunchSupport::latch_reference(
    std::span<const double, 3> frame_pos_world_m, double yaw_rad) noexcept {
    if (!std::isfinite(frame_pos_world_m[0]) ||
        !std::isfinite(frame_pos_world_m[1]) ||
        !std::isfinite(frame_pos_world_m[2]) || !std::isfinite(yaw_rad)) {
        return;
    }
    state_.ref_pos_m = {frame_pos_world_m[0], frame_pos_world_m[1],
                        frame_pos_world_m[2]};
    state_.ref_yaw_rad = yaw_rad;
    state_.reference_latched = true;
}

void LaunchSupport::restore(const LaunchSupportState &state) {
    if (!all_finite3(state.ref_pos_m) || !std::isfinite(state.ref_yaw_rad)) {
        throw std::invalid_argument(
            "launch_support: restore state contains non-finite reference");
    }
    state_ = state;
}

double LaunchSupport::fade_weight(double forward_speed_mps) const noexcept {
    if (!state_.armed) {
        return 0.0;
    }
    const double v_fwd =
        std::isfinite(forward_speed_mps) ? std::max(0.0, forward_speed_mps) : 0.0;
    const double v1 = config_.spec.release_speed_mps;
    const double v0 = 0.5 * v1;
    if (v_fwd >= v1) {
        return 0.0;
    }
    if (v_fwd <= v0) {
        return 1.0;
    }
    const double s = (v_fwd - v0) / (v1 - v0);
    return 1.0 - s * s * s * (10.0 - s * (15.0 - 6.0 * s));
}

double LaunchSupport::compute_into(const LaunchSupportInput &input,
                                   std::span<double> out_qfrc) noexcept {
    std::ranges::fill(out_qfrc, 0.0);
    const auto qa = static_cast<std::size_t>(config_.root_free_qposadr);
    const auto da = static_cast<std::size_t>(config_.root_free_dofadr);
    if (std::cmp_less(input.qpos.size(), config_.root_free_qposadr + 7) ||
        std::cmp_less(input.qvel.size(), config_.root_free_dofadr + 6) ||
        std::cmp_less(out_qfrc.size(), config_.root_free_dofadr + 6)) {
        return 0.0;
    }
    const bike::spatial::Quat quat{
        input.qpos[qa + 3U], input.qpos[qa + 4U],
        input.qpos[qa + 5U], input.qpos[qa + 6U]};
    const bike::spatial::EulerZyx att = bike::spatial::euler_zyx(quat);
    if (!att.valid) {
        return 0.0;
    }

    const double v_fwd = std::isfinite(input.forward_speed_mps)
                             ? std::max(0.0, input.forward_speed_mps)
                             : 0.0;
    if (input.advance) {
        if (state_.armed) {
            if (v_fwd >= config_.spec.release_speed_mps) {
                state_.armed = false;
                state_.reference_latched = false;
                return 0.0;
            }
            if (!state_.reference_latched) {
                latch_reference(input.frame_pos_world_m, att.yaw_rad);
            }
        } else {
            if (input.motor_braking &&
                std::isfinite(input.forward_speed_mps) &&
                std::abs(input.forward_speed_mps) <= config_.standstill_speed_mps) {
                state_.armed = true;
                latch_reference(input.frame_pos_world_m, att.yaw_rad);
            } else {
                return 0.0;
            }
        }
    }

    const double w = fade_weight(v_fwd);
    if (w <= 0.0) {
        return 0.0;
    }

    const std::array<double, 3> ref_pos =
        state_.reference_latched
            ? state_.ref_pos_m
            : std::array<double, 3>{input.frame_pos_world_m[0],
                                    input.frame_pos_world_m[1],
                                    input.frame_pos_world_m[2]};
    const double ref_yaw =
        state_.reference_latched ? state_.ref_yaw_rad : att.yaw_rad;

    const double sin_psi = std::sin(att.yaw_rad);
    const double cos_psi = std::cos(att.yaw_rad);
    const double sin_ref = std::sin(ref_yaw);
    const double cos_ref = std::cos(ref_yaw);

    const double dx = input.frame_pos_world_m[0] - ref_pos[0];
    const double dy = input.frame_pos_world_m[1] - ref_pos[1];
    const double y_perp = (-sin_ref * dx) + (cos_ref * dy);
    const double y_perp_dot =
        (-sin_psi * input.qvel[da + 0U]) + (cos_psi * input.qvel[da + 1U]);

    const double tan_theta = std::tan(att.pitch_rad);
    const double sin_phi = std::sin(att.roll_rad);
    const double cos_phi = std::cos(att.roll_rad);
    constexpr double j0 = 1.0;
    const double j1 = sin_phi * tan_theta;
    const double j2 = cos_phi * tan_theta;
    const double phi_dot =
        (j0 * input.qvel[da + 3U]) + (j1 * input.qvel[da + 4U]) +
        (j2 * input.qvel[da + 5U]);

    const double f_lat =
        w * ((-config_.spec.lateral_stiffness_n_per_m * y_perp) -
             (config_.spec.lateral_damping_ns_per_m * y_perp_dot));
    const double tau_roll =
        w * ((-config_.spec.roll_stiffness_nm_per_rad * att.roll_rad) -
             (config_.spec.roll_damping_nms_per_rad * phi_dot));

    if (!std::isfinite(f_lat) || !std::isfinite(tau_roll)) {
        return 0.0;
    }

    out_qfrc[da + 0U] = f_lat * (-sin_psi);
    out_qfrc[da + 1U] = f_lat * cos_psi;
    out_qfrc[da + 2U] = 0.0;
    out_qfrc[da + 3U] = tau_roll * j0;
    out_qfrc[da + 4U] = tau_roll * j1;
    out_qfrc[da + 5U] = tau_roll * j2;
    return w;
}

} // namespace runtime
