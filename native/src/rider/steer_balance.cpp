#include "steer_balance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace bike::rider {
namespace {

constexpr double kMassNoneKg = 24.40;
constexpr double kMassLumpedKg = 104.70;
constexpr double kMaxOverweightScale = 1.40;
constexpr double kExactTickRelTol = 1e-12;
constexpr double kMinCorridorHalfWidthM = 0.25;

// Table 2.3A: Canonical Rider-Loaded Schedule (104.70 kg)
constexpr std::array<SteerGainNode, 7> kScheduleLumped = {{
    {.speed_mps = 2.5,  .k_phi = 110.0, .k_phidot = 56.0, .k_delta = 18.0, .k_deltadot = 7.5,  .k_psi = 0.14, .k_y = 0.030, .k_ydot = 0.025},
    {.speed_mps = 3.0,  .k_phi = 108.0, .k_phidot = 56.0, .k_delta = 20.0, .k_deltadot = 9.0,  .k_psi = 0.18, .k_y = 0.036, .k_ydot = 0.030},
    {.speed_mps = 4.0,  .k_phi = 92.0,  .k_phidot = 46.0, .k_delta = 22.0, .k_deltadot = 11.0, .k_psi = 0.20, .k_y = 0.042, .k_ydot = 0.032},
    {.speed_mps = 6.0,  .k_phi = 36.0,  .k_phidot = 34.0, .k_delta = 15.0, .k_deltadot = 22.0, .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
    {.speed_mps = 8.0,  .k_phi = 44.0,  .k_phidot = 10.0, .k_delta = 26.0, .k_deltadot = 4.0,  .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
    {.speed_mps = 10.0, .k_phi = 44.0,  .k_phidot = 7.0,  .k_delta = 18.0, .k_deltadot = 5.0,  .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
    {.speed_mps = 12.0, .k_phi = 44.0,  .k_phidot = 5.0,  .k_delta = 15.0, .k_deltadot = 5.0,  .k_psi = 0.22, .k_y = 0.050, .k_ydot = 0.035},
}};

// Table 2.3B: Canonical Riderless Schedule (24.40 kg)
constexpr std::array<SteerGainNode, 7> kScheduleNone = {{
    {.speed_mps = 2.5,  .k_phi = 29.0, .k_phidot = 16.5, .k_delta = 4.2, .k_deltadot = 2.8, .k_psi = 0.14, .k_y = 0.028, .k_ydot = 0.025},
    {.speed_mps = 3.0,  .k_phi = 28.0, .k_phidot = 16.0, .k_delta = 4.8, .k_deltadot = 3.5, .k_psi = 0.18, .k_y = 0.035, .k_ydot = 0.030},
    {.speed_mps = 4.0,  .k_phi = 25.0, .k_phidot = 14.5, .k_delta = 5.8, .k_deltadot = 4.8, .k_psi = 0.20, .k_y = 0.042, .k_ydot = 0.032},
    {.speed_mps = 6.0,  .k_phi = 19.0, .k_phidot = 10.0, .k_delta = 6.5, .k_deltadot = 5.2, .k_psi = 0.22, .k_y = 0.048, .k_ydot = 0.035},
    {.speed_mps = 8.0,  .k_phi = 15.5, .k_phidot = 6.8,  .k_delta = 6.5, .k_deltadot = 4.0, .k_psi = 0.22, .k_y = 0.048, .k_ydot = 0.035},
    {.speed_mps = 10.0, .k_phi = 15.0, .k_phidot = 5.5,  .k_delta = 6.8, .k_deltadot = 2.6, .k_psi = 0.22, .k_y = 0.045, .k_ydot = 0.035},
    {.speed_mps = 12.0, .k_phi = 15.0, .k_phidot = 4.5,  .k_delta = 7.0, .k_deltadot = 1.6, .k_psi = 0.22, .k_y = 0.045, .k_ydot = 0.035},
}};

[[nodiscard]] double preset_mass_kg(SteerRiderPreset preset) {
    switch (preset) {
    case SteerRiderPreset::none:
        return kMassNoneKg;
    case SteerRiderPreset::lumped:
        return kMassLumpedKg;
    }
    throw std::invalid_argument("steer_balance: invalid SteerRiderPreset");
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (duration_s, step_s, min_ticks) matches spatial::require_exact_ticks
[[nodiscard]] std::int64_t require_exact_ticks(double duration_s,
                                               double step_s,
                                               std::int64_t min_ticks,
                                               const char* error_msg) {
// NOLINTEND(bugprone-easily-swappable-parameters)
    if (!std::isfinite(duration_s) || !std::isfinite(step_s) || !(step_s > 0.0)) {
        throw std::invalid_argument(error_msg);
    }
    const double ratio = duration_s / step_s;
    if (!std::isfinite(ratio) ||
        ratio < static_cast<double>(min_ticks) - kExactTickRelTol ||
        ratio > 1e9) {
        throw std::invalid_argument(error_msg);
    }
    const double nearest = std::nearbyint(ratio);
    const double scale = std::max(1.0, std::abs(ratio));
    if (std::abs(ratio - nearest) > kExactTickRelTol * scale) {
        throw std::invalid_argument(error_msg);
    }
    const auto ticks = static_cast<std::int64_t>(nearest);
    if (ticks < min_ticks) {
        throw std::invalid_argument(error_msg);
    }
    return ticks;
}

[[nodiscard]] double lerp_gain(double a, double b, double t) noexcept BIKE_NONBLOCKING {
    return ((1.0 - t) * a) + (t * b);
}

[[nodiscard]] SteerGainNode lerp_node(const SteerGainNode& a,
                                      const SteerGainNode& b,
                                      double t,
                                      double speed_mps) noexcept BIKE_NONBLOCKING {
    return {
        .speed_mps = speed_mps,
        .k_phi = lerp_gain(a.k_phi, b.k_phi, t),
        .k_phidot = lerp_gain(a.k_phidot, b.k_phidot, t),
        .k_delta = lerp_gain(a.k_delta, b.k_delta, t),
        .k_deltadot = lerp_gain(a.k_deltadot, b.k_deltadot, t),
        .k_psi = lerp_gain(a.k_psi, b.k_psi, t),
        .k_y = lerp_gain(a.k_y, b.k_y, t),
        .k_ydot = lerp_gain(a.k_ydot, b.k_ydot, t),
    };
}

[[nodiscard]] std::array<SteerGainNode, 7> build_schedule(double total_mass_kg) noexcept {
    std::array<SteerGainNode, 7> out{};
    if (total_mass_kg <= kMassLumpedKg) {
        const double blend = std::clamp(
            (total_mass_kg - kMassNoneKg) / (kMassLumpedKg - kMassNoneKg), 0.0, 1.0);
        for (std::size_t i = 0; i < out.size(); ++i) {
            out[i] = lerp_node(kScheduleNone[i], kScheduleLumped[i], blend,
                               kScheduleLumped[i].speed_mps);
        }
        return out;
    }

    const double alpha = std::clamp(total_mass_kg / kMassLumpedKg, 1.0, kMaxOverweightScale);
    const double sqrt_alpha = std::sqrt(alpha);
    for (std::size_t i = 0; i < out.size(); ++i) {
        SteerGainNode node = kScheduleLumped[i];
        node.k_phi *= alpha;
        node.k_phidot *= alpha;
        node.k_delta *= alpha;
        node.k_deltadot *= sqrt_alpha;
        out[i] = node;
    }
    return out;
}

[[nodiscard]] bool is_finite_sample(const DelayedBalanceSample& s) noexcept {
    return std::isfinite(s.roll_rad) && std::isfinite(s.roll_rate_rad_s) &&
           std::isfinite(s.heading_err_rad) && std::isfinite(s.lateral_err_m) &&
           std::isfinite(s.lateral_rate_mps);
}

[[nodiscard]] double validated_corridor_half_width(const bike::spatial::SpatialConfig& config) {
    if (!std::isfinite(config.corridor_half_width_m) ||
        !(config.corridor_half_width_m >= kMinCorridorHalfWidthM)) {
        throw std::invalid_argument("steer_balance: corridor_half_width_m must be >= 0.25 m");
    }
    return config.corridor_half_width_m;
}

} // namespace

// NOLINTBEGIN(bugprone-easily-swappable-parameters) (spec, timestep_s, total_mass_kg, max_lean_cmd_rad) matches S3 spec API
SteerBalance::SteerBalance(const bike::spatial::SpatialSteerSpec& spec,
                           double timestep_s,
                           double total_mass_kg,
                           double max_lean_cmd_rad)
    : max_torque_nm_(spec.max_torque_nm),
      max_lean_cmd_rad_(max_lean_cmd_rad) {
    if (!std::isfinite(timestep_s) || !(timestep_s > 0.0)) {
        throw std::invalid_argument("steer_balance: timestep_s must be finite and positive");
    }
    if (!std::isfinite(max_torque_nm_) || !(max_torque_nm_ > 0.0)) {
        throw std::invalid_argument("steer_balance: max_torque_nm must be finite and positive");
    }
    if (!std::isfinite(total_mass_kg) || !(total_mass_kg > 0.0)) {
        throw std::invalid_argument("steer_balance: total_mass_kg must be finite and positive");
    }
    if (!std::isfinite(max_lean_cmd_rad_) || !(max_lean_cmd_rad_ > 0.0)) {
        throw std::invalid_argument("steer_balance: max_lean_cmd_rad must be finite and positive");
    }

    period_steps_ = require_exact_ticks(
        spec.control_period_s, timestep_s, 1,
        "steer_balance: control_period_s must be an exact positive integer multiple of timestep_s");
    delay_steps_ = require_exact_ticks(
        spec.reaction_delay_s, timestep_s, 0,
        "steer_balance: reaction_delay_s must be an exact nonnegative integer multiple of timestep_s");

    schedule_ = build_schedule(total_mass_kg);
    ring_.assign(static_cast<std::size_t>(delay_steps_) + 1U, DelayedBalanceSample{});
    head_ = 0;
    held_torque_nm_ = 0.0;
}

SteerBalance::SteerBalance(const bike::spatial::SpatialSteerSpec& spec,
                           double timestep_s,
                           SteerRiderPreset preset,
                           double max_lean_cmd_rad)
    : SteerBalance(spec, timestep_s, preset_mass_kg(preset), max_lean_cmd_rad) {}

SteerBalance::SteerBalance(const bike::spatial::SpatialConfig& config,
                           double timestep_s,
                           double total_mass_kg,
                           double max_lean_cmd_rad)
    : SteerBalance(config.steer,
                   timestep_s,
                   (static_cast<void>(validated_corridor_half_width(config)), total_mass_kg),
                   max_lean_cmd_rad) {}

SteerBalance::SteerBalance(const bike::spatial::SpatialConfig& config,
                           double timestep_s,
                           SteerRiderPreset preset,
                           double max_lean_cmd_rad)
    : SteerBalance(config.steer,
                   timestep_s,
                   (static_cast<void>(validated_corridor_half_width(config)), preset_mass_kg(preset)),
                   max_lean_cmd_rad) {}
// NOLINTEND(bugprone-easily-swappable-parameters)

void SteerBalance::reset(const DelayedBalanceSample& initial_sample,
                         double initial_torque_nm) noexcept BIKE_NONBLOCKING {
    std::ranges::fill(ring_, initial_sample);
    head_ = 0;
    held_torque_nm_ = std::isfinite(initial_torque_nm)
                          ? std::clamp(initial_torque_nm, -max_torque_nm_, max_torque_nm_)
                          : 0.0;
}

double SteerBalance::step(std::int64_t physics_step,
                          const SteerBalanceObservation& obs) noexcept BIKE_NONBLOCKING {
    if (physics_step < 0) {
        return held_torque_nm_;
    }

    ring_[head_] = {
        .roll_rad = obs.roll_rad,
        .roll_rate_rad_s = obs.roll_rate_rad_s,
        .heading_err_rad = obs.heading_err_rad,
        .lateral_err_m = obs.lateral_err_m,
        .lateral_rate_mps = obs.lateral_rate_mps,
    };
    const std::size_t oldest_idx = (head_ + 1U) % ring_.size();
    const DelayedBalanceSample delayed = (delay_steps_ == 0) ? ring_[head_] : ring_[oldest_idx];
    head_ = oldest_idx;

    if (physics_step % period_steps_ != 0) {
        return held_torque_nm_;
    }

    const double speed_mps = std::isfinite(obs.forward_speed_mps)
                                 ? std::max(0.0, obs.forward_speed_mps)
                                 : 0.0;
    const SteerGainNode g = interpolate(speed_mps);

    const double raw_lean_cmd =
        (g.k_y * delayed.lateral_err_m) +
        (g.k_ydot * delayed.lateral_rate_mps) +
        (g.k_psi * delayed.heading_err_rad);
    const double phi_cmd = std::clamp(raw_lean_cmd, -max_lean_cmd_rad_, max_lean_cmd_rad_);

    const double tau_raw =
        (-g.k_phi * (delayed.roll_rad - phi_cmd)) -
        (g.k_phidot * delayed.roll_rate_rad_s) -
        (g.k_delta * obs.steer_angle_rad) -
        (g.k_deltadot * obs.steer_rate_rad_s);

    if (std::isfinite(tau_raw)) {
        held_torque_nm_ = std::clamp(tau_raw, -max_torque_nm_, max_torque_nm_);
    }
    return held_torque_nm_;
}

SteerBalanceState SteerBalance::state() const {
    return {
        .ring = ring_,
        .head = head_,
        .held_torque_nm = held_torque_nm_,
    };
}

void SteerBalance::restore(const SteerBalanceState& state) {
    if (state.ring.size() != ring_.size()) {
        throw std::invalid_argument("steer_balance: restore ring size mismatch");
    }
    if (state.head >= state.ring.size()) {
        throw std::invalid_argument("steer_balance: restore head index out of range");
    }
    for (const DelayedBalanceSample& sample : state.ring) {
        if (!is_finite_sample(sample)) {
            throw std::invalid_argument("steer_balance: restore ring contains nonfinite sample");
        }
    }
    if (!std::isfinite(state.held_torque_nm) ||
        state.held_torque_nm < -max_torque_nm_ ||
        state.held_torque_nm > max_torque_nm_) {
        throw std::invalid_argument("steer_balance: restore held_torque_nm out of range");
    }

    std::ranges::copy(state.ring, ring_.begin());
    head_ = state.head;
    held_torque_nm_ = state.held_torque_nm;
}

SteerGainNode SteerBalance::interpolate(double speed_mps) const noexcept BIKE_NONBLOCKING {
    const double v = std::max(0.0, speed_mps);
    if (v <= schedule_.front().speed_mps) {
        return schedule_.front();
    }
    if (v >= schedule_.back().speed_mps) {
        return schedule_.back();
    }
    for (std::size_t i = 0; i + 1U < schedule_.size(); ++i) {
        const SteerGainNode& lo = schedule_[i];
        const SteerGainNode& hi = schedule_[i + 1U];
        if (v <= hi.speed_mps) {
            const double t = (v - lo.speed_mps) / (hi.speed_mps - lo.speed_mps);
            return lerp_node(lo, hi, t, v);
        }
    }
    return schedule_.back();
}

} // namespace bike::rider
