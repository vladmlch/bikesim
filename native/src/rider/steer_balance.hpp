#pragma once

#include "rtsan.hpp"
#include "spatial/frames.hpp"
#include "spatial/types.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace bike::rider {

enum class SteerRiderPreset : std::uint8_t { none, lumped };

struct SteerGainNode {
    double speed_mps = 0.0;
    double k_phi = 0.0;      // N*m / rad
    double k_phidot = 0.0;   // N*m*s / rad
    double k_delta = 0.0;    // N*m / rad
    double k_deltadot = 0.0; // N*m*s / rad
    double k_psi = 0.0;      // rad / rad
    double k_y = 0.0;        // rad / m
    double k_ydot = 0.0;     // rad / (m/s)
};

struct SteerBalanceObservation {
    double roll_rad = 0.0;
    double roll_rate_rad_s = 0.0;
    double heading_err_rad = 0.0; // wrapped to (-pi, pi]: atan2(sin(yaw - yaw_ref), cos(yaw - yaw_ref))
    double lateral_err_m = 0.0;   // y - y_ref (world +Y left)
    double lateral_rate_mps = 0.0; // vy (world +Y rate)
    double steer_angle_rad = 0.0; // immediate steer_joint qpos
    double steer_rate_rad_s = 0.0; // immediate steer_joint qvel
    double forward_speed_mps = 0.0;
};

struct DelayedBalanceSample {
    double roll_rad = 0.0;
    double roll_rate_rad_s = 0.0;
    double heading_err_rad = 0.0;
    double lateral_err_m = 0.0;
    double lateral_rate_mps = 0.0;
};

struct SteerBalanceState {
    std::vector<DelayedBalanceSample> ring;
    std::size_t head = 0;
    double held_torque_nm = 0.0;
};

class SteerBalance {
public:
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) (spec, timestep_s, total_mass_kg, max_lean_cmd_rad) matches S3 spec API
    SteerBalance(const bike::spatial::SpatialSteerSpec& spec,
                 double timestep_s,
                 double total_mass_kg,
                 double max_lean_cmd_rad = 0.25);

    SteerBalance(const bike::spatial::SpatialSteerSpec& spec,
                 double timestep_s,
                 SteerRiderPreset preset,
                 double max_lean_cmd_rad = 0.25);

    SteerBalance(const bike::spatial::SpatialConfig& config,
                 double timestep_s,
                 double total_mass_kg,
                 double max_lean_cmd_rad = 0.25);

    SteerBalance(const bike::spatial::SpatialConfig& config,
                 double timestep_s,
                 SteerRiderPreset preset,
                 double max_lean_cmd_rad = 0.25);
    // NOLINTEND(bugprone-easily-swappable-parameters)

    void reset(const DelayedBalanceSample& initial_sample = {},
               double initial_torque_nm = 0.0) noexcept BIKE_NONBLOCKING;

    // Zero-allocation warm-path step: pushes fresh sample every physics step,
    // recomputes torque on ControlClock ticks (physics_step % period_steps_ == 0),
    // and holds held_torque_nm_ (ZOH) between control ticks.
    [[nodiscard]] double step(std::int64_t physics_step,
                              const SteerBalanceObservation& obs) noexcept BIKE_NONBLOCKING;

    [[nodiscard]] double update(std::int64_t physics_step,
                                const SteerBalanceObservation& obs) noexcept BIKE_NONBLOCKING {
        return step(physics_step, obs);
    }

    [[nodiscard]] double held_torque_nm() const noexcept BIKE_NONBLOCKING {
        return held_torque_nm_;
    }
    [[nodiscard]] std::int64_t period_steps() const noexcept BIKE_NONBLOCKING {
        return period_steps_;
    }
    [[nodiscard]] std::int64_t delay_steps() const noexcept BIKE_NONBLOCKING {
        return delay_steps_;
    }
    [[nodiscard]] double max_torque_nm() const noexcept BIKE_NONBLOCKING {
        return max_torque_nm_;
    }
    [[nodiscard]] double max_lean_cmd_rad() const noexcept BIKE_NONBLOCKING {
        return max_lean_cmd_rad_;
    }
    [[nodiscard]] const std::array<SteerGainNode, 7>& schedule() const noexcept BIKE_NONBLOCKING {
        return schedule_;
    }
    [[nodiscard]] SteerGainNode gains_for_speed(double speed_mps) const noexcept BIKE_NONBLOCKING {
        return interpolate(speed_mps);
    }

    [[nodiscard]] SteerBalanceState state() const;
    void restore(const SteerBalanceState& state);

private:
    [[nodiscard]] SteerGainNode interpolate(double speed_mps) const noexcept BIKE_NONBLOCKING;

    std::array<SteerGainNode, 7> schedule_{};
    std::int64_t period_steps_ = 4;
    std::int64_t delay_steps_ = 80;
    double max_torque_nm_ = 35.0;
    double max_lean_cmd_rad_ = 0.25;
    std::vector<DelayedBalanceSample> ring_;
    std::size_t head_ = 0;
    double held_torque_nm_ = 0.0;
};

} // namespace bike::rider
