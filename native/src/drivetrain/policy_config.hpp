#pragma once
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace drivetrain {
inline double finite(double value, const char* name) {
    if (!std::isfinite(value)) throw std::invalid_argument(name);
    return value;
}
inline double nonnegative(double value, const char* name) {
    finite(value, name);
    if (value < 0.) throw std::invalid_argument(name);
    return value;
}
inline double positive(double value, const char* name) {
    finite(value, name);
    if (value <= 0.) throw std::invalid_argument(name);
    return value;
}
inline void finite_optional(std::optional<double> value, const char* name) {
    if (value) finite(*value, name);
}
struct GearingConfig { int front_teeth{}, rear_teeth{}; double chain_pitch_m{}; };
struct PedalingConfig {
    bool enabled{};
    double coast_above_rpm{}, resume_below_rpm{}, stop_time_s{}, coast_cadence_tau_s{};
    double mash_cadence_rpm{}, mash_torque_nm{}, effort_slew_nm_s{};
};
struct ShiftingConfig {
    bool enabled{};
    std::vector<int> cassette;
    double target_cadence_min_rpm{}, target_cadence_max_rpm{}, shift_cooldown_s{};
    double shift_cut_duration_s{}, torque_factor{}, cadence_smoothing_tau_s{}, upshift_slip_limit_mps{};
    std::string upshift_slip_mode;
};
struct MotorProfile {
    double eco{}, tour{}, emtb_low{}, emtb_high{}, turbo{}, emtb_full_gain_at_nm{};
};
struct AssistConfig {
    double gain{}, max_torque{}, max_power{}, tau{}, slew{}, engage_torque_nm{};
    double gate_min_crank_rad_s{}, cutoff_mps{}, taper_width_mps{};
    std::optional<std::vector<std::array<double, 2>>> torque_curve;
    std::optional<MotorProfile> profile;
    std::string mode;
};
struct BatteryConfig {
    bool enabled{};
    double energy_j{}, copper_w_per_nm2{}, speed_w_per_rad_s2{}, idle_w{};
};
struct DrivePolicyConfig {
    GearingConfig gearing;
    PedalingConfig pedaling;
    ShiftingConfig shifting;
    AssistConfig assist;
    BatteryConfig battery;
    double hub_stiffness_nm_rad{}, hub_damping_nm_s{};
};
struct PedalingSnapshot {
    bool coasting{};
    std::optional<double> target_phase_rad;
    double target_rate_rad_s{}, deceleration_rad_s2{}, effort{};
    std::optional<double> cadence_ema;
};
struct ShiftingSnapshot {
    int rear_teeth{}, from_teeth{}, shift_count{};
    double cooldown_s{}, cut_remaining_s{};
    std::string direction = "none";
    std::optional<double> cadence_ema, required_ema;
};
struct AssistSnapshot { double torque{}, last_gain{}; bool pedaling{}; };
struct BatterySnapshot { double initial_energy_j{}, energy_j{}, drawn_energy_j{}; };
struct FreehubSnapshot { std::optional<double> boundary; double energy_j{}, torque_nm{}; };
} // namespace drivetrain
