#pragma once

#include "equilibrium.hpp"

#include <array>
#include <functional>
#include <optional>

class Stepper;

namespace bike::initialization {

struct StaticEquilibriumOptions {
    double start_x_m = 2.;
    std::int64_t max_steps = 40000;
    double tolerance = .05;
    std::optional<std::int64_t> relax_steps_per_cycle;
    std::optional<double> crank_phase_rad;
};

// Typed initialization bridge for the legacy force owners (T26). The
// suspension callback is required; optional force writers keep the exact
// before/tyre/suspension/rider/zero-torque-leg order from equilibrium.py.
// All callbacks operate on this Stepper's model/data, never another pair.
// This solver is not a replacement force assembly for PhysicalStep.
struct StaticEquilibriumContext {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    Stepper& stepper;
    bool initializing = true;
    std::function<void()> suspension_forces{};
    std::function<double(double)> rear_travel_from_shock_mm{};
    std::function<void()> before_forces{}, tire_forces{}, tire_reset{};
    std::function<void()> rider_forces{}, leg_zero_torque_forces{};
    std::function<void()> drive_initialize{};
    std::function<void(double)> leg_initialize{};
    std::function<std::array<double,3>()> rider_loads_n{};
    std::optional<double> rider_mass_kg{};
};

struct StaticEquilibriumResult {
    EquilibriumResult pose;
    // Source order: saddle, pedals, handlebar.
    std::optional<std::array<double,3>> rider_loads_n, rider_weight_shares;
};

[[nodiscard]] StaticEquilibriumResult solve_static_equilibrium(
    StaticEquilibriumContext&, const StaticEquilibriumOptions& = {});

} // namespace bike::initialization
