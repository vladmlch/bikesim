#pragma once

#include <cstdint>
#include <optional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bike::initialization {

struct SpatialEquilibriumReport;

struct EquilibriumOptions {
    std::optional<std::int64_t> max_steps;
    double tolerance = .05;
};

struct EquilibriumResult {
    // Nullable owned report. shared_ptr supports the W00 frozen header's
    // context.hpp dependency without an incomplete std::optional cycle.
    std::shared_ptr<SpatialEquilibriumReport> spatial;
    std::int64_t steps = 0;
    double residual_qacc = 0.;
    bool cache_hit = false;
    double fork_travel_mm = 0.;
    double shock_stroke_mm = 0.;
    double rear_travel_mm = 0.;
    double root_z_m = 0.;
    double pitch_rad = 0.;
    double initial_crank_phase_rad = 0.;
    double settled_crank_phase_rad = 0.;
    std::optional<std::string> state_file;
    double static_front_load_n = 0.;
    double static_rear_load_n = 0.;
    double total_vertical_force_n = 0.;
    std::optional<double> initial_state_target_residual_qacc;
    std::optional<std::string> initial_state_sha256;
    std::optional<std::string> initial_state_source_sha256;
    std::optional<double> initial_state_source_timestep_s;
    std::optional<double> source_equilibrium_residual_qacc;
};

// The unconverged owner remains inspectable; it can never be sealed.
class EquilibriumFailure final : public std::runtime_error {
public:
    EquilibriumFailure(std::int64_t steps, double residual,
                       std::vector<std::pair<std::string, double>> worst);
    ~EquilibriumFailure() override;
    EquilibriumFailure(const EquilibriumFailure&) = default;
    EquilibriumFailure& operator=(const EquilibriumFailure&) = default;
    EquilibriumFailure(EquilibriumFailure&&) = default;
    EquilibriumFailure& operator=(EquilibriumFailure&&) = default;
    std::int64_t steps;
    double residual_qacc;
    std::vector<std::pair<std::string, double>> worst_accelerations;
};

class SetupOwner;
struct SetupContext;
[[nodiscard]] EquilibriumResult settle(SetupOwner&, const EquilibriumOptions& = {});
[[nodiscard]] EquilibriumResult settle_context(SetupContext&, const EquilibriumOptions& = {});
[[nodiscard]] std::int64_t equilibrium_step_budget(double timestep_s,
                                                  const EquilibriumOptions&);
[[nodiscard]] std::int64_t equilibrium_cycle_steps(double timestep_s);
void initialize_pose(SetupContext&);
void settle_contacts(SetupContext&, const std::vector<double>& qpos,
                     const std::vector<double>& qvel);
[[nodiscard]] double acceleration_residual(SetupContext const&);
void apply_initial_forces(SetupContext&, bool advance);
[[nodiscard]] double refine_equilibrium(SetupContext&, int max_evaluations = 80,
                                       double acceleration_tolerance = .05);

} // namespace bike::initialization
