#include "setup_owner.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

namespace bike::initialization {
namespace {
std::array<double, 2> evaluate_sag_point(const SetupRequest& base, double fork_psi, double coil_n_m) {
    if (!std::isfinite(fork_psi) || !std::isfinite(coil_n_m) || fork_psi <= 0. || coil_n_m <= 0.)
        throw std::invalid_argument("sag evaluator pressure and coil rate must be finite and positive");
    auto candidate = std::make_unique<SetupRequest>(base);
    candidate->model.bike.fork_initial_psi = fork_psi;
    candidate->model.bike.shock_stiffness = coil_n_m;
    const auto options = candidate->equilibrium;
    auto setup = make_setup(std::move(*candidate));
    const auto result = settle(*setup, options);
    return std::array<double, 2>{result.fork_travel_mm, result.rear_travel_mm};
}
} // namespace

SagEvaluator build_equilibrium_evaluator(SetupRequest request) {
    if (!request.model.physics)
        throw std::invalid_argument("sag evaluator requires physical setup configuration");
    // Every evaluation owns a fresh compiled model, data, and material state.
    // Candidate pressure/rate cannot reuse a pose or state from another point.
    request.model.physics->equilibrium_cache_enabled = false;
    request.model.physics->equilibrium_state_file.reset();
    request.initial_state.reset();
    auto shared_base = std::make_shared<const SetupRequest>(std::move(request));
    return [base = std::move(shared_base)](double fork_psi, double coil_n_m) {
        return evaluate_sag_point(*base, fork_psi, coil_n_m);
    };
}

} // namespace bike::initialization
