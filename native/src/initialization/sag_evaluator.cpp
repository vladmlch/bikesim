#include "setup_owner.hpp"

#include <cmath>
#include <stdexcept>

namespace bike::initialization {

SagEvaluator build_equilibrium_evaluator(SetupRequest request) {
    if (!request.model.physics)
        throw std::invalid_argument("sag evaluator requires physical setup configuration");
    // Every evaluation owns a fresh compiled model, data, and material state.
    // Candidate pressure/rate cannot reuse a pose or state from another point.
    request.model.physics->equilibrium_cache_enabled = false;
    request.model.physics->equilibrium_state_file.reset();
    request.initial_state.reset();
    return [base = std::move(request)](double fork_psi, double coil_n_m) {
        if (!std::isfinite(fork_psi) || !std::isfinite(coil_n_m) || fork_psi <= 0. || coil_n_m <= 0.)
            throw std::invalid_argument("sag evaluator pressure and coil rate must be finite and positive");
        auto candidate = base;
        candidate.model.bike.fork_initial_psi = fork_psi;
        candidate.model.bike.shock_stiffness = coil_n_m;
        const auto options = candidate.equilibrium;
        auto setup = make_setup(std::move(candidate));
        const auto result = settle(*setup, options);
        return std::array<double, 2>{result.fork_travel_mm, result.rear_travel_mm};
    };
}

} // namespace bike::initialization
