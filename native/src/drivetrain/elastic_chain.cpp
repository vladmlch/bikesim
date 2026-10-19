// T27: residual public chain helpers. Production force application continues
// to use DrivetrainWriter's analytic Jacobian and transactional force rows.
#include "chain.hpp"

#include <cstddef>
#include <stdexcept>

namespace drivetrain {

double chain_extension(Vec2 front, Vec2 rear, double rf, double rr,
                       double theta_f, double theta_r, double reference,
                       Vec2 up, std::optional<double> psi_reference) {
    finite(theta_f, "absolute crank angle");
    finite(theta_r, "absolute cassette angle");
    finite(reference, "chain reference");
    const auto geometry = chain_geometry(front, rear, rf, rr, up, psi_reference);
    return validation::derived(
        geometry.length + rf * theta_f - rr * theta_r - reference,
        "chain extension");
}

std::vector<double> chain_jacobian(
    std::span<const double> q,
    const std::function<double(std::span<const double>)>& evaluate,
    double epsilon) {
    for (double const coordinate : q) finite(coordinate, "generalized coordinates");
    positive(epsilon, "difference step");
    if (!evaluate) throw std::invalid_argument("chain Jacobian requires an evaluator");
    std::vector<double> result(q.size()), plus(q.size()), minus(q.size());
    for (std::size_t i = 0; i < q.size(); ++i) {
        // Match q +/- a zero-filled perturbation vector, including signed
        // zero in the unperturbed lanes. Neither callback receives q itself.
        for (std::size_t j = 0; j < q.size(); ++j) {
            const double delta = i == j ? epsilon : 0.;
            plus[j] = q[j] + delta;
            minus[j] = q[j] - delta;
        }
        const double upper = evaluate(plus);
        finite(upper, "positive chain perturbation");
        const double lower = evaluate(minus);
        finite(lower, "negative chain perturbation");
        result[i] = validation::derived((upper - lower) / (2. * epsilon),
                                        "chain Jacobian");
    }
    return result;
}

} // namespace drivetrain
