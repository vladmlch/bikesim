#include "tabulated_radial.hpp"

#include <algorithm>
#include <stdexcept>

#include "../config_validation.hpp"
#include "../validation.hpp"

namespace contactlaw {
MaterialRangeError::~MaterialRangeError() = default;
TabulatedRadial::TabulatedRadial(const nativecfg::TireTable &table) : deflection_m_(table.deflection_m), force_n_(table.force_n) {
    nativecfg::validate(table);
    
    
    slopes_.reserve(deflection_m_.size() - 1);
    integral_.reserve(deflection_m_.size());
    integral_.push_back(0.);
    for (std::size_t i = 1; i < deflection_m_.size(); ++i) {
        const double dx = deflection_m_[i] - deflection_m_[i - 1];
        slopes_.push_back((force_n_[i] - force_n_[i - 1]) / dx);
        integral_.push_back(integral_.back() + .5 * (force_n_[i - 1] + force_n_[i]) * dx);
    }
}
std::pair<double, double> TabulatedRadial::elastic_response(double penetration_m) const {
    validation::finite(penetration_m, "tire deflection");
    if (penetration_m <= 0.) return {0., 0.};
    if (penetration_m > deflection_m_.back())
        throw MaterialRangeError("tire deflection exceeds supplied curve limit");
    const auto right = std::ranges::upper_bound(deflection_m_, penetration_m);
    const auto i = std::min(static_cast<std::size_t>(right - deflection_m_.begin() - 1), slopes_.size() - 1);
    const double dx = penetration_m - deflection_m_[i];
    const double force = force_n_[i] + slopes_[i] * dx;
    const double energy = integral_[i] + force_n_[i] * dx + .5 * slopes_[i] * dx * dx;
    return {validation::derived(force, "TabulatedTireSpec.elastic_response"),
            validation::derived(energy, "TabulatedTireSpec.elastic_response")};
}
std::pair<double, double> TabulatedRadial::normal_contact(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    double penetration_m, double rate_mps, double damping_ns_m) const {
    validation::finite(rate_mps, "deflection rate");
    validation::nonnegative(damping_ns_m, "radial damping");
    const auto [spring, energy] = elastic_response(penetration_m);
    if (penetration_m <= 0.) return {0., 0.};
    const double total = validation::derived(spring + damping_ns_m * rate_mps, "tabulated normal force");
    return {std::max(0., total), energy};
}
} // namespace contactlaw
