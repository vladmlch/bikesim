#include "tabulated_radial.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "../config_validation.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_tabulated.hpp"
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
    double force = 0.;
    double energy = 0.;
    auto err = bike::physics_core::ErrCode::none;
    const auto status = bike::physics_core::tabulated_radial_elastic_arrays<
        double, bike::physics_core::CpuExactToolkit>(
            static_cast<bike::physics_core::i32>(deflection_m_.size()),
            deflection_m_.data(), force_n_.data(),
            slopes_.data(), integral_.data(),
            penetration_m, force, energy, err);
    if (status == bike::physics_core::Status::out_of_bounds)
        throw MaterialRangeError("tire deflection exceeds supplied curve limit");
    if (status != bike::physics_core::Status::ok)
        (void)validation::derived(
            std::numeric_limits<double>::quiet_NaN(),
            "TabulatedTireSpec.elastic_response");
    return {validation::derived(force, "TabulatedTireSpec.elastic_response"),
            validation::derived(energy, "TabulatedTireSpec.elastic_response")};
}
std::pair<double, double> TabulatedRadial::normal_contact(
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    double penetration_m, double rate_mps, double loss_factor,
    double hysteresis_rate_eps_mps) const {
    validation::finite(rate_mps, "deflection rate");
    validation::nonnegative(loss_factor, "radial loss factor");
    validation::positive(hysteresis_rate_eps_mps,
                         "hysteresis rate scale");
    validation::finite(penetration_m, "tire deflection");
    double total = 0.;
    double energy = 0.;
    auto err = bike::physics_core::ErrCode::none;
    const auto status = bike::physics_core::tabulated_radial_contact_arrays<
        double, bike::physics_core::CpuExactToolkit>(
            static_cast<bike::physics_core::i32>(deflection_m_.size()),
            deflection_m_.data(), force_n_.data(),
            slopes_.data(), integral_.data(),
            penetration_m, rate_mps, loss_factor, hysteresis_rate_eps_mps,
            total, energy, err);
    if (status == bike::physics_core::Status::out_of_bounds)
        throw MaterialRangeError("tire deflection exceeds supplied curve limit");
    if (status != bike::physics_core::Status::ok)
        (void)validation::derived(
            std::numeric_limits<double>::quiet_NaN(),
            "tabulated normal force");
    if (penetration_m <= 0.) return {0., 0.};
    return {validation::derived(total, "tabulated normal force"),
            validation::derived(energy, "TabulatedTireSpec.elastic_response")};
}
} // namespace contactlaw
