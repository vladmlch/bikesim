#pragma once
#include "allocation.hpp"

namespace bike::rider {
class ActivationLaw {
public:
    ActivationLaw(std::vector<double> state,std::vector<double> speeds,
        std::vector<double> lower,std::vector<double> upper,double dt_s,double tau_s,
        std::optional<double> power_limit);
    [[nodiscard]] double gain() const noexcept {return gain_;}
    [[nodiscard]] std::vector<double> latent(std::span<const double> excitation) const;
    [[nodiscard]] std::vector<double> delivered(std::span<const double> excitation) const;
    [[nodiscard]] Matrix derivative(std::span<const double> excitation) const;
    [[nodiscard]] ExtraConstraint constraint(std::vector<std::size_t> torque_indices,
        std::vector<std::size_t> excitation_indices,std::vector<double> scales) const;
    [[nodiscard]] std::vector<double> project(std::span<const double> x,
        std::span<const std::size_t> torque_indices,std::span<const std::size_t> excitation_indices,
        std::span<const double> scales) const;
private:
    std::vector<double> state_,speeds_,lower_,upper_;
    double dt_s_,tau_s_,gain_;
    std::optional<double> power_limit_;
};
} // namespace bike::rider
