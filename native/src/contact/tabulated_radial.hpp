#pragma once

#include <utility>
#include <vector>
#include <stdexcept>

#include "../config_types.hpp"

namespace contactlaw {
class MaterialRangeError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
    ~MaterialRangeError() override;
    MaterialRangeError(const MaterialRangeError&) = default;
    MaterialRangeError& operator=(const MaterialRangeError&) = default;
    MaterialRangeError(MaterialRangeError&&) = default;
    MaterialRangeError& operator=(MaterialRangeError&&) = default;
};
// tire_curve.py TabulatedTireSpec: immutable input and prefix-integral
// caches built before stepping. Pressure is metadata, never a multiplier.
class TabulatedRadial {
public:
    explicit TabulatedRadial(const nativecfg::TireTable &table);
    [[nodiscard]] std::pair<double, double> elastic_response(double penetration_m) const;
    // Decisions 22/39 (2026-11-10): the radial loss is the same
    // rate-independent hysteresis as the linear material — η·F_elastic·
    // tanh(δ̇/ε), not a viscous c·δ̇.
    [[nodiscard]] std::pair<double, double> normal_contact(
        double penetration_m, double rate_mps, double loss_factor,
        double hysteresis_rate_eps_mps) const;
private:
    std::vector<double> deflection_m_, force_n_, slopes_, integral_;
};
} // namespace contactlaw
