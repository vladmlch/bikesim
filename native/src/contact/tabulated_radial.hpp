#pragma once

#include <utility>
#include <vector>
#include <stdexcept>

#include "../config_types.hpp"

namespace contactlaw {
class MaterialRangeError : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};
// tire_curve.py TabulatedTireSpec: immutable input and prefix-integral
// caches built before stepping. Pressure is metadata, never a multiplier.
class TabulatedRadial {
public:
    explicit TabulatedRadial(const nativecfg::TireTable &table);
    [[nodiscard]] std::pair<double, double> elastic_response(double penetration_m) const;
    [[nodiscard]] std::pair<double, double> normal_contact(
        double penetration_m, double rate_mps, double damping_ns_m) const;
private:
    std::vector<double> deflection_m_, force_n_, slopes_, integral_;
};
} // namespace contactlaw
