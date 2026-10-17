#pragma once
#include <cstddef>
#include <functional>

namespace bike::numerics {
struct BrentResult { double root=0.; std::size_t iterations=0; std::size_t function_calls=0; };
[[nodiscard]] BrentResult brentq(const std::function<double(double)>& function,
    double lower,double upper,double xtol=2e-12,double rtol=8.881784197001252e-16,
    std::size_t max_iterations=100);
} // namespace bike::numerics
