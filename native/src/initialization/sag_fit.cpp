#include "sag_fit.hpp"
#include "../numerics/least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace bike::initialization {
std::array<double,2> fit_sag(const SagEvaluator& evaluate,
    const std::array<double,2>& target,const std::array<double,2>& initial,
    const std::array<double,2>& lower,const std::array<double,2>& upper) {
    if(!evaluate)throw std::invalid_argument("equilibrium sag evaluator is required");
    std::array<double,2>x{},lo{},hi{};
    for(std::size_t i=0;i<2;++i){
        if(!std::isfinite(target[i])||!std::isfinite(initial[i])||!std::isfinite(lower[i])||!std::isfinite(upper[i]))
            throw std::invalid_argument("expected finite two-element sag arrays");
        if(target[i]<=0.||lower[i]<=0.||upper[i]<=lower[i])throw std::invalid_argument("invalid sag target or bounds");
        if(initial[i]<lower[i]||initial[i]>upper[i])throw std::invalid_argument("initial sag parameters lie outside bounds");
        x[i]=std::log(initial[i]);lo[i]=std::log(lower[i]);hi[i]=std::log(upper[i]);
    }
    auto const residual=[&](std::span<const double> values){
        const auto measured=evaluate(std::exp(values[0]),std::exp(values[1]));
        if(!std::isfinite(measured[0])||!std::isfinite(measured[1]))throw std::invalid_argument("invalid equilibrium output");
        return std::vector<double>{measured[0]-target[0],measured[1]-target[1]};
    };
    numerics::LeastSquaresOptions options;
    options.max_evaluations=40;options.relative_difference_step=1e-3;
    const auto result=numerics::bounded_least_squares(residual,x,lo,hi,options);
    const auto error=residual(result.x);
    const double magnitude=std::max(std::abs(error[0]),std::abs(error[1]));
    if(!result.converged||magnitude>.5)throw std::runtime_error("requested sag is not achievable within parameter bounds: best error "+std::to_string(magnitude)+" mm");
    return {std::exp(result.x[0]),std::exp(result.x[1])};
}
} // namespace bike::initialization
