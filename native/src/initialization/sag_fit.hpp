#pragma once

#include <array>
#include <functional>

namespace bike::initialization {
using SagEvaluator=std::function<std::array<double,2>(double,double)>;
// Parameters are fork pressure [PSI] and rear coil rate [N/m]; observed
// outputs are fork travel and rear shock shaft stroke [mm] (decision 2026-11-05).
[[nodiscard]] std::array<double,2> fit_sag(const SagEvaluator&,
    const std::array<double,2>& target_mm,const std::array<double,2>& initial,
    const std::array<double,2>& lower,const std::array<double,2>& upper);
} // namespace bike::initialization
