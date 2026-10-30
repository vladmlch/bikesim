#include "evidence.hpp"
#include <cmath>
#include <stdexcept>

namespace bike::research {
EnergyQuality energy_quality(const runtime::WireObject& energy, double maximum_ratio) {
    if (!std::isfinite(maximum_ratio) || maximum_ratio <= 0.)
        throw std::invalid_argument("maximum energy residual ratio must be positive and finite");
    const auto quality = runtime::research_quality(energy, maximum_ratio);
    return {.residual_ratio=quality.residual_ratio, .acceptable=quality.acceptable};
}
std::vector<std::string> model_violations(const runtime::PhysicalSampleData& sample,
    double maximum_compression_fraction, double maximum_linkage_error_m) {
    return runtime::channel_violations(sample.channels, maximum_compression_fraction, maximum_linkage_error_m);
}
} // namespace bike::research
