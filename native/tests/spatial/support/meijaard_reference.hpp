#pragma once

#include "../../../src/model/benchmark_whipple.hpp"

#include <array>

namespace spatial_tests {

struct WhippleMatrices { std::array<double, 4> M, C1, K0, K2; };
// Independent Appendix A reference, separate from the runtime model builder.
WhippleMatrices reference_matrices(const bike::model::WhippleParameters&);

} // namespace spatial_tests
