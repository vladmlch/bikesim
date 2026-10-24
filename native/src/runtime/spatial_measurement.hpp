#pragma once

// Cold JSON projection for the frozen W00 performance records.
#include "spatial_performance.hpp"
#include "../io/value.hpp"

namespace runtime::measurement {
[[nodiscard]] bike::io::Value spatial_profile_value(const spatial::SpatialPerformanceProfile&);
[[nodiscard]] bike::io::Value region_timings_value(const spatial::RegionTimings&);
// Spatial reports reject mixed identities, mislabeled chunk latencies and
// non-finite/zero clocks; only complete strict episodes enter the median.
[[nodiscard]] bike::io::Value build_spatial_report(const bike::io::Value::Array& warmups,
    const bike::io::Value::Array& runs);
} // namespace runtime::measurement
