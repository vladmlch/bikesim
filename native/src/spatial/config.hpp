#pragma once

#include "io/value.hpp"
#include "spatial/types.hpp"

#include <cstdint>
#include <optional>

namespace bike::spatial {

// Returns std::nullopt when root_object has no "spatial" key (planar mode).
[[nodiscard]] std::optional<SpatialConfig> parse_spatial_config(
    const bike::io::Value::Object& root_object, double timestep_s);

void validate_spatial_config(const SpatialConfig& config, double timestep_s);
[[nodiscard]] bike::io::Value spatial_config_to_value(const SpatialConfig& config);

// Exact integer-multiple check (Decision 12): ratio = duration_s / step_s must be
// an integer >= min_ticks within 1e-12 relative tolerance; rejects silent rounding.
[[nodiscard]] std::int64_t require_exact_ticks(
    double duration_s, double step_s, std::int64_t min_ticks, const char* field_path);

} // namespace bike::spatial
