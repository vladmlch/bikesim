#pragma once
// Per-pothole "declared depth vs effective wheel drop" rows, read by the
// track preview (`bike-ride --preview`) and the legacy ride summary.
// Moved unchanged from the removed legacy metrics (2026-11-04).

#include "terrain/types.hpp"

#include <string>
#include <vector>

namespace bike::terrain {
struct PotholeReport {
    std::string label,kind;
    double start_m=0.,length_m=0.,declared_depth_mm=0.,effective_drop_front_mm=0.,effective_drop_rear_mm=0.;
};
[[nodiscard]] std::vector<PotholeReport> pothole_reports(const TrackSpec&);
} // namespace bike::terrain
