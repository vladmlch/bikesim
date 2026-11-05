// test_pothole_report.cpp — pothole_reports() feeds the "declared depth vs
// effective wheel drop" panel of `bike-ride --preview`. It moved from the
// removed legacy metrics unchanged; this pins its contract.
#include "support/fixture_test.hpp"
#include "terrain/pothole_report.hpp"
#include "terrain/trackfile.hpp"

namespace {
// One report per pothole, in track order, with drops from the wheel radii.
void reports_every_pothole_in_order() {
    const auto track = bike::terrain::get_preset("road_worn");
    const auto reports = bike::terrain::pothole_reports(track);
    std::size_t potholes = 0;
    for (const auto& obstacle : track.sorted_obstacles()) potholes += obstacle.is_pothole() ? 1 : 0;
    bt::require(potholes > 0 && reports.size() == potholes, "one report per pothole");
    for (std::size_t i = 1; i < reports.size(); ++i)
        bt::require(reports[i - 1].start_m <= reports[i].start_m, "reports follow track order");
    for (const auto& r : reports) {
        bt::require(r.declared_depth_mm >= 0. && r.length_m > 0., "depth and length are physical");
        bt::require(r.effective_drop_front_mm <= r.declared_depth_mm + 1e-9 &&
                    r.effective_drop_rear_mm <= r.declared_depth_mm + 1e-9,
                    "a wheel never drops deeper than the declared hole");
    }
}
} // namespace

int main() { return bt::run([] { reports_every_pothole_in_order(); }); }
