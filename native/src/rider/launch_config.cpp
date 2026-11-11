// rider/launch_config.cpp — the launch table rules and the pure start
// preparation (launch plan, part 2).
#include "launch_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rider {
namespace {

[[noreturn]] void reject(std::string_view field, std::string_view why) {
    throw std::invalid_argument("rider_launch." + std::string(field) + ": " + std::string(why));
}

void strictly_ascending(const std::vector<double> &values, std::string_view field) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) reject(field, "values must be finite");
        if (i > 0 && !(values[i - 1] < values[i])) reject(field, "values must ascend strictly");
    }
}

} // namespace

void validate_launch_table(const LaunchConfig &table) {
    if (table.start_cog_teeth.empty()) reject("start_cog_teeth", "needs at least one cog");
    for (const int teeth : table.start_cog_teeth)
        if (teeth < 3) reject("start_cog_teeth", "a cog has at least three teeth");
    if (table.start_cog_grade_breaks.size() + 1 != table.start_cog_teeth.size())
        reject("start_cog_grade_breaks", "needs exactly one break fewer than start_cog_teeth");
    strictly_ascending(table.start_cog_grade_breaks, "start_cog_grade_breaks");
    if (!std::isfinite(table.start_pedal_angle_rad)) reject("start_pedal_angle_rad", "must be finite");
    if (table.start_lean_grades.empty()) reject("start_lean_grades", "needs at least one knot");
    if (table.start_lean_rad.size() != table.start_lean_grades.size())
        reject("start_lean_rad", "needs one lean per start_lean_grades knot");
    strictly_ascending(table.start_lean_grades, "start_lean_grades");
    for (const double lean : table.start_lean_rad)
        if (!std::isfinite(lean)) reject("start_lean_rad", "values must be finite");
}

LaunchPreparation prepare_launch(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    if (!std::isfinite(grade)) throw std::invalid_argument("launch preparation grade must be finite");
    // upper_bound counts the breaks at or below the grade: a grade equal to
    // a break already takes the next cog.
    const auto &breaks = table.start_cog_grade_breaks;
    const auto cog = static_cast<std::size_t>(
        std::distance(breaks.begin(), std::ranges::upper_bound(breaks, grade)));
    const auto &knots = table.start_lean_grades;
    const auto &leans = table.start_lean_rad;
    double lean = leans.front();
    if (grade >= knots.back()) {
        lean = leans.back();
    } else if (grade > knots.front()) {
        const auto upper = static_cast<std::size_t>(
            std::distance(knots.begin(), std::ranges::upper_bound(knots, grade)));
        const double t = (grade - knots[upper - 1]) / (knots[upper] - knots[upper - 1]);
        lean = leans[upper - 1] + t * (leans[upper] - leans[upper - 1]);
    }
    return {.rear_teeth = table.start_cog_teeth[cog],
            .crank_phase_rad = table.start_pedal_angle_rad,
            .lean_rad = lean};
}

} // namespace rider
