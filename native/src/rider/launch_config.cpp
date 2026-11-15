// rider/launch_config.cpp — the launch table rules and the pure start
// preparation (launch plan, part 2).
#include "launch_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "intent.hpp"
#include "table_lookup.hpp"

namespace rider {
namespace {

[[noreturn]] void reject_section(std::string_view section, std::string_view field,
                                 std::string_view why) {
    throw std::invalid_argument(
        std::string(section) + "." + std::string(field) + ": " + std::string(why));
}

[[noreturn]] void reject(std::string_view field, std::string_view why) {
    reject_section("rider_launch", field, why);
}

void validate_strictly_ascending(std::span<const double> values,
                                 std::string_view section,
                                 std::string_view field) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) reject_section(section, field, "values must be finite");
        if (i > 0 && !(values[i - 1] < values[i]))
            reject_section(section, field, "values must ascend strictly");
    }
}

void validate_knot_table_shape(std::span<const double> knots,
                               std::span<const double> values,
                               std::string_view section,
                               std::string_view knots_field,
                               std::string_view values_field,
                               std::string_view mismatch_why) {
    if (knots.empty()) reject_section(section, knots_field, "needs at least one knot");
    if (values.size() != knots.size()) reject_section(section, values_field, mismatch_why);
    validate_strictly_ascending(knots, section, knots_field);
}

void validate_grade_cap_table(std::span<const double> grades,
                              std::span<const double> caps_nm,
                              std::string_view section,
                              std::string_view grades_field,
                              std::string_view caps_field) {
    validate_knot_table_shape(
        grades, caps_nm, section, grades_field, caps_field,
        "needs one cap per " + std::string(grades_field) + " knot");
    for (const double cap : caps_nm) {
        if (!(std::isfinite(cap) && cap > 0.))
            reject_section(section, caps_field, "must be finite and positive");
    }
}

} // namespace

void validate_launch_table(const LaunchConfig &table) {
    if (table.start_cog_teeth.empty()) reject("start_cog_teeth", "needs at least one cog");
    for (const int teeth : table.start_cog_teeth)
        if (teeth < 3) reject("start_cog_teeth", "a cog has at least three teeth");
    if (table.start_cog_grade_breaks.size() + 1 != table.start_cog_teeth.size())
        reject("start_cog_grade_breaks", "needs exactly one break fewer than start_cog_teeth");
    validate_strictly_ascending(table.start_cog_grade_breaks, "rider_launch", "start_cog_grade_breaks");
    if (!std::isfinite(table.start_pedal_angle_rad)) reject("start_pedal_angle_rad", "must be finite");
    validate_knot_table_shape(
        table.start_lean_grades, table.start_lean_rad, "rider_launch",
        "start_lean_grades", "start_lean_rad", "needs one lean per start_lean_grades knot");
    for (const double lean : table.start_lean_rad)
        if (!std::isfinite(lean)) reject("start_lean_rad", "values must be finite");
    const auto nonnegative = [](double v, std::string_view field) {
        if (!(std::isfinite(v) && v >= 0.)) reject(field, "must be finite and nonnegative");
    };
    const auto positive = [](double v, std::string_view field) {
        if (!(std::isfinite(v) && v > 0.)) reject(field, "must be finite and positive");
    };
    nonnegative(table.standstill_speed_mps, "standstill_speed_mps");
    nonnegative(table.standstill_crank_rpm, "standstill_crank_rpm");
    positive(table.effort_rate_nm_s, "effort_rate_nm_s");
    positive(table.effort_cap_asphalt_nm, "effort_cap_asphalt_nm");
    positive(table.effort_cap_hardpack_nm, "effort_cap_hardpack_nm");
    positive(table.effort_cap_loose_nm, "effort_cap_loose_nm");
    positive(table.effort_cap_wet_nm, "effort_cap_wet_nm");
    positive(table.effort_cap_scree_nm, "effort_cap_scree_nm");
    validate_grade_cap_table(
        table.effort_cap_grades, table.effort_cap_grade_nm,
        "rider_launch", "effort_cap_grades", "effort_cap_grade_nm");
    positive(table.effort_rise_speed_mps, "effort_rise_speed_mps");
    validate_knot_table_shape(
        table.release_torque_grades, table.release_torque_nm, "rider_launch",
        "release_torque_grades", "release_torque_nm",
        "needs one torque per release_torque_grades knot");
    for (const double torque : table.release_torque_nm) nonnegative(torque, "release_torque_nm");
    nonnegative(table.brake_release_s, "brake_release_s");
    positive(table.max_hold_s, "max_hold_s");
    positive(table.upshift_crank_rpm, "upshift_crank_rpm");
    positive(table.shift_retry_s, "shift_retry_s");
    nonnegative(table.end_speed_mps, "end_speed_mps");
    nonnegative(table.end_crank_rpm, "end_crank_rpm");
    nonnegative(table.end_hold_s, "end_hold_s");
    positive(table.max_launch_s, "max_launch_s");
    nonnegative(table.handover_s, "handover_s");
}

LaunchPreparation prepare_launch(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    const int rear_teeth = detail::launch_teeth_unchecked(table, grade);
    const double lean = detail::piecewise_linear_knot(
        table.start_lean_grades, table.start_lean_rad, grade);
    return {.rear_teeth = rear_teeth,
            .crank_phase_rad = table.start_pedal_angle_rad,
            .lean_rad = lean};
}

double release_torque_nm(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    if (!std::isfinite(grade)) throw std::invalid_argument("launch release grade must be finite");
    return detail::piecewise_linear_knot(
        table.release_torque_grades, table.release_torque_nm, grade);
}

double effort_grade_cap_nm(const LaunchConfig &table, double grade) {
    validate_launch_table(table);
    if (!std::isfinite(grade)) throw std::invalid_argument("launch grade cap grade must be finite");
    return detail::piecewise_linear_knot(
        table.effort_cap_grades, table.effort_cap_grade_nm, grade);
}

namespace detail {
double grade_cap_unchecked(const LaunchConfig &table, double grade) {
    return piecewise_linear_knot(table.effort_cap_grades, table.effort_cap_grade_nm, grade);
}

int launch_teeth_unchecked(const LaunchConfig &table, double grade) {
    if (!std::isfinite(grade)) throw std::invalid_argument("launch preparation grade must be finite");
    // upper_bound counts the breaks at or below the grade: a grade equal to
    // a break already takes the next cog.
    const auto &breaks = table.start_cog_grade_breaks;
    const auto cog = static_cast<std::size_t>(
        std::distance(breaks.begin(), std::ranges::upper_bound(breaks, grade)));
    return table.start_cog_teeth[cog];
}
} // namespace detail

void validate_launch_plan_context(const LaunchConfig &table, double period_s) {
    validate_launch_table(table);
    if (!(std::isfinite(period_s) && period_s > 0.))
        throw std::invalid_argument("launch plan needs a positive intention period");
    const auto on_grid = [period_s](double seconds, std::string_view field) {
        const double ticks = std::rint(seconds / period_s);
        if (std::abs(ticks * period_s - seconds) > 1e-9) reject(field, "must be a whole number of intention periods");
        if (ticks > 0x1p31) reject(field, "must be at most 2^31 intention periods");
        return static_cast<std::int64_t>(ticks);
    };
    // A positive-required duration must not round down to an empty interval.
    const auto positive_on_grid = [&on_grid](double seconds, std::string_view field) {
        if (on_grid(seconds, field) == 0)
            reject(field, "a positive time must count at least one intention period tick");
    };
    on_grid(table.brake_release_s, "brake_release_s");
    positive_on_grid(table.max_hold_s, "max_hold_s");
    on_grid(table.end_hold_s, "end_hold_s");
    positive_on_grid(table.max_launch_s, "max_launch_s");
    on_grid(table.handover_s, "handover_s");
    positive_on_grid(table.shift_retry_s, "shift_retry_s");
}

void validate_climb_effort_cap(const IntentConfig &config) {
    validate_grade_cap_table(
        config.climb_effort_cap_grades,
        config.climb_effort_cap_grade_nm,
        "seated_climb",
        "climb_effort_cap_grades",
        "climb_effort_cap_grade_nm");
    if (!(std::isfinite(config.climb_effort_rise_speed_mps) &&
          config.climb_effort_rise_speed_mps >= 0.)) {
        throw std::invalid_argument(
            "seated_climb.climb_effort_rise_speed_mps: must be finite and nonnegative");
    }
    if (!(std::isfinite(config.climb_effort_full_speed_mps) &&
          config.climb_effort_full_speed_mps >= config.climb_effort_rise_speed_mps)) {
        throw std::invalid_argument(
            "seated_climb.climb_effort_full_speed_mps: must be finite and >= climb_effort_rise_speed_mps");
    }
}

} // namespace rider
