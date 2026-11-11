// Launch part 2 — the rider's launch table and the start preparation it
// drives: the pure table rules (task 1), the setup wiring of the cog and
// pedal angle (task 2) and of the lean (task 3).
#include "support/fixture_test.hpp"
#include "diag.hpp"
#include "rider/launch_config.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

NATIVE_DIAG_PUSH
// NATIVE_DIAG_IGNORE is the project diagnostic-pragma macro from diag.hpp; cppcheck cannot resolve it.
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace {

// Runs `action`, which must throw std::invalid_argument naming `needle`.
template <typename Action>
void rejects(Action &&action, std::string_view needle, std::string_view what) {
    bool threw = false;
    try {
        std::forward<Action>(action)();
    } catch (const std::invalid_argument &failure) {
        threw = true;
        bt::require(std::string_view(failure.what()).find(needle) != std::string_view::npos,
                    std::string(what) + " names " + std::string(needle) + " (" + failure.what() + ")");
    }
    bt::require(threw, std::string(what) + " is rejected");
}

// A valid three-cog table with a two-knot lean ramp — the control every
// rejection below changes in exactly one place.
rider::LaunchConfig ramp_table() {
    return {.start_cog_grade_breaks = {.03, .08},
            .start_cog_teeth = {24, 33, 51},
            .start_pedal_angle_rad = .6,
            .start_lean_grades = {0., .2},
            .start_lean_rad = {0., .2}};
}

// The cog is stepwise and each interval is closed below: a grade equal to a
// break already takes the next cog; below the first break the first cog,
// above the last break the last one.
void test_cog_is_stepwise_by_grade() {
    const auto table = ramp_table();
    bt::require(rider::prepare_launch(table, -.05).rear_teeth == 24, "downhill takes the first cog");
    bt::require(rider::prepare_launch(table, .0299).rear_teeth == 24, "below the first break");
    bt::require(rider::prepare_launch(table, .03).rear_teeth == 33, "a break belongs to the next cog");
    bt::require(rider::prepare_launch(table, .0799).rear_teeth == 33, "below the second break");
    bt::require(rider::prepare_launch(table, .08).rear_teeth == 51, "the second break takes the last cog");
    bt::require(rider::prepare_launch(table, .3).rear_teeth == 51, "steep takes the last cog");
    rider::LaunchConfig single = table;
    single.start_cog_grade_breaks = {};
    single.start_cog_teeth = {51};
    bt::require(rider::prepare_launch(single, -.2).rear_teeth == 51 &&
                    rider::prepare_launch(single, .2).rear_teeth == 51,
                "a one-cog table starts on that cog at every grade");
}

// The lean interpolates linearly between knots and holds the end values
// outside them; the pedal angle is the table's, whatever the grade.
void test_lean_interpolates_and_angle_is_fixed() {
    const auto table = ramp_table();
    bt::require(std::abs(rider::prepare_launch(table, .05).lean_rad - .05) < 1e-15, "a quarter of the way up the ramp");
    bt::require(rider::prepare_launch(table, -.1).lean_rad == 0., "below the first knot holds the first lean");
    bt::require(rider::prepare_launch(table, .5).lean_rad == .2, "above the last knot holds the last lean");
    bt::require(rider::prepare_launch(table, .15).crank_phase_rad == .6, "the pedal angle comes from the table");
    rider::LaunchConfig flat = table;
    flat.start_lean_grades = {0.};
    flat.start_lean_rad = {.04};
    bt::require(rider::prepare_launch(flat, .3).lean_rad == .04, "one knot is a constant lean");
}

// Every shape or domain error names its field; the valid control passes.
void test_table_rules_name_the_field() {
    (void)rider::prepare_launch(ramp_table(), 0.);
    auto with = [](auto change) { auto t = ramp_table(); change(t); return t; };
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_teeth = {}; t.start_cog_grade_breaks = {}; })); },
            "start_cog_teeth", "an empty cog list");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_teeth = {24, 2, 51}; })); },
            "start_cog_teeth", "a two-tooth cog");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.03}; })); },
            "start_cog_grade_breaks", "one break too few");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.03, .03}; })); },
            "start_cog_grade_breaks", "equal breaks");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_cog_grade_breaks = {.08, .03}; })); },
            "start_cog_grade_breaks", "descending breaks");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_cog_grade_breaks = {std::numeric_limits<double>::quiet_NaN(), .08}; })); },
            "start_cog_grade_breaks", "a NaN break");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_pedal_angle_rad = std::numeric_limits<double>::infinity(); })); },
            "start_pedal_angle_rad", "an infinite pedal angle");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_grades = {}; t.start_lean_rad = {}; })); },
            "start_lean_grades", "no lean knot");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_rad = {0.}; })); },
            "start_lean_rad", "a lean count that differs from the knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) { t.start_lean_grades = {.2, 0.}; })); },
            "start_lean_grades", "descending lean knots");
    rejects([&] { rider::validate_launch_table(with([](auto &t) {
                t.start_lean_rad = {0., std::numeric_limits<double>::quiet_NaN()}; })); },
            "start_lean_rad", "a NaN lean");
    rejects([&] { (void)rider::prepare_launch(ramp_table(), std::numeric_limits<double>::infinity()); },
            "grade", "an infinite preparation grade");
    const rider::LaunchConfig unset;
    rejects([&] { rider::validate_launch_table(unset); }, "start_cog_teeth", "a default-constructed table");
}

} // namespace

int main() {
    return bt::run([] {
        test_cog_is_stepwise_by_grade();
        test_lean_interpolates_and_angle_is_fixed();
        test_table_rules_name_the_field();
    });
}
NATIVE_DIAG_POP
