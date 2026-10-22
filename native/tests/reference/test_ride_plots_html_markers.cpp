// Completion 3.22b. Source: tests/reference/test_ride_plots_html_markers.py.
// Native figures carry Marker records on their panels instead of Plotly
// layout shapes/annotations: a shift event is a vertical Marker on the gear
// panel of report 0, an obstacle is a full_height (paper-relative) Marker on
// the suspension report's panels.
#include "support/migration_test.hpp"
#include "viz/plot.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace {

bike::viz::Channels channels(int n_rows, int n_shifts, int n_obstacles) {
    bike::viz::Channels out;
    const auto numeric = [&](const char *name, double value) {
        out.emplace_back(name, std::vector<double>(static_cast<std::size_t>(n_rows), value));
    };
    std::vector<double> time(static_cast<std::size_t>(n_rows)),
        x(static_cast<std::size_t>(n_rows));
    for (int i = 0; i < n_rows; ++i) {
        time[static_cast<std::size_t>(i)] = i * .01;
        x[static_cast<std::size_t>(i)] = i * .01 * 3.;
    }
    out.emplace_back("time_s", time);
    out.emplace_back("x_m", x);
    numeric("speed_kmh", 10.);
    numeric("grade_pct", 0.);
    numeric("cadence_rpm", 70.);
    numeric("required_cadence_rpm", 70.);
    numeric("shift_torque_factor", 1.);
    numeric("gear_rear_teeth", 30.);
    numeric("gear_front_teeth", 34.);
    numeric("freehub_torque_nm", 0.);
    numeric("motor_request_nm", 0.);
    numeric("motor_torque_nm", 0.);
    numeric("front_load_n", 400.);
    numeric("rear_load_n", 500.);
    std::vector<double> count(static_cast<std::size_t>(n_rows), 0.);
    for (int k = 0; k < n_shifts; ++k) {
        const auto i = static_cast<std::size_t>(
            static_cast<int>(1. + (n_rows - 2.) * (n_shifts == 1 ? .5 : k / (n_shifts - 1.))));
        for (std::size_t j = i; j < count.size(); ++j) count[j] = k + 1.;
    }
    out.emplace_back("shift_count", std::move(count));
    out.emplace_back("last_shift_direction",
                     std::vector<std::string>(static_cast<std::size_t>(n_rows), "up"));
    numeric("last_shift_from_teeth", 30.);
    numeric("last_shift_to_teeth", 28.);
    std::vector<std::string> labels(static_cast<std::size_t>(n_rows));
    for (int k = 0; k < n_obstacles; ++k)
        labels[static_cast<std::size_t>((k + 1) * n_rows / (n_obstacles + 1))] =
            "rock" + std::to_string(k) + "@" + std::to_string(k + 1) + ".0m";
    out.emplace_back("obstacle", std::move(labels));
    return out;
}

std::size_t count_markers(const bike::viz::Plot &panel, bike::viz::MarkerKind kind) {
    return static_cast<std::size_t>(std::ranges::count_if(panel.markers,
        [&](const auto &marker) { return marker.kind == kind; }));
}

// test_markers_are_batched_and_match_the_events — every shift becomes one
// vertical marker with the teeth label on the gear panel; every obstacle a
// full-height marker on each suspension panel; the HTML writer emits a file.
void test_markers_are_batched_and_match_the_events() {
    const auto ch = channels(600, 7, 3);
    const auto events = bike::viz::shift_events(ch);
    bt::require(events.size() == 7, "seven shift transitions are detected");
    const auto figures = bike::viz::physical_ride_figures(ch);
    bt::require(figures.size() == 2, "effort and suspension reports are built");
    const auto &gear = figures.front().panels.at(3);
    bt::require(count_markers(gear, bike::viz::MarkerKind::vertical) == 7,
                "the gear panel holds exactly one vertical marker per shift");
    for (const auto &marker : gear.markers)
        bt::require(marker.label == "up:30->28",
                    "each shift marker carries the direction:from->to label");
    const auto &suspension = figures.back();
    for (const auto &panel : suspension.panels) {
        const std::size_t obstacle_markers = static_cast<std::size_t>(
            std::ranges::count_if(panel.markers, [](const auto &marker) {
                return marker.full_height; }));
        bt::require(obstacle_markers == 3,
                    "each suspension panel spans the three obstacle markers");
    }
    std::vector<std::string> obstacle_labels;
    for (const auto &marker : suspension.panels.front().markers)
        if (marker.full_height) obstacle_labels.push_back(marker.label);
    std::ranges::sort(obstacle_labels);
    bt::require(obstacle_labels == std::vector<std::string>(
                    {"rock0@1.0m", "rock1@2.0m", "rock2@3.0m"}),
                "obstacle markers keep the parsed labels");
    const auto destination = std::filesystem::temp_directory_path() /
        "bike_marker_batch_check";
    std::filesystem::create_directories(destination);
    const auto written = bike::viz::plot_physical_ride_html(ch, destination);
    bt::require(std::filesystem::exists(written) && written.filename() == "ride.html",
                "the HTML writer produces the ride page");
    std::filesystem::remove(written);
    std::filesystem::remove(destination);
}

// test_shift_events_ignore_nan_teeth — NaN teeth yield unlabeled events.
void test_shift_events_ignore_nan_teeth() {
    auto ch = channels(50, 2, 0);
    auto &from = std::get<std::vector<double>>(
        std::ranges::find_if(ch, [](const auto &entry) {
            return entry.first == "last_shift_from_teeth"; })->second);
    std::ranges::fill(from, std::nan(""));
    const auto events = bike::viz::shift_events(ch);
    bt::require(events.size() == 2, "both transitions are still detected");
    for (const auto &[time, label] : events)
        bt::require(label.empty(), "a NaN teeth value produces an unlabeled event");
}

// test_html_generation_is_linear_in_ride_length — 10x rows/events must stay
// inside ~25x time; the quadratic per-event path cost ~86x before batching.
void test_html_generation_is_linear_in_ride_length() {
    const auto small = channels(1500, 25, 6);
    const auto big = channels(15000, 250, 6);
    const auto destination = std::filesystem::temp_directory_path() /
        "bike_marker_linear_check";
    std::filesystem::create_directories(destination);
    const auto t0 = std::chrono::steady_clock::now();
    const auto small_path = bike::viz::plot_physical_ride_html(small, destination, "s.html");
    const auto t_small = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    const auto t1 = std::chrono::steady_clock::now();
    const auto big_path = bike::viz::plot_physical_ride_html(big, destination, "b.html");
    const auto t_big = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t1).count();
    bt::require(t_big < 25. * std::max(t_small, .05),
                "HTML generation stays linear in ride length");
    std::filesystem::remove(small_path);
    std::filesystem::remove(big_path);
    std::filesystem::remove(destination);
}

} // namespace

int main() {
    return bt::run([] {
        test_markers_are_batched_and_match_the_events();
        test_shift_events_ignore_nan_teeth();
        test_html_generation_is_linear_in_ride_length();
    });
}
