#pragma once
#include "../geometry/specs.hpp"
#include <cstdint>
#include "../kinematics/solver.hpp"
#include "../terrain/types.hpp"
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace bike::viz {
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class SeriesStyle : std::uint8_t { line, step, bars, points };
struct Series {
    std::string name;
    std::vector<double> x, y;
    std::string color{};
    SeriesStyle style = SeriesStyle::line;
    bool secondary_y = false;
    bool dashed = false;
};
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class MarkerKind : std::uint8_t { vertical, horizontal, band };
struct Marker {
    MarkerKind kind = MarkerKind::vertical;
    double first = 0., second = 0.;
    std::string label, color = "#a855f7";
    // "paper" spans the figure; otherwise this panel's y domain.
    bool full_height = false;
};
struct Plot {
    std::string title, x_label, y_label;
    std::vector<Series> series;
    std::vector<Marker> markers{};
    std::vector<std::string> notes{};
    std::string secondary_y_label{};
    bool log_y = false, equal_axes = false;
    std::optional<std::pair<double, double>> x_range{}, y_range{};
};
struct Report {
    std::string title;
    std::vector<Plot> panels;
    unsigned columns = 1, width = 1280, height = 800;
};
using Column = std::variant<std::vector<double>, std::vector<std::string>>;
using Channels = std::vector<std::pair<std::string, Column>>;
[[nodiscard]] const std::vector<double>* numeric(const Channels&, std::string_view name);
[[nodiscard]] const std::vector<std::string>* strings(const Channels&, std::string_view name);
[[nodiscard]] Channels load_ride_csv(const std::filesystem::path&);
void validate(const Plot&);
void validate(const Report&);
[[nodiscard]] std::string to_svg(const Plot&);
[[nodiscard]] std::string to_svg(const Report&);
// Interactive reports embed an installed local Plotly resource when present;
// otherwise the first-party native-html-v1 backend provides offline SVG charts,
// hover, legends, pan/zoom and exports. Neither backend uses a CDN.
// Nonfinite points serialize as null, preserving gaps in the recorded data.
[[nodiscard]] std::string to_html(const Plot&);
[[nodiscard]] std::string to_html(const Report&);
[[nodiscard]] std::string to_html(std::span<const Report>);
void write_png(const Plot&, const std::filesystem::path&, unsigned width, unsigned height);
void write_png(const Report&, const std::filesystem::path&);
void write_artifact(const Report&, const std::filesystem::path&);
[[nodiscard]] bool png_available() noexcept;
// Optional explicit SDK/resource location; omitted uses the installed
// share/bike-sim/plotly tree next to the running executable or source fallback.
void set_plotly_resource(const std::filesystem::path& js_file);

[[nodiscard]] Report geometry_plot(const geometry::BikeSpecs&,
    const kinematics::HorstLinkageSolver&);
[[nodiscard]] Report leverage_plot(const kinematics::HorstLinkageSolver&);
[[nodiscard]] Report compressed_comparison(const geometry::BikeSpecs&,
    const kinematics::HorstLinkageSolver&);
[[nodiscard]] Report damper_dyno();
[[nodiscard]] Report travel_plot(const Channels&, const terrain::TrackSpec&, double fork_mm, double shock_mm);
[[nodiscard]] Report shaft_velocity_plot(const Channels&, std::span<const unsigned char> window = {}, unsigned bins = 80);
[[nodiscard]] Report acceleration_plot(const Channels&, double sample_interval_s, const terrain::TrackSpec&);
[[nodiscard]] Report tyre_plot(const Channels&, const terrain::TrackSpec&);
[[nodiscard]] Report track_profile(const terrain::TrackSpec&, double resolution_m = .005);
[[nodiscard]] std::vector<Report> physical_ride_plots(const Channels&);
[[nodiscard]] std::vector<Report> physical_ride_figures(const Channels&);
[[nodiscard]] std::vector<std::pair<double, std::string>> shift_events(const Channels&);
[[nodiscard]] std::vector<std::pair<double, std::string>> obstacle_positions(const Channels&);
[[nodiscard]] std::vector<std::filesystem::path> plot_ride(const Channels&, double sample_interval_s,
    const terrain::TrackSpec&, double fork_mm, double shock_mm, const std::filesystem::path&,
    std::span<const unsigned char> window = {});
[[nodiscard]] std::filesystem::path plot_physical_ride_html(const Channels&,
    const std::filesystem::path&, std::string_view filename = "ride.html");
[[nodiscard]] std::string geometry_table(const geometry::BikeSpecs&, const kinematics::HorstLinkageSolver&);
[[nodiscard]] std::string kinematics_table(const kinematics::HorstLinkageSolver&);
[[nodiscard]] std::string suspension_table(const geometry::BikeSpecs&);
} // namespace bike::viz
