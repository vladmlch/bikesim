// T50 semantic plots, file formats and input-validation regressions. These are
// authored tests, not evidence of executed image/HTML or scientific parity gates.
#include "support/migration_test.hpp"
#include "viz/plot.hpp"
#include "platform/files.hpp"
#include "io/json.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>

namespace {
namespace viz = bike::viz;
struct TempDir {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("bike-t50-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    TempDir() { if (!std::filesystem::create_directory(path)) throw std::runtime_error("T50 temp directory already exists"); }
    ~TempDir() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
};
void write(const std::filesystem::path& path, std::string_view value) {
    bike::platform::write_exclusive(path, std::as_bytes(std::span(value)));
}
const bike::io::Value& member(const bike::io::Value& value, std::string_view name) {
    const auto* found = bike::io::find_member(bike::io::as_object(value, "plot"), name);
    if (!found) throw std::runtime_error("missing plot field " + std::string(name));
    return *found;
}
bike::io::Value embedded(const std::string& html) {
    constexpr std::string_view marker = R"(<script id="bike-plot-data" type="application/json">)";
    const auto begin = html.find(marker);
    bt::require(begin != std::string::npos, "offline HTML has an explicit semantic payload");
    const auto start = begin + marker.size(), end = html.find("</script>", start);
    bt::require(end != std::string::npos, "semantic payload has a complete closing boundary");
    return bike::io::parse_json(std::string_view(html).substr(start, end - start));
}
void formats() {
    const double missing = std::numeric_limits<double>::quiet_NaN();
    viz::Plot plot{.title="Torque <front> — </script><script>alert(1)</script>", .x_label="time (s)", .y_label="Nm", .series={}};
    plot.series.push_back({.name="measured", .x={0., 1., 2., 3.}, .y={1., missing, 3., 2.}, .color="#38bdf8"});
    plot.series.push_back({.name="requested", .x={0., 1., 2., 3.}, .y={2., 2., 4., 4.}, .color="#fbbf24", .style=viz::SeriesStyle::step, .secondary_y=true, .dashed=true});
    plot.secondary_y_label = "W";
    plot.markers.push_back({.kind=viz::MarkerKind::vertical, .first=1.5, .second=0., .label="shift 21->24", .color="#a855f7", .full_height=true});
    plot.markers.push_back({.kind=viz::MarkerKind::band, .first=1., .second=1.5, .label="torque cut"});
    const viz::Report report{.title=plot.title, .panels={plot}, .columns=1, .width=640, .height=480};
    const auto svg = viz::to_svg(report), html = viz::to_html(report);
    bt::require(svg.find("&lt;front&gt;") != std::string::npos && svg.find("<script>alert") == std::string::npos,
        "SVG escapes labels while retaining Unicode text");
    bt::require(html.find("<script src=") == std::string::npos && html.find("https://cdn.plot.ly") == std::string::npos &&
        html.find("bike-plot-backend") != std::string::npos && html.find("Export SVG") != std::string::npos,
        "HTML declares its renderer, embeds scripts locally and exposes exports");
    const auto document = embedded(html);
    const auto& figures = bike::io::as_array(document, "figures");
    const auto& panels = bike::io::as_array(member(figures.at(0), "panels"), "panels");
    const auto& series = bike::io::as_array(member(panels.at(0), "series"), "series");
    const auto& values = bike::io::as_array(member(series.at(0), "y"), "y");
    bt::require(values.size() == 4 && bike::io::is_null(values.at(1)),
        "nonfinite samples become explicit gaps without dropping or shifting coordinates");
    bt::require(bike::io::as_bool(member(series.at(1), "secondary"), "secondary") &&
        bike::io::as_bool(member(series.at(1), "dashed"), "dashed") &&
        bike::io::as_string(member(series.at(1), "style"), "style") == "step",
        "interactive payload retains secondary axes, stepped gears and dashed traces");

    TempDir temporary;
    const auto image_path = temporary.path / "plot.png";
    viz::write_png(report, image_path);
    const auto png = bt::text(image_path);
    bt::require(viz::png_available() && png.starts_with(std::string("\x89PNG\r\n\x1a\n", 8)) &&
        png.find("IHDR") == 12 && png.find("IDAT") != std::string::npos && png.find("IEND") != std::string::npos,
        "headless raster export writes a real PNG container without a display service");
    bt::require(static_cast<unsigned char>(png.at(18)) == 2 && static_cast<unsigned char>(png.at(19)) == 128 &&
        static_cast<unsigned char>(png.at(22)) == 1 && static_cast<unsigned char>(png.at(23)) == 224,
        "PNG dimensions match the requested 640 by 480 export");
    auto bad = plot; bad.series[0].y.pop_back();
    bt::invalid([&] { viz::validate(bad); });
    bad = plot; bad.series[0].color = "red;script";
    bt::invalid([&] { viz::validate(bad); });
    bad = std::move(plot); bad.log_y = true; bad.y_range = std::pair{-1., 1.};
    bt::invalid([&] { viz::validate(bad); });
    bt::invalid([&] { viz::write_artifact(report, temporary.path / "plot.exe"); });
}
void csv_and_annotations() {
    TempDir temporary;
    write(temporary.path / "preview.csv", "time_s,x_m,event,note\r\n0,1,,\"a,b\"\r\n0.1,,reset,\"line1\nline2 \"\"quoted\"\"\"\r\n");
    const auto columns = viz::load_ride_csv(temporary.path / "preview.csv");
    bt::require(viz::numeric(columns, "time_s") && viz::numeric(columns, "time_s")->size() == 2 &&
        std::isnan(viz::numeric(columns, "x_m")->at(1)), "CSV blank numeric cells stay explicit missing samples");
    bt::require(viz::strings(columns, "note") && viz::strings(columns, "note")->at(1) == "line1\nline2 \"quoted\"",
        "CSV reader handles commas, newlines and escaped quotes in one field");
    write(temporary.path / "duplicate.csv", "x,x\n1,2\n");
    bt::invalid([&] { (void)viz::load_ride_csv(temporary.path / "duplicate.csv"); });
    write(temporary.path / "broken.csv", "x,note\n1,\"unterminated\n");
    bt::invalid([&] { (void)viz::load_ride_csv(temporary.path / "broken.csv"); });
    write(temporary.path / "empty.csv", "time_s,x_m\n");
    bt::require(viz::load_ride_csv(temporary.path / "empty.csv").empty(), "header-only preview has no invented rows");

    viz::Channels shifts;
    shifts.emplace_back("time_s", std::vector<double>{0., 1., 2., 3., 4., 5., 6., 7.});
    shifts.emplace_back("x_m", std::vector<double>{0., 2., 4., 6., 8., 10., 12., 14.});
    shifts.emplace_back("shift_count", std::vector<double>{0., 1., 2., 3., 4., 5., 6., 7.});
    shifts.emplace_back("last_shift_direction", std::vector<std::string>(8, "up"));
    shifts.emplace_back("last_shift_from_teeth", std::vector<double>(8, 21.));
    shifts.emplace_back("last_shift_to_teeth", std::vector<double>(8, 24.));
    shifts.emplace_back("obstacle", std::vector<std::string>{"hole @10m", "hole @10m", "root @-2.5m", "none", "rock @25m", "rock @25m", "", ""});
    const auto events = viz::shift_events(shifts), obstacles = viz::obstacle_positions(shifts);
    bt::require(events.size() == 7 && events.front().first == 1. && events.back().second == "up:21->24",
        "seven saved shift transitions produce seven ordered annotations");
    bt::require(obstacles.size() == 3 && obstacles.front().first == -2.5 && obstacles.back().first == 25.,
        "obstacle labels deduplicate and retain signed positions before sorting");
    const auto reports = viz::physical_ride_figures(shifts);
    bt::require(reports.size() == 2 && reports[0].panels.size() == 5 && reports[1].panels.size() == 2 &&
        reports[0].panels[3].markers.size() == 7 && reports[1].panels[0].markers.size() == 3 &&
        reports[1].panels[0].markers[0].full_height,
        "physical HTML preserves five drivetrain panels, two suspension panels and figure obstacle annotations");
    const auto physical = viz::physical_ride_plots(shifts);
    bt::require(physical.size() == 4 && physical[0].title == "speed" && physical[1].title == "normal_load" &&
        physical[2].title == "power" && physical[3].title == "energy_residual",
        "physical static exports keep their canonical artifact names even with optional channels absent");
}
}
// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() { return bt::run([] { formats(); csv_and_annotations(); }); }
