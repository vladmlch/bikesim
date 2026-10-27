#include "spatial.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bike::viz {
namespace {
const std::vector<double>& required(const Channels& channels, std::string_view name, std::size_t count) {
    const auto* values = numeric(channels, name);
    if (!values) throw std::invalid_argument("spatial plot: missing numeric channel " + std::string(name));
    if (values->size() != count) throw std::invalid_argument("spatial plot: channel width mismatch " + std::string(name));
    return *values;
}
const std::vector<double>& time_axis(const Channels& channels, std::string_view name = "time_s") {
    const auto* time = numeric(channels, name);
    if (!time || time->empty()) throw std::invalid_argument("spatial plot: nonempty timestamps required");
    double previous = -1.;
    for (const double value : *time) {
        if (!std::isfinite(value) || value < 0. || value < previous)
            throw std::invalid_argument("spatial plot: timestamps must be finite and monotone");
        previous = value;
    }
    return *time;
}
void mask_domain(const std::vector<double>& mask) {
    for (const double value : mask) if (value != 0. && value != 1.)
        throw std::invalid_argument("spatial plot: validity must be boolean");
}
void retain_time_extent(Plot& plot, const std::vector<double>& time) {
    // Invalid samples do not enter a Series. Keep their clock extent on the
    // axis, otherwise a trailing gap and its N/A marker are clipped away by
    // the renderer's bounds over valid points alone.
    double first=time.front(),last=time.back();
    if(first==last) {
        const double margin=std::max(1.,first*.05);
        first=std::max(0.,first-margin);
        last=last>std::numeric_limits<double>::max()-margin?
            std::numeric_limits<double>::max():last+margin;
    }
    plot.x_range=std::pair{first,last};
}
// A rough-terrain ride can toggle contact validity dozens of times. One series
// per valid range would exhaust the 256-series plot bound and the legend rows
// each panel reserves, so a heavily fragmented channel falls back to one gapped
// line plus one points series for its isolated samples.
constexpr std::size_t maximum_segment_series = 6, maximum_gap_markers = 16;
// Each contiguous valid range is its own series. Renderers cannot join two
// sides of an unavailable interval, even when they ignore a NaN convention.
// `on_time_axis` is false for non-temporal abscissae (world X), where a
// vertical N/A marker would sit at a coordinate instead of an interval.
void segments(Plot& plot, const std::vector<double>& x, const std::vector<double>& y,
              const std::vector<double>* mask, const std::string& name, const std::string& color,
              bool on_time_axis = true) {
    if (x.size() != y.size() || (mask && mask->size() != x.size()))
        throw std::invalid_argument("spatial plot: inconsistent series dimensions");
    const auto valid = [&](std::size_t i) {
        return (!mask || (*mask)[i] == 1.) && std::isfinite(x[i]) && std::isfinite(y[i]);
    };
    // Contiguous valid ranges as [first, last) pairs, and the first sample of
    // every unavailable interval.
    std::vector<std::pair<std::size_t, std::size_t>> runs;
    std::vector<std::size_t> gaps;
    for (std::size_t i = 0; i < x.size(); ++i) {
        if (valid(i)) {
            if (!runs.empty() && runs.back().second == i) ++runs.back().second;
            else runs.emplace_back(i, i + 1);
        } else if (i == 0 || valid(i - 1)) gaps.push_back(i);
    }
    const auto append = [&](Series& series, std::size_t first, std::size_t last) {
        for (std::size_t i = first; i < last; ++i) { series.x.push_back(x[i]); series.y.push_back(y[i]); }
    };
    if (runs.size() <= maximum_segment_series) {
        for (const auto& run : runs) {
            Series series{.name=name, .x={}, .y={}, .color=color};
            // A one-sample valid island has no line segment. Explicit points
            // keep it visible in both native HTML and the Plotly backend.
            if (run.second - run.first == 1) series.style = SeriesStyle::points;
            append(series, run.first, run.second);
            plot.series.push_back(std::move(series));
        }
    } else {
        // Non-finite separators break the line in the SVG, native HTML and
        // Plotly renderers alike (they serialize as null).
        constexpr double separator = std::numeric_limits<double>::quiet_NaN();
        Series line{.name=name, .x={}, .y={}, .color=color};
        Series islands{.name=name, .x={}, .y={}, .color=color, .style=SeriesStyle::points};
        for (const auto& run : runs) {
            if (run.second - run.first == 1) { append(islands, run.first, run.second); continue; }
            if (!line.x.empty()) { line.x.push_back(separator); line.y.push_back(separator); }
            append(line, run.first, run.second);
        }
        if (!line.x.empty()) plot.series.push_back(std::move(line));
        if (!islands.x.empty()) plot.series.push_back(std::move(islands));
    }
    if (!on_time_axis) {
        if (!gaps.empty()) plot.notes.push_back("N/A: " + name + " — samples with unavailable coordinates are omitted");
        return;
    }
    for (std::size_t g = 0; g < gaps.size() && g < maximum_gap_markers; ++g)
        if (std::isfinite(x[gaps[g]])) plot.markers.push_back({.kind=MarkerKind::vertical,
            .first=x[gaps[g]], .second=0., .label="N/A: " + name, .color="#8b949e"});
    if (gaps.size() > maximum_gap_markers)
        plot.notes.push_back("N/A: " + name + " — " + std::to_string(gaps.size() - maximum_gap_markers) +
            " further unavailable intervals are not individually marked");
}
Report one(std::string title, Plot plot) {
    Report report{.title=std::move(title), .panels={std::move(plot)}};
    validate(report);
    return report;
}
}

Report spatial_attitude_plot(const Channels& channels) {
    const auto& time = time_axis(channels);
    const auto& valid = required(channels, "spatial.attitude_valid", time.size()); mask_domain(valid);
    const auto& roll = required(channels, "spatial.roll_rad", time.size());
    const auto& steer = required(channels, "spatial.steer_angle_rad", time.size());
    Plot plot{.title="Spatial roll and steering", .x_label="interval start time (s)", .y_label="angle (rad)", .series={}};
    retain_time_extent(plot,time);
    segments(plot, time, roll, &valid, "roll", "#38bdf8");
    segments(plot, time, steer, nullptr, "steer", "#fbbf24");
    return one("Spatial attitude", std::move(plot));
}

Report spatial_contact_plot(const Channels& channels) {
    const auto& time = time_axis(channels);
    Report report{.title="Spatial contact", .panels={
        {.title="Lateral slip", .x_label="interval start time (s)", .y_label="m/s", .series={}},
        {.title="Normal load", .x_label="interval start time (s)", .y_label="N", .series={}}}};
    for(auto& panel:report.panels)retain_time_extent(panel,time);
    for (const auto side : {"front", "rear"}) {
        const std::string prefix = std::string("spatial.") + side;
        const auto& valid = required(channels, prefix + "_contact_valid", time.size()); mask_domain(valid);
        const auto color = side == std::string_view("front") ? "#38bdf8" : "#f97316";
        segments(report.panels[0], time, required(channels,prefix + "_lateral_slip_mps",time.size()), &valid, side, color);
        segments(report.panels[1], time, required(channels,prefix + "_normal_load_n",time.size()), &valid, side, color);
    }
    validate(report); return report;
}

Report spatial_wheel_path_plot(const Channels& channels) {
    const auto& time = time_axis(channels);
    Plot plot{.title="Recorded axle paths — world frame", .x_label="world X (m)", .y_label="world Y (m)", .series={}};
    plot.equal_axes = true;
    plot.notes.emplace_back("Accepted force-stage axle transforms, including steering and suspension motion; world frame.");
    for (const auto side : {"front", "rear"}) {
        const std::string prefix = std::string("spatial.") + side + "_wheel_";
        const auto& x = required(channels, prefix + "x_m", time.size());
        const auto& y = required(channels, prefix + "y_m", time.size());
        segments(plot, x, y, nullptr, std::string(side) + " (world frame)",
                 side == std::string_view("front") ? "#38bdf8" : "#f97316", false);
    }
    return one("Spatial wheel paths", std::move(plot));
}

Report spatial_imu_plot(const Channels& channels) {
    const auto& time = time_axis(channels, "observations.time_s");
    const auto& valid = required(channels, "observations.valid", time.size()); mask_domain(valid);
    const auto& masks = required(channels, "observations.valid_mask", time.size());
    for (const double value : masks) if (!std::isfinite(value) || value < 0. || value > 2047. || std::floor(value) != value)
        throw std::invalid_argument("spatial plot: invalid sensor mask");
    Report report{.title="Spatial IMU", .panels={
        {.title="Specific force — sensor frame", .x_label="delivery time (s)", .y_label="m/s²", .series={}},
        {.title="Angular rate — sensor frame", .x_label="delivery time (s)", .y_label="rad/s", .series={}}}};
    for(auto& panel:report.panels)retain_time_extent(panel,time);
    constexpr std::array names{"X", "Y", "Z"};
    constexpr std::array colors{"#38bdf8", "#f97316", "#34d399"};
    for (std::size_t axis = 0; axis < 6; ++axis) {
        const auto field = std::string(axis < 3 ? "observations.specific_force_sensor_mps2." : "observations.angular_rate_sensor_rad_s.") + std::to_string(axis % 3);
        std::vector<double> mask(time.size());
        for (std::size_t i = 0; i < mask.size(); ++i)
            mask[i] = valid[i] == 1. && (static_cast<std::uint16_t>(masks[i]) & (1U << axis)) != 0 ? 1. : 0.;
        segments(report.panels[axis / 3], time, required(channels,field,time.size()), &mask, names[axis % 3], colors[axis % 3]);
    }
    validate(report); return report;
}

Report spatial_energy_plot(const Channels& channels) {
    const auto& time = time_axis(channels,numeric(channels,"spatial.energy_time_s")?"spatial.energy_time_s":"time_s");
    Plot plot{.title="Spatial work and losses", .x_label="interval end time (s)", .y_label="J", .series={}};
    retain_time_extent(plot,time);
    segments(plot,time,required(channels,"spatial.steer_work_j",time.size()),nullptr,"steering work","#38bdf8");
    for (const auto& [field, name] : {std::pair{"spatial.tyre_loss_j","tyre loss"}, {"spatial.hold_work_j","bootstrap hold work"}}) {
        if (const auto* values = numeric(channels,field)) {
            if (values->size() != time.size()) throw std::invalid_argument("spatial energy: channel width mismatch");
            segments(plot,time,*values,nullptr,name,"#f97316");
        } else plot.notes.push_back(std::string(name) + ": N/A — interval channel was not recorded");
    }
    return one("Spatial energy", std::move(plot));
}
} // namespace bike::viz
