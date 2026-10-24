#include "spatial.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

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
// Each contiguous valid range is its own series. Renderers cannot join two
// sides of an unavailable interval, even when they ignore a NaN convention.
void segments(Plot& plot, const std::vector<double>& x, const std::vector<double>& y,
              const std::vector<double>* mask, std::string name, std::string color) {
    if (x.size() != y.size() || (mask && mask->size() != x.size()))
        throw std::invalid_argument("spatial plot: inconsistent series dimensions");
    Series current{.name=name, .x={}, .y={}, .color=color};
    bool gap = false;
    const auto finish = [&] {
        if (!current.x.empty()) {
            plot.series.push_back(std::move(current));
            current = Series{.name=name, .x={}, .y={}, .color=color};
        }
    };
    for (std::size_t i = 0; i < x.size(); ++i) {
        const bool valid = (!mask || (*mask)[i] == 1.) && std::isfinite(x[i]) && std::isfinite(y[i]);
        if (!valid) {
            finish();
            if (!gap && std::isfinite(x[i])) plot.markers.push_back({.kind=MarkerKind::vertical,
                .first=x[i], .second=0., .label="N/A: " + name, .color="#8b949e"});
            gap = true;
        } else { current.x.push_back(x[i]); current.y.push_back(y[i]); gap = false; }
    }
    finish();
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
    plot.notes.push_back("Accepted force-stage axle transforms, including steering and suspension motion; world frame.");
    for (const auto side : {"front", "rear"}) {
        const std::string prefix = std::string("spatial.") + side + "_wheel_";
        const auto& x = required(channels, prefix + "x_m", time.size());
        const auto& y = required(channels, prefix + "y_m", time.size());
        segments(plot, x, y, nullptr, std::string(side) + " (world frame)", side == std::string_view("front") ? "#38bdf8" : "#f97316");
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
