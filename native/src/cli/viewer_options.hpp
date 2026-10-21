#pragma once
#include "arguments.hpp"
#include "../viewer/session.hpp"

namespace bike::cli {
// resolve_ride validates the interactive flags before either launch path
// reaches this shared projection. Research owns its duration and pending
// control delivery; the viewer observes that runtime's terminal callback.
[[nodiscard]] inline viewer::ViewerOptions ride_viewer_options(const Arguments& args,
    double timestep_s, viewer::Mode mode = viewer::Mode::physical) {
    viewer::ViewerOptions options;
    options.mode = mode;
    options.timestep_s = timestep_s;
    options.time_scale = static_cast<unsigned>(args.integer("time_scale", 1));
    options.close_on_terminal = args.flag("close_on_finish");
    if (mode == viewer::Mode::research) options.title = "bike-ride research";
    else options.duration_s = args.optional_real("duration");
    return options;
}
} // namespace bike::cli
