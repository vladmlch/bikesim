#include "application.hpp"
#include "../legacy/playground.hpp"
#include "../viewer/session.hpp"

namespace bike::cli {
int playground_command() {
    auto playground = legacy::make_playground();
    viewer::ViewerOptions options;
    options.mode = viewer::Mode::playground; options.title = "bike-playground";
    options.timestep_s = playground->snapshot().timestep_s;
    (void)viewer::run_playground(*playground, options);
    return 0;
}
} // namespace bike::cli
