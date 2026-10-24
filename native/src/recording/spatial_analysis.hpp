#pragma once

#include "reader.hpp"
#include "../viz/plot.hpp"

namespace bike::recording {
// Pure reading and geometric projection. No runtime, engine or policy calls.
[[nodiscard]] viz::Channels spatial_report_channels(const Recording&, std::int64_t episode);
}
