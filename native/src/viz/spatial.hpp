#pragma once

#include "plot.hpp"

namespace bike::viz {

Report spatial_attitude_plot(const Channels&);
Report spatial_contact_plot(const Channels&);
Report spatial_wheel_path_plot(const Channels&);
Report spatial_imu_plot(const Channels&);
Report spatial_energy_plot(const Channels&);

} // namespace bike::viz
