#include "environment.hpp"
#include "../numeric_sum.hpp"
#include "../writers/pyfloat.hpp"
#include <cmath>
#include <stdexcept>

namespace bike::research {
RiderBehavior::~RiderBehavior() = default;
RiderSignals signals_from_sample(const runtime::SamplePtr& sample) {
    if (!sample) return {};
    using namespace runtime::sample_wire;
    const auto& sensors = object(required(sample->channels, "sensors"));
    const auto& rider = object(required(sample->channels, "rider"));
    const auto& force = array(required(sensors, "frame_specific_force_body_mps2"));
    const auto& gyro = array(required(sensors, "frame_gyro_body_rad_s"));
    if (force.size() != 3 || gyro.size() != 3) throw std::invalid_argument("rider signals require three-axis IMU channels");
    RiderSignals out;
    out.pitch_rate_up_rad_s = -number(gyro[1]);
    for (std::size_t i = 0; i < 3; ++i) out.specific_force_body_mps2[i] = number(force[i]);
    const auto load = [&](std::string_view name) {
        const auto* support = find(rider, name);
        if (!support) return 0.;
        const auto* normal = find(object(*support), "normal_load_n");
        return normal ? number(*normal) : 0.;
    };
    out.saddle_load_n = load("saddle");
    // sum(... .get('normal_load_n', 0.) ...): float dict values plus a
    // float literal default — every item is a Python float.
    numeric::PythonSum pedal_load;
    pedal_load.add(load("front_pedal")); pedal_load.add(load("rear_pedal"));
    out.pedal_load_n = pedal_load.total();
    if (const auto* grip = find(rider, "grip"))
        if (const auto* values = find(object(*grip), "force_on_bike_n")) {
            numeric::PythonSum squares;
            for (const auto& value : array(*values)) { const auto x = number(value); squares.add(x * x); }
            out.bar_load_n = pyfloat::pow(squares.total(), .5);
        }
    return out;
}
} // namespace bike::research
