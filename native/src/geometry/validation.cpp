// geometry/validation.cpp — validation.py's domain validator, check
// order and message text verbatim (the Python ValueError maps to
// std::invalid_argument, the migration harness's strict-invalid
// contract).
#include "specs.hpp"

#include <stdexcept>
#include <string>

namespace bike::geometry {

bool validate_geometry(const BikeSpecs &specs) {
    if (specs.reach <= 0.0 || specs.stack <= 0.0)
        throw std::invalid_argument(
                "Reach and Stack must be strictly positive.");
    if (!(50.0 <= specs.head_angle_deg && specs.head_angle_deg <= 80.0))
        throw std::invalid_argument(
                "Unphysical head angle: " +
                detail::py_repr(specs.head_angle_deg) + " deg");
    if (specs.wheelbase <= specs.reach)
        throw std::invalid_argument(
                "Wheelbase must be greater than reach.");
    if (specs.front_wheel_radius <= 0.0 ||
        specs.rear_wheel_radius <= 0.0)
        throw std::invalid_argument("Wheel radii must be positive.");
    return true;
}

} // namespace bike::geometry
