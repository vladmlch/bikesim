#include "render_owner.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <iterator>
#include <span>
#include <stdexcept>

namespace bike::viewer {
Camera::Camera(double smoothing) : smoothing_(smoothing) {
    if (!std::isfinite(smoothing) || smoothing < 0. || smoothing > 1.)
        throw std::invalid_argument("camera smoothing must be in [0, 1]");
}
void Camera::set_mode(CameraMode mode) { mode_ = mode; reset_ = true; }
void Camera::cycle_mode() { set_mode(mode_ == CameraMode::side ? CameraMode::chase : CameraMode::side); }
void Camera::reset_preset() noexcept { reset_ = true; lookat_.reset(); prev_heading_.reset(); unwrapped_heading_ = 0.; }
std::string_view Camera::mode_name() const noexcept {
    return mode_ == CameraMode::side ? "2D Side Tracking" : "3D Isometric Chase";
}
void Camera::update(mjvCamera& camera, double bike_x, double bike_z) {
    if (!std::isfinite(bike_x) || !std::isfinite(bike_z))
        throw std::invalid_argument("camera target must be finite");
    const bool side = mode_ == CameraMode::side;
    const std::array<double, 3> target{bike_x + (side ? .15 : .20), 0., bike_z + (side ? .25 : .30)};
    if (reset_ || !lookat_) {
        lookat_ = target;
        camera.azimuth = side ? 90. : 135.;
        camera.elevation = side ? -5. : -15.;
        camera.distance = side ? 2.5 : 3.2;
        reset_ = false;
    } else {
        (*lookat_)[0] = target[0];
        (*lookat_)[1] = target[1];
        (*lookat_)[2] += smoothing_ * (target[2] - (*lookat_)[2]);
    }
    auto const lookat = std::span(camera.lookat);
    for (std::size_t i = 0; i < lookat.size(); ++i) lookat[i] = (*lookat_)[i];
}
void Camera::update_spatial(mjvCamera& camera, const std::array<double, 3>& position,
                           std::optional<double> heading) {
    if (!std::ranges::all_of(position, [](double value) { return std::isfinite(value); }) ||
        (heading && !std::isfinite(*heading)))
        throw std::invalid_argument("spatial camera pose must be finite");
    const bool side = mode_ == CameraMode::side;
    const double h = heading.value_or(0.);
    const std::array<double, 3> target{position[0] + (side ? .15 : heading ? .20 * std::cos(h) : 0.),
        position[1] + (!side && heading ? .20 * std::sin(h) : 0.), position[2] + (side ? .25 : .30)};
    if (heading) {
        if (prev_heading_) unwrapped_heading_ += std::remainder(*heading - *prev_heading_, 2. * std::numbers::pi);
        else unwrapped_heading_ = *heading;
        prev_heading_ = *heading;
    }
    if (reset_ || !lookat_) {
        lookat_ = target;
        camera.azimuth = side ? 90. : 135. + unwrapped_heading_ * (180. / std::numbers::pi);
        camera.elevation = side ? -5. : -15.;
        camera.distance = side ? 2.5 : 3.2;
        reset_ = false;
    } else {
        (*lookat_)[0] = target[0];
        (*lookat_)[1] = target[1];
        (*lookat_)[2] += smoothing_ * (target[2] - (*lookat_)[2]);
        if (!side && heading) camera.azimuth = 135. + unwrapped_heading_ * (180. / std::numbers::pi);
    }
    std::ranges::copy(*lookat_, std::begin(camera.lookat));
}
} // namespace bike::viewer
