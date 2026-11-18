#include "render_owner.hpp"
#include <cmath>
#include <numbers>
#include <span>
#include <stdexcept>

namespace bike::viewer {
Camera::Camera(double smoothing) : smoothing_(smoothing) {
    if (!std::isfinite(smoothing) || smoothing < 0. || smoothing > 1.)
        throw std::invalid_argument("camera smoothing must be in [0, 1]");
}
void Camera::set_mode(CameraMode mode) { mode_ = mode; reset_ = true; }
void Camera::cycle_mode() { set_mode(mode_ == CameraMode::side ? CameraMode::chase : CameraMode::side); }
void Camera::reset_preset() noexcept {
    reset_ = true;
    lookat_.reset();
    last_wrapped_yaw_rad_.reset();
    unwrapped_yaw_rad_ = 0.;
}
double Camera::unwrapped_yaw_rad() const noexcept { return unwrapped_yaw_rad_; }
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
void Camera::update_spatial(mjvCamera& camera, double bike_x, double bike_y, double bike_z, double heading_rad) {
    if (!std::isfinite(bike_x) || !std::isfinite(bike_y) || !std::isfinite(bike_z) ||
        !std::isfinite(heading_rad))
        throw std::invalid_argument("camera target must be finite");
    if (!last_wrapped_yaw_rad_) {
        last_wrapped_yaw_rad_ = std::atan2(std::sin(heading_rad), std::cos(heading_rad));
        unwrapped_yaw_rad_ = *last_wrapped_yaw_rad_;
    } else {
        const double delta = std::atan2(std::sin(heading_rad - *last_wrapped_yaw_rad_),
                                        std::cos(heading_rad - *last_wrapped_yaw_rad_));
        unwrapped_yaw_rad_ += delta;
        last_wrapped_yaw_rad_ = heading_rad;
    }
    const bool side = mode_ == CameraMode::side;
    const double forward_offset = side ? .15 : .20;
    const std::array<double, 3> target{
        bike_x + forward_offset * std::cos(unwrapped_yaw_rad_),
        bike_y + forward_offset * std::sin(unwrapped_yaw_rad_),
        bike_z + (side ? .25 : .30)};
    const double target_azimuth = (side ? 90. : 135.) + unwrapped_yaw_rad_ * (180. / std::numbers::pi);
    const double target_elevation = side ? -5. : -15.;
    const double target_distance = side ? 2.5 : 3.2;
    if (reset_ || !lookat_) {
        lookat_ = target;
        camera.azimuth = target_azimuth;
        camera.elevation = target_elevation;
        camera.distance = target_distance;
        reset_ = false;
    } else {
        (*lookat_)[0] = target[0];
        (*lookat_)[1] = target[1];
        (*lookat_)[2] += smoothing_ * (target[2] - (*lookat_)[2]);
        camera.azimuth += smoothing_ * (target_azimuth - camera.azimuth);
        camera.elevation += smoothing_ * (target_elevation - camera.elevation);
        camera.distance += smoothing_ * (target_distance - camera.distance);
    }
    auto const lookat = std::span(camera.lookat);
    for (std::size_t i = 0; i < lookat.size(); ++i) lookat[i] = (*lookat_)[i];
}
void Camera::update_spatial(mjvCamera& camera, double bike_x, double bike_y, double bike_z,
                            const std::array<double, 2>& heading_xy) {
    if (!std::isfinite(heading_xy[0]) || !std::isfinite(heading_xy[1]))
        throw std::invalid_argument("camera heading must be finite");
    const double norm = std::hypot(heading_xy[0], heading_xy[1]);
    const double heading_rad = norm > 0.
        ? std::atan2(heading_xy[1], heading_xy[0])
        : last_wrapped_yaw_rad_.value_or(0.);
    update_spatial(camera, bike_x, bike_y, bike_z, heading_rad);
}
} // namespace bike::viewer
