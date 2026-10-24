#include "spatial_truth.hpp"
#include "setup_values.hpp"

#include <cmath>
#include <stdexcept>

namespace runtime::spatial {
double integrate_abs_slip(double accumulated,double slip,double dt) {
    if(!std::isfinite(accumulated)||accumulated<0.||!std::isfinite(slip)||!std::isfinite(dt)||dt<=0.)
        throw std::invalid_argument("spatial.slip_path: finite nonnegative accumulation and positive dt required");
    const double result=accumulated+std::abs(slip)*dt;
    if(!std::isfinite(result))throw std::invalid_argument("spatial.slip_path: accumulation overflow");
    return result;
}
bike::io::Value spatial_truth_to_wire(const SpatialTruth& truth) {
    using namespace setupvalue;
    const auto angle=[&](double x){return truth.attitude.valid?real(x):Value{};};
    Object out{{"root_position_world_m",reals(truth.root.position_world_m)},
        {"root_quat_world_from_body_wxyz",reals(truth.root.quat_world_from_body_wxyz)},
        {"root_linear_velocity_world_mps",reals(truth.root.linear_velocity_world_mps)},
        {"root_angular_velocity_world_rad_s",reals(truth.root.angular_velocity_world_rad_s)},
        {"attitude_valid",Value{truth.attitude.valid}},{"roll_rad",angle(truth.attitude.roll_rad)},
        {"pitch_rad",angle(truth.attitude.pitch_rad)},{"heading_rad",angle(truth.attitude.yaw_rad)},
        {"steer_angle_rad",real(truth.steer_angle_rad)},{"steer_rate_rad_s",real(truth.steer_rate_rad_s)},
        {"steer_torque_nm",real(truth.steer_torque_nm)},{"steer_work_j",real(truth.steer_work_j)},
        {"lateral_displacement_m",real(truth.lateral_displacement_m)}};
    for(std::size_t i=0;i<2;++i){const std::string prefix=i==0?"front_":"rear_";
        out.emplace_back(prefix+"contact_valid",Value{truth.contact_valid[i]});
        out.emplace_back(prefix+"lateral_slip_mps",truth.contact_valid[i]?real(truth.lateral_slip_mps[i]):Value{});
        out.emplace_back(prefix+"normal_load_n",truth.contact_valid[i]?real(truth.normal_load_n[i]):Value{});
        out.emplace_back(prefix+"support_normal_world",truth.contact_valid[i]?reals(truth.support_normal_world[i]):Value{});
    }
    return Value{std::move(out)};
}
} // namespace runtime::spatial
