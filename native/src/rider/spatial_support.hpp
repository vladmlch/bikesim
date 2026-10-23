#pragma once

#include "../io/value.hpp"
#include "../spatial/types.hpp"
#include <cstdint>
#include <string>

namespace bike::rider {

struct ShoulderProfile {
    std::uint32_t schema = 1;
    double swivel_range_rad = 0.;
    double swivel_stiffness_nm_per_rad = 0.;
    double swivel_damping_nms_per_rad = 0.;
    double swivel_strength_nm = 0.;
    std::string provenance_status = "provisional";
    std::string sha256_hex;
};
ShoulderProfile read_shoulder_profile(const bike::io::Value&);
struct SpatialSupportFrame {
    bike::spatial::Vec3 normal_world, tangent_x_world, tangent_y_world;
    bool valid = false;
};
SpatialSupportFrame support_frame(const bike::spatial::BodyPose&,
                                  const bike::spatial::Vec3& normal_body) noexcept;
struct GripRigResult {
    double left_gap_m = 0.;
    double right_gap_m = 0.;
    int grip_jacobian_rank = 0;
    double steering_mobility = 0.;
    double internal_power_residual_w = 0.;
};

} // namespace bike::rider
