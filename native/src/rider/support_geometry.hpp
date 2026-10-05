#pragma once
#include "contact_math.hpp"
#include <mujoco/mujoco.h>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace rider {
struct BoxPadContact {
    Vec3 point_m;
    Vec3 normal;
    double gap_m;
    bool within_width;
    bool within_footprint;
    Vec3 tangent() const { return {normal[2], 0., -normal[0]}; }
};
struct BoxFace { Vec3 point_m; Vec3 normal; Vec3 tangent; };
struct SoleGoalDiagnostic {
    double requested_compression_m, applied_compression_m;
    double requested_shear_m, applied_shear_m;
    bool saturated;
    std::vector<std::string> limiting_reasons;
};
struct SoleGoal { Vec3 goal; SoleGoalDiagnostic diagnostic; };
class UnreachableSoleTarget : public std::invalid_argument {
public: using std::invalid_argument::invalid_argument;
};
void validate_box(const Vec3& origin, const Mat3& rotation, const Vec3& half);
BoxPadContact box_pad_contact(const Vec3& center, double radius,
                              const Vec3& origin, const Mat3& rotation, const Vec3& half);
// Requires previously validated finite center/radius and proper planar box.
// Compiled geometry callers preserve those setup invariants.
BoxPadContact box_pad_contact_unchecked(const Vec3& center, double radius,
                                        const Vec3& origin, const Mat3& rotation, const Vec3& half);
BoxFace upper_box_face(const Vec3& origin, const Mat3& rotation, const Vec3& half);
double sole_target_height(const Vec3& origin, const Mat3& rotation, const Vec3& half,
                           double sole_x_m, double pad_half_length_m, double pad_radius_m,
                           double compression_m);
SoleGoal project_sole_goal(const Vec3& origin, const Mat3& rotation, const Vec3& half,
                           double pad_half_length_m, double pad_radius_m,
                           double compression_m, double shear_m);
void validate_planar_support_model(const mjModel* model, const mjData* data,
                                   std::span<const int> geom_ids);
}  // namespace rider
