#pragma once
#include <array>

namespace rider {
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major

void validate_vector(const Vec3& value, const char* name);
void validate_scalar(double value, const char* name);
double dot(const Vec3& a, const Vec3& b);
Vec3 matvec(const Mat3& matrix, const Vec3& vector, bool transpose = false);
Vec3 add(const Vec3& a, const Vec3& b);
Vec3 subtract(const Vec3& a, const Vec3& b);
Vec3 multiply(const Vec3& value, double factor);
double hypot3(const Vec3& value);
double hypot2(double x, double z);

struct GripStep { Vec3 xi; Vec3 force; double energy_j; double loss_j; };
struct GripRelease { Vec3 force; double loss_j; bool overloaded; };
GripStep grip_step(const Vec3& xi, const Vec3& relative_velocity,
                   double k, double c, double dt);
GripRelease release_if_overloaded(const Vec3& force_n,
                                  double old_energy_j, double limit_n);
}  // namespace rider
