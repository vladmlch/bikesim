#include "contact_math.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

extern "C" {
double cblas_ddot(int n, const double* x, int incx, const double* y, int incy);
void cblas_dgemv(int order, int trans, int m, int n, double alpha,
                 const double* a, int lda, const double* x, int incx,
                 double beta, double* y, int incy);
}
namespace rider {
void validate_scalar(double value, const char* name) {
    if (!std::isfinite(value)) throw std::invalid_argument(name);
}
void validate_vector(const Vec3& value, const char* name) {
    for (double x : value) validate_scalar(x, name);
}
double dot(const Vec3& a, const Vec3& b) {
    return cblas_ddot(3, a.data(), 1, b.data(), 1);
}
Vec3 matvec(const Mat3& matrix, const Vec3& vector, bool transpose) {
    Vec3 result{};
    cblas_dgemv(101, transpose ? 112 : 111, 3, 3, 1., matrix.data(), 3,
                vector.data(), 1, 0., result.data(), 1);
    return result;
}
Vec3 add(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
Vec3 subtract(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
Vec3 multiply(const Vec3& v, double factor) { return {v[0]*factor, v[1]*factor, v[2]*factor}; }
namespace {
// CPython 3.14.3 Modules/mathmodule.c vector_norm: exact products and compensated sums,
// then a square-root differential correction. Explicit FMA is confined to
// the error-free product; scalar expression contraction remains disabled.
double vector_norm(Vec3 values, double maximum, bool found_nan) {
    if (std::isinf(maximum)) return maximum;
    if (found_nan) return std::numeric_limits<double>::quiet_NaN();
    if (maximum == 0.) return maximum;
    int exponent = 0;
    (void)std::frexp(maximum, &exponent);
    if (exponent < -1023) {
        constexpr double minimum = std::numeric_limits<double>::min();
        for (double& value : values) value /= minimum;
        return minimum*vector_norm(values, maximum/minimum, found_nan);
    }
    const double scale = std::ldexp(1., -exponent);
    double csum=1., frac1=0., frac2=0.;
    for (double value : values) {
        const double x = value*scale;
        const double product = x*x;
        const double product_error = std::fma(x, x, -product);
        const double sum = csum+product;
        const double sum_error = (csum-sum)+product;
        csum = sum; frac1 += product_error; frac2 += sum_error;
    }
    double h = std::sqrt(csum-1.+(frac1+frac2));
    const double product = -h*h;
    const double product_error = std::fma(-h, h, -product);
    const double sum = csum+product;
    const double sum_error = (csum-sum)+product;
    csum = sum; frac1 += product_error; frac2 += sum_error;
    const double x = csum-1.+(frac1+frac2);
    h += x/(2.*h);
    return h/scale;
}
}  // namespace
double hypot3(const Vec3& values) {
    double maximum = 0.; bool found_nan = false;
    Vec3 absolute{};
    for (std::size_t i=0; i<3; ++i) {
        absolute[i] = std::abs(values[i]);
        found_nan = found_nan || std::isnan(absolute[i]);
        if (absolute[i] > maximum) maximum = absolute[i];
    }
    return vector_norm(absolute, maximum, found_nan);
}
double hypot2(double x, double z) { return hypot3({x, z, 0.}); }
GripStep grip_step(const Vec3& xi, const Vec3& u, double k, double c, double dt) {
    validate_vector(xi, "grip state"); validate_vector(u, "grip velocity");
    validate_scalar(k, "grip stiffness"); validate_scalar(c, "grip damping");
    validate_scalar(dt, "grip dt");
    if (k <= 0. || c < 0. || dt <= 0.) throw std::invalid_argument("invalid grip configuration");
    const Vec3 next = add(xi, multiply(u, dt));
    const Vec3 force = subtract(multiply(next, -k), multiply(u, c));
    const double energy = .5*k*dot(next, next);
    const double loss = .5*k*dt*dt*dot(u, u)+c*dt*dot(u, u);
    validate_vector(next, "grip update overflow"); validate_vector(force, "grip update overflow");
    validate_scalar(energy, "grip update overflow"); validate_scalar(loss, "grip update overflow");
    return {next, force, energy, loss};
}
GripRelease release_if_overloaded(const Vec3& force, double old_energy, double limit) {
    validate_vector(force, "grip force must be a finite 3-vector");
    validate_scalar(old_energy, "invalid stored grip energy");
    validate_scalar(limit, "grip limit must be positive");
    if (old_energy < 0.) throw std::invalid_argument("invalid stored grip energy");
    if (limit <= 0.) throw std::invalid_argument("grip limit must be positive");
    if (std::sqrt(dot(force, force)) > limit) return {Vec3{}, old_energy, true};
    return {force, 0., false};
}
}  // namespace rider
