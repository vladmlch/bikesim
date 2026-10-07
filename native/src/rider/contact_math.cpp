#include "contact_math.hpp"
#include "../cblas_abi.hpp"
#include "../model_access.hpp"
#include "../numeric_norm.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace rider {
    void validate_scalar(double value, const char *name) {
        if (!std::isfinite(value)) throw std::invalid_argument(name);
    }

    void validate_vector(const Vec3 &value, const char *name) {
        for (double const x: value) validate_scalar(x, name);
    }

    double dot(const Vec3 &a, const Vec3 &b) {
        return blas::ddot(model_access::kXYZ, a.data(), 1, b.data(), 1);
    }

    Vec3 matvec(const Mat3 &matrix, const Vec3 &vector,
                blas::Transpose transpose) {
        Vec3 result{};
        blas::dgemv(blas::Order::row_major, transpose, model_access::kXYZ,
                    model_access::kXYZ, 1., matrix.data(), model_access::kXYZ,
                    vector.data(), 1, 0., result.data(), 1);
        return result;
    }

    Vec3 add(const Vec3 &a, const Vec3 &b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
    Vec3 subtract(const Vec3 &a, const Vec3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    Vec3 multiply(const Vec3 &v, double factor) { return {v[0] * factor, v[1] * factor, v[2] * factor}; }

    double hypot3(const Vec3 &values) {
        // CPython math.hypot — the local vector_norm port moved to
        // numeric::python_vector_norm unchanged (same fabs scan and the
        // same DoubleLength compensated kernel, verified bitwise on the
        // math.hypot oracle corpus).
        return numeric::python_vector_norm(values);
    }

    double hypot2(double x, double z) { return hypot3({x, z, 0.}); }

    GripStep grip_step(const Vec3 &xi, const Vec3 &u, double k, double c, double dt) {
        validate_vector(xi, "grip state");
        validate_vector(u, "grip velocity");
        validate_scalar(k, "grip stiffness");
        validate_scalar(c, "grip damping");
        validate_scalar(dt, "grip dt");
        if (k <= 0. || c < 0. || dt <= 0.) throw std::invalid_argument("invalid grip configuration");
        const Vec3 next = add(xi, multiply(u, dt));
        const Vec3 force = subtract(multiply(next, -k), multiply(u, c));
        const double energy = .5 * k * dot(next, next);
        const double loss = .5 * k * dt * dt * dot(u, u) + c * dt * dot(u, u);
        validate_vector(next, "grip update overflow");
        validate_vector(force, "grip update overflow");
        validate_scalar(energy, "grip update overflow");
        validate_scalar(loss, "grip update overflow");
        return {.xi = next, .force = force, .energy_j = energy, .loss_j = loss};
    }

    GripRelease release_if_overloaded(const Vec3 &force, double old_energy, double limit) {
        validate_vector(force, "grip force must be a finite 3-vector");
        validate_scalar(old_energy, "invalid stored grip energy");
        validate_scalar(limit, "grip limit must be positive");
        if (old_energy < 0.) throw std::invalid_argument("invalid stored grip energy");
        if (limit <= 0.) throw std::invalid_argument("grip limit must be positive");
        if (std::sqrt(dot(force, force)) > limit) return {.force = Vec3{}, .loss_j = old_energy, .overloaded = true};
        return {.force = force, .loss_j = 0., .overloaded = false};
    }
} // namespace rider
