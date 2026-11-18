#pragma once
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_contact.hpp"

namespace contactlaw {
    // tire.py:26-28 — _finite_result: ArithmeticError in Python; the closest
    // std exception nanobind translates is overflow_error → PyExc_OverflowError,
    // which is ArithmeticError's subclass on the Python side.
    [[noreturn]] inline void finite_fail() {
        throw std::overflow_error(
            "tire calculation exceeds finite floating-point range");
    }

    inline void finite_result(std::initializer_list<double> values) {
        for (const double v: values)
            if (!std::isfinite(v))
                finite_fail();
    }

    // tire.py:113-122 — _normal_contact (inputs pre-validated upstream).
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] inline std::pair<double, double>
    normal_contact(double delta, double delta_dot, double k, double c) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        if (delta <= 0.0)
            return {0.0, 0.0};
        const double spring_force = k * delta;
        const double damping_force = c * delta_dot;
        const double energy = 0.5 * spring_force * delta;
        const double total_force = spring_force + damping_force;
        finite_result({spring_force, damping_force, total_force, energy});
        return {std::max(0.0, total_force), energy};
    }

    // Decision 22 + ban 39 (2026-11-10): the compliant_2d linear
    // material's radial loss is rate-independent hysteresis — the T2
    // experiment law. The loss force is proportional to the elastic force
    // with the sign of the deflection rate, smoothed by tanh over eps only
    // as much as the solver needs; η is the material's loss_factor and ε
    // its hysteresis_rate_eps_mps (both required profile fields).
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] inline std::pair<double, double>
    normal_contact_hysteresis(double delta, double delta_dot, double k,
                              double eta, double eps) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        double force = 0.0;
        double energy = 0.0;
        auto err = bike::physics_core::ErrCode::none;
        const auto status = bike::physics_core::normal_contact_hysteresis<
            double, bike::physics_core::CpuExactToolkit>(
                delta, delta_dot, k, eta, eps, force, energy, err);
        if (status != bike::physics_core::Status::ok)
            finite_fail();
        return {force, energy};
    }

    // tire.py:157-190 — _brush_step (inputs pre-validated upstream).
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] inline std::tuple<double, double, double>
    brush_step(double xi, double u, double v_roll, double Fn, double k,
               double mu, double length, double dt) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        double new_xi = 0.0;
        double force = 0.0;
        double loss = 0.0;
        auto err = bike::physics_core::ErrCode::none;
        const auto status = bike::physics_core::brush_step<
            double, bike::physics_core::CpuExactToolkit>(
                xi, u, v_roll, Fn, k, mu, length, dt,
                new_xi, force, loss, err);
        if (status != bike::physics_core::Status::ok) {
            if (err == bike::physics_core::ErrCode::overflow_passivity)
                throw std::overflow_error(
                    "brush update violates discrete passivity");
            finite_fail();
        }
        return {new_xi, force, loss};
    }
} // namespace contactlaw
