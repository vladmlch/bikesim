#pragma once
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <tuple>
#include <utility>

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
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror tire.py _normal_contact
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

    // tire.py:157-190 — _brush_step (inputs pre-validated upstream).
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional params mirror tire.py brush step
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    [[nodiscard]] inline std::tuple<double, double, double>
    brush_step(double xi, double u, double v_roll, double Fn, double k,
               double mu, double length, double dt) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const double old_energy = 0.5 * k * xi * xi;
        finite_result({old_energy});
        if (Fn == 0.0 || mu == 0.0)
            return {0.0, 0.0, old_energy};
        const double relaxation = dt * (std::abs(v_roll) / length);
        const double denominator = 1.0 + relaxation;
        const double trial_numerator = xi + dt * u;
        const double friction_limit = mu * Fn;
        finite_result(
            {relaxation, denominator, trial_numerator, friction_limit});
        const double trial = trial_numerator / denominator;
        const double limit = friction_limit / k;
        finite_result({trial, limit});
        const double new_xi = std::max(-limit, std::min(limit, trial));
        const double force = -k * new_xi;
        const double new_energy = 0.5 * k * new_xi * new_xi;
        const double change = new_xi - xi;
        const double implicit_loss = 0.5 * k * change * change;
        const double relaxation_loss = 2.0 * relaxation * new_energy;
        const double projection_loss =
                (k * new_xi) * (denominator * (trial - new_xi));
        const double loss = implicit_loss + relaxation_loss + projection_loss;
        finite_result({force, new_energy, loss});
        if (loss < 0.0)
            throw std::overflow_error(
                "brush update violates discrete passivity");
        return {new_xi, force, loss};
    }
} // namespace contactlaw
