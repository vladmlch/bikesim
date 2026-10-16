// model/checks.hpp — the checks.py validation vocabulary the T15 model
// TUs share. Python signatures take an untyped `value`; the typed C++
// parameters carry the float/int/bool fast paths of checks.py's
// isinstance dispatch, so the only reachable rejection is the domain
// check — message text kept verbatim. Ledger note: focused extra file
// (T15), the three model TUs share these inline helpers; duplicating
// them per-TU would fork the message strings.
#pragma once

#include <cmath>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::model::checks {

// scalar(value, name, minimum=, positive=) — fast-path branch
// (float/int): 'invalid {name}' on non-finite or bound violation.
// No [[nodiscard]]: the oracle calls scalar() for its raise-only domain
// check as often as for the returned value (validate.cpp keeps the same
// convention).
inline double scalar(double value, std::string_view name,
                     std::optional<double> minimum = std::nullopt,
                     bool positive = false) {
    if (!std::isfinite(value) || (minimum && value < *minimum) ||
        (positive && value <= 0.))
        throw std::invalid_argument("invalid " + std::string(name));
    return value;
}

// integer(value, name, minimum=, positive=) — the Integral dispatch is
// satisfied by the int parameter; only '{name}: invalid integer domain'
// can still fire.
inline int integer(int value, std::string_view name,
                   std::optional<int> minimum = std::nullopt,
                   bool positive = false) {
    if ((minimum && value < *minimum) || (positive && value <= 0))
        throw std::invalid_argument(std::string(name) +
                                  ": invalid integer domain");
    return value;
}

// boolean(value, name) — a typed bool always satisfies `type(v) is bool`.
inline bool boolean(bool value, std::string_view) noexcept {
    return value;
}

// derived(value, name) — OverflowError '{name}: non-finite derived
// value' -> std::overflow_error.
inline double derived(double value, std::string_view name) {
    if (!std::isfinite(value))
        throw std::overflow_error(std::string(name) +
                                  ": non-finite derived value");
    return value;
}

// derived_array(value, name) — OverflowError '{name}: non-finite
// derived array' -> std::overflow_error.
inline void derived_array(std::span<const double> values,
                          std::string_view name) {
    for (const double v: values)
        if (!std::isfinite(v))
            throw std::overflow_error(std::string(name) +
                                      ": non-finite derived array");
}

// np.isclose(a, b, rtol, atol) on scalars — numpy's within_tolerance:
// the |a-b| <= atol + rtol*|b| comparison for finite pairs, the x == y
// tie for non-finite inputs (matching infinities close; NaN never does).
[[nodiscard]] inline bool np_isclose_scalar(double a, double b,
                                            double rtol,
                                            double atol) noexcept {
    if (std::isfinite(a) && std::isfinite(b))
        return std::fabs(a - b) <= atol + rtol * std::fabs(b);
    return a == b;
}

// math.isclose(a, b, rel_tol, abs_tol) — CPython's formula:
// abs(a-b) <= max(rel_tol * max(|a|,|b|), abs_tol). rel_tol is
// non-negative by API contract; the oracle passes rel_tol=0.
[[nodiscard]] inline bool math_isclose(double a, double b, double rel_tol,
                                       double abs_tol) noexcept {
    return std::fabs(a - b) <=
           std::fmax(rel_tol * std::fmax(std::fabs(a), std::fabs(b)),
                     abs_tol);
}

} // namespace bike::model::checks
