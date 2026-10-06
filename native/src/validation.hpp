#pragma once
#include <cmath>
#include <optional>
#include <stdexcept>

namespace validation {
    inline double finite(double value, const char *name) {
        if (!std::isfinite(value)) throw std::invalid_argument(name);
        return value;
    }

    inline double nonnegative(double value, const char *name) {
        finite(value, name);
        if (value < 0.) throw std::invalid_argument(name);
        return value;
    }

    inline double positive(double value, const char *name) {
        finite(value, name);
        if (value <= 0.) throw std::invalid_argument(name);
        return value;
    }

    inline void finite_optional(std::optional<double> value, const char *name) {
        if (value) finite(*value, name);
    }
}
