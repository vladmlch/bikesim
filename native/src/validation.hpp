#pragma once
#include <cmath>
#include <optional>
#include <stdexcept>
#include <span>
#include <string>
#include <string_view>

namespace validation {
    inline double finite(double value, std::string_view name) {
        if (!std::isfinite(value)) throw std::invalid_argument(std::string(name));
        return value;
    }

    inline double nonnegative(double value, std::string_view name) {
        finite(value, name);
        if (value < 0.) throw std::invalid_argument(std::string(name));
        return value;
    }

    inline double positive(double value, std::string_view name) {
        finite(value, name);
        if (value <= 0.) throw std::invalid_argument(std::string(name));
        return value;
    }

    inline void finite_optional(std::optional<double> value, const char *name) {
        if (value) finite(*value, name);
    }

    inline double derived(double value, const char *name) {
        if (!std::isfinite(value)) throw std::overflow_error(name);
        return value;
    }

    inline void derived_array(std::span<const double> values, const char *name) {
        for (const double value: values) derived(value, name);
    }
}
