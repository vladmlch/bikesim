#pragma once
#include "validation.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace wire {
    namespace nb = nanobind;

    template <typename... Keys>
    constexpr auto keys(Keys... names) {
        return std::array<std::string_view, sizeof...(Keys)>{names...};
    }

    [[noreturn]] inline void invalid(std::string_view path, std::string_view reason) {
        throw std::invalid_argument(std::string(path) + ": " + std::string(reason));
    }

    [[noreturn]] inline void conversion_error(std::string_view path) {
        if (PyErr_ExceptionMatches(PyExc_MemoryError)) throw nb::python_error();
        PyErr_Clear();
        invalid(path, "invalid value or out of range");
    }

    inline bool boolean(nb::handle value, std::string_view path) {
        if (value.ptr() == Py_True) return true;
        if (value.ptr() == Py_False) return false;
        invalid(path, "expected bool");
    }

    inline bool numeric_boolean(nb::handle value) {
        return nb::isinstance<nb::bool_>(value) ||
            nb::isinstance(value, nb::module_::import_("numpy").attr("bool_"));
    }

    inline double real(nb::handle value, std::string_view path) {
        if (numeric_boolean(value) ||
            !nb::isinstance(value, nb::module_::import_("numbers").attr("Real")))
            invalid(path, "expected finite real scalar");
        const double result = PyFloat_AsDouble(value.ptr());
        if (PyErr_Occurred()) conversion_error(path);
        return result;
    }

    inline double finite_real(nb::handle value, std::string_view path) {
        const double result = real(value, path);
        if (!std::isfinite(result)) invalid(path, "expected finite real scalar");
        return result;
    }

    inline std::optional<double> optional_real(nb::handle value, std::string_view path) {
        return value.is_none() ? std::nullopt : std::optional(finite_real(value, path));
    }

    inline std::int64_t integer(nb::handle value, std::string_view path) {
        if (numeric_boolean(value) ||
            !nb::isinstance(value, nb::module_::import_("numbers").attr("Integral")))
            invalid(path, "expected integer");
        nb::object const indexed = nb::steal<nb::object>(PyNumber_Index(value.ptr()));
        if (!indexed.is_valid()) conversion_error(path);
        const auto result = PyLong_AsLongLong(indexed.ptr());
        if (PyErr_Occurred()) conversion_error(path);
        return result;
    }

    inline int integer32(nb::handle value, std::string_view path) {
        const auto result = integer(value, path);
        if (result < std::numeric_limits<int>::min() || result > std::numeric_limits<int>::max())
            invalid(path, "integer out of range");
        return static_cast<int>(result);
    }

    inline std::string string(nb::handle value, std::string_view path) {
        if (!nb::isinstance<nb::str>(value)) invalid(path, "expected string");
        // nanobind's string caster clears Python encoding errors and allocates
        // inside noexcept. Preserve Python/C++ allocation errors at this boundary.
        Py_ssize_t size{};
        const char *const text = PyUnicode_AsUTF8AndSize(value.ptr(), &size);
        if (!text) {
            if (PyErr_ExceptionMatches(PyExc_MemoryError)) throw nb::python_error();
            PyErr_Clear();
            invalid(path, "expected UTF-8 string");
        }
        return {text, static_cast<std::size_t>(size)};
    }

    inline void exact_keys(const nb::dict &d, std::span<const std::string_view> required,
                           std::span<const std::string_view> optional, std::string_view path) {
        for (auto const item: d) {
            const auto key = string(item.first, path);
            if (std::ranges::find(required, key) == required.end() &&
                std::ranges::find(optional, key) == optional.end())
                invalid(std::string(path) + "." + key, "unknown key");
        }
        for (const auto key: required)
            if (!d.contains(nb::str(key.data(), key.size())))
                invalid(std::string(path) + "." + std::string(key), "missing required key");
    }

    inline nb::dict mapping(nb::handle value, std::string_view path) {
        if (!nb::isinstance<nb::dict>(value)) invalid(path, "expected dict");
        return nb::borrow<nb::dict>(value);
    }

    inline nb::list sequence(nb::handle value, std::string_view path) {
        const auto abc = nb::module_::import_("collections.abc");
        if (nb::isinstance(value, abc.attr("Mapping")) || nb::isinstance<nb::str>(value) ||
            nb::isinstance<nb::bytes>(value) ||
            (!nb::isinstance(value, abc.attr("Sequence")) &&
             !nb::isinstance(value, nb::module_::import_("numpy").attr("ndarray"))))
            invalid(path, "expected ordered sequence");
        try { return nb::list(value); }
        catch (const nb::python_error &error) {
            if (error.matches(PyExc_MemoryError)) throw;
            invalid(path, "invalid ordered sequence");
        }
    }

    inline void numeric_array_type(nb::handle value, std::string_view path) {
        if (nb::isinstance(value, nb::module_::import_("numpy").attr("ndarray")) &&
            string(value.attr("dtype").attr("kind"), path) == "b")
            invalid(path, "boolean arrays are not numeric inputs");
    }

    inline std::vector<double> vector(nb::handle value, std::string_view path) {
        numeric_array_type(value, path);
        const auto values = sequence(value, path);
        std::vector<double> result;
        result.reserve(values.size());
        for (nb::handle const item: values) result.push_back(finite_real(item, path));
        return result;
    }

    template <std::size_t N>
    std::array<double, N> fixed(nb::handle value, std::string_view path) {
        numeric_array_type(value, path);
        const auto values = sequence(value, path);
        if (values.size() != N) invalid(path, "incorrect sequence width");
        std::array<double, N> result{};
        for (std::size_t i = 0; i < N; ++i) result.at(i) = finite_real(values[i], path);
        return result;
    }

    // Parser-local ownership of both the Python mapping and its public path.
    struct Dict {
        nb::dict value;
        std::string path;

        std::string child(const char *key) const { return path + "." + key; }
        bool contains(const char *key) const { return value.contains(key); }
        nb::object operator[](const char *key) const { return value[key]; }
        std::size_t size() const { return value.size(); }
    };

    inline nb::object field(const Dict &d, const char *key) {
        if (!d.contains(key)) invalid(d.child(key), "missing required key");
        return d[key];
    }

    inline Dict section(const Dict &d, const char *key) {
        const auto value = field(d, key);
        if (!nb::isinstance<nb::dict>(value)) invalid(d.child(key), "expected dict");
        return {.value = nb::borrow<nb::dict>(value), .path = d.child(key)};
    }

    inline void exact(const Dict &d, std::span<const std::string_view> required,
                      std::span<const std::string_view> optional = {}) {
        exact_keys(d.value, required, optional, d.path);
    }
}
