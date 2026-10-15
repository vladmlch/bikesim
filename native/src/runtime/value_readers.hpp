// value_readers.hpp — the bike::io::Value twin of binding_readers.hpp
// (task T09; migration/interface-contracts.md §2).
//
// The bootstrap/config decoders read owned Value documents with the same
// acceptance set and the same "path: reason" diagnostics the nanobind
// readers produced — the function names and signatures mirror wire:: so a
// ported decode body reads as a mechanical substitution. Two source-side
// differences are load-bearing:
//
//   * integer() accepts only integer-spelled Number lexemes — the source
//     JSON (or the Python-materialized document) already lost the
//     int-versus-float distinction anywhere except the lexeme spelling,
//     which is exactly the boundary wire::integer's Integral gate encoded.
//   * real() additionally accepts the resolved {"$nonfinite": ...} form —
//     bootstrap_io materializes it as a Tagged{"float64"} value with the
//     recording codec's sentinel spellings, mirroring wire::real() letting
//     a nonfinite Python float through to the finite-only field readers.
//
// wire_from_value() is the inverse of the binding's python_to_wire(): it
// lowers an owned Value tree into the runtime's insertion-ordered Wire
// tree so the shared Wire-consumers (WorkHistory/ModelStatus/
// ReferenceMonitor::from_wire and the accounting budget reads) run
// unchanged on decoded input.
#pragma once

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "../io/value.hpp"
#include "control.hpp"

namespace vread {

namespace bio = bike::io;

template <typename... Keys>
constexpr auto keys(Keys... names) {
    return std::array<std::string_view, sizeof...(Keys)>{names...};
}

[[noreturn]] inline void invalid(std::string_view path,
                                 std::string_view reason) {
    throw std::invalid_argument(std::string(path) + ": " +
                                std::string(reason));
}

// One resolved Value node — the field readers take this like the wire
// readers take nb::handle.
struct Node {
    const bio::Value *value = nullptr;
};

inline bool is_none(Node node) noexcept {
    return node.value == nullptr ||
           std::holds_alternative<std::monostate>(node.value->data);
}

inline bool boolean(Node node, std::string_view path) {
    if (node.value != nullptr) {
        if (const auto *result = std::get_if<bool>(&node.value->data))
            return *result;
    }
    invalid(path, "expected bool");
}

// The resolved nonfinite sentinel carries the recording codec's "float64"
// kind with value spellings "nan" / "+inf" / "-inf" (bike::io::tagged_double
// accepts the same shape).
inline double tagged_nonfinite(const bio::Tagged &tagged,
                               std::string_view path) {
    if (tagged.kind == "float64" && tagged.fields.size() == 1 &&
        tagged.fields.front().first == "value") {
        const std::string &spelling = tagged.fields.front().second;
        if (spelling == "nan")
            return std::numeric_limits<double>::quiet_NaN();
        if (spelling == "+inf" || spelling == "inf")
            return std::numeric_limits<double>::infinity();
        if (spelling == "-inf")
            return -std::numeric_limits<double>::infinity();
    }
    invalid(path, "expected finite real scalar");
}

inline double real(Node node, std::string_view path) {
    if (node.value != nullptr) {
        if (const auto *number = std::get_if<bio::Number>(&node.value->data)) {
            bool ok = false;
            const double result =
                bio::detail::float_from_lexeme(number->lexeme, ok);
            if (ok) return result;
        }
        if (const auto *tagged = std::get_if<bio::Tagged>(&node.value->data))
            return tagged_nonfinite(*tagged, path);
    }
    invalid(path, "expected finite real scalar");
}

inline double finite_real(Node node, std::string_view path) {
    const double result = real(node, path);
    if (!std::isfinite(result)) invalid(path, "expected finite real scalar");
    return result;
}

inline std::optional<double> optional_real(Node node,
                                           std::string_view path) {
    return is_none(node) ? std::nullopt
                         : std::optional<double>(finite_real(node, path));
}

inline std::int64_t integer(Node node, std::string_view path) {
    if (node.value != nullptr) {
        if (const auto *number = std::get_if<bio::Number>(&node.value->data)) {
            if (!bio::is_integer_lexeme(number->lexeme))
                invalid(path, "expected integer");
            std::int64_t result = 0;
            const char *const last =
                std::to_address(number->lexeme.cend());
            const auto parsed =
                std::from_chars(number->lexeme.data(), last, result, 10);
            if (parsed.ec == std::errc::result_out_of_range ||
                parsed.ec == std::errc::invalid_argument || parsed.ptr != last)
                invalid(path, "invalid value or out of range");
            return result;
        }
    }
    invalid(path, "expected integer");
}

inline int integer32(Node node, std::string_view path) {
    const auto result = integer(node, path);
    if (result < std::numeric_limits<int>::min() ||
        result > std::numeric_limits<int>::max())
        invalid(path, "integer out of range");
    return static_cast<int>(result);
}

inline const std::string &string(Node node, std::string_view path) {
    if (node.value != nullptr) {
        if (const auto *result =
                std::get_if<std::string>(&node.value->data))
            return *result;
    }
    invalid(path, "expected string");
}

// Parser-local ownership of the mapping node and its public path — the
// wire::Dict twin.
struct Dict {
    const bio::Value::Object *value = nullptr;
    std::string path;

    std::string child(std::string_view key) const {
        return path + "." + std::string(key);
    }
    bool contains(std::string_view key) const {
        return bio::find_member(*value, key) != nullptr;
    }
    Node operator[](std::string_view key) const {
        return Node{bio::find_member(*value, key)};
    }
    std::size_t size() const { return value->size(); }
};

inline const bio::Value::Object &mapping(Node node, std::string_view path) {
    if (node.value != nullptr) {
        if (const auto *result =
                std::get_if<bio::Value::Object>(&node.value->data))
            return *result;
    }
    invalid(path, "expected dict");
}

inline const bio::Value::Array &sequence(Node node, std::string_view path) {
    if (node.value != nullptr) {
        if (const auto *result =
                std::get_if<bio::Value::Array>(&node.value->data))
            return *result;
    }
    invalid(path, "expected ordered sequence");
}

inline void exact_keys(const bio::Value::Object &object,
                       std::span<const std::string_view> required,
                       std::span<const std::string_view> optional,
                       std::string_view path) {
    for (const auto &[key, item] : object) {
        if (std::ranges::find(required, key) == required.end() &&
            std::ranges::find(optional, key) == optional.end())
            invalid(std::string(path) + "." + key, "unknown key");
    }
    for (const auto key : required)
        if (bio::find_member(object, key) == nullptr)
            invalid(std::string(path) + "." + std::string(key),
                    "missing required key");
}

inline std::vector<double> vector(Node node, std::string_view path) {
    const auto &values = sequence(node, path);
    std::vector<double> result;
    result.reserve(values.size());
    for (const auto &item : values)
        result.push_back(finite_real(Node{&item}, path));
    return result;
}

template <std::size_t N>
std::array<double, N> fixed(Node node, std::string_view path) {
    const auto &values = sequence(node, path);
    if (values.size() != N) invalid(path, "incorrect sequence width");
    std::array<double, N> result{};
    for (std::size_t i = 0; i < N; ++i)
        result.at(i) = finite_real(Node{&values.at(i)}, path);
    return result;
}

inline Node field(const Dict &d, std::string_view key) {
    const bio::Value *const found = bio::find_member(*d.value, key);
    if (found == nullptr) invalid(d.child(key), "missing required key");
    return Node{found};
}

inline Dict section(const Dict &d, std::string_view key) {
    const Node value = field(d, key);
    return {.value = &mapping(value, d.child(key)), .path = d.child(key)};
}

inline void exact(const Dict &d, std::span<const std::string_view> required,
                  std::span<const std::string_view> optional = {}) {
    exact_keys(*d.value, required, optional, d.path);
}

// ---- Value -> Wire ---------------------------------------------------------
// The runtime keeps its accounting/step-facing plain-data trees as
// insertion-ordered Wire records. A Number lexeme carrying an integer
// spelling becomes int64_t (a float spelling stays double even when its
// value is integral — the same split the nanobind path's
// Integral/Real isinstance checks produced); a resolved nonfinite tag
// becomes the sentinel double, mirroring wire::real() passing NaN through.

// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the plain-data tree
[[nodiscard]] inline runtime::Wire wire_from_value(const bio::Value &value,
                                                   std::string_view path) {
    if (std::holds_alternative<std::monostate>(value.data))
        return runtime::Wire{nullptr};
    if (const auto *boolean = std::get_if<bool>(&value.data))
        return runtime::Wire{*boolean};
    if (const auto *number = std::get_if<bio::Number>(&value.data)) {
        if (bio::is_integer_lexeme(number->lexeme)) {
            std::int64_t result = 0;
            const char *const last =
                std::to_address(number->lexeme.cend());
            const auto parsed =
                std::from_chars(number->lexeme.data(), last, result, 10);
            if (parsed.ec == std::errc::result_out_of_range ||
                parsed.ec == std::errc::invalid_argument || parsed.ptr != last)
                invalid(path, "invalid value or out of range");
            return runtime::Wire{result};
        }
        bool ok = false;
        const double converted =
            bio::detail::float_from_lexeme(number->lexeme, ok);
        if (!ok) invalid(path, "expected finite real scalar");
        return runtime::Wire{converted};
    }
    if (const auto *text = std::get_if<std::string>(&value.data))
        return runtime::Wire{*text};
    if (const auto *items = std::get_if<bio::Value::Array>(&value.data)) {
        runtime::WireArray out;
        out.reserve(items->size());
        for (const auto &item : *items)
            out.push_back(wire_from_value(item, path));
        return runtime::Wire{std::move(out)};
    }
    if (const auto *object = std::get_if<bio::Value::Object>(&value.data)) {
        runtime::WireObject out;
        out.reserve(object->size());
        for (const auto &[key, item] : *object)
            out.emplace_back(key, wire_from_value(item, path));
        return runtime::Wire{std::move(out)};
    }
    if (const auto *tagged = std::get_if<bio::Tagged>(&value.data))
        return runtime::Wire{tagged_nonfinite(*tagged, path)};
    // Bytes and unknown tags cannot spell a plain-data leaf the nanobind
    // path would have carried through.
    invalid(path, "expected plain data (dict/list/tuple/scalar)");
}

} // namespace vread
