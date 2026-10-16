// configuration/fields.hpp — the shared typed field declarations for the
// T12 configuration boundary (plan stage "configuration models").
//
// Every record declared in this directory carries an `inline constexpr`
// FieldDecl table in declaration order plus a RecordSchema that names the
// source dataclass and binds the declared-default instance. `schema()`,
// `to_value()` (the dataclasses.asdict/encode_tree equivalent) and the T13
// loader all read these tables — there is no second defaults list to drift.
//
// Layout contract: `offset` is offsetof() into the owning record; every
// record is standard-layout so member reads are plain pointer arithmetic on
// the schema's declared-default instance or on a live config.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <optional>
#include <variant>

#include "../io/value.hpp"

namespace bike::configuration {

struct RecordSchema;

// The C++ member interpretation of one dataclass field. Python `f.type`
// spellings live in `FieldDecl::type` verbatim; this enum is how the codec
// reads `base + offset` for defaults, the `asdict` equivalent and the
// T13 decode surface.
enum class FieldKind : std::uint8_t {
    Real,               // float
    Integer,            // int
    Boolean,            // bool
    Text,               // str
    OptionalReal,       // float | None
    OptionalBoolean,    // bool | None
    OptionalText,       // str | None
    RealTuple,          // tuple[float, ...]          -> std::vector<double>
    IntegerTuple,       // tuple[int, ...]            -> std::vector<int>
    Real2Tuple,         // tuple[float, float]        -> std::array<double,2>
    Real3Tuple,         // tuple[float, float, float] -> std::array<double,3>
    OptionalReal2Tuple, // tuple[float, float] | None -> std::optional<array2>
    OptionalPairTuple,  // tuple[tuple[float,float],...] | None
                        //   -> std::optional<std::vector<std::array<double,2>>>
    UnionMaterial,      // TireSpec | TabulatedTireSpec -> std::variant
    Dataclass,          // nested dataclass (factory or instance default)
    OptionalDataclass,  // DistributedWheelMaterial | None
};

namespace detail {

// Reads the engaged record out of a `std::optional<T>` member, or nullptr.
template <typename T>
const void* optional_record(const void* address) {
    const auto& opt = *static_cast<const std::optional<T>*>(address);
    return opt.has_value() ? static_cast<const void*>(&*opt) : nullptr;
}

// Returns which arm of `std::variant<A, B>` a member currently holds.
template <typename A, typename B>
std::size_t variant_arm(const void* address) {
    return static_cast<const std::variant<A, B>*>(address)->index();
}

// The engaged alternative's address inside a `std::variant<A, B>` member —
// std::get, never pointer arithmetic: the contained object is NOT promised
// to sit at offset 0 of the variant's storage.
template <typename A, typename B>
const void* variant_object(const void* address) {
    const auto& variant = *static_cast<const std::variant<A, B>*>(address);
    return variant.index() == 0
        ? static_cast<const void*>(&std::get<0>(variant))
        : static_cast<const void*>(&std::get<1>(variant));
}

} // namespace detail

// One `dataclasses.field()` row. `required` marks a leaf with no declared
// default (schema emits `default: null`); the member itself is only read
// for defaults when required == false, and always for instance encoding.
struct FieldDecl {
    std::string_view name;
    std::string_view type;      // str(f.type) from the source dataclass
    FieldKind kind;
    std::size_t offset;         // offsetof(Owner, name)
    bool required;              // f.default is MISSING and no factory
    std::string_view factory;   // f.default_factory.__qualname__ or ""
    const RecordSchema* nested;   // dataclass target for Dataclass kinds
                                // (first arm for UnionMaterial)
    const RecordSchema* nested2;  // second arm of UnionMaterial
    // OptionalDataclass: engaged-record access (default/nullptr meaning the
    // member is read as a plain nested record).
    const void* (*optional_get)(const void*) = nullptr;
    // UnionMaterial: which arm is engaged, and its address inside the
    // variant.
    std::size_t (*variant_index)(const void*) = nullptr;
    const void* (*variant_object)(const void*) = nullptr;
};

// A dataclass declaration: source path, ordered fields and the address of
// the record's declared-default instance (`T{}` spelled with the Python
// declared defaults; required members hold sentinels and are never read).
struct RecordSchema {
    std::string_view path;  // "bike_sim.physics.<module>.<Class>"
    std::span<const FieldDecl> fields;
    const void* declared_default;
};

namespace detail {

// CPython repr() of a binary64: shortest round-trip digits, fixed notation
// when the decimal point position lies in (-4, 16], else `e±NN` scientific.
// Integral values always carry a trailing ".0" — "7000.0", never "7000".
[[nodiscard]] std::string repr_double(double value);

// Member encoders over `base + field.offset` (the field tables are the
// single declaration for defaults AND live values).
[[nodiscard]] bike::io::Value member_value(const FieldDecl& field,
                                           const void* base);
[[nodiscard]] bike::io::Value::Array describe(const RecordSchema& record);
[[nodiscard]] bike::io::Value as_object(const RecordSchema& record,
                                        const void* base);

} // namespace detail
} // namespace bike::configuration
