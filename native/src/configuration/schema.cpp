// configuration/schema.cpp — the T12 introspection/encoding boundary.
//
// Two surfaces, both driven by the FieldDecl tables (the single shared
// declaration — plan step 4.4):
//
//   * schema()/describe()/member_value()/as_object()/to_value(): the
//     capture_reference.py dataclass-introspection document and the
//     dataclasses.asdict + encode_tree codec behind the resolved fixtures
//     ({$tuple: ...} tags on tuple members, plain objects for nested
//     dataclasses, {"$dataclass": path, "fields": {...}} instance defaults,
//     Python repr() number spellings, explicit null optionals).
//   * wire:: — native/schema.py (F049): the resolved-wire validator
//     (SCHEMAS/exact_keys/integer/boolean/sequence/_schema/
//     validate_mapping/validate_policy_modes/validate_drive_policies/
//     validate_rider_contacts/_check_*/validate_config) and plain(), the
//     post-validation coercion to canonical leaf values. All messages and
//     orderings are carried verbatim.
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "configuration/suspension_spec.hpp"
#include "configuration/wire_decode.hpp"
#include "config_validation.hpp"
#include "io/json.hpp"
#include "io/float_format.hpp"
#include "io/value.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace bike::configuration {
SpatialTyreCoefficientProfile spatial_tyre_coefficient_profile(std::string_view id) {
    if(id.empty())id="mtb-provisional-v1";
    if(id!="mtb-provisional-v1" && id!="mtb-magic-mary-29x2.4-front" &&
       id!="mtb-hans-dampf-27.5x2.4-rear")
        throw std::invalid_argument("tires.pneumatic_3d.lateral_coefficients: unknown profile '"+std::string(id)+"'");
    SpatialTyreCoefficientProfile out;out.id=id;return out;
}
bike::io::Value spatial_tyre_profile_registry() {
    using V=bike::io::Value;
    V::Array profiles;
    for(const auto id:{"mtb-magic-mary-29x2.4-front","mtb-hans-dampf-27.5x2.4-rear"}) {
        const auto p=spatial_tyre_coefficient_profile(id);
        profiles.push_back(V{V::Object{{"id",V{p.id}},{"status",V{p.status}},
            {"source_measurements",V{bike::io::Number{"0"}}},{"digest",V{}},
            {"measurement_domain",V{}},{"verification",V{std::string("pending V06")}}}});
    }
    return V{V::Object{{"schema",V{bike::io::Number{"1"}}},{"profiles",V{std::move(profiles)}}}};
}
namespace detail {

// ===========================================================================
// repr_double — CPython repr() of a binary64 (shortest round-trip digits;
// fixed notation when the decimal exponent lands in (-4, 16), e-notation
// otherwise; integral values always spell with a trailing '.0').
// ===========================================================================
std::string repr_double(double value) {
    return io::repr_binary64(value);
}

namespace {

using bike::io::Number;
using bike::io::Value;

Value make_number(std::string lexeme) {
    Value value;
    value.data = Number{std::move(lexeme)};
    return value;
}
Value make_text(std::string text) { Value v; v.data = std::move(text); return v; }
Value make_bool(bool flag) { Value v; v.data = flag; return v; }
Value make_null() { return Value{}; }
Value make_array(Value::Array items) {
    Value v; v.data = std::move(items); return v;
}
Value make_object(Value::Object members) {
    Value v; v.data = std::move(members); return v;
}

// encode_tree's nonfinite leaf: {"$nonfinite": "nan"|"inf"|"-inf"}.
Value encode_nonfinite(double value) {
    const char *spelling = std::isnan(value) ? "nan"
        : value > 0. ? "inf" : "-inf";
    return make_object({{"$nonfinite", make_text(spelling)}});
}
Value encode_real(double value) {
    if (!std::isfinite(value)) return encode_nonfinite(value);
    return make_number(repr_double(value));
}
Value encode_integer(long long value) {
    return make_number(std::to_string(value));
}
// tuple[...] leaf: {"$tuple": [...]}.
Value encode_tuple(Value::Array items) {
    return make_object({{"$tuple", make_array(std::move(items))}});
}

const void *member_address(const FieldDecl &field, const void *base) {
    // offsetof() member access over the type-erased record base. The offset
    // is a compile-time offsetof constant bounded by the owning record's
    // size — the uintptr round-trip keeps the arithmetic out of the
    // pointer domain (-Wunsafe-buffer-usage) without losing the bound.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr) see above
    return reinterpret_cast<const void *>(
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) see above
        reinterpret_cast<std::uintptr_t>(base) + field.offset);
}

} // namespace

// ---- member_value: asdict/encode_tree member read over base+offset ---------
// NOLINTNEXTLINE(misc-no-recursion) nested-record descent mirrors dataclasses.asdict recursion
Value member_value(const FieldDecl &field, const void *base) {
    const void *at = member_address(field, base);
    switch (field.kind) {
    case FieldKind::Real:
        return encode_real(*static_cast<const double *>(at));
    case FieldKind::Integer:
        return encode_integer(*static_cast<const int *>(at));
    case FieldKind::Boolean:
        return make_bool(*static_cast<const bool *>(at));
    case FieldKind::Text:
        return make_text(*static_cast<const std::string *>(at));
    case FieldKind::OptionalReal: {
        const auto &opt = *static_cast<const std::optional<double> *>(at);
        return opt ? encode_real(*opt) : make_null();
    }
    case FieldKind::OptionalBoolean: {
        const auto &opt = *static_cast<const std::optional<bool> *>(at);
        return opt ? make_bool(*opt) : make_null();
    }
    case FieldKind::OptionalText: {
        const auto &opt =
            *static_cast<const std::optional<std::string> *>(at);
        return opt ? make_text(*opt) : make_null();
    }
    case FieldKind::RealTuple: {
        const auto &items = *static_cast<const std::vector<double> *>(at);
        Value::Array encoded;
        for (const double v : items) encoded.push_back(encode_real(v));
        return encode_tuple(std::move(encoded));
    }
    case FieldKind::IntegerTuple: {
        const auto &items = *static_cast<const std::vector<int> *>(at);
        Value::Array encoded;
        for (const int v : items) encoded.push_back(encode_integer(v));
        return encode_tuple(std::move(encoded));
    }
    case FieldKind::Real2Tuple: {
        const auto &items = *static_cast<const std::array<double, 2> *>(at);
        return encode_tuple({encode_real(items[0]), encode_real(items[1])});
    }
    case FieldKind::Real3Tuple: {
        const auto &items = *static_cast<const std::array<double, 3> *>(at);
        return encode_tuple({encode_real(items[0]), encode_real(items[1]),
                             encode_real(items[2])});
    }
    case FieldKind::OptionalReal2Tuple: {
        const auto &opt =
            *static_cast<const std::optional<std::array<double, 2>> *>(at);
        if (!opt) return make_null();
        return encode_tuple({encode_real((*opt)[0]), encode_real((*opt)[1])});
    }
    case FieldKind::OptionalPairTuple: {
        const auto &opt = *static_cast<
            const std::optional<std::vector<std::array<double, 2>>> *>(at);
        if (!opt) return make_null();
        Value::Array rows;
        for (const auto &row : *opt)
            rows.push_back(
                encode_tuple({encode_real(row[0]), encode_real(row[1])}));
        return encode_tuple(std::move(rows));
    }
    case FieldKind::UnionMaterial: {
        const std::size_t arm = field.variant_index(at);
        const RecordSchema &schema =
            *(arm == 0 ? field.nested : field.nested2);
        return as_object(schema, field.variant_object(at));
    }
    case FieldKind::Dataclass:
        return as_object(*field.nested, at);
    case FieldKind::OptionalDataclass: {
        const void *engaged = field.optional_get(at);
        if (engaged == nullptr) return make_null();
        return as_object(*field.nested, engaged);
    }
    }
    return make_null();
}

// ---- as_object: dataclasses.asdict(record) through encode_tree -------------
// NOLINTNEXTLINE(misc-no-recursion) nested-record descent mirrors dataclasses.asdict recursion
Value as_object(const RecordSchema &record, const void *base) {
    Value::Object members;
    members.reserve(record.fields.size());
    for (const FieldDecl &field : record.fields)
        members.emplace_back(std::string(field.name),
                             member_value(field, base));
    return make_object(std::move(members));
}

// ---- describe: the capture_reference.py _fields() introspection ------------
// NOLINTNEXTLINE(misc-no-recursion) nested-record descent mirrors dataclasses.asdict recursion
Value::Array describe(const RecordSchema &record) {
    Value::Array rows;
    rows.reserve(record.fields.size());
    const void *base = record.declared_default;
    for (const FieldDecl &field : record.fields) {
        Value::Object entry;
        // entry insertion order is irrelevant — canonical_json sorts keys.
        entry.emplace_back("name", make_text(std::string(field.name)));
        entry.emplace_back("type", make_text(std::string(field.type)));
        if (field.required || !field.factory.empty()) {
            entry.emplace_back("default", make_null());
        } else if (field.kind == FieldKind::Dataclass &&
                   field.nested != nullptr) {
            // field(default=<instance>): {"$dataclass": path, "fields": asdict}
            entry.emplace_back(
                "default",
                make_object(
                    {{"$dataclass",
                      make_text(std::string(field.nested->path))},
                     {"fields",
                      as_object(*field.nested,
                                member_address(field, base))}}));
        } else {
            entry.emplace_back("default", member_value(field, base));
        }
        entry.emplace_back("default_factory",
                           field.factory.empty()
                               ? make_null()
                               : make_text(std::string(field.factory)));
        // dataclasses.is_dataclass(f.type): only the plain dataclass kind
        // carries a nested "fields" list — unions and Optional records do
        // not (verified against config/schema.json).
        if (field.kind == FieldKind::Dataclass && field.nested != nullptr)
            entry.emplace_back("fields", make_array(describe(*field.nested)));
        rows.push_back(make_object(std::move(entry)));
    }
    return rows;
}

} // namespace detail

// ===========================================================================
// Public boundary
// ===========================================================================
bike::io::Value schema() {
    return bike::io::Value{.data = bike::io::Value::Object{
        {"config_class",
         bike::io::Value{.data = std::string("SimulationPhysicsConfig")}},
        {"fields",
         bike::io::Value{.data = detail::describe(
                   detail::kSimulationPhysicsConfigSchema)}},
    }};
}

bike::io::Value to_value(const SimulationPhysicsConfig &config) {
    using bike::io::Value;
    Value result = detail::as_object(detail::kSimulationPhysicsConfigSchema,
                                     &config);
    if (!config.spatial ||
        config.spatial->mode == bike::spatial::Mode::planar)
        return result;

    // The runtime wire is the single fully resolved spatial representation.
    // Reproject its sections to public input spelling for resolve_physics,
    // metadata and TOML round trips without extending the planar field table.
    const Value encoded = bike::spatial::spatial_config_to_wire(*config.spatial);
    const auto& spatial = io::as_object(encoded, "config.spatial");
    const auto member = [&](std::string_view name) -> const Value& {
        const Value* value = io::find_member(spatial, name);
        if (value == nullptr)
            throw std::logic_error("config.spatial: missing resolved field " +
                                   std::string(name));
        return *value;
    };
    const auto& profile = io::as_object(member("profile"),
                                       "config.spatial.profile");
    const auto profile_member = [&](std::string_view name) -> const Value& {
        const Value* value = io::find_member(profile, name);
        if (value == nullptr)
            throw std::logic_error(
                "config.spatial.profile: missing resolved field " +
                std::string(name));
        return *value;
    };
    // Optional source fields use omission; runtime wire retains exact keys.
    const auto source_section = [](const Value& value,
                                   std::string_view derived_key = {}) {
        Value::Object members;
        for (const auto& [key, field] : io::as_object(value, "spatial section")) {
            if (key == derived_key) continue;
            if (io::is_null(field)) continue;
            const auto* text = std::get_if<std::string>(&field.data);
            if (text != nullptr && text->empty() &&
                (key == "profile_id" || key == "profile_sha256" ||
                 key == "calibration_revision"))
                continue;
            members.emplace_back(key, field);
        }
        return Value{.data = std::move(members)};
    };
    Value::Object source{
        {"root", member("root")},
        {"corridor_half_width_m", profile_member("corridor_half_width_m")},
        {"capsize_roll_rad", profile_member("capsize_roll_rad")},
        {"steer", source_section(member("steer"))},
        {"bootstrap", member("bootstrap")},
        {"imu", source_section(member("imu"))},
        // Public input derives this equality-constrained field from the root.
        // Keeping it only on the resolved wire lets a root-only override
        // update the corridor without inheriting a contradictory duplicate.
        {"terrain", source_section(member("terrain"), "corridor_half_width_m")},
    };
    Value::Object tyre{
        {"tier", profile_member("tyre_tier")},
    };
    for (const auto& [wire_key, input_key] :
         {std::pair{"lateral_coefficients_id", "lateral_coefficients"},
          std::pair{"lateral_coefficients_sha256", "lateral_coefficients_sha256"}}) {
        const auto& value = profile_member(wire_key);
        if (!io::as_string(value, wire_key).empty())
            tyre.emplace_back(input_key, value);
    }
    auto& root = std::get<Value::Object>(result.data);
    for (auto& [key, value] : root) {
        if (key != "articulated") continue;
        auto& rider = std::get<Value::Object>(value.data);
        if (config.articulated.shoulder_profile)
            rider.emplace_back("shoulder_profile", Value{*config.articulated.shoulder_profile});
        if (config.articulated.shoulder_profile_sha256)
            rider.emplace_back("shoulder_profile_sha256", Value{*config.articulated.shoulder_profile_sha256});
    }
    for (auto& [key, value] : root)
        if (key == "tires")
            std::get<Value::Object>(value.data).emplace_back(
                "pneumatic_3d", Value{.data = tyre});
    root.emplace_back("spatial", Value{.data = std::string("3d")});
    root.emplace_back("spatial_3d", Value{.data = std::move(source)});
    return result;
}

// ===========================================================================
// native/schema.py — the resolved-wire validator + plain() coercion
// ===========================================================================
namespace wire {
namespace {

using bike::io::Number;
using bike::io::Value;

[[noreturn]] void fail(std::string_view message) {
    throw std::invalid_argument(std::string(message));
}

// ---- checks.py scalar() on decoded wire members -----------------------------
// checks.py scalar(): the type gate is isinstance(Real) with an explicit
// bool/np.bool_ rejection BEFORE float() coercion — JSON booleans and
// strings are NOT scalars on this wire ('<path> must be a finite real
// scalar'); only a decoded Number is. Bounds/nonfinite failures are
// 'invalid {path}'. py_float's str->float() stays on the typed-loader
// side, which mirrors float() rather than scalar().
double wire_scalar(const Value &value, const std::string &path,
                   std::optional<double> minimum = std::nullopt,
                   bool positive = false) {
    using io::Value;
    if (!std::holds_alternative<Number>(value.data))
        fail(path + " must be a finite real scalar");
    const Number &number = std::get<Number>(value.data);
    bool ok = false;
    const double result =
        bike::io::detail::float_from_lexeme(number.lexeme, ok);
    if (!ok) fail(path + " must be a finite real scalar");
    if (!std::isfinite(result) || (minimum && result < *minimum) ||
        (positive && result <= 0.))
        fail("invalid " + path);
    return result;
}

// schema.py integer(): JSON bools and non-integer lexemes are rejected;
// Integral values carry the int32 domain.
int wire_integer(const Value &value, const std::string &path) {
    if (!std::holds_alternative<Number>(value.data))
        fail(path + ": expected integer");
    const Number &number = std::get<Number>(value.data);
    if (!bike::io::is_integer_lexeme(number.lexeme))
        fail(path + ": expected integer");
    bool ok = false;
    const double parsed =
        bike::io::detail::float_from_lexeme(number.lexeme, ok);
    if (!ok || !std::isfinite(parsed) || parsed < -2147483648. ||
        parsed >= 2147483648.)
        fail(path + ": integer out of range");
    return static_cast<int>(parsed);
}

// schema.py boolean(): exact bool only.
bool wire_boolean(const Value &value, const std::string &path) {
    if (!std::holds_alternative<bool>(value.data))
        fail(path + ": expected bool");
    return std::get<bool>(value.data);
}

// ---- Python value algebra over decoded wire members -------------------------
// scalar() admits parseable strings and bools, so the _check_* gates below
// compare/decree on the raw decoded values exactly like Python does:
//   * str vs str is a lexicographic (codepoint — equal to UTF-8 byte) order;
//   * str vs number raises TypeError, which propagates out of validate_config
//     uncaught (the port surfaces it as invalid_argument carrying the
//     TypeError text);
//   * equality across unlike types is simply False;
//   * bool is numeric (True == 1).
std::optional<double> py_numeric(const Value &value) {
    if (const auto *number =
            std::get_if<Number>(&value.data)) {
        bool ok = false;
        const double parsed =
            bike::io::detail::float_from_lexeme(number->lexeme, ok);
        if (ok) return parsed;
        return std::nullopt;  // unparsed lexeme — falls to TypeError arm
    }
    if (const auto *flag = std::get_if<bool>(&value.data))
        return *flag ? 1. : 0.;
    return std::nullopt;
}

std::string_view py_wire_type(const Value &value) {
    if (const auto *number =
            std::get_if<Number>(&value.data))
        return bike::io::is_integer_lexeme(number->lexeme) ? "int"
                                                           : "float";
    return detail::python_type_name(value);
}

[[noreturn]] void type_error(std::string_view op, const Value &a,
                             const Value &b) {
    throw std::invalid_argument(
        "'" + std::string(op) +
        "' not supported between instances of '" +
        std::string(py_wire_type(a)) + "' and '" +
        std::string(py_wire_type(b)) + "'");
}

// a <= b with Python operand semantics.
bool py_le(const Value &a, const Value &b) {
    const auto *sa = std::get_if<std::string>(&a.data);
    const auto *sb = std::get_if<std::string>(&b.data);
    if (sa != nullptr && sb != nullptr) return *sa <= *sb;
    const auto na = py_numeric(a), nb = py_numeric(b);
    if (na && nb) return *na <= *nb;
    type_error("<=", a, b);
}
bool py_lt(const Value &a, const Value &b) {
    const auto *sa = std::get_if<std::string>(&a.data);
    const auto *sb = std::get_if<std::string>(&b.data);
    if (sa != nullptr && sb != nullptr) return *sa < *sb;
    const auto na = py_numeric(a), nb = py_numeric(b);
    if (na && nb) return *na < *nb;
    type_error("<", a, b);
}
// lhs(double) <= rhs(value).
bool py_le_num(double lhs, const Value &rhs) {
    if (std::holds_alternative<std::string>(rhs.data))
        throw std::invalid_argument(
            "'<=' not supported between instances of 'float' and '" +
            std::string(py_wire_type(rhs)) + "'");
    const auto nb = py_numeric(rhs);
    if (nb) return lhs <= *nb;
    throw std::invalid_argument(
        "'<=' not supported between instances of 'float' and '" +
        std::string(py_wire_type(rhs)) + "'");
}
bool py_gt_num(const Value &lhs, double rhs) {
    if (std::holds_alternative<std::string>(lhs.data))
        throw std::invalid_argument(
            "'>' not supported between instances of '" +
            std::string(py_wire_type(lhs)) + "' and 'float'");
    const auto na = py_numeric(lhs);
    if (na) return *na > rhs;
    throw std::invalid_argument(
        "'>' not supported between instances of '" +
        std::string(py_wire_type(lhs)) + "' and 'float'");
}
bool py_lt_num(const Value &lhs, double rhs) {
    if (std::holds_alternative<std::string>(lhs.data)) {
        throw std::invalid_argument(
            "'<' not supported between instances of '" +
            std::string(py_wire_type(lhs)) + "' and 'float'");
    }
    const auto na = py_numeric(lhs);
    if (na) return *na < rhs;
    throw std::invalid_argument(
        "'<' not supported between instances of '" +
        std::string(py_wire_type(lhs)) + "' and 'float'");
}
// a - b — both operands must be numeric-ish (str raises TypeError).
double py_sub(const Value &a, const Value &b) {
    const auto na = py_numeric(a), nb = py_numeric(b);
    if (na && nb) return *na - *nb;
    throw std::invalid_argument(
        "unsupported operand type(s) for -: '" +
        std::string(py_wire_type(a)) + "' and '" +
        std::string(py_wire_type(b)) + "'");
}
// lhs(double) > rhs(value).
bool py_gt_dnum(double lhs, const Value &rhs) {
    const auto nb = py_numeric(rhs);
    if (nb) return lhs > *nb;
    throw std::invalid_argument(
        "'>' not supported between instances of 'float' and '" +
        std::string(py_wire_type(rhs)) + "'");
}
// lhs(double) >= rhs(value).
bool py_ge_dnum(double lhs, const Value &rhs) {
    const auto nb = py_numeric(rhs);
    if (nb) return lhs >= *nb;
    throw std::invalid_argument(
        "'>=' not supported between instances of 'float' and '" +
        std::string(py_wire_type(rhs)) + "'");
}
// value == rhs — Python equality: unlike types compare False, bool numeric.
bool py_eq_num(const Value &value, double rhs) {
    const auto n = py_numeric(value);
    return n && *n == rhs;
}
bool py_eq_int(const Value &value, long long rhs) {
    // float == int compares numerically (rhs fits binary64 here; the
    // comparisons the wire runs never carry >2^53 magnitudes).
    const auto n = py_numeric(value);
    return n && *n == static_cast<double>(rhs);
}
// value == rhs — True == 1 in Python, so numbers compare numerically.
bool py_eq_bool(const Value &value, bool rhs) {
    const auto n = py_numeric(value);
    return n && *n == (rhs ? 1. : 0.);
}

// schema.py sequence(): an Array only — strings/bytes/mappings/scalars are
// rejected like the Python Sequence dispatch (str/bytes/Mapping excluded,
// ndarray 0-d rejected; neither exists on this wire).
const Value::Array &wire_sequence(const Value &value,
                                  const std::string &path) {
    if (!std::holds_alternative<Value::Array>(value.data))
        fail(path + ": expected ordered sequence");
    return std::get<Value::Array>(value.data);
}

const Value::Object &wire_object(const Value &value,
                                 const std::string &path) {
    if (!std::holds_alternative<Value::Object>(value.data))
        fail(path + ": expected dict");
    return std::get<Value::Object>(value.data);
}

// ---- SCHEMAS ---------------------------------------------------------------
// The (required, optional) key tables, transcribed verbatim; the DAMPER
// shared prefix is already expanded inside the two damper rows.
struct SchemaEntry {
    std::string_view required;
    std::string_view optional;
};
// Aggregate row so the nested {.required/.optional} initializers stay legal
// (std::pair is not an aggregate and cannot carry designators).
struct KeyedSchemaEntry {
    std::string_view name;
    SchemaEntry entry;
};

// _schema(name) aliases applied at lookup time.
std::string_view schema_name(std::string_view name) {
    if (name == "tire.front" || name == "tire.rear") return "tire.side";
    if (name == "tire.front.material" || name == "tire.rear.material")
        return "tire.side.material";
    if (name == "tire.distributed.front" || name == "tire.distributed.rear")
        return "tire.distributed.material";
    if (name == "tire.pneumatic.front" || name == "tire.pneumatic.rear")
        return "tire.pneumatic.side";
    if (name.ends_with(".density")) return "tire.distributed.density";
    if (name.ends_with(".surface")) return "surface";
    return name;
}

// SCHEMAS[_schema(name)] — nullptr when the name carries no schema.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) required/optional out-pairs mirror the SCHEMAS row shape
bool schema_entry(std::string_view name, std::string_view &required,
                  std::string_view &optional) {
    const std::string_view key = schema_name(name);
    using SV = std::string_view;
    static const std::array<KeyedSchemaEntry, 36> entries{{
        {.name = "config", .entry =
         {.required = "schema",
          .optional = "drive cruise suspension brake resistance tire rider_forces "
          "rider_contacts"}},
        {.name = "suspension", .entry =
         {.required = "physics_mode joints air_spring fork_damper shock_damper coil "
          "end_stops",
          .optional = ""}},
        {.name = "suspension.joints", .entry = {.required = "fork shock",
          .optional = ""}},
        {.name = "suspension.air_spring", .entry =
         {.required = "stanchion_inner_diam_mm total_travel_mm pos_chamber_length_mm "
          "neg_chamber_length_mm token_volume_cm3 max_tokens gamma "
          "atm_pressure_pa num_tokens gauge_pressure_psi",
          .optional = ""}},
        {.name = "suspension.fork_damper", .entry =
         // DAMPER + ' total_travel_mm hbo_start_mm c_hbo_base'
         {.required = SV("max_hsc max_lsc max_reb hsc_clicks lsc_clicks rebound_clicks "
             "c_lsc_min c_lsc_max c_hsc_min c_hsc_max c_reb_min c_reb_max "
             "v_knee_comp v_knee_reb total_travel_mm hbo_start_mm c_hbo_base"),
          .optional = "legacy_behavior"}},
        {.name = "suspension.shock_damper", .entry =
         // DAMPER + ' total_stroke_mm max_hbo hbo_clicks lockout_firm
         // legacy_behavior hbo_start_mm c_hbo_min c_hbo_max
         // lockout_preload_n lockout_stiffness'
         {.required = SV("max_hsc max_lsc max_reb hsc_clicks lsc_clicks rebound_clicks "
             "c_lsc_min c_lsc_max c_hsc_min c_hsc_max c_reb_min c_reb_max "
             "v_knee_comp v_knee_reb total_stroke_mm max_hbo hbo_clicks "
             "lockout_firm legacy_behavior hbo_start_mm c_hbo_min c_hbo_max "
             "lockout_preload_n lockout_stiffness"),
          .optional = ""}},
        {.name = "suspension.coil", .entry =
         {.required = "rate_n_m preload_mm stroke_mm bumper_length_mm bumper_peak_n "
          "legacy_behavior",
          .optional = ""}},
        {.name = "suspension.end_stops", .entry = {.required = "stiffness_n_m damping_n_s_m",
          .optional = ""}},
        {.name = "brake", .entry = {.required = "torque_ceiling_nm taper_radps",
          .optional = ""}},
        {.name = "cruise", .entry =
         {.required = "target_speed_kmh kp_nm_per_mps ki_nm_per_mps_s "
          "torque_ceiling_nm",
          .optional = ""}},
        {.name = "resistance", .entry =
         {.required = "crr rolling_taper_rad_s rho_kg_m3 cda_m2 wind_world_mps "
          "point_body_m bodies",
          .optional = ""}},
        {.name = "resistance.bodies", .entry = {.required = "frame front_wheel rear_wheel",
          .optional = ""}},
        {.name = "tire", .entry =
         {.required = "backend surface_mode front rear significant_delta_m "
          "significance_fraction distinct_normal_deg",
          .optional = "surface_map distributed pneumatic"}},
        {.name = "tire.side", .entry = {.required = "tangent_k_n_m mu relaxation_length_m",
          .optional = "material"}},
        {.name = "tire.side.material", .entry =
         {.required = "radial_k_n_m radial_c_ns_m pressure_pa_gauge provenance "
          "valid_load_range_n",
          .optional = ""}},
        {.name = "tire.distributed", .entry = {.required = "station_count density front rear",
          .optional = ""}},
        {.name = "tire.distributed.density", .entry =
         {.required = "knots_m stiffness_n_m damping_ns_m provenance",
          .optional = ""}},
        {.name = "tire.distributed.material", .entry =
         {.required = "density fitting_dataset_id valid_load_range_n "
          "valid_deflection_range_m calibration_status",
          .optional = ""}},
        {.name = "tire.pneumatic", .entry =
         {.required = "tier n_rays half_angle_deg discretised_brush timestep_s front "
          "rear profile",
          .optional = "surface"}},
        {.name = "tire.pneumatic.side", .entry =
         {.required = "name outer_radius_mm rim_radius_mm section_height_mm "
          "pressure_bar crr_reference casing_width_mm compressed_casing_mm "
          "area_factor carcass_stiffness_n_mm2 contact_length_factor "
          "rate_stiffening rate_relaxation_s loss_factor "
          "hysteresis_rate_eps_mps relaxation_length_mm "
          "rim_stiffness_n_mm2 tread_loss_crr",
          .optional = ""}},
        {.name = "tire.pneumatic.profile", .entry = {.required = "x0_m dx_m z_m",
          .optional = ""}},
        {.name = "surface", .entry =
         {.required = "name mu_peak mu_slide slip_stiffness_per_load "
          "stribeck_speed_mps",
          .optional = ""}},
        {.name = "tire.surface_map", .entry = {.required = "surface sections",
          .optional = ""}},
        {.name = "tire.surface_map.sections", .entry = {.required = "start_m end_m surface",
          .optional = ""}},
        {.name = "rider_forces", .entry = {.required = "paths",
          .optional = ""}},
        {.name = "rider_forces.paths", .entry =
         {.required = "name joint stiffness_n_m damping_ns_m preload_deflection_m "
          "offset_m unilateral",
          .optional = ""}},
        {.name = "drive", .entry =
         {.required = "gearing pedaling shifting assist battery hub_stiffness_nm_rad "
          "hub_damping_nm_s drive_mode transmission_model human_torque_nm "
          "torque_ripple crank_phase_rad chain_k_n_m chain_c_ns_m "
          "bearing_c_nms_rad rotor_inertia_kgm2 motor_clutch "
          "brake_ceiling_nm",
          .optional = "brake_hold_torque_nm brake_hold_stiffness_nm_rad "
          "brake_hold_damping_nms_rad brake_hold_engage_radps "
          "brake_hold_release_radps"}},
        {.name = "policies", .entry =
         {.required = "gearing pedaling shifting assist battery "
          "hub_stiffness_nm_rad hub_damping_nm_s",
          .optional = ""}},
        {.name = "drive.gearing", .entry = {.required = "front_teeth rear_teeth chain_pitch_m",
          .optional = ""}},
        {.name = "drive.pedaling", .entry =
         {.required = "enabled coast_above_rpm resume_below_rpm stop_time_s "
          "coast_cadence_tau_s mash_cadence_rpm mash_torque_nm "
          "effort_slew_nm_s rollback_brake rollback_engage_mps "
          "rollback_release_mps rollback_demand",
          .optional = ""}},
        {.name = "drive.shifting", .entry =
         {.required = "enabled cassette target_cadence_min_rpm target_cadence_max_rpm "
          "shift_cooldown_s shift_cut_duration_s torque_factor "
          "cadence_smoothing_tau_s upshift_slip_limit_mps upshift_slip_mode",
          .optional = ""}},
        {.name = "drive.assist", .entry =
         {.required = "gain max_torque max_power tau slew engage_torque_nm "
          "gate_min_crank_rad_s cutoff_mps taper_width_mps mode",
          .optional = "profile torque_curve"}},
        {.name = "drive.assist.profile", .entry =
         {.required = "mode_gains emtb_full_gain_at_nm",
          .optional = ""}},
        {.name = "drive.assist.profile.mode_gains", .entry = {.required = "eco tour emtb turbo",
          .optional = ""}},
        {.name = "drive.battery", .entry =
         {.required = "enabled energy_j copper_w_per_nm2 speed_w_per_rad_s2 idle_w",
          .optional = ""}},
        {.name = "rider_contacts", .entry =
         {.required = "arm_reach_m saddle_patch_half_length_m pedal_patch_half_length_m "
          "support_pad_radius_m support_k_n_m support_c_ns_m pedal_c_ns_m "
          "support_tangent_k_n_m support_mu support_length_m grip_k_n_m "
          "grip_c_ns_m grip_release_distance_m grip_capture_distance_m "
          "grip_capture_speed_mps pedal_attachment saddle_attachment "
          "grip_attachment",
          .optional = "grip_pair_force_limit_n"}},
    }};
    for (const auto &[entry_key, entry] : entries)
        if (entry_key == key) {
            required = entry.required;
            optional = entry.optional;
            return true;
        }
    return false;
}

// The frozensets, kept as flat lookup tables.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (word, list) mirrors the Python membership predicate
bool in_words(std::string_view word, std::string_view list) {
    for (std::size_t pos = 0; pos < list.size();) {
        const std::size_t space = list.find(' ', pos);
        const std::string_view item =
            list.substr(pos, space == std::string_view::npos
                             ? std::string_view::npos
                             : space - pos);
        if (item == word) return true;
        pos = space == std::string_view::npos ? list.size() : space + 1;
    }
    return false;
}

constexpr std::string_view kBoolean =
    "enabled lockout_firm legacy_behavior unilateral motor_clutch "
    "rollback_brake discretised_brush";
constexpr std::string_view kInteger =
    "schema max_tokens num_tokens max_hsc max_lsc max_reb hsc_clicks "
    "lsc_clicks rebound_clicks max_hbo hbo_clicks front_teeth rear_teeth "
    "station_count n_rays";
constexpr std::string_view kString =
    "physics_mode fork shock frame front_wheel rear_wheel provenance name "
    "backend surface_mode joint drive_mode transmission_model "
    "upshift_slip_mode mode pedal_attachment saddle_attachment "
    "grip_attachment fitting_dataset_id calibration_status tier";
constexpr std::string_view kRealSequences = "knots_m stiffness_n_m z_m";

// WIDTHS — a name->width map (lookup by member key).
int sequence_width(std::string_view key) {
    static const std::array widths{
        std::pair{"wind_world_mps", 3}, std::pair{"point_body_m", 3},
        std::pair{"valid_load_range_n", 2},
        std::pair{"valid_deflection_range_m", 2},
        std::pair{"emtb", 2}};
    for (const auto &[name, w] : widths)
        if (name == key) return w;
    return 0;
}

std::vector<std::string_view> words(std::string_view list) {
    std::vector<std::string_view> out;
    for (std::size_t pos = 0; pos < list.size();) {
        const std::size_t space = list.find(' ', pos);
        out.push_back(list.substr(
            pos, space == std::string_view::npos ? std::string_view::npos
                                                 : space - pos));
        pos = space == std::string_view::npos ? list.size() : space + 1;
    }
    return out;
}

// schema.py exact_keys.
void exact_keys(const Value::Object &object, std::string_view required_list,
                std::string_view optional_list, const std::string &path) {
    for (const auto &[key, member] : object) {
        if (!in_words(key, required_list) && !in_words(key, optional_list)) {
            std::string message = path;
            message += '.';
            message += key;
            message += ": unknown key";
            fail(message);
        }
    }
    for (const std::string_view key : words(required_list))
        if (bike::io::find_member(object, key) == nullptr) {
            std::string message = path;
            message += '.';
            message += key;
            message += ": missing required key";
            fail(message);
        }
}

void validate_mapping(const Value &value, const std::string &name,
                      const std::string &path);

// ---- the _check_* domain validators -----------------------------------------
// _check_hinge_density(density, path) — HingeDensity.__post_init__ on wire.
void check_hinge_density(const Value::Object &density,
                         const std::string &path) {
    // Named path strings keep every wire_* argument an lvalue — a prvalue
    // bound to a const-ref parameter trips -Wdangling-reference on the
    // reference-typed result.
    const std::string knots_path = path + ".knots_m";
    const auto &knots = wire_sequence(
        *bike::io::find_member(density, "knots_m"), knots_path);
    const std::string stiffness_path = path + ".stiffness_n_m";
    const auto &stiffness = wire_sequence(
        *bike::io::find_member(density, "stiffness_n_m"), stiffness_path);
    if (knots.empty() || knots.size() != stiffness.size())
        fail(path + ": invalid density table shape");
    // knots[0] != 0. compares the raw wire value — a string member never
    // equals the float zero, and bools compare numerically.
    bool monotone = py_eq_num(knots[0], 0.);
    // any(b <= a for a, b in zip(knots, knots[1:]))
    for (std::size_t i = 1; i < knots.size() && monotone; ++i)
        if (py_le(knots[i], knots[i - 1])) monotone = false;
    if (!monotone)
        fail(path + ".knots_m: strictly increasing from zero required");
    for (const Value &k : stiffness)
        if (py_lt_num(k, 0.))
            fail(path + ".stiffness_n_m: negative stiffness");
    wire_scalar(*bike::io::find_member(density, "damping_ns_m"),
                path + ".damping_ns_m", 0.);
    const Value *provenance =
        bike::io::find_member(density, "provenance");
    const auto *text = provenance == nullptr
        ? nullptr
        : std::get_if<std::string>(&provenance->data);
    if (text == nullptr || !nativecfg::nonblank(*text))
        fail(path + ".provenance: nonblank description required");
}

// _check_distributed_material(material, path).
void check_distributed_material(const Value &material,
                                const std::string &path) {
    const auto &object =
        wire_object(material, path);  // material['density'] requires dict
    check_hinge_density(
        wire_object(*bike::io::find_member(object, "density"),
                    path + ".density"),
        path + ".density");
    const Value *fit = bike::io::find_member(object, "fitting_dataset_id");
    const auto *fit_text =
        fit == nullptr ? nullptr : std::get_if<std::string>(&fit->data);
    if (fit_text == nullptr || !nativecfg::nonblank(*fit_text))
        fail(path + ".fitting_dataset_id: nonblank ID required");
    for (const char *key : {"valid_load_range_n",
                           "valid_deflection_range_m"}) {
        const std::string range_path = path + "." + key;
        const auto &range = wire_sequence(
            *bike::io::find_member(object, key), range_path);
        // WIDTHS validated the two-element shape; the bounds compare the
        // raw values (str members raise the '<=' TypeError like Python).
        if (!(py_le_num(0., range[0]) && py_lt(range[0], range[1])))
            fail(path + "." + key +
                 ": expected 0 <= minimum < maximum");
    }
    const Value *status =
        bike::io::find_member(object, "calibration_status");
    const auto *status_text = status == nullptr
        ? nullptr
        : std::get_if<std::string>(&status->data);
    if (status_text == nullptr ||
        *status_text != "experimental_unvalidated")
        fail(path + ".calibration_status: unsupported status");
}

// _check_distributed(dist).
void check_distributed(const Value::Object &dist) {
    const std::string path = "config.tire.distributed";
    const long long count = bike::io::as_int64(
        *bike::io::find_member(dist, "station_count"),
        path + ".station_count");
    if (count < 16 || count > 512)
        fail(path + ".station_count: outside 16..512");
    check_hinge_density(
        wire_object(*bike::io::find_member(dist, "density"),
                    path + ".density"),
        path + ".density");
    check_distributed_material(
        *bike::io::find_member(dist, "front"), path + ".front");
    check_distributed_material(
        *bike::io::find_member(dist, "rear"), path + ".rear");
}

// _check_tyre_specs(spec, path) — TyreSpecs.__post_init__ domains on wire.
void check_tyre_specs(const Value::Object &spec, const std::string &path) {
    for (const char *key :
         {"outer_radius_mm", "rim_radius_mm", "section_height_mm",
          "pressure_bar", "crr_reference", "casing_width_mm",
          "compressed_casing_mm", "area_factor", "contact_length_factor",
          "rate_relaxation_s", "hysteresis_rate_eps_mps",
          "relaxation_length_mm", "rim_stiffness_n_mm2"})
        wire_scalar(*bike::io::find_member(spec, key), path + "." + key,
                    std::nullopt, true);
    for (const char *key :
         {"carcass_stiffness_n_mm2", "rate_stiffening", "loss_factor",
          "tread_loss_crr"})
        wire_scalar(*bike::io::find_member(spec, key), path + "." + key,
                    0.);
    const Value &rim_v = *bike::io::find_member(spec, "rim_radius_mm");
    const Value &outer_v = *bike::io::find_member(spec, "outer_radius_mm");
    const Value &casing_v =
        *bike::io::find_member(spec, "compressed_casing_mm");
    if (!py_lt(rim_v, outer_v))
        fail(path + ".rim_radius_mm: rim is not inside the outer radius");
    // outer - rim > casing — str operands raise the '-' TypeError first.
    const double travel = py_sub(outer_v, rim_v);
    if (!py_gt_dnum(travel, casing_v))
        fail(path + ".compressed_casing_mm: no travel above the rim");
    const Value &area_v = *bike::io::find_member(spec, "area_factor");
    const Value &contact_v =
        *bike::io::find_member(spec, "contact_length_factor");
    if (py_gt_num(area_v, 1.) || py_gt_num(contact_v, 1.))
        fail(path + ": area and contact-length factors are at most 1");
    const Value &loss_v = *bike::io::find_member(spec, "loss_factor");
    if (!py_lt_num(loss_v, 1.))
        fail(path + ".loss_factor: must be below 1");
}

// _check_pneumatic(pneu) — the resolved tier must match its registry row
// bitwise, each side passes the TyreSpecs domains, the road profile holds
// its uniform-grid shape.
void check_pneumatic(const Value::Object &pneu) {
    const std::string path = "config.tire.pneumatic";
    const Value *tier_value = bike::io::find_member(pneu, "tier");
    const auto *tier_member = tier_value == nullptr
        ? nullptr
        : std::get_if<std::string>(&tier_value->data);
    const std::string_view tier_name =
        tier_member == nullptr ? std::string_view{} : *tier_member;
    const TierSpec &tier = [&]() -> const TierSpec & {
        if (const TierSpec *resolved = find_tire_tier(tier_name))
            return *resolved;
        fail(path + ".tier: unsupported tier");
    }();
    const auto member_of = [&](const char *key) -> const Value & {
        return *bike::io::find_member(pneu, key);
    };
    if (!py_eq_int(member_of("n_rays"), tier.n_rays))
        fail(path + ".n_rays: not the resolved value for the tier");
    if (!py_eq_num(member_of("half_angle_deg"), tier.half_angle_deg))
        fail(path + ".half_angle_deg: not the resolved value for the tier");
    if (!py_eq_bool(member_of("discretised_brush"),
                    tier.discretised_brush))
        fail(path +
             ".discretised_brush: not the resolved value for the tier");
    if (!py_eq_num(member_of("timestep_s"), tier.timestep_s))
        fail(path + ".timestep_s: not the resolved value for the tier");
    check_tyre_specs(
        wire_object(*bike::io::find_member(pneu, "front"),
                    path + ".front"),
        path + ".front");
    check_tyre_specs(
        wire_object(*bike::io::find_member(pneu, "rear"),
                    path + ".rear"),
        path + ".rear");
    const std::string profile_path = path + ".profile";
    const Value::Object &profile = wire_object(
        *bike::io::find_member(pneu, "profile"), profile_path);
    wire_scalar(*bike::io::find_member(profile, "x0_m"),
                path + ".profile.x0_m");
    wire_scalar(*bike::io::find_member(profile, "dx_m"),
                path + ".profile.dx_m", std::nullopt, true);
    const std::string z_path = path + ".profile.z_m";
    const auto &z = wire_sequence(
        *bike::io::find_member(profile, "z_m"), z_path);
    if (z.size() < 2)
        fail(path + ".profile.z_m: at least two samples required");
}

// ---- validate_mapping -------------------------------------------------------
// NOLINTNEXTLINE(misc-no-recursion) child-map descent mirrors schema.py validate_mapping recursion
void validate_mapping(const Value &value, const std::string &name,
                      const std::string &path) {
    std::string_view required, optional;
    // SCHEMAS[_schema(name)] — a schema'd name is guaranteed by callers.
    if (!schema_entry(name, required, optional))
        fail(path + ": no schema for " + name);
    const Value::Object &object = wire_object(value, path);
    if (schema_name(name) == "tire.side.material" &&
        (bike::io::find_member(object, "deflection_m") != nullptr ||
         bike::io::find_member(object, "force_n") != nullptr)) {
        required = "deflection_m force_n radial_c_ns_m pressure_pa_gauge provenance valid_load_range_n";
        optional = "";
    }
    exact_keys(object, required, optional, path);
    for (const auto &[key, item] : object) {
        std::string child = path;
        child += '.';
        child += key;
        std::string nested;
        if (name == "config" || name == "policies") {
            nested = key;
        } else {
            nested = name;
            nested += '.';
            nested += key;
        }
        if (name == "policies" &&
            (key == "gearing" || key == "pedaling" || key == "shifting" ||
             key == "assist" || key == "battery"))
            nested = "drive." + key;
        if (key == "sections" || key == "paths") {
            std::size_t index = 0;
            for (const Value &row : wire_sequence(item, child))
                validate_mapping(row, nested,
                                 child + "." + std::to_string(index++));
        } else if (schema_entry(nested, required, optional)) {
            // The one nullable nested mapping is drive.assist.profile.
            if (std::holds_alternative<std::monostate>(item.data) &&
                schema_name(nested) == "drive.assist.profile")
                continue;
            validate_mapping(item, nested, child);
        } else if (const int width = sequence_width(key)) {
            const auto &values = wire_sequence(item, child);
            if (values.size() != static_cast<std::size_t>(width))
                fail(child + ": incorrect sequence width");
            for (std::size_t index = 0; index < values.size(); ++index)
                wire_scalar(values[index],
                            child + "." + std::to_string(index));
        } else if (key == "cassette") {
            const auto &values = wire_sequence(item, child);
            for (std::size_t index = 0; index < values.size(); ++index)
                wire_integer(values[index],
                             child + "." + std::to_string(index));
        } else if (((key == "deflection_m" || key == "force_n") &&
                    schema_name(name) == "tire.side.material") ||
                   (in_words(key, kRealSequences) &&
                   (schema_name(name) == "tire.distributed.density" ||
                    schema_name(name) == "tire.pneumatic.profile"))) {
            const auto &values = wire_sequence(item, child);
            for (std::size_t index = 0; index < values.size(); ++index)
                wire_scalar(values[index],
                            child + "." + std::to_string(index));
        } else if (key == "grip_pair_force_limit_n") {
            if (!std::holds_alternative<std::monostate>(item.data))
                wire_scalar(item, child);
        } else if (key == "torque_curve") {
            if (!std::holds_alternative<std::monostate>(item.data)) {
                const auto &rows = wire_sequence(item, child);
                for (std::size_t index = 0; index < rows.size(); ++index) {
                    const std::string row_path =
                        child + "." + std::to_string(index);
                    const auto &row = wire_sequence(rows[index], row_path);
                    if (row.size() != 2)
                        fail(row_path + ": incorrect sequence width");
                    for (const Value &number : row)
                        wire_scalar(number, row_path);
                }
            }
        } else if (in_words(key, kBoolean)) {
            wire_boolean(item, child);
        } else if (in_words(key, kInteger)) {
            wire_integer(item, child);
        } else if (in_words(key, kString)) {
            const auto *text = std::get_if<std::string>(&item.data);
            if (text == nullptr) fail(child + ": expected string");
            // io strings are validated UTF-8 at parse — str.encode cannot
            // fail here.
        } else {
            wire_scalar(item, child);
        }
    }
}

} // namespace

// schema.py validate_policy_modes(drive, path).
void validate_policy_modes(const Value &drive, std::string_view path) {
    const std::string p(path);
    const auto &object = wire_object(drive, p);
    const std::string shifting_path = p + ".shifting";
    const auto &shifting = wire_object(
        *bike::io::find_member(object, "shifting"), shifting_path);
    const Value *mode_member =
        bike::io::find_member(shifting, "upshift_slip_mode");
    const auto *mode = mode_member == nullptr
        ? nullptr
        : std::get_if<std::string>(&mode_member->data);
    if (mode == nullptr ||
        (*mode != "legacy_signed" && *mode != "magnitude"))
        fail(p + ".shifting.upshift_slip_mode: unsupported mode");
    const std::string assist_path = p + ".assist";
    const auto &assist = wire_object(
        *bike::io::find_member(object, "assist"), assist_path);
    const Value *profile =
        bike::io::find_member(assist, "profile");
    const Value *assist_mode_member =
        bike::io::find_member(assist, "mode");
    const auto *assist_mode = assist_mode_member == nullptr
        ? nullptr
        : std::get_if<std::string>(&assist_mode_member->data);
    const bool profile_present =
        profile != nullptr &&
        !std::holds_alternative<std::monostate>(profile->data);
    if (profile_present &&
        (assist_mode == nullptr ||
         (*assist_mode != "eco" && *assist_mode != "tour" &&
          *assist_mode != "emtb" && *assist_mode != "turbo")))
        fail(p + ".assist.mode: unsupported profiled mode");
}

// schema.py validate_drive_policies(config): the 'policies' schema over the
// drive section, then the mode checks.
void validate_drive_policies(const Value &config) {
    validate_mapping(config, "policies", "config.drive");
    validate_policy_modes(config, "config.drive");
}

// schema.py validate_rider_contacts(contacts, path='config.rider_contacts').
void validate_rider_contacts(const Value &contacts,
                             std::string_view path) {
    const std::string p(path);
    const auto &object = wire_object(contacts, p);
    const auto attachment = [&](const char *key) {
        const Value *member = bike::io::find_member(object, key);
        return member == nullptr ||
                   !std::holds_alternative<std::string>(member->data)
            ? std::string_view{}
            : std::string_view(*std::get_if<std::string>(&member->data));
    };
    if (!in_words(attachment("pedal_attachment"), "flat weld spindle"))
        fail(p + ".pedal_attachment: unsupported attachment");
    if (!in_words(attachment("saddle_attachment"), "flat weld pin"))
        fail(p + ".saddle_attachment: unsupported attachment");
    if (!in_words(attachment("grip_attachment"), "spring connect"))
        fail(p + ".grip_attachment: unsupported attachment");
    wire_scalar(*bike::io::find_member(object, "arm_reach_m"),
                p + ".arm_reach_m", std::nullopt, true);
    for (const char *key :
         {"support_pad_radius_m", "support_k_n_m", "support_tangent_k_n_m",
          "support_length_m", "grip_k_n_m", "grip_release_distance_m"})
        wire_scalar(*bike::io::find_member(object, key), p + "." + key,
                    std::nullopt, true);
    for (const char *key :
         {"saddle_patch_half_length_m", "pedal_patch_half_length_m",
          "support_c_ns_m", "pedal_c_ns_m", "support_mu", "grip_c_ns_m",
          "grip_capture_distance_m", "grip_capture_speed_mps"})
        wire_scalar(*bike::io::find_member(object, key), p + "." + key, 0.);
    const Value *limit =
        bike::io::find_member(object, "grip_pair_force_limit_n");
    if (limit != nullptr &&
        !std::holds_alternative<std::monostate>(limit->data))
        wire_scalar(*limit, p + ".grip_pair_force_limit_n", std::nullopt,
                    true);
}

// schema.py validate_config(config).
void validate_config(const Value &config) {
    const auto *object = std::get_if<Value::Object>(&config.data);
    if (object == nullptr) fail("config: expected dict");
    if (object->empty()) return;
    validate_mapping(config, "config", "config");
    const long long schema_version = bike::io::as_int64(
        *bike::io::find_member(*object, "schema"), "config.schema");
    if (schema_version != 1 && schema_version != 2)
        fail("config.schema: unsupported schema");
    const Value *suspension =
        bike::io::find_member(*object, "suspension");
    if (suspension != nullptr &&
        std::holds_alternative<Value::Object>(suspension->data)) {
        const auto &susp = std::get<Value::Object>(suspension->data);
        const Value *mode_member =
            bike::io::find_member(susp, "physics_mode");
        const auto *mode = mode_member == nullptr
            ? nullptr
            : std::get_if<std::string>(&mode_member->data);
        if (mode == nullptr ||
            (*mode != "legacy" && *mode != "physical"))
            fail("config.suspension.physics_mode: unsupported mode");
        const std::string fork_path = "config.suspension.fork_damper";
        const auto &fork = wire_object(
            *bike::io::find_member(susp, "fork_damper"), fork_path);
        const Value *legacy_member =
            bike::io::find_member(fork, "legacy_behavior");
        if (schema_version == 2 && legacy_member == nullptr)
            fail("config.suspension.fork_damper.legacy_behavior: missing "
                 "required key");
        if (schema_version == 1 && legacy_member != nullptr)
            fail("config.suspension.fork_damper.legacy_behavior: unknown "
                 "schema-1 key");
        const Value *hbo_member =
            bike::io::find_member(fork, "hbo_start_mm");
        // schema 1: hbo_start_mm == 160. on the raw value — a string member
        // never compares equal, so it takes the modern (strict) path.
        const bool legacy =
            schema_version == 1
                ? py_eq_num(*hbo_member, 160.)
                : legacy_member != nullptr &&
                      std::holds_alternative<bool>(legacy_member->data) &&
                      std::get<bool>(legacy_member->data);
        wire_scalar(
            *bike::io::find_member(fork, "total_travel_mm"),
            "config.suspension.fork_damper.total_travel_mm",
            std::nullopt, true);
        const double start = wire_scalar(
            *hbo_member, "config.suspension.fork_damper.hbo_start_mm", 0.);
        // `start >= fork['total_travel_mm']` compares the raw member —
        // a parseable string passed scalar() but raises the TypeError
        // here, exactly like Python.
        if (!legacy && py_ge_dnum(start, *bike::io::find_member(
                                          fork, "total_travel_mm")))
            fail("config.suspension.fork_damper.hbo_start_mm: invalid "
                 "modern HBO zone");
    }
    const Value *tire = bike::io::find_member(*object, "tire");
    if (tire != nullptr &&
        std::holds_alternative<Value::Object>(tire->data)) {
        const auto &tire_map = std::get<Value::Object>(tire->data);
        const auto member = [&](std::string_view key) -> const Value * {
            return bike::io::find_member(tire_map, key);
        };
        const auto *backend_member =
            std::get_if<std::string>(&member("backend")->data);
        const std::string_view backend =
            backend_member == nullptr ? std::string_view{} : *backend_member;
        if (backend != "compliant_2d" &&
            backend != "distributed_2d_reference" && backend != "pneumatic")
            fail("config.tire.backend: unsupported backend");
        const auto *surface_mode_member =
            std::get_if<std::string>(&member("surface_mode")->data);
        const std::string_view surface_mode =
            surface_mode_member == nullptr ? std::string_view{}
                                           : *surface_mode_member;
        if (surface_mode != "configured" && surface_mode != "track")
            fail("config.tire.surface_mode: unsupported mode");
        if (surface_mode == "track" &&
            member("surface_map") == nullptr)
            fail("config.tire.surface_map: missing required key");
        const bool distributed = backend == "distributed_2d_reference";
        const bool pneumatic = backend == "pneumatic";
        if (distributed && member("distributed") == nullptr)
            fail("config.tire.distributed: missing required key");
        if (!distributed && member("distributed") != nullptr)
            fail("config.tire.distributed: unknown key");
        if (pneumatic && member("pneumatic") == nullptr)
            fail("config.tire.pneumatic: missing required key");
        if (!pneumatic && member("pneumatic") != nullptr)
            fail("config.tire.pneumatic: unknown key");
        for (const char *side : {"front", "rear"}) {
            const std::string side_map_path =
                std::string("config.tire.") + side;
            const auto &side_map =
                wire_object(*member(side), side_map_path);
            const bool present =
                bike::io::find_member(side_map, "material") != nullptr;
            const std::string side_path =
                std::string("config.tire.") + side + ".material";
            if ((distributed || pneumatic) && present)
                fail(side_path + ": unknown key");
            if (!distributed && !pneumatic && !present)
                fail(side_path + ": missing required key");
        }
        if (distributed)
            check_distributed(
                wire_object(*member("distributed"),
                            "config.tire.distributed"));
        if (pneumatic) {
            const std::string pneu_path = "config.tire.pneumatic";
            const auto &pneu = wire_object(*member("pneumatic"), pneu_path);
            if (surface_mode != "track" &&
                bike::io::find_member(pneu, "surface") == nullptr)
                fail("config.tire.pneumatic.surface: missing required key");
            check_pneumatic(pneu);
        }
    }
    const Value *drive = bike::io::find_member(*object, "drive");
    if (drive != nullptr &&
        std::holds_alternative<Value::Object>(drive->data)) {
        const auto &drive_map = std::get<Value::Object>(drive->data);
        const auto *mode_member = std::get_if<std::string>(
            &bike::io::find_member(drive_map, "drive_mode")->data);
        const std::string_view mode =
            mode_member == nullptr ? std::string_view{} : *mode_member;
        if (mode != "crank_effort" && mode != "articulated_effort" &&
            mode != "coast" && mode != "ideal_speed_control")
            fail("config.drive.drive_mode: unsupported mode");
        const auto *transmission_member = std::get_if<std::string>(
            &bike::io::find_member(drive_map, "transmission_model")->data);
        const std::string_view transmission =
            transmission_member == nullptr ? std::string_view{}
                                           : *transmission_member;
        if (transmission != "elastic_chain" &&
            transmission != "ideal_mid_drive" &&
            transmission != "geometric_ideal_mid_drive")
            fail("config.drive.transmission_model: unsupported model");
        validate_policy_modes(*drive, "config.drive");
    }
    const Value *rider_contacts =
        bike::io::find_member(*object, "rider_contacts");
    if (rider_contacts != nullptr &&
        std::holds_alternative<Value::Object>(rider_contacts->data))
        validate_rider_contacts(*rider_contacts);
}

// schema.py plain(value, key): post-validation canonicalization. Dict ->
// object (members recurse carrying the member key), sequence -> array,
// str -> itself, bool/None -> itself, Integral at INTEGER/'cassette' keys ->
// int, anything else -> float(value). The C++ port re-emits numbers with
// their canonical lexemes so canonical_json byte-compares cleanly.
// NOLINTNEXTLINE(misc-no-recursion) member-wise descent mirrors schema.py plain() recursion
Value plain(const Value &value, std::string_view key) {
    if (const auto *object = std::get_if<Value::Object>(&value.data)) {
        Value::Object members;
        members.reserve(object->size());
        for (const auto &[member_key, member] : *object)
            members.emplace_back(member_key, plain(member, member_key));
        return Value{.data = std::move(members)};
    }
    if (const auto *items = std::get_if<Value::Array>(&value.data)) {
        Value::Array out;
        out.reserve(items->size());
        for (const Value &item : *items) out.push_back(plain(item, key));
        return Value{.data = std::move(out)};
    }
    if (const auto *text = std::get_if<std::string>(&value.data))
        return Value{.data = *text};
    if (std::holds_alternative<bool>(value.data) ||
        std::holds_alternative<std::monostate>(value.data))
        return value;
    if (const auto *number = std::get_if<Number>(&value.data)) {
        if ((in_words(key, kInteger) || key == "cassette") &&
            bike::io::is_integer_lexeme(number->lexeme)) {
            // Python int(value) is arbitrary-precision: emit the exact
            // integral value, not a binary64 round-trip. The validated-wire
            // members are int32-bounded; a standalone call on a wider
            // lexeme still keeps its magnitude.
            std::string_view digits = number->lexeme;
            const bool negative = digits.front() == '-';
            if (negative) digits.remove_prefix(1);
            const auto first = digits.find_first_not_of('0');
            digits.remove_prefix(
                first == std::string_view::npos ? digits.size() : first);
            std::string normalized = digits.empty()
                ? "0"
                : (negative ? "-" + std::string(digits)
                            : std::string(digits));
            return Value{.data = Number{std::move(normalized)}};
        }
        bool ok = false;
        const double parsed = bike::io::detail::float_from_lexeme(
            number->lexeme, ok);
        return Value{.data = Number{detail::repr_double(parsed)}};
    }
    // Bytes/Tagged cannot appear after validation; Python float(value)
    // would fail — the port fails closed with the float() diagnostic.
    throw std::invalid_argument(
        "float() argument must be a string or a real number, not '" +
        std::string(detail::python_type_name(value)) + "'");
}

} // namespace wire
} // namespace bike::configuration
