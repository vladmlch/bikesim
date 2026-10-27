// configuration/load.cpp — T13 port of physics/resolution.py: the TOML ->
// bike::io::Value decode (pinned toml++ 3.4.0), the `_merge`/`_construct`
// resolution chain, and the produced load_physics/resolve_physics boundary.
// File-level concerns (bounded reads, asset path resolution) live in
// paths.cpp; the TOML writer lives in profile_writer.cpp.
//
// The construct path walks the T12 FieldDecl/RecordSchema tables — there is
// no second field registry to drift. Semantics follow resolution.py:
// table/unknown/missing checks run in source order, child sections
// construct before the parent's cls(**kwargs) (here: missing-required check
// and leaf decode), and each record's T12 detail::post_init performs the
// __post_init__ domain validation.

#include "configuration/load.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "configuration/fields.hpp"
#include "configuration/physical_config.hpp"
#include "configuration/wire_decode.hpp"

namespace bike::configuration {
namespace {

using bike::io::Number;
using bike::io::Value;

// ---- TOML -> Value ---------------------------------------------------------

Value toml_node(const toml::node& node);

// NOLINTNEXTLINE(misc-no-recursion) mirrors the document tree
Value toml_table(const toml::table& table) {
    Value::Object members;
    members.reserve(table.size());
    for (const auto& [key, node] : table)
        members.emplace_back(std::string(key.str()), toml_node(node));
    return Value{.data = std::move(members)};
}

// NOLINTNEXTLINE(misc-no-recursion) mirrors the document tree
Value toml_array(const toml::array& array) {
    Value::Array items;
    items.reserve(array.size());
    for (const toml::node& item : array)
        items.push_back(toml_node(item));
    return Value{.data = std::move(items)};
}

// NOLINTNEXTLINE(misc-no-recursion) mirrors the document tree
Value toml_node(const toml::node& node) {
    switch (node.type()) {
    case toml::node_type::table:
        return toml_table(*node.as_table());
    case toml::node_type::array:
        return toml_array(*node.as_array());
    case toml::node_type::string:
        return Value{.data = node.as_string()->get()};
    case toml::node_type::integer:
        return Value{.data = Number{
                        std::to_string(node.as_integer()->get())}};
    case toml::node_type::floating_point:
        // tomllib hands back a binary64 — the lexeme becomes the shortest
        // repr() spelling, the same one asdict() carries downstream.
        return Value{.data = Number{
                        detail::repr_double(node.as_floating_point()->get())}};
    case toml::node_type::boolean:
        return Value{.data = node.as_boolean()->get()};
    case toml::node_type::date:
        return Value{.data =
                         bike::io::Tagged{.kind = "toml.date", .fields = {}}};
    case toml::node_type::time:
        return Value{.data =
                         bike::io::Tagged{.kind = "toml.time", .fields = {}}};
    case toml::node_type::date_time:
        return Value{
            .data = bike::io::Tagged{.kind = "toml.date_time", .fields = {}}};
    case toml::node_type::none:
        break;
    }
    return Value{};
}

// ---- shared record helpers -------------------------------------------------

std::string_view class_name(const RecordSchema& schema) {
    const std::size_t dot = schema.path.rfind('.');
    return dot == std::string_view::npos ? schema.path
                                         : schema.path.substr(dot + 1);
}

const void* member_addr(const FieldDecl& field, const void* base) {
    // Same bounded uintptr round-trip schema.cpp uses — offsetof is a
    // compile-time constant, so the arithmetic never escapes the record.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast,performance-no-int-to-ptr) bounded offsetof arithmetic
    return reinterpret_cast<const void*>(
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) see above
        reinterpret_cast<std::uintptr_t>(base) + field.offset);
}
// NOLINTNEXTLINE(misc-const-correctness) callers write through the returned member alias
void* member_addr(const FieldDecl& field, void* base) {
    // The writable view writes through the record the caller owns — the
    // const overload carries the arithmetic; this only widens the alias.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) caller-owned record member
    return const_cast<void*>(
        member_addr(field, static_cast<const void*>(base)));
}

[[noreturn]] void construct_error(const std::string& message) {
    throw std::invalid_argument(message);
}

// CPython argument-list spelling: 'a' and 'b' for two names, the Oxford
// ', and ' before the last of three or more.
std::string and_join(std::span<const std::string> names) {
    std::string out;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i != 0)
            out += (i + 1 == names.size())
                       ? (names.size() > 2 ? ", and " : " and ")
                       : ", ";
        out += '\'';
        out += names[i];
        out += '\'';
    }
    return out;
}

Value::Object& object_at(Value& value) {
    return std::get<Value::Object>(value.data);
}

Value::Object merge_objects(const Value::Object& base,
                            const Value::Object& overrides);

// The _merge recursion: deep-copied base, override members applied in order.
// NOLINTNEXTLINE(misc-no-recursion) mirrors the document tree
void merge_into(Value::Object& result, const Value::Object& overrides) {
    for (const auto& [key, member] : overrides) {
        if (std::holds_alternative<std::monostate>(member.data))
            continue;  // `if value is None: continue` — flag absent
        Value::Object::iterator found = result.end();
        for (auto it = result.begin(); it != result.end(); ++it)
            if (it->first == key) {
                found = it;
                break;
            }
        const auto* override_object =
            std::get_if<Value::Object>(&member.data);
        if (override_object != nullptr && found != result.end() &&
            std::holds_alternative<Value::Object>(found->second.data)) {
            merge_into(object_at(found->second), *override_object);
        } else if (found != result.end()) {
            found->second = member;
        } else {
            result.emplace_back(key, member);
        }
    }
}

// NOLINTNEXTLINE(misc-no-recursion,bugprone-easily-swappable-parameters) mirrors the document tree; signature order mirrors _merge(base, override)
Value::Object merge_objects(const Value::Object& base,
                            const Value::Object& overrides) {
    Value::Object result = base;
    merge_into(result, overrides);
    return result;
}

// Public spatial input has equivalent tyre spellings, while to_value(base)
// always emits the preferred root tires.pneumatic_3d form. Remove only the
// inherited semantic fields explicitly overridden through an alias. The
// override document remains intact so contradictory declarations in that
// same document still reach the strict spatial parser together.
void prepare_spatial_overrides(Value& base, const Value& overrides) {
    auto* base_object = std::get_if<Value::Object>(&base.data);
    const auto* override_object = std::get_if<Value::Object>(&overrides.data);
    if (base_object == nullptr || override_object == nullptr) return;

    const auto erase_member = [](Value::Object& object, std::string_view key) {
        std::erase_if(object, [&](const auto& member) { return member.first == key; });
    };
    const auto mutable_object = [](Value::Object& object, std::string_view key) {
        for (auto& [name, value] : object)
            if (name == key) return std::get_if<Value::Object>(&value.data);
        return static_cast<Value::Object*>(nullptr);
    };
    const auto nested_object = [](const Value::Object* object, std::string_view key) {
        const Value* value = object == nullptr ? nullptr : io::find_member(*object, key);
        return value == nullptr ? nullptr : std::get_if<Value::Object>(&value->data);
    };
    const auto explicit_field = [](const Value::Object* object, std::string_view key) {
        const Value* value = object == nullptr ? nullptr : io::find_member(*object, key);
        return value == nullptr || io::is_null(*value) ? nullptr : value;
    };

    const Value* mode = io::find_member(*override_object, "spatial");
    const auto* mode_name = mode == nullptr ? nullptr : std::get_if<std::string>(&mode->data);
    if (mode_name != nullptr && *mode_name == "planar") {
        // Explicit planar+spatial data is invalid even when the base was 3D.
        // Reject its presence before merging, including explicit null sections.
        // Legacy tires/backend nulls retain merge's absent-override semantics;
        // their types and backend choice validate on the final merged document.
        if (io::find_member(*override_object, "spatial_3d") != nullptr)
            construct_error("spatial_3d: requires spatial='3d'");
        if (const auto* tires = nested_object(override_object, "tires"))
            if (io::find_member(*tires, "pneumatic_3d") != nullptr)
                construct_error("tires.pneumatic_3d: requires spatial='3d'");
        erase_member(*base_object, "spatial_3d");
        if (auto* tires = mutable_object(*base_object, "tires"))
            erase_member(*tires, "pneumatic_3d");
        // The shoulder profile is spatial-only data carried inside the planar
        // articulated table. A null override cannot clear it (merge treats
        // null as absent), so inheriting it would make 3D -> planar
        // impossible. An override that itself supplies one stays invalid.
        if (auto* articulated = mutable_object(*base_object, "articulated")) {
            erase_member(*articulated, "shoulder_profile");
            erase_member(*articulated, "shoulder_profile_sha256");
        }
        // The backend is intentionally retained: a 3D -> planar transition
        // must explicitly choose a valid legacy backend in the override.
        return;
    }

    auto* base_tires = mutable_object(*base_object, "tires");
    auto* inherited = base_tires == nullptr
        ? nullptr : mutable_object(*base_tires, "pneumatic_3d");
    if (inherited == nullptr) return;

    const auto shadow_lateral = [&](const Value* value) {
        if (value == nullptr) return;
        erase_member(*inherited, "lateral_coefficients");
        // The object spelling owns both id and sha256; a string owns the id
        // only, so an unmodified inherited digest keeps normal merge semantics.
        if (const auto* object = std::get_if<Value::Object>(&value->data))
            if (explicit_field(object, "sha256") != nullptr)
                erase_member(*inherited, "lateral_coefficients_sha256");
    };
    const auto shadow_tyre = [&](const Value::Object* object) {
        if (explicit_field(object, "tier") != nullptr)
            erase_member(*inherited, "tier");
        shadow_lateral(explicit_field(object, "lateral_coefficients"));
        if (explicit_field(object, "lateral_coefficients_sha256") != nullptr)
            erase_member(*inherited, "lateral_coefficients_sha256");
    };
    const auto* source_spatial = nested_object(override_object, "spatial_3d");
    shadow_tyre(nested_object(nested_object(override_object, "tires"), "pneumatic_3d"));
    shadow_tyre(nested_object(nested_object(source_spatial, "tires"), "pneumatic_3d"));
    shadow_lateral(explicit_field(source_spatial, "lateral_coefficients"));
}

// ---- leaf decode -------------------------------------------------------------
// Python stores raw members and rejects wrong kinds in __post_init__; the
// typed port cannot hold a wrong kind, so the equivalent kind diagnostics
// fire here, naming the member path.

[[noreturn]] void leaf_error(std::string_view path,
                             std::string_view expected) {
    bike::io::detail::kind_error("bike::configuration::construct", path,
                                 expected);
}

double leaf_real(const Value& value, const std::string& path) {
    if (const auto* number = std::get_if<Number>(&value.data)) {
        bool ok = false;
        const double result =
            bike::io::detail::float_from_lexeme(number->lexeme, ok);
        if (!ok) leaf_error(path, "real number");
        return result;
    }
    leaf_error(path, "real number");
}

int leaf_int(const Value& value, const std::string& path) {
    if (const auto* number = std::get_if<Number>(&value.data)) {
        if (!bike::io::is_integer_lexeme(number->lexeme))
            leaf_error(path, "integer");
        long long parsed = 0;
        const auto* end = std::to_address(number->lexeme.cend());
        const auto rc =
            std::from_chars(number->lexeme.data(), end, parsed, 10);
        if (rc.ec == std::errc::invalid_argument || rc.ptr != end)
            leaf_error(path, "integer");
        // py_int saturation: out-of-range magnitudes clamp so downstream
        // bound comparisons keep every Python outcome.
        if (rc.ec == std::errc::result_out_of_range)
            return number->lexeme.front() == '-'
                       ? std::numeric_limits<int>::min()
                       : std::numeric_limits<int>::max();
        if (parsed > std::numeric_limits<int>::max())
            return std::numeric_limits<int>::max();
        if (parsed < std::numeric_limits<int>::min())
            return std::numeric_limits<int>::min();
        return static_cast<int>(parsed);
    }
    leaf_error(path, "integer");
}

bool leaf_bool(const Value& value, const std::string& path) {
    if (const auto* flag = std::get_if<bool>(&value.data)) return *flag;
    leaf_error(path, "boolean");
}

std::string leaf_text(const Value& value, const std::string& path) {
    if (const auto* text = std::get_if<std::string>(&value.data)) return *text;
    leaf_error(path, "string");
}

// NOLINTNEXTLINE(misc-no-recursion) nested arrays recurse through elements
std::vector<double> leaf_real_vector(const Value& value,
                                     const std::string& path) {
    if (!std::holds_alternative<Value::Array>(value.data))
        leaf_error(path, "real sequence");
    const auto& items = std::get<Value::Array>(value.data);
    std::vector<double> out;
    out.reserve(items.size());
    for (std::size_t i = 0; i < items.size(); ++i)
        out.push_back(
            leaf_real(items.at(i), path + "[" + std::to_string(i) + "]"));
    return out;
}

std::vector<int> leaf_int_vector(const Value& value,
                                 const std::string& path) {
    if (!std::holds_alternative<Value::Array>(value.data))
        leaf_error(path, "integer sequence");
    const auto& items = std::get<Value::Array>(value.data);
    std::vector<int> out;
    out.reserve(items.size());
    for (std::size_t i = 0; i < items.size(); ++i)
        out.push_back(
            leaf_int(items.at(i), path + "[" + std::to_string(i) + "]"));
    return out;
}

template <std::size_t N>
std::array<double, N> leaf_real_array(const Value& value,
                                      const std::string& path) {
    if (!std::holds_alternative<Value::Array>(value.data))
        leaf_error(path, std::to_string(N) + "-element real sequence");
    const auto& items = std::get<Value::Array>(value.data);
    if (items.size() != N)
        leaf_error(path, std::to_string(N) + "-element real sequence");
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i)
        out[i] =
            leaf_real(items.at(i), path + "[" + std::to_string(i) + "]");
    return out;
}

std::vector<std::array<double, 2>>
leaf_pair_table(const Value& value, const std::string& path) {
    if (!std::holds_alternative<Value::Array>(value.data))
        leaf_error(path, "point table");
    const auto& rows = std::get<Value::Array>(value.data);
    std::vector<std::array<double, 2>> out;
    out.reserve(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i)
        out.push_back(leaf_real_array<2>(
            rows.at(i), path + "[" + std::to_string(i) + "]"));
    return out;
}

// ---- record decode -----------------------------------------------------------

void construct_record(const RecordSchema& schema, const Value::Object& obj,
                      void* out, const std::string& path);

void assign_field(const FieldDecl& field, const Value& value, void* base,
                  const std::string& path) {
    void* member = member_addr(field, base);
    switch (field.kind) {
    case FieldKind::Real:
        *static_cast<double*>(member) = leaf_real(value, path);
        return;
    case FieldKind::Integer:
        *static_cast<int*>(member) = leaf_int(value, path);
        return;
    case FieldKind::Boolean:
        *static_cast<bool*>(member) = leaf_bool(value, path);
        return;
    case FieldKind::Text:
        *static_cast<std::string*>(member) = leaf_text(value, path);
        return;
    case FieldKind::OptionalReal: {
        auto& opt = *static_cast<std::optional<double>*>(member);
        if (io::is_null(value)) {
            opt.reset();
        } else {
            opt = leaf_real(value, path);
        }
        return;
    }
    case FieldKind::OptionalBoolean: {
        auto& opt = *static_cast<std::optional<bool>*>(member);
        if (io::is_null(value)) {
            opt.reset();
        } else {
            opt = leaf_bool(value, path);
        }
        return;
    }
    case FieldKind::OptionalText: {
        auto& opt = *static_cast<std::optional<std::string>*>(member);
        if (io::is_null(value)) {
            opt.reset();
        } else {
            opt = leaf_text(value, path);
        }
        return;
    }
    case FieldKind::RealTuple:
        *static_cast<std::vector<double>*>(member) =
            leaf_real_vector(value, path);
        return;
    case FieldKind::IntegerTuple:
        *static_cast<std::vector<int>*>(member) =
            leaf_int_vector(value, path);
        return;
    case FieldKind::Real2Tuple:
        *static_cast<std::array<double, 2>*>(member) =
            leaf_real_array<2>(value, path);
        return;
    case FieldKind::Real3Tuple:
        *static_cast<std::array<double, 3>*>(member) =
            leaf_real_array<3>(value, path);
        return;
    case FieldKind::OptionalReal2Tuple: {
        auto& opt =
            *static_cast<std::optional<std::array<double, 2>>*>(member);
        if (io::is_null(value)) {
            opt.reset();
        } else {
            opt = leaf_real_array<2>(value, path);
        }
        return;
    }
    case FieldKind::OptionalPairTuple: {
        auto& opt = *static_cast<
            std::optional<std::vector<std::array<double, 2>>>*>(member);
        if (io::is_null(value)) {
            opt.reset();
        } else {
            opt = leaf_pair_table(value, path);
        }
        return;
    }
    case FieldKind::UnionMaterial:
    case FieldKind::Dataclass:
    case FieldKind::OptionalDataclass:
        break;  // child kinds decode in assign_child, before this pass
    }
    leaf_error(path, "non-child field kind");
}

// EXTENDS_DEFAULT — resolution.py line 38: PneumaticTireConfig.front/rear
// merge partial sections onto the declared TyreSpecs defaults.
bool extends_default(const RecordSchema& schema, std::string_view field) {
    return &schema == &detail::kPneumaticTireConfigSchema &&
           (field == "front" || field == "rear");
}

// Child-section construction for Dataclass/OptionalDataclass/UnionMaterial
// members — resolution.py's CHILDREN dispatch, in field order.
// NOLINTNEXTLINE(misc-no-recursion) nested-record descent mirrors _construct
void assign_child(const RecordSchema& schema, const FieldDecl& field,
                  const Value& section, void* base,
                  const std::string& path) {
    void* member = member_addr(field, base);
    switch (field.kind) {
    case FieldKind::Dataclass: {
        const auto* object = std::get_if<Value::Object>(&section.data);
        if (object == nullptr)
            construct_error(std::string(class_name(*field.nested)) +
                            " requires a TOML table");
        Value merged_storage;
        const Value::Object* effective = object;
        if (extends_default(schema, field.name)) {
            // _merge(asdict(_field_default(cls, name)), section): the
            // parent's declared-default member holds that default.
            const Value declared = detail::plain_document(
                detail::as_object(*field.nested, member));
            merged_storage.data = merge_objects(
                std::get<Value::Object>(declared.data), *object);
            effective = &object_at(merged_storage);
        }
        construct_record(*field.nested, *effective, member, path);
        return;
    }
    case FieldKind::OptionalDataclass: {
        if (io::is_null(section)) {
            if (field.nested == &detail::kDistributedWheelMaterialSchema) {
                static_cast<std::optional<DistributedWheelMaterial>*>(
                    member)
                    ->reset();
                return;
            }
            construct_error("unsupported optional child schema");
        }
        const auto* object = std::get_if<Value::Object>(&section.data);
        if (object == nullptr)
            construct_error(std::string(class_name(*field.nested)) +
                            " requires a TOML table");
        if (field.nested == &detail::kDistributedWheelMaterialSchema) {
            DistributedWheelMaterial material{};
            construct_record(detail::kDistributedWheelMaterialSchema,
                             *object, &material, path);
            *static_cast<std::optional<DistributedWheelMaterial>*>(
                member) = std::move(material);
            return;
        }
        construct_error("unsupported optional child schema");
    }
    case FieldKind::UnionMaterial: {
        const auto* object = std::get_if<Value::Object>(&section.data);
        if (object == nullptr)
            // _construct(TireSpec, section) sees a non-dict first.
            construct_error("TireSpec requires a TOML table");
        // `cls is TireSpec and ('deflection_m' in values or
        // 'force_n' in values)` — the material arm promotion.
        const bool tabulated =
            io::find_member(*object, "deflection_m") != nullptr ||
            io::find_member(*object, "force_n") != nullptr;
        auto& material =
            *static_cast<std::variant<TireSpec, TabulatedTireSpec>*>(member);
        if (tabulated) {
            TabulatedTireSpec spec{};
            construct_record(*field.nested2, *object, &spec, path);
            material = std::move(spec);
        } else {
            TireSpec spec{};
            construct_record(*field.nested, *object, &spec, path);
            material = std::move(spec);
        }
        return;
    }
    case FieldKind::Real:
    case FieldKind::Integer:
    case FieldKind::Boolean:
    case FieldKind::Text:
    case FieldKind::OptionalReal:
    case FieldKind::OptionalBoolean:
    case FieldKind::OptionalText:
    case FieldKind::RealTuple:
    case FieldKind::IntegerTuple:
    case FieldKind::Real2Tuple:
    case FieldKind::Real3Tuple:
    case FieldKind::OptionalReal2Tuple:
    case FieldKind::OptionalPairTuple:
        break;  // leaf kinds decode in assign_field, after this pass
    }
    leaf_error(path, "child field kind");
}

// NOLINTNEXTLINE(misc-no-recursion) nested-record descent mirrors _construct
void decode_record(const RecordSchema& schema, const Value::Object& obj,
                   void* base, const std::string& path) {
    const std::string cls(class_name(schema));
    // `names = {f.name for f in fields(cls)}` — unknown members name the
    // class and sort lexically.
    std::vector<std::string> unknown;
    for (const auto& [key, member] : obj) {
        bool known = false;
        for (const FieldDecl& field : schema.fields)
            if (field.name == key) {
                known = true;
                break;
            }
        if (!known) unknown.push_back(key);
    }
    if (!unknown.empty()) {
        std::ranges::sort(unknown);
        std::string joined;
        for (std::size_t i = 0; i < unknown.size(); ++i) {
            if (i != 0) joined += ", ";
            joined += unknown[i];
        }
        construct_error("unknown " + cls + " parameter(s): " + joined);
    }

    const auto field_path = [&](std::string_view name) {
        return path.empty() ? std::string(name) : path + "." + std::string(name);
    };

    // Child sections construct before the parent's kwargs call —
    // `kwargs[name] is not None` gates the None case for every kind.
    for (const FieldDecl& field : schema.fields) {
        if (field.kind != FieldKind::Dataclass &&
            field.kind != FieldKind::OptionalDataclass &&
            field.kind != FieldKind::UnionMaterial)
            continue;
        const Value* section = io::find_member(obj, field.name);
        if (section == nullptr) continue;
        assign_child(schema, field, *section, base,
                     field_path(field.name));
    }

    // cls(**kwargs) -> TypeError on absent required members, wrapped by
    // _construct as 'invalid {cls}: {exc}' — the CPython missing-arguments
    // text in signature order.
    std::vector<std::string> missing;
    for (const FieldDecl& field : schema.fields)
        if (field.required && io::find_member(obj, field.name) == nullptr)
            missing.emplace_back(field.name);
    if (!missing.empty()) {
        std::string message = "invalid " + cls + ": " + cls +
                              ".__init__() missing " +
                              std::to_string(missing.size()) +
                              " required positional argument";
        if (missing.size() != 1) message += 's';
        message += ": " + and_join(missing);
        construct_error(message);
    }

    // Leaf members decode in declaration order — every value already
    // survived the table/unknown/missing checks, matching the kwargs
    // assignment order inside the source constructor call.
    for (const FieldDecl& field : schema.fields) {
        if (field.kind == FieldKind::Dataclass ||
            field.kind == FieldKind::OptionalDataclass ||
            field.kind == FieldKind::UnionMaterial)
            continue;
        const Value* value = io::find_member(obj, field.name);
        if (value == nullptr) continue;
        assign_field(field, *value, base, field_path(field.name));
    }
}

// record-level __post_init__: public post_init for the root record,
// detail::post_init for every nested record (const overloads bind to
// non-const arguments).
void record_post_init(SimulationPhysicsConfig& record) { post_init(record); }
template <typename T> void record_post_init(T& record) {
    detail::post_init(record);
}

// NOLINTBEGIN(misc-no-recursion) nested-record descent mirrors _construct
template <typename T>
void construct_typed(const RecordSchema& schema, const Value::Object& obj,
                     T& out, const std::string& path) {
    T working{};
    decode_record(schema, obj, &working, path);
    record_post_init(working);
    out = std::move(working);
}

// The recursion carries each record's working storage — keep it a real call
// boundary so nested records do not accumulate into one caller's frame
// (-Wframe-larger-than at the strict warning bound).
[[gnu::noinline]] void
construct_record(const RecordSchema& schema, const Value::Object& obj,
                 void* out, const std::string& path) {
#define BIKE_T13_RECORD(Type, Schema)                                         \
    if (&schema == &detail::Schema) {                                         \
        construct_typed<Type>(schema, obj, *static_cast<Type*>(out), path);   \
        return;                                                               \
    }
    BIKE_T13_RECORD(SimulationPhysicsConfig, kSimulationPhysicsConfigSchema)
    BIKE_T13_RECORD(EndStopConfig, kEndStopConfigSchema)
    BIKE_T13_RECORD(TireBackendConfig, kTireBackendConfigSchema)
    BIKE_T13_RECORD(TireParameters, kTireParametersSchema)
    BIKE_T13_RECORD(TireSpec, kTireSpecSchema)
    BIKE_T13_RECORD(TabulatedTireSpec, kTabulatedTireSpecSchema)
    BIKE_T13_RECORD(DistributedTireConfig, kDistributedTireConfigSchema)
    BIKE_T13_RECORD(DistributedWheelMaterial,
                    kDistributedWheelMaterialSchema)
    BIKE_T13_RECORD(HingeDensity, kHingeDensitySchema)
    BIKE_T13_RECORD(PneumaticTireConfig, kPneumaticTireConfigSchema)
    BIKE_T13_RECORD(TyreSpecs, kTyreSpecsSchema)
    BIKE_T13_RECORD(PhysicalDriveConfig, kPhysicalDriveConfigSchema)
    BIKE_T13_RECORD(DrivetrainSpecs, kDrivetrainSpecsSchema)
    BIKE_T13_RECORD(AssistConfig, kAssistConfigSchema)
    BIKE_T13_RECORD(BatteryConfig, kBatteryConfigSchema)
    BIKE_T13_RECORD(PedalingConfig, kPedalingConfigSchema)
    BIKE_T13_RECORD(ShiftingConfig, kShiftingConfigSchema)
    BIKE_T13_RECORD(ResistanceConfig, kResistanceConfigSchema)
    BIKE_T13_RECORD(ArticulatedConfig, kArticulatedConfigSchema)
    BIKE_T13_RECORD(SeatedClimbConfig, kSeatedClimbConfigSchema)
#undef BIKE_T13_RECORD
    construct_error("unsupported configuration schema " +
                    std::string(schema.path));
}
// NOLINTEND(misc-no-recursion)

SimulationPhysicsConfig construct_simulation(const Value& values) {
    const auto* obj = std::get_if<Value::Object>(&values.data);
    if (obj == nullptr)
        construct_error("SimulationPhysicsConfig requires a TOML table");
    SimulationPhysicsConfig config{};
    const Value* tire_value = io::find_member(*obj, "tires");
    const auto* tire_object = tire_value == nullptr
        ? nullptr : std::get_if<Value::Object>(&tire_value->data);
    const bool extension_input =
        io::find_member(*obj, "spatial") != nullptr ||
        io::find_member(*obj, "spatial_3d") != nullptr ||
        (tire_object != nullptr &&
         io::find_member(*tire_object, "pneumatic_3d") != nullptr);
    if (extension_input) {
        // Validate the extension against the complete public input before
        // removing only its owned keys from the unchanged legacy decoder.
        // That decoder still rejects every unknown legacy/nested field.
        auto extension = bike::spatial::parse_spatial_config(values);
        std::optional<std::string> shoulder_path, shoulder_digest;
        Value::Object legacy;
        legacy.reserve(obj->size());
        for (const auto& [key, value] : *obj) {
            if (key == "spatial" || key == "spatial_3d") continue;
            if (key == "resistance" && extension.mode == bike::spatial::Mode::spatial3d) {
                // Spatial vectors are still decoded by the same strict
                // schema, but their Y component belongs to the spatial
                // parent. The legacy child constructor has no mode context.
                const auto* resistance = std::get_if<Value::Object>(&value.data);
                if (resistance == nullptr)
                    construct_error("ResistanceConfig requires a TOML table");
                decode_record(detail::kResistanceConfigSchema, *resistance,
                              &config.resistance, "resistance");
                detail::post_init(config.resistance, true);
            } else if (key == "tires" && tire_object != nullptr) {
                Value::Object tires;
                tires.reserve(tire_object->size());
                for (const auto& [tire_key, tire_member] : *tire_object)
                    if (tire_key != "pneumatic_3d")
                        tires.emplace_back(tire_key, tire_member);
                legacy.emplace_back(key, Value{.data = std::move(tires)});
            } else if (key == "articulated" && std::holds_alternative<Value::Object>(value.data)) {
                Value::Object articulated;
                for (const auto& [rider_key, rider_member] : std::get<Value::Object>(value.data)) {
                    if (rider_key == "shoulder_profile" || rider_key == "shoulder_profile_sha256") {
                        if (extension.mode != bike::spatial::Mode::spatial3d)
                            construct_error("articulated." + rider_key + ": requires spatial='3d'");
                        auto& target = rider_key == "shoulder_profile" ? shoulder_path : shoulder_digest;
                        if (!io::is_null(rider_member))
                            target = io::as_string(rider_member, "articulated." + rider_key);
                    } else articulated.emplace_back(rider_key, rider_member);
                }
                legacy.emplace_back(key, Value{std::move(articulated)});
            } else {
                legacy.emplace_back(key, value);
            }
        }
        decode_record(detail::kSimulationPhysicsConfigSchema, legacy,
                      &config, "");
        if (extension.mode == bike::spatial::Mode::spatial3d)
            config.spatial = std::move(extension);
        config.articulated.shoulder_profile = std::move(shoulder_path);
        config.articulated.shoulder_profile_sha256 = std::move(shoulder_digest);
        detail::post_init(config.articulated);
        post_init(config);
        return config;
    }
    // Keep the legacy construction and its validation order untouched.
    construct_record(detail::kSimulationPhysicsConfigSchema, *obj, &config,
                     "");
    return config;
}

} // namespace

namespace detail {

bike::io::Value parse_toml(std::string_view source, std::string_view origin) {
    try {
        return toml_table(toml::parse(source, origin));
    } catch (const toml::parse_error& error) {
        throw std::invalid_argument(std::string(origin) + ": " +
                                    std::string(error.description()));
    }
}

bike::io::Value merge_values(const bike::io::Value& base,
                             const bike::io::Value& overrides) {
    const auto* base_object = std::get_if<Value::Object>(&base.data);
    const auto* override_object =
        std::get_if<Value::Object>(&overrides.data);
    if (base_object == nullptr || override_object == nullptr)
        construct_error("configuration and overrides must be mappings");
    return Value{.data = merge_objects(*base_object, *override_object)};
}

// NOLINTNEXTLINE(misc-no-recursion) mirrors the document tree
bike::io::Value plain_document(const bike::io::Value& encoded) {
    if (const auto* object = std::get_if<Value::Object>(&encoded.data)) {
        // encode_tree leaves: {"$tuple": [...]} and {"$nonfinite": "..."}.
        if (object->size() == 1) {
            const auto& [key, member] = object->front();
            if (key == "$tuple") return plain_document(member);
            if (key == "$nonfinite") {
                if (const auto* spelling =
                        std::get_if<std::string>(&member.data))
                    return Value{.data = Number{*spelling}};
                return member;
            }
        }
        Value::Object out;
        out.reserve(object->size());
        for (const auto& [key, member] : *object)
            out.emplace_back(key, plain_document(member));
        return Value{.data = std::move(out)};
    }
    if (const auto* array = std::get_if<Value::Array>(&encoded.data)) {
        Value::Array out;
        out.reserve(array->size());
        for (const Value& member : *array)
            out.push_back(plain_document(member));
        return Value{.data = std::move(out)};
    }
    return encoded;
}

SimulationPhysicsConfig construct_config(const bike::io::Value& values) {
    return construct_simulation(values);
}

} // namespace detail

SimulationPhysicsConfig
resolve_physics(const bike::io::Value& overrides,
                const SimulationPhysicsConfig& base) {
    // resolve_physics_config(asdict(base), overrides): plain() strips the
    // encode_tree tags so the merge sees the same document shape the TOML
    // decode produces; overrides pass through plain() for symmetry (a
    // to_value-derived override document is a supported spelling).
    Value base_doc = detail::plain_document(to_value(base));
    const Value override_doc = detail::plain_document(overrides);
    prepare_spatial_overrides(base_doc, override_doc);
    return construct_simulation(
        detail::merge_values(base_doc, override_doc));
}

SimulationPhysicsConfig load_physics(const std::filesystem::path& file) {
    Value values = detail::parse_toml_file(file);
    detail::resolve_config_paths(values, file.parent_path());
    return construct_simulation(values);
}

} // namespace bike::configuration
