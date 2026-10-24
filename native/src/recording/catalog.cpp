#include "catalog.hpp"
#include "versioned.hpp"
#include "value_util.hpp"
#include <algorithm>
#include <array>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bike::recording {
namespace {
#include "catalog_data.inc"
#include "catalog_v2_data.inc"
using io::Value;
std::vector<std::string> read_strings(const Value& value) {
    std::vector<std::string> out;
    for (const auto& x : io::as_array(value, "catalog strings")) out.push_back(io::as_string(x, "catalog string"));
    return out;
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
FieldSpec read_field(const Value& document) {
    const auto& d = io::as_object(document, "catalog field");
    auto const string = [&](std::string_view key) { return io::as_string(value::require(d, key), key); };
    FieldSpec out;
    out.key = string("key"); out.dtype = string("dtype"); out.unit = string("unit"); out.phase = string("phase");
    out.optional = io::as_bool(value::require(d, "optional"), "optional");
    out.measurement = string("measurement"); out.source = string("source"); out.description = string("description");
    out.frame = string("frame"); out.path = read_strings(value::require(d, "path"));
    if (out.path.empty()) out.path.push_back(out.key);
    out.aliases = read_strings(value::require(d, "aliases"));
    out.except_keys = read_strings(value::require(d, "except_keys"));
    const auto& shape = value::require(d, "shape");
    if (!io::is_null(shape)) {
        out.shape.emplace();
        for (const auto& n : io::as_array(shape, "shape")) out.shape->push_back(io::as_int64(n, "shape"));
    }
    const auto& axes = value::require(d, "axes");
    if (!io::is_null(axes)) out.axes = read_strings(axes);
    for (const auto& child : io::as_array(value::require(d, "children"), "children")) out.children.push_back(read_field(child));
    out.validate(); return out;
}
TableSpec read_table(const Value& document) {
    const auto& d = io::as_object(document, "catalog table");
    TableSpec out;
    out.name = io::as_string(value::require(d, "name"), "name");
    out.kind = io::as_string(value::require(d, "kind"), "kind");
    out.description = io::as_string(value::require(d, "description"), "description");
    out.source = io::as_string(value::require(d, "source"), "source");
    for (const auto& f : io::as_array(value::require(d, "keys"), "keys")) out.keys.push_back(read_field(f));
    for (const auto& f : io::as_array(value::require(d, "fields"), "fields")) out.fields.push_back(read_field(f));
    out.claims = read_strings(value::require(d, "claims"));
    out.exempt_keys = read_strings(value::require(d, "exempt_keys"));
    out.validate(); return out;
}
struct Descriptors { std::vector<TableSpec> physical, research; };
const Descriptors& descriptors() {
    static const Descriptors out = [] {
        std::string source;
        for (const auto part : kCatalogDocumentParts) source += part;
        const auto document = io::parse_json(source);
        if (io::as_int64(value::require(document, "schema"), "catalog.schema") != schema_version)
            throw std::invalid_argument("recording catalog schema differs");
        Descriptors result;
        for (const auto& table : io::as_array(value::require(document, "tables"), "tables")) result.physical.push_back(read_table(table));
        for (const auto& table : io::as_array(value::require(document, "research_tables"), "research_tables")) result.research.push_back(read_table(table));
        return result;
    }();
    return out;
}

const Descriptors& spatial_descriptors() {
    static const Descriptors out = [] {
        std::string source; for (const auto part : kCatalogV2DocumentParts) source += part;
        const auto document = io::parse_json(source);
        if (io::as_int64(value::require(document, "schema"), "catalog.schema") != 2)
            throw std::invalid_argument("recording catalog schema differs from spatial2");
        Descriptors result;
        for (const auto& t : io::as_array(value::require(document, "tables"), "tables")) result.physical.push_back(read_table(t));
        for (const auto& t : io::as_array(value::require(document, "research_tables"), "research_tables")) result.research.push_back(read_table(t));
        return result;
    }(); return out;
}

// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
std::optional<AliasSpec> walk_struct(const FieldSpec& field, std::span<const std::string> rest,
                                    const std::string& table, std::string_view name) {
    if (rest.empty()) return {};
    if (field.dtype == "map<struct>") {
        if (rest.size() != 2) return {};
        for (const auto& child : field.children) if (child.key == rest[1])
            return AliasSpec{.alias=std::string(name), .canonical=table + "." + field.key + "." + rest[0] + "." + child.key, .index={}, .parameter=rest[0], .note="flattened schema-2 channel leaf"};
        return {};
    }
    for (const auto& child : field.children) if (child.key == rest[0]) {
        if (rest.size() == 1) return AliasSpec{.alias=std::string(name), .canonical=table + "." + field.key + "." + child.key, .index={}, .parameter={}, .note="flattened schema-2 channel leaf"};
        if (child.dtype == "struct" || child.dtype == "map<struct>") return walk_struct(child, rest.subspan(1), table + "." + field.key, name);
    }
    return {};
}
} // namespace

// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void FieldSpec::validate() const {
    static const std::set<std::string, std::less<>> types{"bigint", "double", "boolean", "varchar", "bigint[]", "double[]", "varchar[]", "map<bigint>", "map<double>", "map<boolean>", "map<varchar>", "map<double[]>", "struct", "struct[]", "map<struct>"};
    static const std::set<std::string, std::less<>> phases{"interval_start", "interval", "interval_end", "control_period", "episode", "n/a"};
    static const std::set<std::string, std::less<>> measurements{"measured", "forwarded", "solved", "derived", "declared", "diagnostic"};
    if (key.empty() || !types.contains(dtype) || !phases.contains(phase) || !measurements.contains(measurement) || unit.empty() || source.empty() || description.empty())
        throw std::invalid_argument("recording catalog: invalid field " + key);
    if (shape && axes && (*shape != std::vector<std::int64_t>{static_cast<std::int64_t>(axes->size())}))
        throw std::invalid_argument(key + ": shape/axes mismatch");
    if (shape && std::ranges::any_of(*shape, [](auto n) { return n < 0; })) throw std::invalid_argument(key + ": negative shape");
    if ((dtype == "struct" || dtype == "struct[]" || dtype == "map<struct>") && children.empty()) throw std::invalid_argument(key + ": structured dtype needs children");
    std::set<std::string> names;
    for (const auto& child : children) {
        child.validate();
        if (!names.insert(child.key).second || (child.optional && !names.insert(child.key + "_present").second))
            throw std::invalid_argument(key + ": duplicate struct member");
    }
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
std::string FieldSpec::duckdb_type() const {
    static const std::map<std::string, std::string, std::less<>> scalar{
        {"bigint", "BIGINT"}, {"double", "DOUBLE"}, {"boolean", "BOOLEAN"}, {"varchar", "VARCHAR"},
        {"bigint[]", "BIGINT[]"}, {"double[]", "DOUBLE[]"}, {"varchar[]", "VARCHAR[]"},
        {"map<bigint>", "MAP(VARCHAR, BIGINT)"}, {"map<double>", "MAP(VARCHAR, DOUBLE)"},
        {"map<boolean>", "MAP(VARCHAR, BOOLEAN)"}, {"map<varchar>", "MAP(VARCHAR, VARCHAR)"}, {"map<double[]>", "MAP(VARCHAR, DOUBLE[])"}};
    if (auto const it = scalar.find(dtype); it != scalar.end()) return it->second;
    std::string out = "STRUCT(";
    for (const auto& child : children) {
        if (out != "STRUCT(") out += ", ";
        out += quote(child.key) + " " + child.duckdb_type();
        if (child.optional) out += ", " + quote(child.key + "_present") + " BOOLEAN";
    }
    out += ')';
    if (dtype == "struct") return out;
    if (dtype == "struct[]") return out + "[]";
    if (dtype == "map<struct>") return "MAP(VARCHAR, " + out + ")";
    throw std::invalid_argument(key + ": unsupported dtype " + dtype);
}
void TableSpec::validate() const {
    if (name.empty() || (kind != "physics" && kind != "forces" && kind != "channels")) throw std::invalid_argument("invalid recording table " + name);
    if (kind == "channels" && claims.empty()) throw std::invalid_argument(name + ": channel table must claim roots");
    std::set<std::string> names;
    auto const add = [&](const FieldSpec& f) {
        f.validate();
        if (!names.insert(f.key).second || (f.optional && !names.insert(f.key + "_present").second)) throw std::invalid_argument(name + ": duplicate field key");
    };
    for (const auto& key : keys) add(key);
    std::set<std::vector<std::string>> paths;
    for (const auto& field : fields) { add(field); if (!paths.insert(field.path).second) throw std::invalid_argument(name + ": duplicate field path"); }
    for (const auto& f : fields) for (const auto& path : paths) if (path.size() > f.path.size() && std::equal(f.path.begin(), f.path.end(), path.begin()) &&
        !f.dtype.starts_with("map<") && f.dtype != "struct" && f.dtype != "struct[]") throw std::invalid_argument(name + ": shared prefix must be a container");
    for (const auto& mask : parent_masks(*this)) if (!names.insert(mask).second) throw std::invalid_argument(name + ": parent mask collision");
}
const FieldSpec* TableSpec::field(std::string_view key) const noexcept {
    for (const auto& f : keys) if (f.key == key) return &f;
    for (const auto& f : fields) if (f.key == key) return &f;
    return nullptr;
}
Catalog::Catalog(std::vector<TableSpec> tables, CoordinateCatalog coordinates, std::vector<AliasSpec> aliases)
    : tables_(std::move(tables)), coordinates_(std::move(coordinates)), aliases_(std::move(aliases)) {
    std::set<std::string> names;
    for (const auto& t : tables_) {
        t.validate();
        if (!names.insert(t.name).second) throw std::invalid_argument("duplicate recording table");
        for (const auto& root : t.claims) if (!roots_.emplace(root, t.name).second) throw std::invalid_argument("recording channel root claimed twice: " + root);
    }
}
const TableSpec& Catalog::table(std::string_view name) const {
    for (const auto& t : tables_) if (t.name == name) return t;
    throw std::invalid_argument("unknown recording table: " + std::string(name));
}
std::vector<std::string> Catalog::channel_roots() const { std::vector<std::string> out; for (const auto& [root, table] : roots_) { (void)table; out.push_back(root); } return out; }
const TableSpec& Catalog::channel_table(std::string_view root) const {
    auto const it = roots_.find(root); if (it == roots_.end()) throw std::invalid_argument("unknown recording channel root: " + std::string(root)); return table(it->second);
}
std::optional<AliasSpec> Catalog::resolve_alias(std::string_view name) const {
    for (const auto& a : aliases_) if (a.alias == name) return a;
    for (const auto& a : aliases_) {
        auto const pos = a.alias.find("{component}"); if (pos == std::string::npos) continue;
        const auto prefix = std::string_view(a.alias).substr(0, pos), suffix = std::string_view(a.alias).substr(pos + 11);
        if (name.size() > prefix.size() + suffix.size() && name.starts_with(prefix) && name.ends_with(suffix)) {
            auto out = a; out.alias = name; out.parameter = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size()); return out;
        }
    }
    std::vector<std::string> path;
    for (std::size_t start = 0; start <= name.size();) {
        auto const end = name.find('.', start); path.emplace_back(name.substr(start, end == std::string_view::npos ? end : end - start));
        if (end == std::string_view::npos) break;
        start = end + 1;
    }
    if (path.size() < 2 || !roots_.contains(path.front())) return {};
    const auto& spec = channel_table(path.front());
    for (const auto& f : spec.fields) {
        if (f.path == path) return AliasSpec{.alias=std::string(name), .canonical=spec.name + "." + f.key, .index={}, .parameter={}, .note="flattened schema-2 channel leaf"};
        if (path.size() <= f.path.size() || !std::equal(f.path.begin(), f.path.end(), path.begin())) continue;
        const auto rest = std::span<const std::string>(path).subspan(f.path.size());
        if (f.dtype.starts_with("map<") && rest.size() == 1) return AliasSpec{.alias=std::string(name), .canonical=spec.name + "." + f.key, .index={}, .parameter=rest.front(), .note="flattened schema-2 map member"};
        if (f.dtype == "struct" || f.dtype == "struct[]" || f.dtype == "map<struct>") if (auto out = walk_struct(f, rest, spec.name, name)) return out;
    }
    return {};
}
const Catalog& schema_catalog() { static const Catalog out(descriptors().physical, {}, {}); return out; }
const std::vector<TableSpec>& research_tables() { return descriptors().research; }
const std::vector<TableSpec>& all_tables() { static const auto out = [] { auto result = descriptors().physical; result.insert(result.end(), descriptors().research.begin(), descriptors().research.end()); return result; }(); return out; }

const Catalog& schema_catalog(CatalogVersion version) {
    if (version == CatalogVersion::planar1) return schema_catalog();
    if (version != CatalogVersion::spatial2) throw std::invalid_argument("unsupported recording catalog version");
    static const Catalog out(spatial_descriptors().physical, {}, {}); return out;
}
std::span<const TableDescriptor> all_tables(CatalogVersion version) {
    if (version == CatalogVersion::planar1) return all_tables();
    if (version != CatalogVersion::spatial2) throw std::invalid_argument("unsupported recording catalog version");
    static const auto out = [] { auto result = spatial_descriptors().physical;
        result.insert(result.end(), spatial_descriptors().research.begin(), spatial_descriptors().research.end()); return result; }();
    return out;
}

CatalogVersion catalog_version(const Catalog& catalog) noexcept {
    for (const auto& t : catalog.table_order()) if (t.name == "channel_spatial") return CatalogVersion::spatial2;
    return CatalogVersion::planar1;
}
const TableSpec& table_spec(std::string_view name, CatalogVersion version) {
    for (const auto& spec : all_tables(version)) if (spec.name == name) return spec;
    throw std::invalid_argument("unreviewed recording table: " + std::string(name));
}
const TableSpec& table_spec(std::string_view name) { for (const auto& spec : all_tables()) if (spec.name == name) return spec; throw std::invalid_argument("unreviewed recording table: " + std::string(name)); }

// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
io::Value describe_field(const FieldSpec& f) {
    Value::Array children; for (const auto& child : f.children) children.push_back(describe_field(child));
    Value shape; if (f.shape) { Value::Array a; for (auto const n : *f.shape) a.push_back(value::integer(n)); shape.data = std::move(a); }
    return Value{Value::Object{{"key", value::text(f.key)}, {"dtype", value::text(f.dtype)}, {"unit", value::text(f.unit)},
        {"phase", value::text(f.phase)}, {"optional", Value{f.optional}}, {"measurement", value::text(f.measurement)},
        {"source", value::text(f.source)}, {"description", value::text(f.description)}, {"frame", value::text(f.frame)},
        {"path", value::strings(f.path)}, {"aliases", value::strings(f.aliases)}, {"shape", std::move(shape)},
        {"axes", f.axes ? value::strings(*f.axes) : Value{}}, {"except_keys", value::strings(f.except_keys)}, {"children", Value{std::move(children)}}}};
}
io::Value describe_catalog(const Catalog& catalog) {
    auto const coordinates = [](const std::vector<Coordinate>& cs) {
        Value::Array out;
        for (const auto& c : cs) out.push_back(Value{Value::Object{{"index", value::integer(c.index)}, {"kind", value::text(c.kind)}, {"joint", value::text(c.joint)}, {"component", value::text(c.component)}, {"dof_kind", value::text(c.dof_kind)}, {"unit", value::text(c.unit)}}});
        return Value{std::move(out)};
    };
    Value::Array aliases;
    for (const auto& a : catalog.aliases()) aliases.push_back(Value{Value::Object{{"alias", value::text(a.alias)}, {"canonical", value::text(a.canonical)}, {"index", a.index ? value::integer(*a.index) : Value{}}, {"parameter", a.parameter ? value::text(*a.parameter) : Value{}}, {"note", value::text(a.note)}}});
    Value::Array tables; for (const auto& t : catalog.table_order()) tables.push_back(value::text(t.name));
    return Value{Value::Object{{"aliases", Value{std::move(aliases)}}, {"channel_roots", value::strings(catalog.channel_roots())},
        {"coordinates", Value{Value::Object{{"qpos", coordinates(catalog.coordinates().qpos)}, {"dof", coordinates(catalog.coordinates().dof)}}}}, {"tables", Value{std::move(tables)}}}};
}
} // namespace bike::recording
