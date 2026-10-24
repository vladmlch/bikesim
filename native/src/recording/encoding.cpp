#include "encoding.hpp"
#include "value_util.hpp"
#include <algorithm>
#include <limits>
#include <ranges>
#include <set>
#include <utility>

namespace bike::recording {
namespace {
using runtime::Piece;
using io::Value;
using Object = Value::Object;
using Array = Value::Array;
[[noreturn]] void invalid(std::string_view path, std::string_view why) { throw std::invalid_argument("recording " + std::string(path) + ": " + std::string(why)); }
bool contains(std::span<const std::string> values, std::string_view value) { return std::ranges::any_of(values, [&](const auto& x) { return x == value; }); }
FieldSpec boolean_field(std::string name) { FieldSpec f; f.key = std::move(name); f.dtype = "boolean"; return f; }
FieldSpec item_field(const FieldSpec& field) {
    auto out = field;
    if (field.dtype == "struct[]" || field.dtype == "map<struct>") out.dtype = "struct";
    else if (field.dtype.starts_with("map<")) out.dtype = field.dtype.substr(4, field.dtype.size() - 5);
    else if (field.dtype.ends_with("[]")) out.dtype.resize(out.dtype.size() - 2);
    return out;
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
Piece make_piece(const FieldSpec& f) {
    Piece out;
    if (f.dtype == "double") out.kind = Piece::Kind::scalar_d;
    else if (f.dtype == "bigint") out.kind = Piece::Kind::scalar_i;
    else if (f.dtype == "boolean") out.kind = Piece::Kind::scalar_b;
    else if (f.dtype == "varchar") out.kind = Piece::Kind::scalar_s;
    else if (f.dtype == "double[]") { out.kind = Piece::Kind::list_d; out.offsets.push_back(0); }
    else if (f.dtype == "bigint[]") { out.kind = Piece::Kind::list_i; out.offsets.push_back(0); }
    else if (f.dtype == "varchar[]") { out.kind = Piece::Kind::list_s; out.offsets.push_back(0); }
    else if (f.dtype == "struct" || f.dtype == "struct[]") {
        out.kind = Piece::Kind::structure;
        if (f.dtype == "struct[]") out.offsets.push_back(0);
        for (const auto& child : f.children) {
            out.children.emplace(child.key, make_piece(child));
            if (child.optional) out.children.emplace(child.key + "_present", make_piece(boolean_field(child.key + "_present")));
        }
    } else if (f.dtype.starts_with("map<")) {
        out.kind = Piece::Kind::map; out.offsets.push_back(0);
        out.items = std::make_unique<Piece>(make_piece(item_field(f)));
    } else invalid(f.key, "unsupported dtype");
    return out;
}
std::int32_t offset(std::size_t n) {
    if (n > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) throw std::overflow_error("recording piece offset exceeds int32");
    return static_cast<std::int32_t>(n);
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void append_null(const FieldSpec& f, Piece& p) {
    p.valid.push_back(0);
    switch (p.kind) {
        case Piece::Kind::scalar_d: p.d.push_back(0.); break;
        case Piece::Kind::scalar_i: p.i.push_back(0); break;
        case Piece::Kind::scalar_b: p.b.push_back(0); break;
        case Piece::Kind::scalar_s: p.s.emplace_back(); break;
        case Piece::Kind::list_d: p.offsets.push_back(offset(p.d.size())); break;
        case Piece::Kind::list_i: p.offsets.push_back(offset(p.i.size())); break;
        case Piece::Kind::list_s: case Piece::Kind::map: p.offsets.push_back(offset(p.s.size())); break;
        case Piece::Kind::structure:
            if (!p.offsets.empty()) p.offsets.push_back(p.offsets.back());
            else for (const auto& c : f.children) {
                append_null(c, p.children.at(c.key));
                if (c.optional) p.children.at(c.key + "_present").b.push_back(0);
            }
            break;
    }
}
void append(const FieldSpec&, Piece&, const Value&, std::string_view);
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void append_struct(const FieldSpec& f, Piece& p, const Value& input, std::string_view path) {
    const auto& object = io::as_object(input, path);
    std::set<std::string> names;
    for (const auto& [key, v] : object) {
        (void)v;
        if (!names.insert(key).second || !std::ranges::any_of(f.children, [&](const auto& c) { return c.key == key; })) invalid(path, "unknown or duplicate struct member " + key);
    }
    for (const auto& child : f.children) {
        const auto* v = io::find_member(object, child.key);
        if (child.optional) p.children.at(child.key + "_present").b.push_back(v ? 1 : 0);
        if (!v || io::is_null(*v)) {
            if (!child.optional) invalid(std::string(path) + "." + child.key, "required");
            append_null(child, p.children.at(child.key));
        } else append(child, p.children.at(child.key), *v, std::string(path) + "." + child.key);
    }
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void append(const FieldSpec& f, Piece& p, const Value& input, std::string_view path) {
    if (io::is_null(input)) { append_null(f, p); return; }
    p.valid.push_back(1);
    switch (p.kind) {
        case Piece::Kind::scalar_d: p.d.push_back(io::tagged_double(input, path)); break;
        case Piece::Kind::scalar_i: p.i.push_back(io::as_int64(input, path)); break;
        case Piece::Kind::scalar_b: p.b.push_back(io::as_bool(input, path) ? 1 : 0); break;
        case Piece::Kind::scalar_s: p.s.push_back(io::as_string(input, path)); break;
        case Piece::Kind::list_d: case Piece::Kind::list_i: case Piece::Kind::list_s: {
            const auto& a = io::as_array(input, path);
            if (f.shape && (f.shape->empty() || a.size() != static_cast<std::size_t>(f.shape->front()))) invalid(path, "fixed array shape differs");
            for (const auto& cell : a) {
                if (p.kind == Piece::Kind::list_d) p.d.push_back(io::tagged_double(cell, path));
                else if (p.kind == Piece::Kind::list_i) p.i.push_back(io::as_int64(cell, path));
                else p.s.push_back(io::as_string(cell, path));
            }
            p.offsets.push_back(offset(p.kind == Piece::Kind::list_d ? p.d.size() : p.kind == Piece::Kind::list_i ? p.i.size() : p.s.size()));
            break;
        }
        case Piece::Kind::structure:
            if (f.dtype == "struct[]") {
                const auto& a = io::as_array(input, path);
                for (const auto& cell : a) append_struct(f, p, cell, path);
                const auto old = static_cast<std::size_t>(p.offsets.back());
                if (a.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) - old) invalid(path, "struct list offset overflow");
                p.offsets.push_back(offset(old + a.size()));
            } else append_struct(f, p, input, path);
            break;
        case Piece::Kind::map: {
            const auto& entries = io::as_object(input, path);
            std::set<std::string> keys;
            const auto item = item_field(f);
            for (const auto& [key, v] : entries) {
                if (key.empty() || !keys.insert(key).second) invalid(path, "map keys must be distinct nonempty strings");
                if (io::is_null(v)) invalid(path, "null map value is not a reviewed source value");
                p.s.push_back(key); append(item, *p.items, v, std::string(path) + "[" + key + "]");
            }
            p.offsets.push_back(offset(p.s.size())); break;
        }
    }
}
const FieldSpec* terminal(const TableSpec& spec, const std::vector<std::string>& path) {
    for (const auto& f : spec.fields) if (f.path == path) return &f;
    return nullptr;
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void distribute(const TableSpec& spec, const std::vector<std::string>& prefix, const Value& payload, Object& out) {
    const auto* leaf = terminal(spec, prefix);
    if (io::is_null(payload) && !leaf) return;
    const auto* object = std::get_if<Object>(&payload.data);
    if (!object) {
        if (!leaf) invalid(spec.name + "." + value::join(prefix), "no reviewed field accepts value");
        value::set(out, leaf->key, payload); return;
    }
    Object remainder;
    std::set<std::string> members;
    for (const auto& [key, v] : *object) {
        if(!members.insert(key).second)invalid(spec.name+"."+value::join(prefix),"duplicate channel member "+key);
        const bool deeper = std::ranges::any_of(spec.fields, [&](const auto& f) { return f.path.size() > prefix.size() && std::equal(prefix.begin(), prefix.end(), f.path.begin()) && f.path[prefix.size()] == key; });
        if (!deeper) { remainder.emplace_back(key, v); continue; }
        auto path = prefix; path.push_back(key);
        if (const auto* child = terminal(spec, path)) value::set(out, child->key, v);
        else if (io::is_null(v) || std::holds_alternative<Object>(v.data)) distribute(spec, path, v, out);
        else invalid(spec.name + "." + value::join(path), "no reviewed field accepts value");
    }
    if (!remainder.empty()) {
        if (!leaf || (!leaf->dtype.starts_with("map<") && leaf->dtype != "struct" && leaf->dtype != "struct[]")) invalid(spec.name + "." + value::join(prefix), "unknown channel members");
        value::set(out, leaf->key, Value{std::move(remainder)});
    }
}
Object project(const TableSpec& spec, const Object& row) {
    Object out; std::set<std::string> names;
    for (const auto& [key, v] : row) {
        if (!names.insert(key).second) invalid(spec.name, "duplicate source field");
        const bool identity = std::ranges::any_of(spec.keys, [&](const auto& f) { return f.key == key; });
        if (spec.kind == "channels") {
            if (identity || contains(spec.exempt_keys, key)) out.emplace_back(key, v);
            else if (contains(spec.claims, key)) distribute(spec, {key}, v, out);
            else invalid(spec.name, "unknown channel root " + key);
        } else {
            if (contains(spec.exempt_keys, key)) continue;
            if (identity) { out.emplace_back(key, v); continue; }
            const auto it = std::ranges::find_if(spec.fields, [&](const auto& f) { return !f.path.empty() && f.path.front() == key; });
            if (it == spec.fields.end()) invalid(spec.name, "unknown source field " + key);
            // invalid() is [[noreturn]] — reaching this line proves it != end().
            // cppcheck-suppress derefInvalidIteratorRedundantCheck
            out.emplace_back(it->key, v);
        }
    }
    return out;
}
const Value* at_path(const Object& row, std::span<const std::string> path) {
    const Object* object = &row; const Value* current = nullptr;
    for (std::size_t i = 0; i < path.size(); ++i) {
        current = io::find_member(*object, path[i]);
        if (!current) return nullptr;
        if (i + 1 != path.size()) { object = std::get_if<Object>(&current->data); if (!object) return nullptr; }
    }
    return current;
}
bool valid_at(const Piece& p, std::size_t row) { return p.valid.empty() || p.valid.at(row) != 0; }
void lane(const Piece& p, std::size_t count, std::string_view path) {
    if (!p.valid.empty() && p.valid.size() != count) invalid(path, "validity length differs");
    if (!std::ranges::all_of(p.valid, [](char v) { return v == 0 || v == 1; })) invalid(path, "invalid validity byte");
}
std::size_t offsets(const Piece& p, std::size_t count, std::string_view path) {
    if (p.offsets.size() != count + 1 || p.offsets.empty() || p.offsets.front() != 0) invalid(path, "offset vector shape/origin differs");
    for (std::size_t i = 0; i < count; ++i) if (p.offsets[i] < 0 || p.offsets[i] > p.offsets[i+1] || (!valid_at(p, i) && p.offsets[i] != p.offsets[i+1])) invalid(path, "invalid list offsets or nonempty null slot");
    return static_cast<std::size_t>(p.offsets.back());
}
// `active` carries ancestor-struct presence into leaf checks: a NULL struct
// slot leaves every child at its default-valid Arrow/DuckDB placeholder, so
// child payload (validity, list width, presence-vs-null consistency) is
// padding and only the child's own _present flag is a semantic claim.
bool row_active(std::span<const char> active, std::size_t row) {
    return active.empty() || active[row] != 0;
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
void validate_node(const FieldSpec& f, const Piece& p, std::size_t rows, std::string_view path,
                   std::span<const char> active = {}) {
    lane(p, rows, path);
    const auto kind = make_piece(f).kind;
    if (p.kind != kind) invalid(path, "piece kind differs from reviewed type");
    if (f.dtype == "double" && p.d.size() != rows) invalid(path, "double lane length differs");
    else if (f.dtype == "bigint" && p.i.size() != rows) invalid(path, "integer lane length differs");
    else if (f.dtype == "boolean" && (p.b.size() != rows || !std::ranges::all_of(p.b, [](char v) { return v == 0 || v == 1; }))) invalid(path, "boolean lane differs");
    else if (f.dtype == "varchar" && p.s.size() != rows) invalid(path, "string lane length differs");
    if (kind == Piece::Kind::list_d || kind == Piece::Kind::list_i || kind == Piece::Kind::list_s) {
        const auto size = offsets(p, rows, path);
        if (size != (kind == Piece::Kind::list_d ? p.d.size() : kind == Piece::Kind::list_i ? p.i.size() : p.s.size())) invalid(path, "list extent differs");
        if (f.shape) for (std::size_t row = 0; row < rows; ++row) if (row_active(active, row) && valid_at(p, row) && (f.shape->empty() || p.offsets[row+1] - p.offsets[row] != f.shape->front())) invalid(path, "fixed list width differs");
    }
    if (kind == Piece::Kind::map) {
        const auto size = offsets(p, rows, path);
        if (size != p.s.size() || !p.items) invalid(path, "map extent/items differ");
        for (std::size_t row = 0; row < rows; ++row) {
            std::set<std::string_view> keys;
            for (auto index = p.offsets[row]; index < p.offsets[row+1]; ++index) if (p.s[static_cast<std::size_t>(index)].empty() || !keys.insert(p.s[static_cast<std::size_t>(index)]).second) invalid(path, "invalid map keys");
        }
        validate_node(item_field(f), *p.items, size, path);
        for (std::size_t index = 0; index < size; ++index)
            if (!valid_at(*p.items, index)) invalid(path, "null map item is outside the reviewed source domain");
    }
    if (kind == Piece::Kind::structure) {
        const bool sequence = f.dtype == "struct[]";
        const auto count = sequence ? offsets(p, rows, path) : rows;
        // Non-sequence children share the parent's row axis: a row's children
        // are semantically checked only when the parent's struct slot is
        // valid. struct[]/map children index elements, which only exist under
        // valid list slots, so they keep the unrestricted domain.
        std::vector<char> inner;
        if (!sequence) {
            inner.reserve(count);
            for (std::size_t row = 0; row < count; ++row)
                inner.push_back(row_active(active, row) && valid_at(p, row) ? 1 : 0);
        }
        std::size_t expected = 0;
        for (const auto& c : f.children) {
            ++expected;
            auto const it = p.children.find(c.key);
            if (it == p.children.end()) invalid(path, "missing structure child");
            validate_node(c, it->second, count, std::string(path) + "." + c.key,
                          sequence ? std::span<const char>{} : std::span<const char>(inner));
            const Piece* present = nullptr;
            if (c.optional) {
                ++expected; auto const mask = p.children.find(c.key + "_present"); if (mask == p.children.end()) invalid(path, "missing structure presence");
                validate_node(boolean_field(c.key), mask->second, count, path,
                              sequence ? std::span<const char>{} : std::span<const char>(inner));
                present = &mask->second;
            }
            for (std::size_t row = 0; row < count; ++row) {
                const bool active_row = sequence || inner[row] != 0;
                if (present && (!valid_at(*present, row) || (active_row && !present->b[row] && valid_at(it->second, row)))) invalid(path, "inconsistent nested presence");
                if (!c.optional && active_row && !valid_at(it->second, row)) invalid(path, "missing required structure member");
                if (!sequence && !inner[row] && present && present->b[row])
                    invalid(path, "null structure contains a present member");
            }
        }
        if (p.children.size() != expected) invalid(path, "unknown structure children");
    }
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
std::size_t piece_rows(const Piece& p) {
    if (!p.valid.empty()) return p.valid.size();
    if (!p.offsets.empty()) return p.offsets.size() - 1;
    switch (p.kind) {
        case Piece::Kind::scalar_d: return p.d.size(); case Piece::Kind::scalar_i: return p.i.size();
        case Piece::Kind::scalar_b: return p.b.size(); case Piece::Kind::scalar_s: return p.s.size();
        case Piece::Kind::structure: return p.children.empty() ? 0 : piece_rows(p.children.begin()->second);
        case Piece::Kind::list_d: case Piece::Kind::list_i: case Piece::Kind::list_s:
        case Piece::Kind::map: return 0;
    }
    return 0;
}
template<class T> std::vector<T> slice(const std::vector<T>& v, std::size_t first, std::size_t last) {
    if (first > last || last > v.size()) throw std::invalid_argument("recording piece slice outside lane");
    return {v.begin() + static_cast<std::ptrdiff_t>(first), v.begin() + static_cast<std::ptrdiff_t>(last)};
}
} // namespace

runtime::EncodedTable encode_rows(const TableSpec& spec, std::span<const Object> rows, std::int64_t episode_id) {
    if (episode_id < 0) invalid(spec.name, "negative episode identity");
    runtime::EncodedTable out; out.rows = static_cast<std::int64_t>(rows.size());
    for (const auto& [name, dtype] : column_types(spec)) { (void)dtype; const auto* field = spec.field(name); out.columns.emplace(name, make_piece(field ? *field : boolean_field(name))); }
    const auto parents = parent_paths(spec);
    for (const auto & row : rows) {
        const auto view = project(spec, row);
        for (const auto& parent : parents) {
            const auto* v = at_path(row, parent); const auto marker = "__" + value::join(parent);
            out.columns.at(marker + "_present").b.push_back(v ? 1 : 0);
            out.columns.at(marker + "_valid").b.push_back(v && !io::is_null(*v) ? 1 : 0);
        }
        for (const auto& key : spec.keys) {
            const auto* v = io::find_member(view, key.key);
            if (key.key == "episode_id") {
                if (v && !io::is_null(*v) && io::as_int64(*v, key.key) != episode_id) invalid(spec.name, "row episode identity differs");
                append(key, out.columns.at(key.key), value::integer(episode_id), key.key);
            } else {
                if (!v || io::is_null(*v)) invalid(spec.name + "." + key.key, "required key");
                // invalid() is [[noreturn]] — `v` is non-null on this path.
                // cppcheck-suppress nullPointerRedundantCheck
                append(key, out.columns.at(key.key), *v, key.key);
            }
        }
        for (const auto& field : spec.fields) {
            const auto* v = io::find_member(view, field.key);
            if (field.optional) out.columns.at(field.key + "_present").b.push_back(v ? 1 : 0);
            if (v) {
                if (io::is_null(*v) && !field.optional) invalid(spec.name + "." + field.key, "required field may not be null");
                append(field, out.columns.at(field.key), *v, spec.name + "." + field.key);
            } else {
                bool inactive = false;
                if (spec.kind == "channels") {
                    const auto* root = io::find_member(row, field.path.front()); inactive = !root || io::is_null(*root);
                    for (std::size_t depth = 1; depth < field.path.size(); ++depth) { const auto* parent = at_path(row, std::span<const std::string>(field.path).first(depth)); inactive = inactive || !parent || io::is_null(*parent); }
                }
                if (!field.optional && !inactive) invalid(spec.name + "." + field.key, "required field missing");
                append_null(field, out.columns.at(field.key));
            }
        }
    }
    validate_pieces(out, spec); return out;
}
void validate_pieces(const runtime::EncodedTable& table, const Catalog& catalog, std::string_view name) {
    const auto& spec = catalog.table(name); validate_pieces(table, spec);
}
void validate_pieces(const runtime::EncodedTable& table, const TableSpec& spec) {
    if (table.rows < 0) invalid(spec.name, "negative row count");
    if (table.rows == 0 && table.columns.empty()) return;
    std::set<std::string> expected;
    for (const auto& [name, dtype] : column_types(spec)) {
        (void)dtype; expected.insert(name); auto const found = table.columns.find(name);
        if (name == "episode_id" && found == table.columns.end()) continue;
        if (found == table.columns.end()) invalid(spec.name + "." + name, "missing declared column");
        const auto* field = spec.field(name);
        validate_node(field ? *field : boolean_field(name), found->second, static_cast<std::size_t>(table.rows), spec.name + "." + name);
    }
    for (const auto& [name, p] : table.columns) { (void)p; if (!expected.contains(name)) invalid(spec.name, "unreviewed column " + name); }
    const auto parents = parent_paths(spec);
    auto const flag = [&](const std::string& name,std::size_t row) {
        const auto& piece = table.columns.at(name);
        if (!valid_at(piece,row)) invalid(spec.name + "." + name,"presence mask may not be null");
        return piece.b[row] != 0;
    };
    for (std::size_t row = 0; std::cmp_less(row ,table.rows); ++row) {
        if(spec.name=="channel_spatial"||spec.name=="channel_spatial_energy"||spec.name=="channel_spatial_geometry"||spec.name=="channel_spatial_outcome")
            for(const auto& root:spec.claims)
                if(!flag("__"+root+"_present",row)||!flag("__"+root+"_valid",row))invalid(spec.name,"spatial physical interval requires explicit source root");
        for (const auto& parent : parents) {
            const auto marker = "__" + value::join(parent);
            const bool present = flag(marker+"_present",row), valid = flag(marker+"_valid",row);
            if (valid && !present) invalid(spec.name,"valid parent is absent");
            if (parent.size()>1 && present && !flag("__"+value::join(std::span<const std::string>(parent).first(parent.size()-1))+"_valid",row))
                invalid(spec.name,"present child has an absent/null parent");
        }
        for (const auto& key : spec.keys) {
            const auto it = table.columns.find(key.key);
            if (it != table.columns.end() && !valid_at(it->second,row)) invalid(spec.name+"."+key.key,"null identity");
        }
        for (const auto& field : spec.fields) {
            const bool valid = valid_at(table.columns.at(field.key),row);
            bool active = true;
            for (std::size_t depth=1;depth<field.path.size();++depth)
                active = active && flag("__"+value::join(std::span<const std::string>(field.path).first(depth))+"_valid",row);
            const bool root = field.path.size()==1 && contains(spec.claims,field.path.front());
            bool empty_remainder = false;
            if (field.optional) {
                const bool present = flag(field.key+"_present",row);
                if(spec.name=="channel_spatial"&&!present)invalid(spec.name+"."+field.key,"nullable spatial fields must be explicit, including null values");
                if ((!present&&valid)||(!active&&(present||valid))) invalid(spec.name+"."+field.key,"inconsistent source presence");
                // _distribute deliberately leaves a root container's terminal
                // absent when its routed remainder is empty. The raw root
                // still existed and was a non-null mapping (T11 source test).
                empty_remainder = root && !present && flag("__"+field.path.front()+"_valid",row) &&
                    (field.dtype.starts_with("map<") || field.dtype == "struct" || field.dtype == "struct[]");
                if (root && !empty_remainder && present != flag("__"+field.path.front()+"_present",row)) invalid(spec.name,"root presence differs from its value");
            } else {
                if (root) active=flag("__"+field.path.front()+"_valid",row);
                if (active != valid) invalid(spec.name+"."+field.key,"required value differs from parent validity");
            }
            if (root && !empty_remainder && valid != flag("__"+field.path.front()+"_valid",row)) invalid(spec.name,"root validity differs from its value");
        }
    }
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
runtime::Piece slice_piece(const Piece& p, std::size_t first, std::size_t last) {
    if (first > last || last > piece_rows(p)) throw std::invalid_argument("recording piece row slice outside bounds");
    Piece out; out.kind = p.kind;
    if (!p.valid.empty()) out.valid = slice(p.valid, first, last);
    std::size_t low = first, high = last;
    if (!p.offsets.empty()) {
        if (last >= p.offsets.size() || p.offsets[first] < 0 || p.offsets[last] < p.offsets[first]) throw std::invalid_argument("recording piece offset slice invalid");
        low = static_cast<std::size_t>(p.offsets[first]); high = static_cast<std::size_t>(p.offsets[last]);
        for (std::size_t i = first; i <= last; ++i) out.offsets.push_back(p.offsets[i] - p.offsets[first]);
    }
    switch (p.kind) {
        case Piece::Kind::scalar_d: case Piece::Kind::list_d: out.d = slice(p.d, low, high); break;
        case Piece::Kind::scalar_i: case Piece::Kind::list_i: out.i = slice(p.i, low, high); break;
        case Piece::Kind::scalar_b: out.b = slice(p.b, low, high); break;
        case Piece::Kind::scalar_s: case Piece::Kind::list_s: out.s = slice(p.s, low, high); break;
        case Piece::Kind::map:
            if (!p.items) throw std::invalid_argument("recording map has no items");
            out.s = slice(p.s, low, high); out.items = std::make_unique<Piece>(slice_piece(*p.items, low, high)); break;
        case Piece::Kind::structure:
            for (const auto& [name, child] : p.children) out.children.emplace(name, slice_piece(child, low, high));
            break;
    }
    // member_slots deliberately stays empty. Rebuilt map ownership never
    // retains pointers into the unsliced/source piece.
    return out;
}
runtime::EncodedTable slice_table(const runtime::EncodedTable& table, std::size_t first, std::size_t last) {
    if (table.rows < 0 || first > last || std::cmp_greater(last ,table.rows)) throw std::invalid_argument("recording table slice outside bounds");
    runtime::EncodedTable out; out.rows = static_cast<std::int64_t>(last - first);
    for (const auto& [name, piece] : table.columns) out.columns.emplace(name, slice_piece(piece, first, last));
    return out;
}
void piece_fragments(std::string_view name, runtime::EncodedTable table, std::size_t maximum, const BatchSink& sink) {
    if (!maximum || table.rows < 0) throw std::invalid_argument("recording batch bound must be positive");
    const auto count = static_cast<std::size_t>(table.rows);
    if (count <= maximum) { sink(name, std::move(table)); return; }
    for (std::size_t first = 0; first < count;) { const auto last = first + std::min(maximum, count-first); sink(name, slice_table(table, first, last)); first = last; }
}
} // namespace bike::recording
