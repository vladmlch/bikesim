// Plan executor for the columnar recording encoder. The Python plan
// compiler (recording/native_encode.py::compile_plan) stays the source of
// truth for which tables are eligible; this side only validates the plan's
// declared kind/table and runs the fixed per-kind emitters over retained
// PhysicalSampleData. Emitted buffers copy retained values verbatim —
// including NaN payloads, ±inf and -0.0 — so pieces_to_arrow assembles a
// RecordBatch bitwise-identical to arrow_batch(encode_rows(...)).
#include "encode.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <variant>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "samples_binding.hpp"

namespace runtime {
namespace {

// compile_plan emits {'table', 'kind'} plus an optional 'columns' list for
// physics/forces; a channels plan adds 'roots', 'all_roots', 'columns' (per
// leaf) and 'masks'. physics/forces columns are fixed emitters; a provided
// list that does not name exactly the fixed set is rejected rather than
// silently narrowed.
struct PlanColumn {
    std::string name;
    std::string dtype;
    std::vector<std::string> path;
    bool optional = false;
    std::int64_t shape = -1;              // >= 0 pins the x[] element count
    std::vector<std::string> except_keys; // parsed; only map emitters read it
    std::string convert;                  // "" | "verdict"
    // Indices of the op=='valid' masks whose paths are this leaf's prefix
    // chain (path[:1..len-1]; the claim mask itself for depth-1 leaves). A
    // required leaf encodes NULL iff one of them reads invalid this row —
    // the encode_rows "ancestor absent-or-null" exemption.
    std::vector<std::size_t> ancestors;
};

struct PlanMask {
    std::string name;
    std::vector<std::string> path;
    enum class Op : std::uint8_t { present, valid } op = Op::present;
};

struct PlanView {
    std::string table;
    std::string kind;
    std::vector<std::string> columns;     // physics/forces fixed-list check
    bool has_columns = false;
    // channels payloads
    std::vector<PlanColumn> leaf_columns;
    std::vector<PlanMask> masks;
    std::vector<std::string> roots;       // this table's claimed roots
    std::set<std::string> all_roots;      // every catalog channel root
    // Routing indexes mirroring encoding.py::_distribute's lookups: full
    // leaf paths, and each internal prefix's routed member components.
    std::set<std::vector<std::string>> leaf_paths;
    std::map<std::vector<std::string>, std::set<std::string>> deeper;
};

constexpr std::array<std::string_view, 6> kPhysicsColumns{
    "interval_id", "time_s", "interval_end_s", "dt_s", "qpos", "qvel"};
constexpr std::array<std::string_view, 4> kForcesColumns{
    "interval_id", "component", "qfrc", "power_w"};

// constexpr array — a std::set initializer could throw at static init.
constexpr std::array<std::string_view, 7> kLeafDtypes{
    "double", "bigint", "boolean", "varchar",
    "double[]", "bigint[]", "varchar[]"};

bool is_leaf_dtype(const std::string &dtype) {
    return std::ranges::find(kLeafDtypes, dtype) != kLeafDtypes.end();
}

PlanColumn parse_column(nb::handle item) {
    const wire::Dict dict{.value = wire::mapping(item, "encode plan.columns[]"),
                          .path = "encode plan.columns[]"};
    wire::exact(dict, wire::keys("name", "dtype", "path", "optional"),
                wire::keys("shape", "except_keys", "convert"));
    PlanColumn column;
    column.name = wire::string(wire::field(dict, "name"),
                               dict.child("name"));
    column.dtype = wire::string(wire::field(dict, "dtype"),
                                dict.child("dtype"));
    if (!is_leaf_dtype(column.dtype))
        throw std::invalid_argument("encode plan.columns: unsupported "
                                    "channels dtype '" + column.dtype + "'");
    for (const nb::handle part :
         wire::sequence(wire::field(dict, "path"), dict.child("path")))
        column.path.push_back(wire::string(part, dict.child("path")));
    if (column.path.empty())
        throw std::invalid_argument("encode plan.columns: empty path");
    column.optional = wire::boolean(wire::field(dict, "optional"),
                                    dict.child("optional"));
    if (dict.contains("shape") && !dict["shape"].is_none()) {
        const nb::list dims =
            wire::sequence(dict["shape"], dict.child("shape"));
        if (dims.size() != 1)
            throw std::invalid_argument(
                "encode plan.columns: shape must name one dimension");
        for (const nb::handle dim : dims)
            column.shape = wire::integer(dim, dict.child("shape"));
    }
    if (dict.contains("except_keys"))
        for (const nb::handle key : wire::sequence(
                 dict["except_keys"], dict.child("except_keys")))
            column.except_keys.push_back(
                wire::string(key, dict.child("except_keys")));
    if (dict.contains("convert") && !dict["convert"].is_none()) {
        column.convert = wire::string(dict["convert"],
                                      dict.child("convert"));
        if (column.convert != "verdict")
            throw std::invalid_argument("encode plan.columns: unknown "
                                        "convert '" + column.convert + "'");
    }
    return column;
}

PlanMask parse_mask(nb::handle item) {
    const wire::Dict dict{.value = wire::mapping(item, "encode plan.masks[]"),
                          .path = "encode plan.masks[]"};
    wire::exact(dict, wire::keys("name", "path", "op"));
    PlanMask mask;
    mask.name = wire::string(wire::field(dict, "name"),
                             dict.child("name"));
    for (const nb::handle part :
         wire::sequence(wire::field(dict, "path"), dict.child("path")))
        mask.path.push_back(wire::string(part, dict.child("path")));
    if (mask.path.empty())
        throw std::invalid_argument("encode plan.masks: empty path");
    const std::string op =
        wire::string(wire::field(dict, "op"), dict.child("op"));
    if (op == "present") mask.op = PlanMask::Op::present;
    else if (op == "valid") mask.op = PlanMask::Op::valid;
    else throw std::invalid_argument("encode plan.masks: unknown op '" +
                                     op + "'");
    return mask;
}

void parse_channels_plan(const wire::Dict &dict, PlanView &parsed) {
    wire::exact(dict, wire::keys("table", "kind", "roots", "all_roots",
                                 "columns", "masks"));
    for (const nb::handle item :
         wire::sequence(dict["columns"], "encode plan.columns"))
        parsed.leaf_columns.push_back(parse_column(item));
    for (const nb::handle item :
         wire::sequence(dict["masks"], "encode plan.masks"))
        parsed.masks.push_back(parse_mask(item));
    for (const nb::handle item :
         wire::sequence(dict["roots"], "encode plan.roots"))
        parsed.roots.push_back(wire::string(item, "encode plan.roots"));
    for (const nb::handle item :
         wire::sequence(dict["all_roots"], "encode plan.all_roots"))
        parsed.all_roots.insert(wire::string(item, "encode plan.all_roots"));
    const std::set<std::string> claims(parsed.roots.begin(),
                                       parsed.roots.end());
    for (const PlanColumn &column : parsed.leaf_columns) {
        if (!claims.contains(column.path.front()))
            throw std::invalid_argument(
                "encode plan.columns: '" + column.name +
                "' path is outside the claimed roots");
        parsed.leaf_paths.insert(column.path);
        for (std::size_t depth = 1; depth < column.path.size(); ++depth)
            parsed.deeper[std::vector<std::string>(
                column.path.begin(),
                column.path.begin() + static_cast<std::ptrdiff_t>(depth))]
                .insert(column.path[depth]);
    }
    // Resolve each leaf's absent-or-null prefix chain to its valid-mask
    // slot; the plan must carry every mask the exemption consults.
    std::map<std::vector<std::string>, std::size_t> valid_masks;
    for (std::size_t j = 0; j < parsed.masks.size(); ++j)
        if (parsed.masks[j].op == PlanMask::Op::valid)
            valid_masks.emplace(parsed.masks[j].path, j);
    for (PlanColumn &column : parsed.leaf_columns) {
        // A depth-1 leaf's exemption reads the claim mask at its own path;
        // deeper leaves read every proper prefix's mask.
        const std::size_t deepest = column.path.size() > 1
            ? column.path.size() - 1 : 1;
        for (std::size_t depth = 1; depth <= deepest; ++depth) {
            const auto found = valid_masks.find(std::vector<std::string>(
                column.path.begin(),
                column.path.begin() + static_cast<std::ptrdiff_t>(depth)));
            if (found == valid_masks.end())
                throw std::invalid_argument(
                    "encode plan is missing the presence mask at " +
                    column.name);
            column.ancestors.push_back(found->second);
        }
    }
}

PlanView parse_plan(nb::handle plan) {
    const wire::Dict dict{.value = wire::mapping(plan, "encode plan"),
                          .path = "encode plan"};
    PlanView parsed;
    parsed.table = wire::string(wire::field(dict, "table"), "encode plan.table");
    parsed.kind = wire::string(wire::field(dict, "kind"), "encode plan.kind");
    if (parsed.kind == "channels") {
        parse_channels_plan(dict, parsed);
        return parsed;
    }
    wire::exact(dict, wire::keys("table", "kind"), wire::keys("columns"));
    if (dict.contains("columns")) {
        parsed.has_columns = true;
        for (const nb::handle item :
             wire::sequence(dict["columns"], "encode plan.columns"))
            parsed.columns.emplace_back(
                wire::string(item, "encode plan.columns"));
    }
    return parsed;
}

void check_columns(const PlanView &plan,
                   std::span<const std::string_view> emitted) {
    if (!plan.has_columns) return;
    if (plan.columns.size() != emitted.size() ||
        !std::ranges::equal(plan.columns, emitted))
        throw std::invalid_argument(
            "encode plan.columns does not match the fixed emitters");
}

// powers_w members are authored recording values: NaN and ±inf round-trip
// verbatim through encode_rows (a recorded double never becomes NULL), so
// this is sample_wire::number minus the finiteness gate. The bool/None/
// string lanes still reject, matching _convert_scalar's 'double' domain.
double recorded_number(const Wire &value) {
    if (const auto *real = std::get_if<double>(&value.value)) return *real;
    if (const auto *whole = std::get_if<std::int64_t>(&value.value))
        return static_cast<double>(*whole);
    throw std::invalid_argument("forces.power_w: expected a real number");
}

// physics: one output row per retained interval. dt_s is the same single
// IEEE subtraction the oracle performs (end_time_s - time_s).
Piece emit_physics_column(std::span<const SamplePtr> rows,
                          std::string_view name) {
    Piece out;
    if (name == "interval_id") {
        out.kind = Piece::Kind::scalar_i;
        out.i.reserve(rows.size());
        for (const auto &s : rows) out.i.push_back(s->interval_id);
        return out;
    }
    if (name == "time_s" || name == "interval_end_s" || name == "dt_s") {
        out.kind = Piece::Kind::scalar_d;
        out.d.reserve(rows.size());
        for (const auto &s : rows)
            out.d.push_back(name == "time_s" ? s->time_s :
                            name == "dt_s" ? s->dt_s() : s->end_time_s);
        return out;
    }
    if (name == "qpos" || name == "qvel") {
        // Flat payload plus per-row end offsets — the list piece.
        out.kind = Piece::Kind::list_d;
        out.offsets.reserve(rows.size() + 1);
        out.offsets.push_back(0);
        for (const auto &s : rows) {
            const auto &v = name == "qpos" ? s->qpos : s->qvel;
            out.d.insert(out.d.end(), v.begin(), v.end());
            out.offsets.push_back(static_cast<std::int32_t>(out.d.size()));
        }
        return out;
    }
    throw std::invalid_argument("encode: unknown physics column " +
                                std::string(name));
}

// forces: explode interval × component in authored order. Every component
// requires its powers_w entry — parity with force_rows' MissingFieldError.
std::pair<std::int64_t, std::map<std::string, Piece>>
emit_forces(std::span<const SamplePtr> rows) {
    std::size_t total = 0;
    for (const auto &s : rows) total += s->forces.size();
    Piece ids, component, qfrc, power;
    ids.kind = Piece::Kind::scalar_i;
    component.kind = Piece::Kind::scalar_s;
    qfrc.kind = Piece::Kind::list_d;
    power.kind = Piece::Kind::scalar_d;
    ids.i.reserve(total);
    component.s.reserve(total);
    power.d.reserve(total);
    qfrc.offsets.reserve(total + 1);
    qfrc.offsets.push_back(0);
    for (const auto &s : rows)
        for (const auto &[name, vector] : s->forces) {
            const Wire *entry = sample_wire::find(s->powers_w, name);
            if (entry == nullptr)
                throw std::invalid_argument(
                    "forces." + name + ": no matching powers_w entry");
            ids.i.push_back(s->interval_id);
            component.s.push_back(name);
            qfrc.d.insert(qfrc.d.end(), vector.begin(), vector.end());
            qfrc.offsets.push_back(static_cast<std::int32_t>(qfrc.d.size()));
            power.d.push_back(recorded_number(*entry));
        }
    std::map<std::string, Piece> emitted;
    emitted.emplace("interval_id", std::move(ids));
    emitted.emplace("component", std::move(component));
    emitted.emplace("qfrc", std::move(qfrc));
    emitted.emplace("power_w", std::move(power));
    return {static_cast<std::int64_t>(total), std::move(emitted)};
}

// ---------------------------------------------------------------------------
// channels: path-walking leaf emitters and presence/valid masks

std::string join_path(std::span<const std::string> path) {
    std::string out;
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) out += '.';
        out += path[i];
    }
    return out;
}

// `label.member` without the operator+ temporaries the linter dislikes.
std::string member_label(const std::string &label, std::string_view member) {
    std::string out = label;
    out += '.';
    out += member;
    return out;
}

// sorted() member names rendered the way Python prints the oracle's
// 'unrecognized domains'/'unknown channel members' lists.
std::string quoted_list(std::vector<std::string_view> names) {
    std::ranges::sort(names);
    std::string out = "[";
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i != 0) out += ", ";
        out += '\'';
        out += names[i];
        out += '\'';
    }
    out += ']';
    return out;
}

// Follows the member chain under the retained channels object; every node
// along the chain must be an object member. *complete reports whether each
// member resolved (the mask `present` reading); the returned pointer is the
// terminal member — a null member still resolves, distinguishing an
// explicit None from an absent key.
const Wire *walk_path(const WireObject &channels,
                      std::span<const std::string> path, bool &complete) {
    const WireObject *node = &channels;
    const Wire *value = nullptr;
    complete = true;
    for (const std::string &member : path) {
        if (node == nullptr) {
            complete = false;
            return nullptr;
        }
        value = sample_wire::find(*node, member);
        if (value == nullptr) {
            complete = false;
            return nullptr;
        }
        node = std::get_if<WireObject>(&value->value);
    }
    return value;
}

// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the declared field paths
void distribute_validate(const PlanView &plan, std::vector<std::string> &prefix,
                         const Wire &payload, const std::string &label) {
    const bool is_leaf = plan.leaf_paths.contains(prefix);
    if (sample_wire::is_none(payload)) return;
    const auto *object = std::get_if<WireObject>(&payload.value);
    if (object == nullptr) {
        // A scalar/list payload under a non-leaf prefix mirrors the oracle's
        // "unrouted" rejection; leaf prefixes pass it to the leaf walk.
        if (!is_leaf)
            throw std::invalid_argument(
                label + ": no reviewed field accepts the payload");
        return;
    }
    if (is_leaf) {
        // A leaf-dtype terminal cannot absorb object members (map/struct
        // terminals belong to the Task-4/5 machinery); name them all.
        if (!object->empty()) {
            std::vector<std::string_view> members;
            members.reserve(object->size());
            for (const auto &[member, _] : *object) members.push_back(member);
            throw std::invalid_argument(label + ": unknown channel members " +
                                        quoted_list(std::move(members)));
        }
        return;
    }
    const auto deeper = plan.deeper.find(prefix);
    std::vector<std::string_view> unrouted;
    for (const auto &[member, value] : *object) {
        if (deeper == plan.deeper.end() || !deeper->second.contains(member)) {
            unrouted.push_back(member);
            continue;
        }
        prefix.push_back(member);
        const bool child_leaf = plan.leaf_paths.contains(prefix);
        const std::string child_label = member_label(label, member);
        if (!child_leaf && !sample_wire::is_none(value) &&
            !std::holds_alternative<WireObject>(value.value))
            throw std::invalid_argument(
                child_label + ": no reviewed field accepts the payload");
        if (!child_leaf)
            distribute_validate(plan, prefix, value, child_label);
        prefix.pop_back();
    }
    if (!unrouted.empty())
        throw std::invalid_argument(label + ": unknown channel members " +
                                    quoted_list(std::move(unrouted)));
}

// One NULL slot in a leaf piece: placeholder payload behind a false
// validity bit (or an empty span for a list leaf).
void push_null(Piece &piece) {
    switch (piece.kind) {
        case Piece::Kind::scalar_d: piece.d.push_back(0.0); break;
        case Piece::Kind::scalar_i: piece.i.push_back(0); break;
        case Piece::Kind::scalar_b: piece.b.push_back(0); break;
        case Piece::Kind::scalar_s: piece.s.emplace_back(); break;
        case Piece::Kind::list_d:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s:
            piece.offsets.push_back(static_cast<std::int32_t>(
                piece.kind == Piece::Kind::list_i ? piece.i.size()
                : piece.kind == Piece::Kind::list_s ? piece.s.size()
                : piece.d.size()));
            break;
        case Piece::Kind::structure:
        case Piece::Kind::map:
            throw std::invalid_argument(
                "encode: non-leaf piece kind in a channels leaf column");
    }
    piece.valid.push_back(0);
}

// One element of an x[] leaf, mirroring the member dtype's per-element
// conversion boundary (encoding.py::_convert_scalar inside _convert).
void append_list_element(Piece &piece, const Wire &element,
                         const std::string &label) {
    if (piece.kind == Piece::Kind::list_d) {
        const auto *real = std::get_if<double>(&element.value);
        const auto *whole = std::get_if<std::int64_t>(&element.value);
        if (real != nullptr) piece.d.push_back(*real);
        else if (whole != nullptr)
            piece.d.push_back(static_cast<double>(*whole));
        else
            throw std::invalid_argument(label + ": expected a real number");
        return;
    }
    if (piece.kind == Piece::Kind::list_i) {
        if (const auto *whole = std::get_if<std::int64_t>(&element.value))
            piece.i.push_back(*whole);
        else
            throw std::invalid_argument(label + ": expected an integer");
        return;
    }
    const auto *text = std::get_if<std::string>(&element.value);
    if (text == nullptr)
        throw std::invalid_argument(label + ": expected a string");
    piece.s.push_back(*text);
}

// UTF-8 lead byte -> code-point byte width, validating the continuation
// bytes so a malformed payload rejects instead of mis-splitting (wire
// strings are UTF-8 by construction, but an authored C++ row could carry
// anything; Python only ever iterates valid str).
std::size_t utf8_width(char lead, const std::string &text,
                       std::size_t at, const std::string &label) {
    const auto byte = static_cast<unsigned char>(lead);
    std::size_t width = 0;
    if (byte < 0x80) width = 1;
    else if (byte >= 0xC0 && byte < 0xE0) width = 2;  // 110xxxxx
    else if (byte >= 0xE0 && byte < 0xF0) width = 3;  // 1110xxxx
    else if (byte >= 0xF0 && byte < 0xF8) width = 4;  // 11110xxx
    const bool well_formed =
        width != 0 && at + width <= text.size() &&
        std::ranges::all_of(text.substr(at + 1, width - 1), [](char c) {
            const auto cont = static_cast<unsigned char>(c);
            return cont >= 0x80 && cont < 0xC0;   // 10xxxxxx
        });
    if (!well_formed)
        throw std::invalid_argument(label + ": invalid UTF-8");
    return width;
}

// Converts one resolved (non-null) leaf value into its piece; lists append
// their elements plus the end offset. Mirrors encoding.py::_convert's
// dtype/shape gates; a dict routed to a deeper leaf still converts, so no
// Mapping check precedes the dtype gates.
void append_leaf(const PlanColumn &column, const Wire &value, Piece &piece,
                 const std::string &label) {
    switch (piece.kind) {
        case Piece::Kind::scalar_d: {
            const auto *real = std::get_if<double>(&value.value);
            const auto *whole = std::get_if<std::int64_t>(&value.value);
            if (real != nullptr) piece.d.push_back(*real);
            else if (whole != nullptr)
                piece.d.push_back(static_cast<double>(*whole));
            else
                throw std::invalid_argument(
                    label + ": expected a real number");
            return;
        }
        case Piece::Kind::scalar_i:
            if (const auto *whole = std::get_if<std::int64_t>(&value.value)) {
                piece.i.push_back(*whole);
                return;
            }
            throw std::invalid_argument(label + ": expected an integer");
        case Piece::Kind::scalar_b:
            if (const auto *flag = std::get_if<bool>(&value.value)) {
                piece.b.push_back(static_cast<char>(*flag));
                return;
            }
            throw std::invalid_argument(label + ": expected a bool");
        case Piece::Kind::scalar_s:
            // 'verdict' mirrors NativePhysicalSource._canonical's coercion
            // of model_status.numerically_valid: a bool becomes 'true'/
            // 'false' verdict text; every other value converts verbatim
            // (which still rejects non-strings below).
            if (column.convert == "verdict")
                if (const auto *flag = std::get_if<bool>(&value.value)) {
                    piece.s.emplace_back(*flag ? "true" : "false");
                    return;
                }
            if (const auto *text = std::get_if<std::string>(&value.value)) {
                piece.s.push_back(*text);
                return;
            }
            throw std::invalid_argument(label + ": expected a string");
        case Piece::Kind::list_d:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s: {
            // Python `enumerate(value)` accepts any iterable: an array
            // yields elements, a str its code points (only 'varchar[]'
            // accepts them), and a Mapping its keys — _distribute assigns
            // a dict at a leaf terminal verbatim, so it is reachable here
            // (a depth-1 leaf root's dict still rejects in routing, the
            // oracle's "unknown channel members" remainder path).
            std::size_t count = 0;
            const auto element_label = [&]() {
                return label + "[" + std::to_string(count) + "]";
            };
            if (const auto *items =
                    std::get_if<WireArray>(&value.value)) {
                for (const Wire &element : *items) {
                    append_list_element(piece, element, element_label());
                    ++count;
                }
            } else if (const auto *text =
                           std::get_if<std::string>(&value.value)) {
                for (std::size_t at = 0; at < text->size(); ++count) {
                    const std::size_t width =
                        utf8_width((*text)[at], *text, at, label);
                    const Wire element{text->substr(at, width)};
                    append_list_element(piece, element, element_label());
                    at += width;
                }
            } else if (const auto *object =
                           std::get_if<WireObject>(&value.value)) {
                for (const auto &[member, _] : *object) {
                    const Wire element{member};
                    append_list_element(piece, element, element_label());
                    ++count;
                }
            } else {
                throw std::invalid_argument(
                    label + ": expected a sequence");
            }
            if (column.shape >= 0 &&
                std::cmp_not_equal(count, column.shape))
                throw std::invalid_argument(
                    label + ": expected " + std::to_string(column.shape) +
                    " elements, got " + std::to_string(count));
            piece.offsets.push_back(static_cast<std::int32_t>(
                piece.kind == Piece::Kind::list_i ? piece.i.size()
                : piece.kind == Piece::Kind::list_s ? piece.s.size()
                : piece.d.size()));
            return;
        }
        case Piece::Kind::structure:
        case Piece::Kind::map:
            throw std::invalid_argument(
                "encode: non-leaf piece kind in a channels leaf column");
    }
}

Piece::Kind leaf_kind(std::string_view dtype) {
    if (dtype == "double") return Piece::Kind::scalar_d;
    if (dtype == "bigint") return Piece::Kind::scalar_i;
    if (dtype == "boolean") return Piece::Kind::scalar_b;
    if (dtype == "varchar") return Piece::Kind::scalar_s;
    if (dtype == "double[]") return Piece::Kind::list_d;
    if (dtype == "bigint[]") return Piece::Kind::list_i;
    return Piece::Kind::list_s;
}

std::int64_t emit_channels(std::span<const SamplePtr> rows,
                           const PlanView &plan,
                           std::map<std::string, Piece> &emitted) {
    std::vector<Piece> values(plan.leaf_columns.size());
    std::vector<Piece> presence(plan.leaf_columns.size());
    std::vector<Piece> mask_pieces(plan.masks.size());
    Piece ids;
    ids.kind = Piece::Kind::scalar_i;
    ids.i.reserve(rows.size());
    for (std::size_t c = 0; c < plan.leaf_columns.size(); ++c) {
        Piece &piece = values[c];
        piece.kind = leaf_kind(plan.leaf_columns[c].dtype);
        piece.valid.reserve(rows.size());
        switch (piece.kind) {
            case Piece::Kind::scalar_d: piece.d.reserve(rows.size()); break;
            case Piece::Kind::scalar_i: piece.i.reserve(rows.size()); break;
            case Piece::Kind::scalar_b: piece.b.reserve(rows.size()); break;
            case Piece::Kind::scalar_s: piece.s.reserve(rows.size()); break;
            case Piece::Kind::list_d:
            case Piece::Kind::list_i:
            case Piece::Kind::list_s:
                piece.offsets.reserve(rows.size() + 1);
                piece.offsets.push_back(0);
                break;
            case Piece::Kind::structure:
            case Piece::Kind::map:
                throw std::invalid_argument(
                    "encode: non-leaf piece kind in a channels leaf column");
        }
        if (plan.leaf_columns[c].optional) {
            presence[c].kind = Piece::Kind::scalar_b;
            presence[c].b.reserve(rows.size());
        }
    }
    for (Piece &piece : mask_pieces) {
        piece.kind = Piece::Kind::scalar_b;
        piece.b.reserve(rows.size());
    }

    // NativePhysicalSource rejects roots outside the catalog's all_roots
    // while materializing canonical rows — before any per-spec encoding —
    // so the gate scans the whole window first.
    for (const auto &sample : rows) {
        std::vector<std::string_view> unknown;
        for (const auto &[root, _] : sample->channels)
            if (!plan.all_roots.contains(root)) unknown.push_back(root);
        if (!unknown.empty())
            throw std::invalid_argument("channels: unrecognized domains " +
                                        quoted_list(std::move(unknown)));
    }

    std::vector<char> mask_valid(plan.masks.size());
    for (std::size_t row = 0; row < rows.size(); ++row) {
        const PhysicalSampleData &sample = *rows[row];
        ids.i.push_back(sample.interval_id);
        const std::string row_label =
            plan.table + "[" + std::to_string(row) + "]";
        // _view() routes every claimed root's payload first: unknown
        // members and unrouted payloads reject ahead of field conversion.
        for (const std::string &root : plan.roots) {
            const Wire *payload = sample_wire::find(sample.channels, root);
            if (payload != nullptr) {
                std::vector<std::string> prefix{root};
                distribute_validate(plan, prefix, *payload,
                                    member_label(row_label, root));
            }
        }
        for (std::size_t j = 0; j < plan.masks.size(); ++j) {
            bool complete = false;
            const Wire *node =
                walk_path(sample.channels, plan.masks[j].path, complete);
            const bool valid = complete && node != nullptr &&
                               !sample_wire::is_none(*node);
            mask_valid[j] = static_cast<char>(valid);
            mask_pieces[j].b.push_back(static_cast<char>(
                plan.masks[j].op == PlanMask::Op::present ? complete
                                                        : valid));
        }
        for (std::size_t c = 0; c < plan.leaf_columns.size(); ++c) {
            const PlanColumn &column = plan.leaf_columns[c];
            Piece &piece = values[c];
            const std::string label =
                member_label(row_label, join_path(column.path));
            bool complete = false;
            const Wire *leaf =
                walk_path(sample.channels, column.path, complete);
            // A dict payload under a depth-1 claim routes nothing (non-empty
            // already threw above; an empty dict leaves the leaf absent).
            const bool resolved =
                leaf != nullptr &&
                (column.path.size() > 1 ||
                 !std::holds_alternative<WireObject>(leaf->value));
            if (resolved && sample_wire::is_none(*leaf)) {
                if (!column.optional)
                    throw std::invalid_argument(
                        label + ": required field may not be null");
                push_null(piece);
                presence[c].b.push_back(1);
            } else if (resolved) {
                append_leaf(column, *leaf, piece, label);
                piece.valid.push_back(1);
                if (column.optional) presence[c].b.push_back(1);
            } else if (column.optional) {
                push_null(piece);
                presence[c].b.push_back(0);
            } else {
                // A required leaf encodes NULL iff an ancestor on its path
                // read absent-or-null this row — encoding.py exempts those
                // so a missing/null parent does not poison its leaves.
                const bool exempt = std::ranges::any_of(
                    column.ancestors,
                    [&](std::size_t j) { return mask_valid[j] == 0; });
                if (!exempt)
                    throw std::invalid_argument(label + ": required");
                push_null(piece);
            }
        }
    }
    emitted["interval_id"] = std::move(ids);
    for (std::size_t c = 0; c < plan.leaf_columns.size(); ++c) {
        emitted[plan.leaf_columns[c].name] = std::move(values[c]);
        if (plan.leaf_columns[c].optional)
            emitted[plan.leaf_columns[c].name + "_present"] =
                std::move(presence[c]);
    }
    for (std::size_t j = 0; j < plan.masks.size(); ++j)
        emitted[plan.masks[j].name] = std::move(mask_pieces[j]);
    return static_cast<std::int64_t>(rows.size());
}

// ---------------------------------------------------------------------------
// applied-command bridge (the ordinary_commands mirror in native.py)

// The Python row-level `actual` dict at wire level: control plus the three
// brake demands, each either a wire pointer or nullptr for an absent key
// (Python keeps absent keys out of the dict entirely).
struct AppliedTuple {
    const Wire *control = nullptr;
    std::array<const Wire *, 3> demands{};
    bool engaged = false;  // mirrors `previous_applied is not None`
};

constexpr std::array<std::string_view, 3> kBrakeDemands{
    "front_brake_demand", "rear_brake_demand", "rollback_brake_demand"};

// Mirrors `{key: drive[key] for key in kBrakeDemands if key in drive}` over
// `channels.get('drive') or {}`: an absent/falsy/non-dict drive yields an
// empty map; a dict yields its members. For a truthy non-dict Python's `in`
// still runs — str does a substring check, a list an element check; a hit
// would then reject inside `drive[key]`, so reaching one raises. Numbers
// and bools reject at `key in drive` outright.
std::array<const Wire *, 3> applied_demands(const Wire &drive) {
    std::array<const Wire *, 3> demands{};
    if (const auto *object = std::get_if<WireObject>(&drive.value)) {
        for (std::size_t k = 0; k < kBrakeDemands.size(); ++k)
            demands[k] = sample_wire::find(*object, kBrakeDemands[k]);
        return demands;
    }
    const auto truthy = [&drive]() {
        if (const auto *flag = std::get_if<bool>(&drive.value)) return *flag;
        if (const auto *whole = std::get_if<std::int64_t>(&drive.value))
            return *whole != 0;
        if (const auto *real = std::get_if<double>(&drive.value))
            return *real != 0.0;   // NaN is truthy, like Python
        if (const auto *text = std::get_if<std::string>(&drive.value))
            return !text->empty();
        if (const auto *items = std::get_if<WireArray>(&drive.value))
            return !items->empty();
        return false;            // monostate (None)
    };
    if (!truthy()) return demands;   // `or {}` — no demands to mirror
    if (const auto *text = std::get_if<std::string>(&drive.value)) {
        for (const std::string_view key : kBrakeDemands)
            if (text->find(key) != std::string::npos)
                throw std::invalid_argument(
                    "channels.drive: expected a dict");
        return demands;
    }
    if (const auto *items = std::get_if<WireArray>(&drive.value)) {
        for (const Wire &element : *items)
            if (const auto *text = std::get_if<std::string>(&element.value);
                text != nullptr &&
                std::ranges::find(kBrakeDemands, std::string_view(*text)) !=
                    kBrakeDemands.end())
                throw std::invalid_argument(
                    "channels.drive: expected a dict");
        return demands;
    }
    throw std::invalid_argument("channels.drive: expected a dict");
}

AppliedTuple applied_tuple(const PhysicalSampleData &sample) {
    AppliedTuple tuple{.engaged = true};
    const Wire *control = sample_wire::find(sample.channels, "control");
    if (control == nullptr || sample_wire::is_none(*control))
        throw std::invalid_argument(
            "ordinary applied control is missing from the physical source");
    tuple.control = control;
    const Wire *drive = sample_wire::find(sample.channels, "drive");
    if (drive != nullptr) tuple.demands = applied_demands(*drive);
    return tuple;
}

// Python `==` across the numeric lanes: int/float/bool compare by exact
// promotion (no int->float rounding), doubles by IEEE equality.
bool numeric_equal(const Wire &a, const Wire &b) {
    const auto to_int = [](const Wire &wire, std::int64_t &out) {
        if (const auto *whole = std::get_if<std::int64_t>(&wire.value)) {
            out = *whole;
            return true;
        }
        if (const auto *flag = std::get_if<bool>(&wire.value)) {
            out = *flag ? 1 : 0;
            return true;
        }
        return false;
    };
    const auto to_double = [](const Wire &wire, double &out) {
        if (const auto *real = std::get_if<double>(&wire.value)) {
            out = *real;
            return true;
        }
        return false;
    };
    double fa = 0.0, fb = 0.0;
    const bool a_double = to_double(a, fa);
    const bool b_double = to_double(b, fb);
    if (!a_double && !b_double) {
        std::int64_t ia = 0, ib = 0;
        return to_int(a, ia) && to_int(b, ib) && ia == ib;
    }
    if (a_double && b_double) return fa == fb;
    // Mixed lane: the non-double side must be int/bool for Python == to
    // hold (a non-numeric other compares unequal).
    const double real = a_double ? fa : fb;
    const Wire &other = a_double ? b : a;
    std::int64_t whole = 0;
    if (!to_int(other, whole)) return false;
    // Python int==float is exact: only an integral in-range float may equal
    // an int (NaN, fractions and |x|>=2**63 compare unequal).
    if (!std::isfinite(real) || std::trunc(real) != real) return false;
    constexpr double kMin = -9223372036854775808.0;
    constexpr double kMaxExclusive = 9223372036854775808.0;
    if (real < kMin || real >= kMaxExclusive) return false;
    return static_cast<std::int64_t>(real) == whole;
}

// Recursive wire equality mirroring Python `==` over plain()-ed values:
// mismatched containers/scalars compare unequal, dicts need equal key sets
// and values, lists compare elementwise. NaN stays unequal to NaN.
// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the wire tree
bool wire_equal(const Wire &a, const Wire &b) {
    const auto is_numeric = [](const Wire &wire) {
        return std::holds_alternative<bool>(wire.value) ||
               std::holds_alternative<std::int64_t>(wire.value) ||
               std::holds_alternative<double>(wire.value);
    };
    if (is_numeric(a) || is_numeric(b)) return numeric_equal(a, b);
    if (a.value.index() != b.value.index()) return false;
    if (std::holds_alternative<std::monostate>(a.value)) return true;
    if (const auto *text = std::get_if<std::string>(&a.value))
        return *text == std::get<std::string>(b.value);
    if (const auto *items = std::get_if<WireArray>(&a.value)) {
        const auto &other = std::get<WireArray>(b.value);
        if (items->size() != other.size()) return false;
        for (std::size_t i = 0; i < items->size(); ++i)
            if (!wire_equal((*items)[i], other[i])) return false;
        return true;
    }
    const auto &object = std::get<WireObject>(a.value);
    const auto &other = std::get<WireObject>(b.value);
    if (object.size() != other.size()) return false;
    // A plain loop keeps the self-recursion on one line the NOLINT covers —
    // an all_of lambda would itself sit inside the recursive call chain.
    for (const auto &entry : object) {
        const Wire *match = sample_wire::find(other, entry.first);
        if (match == nullptr || !wire_equal(entry.second, *match))
            return false;
    }
    return true;
}

bool applied_equal(const AppliedTuple &a, const AppliedTuple &b) {
    if (!a.engaged || !b.engaged) return a.engaged == b.engaged;
    if (!wire_equal(*a.control, *b.control)) return false;
    for (std::size_t k = 0; k < a.demands.size(); ++k) {
        if ((a.demands[k] == nullptr) != (b.demands[k] == nullptr))
            return false;
        if (a.demands[k] != nullptr &&
            !wire_equal(*a.demands[k], *b.demands[k]))
            return false;
    }
    return true;
}

// The scalar/list payload vector, boxed as an owned numpy array (numeric),
// an owned flag array (bool) or a Python string list (varchar/map keys).
nb::object piece_payload(const Piece &piece) {
    switch (piece.kind) {
        case Piece::Kind::scalar_d:
        case Piece::Kind::list_d:
            return wire::owned_array<double>(piece.d).cast();
        case Piece::Kind::scalar_i:
        case Piece::Kind::list_i:
            return wire::owned_array<std::int64_t>(piece.i).cast();
        case Piece::Kind::scalar_b:
            return wire::owned_flags(piece.b).cast();
        case Piece::Kind::scalar_s:
        case Piece::Kind::list_s:
        case Piece::Kind::map: {
            nb::list strings;
            for (const std::string &text : piece.s)
                strings.append(nb::str(text.c_str(), text.size()));
            return strings;
        }
        case Piece::Kind::structure:
            break;
    }
    throw std::invalid_argument(
        "encode: structure pieces carry children, not flat data");
}

// NOLINTNEXTLINE(misc-no-recursion) children/items mirror the finite piece tree
nb::dict to_python(const Piece &piece) {
    nb::dict out;
    switch (piece.kind) {
        case Piece::Kind::scalar_d:
        case Piece::Kind::scalar_i:
        case Piece::Kind::scalar_b:
        case Piece::Kind::scalar_s:
            out["kind"] = nb::str("scalar");
            out["data"] = piece_payload(piece);
            break;
        case Piece::Kind::list_d:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s:
            out["kind"] = nb::str("list");
            out["data"] = piece_payload(piece);
            out["offsets"] = wire::owned_array<std::int32_t>(piece.offsets);
            break;
        case Piece::Kind::structure: {
            out["kind"] = nb::str("structure");
            nb::dict children;
            for (const auto &[name, child] : piece.children)
                children[nb::str(name.c_str(), name.size())] =
                    to_python(child);
            out["children"] = children;
            if (piece.items) out["items"] = to_python(*piece.items);
            break;
        }
        case Piece::Kind::map:
            out["kind"] = nb::str("map");
            out["keys"] = piece_payload(piece);
            if (piece.items) out["items"] = to_python(*piece.items);
            out["offsets"] = wire::owned_array<std::int32_t>(piece.offsets);
            break;
    }
    // An empty validity mask means all-valid; Python sees None, matching
    // the "no mask" convention pieces_to_arrow reads.
    if (piece.valid.empty()) out["valid"] = nb::none();
    else out["valid"] = wire::owned_flags(piece.valid).cast();
    return out;
}

} // namespace

std::pair<std::int64_t, nb::dict> encode_table(
    const OwnedNativeEpisode &episode, nb::handle plan,
    std::int64_t start, std::int64_t limit) {
    const PlanView parsed = parse_plan(plan);
    // Same slice domain as sample_batch: the retained prefix admits
    // start == row_count as the empty suffix; limit must be positive.
    const auto size = static_cast<std::int64_t>(episode.samples.size());
    if (start < 0 || start > size)
        throw std::invalid_argument(
            "encode_table start is outside the retained extent");
    if (limit < 1)
        throw std::invalid_argument("encode_table limit must be positive");
    const auto last = start + std::min(limit, size - start);
    const std::span<const SamplePtr> rows =
        std::span<const SamplePtr>(episode.samples)
            .subspan(static_cast<std::size_t>(start),
                     static_cast<std::size_t>(last - start));

    std::map<std::string, Piece> emitted;
    std::int64_t count = 0;
    if (parsed.kind == "physics" && parsed.table == "physics") {
        check_columns(parsed, kPhysicsColumns);
        for (const std::string_view name : kPhysicsColumns)
            emitted.emplace(name, emit_physics_column(rows, name));
        count = static_cast<std::int64_t>(rows.size());
    } else if (parsed.kind == "forces" && parsed.table == "forces") {
        check_columns(parsed, kForcesColumns);
        auto [row_count, pieces] = emit_forces(rows);
        count = row_count;
        emitted = std::move(pieces);
    } else if (parsed.kind == "channels") {
        count = emit_channels(rows, parsed, emitted);
    } else {
        throw std::invalid_argument(
            "encode_table: unsupported plan kind '" + parsed.kind +
            "' for table '" + parsed.table + "'");
    }

    const nb::dict pieces;
    for (auto &[name, piece] : emitted)
        pieces[nb::str(name.c_str(), name.size())] = to_python(piece);
    return {count, pieces};
}

nb::list applied_command_rows(const OwnedNativeEpisode &episode,
                              std::int64_t start, std::int64_t limit) {
    // Same slice domain as encode_table: the retained prefix admits
    // start == row_count as the empty suffix; limit must be positive.
    const auto size = static_cast<std::int64_t>(episode.samples.size());
    if (start < 0 || start > size)
        throw std::invalid_argument(
            "applied_rows start is outside the retained extent");
    if (limit < 1)
        throw std::invalid_argument("applied_rows limit must be positive");
    const auto last = start + std::min(limit, size - start);

    // application_id counts changes, so seeding replays rows [0, start):
    // the walk yields both the previous emitted tuple and the emitted
    // count — identical to the Python mirror's carried state.
    std::int64_t application_id = 0;
    AppliedTuple previous;
    nb::list out;
    for (std::int64_t i = 0; i < last; ++i) {
        const PhysicalSampleData &sample = *episode.samples[
            static_cast<std::size_t>(i)];
        const AppliedTuple actual = applied_tuple(sample);
        if (previous.engaged && applied_equal(previous, actual)) continue;
        if (i >= start) {
            static constexpr std::array<const char *, 3> kKeys{
                "front_brake_demand", "rear_brake_demand",
                "rollback_brake_demand"};
            nb::dict row;
            row["application_id"] = nb::int_(application_id);
            row["time_s"] = nb::float_(sample.time_s);
            row["step"] = nb::int_(sample.interval_id);
            row["control"] = wire_to_python(*actual.control);
            for (std::size_t k = 0; k < kKeys.size(); ++k)
                if (actual.demands[k] != nullptr)
                    row[kKeys[k]] = wire_to_python(*actual.demands[k]);
            out.append(row);
        }
        previous = actual;
        ++application_id;
    }
    return out;
}

} // namespace runtime
