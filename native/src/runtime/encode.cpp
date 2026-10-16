// Plan executor for the columnar recording encoder — the Python-free
// core (task T11). The Python plan compiler
// (recording/native_encode.py::compile_plan) stays the source of truth
// for which tables are eligible; this side validates the staged plan
// document (bike::io::Value — encode_binding.cpp materializes the Python
// plan into it; native consumers stage JSON/Value documents directly) and
// runs the fixed per-kind emitters over retained PhysicalSampleData.
// Emitted buffers copy retained values verbatim — including NaN payloads,
// ±inf and -0.0 — so pieces_to_arrow assembles a RecordBatch
// bitwise-identical to arrow_batch(encode_rows(...)). The held-GIL Piece
// boxing and the nb-facing adapters live in encode_binding.cpp; this TU
// reads no Python object.
#include "encode.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <set>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <variant>

#include "value_readers.hpp"

namespace runtime {

namespace bio = bike::io;

namespace {

// compile_plan emits {'table', 'kind'} plus an optional 'columns' list for
// physics/forces; a channels plan adds 'roots', 'all_roots', 'columns' (per
// leaf) and 'masks'. physics/forces columns are fixed emitters; a provided
// list that does not name exactly the fixed set is rejected rather than
// silently narrowed. The parsed node shapes — EncodePlanColumn,
// EncodePlanMask, EncodePlan, DerivedEventPlan — are declared in
// encode.hpp with the piece/public record types.

constexpr std::array<std::string_view, 6> kPhysicsColumns{
    "interval_id", "time_s", "interval_end_s", "dt_s", "qpos", "qvel"};
constexpr std::array<std::string_view, 4> kForcesColumns{
    "interval_id", "component", "qfrc", "power_w"};

// constexpr array — a std::set initializer could throw at static init.
constexpr std::array<std::string_view, 11> kLeafDtypes{
    "double", "bigint", "boolean", "varchar",
    "double[]", "bigint[]", "varchar[]",
    "map<double>", "map<boolean>", "map<varchar>", "map<double[]>"};

bool is_leaf_dtype(const std::string &dtype) {
    return std::ranges::find(kLeafDtypes, dtype) != kLeafDtypes.end();
}

// The recursive member-carrying dtypes: their plan nodes keep `children`.
bool is_struct_dtype(std::string_view dtype) {
    return dtype == "struct" || dtype == "struct[]" ||
           dtype == "map<struct>";
}

// The container terminals _distribute lets absorb the dict members deeper
// fields did not claim; scalar/x[] terminals reject the same remainder.
bool is_container_dtype(std::string_view dtype) {
    return dtype.starts_with("map<") || dtype == "struct" ||
           dtype == "struct[]";
}

// Definition lives with the emitters; the plan index pass joins paths.
std::string join_path(std::span<const std::string> path);

// The routed-member child under `node`, or EncodePlan::kNoNode — binary
// search over the node's name-sorted edges. Struct member_lookup lists
// share the sorted-pair shape but keep their own equal-run scan.
std::size_t path_edge(const EncodePlan::PathNode &node,
                      std::string_view member) {
    const auto at = std::ranges::lower_bound(
        node.edges, member, std::less<>{},
        &std::pair<std::string_view, std::size_t>::first);
    return at != node.edges.end() && at->first == member
        ? at->second : EncodePlan::kNoNode;
}

// The trie node for `path`, creating any missing node along the way.
// Edge keys view the path's std::string members — the plan outlives every
// lookup, and a parent is always created before its children.
std::size_t path_extend(EncodePlan &plan, std::span<const std::string> path) {
    std::size_t node = 0;
    for (const std::string &member : path) {
        const std::size_t child = path_edge(plan.path_nodes[node], member);
        if (child != EncodePlan::kNoNode) {
            node = child;
            continue;
        }
        // emplace_back can reallocate path_nodes: index, never reference.
        const std::size_t fresh = plan.path_nodes.size();
        plan.path_nodes.emplace_back();
        auto &edges = plan.path_nodes[node].edges;
        edges.insert(std::ranges::lower_bound(
                         edges, member, std::less<>{},
                         &std::pair<std::string_view, std::size_t>::first),
                     {std::string_view(member), fresh});
        node = fresh;
    }
    return node;
}

// The trie node for `path`, or kNoNode when some member never routed.
std::size_t path_find(const EncodePlan &plan,
                      std::span<const std::string> path) {
    std::size_t node = 0;
    for (const std::string &member : path) {
        node = path_edge(plan.path_nodes[node], member);
        if (node == EncodePlan::kNoNode) return EncodePlan::kNoNode;
    }
    return node;
}

// NOLINTNEXTLINE(misc-no-recursion) children mirror the finite plan tree
void index_member_lookup(EncodePlanColumn &node) {
    if (!node.children.empty()) {
        node.member_lookup.reserve(node.children.size());
        for (std::size_t i = 0; i < node.children.size(); ++i)
            node.member_lookup.emplace_back(node.children[i].name, i);
        std::ranges::sort(
            node.member_lookup, std::less<>{},
            &std::pair<std::string_view, std::size_t>::first);
    }
    for (EncodePlanColumn &child : node.children) index_member_lookup(child);
}

// Compile the row-loop handles the emitters use: the path trie over leaf
// paths plus claimed roots, each column's trie node/label/deeper-owned
// set, each mask's trie node (kNoNode falls back to walk_path — a plan
// may name a mask path no column claims), and the member lookup tables.
void index_channels_plan(EncodePlan &parsed) {
    for (const std::string &root : parsed.roots)
        parsed.root_nodes.push_back(
            path_extend(parsed, std::span<const std::string>{&root, 1}));
    parsed.column_nodes.reserve(parsed.leaf_columns.size());
    parsed.column_labels.reserve(parsed.leaf_columns.size());
    parsed.column_owned.reserve(parsed.leaf_columns.size());
    for (std::size_t c = 0; c < parsed.leaf_columns.size(); ++c) {
        EncodePlanColumn &column = parsed.leaf_columns[c];
        const std::size_t node =
            path_extend(parsed, std::span<const std::string>(column.path));
        parsed.path_nodes[node].leaf = c;
        parsed.column_nodes.push_back(node);
        parsed.column_labels.push_back(join_path(column.path));
        const std::set<std::string> *owned = nullptr;
        if (column.path.size() == 1) {
            const auto routed = parsed.deeper.find(column.path);
            if (routed != parsed.deeper.end()) owned = &routed->second;
        }
        parsed.column_owned.push_back(owned);
        index_member_lookup(column);
    }
    parsed.mask_nodes.reserve(parsed.masks.size());
    for (const EncodePlanMask &mask : parsed.masks)
        parsed.mask_nodes.push_back(
            path_find(parsed, std::span<const std::string>(mask.path)));
}

// NOLINTNEXTLINE(misc-no-recursion) children mirror the finite plan tree
EncodePlanColumn parse_column(const bio::Value &item) {
    const vread::Dict dict{
        .value = &vread::mapping(vread::Node{&item}, "encode plan.columns[]"),
        .path = "encode plan.columns[]"};
    vread::exact(dict, vread::keys("name", "dtype", "path", "optional"),
                 vread::keys("shape", "except_keys", "convert", "children"));
    EncodePlanColumn column;
    column.name = vread::string(vread::field(dict, "name"),
                                dict.child("name"));
    column.dtype = vread::string(vread::field(dict, "dtype"),
                                 dict.child("dtype"));
    if (!is_leaf_dtype(column.dtype) && !is_struct_dtype(column.dtype))
        throw std::invalid_argument("encode plan.columns: unsupported "
                                    "channels dtype '" + column.dtype + "'");
    for (const bio::Value &part :
         vread::sequence(vread::field(dict, "path"), dict.child("path")))
        column.path.push_back(
            vread::string(vread::Node{&part}, dict.child("path")));
    if (column.path.empty())
        throw std::invalid_argument("encode plan.columns: empty path");
    column.optional = vread::boolean(vread::field(dict, "optional"),
                                     dict.child("optional"));
    if (dict.contains("shape") && !vread::is_none(dict["shape"])) {
        const bio::Value::Array &dims =
            vread::sequence(dict["shape"], dict.child("shape"));
        if (dims.size() != 1)
            throw std::invalid_argument(
                "encode plan.columns: shape must name one dimension");
        for (const bio::Value &dim : dims)
            column.shape =
                vread::integer(vread::Node{&dim}, dict.child("shape"));
    }
    if (dict.contains("except_keys"))
        for (const bio::Value &key : vread::sequence(
                 dict["except_keys"], dict.child("except_keys")))
            column.except_keys.push_back(
                vread::string(vread::Node{&key},
                              dict.child("except_keys")));
    if (dict.contains("convert") && !vread::is_none(dict["convert"])) {
        column.convert = vread::string(dict["convert"],
                                       dict.child("convert"));
        if (column.convert != "verdict")
            throw std::invalid_argument("encode plan.columns: unknown "
                                        "convert '" + column.convert + "'");
    }
    if (dict.contains("children"))
        for (const bio::Value &child : vread::sequence(
                 dict["children"], dict.child("children")))
            column.children.push_back(parse_column(child));
    // Member schemas belong to the struct dtypes only; a container node
    // without members would emit a childless struct Arrow never declares.
    if (is_struct_dtype(column.dtype) == column.children.empty())
        throw std::invalid_argument(
            "encode plan.columns: '" + column.name + "' children do not "
            "match dtype '" + column.dtype + "'");
    // Child nodes address struct members, not catalog paths: each child's
    // path is exactly its member name.
    for (const EncodePlanColumn &child : column.children)
        if (child.path.size() != 1 || child.path.front() != child.name)
            throw std::invalid_argument(
                "encode plan.columns: '" + column.name +
                "' child paths must be single member names");
    return column;
}

EncodePlanMask parse_mask(const bio::Value &item) {
    const vread::Dict dict{
        .value = &vread::mapping(vread::Node{&item}, "encode plan.masks[]"),
        .path = "encode plan.masks[]"};
    vread::exact(dict, vread::keys("name", "path", "op"));
    EncodePlanMask mask;
    mask.name = vread::string(vread::field(dict, "name"),
                              dict.child("name"));
    for (const bio::Value &part :
         vread::sequence(vread::field(dict, "path"), dict.child("path")))
        mask.path.push_back(
            vread::string(vread::Node{&part}, dict.child("path")));
    if (mask.path.empty())
        throw std::invalid_argument("encode plan.masks: empty path");
    const std::string op =
        vread::string(vread::field(dict, "op"), dict.child("op"));
    if (op == "present") mask.op = EncodePlanMask::Op::present;
    else if (op == "valid") mask.op = EncodePlanMask::Op::valid;
    else throw std::invalid_argument("encode plan.masks: unknown op '" +
                                     op + "'");
    return mask;
}

void parse_channels_plan(const vread::Dict &dict, EncodePlan &parsed) {
    vread::exact(dict, vread::keys("table", "kind", "roots", "all_roots",
                                   "columns", "masks"));
    for (const bio::Value &item :
         vread::sequence(dict["columns"], "encode plan.columns"))
        parsed.leaf_columns.push_back(parse_column(item));
    for (const bio::Value &item :
         vread::sequence(dict["masks"], "encode plan.masks"))
        parsed.masks.push_back(parse_mask(item));
    for (const bio::Value &item :
         vread::sequence(dict["roots"], "encode plan.roots"))
        parsed.roots.push_back(
            vread::string(vread::Node{&item}, "encode plan.roots"));
    for (const bio::Value &item :
         vread::sequence(dict["all_roots"], "encode plan.all_roots"))
        parsed.all_roots.insert(
            vread::string(vread::Node{&item}, "encode plan.all_roots"));
    const std::set<std::string> claims(parsed.roots.begin(),
                                       parsed.roots.end());
    for (std::size_t c = 0; c < parsed.leaf_columns.size(); ++c) {
        const EncodePlanColumn &column = parsed.leaf_columns[c];
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
    for (EncodePlanColumn &column : parsed.leaf_columns) {
        for (std::size_t depth = 2;
             depth < column.path.size() && !column.shadowed; ++depth)
            column.shadowed = parsed.leaf_paths.contains(
                std::vector<std::string>(
                    column.path.begin(),
                    column.path.begin() + static_cast<std::ptrdiff_t>(depth)));
        // A container terminal at a claimed root receives the remainder
        // left by deeper sibling routing; except_keys is the catalog's
        // declaration of that ownership, so the plan must agree with the
        // routing index or the two encoders would split the members
        // differently. (Routing only strips at depth-1 terminals — the
        // root-level _distribute call.)
        if (column.path.size() == 1 &&
            is_container_dtype(column.dtype)) {
            const auto owned = parsed.deeper.find(column.path);
            const std::set<std::string> routed =
                owned == parsed.deeper.end() ? std::set<std::string>{}
                                             : owned->second;
            if (routed != std::set<std::string>(column.except_keys.begin(),
                                                column.except_keys.end()))
                throw std::invalid_argument(
                    "encode plan.columns: '" + column.name +
                    "' except_keys do not name the deeper sibling members");
        }
    }
    // Resolve each leaf's absent-or-null prefix chain to its valid-mask
    // slot; the plan must carry every mask the exemption consults.
    std::map<std::vector<std::string>, std::size_t> valid_masks;
    for (std::size_t j = 0; j < parsed.masks.size(); ++j)
        if (parsed.masks[j].op == EncodePlanMask::Op::valid)
            valid_masks.emplace(parsed.masks[j].path, j);
    for (EncodePlanColumn &column : parsed.leaf_columns) {
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
    index_channels_plan(parsed);
}

} // namespace

EncodePlan parse_encode_plan(const bio::Value &plan) {
    const vread::Dict dict{
        .value = &vread::mapping(vread::Node{&plan}, "encode plan"),
        .path = "encode plan"};
    EncodePlan parsed;
    parsed.table =
        vread::string(vread::field(dict, "table"), "encode plan.table");
    parsed.kind =
        vread::string(vread::field(dict, "kind"), "encode plan.kind");
    if (parsed.kind == "channels") {
        parse_channels_plan(dict, parsed);
        return parsed;
    }
    vread::exact(dict, vread::keys("table", "kind"), vread::keys("columns"));
    if (dict.contains("columns")) {
        parsed.has_columns = true;
        for (const bio::Value &item :
             vread::sequence(dict["columns"], "encode plan.columns"))
            parsed.columns.emplace_back(
                vread::string(vread::Node{&item}, "encode plan.columns"));
    }
    return parsed;
}

namespace {

void check_columns(const EncodePlan &plan,
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

// Deferred error-path label: a chain of (parent, segment) links that
// materializes only when an error message reads it — happy-path rows
// never build the text. Segment renderings mirror the oracle's f-strings:
// `text` — the root's whole string, `member` — '.member',
// `index` — '[i]', `key` — "['key']".
struct Label {
    enum class Seg : std::uint8_t { text, member, index, key };
    const Label *parent;
    std::string_view text;
    std::size_t index;
    Seg seg;

    // NOLINTNEXTLINE(misc-no-recursion) bounded by the finite label chain
    [[nodiscard]] std::string str() const {
        std::string out =
            parent != nullptr ? parent->str() : std::string();
        switch (seg) {
            case Seg::text:
                out += text;
                break;
            case Seg::member:
                out += '.';
                out += text;
                break;
            case Seg::key:
                out += "['";
                out += text;
                out += "']";
                break;
            case Seg::index: {
                std::array<char, 20> digits{};
                char *const end =
                    std::to_chars(digits.data(), digits.end(), index).ptr;
                out += '[';
                out.append(digits.data(),
                           static_cast<std::size_t>(
                               std::distance(digits.data(), end)));
                out += ']';
                break;
            }
        }
        return out;
    }
};

// 'label.member' — the '.member' segment the old member_label produced.
Label member_of(const Label &parent, std::string_view member) {
    return {.parent = &parent, .text = member, .index = 0,
            .seg = Label::Seg::member};
}
// 'label[i]' — the per-element segment _convert renders for lists.
Label index_of(const Label &parent, std::size_t i) {
    return {.parent = &parent, .text = {}, .index = i,
            .seg = Label::Seg::index};
}
// "label['key']" — the map-entry segment the oracle renders.
Label key_of(const Label &parent, std::string_view key) {
    return {.parent = &parent, .text = key, .index = 0,
            .seg = Label::Seg::key};
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
void distribute_validate(const EncodePlan &plan, std::size_t node_id,
                         const Wire &payload, const Label &label) {
    const EncodePlan::PathNode &node = plan.path_nodes[node_id];
    if (sample_wire::is_none(payload)) return;
    const auto *object = std::get_if<WireObject>(&payload.value);
    if (object == nullptr) {
        // A scalar/list payload under a non-leaf prefix mirrors the oracle's
        // "unrouted" rejection; leaf prefixes pass it to the leaf walk.
        if (node.leaf == EncodePlan::kNoNode)
            throw std::invalid_argument(
                label.str() + ": no reviewed field accepts the payload");
        return;
    }
    std::vector<std::string_view> unrouted;
    for (const auto &[member, value] : *object) {
        const std::size_t child = path_edge(node, member);
        if (child == EncodePlan::kNoNode) {
            unrouted.push_back(member);
            continue;
        }
        const bool child_leaf =
            plan.path_nodes[child].leaf != EncodePlan::kNoNode;
        if (!child_leaf && !sample_wire::is_none(value) &&
            !std::holds_alternative<WireObject>(value.value))
            throw std::invalid_argument(
                member_of(label, member).str() +
                ": no reviewed field accepts the payload");
        if (!child_leaf)
            distribute_validate(plan, child, value,
                                member_of(label, member));
    }
    if (unrouted.empty()) return;
    // _distribute remainder routing: a map/struct terminal absorbs the
    // members deeper sibling fields did not claim (the leaf walk reads
    // them back out of the same dict); every other prefix names and
    // rejects the leftover members.
    if (node.leaf == EncodePlan::kNoNode ||
        !is_container_dtype(plan.leaf_columns[node.leaf].dtype))
        throw std::invalid_argument(label.str() +
                                    ": unknown channel members " +
                                    quoted_list(std::move(unrouted)));
}

// Slot count of a piece — scalars count payload lanes, list/map count
// offsets (one end mark per slot beyond the seed), and a structure counts
// member slots through any child: member columns and their `_present`
// companions advance one slot per struct value, so every child agrees.
// NOLINTNEXTLINE(misc-no-recursion) structure pieces nest finitely
std::size_t piece_rows(const Piece &piece) {
    switch (piece.kind) {
        case Piece::Kind::scalar_d: return piece.d.size();
        case Piece::Kind::scalar_i: return piece.i.size();
        case Piece::Kind::scalar_b: return piece.b.size();
        case Piece::Kind::scalar_s: return piece.s.size();
        case Piece::Kind::list_d:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s:
        case Piece::Kind::map:
            return piece.offsets.size() - 1;
        case Piece::Kind::structure:
            return piece.children.empty()
                ? 0 : piece_rows(piece.children.begin()->second);
    }
    return 0;
}

// One slot of placeholder payload behind an explicit validity bit —
// `valid` 0 writes a NULL (absent member, explicit None, exempted
// required leaf), 1 a *valid default*: the state Arrow's `pa.array`
// infers for the children of a NULL struct slot (zero scalars, empty
// spans and default-valid nested structs with every companion False —
// probed against pa.array([{...}, None], type=struct<...>)). A list or
// map slot contributes an empty span either way; the entry/element
// payloads never see it. A 'struct' slot fills every member recursively.
// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the finite plan tree
void append_placeholder(const EncodePlanColumn &node, Piece &piece, char valid) {
    switch (piece.kind) {
        case Piece::Kind::scalar_d: piece.d.push_back(0.0); break;
        case Piece::Kind::scalar_i: piece.i.push_back(0); break;
        case Piece::Kind::scalar_b: piece.b.push_back(0); break;
        case Piece::Kind::scalar_s: piece.s.emplace_back(); break;
        case Piece::Kind::list_d:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s:
            // list_* offsets count flat payload elements.
            piece.offsets.push_back(static_cast<std::int32_t>(
                piece.kind == Piece::Kind::list_i ? piece.i.size()
                : piece.kind == Piece::Kind::list_d ? piece.d.size()
                : piece.s.size()));
            break;
        case Piece::Kind::map:
            // map offsets count entries, which the keys vector tracks
            // one-to-one; a NULL/empty map span contributes no entries.
            piece.offsets.push_back(
                static_cast<std::int32_t>(piece.s.size()));
            break;
        case Piece::Kind::structure:
            if (node.dtype == "struct[]") {
                piece.offsets.push_back(static_cast<std::int32_t>(
                    piece.children.empty() ? 0 :
                        piece_rows(piece.children.begin()->second)));
                break;
            }
            if (piece.member_slots.size() == node.children.size()) {
                for (std::size_t i = 0; i < node.children.size(); ++i) {
                    const EncodePlanColumn &child = node.children[i];
                    append_placeholder(child, *piece.member_slots[i].member,
                                       1);
                    if (child.optional)
                        piece.member_slots[i].present->b.push_back(0);
                }
                break;
            }
            for (const EncodePlanColumn &child : node.children) {
                append_placeholder(child, piece.children[child.name], 1);
                if (child.optional)
                    piece.children[child.name + "_present"].b.push_back(0);
            }
            break;
    }
    piece.valid.push_back(valid);
}

void append_null(const EncodePlanColumn &node, Piece &piece) {
    append_placeholder(node, piece, 0);
}

// One element of an x[] leaf, mirroring the member dtype's per-element
// conversion boundary (encoding.py::_convert_scalar inside _convert).
void append_list_element(Piece &piece, const Wire &element,
                         const Label &label) {
    if (piece.kind == Piece::Kind::list_d) {
        const auto *real = std::get_if<double>(&element.value);
        const auto *whole = std::get_if<std::int64_t>(&element.value);
        if (real != nullptr) piece.d.push_back(*real);
        else if (whole != nullptr)
            piece.d.push_back(static_cast<double>(*whole));
        else
            throw std::invalid_argument(label.str() +
                                        ": expected a real number");
        return;
    }
    if (piece.kind == Piece::Kind::list_i) {
        if (const auto *whole = std::get_if<std::int64_t>(&element.value))
            piece.i.push_back(*whole);
        else
            throw std::invalid_argument(label.str() + ": expected an integer");
        return;
    }
    const auto *text = std::get_if<std::string>(&element.value);
    if (text == nullptr)
        throw std::invalid_argument(label.str() + ": expected a string");
    piece.s.push_back(*text);
}

// UTF-8 lead byte -> code-point byte width, validating the continuation
// bytes so a malformed payload rejects instead of mis-splitting (wire
// strings are UTF-8 by construction, but an authored C++ row could carry
// anything; Python only ever iterates valid str).
std::size_t utf8_width(char lead, const std::string &text,
                       std::size_t at, const Label &label) {
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
        throw std::invalid_argument(label.str() + ": invalid UTF-8");
    return width;
}

// A map entry's member value converted at the map's member dtype — the
// encoding.py::_convert recursion inside the map<K> branch. items is the
// member piece the row loop allocated for this column.
void append_map_member(const EncodePlanColumn &column, Piece &items,
                       const Wire &member, const Label &label) {
    switch (items.kind) {
        case Piece::Kind::scalar_d: {
            const auto *real = std::get_if<double>(&member.value);
            const auto *whole = std::get_if<std::int64_t>(&member.value);
            if (real != nullptr) items.d.push_back(*real);
            else if (whole != nullptr)
                items.d.push_back(static_cast<double>(*whole));
            else
                throw std::invalid_argument(
                    label.str() + ": expected a real number");
            return;
        }
        case Piece::Kind::scalar_b:
            if (const auto *flag = std::get_if<bool>(&member.value)) {
                items.b.push_back(static_cast<char>(*flag));
                return;
            }
            throw std::invalid_argument(label.str() + ": expected a bool");
        case Piece::Kind::scalar_s:
            if (const auto *text = std::get_if<std::string>(&member.value)) {
                items.s.push_back(*text);
                return;
            }
            throw std::invalid_argument(label.str() + ": expected a string");
        case Piece::Kind::list_d: {
            // map<double[]> members walk the same iterable lanes as an
            // x[] leaf — array elements, str code points and dict keys —
            // and hit the same per-element and shape gates; an empty
            // iterable stays an empty member list.
            std::size_t count = 0;
            const auto element_label = [&]() {
                return index_of(label, count);
            };
            if (const auto *values =
                    std::get_if<WireArray>(&member.value)) {
                for (const Wire &element : *values) {
                    append_list_element(items, element, element_label());
                    ++count;
                }
            } else if (const auto *text =
                           std::get_if<std::string>(&member.value)) {
                for (std::size_t at = 0; at < text->size(); ++count) {
                    const std::size_t width =
                        utf8_width((*text)[at], *text, at, label);
                    const Wire element{text->substr(at, width)};
                    append_list_element(items, element, element_label());
                    at += width;
                }
            } else if (const auto *bag =
                           std::get_if<WireObject>(&member.value)) {
                for (const auto &[name, ignored] : *bag) {
                    const Wire element{name};
                    append_list_element(items, element, element_label());
                    ++count;
                }
            } else {
                throw std::invalid_argument(
                    label.str() + ": expected a sequence");
            }
            if (column.shape >= 0 &&
                std::cmp_not_equal(count, column.shape))
                throw std::invalid_argument(
                    label.str() + ": expected " +
                    std::to_string(column.shape) + " elements, got " +
                    std::to_string(count));
            items.offsets.push_back(
                static_cast<std::int32_t>(items.d.size()));
            return;
        }
        case Piece::Kind::scalar_i:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s:
        case Piece::Kind::structure:
        case Piece::Kind::map:
            break;
    }
    throw std::invalid_argument(
        "encode: unsupported map member piece kind");
}

// Members a deeper sibling leaf owns — routing strips them before the
// container terminal at this path sees the dict (encoding.py::_distribute
// pops them out of the remainder). `owned` is the routing index's member
// set at the leaf's own path, nullptr when no deeper field claims below.
bool owned_member(const std::set<std::string> *owned,
                  const std::string &member) {
    return owned != nullptr && owned->contains(member);
}

// The routed remainder of a container leaf: a dict whose members are all
// owned by deeper siblings encodes the leaf absent rather than an empty
// container (encoding.py::_distribute assigns only a non-empty
// remainder). This binds at every depth a container terminal sits at — a
// depth-1 root dict is the common case.
bool remainder_nonempty(const std::set<std::string> *owned,
                        const WireObject &object) {
    for (const auto &[member, ignored] : object)
        if (!owned_member(owned, member)) return true;
    return false;
}

// Converts one resolved (non-null) leaf value into its piece; lists append
// their elements plus the end offset. Mirrors encoding.py::_convert's
// dtype/shape gates; a dict routed to a deeper leaf still converts, so no
// Mapping check precedes the dtype gates. `owned` carries the member set
// deeper sibling fields claim at this leaf's path (nullptr nested or when
// nothing is claimed); container leaves skip owned members.
void append_leaf(const EncodePlanColumn &column, const Wire &value, Piece &piece,
                 const Label &label,
                 const std::set<std::string> *owned) {
    switch (piece.kind) {
        case Piece::Kind::scalar_d: {
            const auto *real = std::get_if<double>(&value.value);
            const auto *whole = std::get_if<std::int64_t>(&value.value);
            if (real != nullptr) piece.d.push_back(*real);
            else if (whole != nullptr)
                piece.d.push_back(static_cast<double>(*whole));
            else
                throw std::invalid_argument(
                    label.str() + ": expected a real number");
            return;
        }
        case Piece::Kind::scalar_i:
            if (const auto *whole = std::get_if<std::int64_t>(&value.value)) {
                piece.i.push_back(*whole);
                return;
            }
            throw std::invalid_argument(label.str() + ": expected an integer");
        case Piece::Kind::scalar_b:
            if (const auto *flag = std::get_if<bool>(&value.value)) {
                piece.b.push_back(static_cast<char>(*flag));
                return;
            }
            throw std::invalid_argument(label.str() + ": expected a bool");
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
            throw std::invalid_argument(label.str() + ": expected a string");
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
                return index_of(label, count);
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
                    label.str() + ": expected a sequence");
            }
            if (column.shape >= 0 &&
                std::cmp_not_equal(count, column.shape))
                throw std::invalid_argument(
                    label.str() + ": expected " +
                    std::to_string(column.shape) + " elements, got " +
                    std::to_string(count));
            piece.offsets.push_back(static_cast<std::int32_t>(
                piece.kind == Piece::Kind::list_i ? piece.i.size()
                : piece.kind == Piece::Kind::list_s ? piece.s.size()
                : piece.d.size()));
            return;
        }
        case Piece::Kind::map: {
            // _convert's map branch: the payload must be a dict, every
            // remaining key must be a nonempty string (_check_map_keys
            // runs ahead of any member conversion), and each member
            // converts at the member dtype in dict order. Members a
            // deeper sibling leaf owns are not entries — routing already
            // stripped them from the remainder this leaf sees.
            const auto *object = std::get_if<WireObject>(&value.value);
            if (object == nullptr)
                throw std::invalid_argument(label.str() + ": expected a dict");
            for (const auto &[member, ignored] : *object)
                if (!owned_member(owned, member) && member.empty())
                    throw std::invalid_argument(
                        label.str() + ": map keys must be nonempty strings");
            for (const auto &[member, entry] : *object) {
                if (owned_member(owned, member)) continue;
                piece.s.push_back(member);
                append_map_member(column, *piece.items, entry,
                                  key_of(label, member));
            }
            piece.offsets.push_back(
                static_cast<std::int32_t>(piece.s.size()));
            return;
        }
        case Piece::Kind::structure:
            // struct/struct[] dispatch is append_node's, not append_leaf's.
            throw std::invalid_argument(
                "encode: append_leaf reached a structure piece");
    }
}

// NOLINTNEXTLINE(misc-no-recursion) node tree mirrors the finite plan tree
void append_node(const EncodePlanColumn &node, const Wire &value, Piece &piece,
                 const Label &label,
                 const std::set<std::string> *owned);

// One declared member of a struct value -> its child piece plus its
// `_present` companion. Mirrors encoding.py::_convert_struct's member
// loop: `present` is `child.key in value` — an explicit None reads present
// with a NULL member — and a required member absent-or-null rejects.
// NOLINTNEXTLINE(misc-no-recursion) member tree mirrors the plan tree
void append_member(const EncodePlanColumn &child, const Wire *member,
                   Piece &child_piece, Piece *companion,
                   const Label &label) {
    if (member == nullptr || sample_wire::is_none(*member)) {
        if (!child.optional)
            throw std::invalid_argument(label.str() + ": required");
        append_placeholder(child, child_piece, 0);
        companion->b.push_back(member != nullptr ? 1 : 0);
        return;
    }
    append_node(child, *member, child_piece, label, nullptr);
    child_piece.valid.push_back(1);
    if (companion != nullptr) companion->b.push_back(1);
}

// encoding.py::_convert_struct: dict payload, unknown-member rejection
// ahead of conversion, then every declared member in catalog order.
// `owned` strips members claimed by deeper sibling leaves (remainder
// routing); nested calls always pass nullptr — routing ends at the leaf.
// NOLINTNEXTLINE(misc-no-recursion) member tree mirrors the plan tree
void convert_struct(const EncodePlanColumn &node, const Wire &value, Piece &piece,
                    const Label &label,
                    const std::set<std::string> *owned) {
    const auto *object = std::get_if<WireObject>(&value.value);
    if (object == nullptr)
        throw std::invalid_argument(label.str() + ": expected a dict");
    // One pass over the payload members collects the unknowns and resolves
    // every declared child's wire through the plan's name index — the
    // first wire member wins each slot, exactly the sample_wire::find
    // scan each child used to run. A member owned by a deeper sibling
    // leaf is stripped from the remainder the container sees — for member
    // lookup it is absent.
    std::vector<std::string_view> unknown;
    std::vector<const Wire *> members(node.children.size(), nullptr);
    for (const auto &[member, item] : *object) {
        if (owned_member(owned, member)) continue;
        const auto at = std::ranges::lower_bound(
            node.member_lookup, member, std::less<>{},
            &std::pair<std::string_view, std::size_t>::first);
        if (at == node.member_lookup.end() || at->first != member) {
            unknown.push_back(member);
            continue;
        }
        // A duplicated child name (catalogs never declare one, but a
        // hand-built plan can) claims the member for every shared slot.
        for (auto same = at;
             same != node.member_lookup.end() && same->first == member;
             ++same)
            if (members[same->second] == nullptr)
                members[same->second] = &item;
    }
    if (!unknown.empty())
        throw std::invalid_argument(label.str() +
                                    ": unknown struct members " +
                                    quoted_list(std::move(unknown)));
    // structure_piece caches each declared member's piece (and *_present
    // companion) in plan order, so the per-row loop below is pure
    // indexing — the map lookups repeat only for a hand-built piece.
    if (piece.member_slots.size() == node.children.size()) {
        for (std::size_t i = 0; i < node.children.size(); ++i) {
            const EncodePlanColumn &child = node.children[i];
            append_member(child, members[i], *piece.member_slots[i].member,
                          piece.member_slots[i].present,
                          member_of(label, child.name));
        }
        return;
    }
    for (std::size_t i = 0; i < node.children.size(); ++i) {
        const EncodePlanColumn &child = node.children[i];
        Piece *companion = child.optional
            ? &piece.children[child.name + "_present"] : nullptr;
        append_member(child, members[i], piece.children[child.name],
                      companion, member_of(label, child.name));
    }
}

// encoding.py::_convert's `struct[]` branch: every element converts
// through _convert_struct into the shared member columns; this piece's
// offsets close the slot once, like a list leaf.
// NOLINTNEXTLINE(misc-no-recursion) element tree mirrors the plan tree
void convert_struct_list(const EncodePlanColumn &node, const Wire &value,
                         Piece &piece, const Label &label) {
    const auto *items = std::get_if<WireArray>(&value.value);
    if (items == nullptr)
        throw std::invalid_argument(label.str() + ": expected a list");
    for (std::size_t index = 0; index < items->size(); ++index)
        convert_struct(node, (*items)[index], piece,
                       index_of(label, index), nullptr);
    piece.offsets.push_back(static_cast<std::int32_t>(
        piece.children.empty() ? 0 :
            piece_rows(piece.children.begin()->second)));
}

// A `map<struct>` slot: `_check_map_keys` ahead of member conversion
// (owned members skipped — they are columns, not entries), then each entry
// converts through the shared `items` struct in authored key order.
// NOLINTNEXTLINE(misc-no-recursion) entry tree mirrors the plan tree
void convert_map_struct(const EncodePlanColumn &node, const Wire &value,
                        Piece &piece, const Label &label,
                        const std::set<std::string> *owned) {
    const auto *object = std::get_if<WireObject>(&value.value);
    if (object == nullptr)
        throw std::invalid_argument(label.str() + ": expected a dict");
    for (const auto &[member, ignored] : *object)
        if (!owned_member(owned, member) && member.empty())
            throw std::invalid_argument(
                label.str() + ": map keys must be nonempty strings");
    for (const auto &[member, entry] : *object) {
        if (owned_member(owned, member)) continue;
        piece.s.push_back(member);
        convert_struct(node, entry, *piece.items,
                       member_of(label, member), nullptr);
    }
    piece.offsets.push_back(static_cast<std::int32_t>(piece.s.size()));
}

// NOLINTNEXTLINE(misc-no-recursion) node tree mirrors the finite plan tree
void append_node(const EncodePlanColumn &node, const Wire &value, Piece &piece,
                 const Label &label,
                 const std::set<std::string> *owned) {
    if (piece.kind == Piece::Kind::structure) {
        if (node.dtype == "struct[]") {
            convert_struct_list(node, value, piece, label);
            return;
        }
        convert_struct(node, value, piece, label, owned);
        return;
    }
    if (node.dtype == "map<struct>") {
        convert_map_struct(node, value, piece, label, owned);
        return;
    }
    append_leaf(node, value, piece, label, owned);
}

Piece::Kind leaf_kind(std::string_view dtype) {
    if (dtype == "double") return Piece::Kind::scalar_d;
    if (dtype == "bigint") return Piece::Kind::scalar_i;
    if (dtype == "boolean") return Piece::Kind::scalar_b;
    if (dtype == "varchar") return Piece::Kind::scalar_s;
    if (dtype == "double[]") return Piece::Kind::list_d;
    if (dtype == "bigint[]") return Piece::Kind::list_i;
    if (dtype == "varchar[]") return Piece::Kind::list_s;
    return Piece::Kind::map;   // map<...>; is_leaf_dtype gated the rest
}

// The member-value piece kind inside a map leaf's `items` child.
Piece::Kind map_member_kind(std::string_view dtype) {
    if (dtype == "map<double>") return Piece::Kind::scalar_d;
    if (dtype == "map<boolean>") return Piece::Kind::scalar_b;
    if (dtype == "map<varchar>") return Piece::Kind::scalar_s;
    return Piece::Kind::list_d;  // map<double[]>
}

Piece node_piece(const EncodePlanColumn &node, std::size_t reserve_rows);

// A structure piece seeded from a member list: one member piece per
// child plus a `_present` scalar_b companion per optional child, in
// catalog order. Shared by 'struct'/'struct[]' columns and the entry
// struct that backs a 'map<struct>' piece's `items` — Arrow's layout
// uses one shared entries array per map column.
// NOLINTNEXTLINE(misc-no-recursion) children mirror the finite plan tree
Piece structure_piece(const std::vector<EncodePlanColumn> &children,
                      std::size_t reserve_rows) {
    Piece piece;
    piece.kind = Piece::Kind::structure;
    piece.valid.reserve(reserve_rows);
    piece.member_slots.reserve(children.size());
    for (const EncodePlanColumn &child : children) {
        Piece *member = &piece.children
                             .emplace(child.name, node_piece(child, 0))
                             .first->second;
        Piece *present = nullptr;
        if (child.optional) {
            Piece companion;
            companion.kind = Piece::Kind::scalar_b;
            companion.b.reserve(reserve_rows);
            present = &piece.children
                           .emplace(child.name + "_present",
                                    std::move(companion))
                           .first->second;
        }
        piece.member_slots.push_back(
            {.member = member, .present = present});
    }
    return piece;
}

// The piece tree for one plan node: flat leaves map one-to-one; `struct`
// and `struct[]` are structure pieces (`struct[]` keeps slot offsets on
// the piece itself); `map<struct>` is a map piece whose `items` is the
// shared entry struct.
// NOLINTNEXTLINE(misc-no-recursion) children mirror the finite plan tree
Piece node_piece(const EncodePlanColumn &node, std::size_t reserve_rows) {
    Piece piece;
    if (node.dtype == "struct" || node.dtype == "struct[]") {
        piece = structure_piece(node.children, reserve_rows);
        if (node.dtype == "struct[]") {
            piece.offsets.reserve(reserve_rows + 1);
            piece.offsets.push_back(0);
        }
        return piece;
    }
    if (node.dtype == "map<struct>") {
        piece.kind = Piece::Kind::map;
        piece.offsets.reserve(reserve_rows + 1);
        piece.offsets.push_back(0);
        piece.valid.reserve(reserve_rows);
        piece.items =
            std::make_unique<Piece>(structure_piece(node.children, 0));
        return piece;
    }
    piece.kind = leaf_kind(node.dtype);
    piece.valid.reserve(reserve_rows);
    switch (piece.kind) {
        case Piece::Kind::scalar_d: piece.d.reserve(reserve_rows); break;
        case Piece::Kind::scalar_i: piece.i.reserve(reserve_rows); break;
        case Piece::Kind::scalar_b: piece.b.reserve(reserve_rows); break;
        case Piece::Kind::scalar_s: piece.s.reserve(reserve_rows); break;
        case Piece::Kind::list_d:
        case Piece::Kind::list_i:
        case Piece::Kind::list_s:
            piece.offsets.reserve(reserve_rows + 1);
            piece.offsets.push_back(0);
            break;
        case Piece::Kind::map:
            // keys live in piece.s; the items child carries the
            // member-value payload for every entry across all rows
            // (a list child starts its own offsets span at 0).
            piece.offsets.reserve(reserve_rows + 1);
            piece.offsets.push_back(0);
            piece.items = std::make_unique<Piece>();
            piece.items->kind = map_member_kind(node.dtype);
            if (piece.items->kind == Piece::Kind::list_d)
                piece.items->offsets.push_back(0);
            break;
        case Piece::Kind::structure:
            break;   // unreachable: struct dtypes return above
    }
    return piece;
}

// A row's payload at one path-trie node: the resolved member wire (the
// walk_path terminal — nullptr while the chain is broken, a monostate
// wire when the member resolves to None), the wire's dict view, and
// `complete` — every member along the prefix resolved.
struct ResolvedPath {
    const Wire *wire = nullptr;
    const WireObject *object = nullptr;
    bool complete = false;
};

// One forward pass over the plan's path trie projects a row's channels
// onto every plan path: each present dict's members dispatch once to the
// node's routed children — first wire member wins, matching the
// sample_wire::find scan each leaf walk used to run — replacing a
// per-leaf rescan of the shared prefixes.
void resolve_paths(const EncodePlan &plan, const WireObject &channels,
                   std::vector<ResolvedPath> &resolved) {
    resolved.assign(plan.path_nodes.size(), ResolvedPath{});
    resolved[0].object = &channels;
    resolved[0].complete = true;
    for (std::size_t n = 0; n < plan.path_nodes.size(); ++n) {
        const ResolvedPath &node = resolved[n];
        if (!node.complete || node.object == nullptr) continue;
        for (const auto &[member, value] : *node.object) {
            const std::size_t child = path_edge(plan.path_nodes[n], member);
            if (child != EncodePlan::kNoNode && !resolved[child].complete)
                resolved[child] = {.wire = &value,
                                   .object =
                                       std::get_if<WireObject>(&value.value),
                                   .complete = true};
        }
    }
}

std::int64_t emit_channels(std::span<const SamplePtr> rows,
                           const EncodePlan &plan,
                           std::map<std::string, Piece> &emitted) {
    std::vector<Piece> values(plan.leaf_columns.size());
    std::vector<Piece> presence(plan.leaf_columns.size());
    std::vector<Piece> mask_pieces(plan.masks.size());
    Piece ids;
    ids.kind = Piece::Kind::scalar_i;
    ids.i.reserve(rows.size());
    for (std::size_t c = 0; c < plan.leaf_columns.size(); ++c) {
        values[c] = node_piece(plan.leaf_columns[c], rows.size());
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
    std::vector<ResolvedPath> resolved;
    const Label table_label{.parent = nullptr, .text = plan.table,
                            .index = 0, .seg = Label::Seg::text};
    for (std::size_t row = 0; row < rows.size(); ++row) {
        const PhysicalSampleData &sample = *rows[row];
        ids.i.push_back(sample.interval_id);
        resolve_paths(plan, sample.channels, resolved);
        const Label row_label{.parent = &table_label, .text = {},
                              .index = row, .seg = Label::Seg::index};
        // _view() routes every claimed root's payload first: unknown
        // members and unrouted payloads reject ahead of field conversion.
        for (std::size_t i = 0; i < plan.roots.size(); ++i) {
            const std::size_t node = plan.root_nodes[i];
            if (resolved[node].complete)
                distribute_validate(plan, node, *resolved[node].wire,
                                    member_of(row_label, plan.roots[i]));
        }
        for (std::size_t j = 0; j < plan.masks.size(); ++j) {
            bool complete = false;
            const Wire *node = nullptr;
            if (plan.mask_nodes[j] != EncodePlan::kNoNode) {
                const ResolvedPath &hit = resolved[plan.mask_nodes[j]];
                complete = hit.complete;
                node = hit.wire;
            } else {
                node = walk_path(sample.channels, plan.masks[j].path,
                                 complete);
            }
            const bool valid = complete && node != nullptr &&
                               !sample_wire::is_none(*node);
            mask_valid[j] = static_cast<char>(valid);
            mask_pieces[j].b.push_back(static_cast<char>(
                plan.masks[j].op == EncodePlanMask::Op::present ? complete
                                                        : valid));
        }
        for (std::size_t c = 0; c < plan.leaf_columns.size(); ++c) {
            const EncodePlanColumn &column = plan.leaf_columns[c];
            Piece &piece = values[c];
            const Label label =
                member_of(row_label, plan.column_labels[c]);
            // Members deeper sibling fields claim under this leaf's path
            // never reach its payload — routing strips them before the
            // container sees the remainder. Routing strips only at a
            // depth-1 terminal's own call — the only prefix _distribute
            // visits on a leaf; a deeper container receives its dict
            // verbatim, so members under it stay entries/unknowns. The
            // column's owned set resolves at plan-compile time.
            const std::set<std::string> *owned = plan.column_owned[c];
            // The resolved wire is non-null exactly when the path's
            // member chain resolved, matching walk_path's terminal.
            const Wire *leaf = nullptr;
            if (plan.column_nodes[c] != EncodePlan::kNoNode) {
                leaf = resolved[plan.column_nodes[c]].wire;
            } else {
                bool complete = false;
                leaf = walk_path(sample.channels, column.path, complete);
            }
            // A dict payload at a leaf path routes nothing itself: a
            // scalar/list leaf under a depth-1 claim is absent (non-empty
            // dicts already threw in distribute_validate), while a
            // container leaf owns the routed remainder — absent when
            // every member went to a deeper sibling (or the dict is
            // empty), an empty container only when routing assigned it
            // verbatim at depth > 1. A shadowed leaf sits under a
            // container terminal; routing never reaches it, so it encodes
            // absent on every row.
            bool found = leaf != nullptr && !column.shadowed;
            if (found)
                if (const auto *object =
                        std::get_if<WireObject>(&leaf->value))
                    found = is_container_dtype(column.dtype)
                        ? column.path.size() > 1 ||
                              remainder_nonempty(owned, *object)
                        : column.path.size() > 1;
            if (found && sample_wire::is_none(*leaf)) {
                if (!column.optional)
                    throw std::invalid_argument(
                        label.str() + ": required field may not be null");
                append_null(column, piece);
                presence[c].b.push_back(1);
            } else if (found) {
                append_node(column, *leaf, piece, label, owned);
                piece.valid.push_back(1);
                if (column.optional) presence[c].b.push_back(1);
            } else if (column.optional) {
                append_null(column, piece);
                presence[c].b.push_back(0);
            } else {
                // A required leaf encodes NULL iff an ancestor on its path
                // read absent-or-null this row — encoding.py exempts those
                // so a missing/null parent does not poison its leaves.
                const bool exempt = std::ranges::any_of(
                    column.ancestors,
                    [&](std::size_t j) { return mask_valid[j] == 0; });
                if (!exempt)
                    throw std::invalid_argument(label.str() + ": required");
                append_null(column, piece);
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

// ---------------------------------------------------------------------------
// derived-transition scan (the channel diff inside iter_derived_events)

// The DerivedEventPlan shape is declared in encode.hpp; the label/kind
// bookkeeping below mirrors native.py's _DERIVED_FIELDS handling.
DerivedEventPlan parse_derived_body(const bio::Value &plan) {
    const vread::Dict dict{
        .value = &vread::mapping(vread::Node{&plan}, "derived plan"),
        .path = "derived plan"};
    vread::exact(dict, vread::keys("fields"));
    DerivedEventPlan parsed;
    std::map<std::string, std::size_t> slots;
    for (const bio::Value &item : vread::sequence(
             vread::field(dict, "fields"), dict.child("fields"))) {
        const vread::Dict entry{
            .value = &vread::mapping(vread::Node{&item},
                                     "derived plan.fields[]"),
            .path = "derived plan.fields[]"};
        vread::exact(entry, vread::keys("kind", "path"));
        DerivedEventPlan::Field field;
        field.kind = vread::string(vread::field(entry, "kind"),
                                   entry.child("kind"));
        for (const bio::Value &part : vread::sequence(
                 vread::field(entry, "path"), entry.child("path")))
            field.path.push_back(
                vread::string(vread::Node{&part}, entry.child("path")));
        if (field.path.empty())
            throw std::invalid_argument(
                "derived plan.fields: empty path");
        // '.'.join(path[1:]) or path[0] — an empty join (single-element
        // path or an empty last member) falls back to the root name.
        field.label = join_path(std::span<const std::string>(field.path)
                                    .subspan(1));
        if (field.label.empty()) field.label = field.path.front();
        const auto [slot, inserted] =
            slots.emplace(field.kind, parsed.kinds.size());
        if (inserted) parsed.kinds.push_back(field.kind);
        field.kind_slot = slot->second;
        parsed.fields.push_back(std::move(field));
    }
    return parsed;
}

// One kind's ordered label -> wire pointer view of a row — the
// `current.setdefault(kind, {})[label] = value` fold: first-insertion
// order per label, a repeated (kind, label) field writes through. The
// string_views address the parsed plan, which outlives every scan.
using KindValues = std::vector<std::pair<std::string_view, const Wire *>>;

void tracked_set(KindValues &values, std::string_view label,
                 const Wire &value) {
    for (auto &[name, slot] : values)
        if (name == label) {
            slot = &value;
            return;
        }
    values.emplace_back(label, &value);
}

// previous.get(kind, {}).get(label, _MISSING) — nullptr is the sentinel:
// an absent label always differs from a present current value.
const Wire *tracked_find(const KindValues &values, std::string_view label) {
    for (const auto &[name, value] : values)
        if (name == label) return value;
    return nullptr;
}

// Resolve every tracked path on one retained row; present terminals fold
// into their kind's label map. `current` is cleared, never reallocated.
void resolve_derived(const DerivedEventPlan &plan,
                     const PhysicalSampleData &sample,
                     std::vector<KindValues> &current) {
    for (KindValues &values : current) values.clear();
    for (const DerivedEventPlan::Field &field : plan.fields) {
        bool complete = false;
        const Wire *value =
            walk_path(sample.channels, field.path, complete);
        // complete implies a resolved (possibly null) terminal — an
        // explicit None still tracks, matching _channel_at's present.
        if (complete && value != nullptr)
            tracked_set(current[field.kind_slot], field.label, *value);
    }
}

} // namespace

DerivedEventPlan parse_derived_event_plan(const bio::Value &plan) {
    return parse_derived_body(plan);
}

EncodedTable encode_table_native(const OwnedNativeEpisode &episode,
                                 const EncodePlan &parsed,
                                 std::int64_t start, std::int64_t limit) {
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

    EncodedTable table;
    if (parsed.kind == "physics" && parsed.table == "physics") {
        check_columns(parsed, kPhysicsColumns);
        for (const std::string_view name : kPhysicsColumns)
            table.columns.emplace(name, emit_physics_column(rows, name));
        table.rows = static_cast<std::int64_t>(rows.size());
    } else if (parsed.kind == "forces" && parsed.table == "forces") {
        check_columns(parsed, kForcesColumns);
        auto [row_count, pieces] = emit_forces(rows);
        table.rows = row_count;
        table.columns = std::move(pieces);
    } else if (parsed.kind == "channels") {
        table.rows = emit_channels(rows, parsed, table.columns);
    } else {
        throw std::invalid_argument(
            "encode_table: unsupported plan kind '" + parsed.kind +
            "' for table '" + parsed.table + "'");
    }
    return table;
}

std::vector<AppliedCommandRow> applied_command_rows_native(
    const OwnedNativeEpisode &episode, std::int64_t start,
    std::int64_t limit) {
    // Same slice domain as encode_table_native: the retained prefix
    // admits start == row_count as the empty suffix; limit must be
    // positive.
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
    std::vector<AppliedCommandRow> out;
    for (std::int64_t i = 0; i < last; ++i) {
        const PhysicalSampleData &sample = *episode.samples[
            static_cast<std::size_t>(i)];
        const AppliedTuple actual = applied_tuple(sample);
        if (previous.engaged && applied_equal(previous, actual)) continue;
        if (i >= start) {
            AppliedCommandRow row;
            row.application_id = application_id;
            row.time_s = sample.time_s;
            row.step = sample.interval_id;
            row.control = *actual.control;
            if (actual.demands[0] != nullptr)
                row.front_brake_demand = *actual.demands[0];
            if (actual.demands[1] != nullptr)
                row.rear_brake_demand = *actual.demands[1];
            if (actual.demands[2] != nullptr)
                row.rollback_brake_demand = *actual.demands[2];
            out.push_back(std::move(row));
        }
        previous = actual;
        ++application_id;
    }
    return out;
}

std::vector<DerivedEventRow> derived_event_rows_native(
    const OwnedNativeEpisode &episode, const DerivedEventPlan &parsed,
    std::int64_t start, std::int64_t limit) {
    // Same slice domain as encode_table_native/applied_rows: the retained
    // prefix admits start == row_count as the empty suffix; limit must be
    // positive.
    const auto size = static_cast<std::int64_t>(episode.samples.size());
    if (start < 0 || start > size)
        throw std::invalid_argument(
            "derived_rows start is outside the retained extent");
    if (limit < 1)
        throw std::invalid_argument("derived_rows limit must be positive");
    const auto last = start + std::min(limit, size - start);

    // iter_derived_events carries `previous` across the whole scan, so a
    // windowed call seeds it from row start-1 — chunked calls then emit
    // exactly the full-pass sequence, and the first row of a start-0 call
    // never emits (previous is None until the first row resolves).
    std::vector<KindValues> previous(parsed.kinds.size());
    std::vector<KindValues> current(parsed.kinds.size());
    bool have_previous = false;
    if (start > 0) {
        resolve_derived(parsed,
                        *episode.samples[static_cast<std::size_t>(start - 1)],
                        previous);
        have_previous = true;
    }
    std::vector<DerivedEventRow> out;
    for (std::int64_t i = start; i < last; ++i) {
        const PhysicalSampleData &sample =
            *episode.samples[static_cast<std::size_t>(i)];
        resolve_derived(parsed, sample, current);
        if (have_previous)
            for (std::size_t slot = 0; slot < parsed.kinds.size(); ++slot) {
                DerivedEventRow row;
                for (const auto &[label, value] : current[slot]) {
                    const Wire *before =
                        tracked_find(previous[slot], label);
                    // Python `prev != cur` — an absent previous label
                    // (_MISSING) or a wire difference both record the
                    // change.
                    if (before != nullptr && wire_equal(*before, *value))
                        continue;
                    row.changes.emplace_back(std::string(label), *value);
                }
                if (row.changes.empty()) continue;
                row.interval_id = sample.interval_id;
                row.derived_kind = parsed.kinds[slot];
                out.push_back(std::move(row));
            }
        std::swap(previous, current);
        have_previous = true;
    }
    return out;
}

} // namespace runtime
