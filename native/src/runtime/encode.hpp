// Catalog-plan column projector for the native recording encoder — the
// Python-free half (task T11). recording/native_encode.py::compile_plan
// stays the source of truth for which tables are eligible; this side
// validates a staged plan document (bike::io::Value — the binding
// materializes the Python plan into it first) and runs the fixed
// per-kind emitters over retained PhysicalSampleData. Emitted buffers
// copy retained values verbatim — including NaN payloads, ±inf and -0.0 —
// so a DuckDB/Arrow ingestion assembles a RecordBatch bitwise-identical
// to arrow_batch(encode_rows(...)) without a Python round-trip.
// The held-GIL plan materialization, Piece boxing and module
// registration twins live in encode_binding.{hpp,cpp} on the optional
// extension; nothing below names Python or nanobind.
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "../io/value.hpp"
#include "encoding_values.hpp"
#include "samples.hpp"

namespace runtime {

// ---- parsed encode plans ---------------------------------------------------

// One channels leaf/member schema node from the plan's 'columns' list.
// Nested under the struct dtypes (children) and map<struct> entry member
// schemas — the same recursion _leaf_plan emits in native_encode.py.
struct EncodePlanColumn {
    std::string name;
    std::string dtype;
    std::vector<std::string> path;
    bool optional = false;
    std::int64_t shape = -1;              // >= 0 pins the x[] element count
    std::vector<std::string> except_keys; // members deeper leaves own
    std::string convert;                  // "" | "verdict"
    // True when a proper prefix at depth >= 2 is itself a leaf: routing
    // only enters non-leaf members (the root call still routes claimed
    // members at a depth-1 leaf before the terminal gets the remainder),
    // so a leaf nested under a deeper terminal is unreachable and encodes
    // absent on every row.
    bool shadowed = false;
    // Indices of the op=='valid' masks whose paths are this leaf's prefix
    // chain (path[:1..len-1]; the claim mask itself for depth-1 leaves). A
    // required leaf encodes NULL iff one of them reads invalid this row —
    // the encode_rows "ancestor absent-or-null" exemption. Top-level only.
    std::vector<std::size_t> ancestors;
    // struct/struct[] member schemas and map<struct> entry members, in
    // catalog order; each child's path is the single relative member name.
    // Empty on leaf dtypes — _convert_struct consumes members only through
    // this list, so unknown members reject there.
    std::vector<EncodePlanColumn> children;
    // Name-sorted member-name -> children slot index, compiled at parse
    // for the struct dtypes: convert_struct dispatches a payload dict's
    // members through it in one pass. Views the child `name` strings —
    // built only after leaf_columns stops moving. Empty on leaf dtypes.
    std::vector<std::pair<std::string_view, std::size_t>> member_lookup;
};

// One presence/validity mask column the channels emitter fills.
struct EncodePlanMask {
    std::string name;
    std::vector<std::string> path;
    enum class Op : std::uint8_t { present, valid } op = Op::present;
};

// The staged plan parsed once per (table, chunk-run): the declared
// kind/table plus every compiled row-loop handle the emitters use.
//
// Self-references: PathNode edges view the leaf column path members and
// claimed roots, member_lookup views child `name` strings, and
// column_owned points into `deeper` — every level is a container-stolen
// member of the plan itself, so moving a whole EncodePlan keeps them
// valid (map nodes and vector buffers move by steal). Mutating any plan
// member after parse invalidates the compiled indexes, so parsed plans
// are read-only artifacts for the emitters.
struct EncodePlan {
    // Move-only like Piece: path_nodes edges view this plan's own
    // leaf_columns/roots strings and column_owned points into `deeper`,
    // so a copy would leave the views aimed at the source.
    EncodePlan() = default;
    EncodePlan(const EncodePlan &) = delete;
    EncodePlan &operator=(const EncodePlan &) = delete;
    EncodePlan(EncodePlan &&) = default;
    EncodePlan &operator=(EncodePlan &&) = default;

    std::string table;
    std::string kind;
    std::vector<std::string> columns;     // physics/forces fixed-list check
    bool has_columns = false;
    // channels payloads
    std::vector<EncodePlanColumn> leaf_columns;
    std::vector<EncodePlanMask> masks;
    std::vector<std::string> roots;       // this table's claimed roots
    std::set<std::string> all_roots;      // every catalog channel root
    // Routing indexes mirroring encoding.py::_distribute's lookups: full
    // leaf paths and each internal prefix's routed member components
    // (the parse-time shadowed/except_keys checks read these).
    std::set<std::vector<std::string>> leaf_paths;
    std::map<std::vector<std::string>, std::set<std::string>> deeper;
    // Compiled prefix tree over the leaf paths and claimed roots: a node's
    // edges are the routed members _distribute tracks at that prefix and
    // `leaf` marks the leaf_columns slot when the prefix is itself a leaf.
    // Insertion order keeps parents ahead of children, so resolve_paths
    // resolves a whole row in one forward pass.
    struct PathNode {
        std::vector<std::pair<std::string_view, std::size_t>> edges;
        std::size_t leaf = std::numeric_limits<std::size_t>::max();
    };
    static constexpr std::size_t kNoNode =
        std::numeric_limits<std::size_t>::max();
    std::vector<PathNode> path_nodes{PathNode{}};   // [0]: channels object
    std::vector<std::size_t> root_nodes;            // roots[i] -> node
    std::vector<std::size_t> column_nodes;          // leaf_columns[i] -> node
    std::vector<std::string> column_labels;         // join_path(path)
    // deeper set a depth-1 container leaf's remainder strips, nullptr
    // otherwise — the row loop's erstwhile deeper.find(column.path).
    std::vector<const std::set<std::string> *> column_owned;
    std::vector<std::size_t> mask_nodes;            // masks[j] -> node|kNoNode
};

// The derived-transition scan's compiled field list — one tracked
// (kind, path) entry from the plan, `label` the oracle's changes key
// ('.'.join(path[1:]) or path[0]) and kind_slot indexing the unique kinds
// in first-appearance order.
struct DerivedEventPlan {
    struct Field {
        std::string kind;
        std::vector<std::string> path;
        std::string label;
        std::size_t kind_slot = 0;
    };
    std::vector<Field> fields;
    std::vector<std::string> kinds;
};

// Parses the staged plan document into the compiled EncodePlan: exact-key
// gates, dtype/member rules, claimed-root and presence-mask consistency.
// std::invalid_argument naming the plan member on every rejection — the
// same diagnostics the former nanobind readers produced.
[[nodiscard]] EncodePlan parse_encode_plan(const bike::io::Value &plan);
// The {'fields': [{'kind', 'path'}, ...]} document native.py compiles for
// iter_derived_events.
[[nodiscard]] DerivedEventPlan
parse_derived_event_plan(const bike::io::Value &plan);

// ---- emission ---------------------------------------------------------------

// Runs the declared per-kind emitter over the retained slice [start,
// start+limit): physics/forces fixed columns or the channels
// path-projected leaf/mask set. Same slice domain as
// OwnedNativeEpisode::sample_batch — start == row_count admits the empty
// suffix, limit must be positive — and the same failure surfaces as the
// Python oracle (unknown roots/members, absent-vs-null handling, the
// ancestor-absent exemption).
[[nodiscard]] EncodedTable
encode_table_native(const OwnedNativeEpisode &episode,
                    const EncodePlan &plan, std::int64_t start,
                    std::int64_t limit);

// ---- native row records ----------------------------------------------------

// One commands_applied row — the ordinary_commands mirror in
// recording/native.py::NativePhysicalSource.tables: emitted on each
// interval whose (control, brake demands) tuple differs from the
// previous interval's. application_id counts changes from the retained
// episode start; rows [0, start) are replayed to seed the compare, so
// chunked calls emit exactly the full-pass sequence.
struct AppliedCommandRow {
    std::int64_t application_id = 0;
    double time_s = 0.;
    std::int64_t step = 0;
    Wire control;
    // Absent demand keys stay absent (Python keeps them out of the dict
    // entirely) — monostate would wrongly spell an explicit null.
    std::optional<Wire> front_brake_demand;
    std::optional<Wire> rear_brake_demand;
    std::optional<Wire> rollback_brake_demand;
};

// One derived_transition event row — the channel diff inside
// recording/native.py::iter_derived_events: emitted for each
// (interval, kind) whose tracked labels changed against the previous
// retained interval. `changes` keeps the labels' first-insertion order
// and carries only the changed values; kind=="derived_transition" and the
// InputEvent/event_id wrap stay at the caller's seam.
struct DerivedEventRow {
    std::int64_t interval_id = 0;
    std::string derived_kind;
    WireObject changes;
};

[[nodiscard]] std::vector<AppliedCommandRow> applied_command_rows_native(
    const OwnedNativeEpisode &episode, std::int64_t start,
    std::int64_t limit);
[[nodiscard]] std::vector<DerivedEventRow> derived_event_rows_native(
    const OwnedNativeEpisode &episode, const DerivedEventPlan &plan,
    std::int64_t start, std::int64_t limit);

} // namespace runtime
