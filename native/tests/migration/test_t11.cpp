// test_t11.cpp — pure encoder + provenance boundary regressions (task
// T11): parse_encode_plan / parse_derived_event_plan over staged
// bike::io::Value documents, encode_table_native /
// applied_command_rows_native / derived_event_rows_native over authored
// PhysicalSampleData, and runtime_identity() — all exercised in the
// Python-free graph. This target's compile context carries no
// Python/nanobind include, so compiling runtime/encode.hpp +
// runtime/provenance.hpp IS the include-closure check — binding headers
// (encode_binding.hpp, samples_binding.hpp) are deliberately absent.
//
// The oracle halves live in tests/reference/test_native_encode_columns.py
// (differential through the optional module); here the same acceptance
// set and slice/change-detection domains run against hand-authored
// samples so every assertion names the exact wire tree and plan document.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "runtime/encode.hpp"
#include "runtime/provenance.hpp"
#include "runtime/samples.hpp"
#include "engaged.hpp"
#include "io/json.hpp"
#include "io/value.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace {

namespace bio = bike::io;
using runtime::EncodedTable;
using runtime::EncodePlan;
using runtime::OwnedNativeEpisode;
using runtime::PhysicalSampleData;
using runtime::Piece;
using runtime::SamplePtr;
using runtime::Wire;
using runtime::WireArray;
using runtime::WireObject;

[[nodiscard]] bio::Value plan_doc(std::string_view json) {
    return bio::parse_json(json);
}

[[nodiscard]] EncodePlan encode_plan(std::string_view json) {
    return runtime::parse_encode_plan(plan_doc(json));
}

// (start, end) is the authored interval — callers always pass the pair
// positionally.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
[[nodiscard]] SamplePtr sample(std::int64_t id, double start, double end,
                               WireObject channels = {},
                               runtime::ForceComponents forces = {},
                               WireObject powers = {}) {
    auto data = std::make_shared<PhysicalSampleData>();
    data->interval_id = id;
    data->time_s = start;
    data->end_time_s = end;
    data->qpos = {1.5 + static_cast<double>(id), -0.0};
    data->qvel = {std::numeric_limits<double>::quiet_NaN(), 0.25};
    data->forces = std::move(forces);
    data->powers_w = std::move(powers);
    data->channels = std::move(channels);
    return data;
}

[[nodiscard]] OwnedNativeEpisode episode(std::vector<SamplePtr> &rows) {
    return OwnedNativeEpisode::seal(1, std::span<const SamplePtr>(rows),
                                    runtime::ColumnLayout{});
}

[[nodiscard]] const Piece &column(const EncodedTable &table,
                                  std::string_view name) {
    const auto found = table.columns.find(std::string(name));
    bt::require(found != table.columns.end(),
                "encoded column missing: " + std::string(name));
    return found->second;
}

// ---- plan parsing and slice domains -----------------------------------------

void checks_parse_and_domain() {
    const EncodePlan physics = encode_plan(
        R"({"table":"physics","kind":"physics"})");
    bt::require(physics.table == "physics" && physics.kind == "physics",
                "physics plan did not stage table/kind");
    bt::require(!physics.has_columns,
                "physics plan reported columns it did not declare");

    const EncodePlan declared = encode_plan(
        R"({"table":"physics","kind":"physics","columns":)"
        R"(["interval_id","time_s","interval_end_s","dt_s","qpos","qvel"]})");
    bt::require(declared.has_columns && declared.columns.size() == 6,
                "declared physics columns did not stage");

    const runtime::DerivedEventPlan derived =
        runtime::parse_derived_event_plan(plan_doc(
            R"({"fields":[{"kind":"demands","path":["drive","front"]},)"
            R"({"kind":"demands","path":["drive","rear"]},)"
            R"({"kind":"status","path":["mode"]}]})"));
    bt::require(derived.fields.size() == 3 && derived.kinds.size() == 2,
                "derived plan did not stage fields/kinds");
    bt::require(derived.fields[0].label == "front" &&
                    derived.fields[2].label == "mode",
                "derived plan labels were not '.'.join(path[1:])");
    bt::require(derived.kinds[0] == "demands" &&
                    derived.kinds[1] == "status",
                "derived plan kinds lost first-appearance order");

    // Plan-level rejections — the same invalid_argument domain the
    // binding surfaces as ValueError.
    bt::invalid([] {
        static_cast<void>(encode_plan(R"({"kind":"physics"})")); });
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"physics","kind":"physics","extra":1})")); });
    bt::invalid([] {
        static_cast<void>(encode_plan(R"({"table":"physics"})")); });
    bt::invalid([] {
        static_cast<void>(runtime::parse_derived_event_plan(plan_doc(
            R"({"fields":[{"kind":"k","path":[]}]})"))); });
    bt::invalid([] {
        static_cast<void>(runtime::parse_derived_event_plan(plan_doc(
            R"({"fields":[{"kind":"k","path":["a"],"extra":1}]})"))); });
    bt::invalid([] {
        static_cast<void>(runtime::parse_derived_event_plan(plan_doc(
            R"({"fields":[] , "extra":1})"))); });

    // Slice domains on a one-row episode: start == row_count admits the
    // empty suffix; limit must be positive; start stays in range.
    std::vector<SamplePtr> rows{sample(0, 0.0, 0.001)};
    const OwnedNativeEpisode ep = episode(rows);
    const EncodePlan plan =
        encode_plan(R"({"table":"physics","kind":"physics"})");
    bt::invalid([&] { static_cast<void>(
        runtime::encode_table_native(ep, plan, -1, 1)); });
    bt::invalid([&] { static_cast<void>(
        runtime::encode_table_native(ep, plan, 0, 0)); });
    bt::invalid([&] { static_cast<void>(
        runtime::encode_table_native(ep, plan, 2, 1)); });
    const EncodedTable empty =
        runtime::encode_table_native(ep, plan, 1, 1);
    bt::require(empty.rows == 0,
                "empty suffix did not report zero rows");
    bt::invalid([&] { static_cast<void>(
        runtime::applied_command_rows_native(ep, -1, 1)); });
    bt::invalid([&] { static_cast<void>(
        runtime::applied_command_rows_native(ep, 0, 0)); });
    bt::invalid([&] { static_cast<void>(
        runtime::applied_command_rows_native(ep, 2, 1)); });
    bt::invalid([&] { static_cast<void>(
        runtime::derived_event_rows_native(ep, derived, -1, 1)); });
    bt::invalid([&] { static_cast<void>(
        runtime::derived_event_rows_native(ep, derived, 0, 0)); });
    bt::invalid([&] { static_cast<void>(
        runtime::derived_event_rows_native(ep, derived, 2, 1)); });
    bt::require(runtime::derived_event_rows_native(ep, derived, 1, 1)
                    .empty(),
                "derived empty suffix was not empty");
}

// ---- plan-shape rejection matrix ---------------------------------------------

void checks_plan_rejects() {
    // kind/table dispatch happens at emission, not parse: a staged plan
    // parses, then encode_table_native rejects the unsupported pair.
    std::vector<SamplePtr> rows{sample(0, 0.0, 0.001)};
    const OwnedNativeEpisode ep = episode(rows);
    const EncodePlan crossed = encode_plan(
        R"({"table":"forces","kind":"physics"})");
    bt::invalid([&] { static_cast<void>(
        runtime::encode_table_native(ep, crossed, 0, 1)); });
    const EncodePlan subset = encode_plan(
        R"({"table":"physics","kind":"physics","columns":["qpos"]})");
    bt::invalid([&] { static_cast<void>(
        runtime::encode_table_native(ep, subset, 0, 1)); });

    // Channels plan rejections.
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["drive"],)"
            R"("all_roots":["drive"],"columns":[)"
            R"({"name":"x","dtype":"double","path":["other","x"],)"
            R"("optional":false}],"masks":[]})")); });
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["drive"],)"
            R"("all_roots":["drive"],"columns":[)"
            R"({"name":"x","dtype":"decimal","path":["drive","x"],)"
            R"("optional":false}],"masks":[]})")); });
    // A depth-2 leaf needs the op=='valid' mask at its depth-1 prefix.
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["drive"],)"
            R"("all_roots":["drive"],"columns":[)"
            R"({"name":"x","dtype":"double","path":["drive","x"],)"
            R"("optional":false}],)"
            R"("masks":[{"name":"__drive_present","path":["drive"],)"
            R"("op":"present"}]})")); });
    // A container terminal's except_keys must name the deeper sibling
    // members the routing index claims.
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["bag"],)"
            R"("all_roots":["bag"],"columns":[)"
            R"({"name":"bag","dtype":"map<double>","path":["bag"],)"
            R"("optional":false,"except_keys":["free"]},)"
            R"({"name":"owned","dtype":"double","path":["bag","owned"],)"
            R"("optional":false}],)"
            R"("masks":[{"name":"__bag_valid","path":["bag"],)"
            R"("op":"valid"}]})")); });
    // struct dtypes need children; leaf dtypes must not carry them, and
    // each child's path is its member name.
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["m"],)"
            R"("all_roots":["m"],"columns":[)"
            R"({"name":"m","dtype":"struct","path":["m"],)"
            R"("optional":false}],)"
            R"("masks":[{"name":"__m_valid","path":["m"],)"
            R"("op":"valid"}]})")); });
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["m"],)"
            R"("all_roots":["m"],"columns":[)"
            R"({"name":"m","dtype":"struct","path":["m"],)"
            R"("optional":false,"children":[)"
            R"({"name":"a","dtype":"double","path":["m","a"],)"
            R"("optional":false}]}],)"
            R"("masks":[{"name":"__m_valid","path":["m"],)"
            R"("op":"valid"}]})")); });
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["m"],)"
            R"("all_roots":["m"],"columns":[)"
            R"({"name":"m","dtype":"struct","path":["m"],)"
            R"("optional":false,"children":[)"
            R"({"name":"a","dtype":"double","path":["a"],)"
            R"("optional":false}]}],)"
            R"("masks":[{"name":"__m_valid","path":["m"],)"
            R"("op":"bogus"}]})")); });
    bt::invalid([] {
        static_cast<void>(encode_plan(
            R"({"table":"t","kind":"channels","roots":["drive"],)"
            R"("all_roots":["drive"],"columns":[)"
            R"({"name":"x","dtype":"varchar","path":["drive","x"],)"
            R"("optional":false,"convert":"upper"}],)"
            R"("masks":[{"name":"__drive_valid","path":["drive"],)"
            R"("op":"valid"}]})")); });
}

// ---- physics/forces fixed emitters -------------------------------------------

void checks_physics_forces() {
    std::vector<SamplePtr> rows{
        sample(0, 0.0, 0.001,
               {},
               {{"gravity", {1.0, 2.0}}, {"motor", {0.5, -0.5}}},
               {{"gravity", Wire{0.25}},
                {"motor", Wire{std::numeric_limits<double>::quiet_NaN()}}}),
        sample(1, 0.001, 0.003,
               {},
               {{"gravity", {3.0, 4.0}}, {"motor", {1.5, -1.5}}},
               {{"gravity", Wire{2.5}}, {"motor", Wire{4.5}}})};
    const OwnedNativeEpisode ep = episode(rows);
    const EncodePlan plan =
        encode_plan(R"({"table":"physics","kind":"physics"})");
    const EncodedTable table =
        runtime::encode_table_native(ep, plan, 0, 2);
    bt::require(table.rows == 2, "physics did not emit two rows");
    bt::require(table.columns.size() == 6,
                "physics did not emit the six fixed columns");
    const Piece &ids = column(table, "interval_id");
    bt::require(ids.kind == Piece::Kind::scalar_i &&
                    ids.i == std::vector<std::int64_t>({0, 1}),
                "physics interval_id values wrong");
    const Piece &times = column(table, "time_s");
    bt::require(times.kind == Piece::Kind::scalar_d &&
                    times.d.size() == 2 && times.d[1] == 0.001,
                "physics time_s values wrong");
    // dt_s is the same IEEE subtraction the oracle performs.
    const Piece &dts = column(table, "dt_s");
    bt::require(dts.d.size() == 2 &&
                    std::bit_cast<std::uint64_t>(dts.d[1]) ==
                        std::bit_cast<std::uint64_t>(0.003 - 0.001),
                "dt_s was not the verbatim IEEE subtraction");
    const Piece &qpos = column(table, "qpos");
    bt::require(qpos.kind == Piece::Kind::list_d &&
                    qpos.offsets.size() == 3 && qpos.offsets[1] == 2 &&
                    qpos.offsets[2] == 4 && qpos.d.size() == 4,
                "qpos list payload/offsets wrong");
    // -0.0 stays bitwise distinct from +0.0; the NaN payload round-trips.
    bt::require(std::bit_cast<std::uint64_t>(qpos.d[1]) ==
                    std::bit_cast<std::uint64_t>(-0.0),
                "qpos -0.0 lost its sign bit");
    const Piece &qvel = column(table, "qvel");
    bt::require(std::isnan(qvel.d[0]) &&
                    std::bit_cast<std::uint64_t>(qvel.d[0]) ==
                        std::bit_cast<std::uint64_t>(
                            std::numeric_limits<double>::quiet_NaN()),
                "qvel NaN payload did not round-trip bitwise");

    // forces: interval × component explosion in authored order.
    const EncodePlan forces_plan =
        encode_plan(R"({"table":"forces","kind":"forces"})");
    const EncodedTable forces =
        runtime::encode_table_native(ep, forces_plan, 0, 2);
    bt::require(forces.rows == 4, "forces did not explode to four rows");
    const Piece &fids = column(forces, "interval_id");
    bt::require(fids.i == std::vector<std::int64_t>({0, 0, 1, 1}),
                "forces interval_id explosion wrong");
    const Piece &comp = column(forces, "component");
    bt::require(comp.kind == Piece::Kind::scalar_s && comp.s.size() == 4 &&
                    comp.s[0] == "gravity" && comp.s[1] == "motor" &&
                    comp.s[2] == "gravity" && comp.s[3] == "motor",
                "forces component order wrong");
    const Piece &qfrc = column(forces, "qfrc");
    bt::require(qfrc.kind == Piece::Kind::list_d &&
                    qfrc.offsets.size() == 5 && qfrc.d.size() == 8 &&
                    qfrc.d[0] == 1.0 && qfrc.d[3] == -0.5 &&
                    qfrc.d[4] == 3.0,
                "forces qfrc payload wrong");
    const Piece &power = column(forces, "power_w");
    bt::require(power.d.size() == 4 && power.d[0] == 0.25 &&
                    std::isnan(power.d[1]) && power.d[2] == 2.5,
                "forces powers_w lookup wrong or NaN not verbatim");

    // A component without its powers_w entry rejects — the oracle's
    // MissingFieldError parity.
    std::vector<SamplePtr> ghost{
        sample(0, 0.0, 0.001, {}, {{"ghost", {0.0}}}, {})};
    const OwnedNativeEpisode ep_ghost = episode(ghost);
    bt::invalid([&] { static_cast<void>(runtime::encode_table_native(
        ep_ghost, forces_plan, 0, 1)); });
}

// ---- channels path projector -------------------------------------------------

// The staged mini-plan: double + bigint + bool + varchar + double[] +
// map<double> + optional members + a verdict convert + a struct — one
// plan covering every lane the leaf matrix declares. `meta` is a
// depth-1 struct claim so structure_piece fills member_slots, the
// move-safety lane.
constexpr std::string_view kChannelsPlan = R"({
    "table": "channel_drive", "kind": "channels",
    "roots": ["drive", "sensors", "energy", "model_status", "meta"],
    "all_roots": ["drive", "sensors", "energy", "model_status", "meta"],
    "columns": [
        {"name": "watts", "dtype": "double",
         "path": ["drive", "watts"], "optional": true},
        {"name": "gear", "dtype": "bigint",
         "path": ["drive", "gear"], "optional": false},
        {"name": "coasting", "dtype": "boolean",
         "path": ["drive", "coasting"], "optional": false},
        {"name": "accel", "dtype": "double[]", "shape": [3],
         "path": ["sensors", "accel"], "optional": false},
        {"name": "by_part", "dtype": "map<double>",
         "path": ["energy", "by_part"], "optional": true},
        {"name": "verdict", "dtype": "varchar", "convert": "verdict",
         "path": ["model_status", "numerically_valid"],
         "optional": false},
        {"name": "meta", "dtype": "struct",
         "path": ["meta"], "optional": true,
         "children": [
            {"name": "note", "dtype": "varchar", "path": ["note"],
             "optional": false},
            {"name": "tries", "dtype": "bigint[]", "path": ["tries"],
             "optional": true}]}
    ],
    "masks": [
        {"name": "__drive_present", "path": ["drive"], "op": "present"},
        {"name": "__drive_valid", "path": ["drive"], "op": "valid"},
        {"name": "__sensors_present", "path": ["sensors"],
         "op": "present"},
        {"name": "__sensors_valid", "path": ["sensors"], "op": "valid"},
        {"name": "__energy_present", "path": ["energy"],
         "op": "present"},
        {"name": "__energy_valid", "path": ["energy"], "op": "valid"},
        {"name": "__model_status_present", "path": ["model_status"],
         "op": "present"},
        {"name": "__model_status_valid", "path": ["model_status"],
         "op": "valid"},
        {"name": "__meta_present", "path": ["meta"], "op": "present"},
        {"name": "__meta_valid", "path": ["meta"], "op": "valid"}]
})";

[[nodiscard]] WireObject drive_members(Wire watts, Wire gear,
                                       Wire coasting) {
    WireObject out;
    out.emplace_back("watts", std::move(watts));
    out.emplace_back("gear", std::move(gear));
    out.emplace_back("coasting", std::move(coasting));
    return out;
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void checks_channels() {
    // EncodePlan moves by steal: path-trie edge views into leaf column
    // path strings stay valid — encode through the moved plan to prove it.
    EncodePlan moved = encode_plan(kChannelsPlan);
    const EncodePlan &live = moved;

    // Row 0: every leaf authored; row 1: watts absent, by_part empty map,
    // meta explicit null; row 2: all roots absent.
    std::vector<SamplePtr> rows{
        sample(0, 0.0, 0.001,
               WireObject{
                   {"drive", drive_members(Wire{150.5}, Wire{4},
                                           Wire{true})},
                   {"sensors", WireObject{
                       {"accel", WireArray{Wire{1.0}, Wire{-0.0},
                                           Wire{3.5}}}}},
                   {"energy", WireObject{
                       {"by_part", WireObject{{"chain", Wire{0.1}},
                                              {"tires", Wire{-0.25}}}}}},
                   {"model_status", WireObject{
                       {"numerically_valid", Wire{true}}}},
                   {"meta", WireObject{
                       {"note", Wire{"spin-up"}},
                       {"tries", WireArray{Wire{1}, Wire{2}}}}}}),
        sample(1, 0.001, 0.002,
               WireObject{
                   {"drive", drive_members(Wire{nullptr}, Wire{2},
                                           Wire{false})},
                   {"sensors", WireObject{
                       {"accel", WireArray{Wire{0.0}, Wire{0.0},
                                           Wire{0.0}}}}},
                   {"energy", WireObject{
                       {"by_part", WireObject{}}}},
                   {"model_status", WireObject{
                       {"numerically_valid", Wire{false}}}},
                   {"meta", Wire{nullptr}}}),
        sample(2, 0.002, 0.003, WireObject{})};
    const OwnedNativeEpisode ep = episode(rows);
    EncodedTable table =
        runtime::encode_table_native(ep, live, 0, 3);
    bt::require(table.rows == 3, "channels did not emit three rows");

    const Piece &watts = column(table, "watts");
    bt::require(watts.kind == Piece::Kind::scalar_d &&
                    watts.d.size() == 3 && watts.d[0] == 150.5,
                "watts scalar payload wrong");
    bt::require(watts.valid.size() == 3 && watts.valid[0] == 1 &&
                    watts.valid[1] == 0 && watts.valid[2] == 0,
                "watts validity did not split present-null/absent");
    const Piece &watts_present = column(table, "watts_present");
    bt::require(watts_present.kind == Piece::Kind::scalar_b &&
                    watts_present.b.size() == 3 &&
                    watts_present.b[0] == 1 && watts_present.b[1] == 1 &&
                    watts_present.b[2] == 0,
                "watts_present did not distinguish null from absent");

    const Piece &gear = column(table, "gear");
    bt::require(gear.kind == Piece::Kind::scalar_i &&
                    gear.i == std::vector<std::int64_t>({4, 2, 0}) &&
                    gear.valid.size() == 3 && gear.valid[2] == 0,
                "gear required leaf wrong — row 2 needs the "
                "ancestor-absent exemption NULL");

    const Piece &verdict = column(table, "verdict");
    bt::require(verdict.kind == Piece::Kind::scalar_s &&
                    verdict.s.size() == 3 && verdict.s[0] == "true" &&
                    verdict.s[1] == "false",
                "verdict conversion did not emit 'true'/'false' text");

    const Piece &accel = column(table, "accel");
    // Rows 0/1 carry 3 elements each; row 2's absent leaf is an empty
    // NULL span (repeating offset 6), not three payload zeros.
    bt::require(accel.kind == Piece::Kind::list_d &&
                    accel.offsets ==
                        std::vector<std::int32_t>({0, 3, 6, 6}) &&
                    accel.d.size() == 6 &&
                    accel.d[0] == 1.0 &&
                    std::bit_cast<std::uint64_t>(accel.d[1]) ==
                        std::bit_cast<std::uint64_t>(-0.0),
                "double[] list payload/offsets wrong");

    const Piece &by_part = column(table, "by_part");
    bt::require(by_part.kind == Piece::Kind::map &&
                    by_part.offsets.size() == 4 &&
                    by_part.offsets[1] == 2 && by_part.s.size() == 2 &&
                    by_part.s[0] == "chain" && by_part.s[1] == "tires" &&
                    by_part.valid.size() == 3 && by_part.valid[0] == 1 &&
                    by_part.valid[1] == 1 && by_part.valid[2] == 0,
                "map<double> keys/offsets/validity wrong");
    bt::require(by_part.items != nullptr &&
                    by_part.items->kind == Piece::Kind::scalar_d &&
                    by_part.items->d.size() == 2 &&
                    by_part.items->d[0] == 0.1 &&
                    by_part.items->d[1] == -0.25,
                "map<double> items payload wrong");
    const Piece &by_part_present = column(table, "by_part_present");
    bt::require(by_part_present.b == std::vector<char>({1, 1, 0}),
                "empty map must read present, absent map absent");

    const Piece &meta = column(table, "meta");
    bt::require(meta.kind == Piece::Kind::structure &&
                    meta.valid.size() == 3 && meta.valid[0] == 1 &&
                    meta.valid[1] == 0 && meta.valid[2] == 0,
                "struct piece kind/validity wrong");
    const Piece &meta_present = column(table, "meta_present");
    bt::require(meta_present.b == std::vector<char>({1, 1, 0}),
                "explicit-null struct must read present");
    const Piece &note = meta.children.at("note");
    bt::require(note.s.size() == 3 && note.s[0] == "spin-up",
                "struct member payload wrong");
    const Piece &tries = meta.children.at("tries");
    bt::require(tries.kind == Piece::Kind::list_i &&
                    tries.i == std::vector<std::int64_t>({1, 2}) &&
                    tries.offsets ==
                        std::vector<std::int32_t>({0, 2, 2, 2}),
                "struct member list payload wrong");
    const Piece &tries_present = meta.children.at("tries_present");
    bt::require(tries_present.b == std::vector<char>({1, 0, 0}),
                "struct member _present companion wrong");

    // member_slots point into the children map's nodes — moving the
    // whole table must keep them dereferenceable and pointing at the
    // same (now moved) member pieces, not stale source slots.
    EncodedTable moved_table = std::move(table);
    const Piece &moved_meta = column(moved_table, "meta");
    bt::require(moved_meta.member_slots.size() == 2,
                "member_slots lost slots across the move");
    for (std::size_t i = 0; i < moved_meta.member_slots.size(); ++i) {
        const Piece::MemberSlot &slot = moved_meta.member_slots[i];
        bt::require(slot.member != nullptr &&
                        (slot.member == &moved_meta.children.at("note") ||
                         slot.member ==
                             &moved_meta.children.at("tries")),
                    "member_slots member no longer addresses a child "
                    "after the move");
    }
    bt::require(moved_meta.member_slots[1].present ==
                    &moved_meta.children.at("tries_present"),
                "optional member's _present slot detached after move");

    // Root masks: row 2 dropped every root — present and valid both 0;
    // the explicit-null meta reads present=1, valid=0 on row 1.
    const Piece &meta_p = column(moved_table, "__meta_present");
    bt::require(meta_p.b == std::vector<char>({1, 1, 0}),
                "__meta_present mask wrong");
    const Piece &meta_v = column(moved_table, "__meta_valid");
    bt::require(meta_v.b == std::vector<char>({1, 0, 0}),
                "__meta_valid must read 0 for the explicit-null struct");
    const Piece &drive_v = column(moved_table, "__drive_valid");
    bt::require(drive_v.b == std::vector<char>({1, 1, 0}),
                "__drive_valid mask wrong — absent root reads 0");

    // Domain rejects at row level.
    std::vector<SamplePtr> unknown_root{
        sample(0, 0.0, 0.001, WireObject{{"forged", WireObject{}}})};
    const OwnedNativeEpisode ep_forged = episode(unknown_root);
    bt::invalid([&] { static_cast<void>(runtime::encode_table_native(
        ep_forged, live, 0, 1)); });

    {
        // drive gains a member no leaf claims — routing rejects it
        // before any leaf converts.
        WireObject drive = drive_members(Wire{1.0}, Wire{1}, Wire{true});
        drive.emplace_back("bogus_member", Wire{1});
        std::vector<SamplePtr> forged_rows{sample(
            0, 0.0, 0.001,
            WireObject{{"drive", std::move(drive)},
                       {"sensors", WireObject{
                           {"accel", WireArray{Wire{0.0}, Wire{0.0},
                                               Wire{0.0}}}}},
                       {"model_status", WireObject{
                           {"numerically_valid", Wire{true}}}}})};
        const OwnedNativeEpisode ep_bad = episode(forged_rows);
        bt::invalid([&] { static_cast<void>(runtime::encode_table_native(
            ep_bad, live, 0, 1)); });
    }

    // A required leaf absent under a present parent rejects; a missing
    // map payload rejects the member-dtype gate; a short double[]
    // rejects the shape gate.
    std::vector<SamplePtr> missing_required{sample(
        0, 0.0, 0.001,
        WireObject{{"drive", WireObject{{"watts", Wire{1.0}}}},
                   {"sensors", WireObject{
                       {"accel", WireArray{Wire{0.0}, Wire{0.0},
                                           Wire{0.0}}}}},
                   {"model_status", WireObject{
                       {"numerically_valid", Wire{true}}}}})};
    const OwnedNativeEpisode ep_missing = episode(missing_required);
    bt::invalid([&] { static_cast<void>(runtime::encode_table_native(
        ep_missing, live, 0, 1)); });

    std::vector<SamplePtr> bad_shape{sample(
        0, 0.0, 0.001,
        WireObject{{"drive", drive_members(Wire{1.0}, Wire{1},
                                          Wire{true})},
                   {"sensors", WireObject{
                       {"accel", WireArray{Wire{1.0}, Wire{2.0}}}}},
                   {"model_status", WireObject{
                       {"numerically_valid", Wire{true}}}}})};
    const OwnedNativeEpisode ep_shape = episode(bad_shape);
    bt::invalid([&] { static_cast<void>(runtime::encode_table_native(
        ep_shape, live, 0, 1)); });
}
NATIVE_DIAG_POP

// ---- applied-command rows ----------------------------------------------------

[[nodiscard]] WireObject control_channels(Wire control,
                                          WireObject drive = {}) {
    WireObject out;
    out.emplace_back("control", std::move(control));
    out.emplace_back("drive", Wire{std::move(drive)});
    return out;
}

void checks_applied_rows() {
    const WireObject drive_a{{"front_brake_demand", Wire{0.1}}};
    const WireObject drive_b{{"front_brake_demand", Wire{0.1}},
                             {"rear_brake_demand", Wire{nullptr}}};
    // row0/row1 identical tuples; row2 changes control; row3 only the
    // rear demand appears — each change emits exactly once.
    std::vector<SamplePtr> rows{
        sample(0, 0.0, 0.001,
               control_channels(WireObject{{"motor_nm", Wire{5}}},
                                drive_a)),
        sample(1, 0.001, 0.002,
               control_channels(WireObject{{"motor_nm", Wire{5}}},
                                drive_a)),
        sample(2, 0.002, 0.003,
               control_channels(WireObject{{"motor_nm", Wire{7}}},
                                drive_a)),
        sample(3, 0.003, 0.004,
               control_channels(WireObject{{"motor_nm", Wire{7}}},
                                drive_b))};
    const OwnedNativeEpisode ep = episode(rows);
    const std::vector<runtime::AppliedCommandRow> full =
        runtime::applied_command_rows_native(ep, 0, 4);
    bt::require(full.size() == 3,
                "applied rows did not emit one row per change");
    bt::require(full[0].application_id == 0 && full[0].step == 0 &&
                    full[1].application_id == 1 && full[1].step == 2 &&
                    full[2].application_id == 2 && full[2].step == 3,
                "application_id/step sequencing wrong");
    bt::require(full[0].front_brake_demand.has_value() &&
                    !full[0].rear_brake_demand.has_value() &&
                    !full[0].rollback_brake_demand.has_value(),
                "absent demand keys must stay absent");
    bt::require(full[2].rear_brake_demand.has_value() &&
                    std::holds_alternative<std::monostate>(
                        engaged(full[2].rear_brake_demand).value),
                "explicit-null demand did not stay a null wire");
    const auto &control0 =
        std::get<WireObject>(full[0].control.value);
    bt::require(std::get<std::int64_t>(control0[0].second.value) == 5,
                "control wire did not carry the authored object");

    // Windowed parity: rows [0, start) replay to seed, so chunked calls
    // emit exactly the full-pass sequence.
    const std::vector<runtime::AppliedCommandRow> window =
        runtime::applied_command_rows_native(ep, 2, 2);
    bt::require(window.size() == 2 &&
                    window[0].application_id == full[1].application_id &&
                    window[0].step == full[1].step &&
                    window[1].application_id == full[2].application_id &&
                    window[1].step == full[2].step,
                "windowed applied rows diverged from the full pass");

    // A missing control channel rejects — the mirror requires it.
    std::vector<SamplePtr> no_control{
        sample(0, 0.0, 0.001, WireObject{{"drive", WireObject{}}})};
    const OwnedNativeEpisode ep_none = episode(no_control);
    bt::invalid([&] { static_cast<void>(
        runtime::applied_command_rows_native(ep_none, 0, 1)); });
}

// ---- derived-transition rows --------------------------------------------------

void checks_derived_rows() {
    const runtime::DerivedEventPlan plan =
        runtime::parse_derived_event_plan(plan_doc(
            R"({"fields":[{"kind":"demands","path":["drive","front"]},)"
            R"({"kind":"demands","path":["drive","rear"]},)"
            R"({"kind":"status","path":["mode"]}]})"));

    std::vector<SamplePtr> rows{
        sample(0, 0.0, 0.001,
               WireObject{{"drive", WireObject{{"front", Wire{1.0}},
                                             {"rear", Wire{0.0}}}},
                          {"mode", Wire{"cruise"}}}),
        sample(1, 0.001, 0.002,
               WireObject{{"drive", WireObject{{"front", Wire{2.0}},
                                             {"rear", Wire{0.0}}}},
                          {"mode", Wire{"cruise"}}}),
        sample(2, 0.002, 0.003,
               WireObject{{"drive", WireObject{{"front", Wire{1.0}},
                                             {"rear", Wire{0.0}}}},
                          {"mode", Wire{"brake"}}})};
    const OwnedNativeEpisode ep = episode(rows);
    const std::vector<runtime::DerivedEventRow> full =
        runtime::derived_event_rows_native(ep, plan, 0, 3);
    // Row 0 only seeds; row 1 emits demands.front; row 2 emits
    // demands.front and status.mode — each changed value only.
    bt::require(full.size() == 3,
                "derived rows did not emit the changed slots only");
    bt::require(full[0].interval_id == 1 &&
                    full[0].derived_kind == "demands" &&
                    full[0].changes.size() == 1 &&
                    full[0].changes[0].first == "front" &&
                    std::get<double>(full[0].changes[0].second.value) ==
                        2.0,
                "derived demands row wrong");
    bt::require(full[1].interval_id == 2 &&
                    full[1].derived_kind == "demands" &&
                    full[1].changes.size() == 1 &&
                    std::get<double>(full[1].changes[0].second.value) ==
                        1.0,
                "derived demands revert row wrong");
    bt::require(full[2].interval_id == 2 &&
                    full[2].derived_kind == "status" &&
                    full[2].changes.size() == 1 &&
                    full[2].changes[0].first == "mode" &&
                    std::get<std::string>(
                        full[2].changes[0].second.value) == "brake",
                "derived status row wrong");

    // Windowed: start=1 seeds `previous` from row 0, emitting exactly
    // the full-pass suffix.
    const std::vector<runtime::DerivedEventRow> window =
        runtime::derived_event_rows_native(ep, plan, 1, 2);
    bt::require(window.size() == 3 &&
                    window[0].interval_id == full[0].interval_id &&
                    window[2].derived_kind == full[2].derived_kind,
                "windowed derived rows diverged from the full pass");
}

// ---- provenance ---------------------------------------------------------------

void checks_provenance() {
    const runtime::RuntimeIdentity identity = runtime::runtime_identity();
    bt::require(!identity.mujoco_version.empty() &&
                    !identity.mujoco_library_path.empty() &&
                    !identity.native_source_sha256.empty() &&
                    !identity.build_context_json.empty(),
                "runtime_identity produced an empty field");
    bt::require(identity.native_source_sha256.size() == 64,
                "native_source_sha256 is not a SHA-256 hex digest");
    bool hex = true;
    for (const char c : identity.native_source_sha256)
        hex = hex && ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
    bt::require(hex, "native_source_sha256 is not lowercase hex");
    bt::require(identity.mujoco_library_path.find("mujoco") !=
                    std::string::npos,
                "mujoco_library_path does not name the engine");
    // The compiled-in context stays a parseable JSON object carrying the
    // same keys the extension's dict reported.
    const bio::Value context =
        bio::parse_json(identity.build_context_json);
    const auto *object = std::get_if<bio::Value::Object>(&context.data);
    bt::require(object != nullptr,
                "build_context_json did not parse to an object");
    for (const std::string_view key :
         {"build_type", "compiler_id", "compiler_version",
          "numerical_flags"})
        bt::require(bio::find_member(*object, key) != nullptr,
                    "build_context_json is missing key: " +
                        std::string(key));
}

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(checks_parse_and_domain);
        failures += bt::run(checks_plan_rejects);
        failures += bt::run(checks_physics_forces);
        failures += bt::run(checks_channels);
        failures += bt::run(checks_applied_rows);
        failures += bt::run(checks_derived_rows);
        failures += bt::run(checks_provenance);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
