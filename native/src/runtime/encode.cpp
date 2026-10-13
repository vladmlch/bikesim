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
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string_view>
#include <variant>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"

namespace runtime {
namespace {

// compile_plan emits {'table', 'kind'} plus an optional 'columns' list.
// physics/forces columns are fixed emitters; a provided list that does not
// name exactly the fixed set is rejected rather than silently narrowed.
struct PlanView {
    std::string table;
    std::string kind;
    std::vector<std::string> columns;
    bool has_columns = false;
};

constexpr std::array<std::string_view, 6> kPhysicsColumns{
    "interval_id", "time_s", "interval_end_s", "dt_s", "qpos", "qvel"};
constexpr std::array<std::string_view, 4> kForcesColumns{
    "interval_id", "component", "qfrc", "power_w"};

PlanView parse_plan(nb::handle plan) {
    const wire::Dict dict{.value = wire::mapping(plan, "encode plan"),
                          .path = "encode plan"};
    wire::exact(dict, wire::keys("table", "kind"), wire::keys("columns"));
    PlanView parsed;
    parsed.table = wire::string(wire::field(dict, "table"), "encode plan.table");
    parsed.kind = wire::string(wire::field(dict, "kind"), "encode plan.kind");
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

} // namespace runtime
