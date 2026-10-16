// encode_binding.cpp — the nanobind adapter over the pure
// runtime/encode.cpp projector and runtime/provenance.cpp identity
// (task T11). Everything Python-shaped for the episode-encoding seam
// lives here and only here: the plan handle materializes into the owned
// bike::io::Value the pure parser validates, and the emitted Piece tree
// and row records box into Python containers byte-identical to the dicts
// the former monolithic encode.cpp produced.
//
// Materialization shape notes (the Value tree parse_encode_plan and
// parse_derived_event_plan consume):
//   dict            -> Object (str-key gate mirrors the wire::exact_keys
//                      first pass — the leaf readers never see a non-str
//                      key because Object cannot spell one)
//   Sequence/ndarray -> Array (wire::sequence's own gate — Mapping, str
//                      and bytes are handled by their lanes above it, and
//                      nb::list materializes ndarray elements like the
//                      readers' iteration did)
//   str             -> String; Integral -> integer lexeme via
//                      PyNumber_Index + str() (arbitrary precision
//                      survives; vread::integer range-checks it at the
//                      same member the reader did)
//   Real            -> shortest round-trip float lexeme (never integer-
//                      spelled); a nonfinite one becomes the recording
//                      Tagged{"float64"} sentinel
//   bool            -> Bool (literal Py_True/Py_False only — np.bool_
//                      joins every other unrecognized object in the
//                      opaque-Bytes lane below, so the leaf readers keep
//                      producing their own "expected X" diagnostics at
//                      the same member paths)
//   bytes              -> the Sequence lane: wire::sequence's own gate
//                      rejects it ("expected ordered sequence") — the
//                      same arm that rejected it before the split
//   bytearray          -> the Sequence lane: materializes to an int
//                      Array, exactly like wire::sequence's list arm
//   other objects      -> opaque Bytes{}: it names no plan lane, so
//                      every leaf reader rejects it with the same
//                      "path: expected ..." the wire:: readers produced
//                      — acceptance set identical, diagnostics carried
//                      by the pure readers instead of the handle
//                      materializer. Member paths inside a document keep
//                      the document-root path (the value_from_python
//                      convention) — only conversion-side errors (string
//                      keys, sequence materialization) sit at the root;
//                      every leaf type rejection keeps its member path.
#include "encode_binding.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nanobind/stl/string.h>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "samples_binding.hpp"

namespace nb = nanobind;

namespace runtime {
namespace {

namespace bio = bike::io;

// The ABC/type objects the materializer gates on — same imports the
// wire:: readers ran per call.
struct PlanTypes {
    nb::object integral;
    nb::object real;
    nb::object sequence;
    nb::object ndarray;
};

PlanTypes plan_types() {
    const nb::module_ numbers = nb::module_::import_("numbers");
    return {.integral = numbers.attr("Integral"),
            .real = numbers.attr("Real"),
            .sequence = nb::module_::import_("collections.abc")
                           .attr("Sequence"),
            .ndarray = nb::module_::import_("numpy").attr("ndarray")};
}

// nb::handle -> bike::io::Value for one plan document — the encode-plan
// twin of runtime_binding.cpp's value_from_python, restricted to the
// acceptance set the wire:: readers reached (no dataclass lane: plans
// are plain data, and the opaque lane reproduces the readers' reject).
// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the document shape
[[nodiscard]] bio::Value plan_value(nb::handle value,
                                    const PlanTypes &types,
                                    std::string_view path) {
    if (value.is_none()) return bio::Value{};
    // Strict bools only — wire::boolean requires the literal
    // Py_True/Py_False; the Integral/Real lanes below must not reclaim
    // them (numbers registers bool under Integral).
    if (value.ptr() == Py_True) return bio::Value{true};
    if (value.ptr() == Py_False) return bio::Value{false};
    if (nb::isinstance<nb::str>(value))
        return bio::Value{wire::string(value, path)};
    if (nb::isinstance<nb::dict>(value)) {
        bio::Value::Object out;
        for (const auto item : nb::borrow<nb::dict>(value)) {
            // wire::exact_keys' first pass ran wire::string over every
            // key: a non-str key rejects as "expected string" before any
            // unknown-key check — the owned Object cannot spell one.
            if (!nb::isinstance<nb::str>(item.first))
                wire::invalid(path, "expected string");
            out.emplace_back(wire::string(item.first, path),
                             plan_value(item.second, types, path));
        }
        return bio::Value{std::move(out)};
    }
    // numeric_boolean covers np.bool_ too: it lands in the opaque lane
    // like the readers' strict-bool gate produced.
    if (!wire::numeric_boolean(value) &&
        nb::isinstance(value, types.integral)) {
        // PyNumber_Index collapses IntEnum and np.integer to the int
        // domain; str() spells the full decimal — wide Python ints
        // arrive whole and fail vread::integer's range check at the same
        // position the PyLong_AsLongLong overflow used to.
        nb::object const indexed =
            nb::steal<nb::object>(PyNumber_Index(value.ptr()));
        if (!indexed.is_valid()) wire::conversion_error(path);
        return bio::Value{
            bio::Number{nb::cast<std::string>(nb::str(indexed))}};
    }
    if (!wire::numeric_boolean(value) &&
        nb::isinstance(value, types.real)) {
        const double result = PyFloat_AsDouble(value.ptr());
        if (PyErr_Occurred()) wire::conversion_error(path);
        if (!std::isfinite(result)) {
            const char *spelling = std::isnan(result) ? "nan"
                                   : result > 0.    ? "+inf"
                                                    : "-inf";
            return bio::Value{bio::Tagged{
                .kind = "float64", .fields = {{"value", spelling}}}};
        }
        std::array<char, 32> buffer{};
        const auto written = std::to_chars(
            buffer.data(), std::to_address(buffer.end()), result);
        if (written.ec != std::errc())
            throw std::runtime_error("float lexeme encode failed");
        std::string lexeme(buffer.data(),
                           static_cast<std::size_t>(written.ptr -
                                                    buffer.data()));
        // A float must never spell an integer token — vread::integer is
        // the Integral gate, and a "3"-spelled float would alias it.
        if (lexeme.find_first_of(".eE") == std::string::npos)
            lexeme += ".0";
        return bio::Value{bio::Number{std::move(lexeme)}};
    }
    if (nb::isinstance(value, types.sequence) ||
        nb::isinstance(value, types.ndarray)) {
        // wire::sequence's own gate + nb::list materialization: the same
        // "invalid ordered sequence" wrapper on iteration failures.
        bio::Value::Array out;
        for (const nb::handle item : wire::sequence(value, path))
            out.push_back(plan_value(item, types, path));
        return bio::Value{std::move(out)};
    }
    return bio::Value{bio::Bytes{}};
}

// ---- Piece boxing -----------------------------------------------------------
// Verbatim move of the dict assembly the former monolithic encode.cpp
// ran: kind tags, data/keys/offsets payloads as owned numpy arrays or
// Python string lists, 'valid' None for the all-valid convention.

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
            out["kind"] = nb::str("struct");
            nb::dict children;
            for (const auto &[name, child] : piece.children)
                children[nb::str(name.c_str(), name.size())] =
                    to_python(child);
            out["children"] = children;
            // 'struct[]' slots keep their own offsets; a plain 'struct'
            // piece carries none, and Python tells them apart by the key.
            if (!piece.offsets.empty())
                out["offsets"] =
                    wire::owned_array<std::int32_t>(piece.offsets);
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
    const EncodePlan parsed =
        parse_encode_plan(plan_value(plan, plan_types(), "encode plan"));
    const EncodedTable encoded =
        encode_table_native(episode, parsed, start, limit);
    const nb::dict pieces;
    for (const auto &[name, piece] : encoded.columns)
        pieces[nb::str(name.c_str(), name.size())] = to_python(piece);
    return {encoded.rows, pieces};
}

nb::list applied_command_rows(const OwnedNativeEpisode &episode,
                              std::int64_t start, std::int64_t limit) {
    nb::list out;
    for (const AppliedCommandRow &record :
         applied_command_rows_native(episode, start, limit)) {
        nb::dict row;
        row["application_id"] = nb::int_(record.application_id);
        row["time_s"] = nb::float_(record.time_s);
        row["step"] = nb::int_(record.step);
        row["control"] = wire_to_python(record.control);
        // Demand keys stay absent when the record has none — the same
        // dict shape the former `if key in drive` projection produced.
        if (record.front_brake_demand.has_value())
            row["front_brake_demand"] =
                wire_to_python(*record.front_brake_demand);
        if (record.rear_brake_demand.has_value())
            row["rear_brake_demand"] =
                wire_to_python(*record.rear_brake_demand);
        if (record.rollback_brake_demand.has_value())
            row["rollback_brake_demand"] =
                wire_to_python(*record.rollback_brake_demand);
        out.append(row);
    }
    return out;
}

nb::list derived_event_rows(const OwnedNativeEpisode &episode,
                            nb::handle plan, std::int64_t start,
                            std::int64_t limit) {
    const DerivedEventPlan parsed = parse_derived_event_plan(
        plan_value(plan, plan_types(), "derived plan"));
    nb::list out;
    for (const DerivedEventRow &record :
         derived_event_rows_native(episode, parsed, start, limit)) {
        nb::dict changes;
        for (const auto &[label, value] : record.changes)
            changes[nb::str(label.c_str(), label.size())] =
                wire_to_python(value);
        nb::dict row;
        row["interval_id"] = nb::int_(record.interval_id);
        row["kind"] = nb::str("derived_transition");
        row["derived_kind"] = nb::str(record.derived_kind.c_str(),
                                      record.derived_kind.size());
        row["changes"] = std::move(changes);
        out.append(std::move(row));
    }
    return out;
}

void bind_provenance(nanobind::module_ &module) {
    module.def("runtime_identity", [] {
        const RuntimeIdentity identity = runtime_identity();
        nanobind::dict result;
        result["mujoco_version"] =
            nanobind::str(identity.mujoco_version.c_str(),
                          identity.mujoco_version.size());
        result["mujoco_library_path"] =
            nanobind::str(identity.mujoco_library_path.c_str(),
                          identity.mujoco_library_path.size());
        result["native_source_sha256"] =
            nanobind::str(identity.native_source_sha256.c_str(),
                          identity.native_source_sha256.size());
        result["build_context_json"] =
            nanobind::str(identity.build_context_json.c_str(),
                          identity.build_context_json.size());
        return result;
    });
}

} // namespace runtime
