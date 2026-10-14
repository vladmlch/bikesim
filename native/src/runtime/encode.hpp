// Catalog-plan column projector for the native recording encoder.
// Emits raw column payloads ("pieces") — flat buffers, offsets, validity
// masks and nested child pieces — never Arrow objects and never per-row
// Python dicts; recording/native_encode.py assembles the RecordBatch.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nanobind/nanobind.h>

#include "samples.hpp"

namespace nb = nanobind;

namespace runtime {

// One emitted Arrow column's raw payload; kind picks which members are live.
struct Piece {
    enum class Kind : std::uint8_t { scalar_d, scalar_i, scalar_b, scalar_s,
                      list_d, list_i, list_s, structure, map }
        kind = Kind::scalar_d;
    std::vector<double> d;
    std::vector<std::int64_t> i;
    std::vector<char> b;                    // bool payload
    std::vector<std::string> s;             // varchar payload / map keys
    std::vector<std::int32_t> offsets;      // list/struct[]/map member spans
    std::vector<char> valid;                // empty => all-valid
    std::map<std::string, Piece> children;  // struct members incl *_present
    std::unique_ptr<Piece> items;           // map values / struct[] elements
};

struct EncodePlan;                          // parsed plan, Task 3 grows it

// Dispatch entry: parses plan, runs the emitter, boxes pieces.
std::pair<std::int64_t, nb::dict> encode_table(
    const OwnedNativeEpisode &episode, nb::handle plan,
    std::int64_t start, std::int64_t limit);

// Applied-command diff projection for ordinary rides — the C++ twin of the
// Python mirror in recording/native.py::NativePhysicalSource.tables: one
// row dict per interval whose (control, brake demands) tuple differs from
// the previous interval's, so a ride's commands_applied table emits only
// control-boundary rows. Rows [0, start) are replayed to seed the compare
// and application_id, keeping chunked calls identical to one full pass.
nb::list applied_command_rows(const OwnedNativeEpisode &episode,
                              std::int64_t start, std::int64_t limit);

// Derived-transition scan — the C++ twin of the channel diff inside
// recording/native.py::iter_derived_events: one row dict per
// (interval, kind) whose tracked labels changed against the previous
// retained interval, boxing only the changed values. `plan` is
// {'fields': [{'kind': str, 'path': [str, ...]}, ...]}, compiled from
// _DERIVED_FIELDS in native.py. Rows [0, start) are skipped but the
// previous-row compare state seeds from row start-1, keeping chunked
// calls identical to one full pass. event_id assignment and the
// InputEvent wrap stay in Python.
nb::list derived_event_rows(const OwnedNativeEpisode &episode,
                            nb::handle plan, std::int64_t start,
                            std::int64_t limit);

} // namespace runtime
