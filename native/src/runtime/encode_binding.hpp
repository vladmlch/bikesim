// encode_binding.hpp — the held-GIL adapter face over the pure
// runtime/encode.hpp projector and runtime/provenance.hpp identity
// (task T11). The pure TUs carry no Python; everything declared here is
// called from the extension's module/object registration with the GIL
// already held. These are the nb-facing twins of the native row/table
// APIs: plan handles materialize into the staged bike::io::Value the
// pure parser validates, and emitted pieces/records box into Python
// containers — byte-identical to the dicts the former monolithic
// encode.cpp produced.
#pragma once

#include <cstdint>
#include <utility>

#include <nanobind/nanobind.h>

#include "encode.hpp"
#include "provenance.hpp"

namespace runtime {

// Python plan dict + episode slice -> (row count, piece dicts).
std::pair<std::int64_t, nanobind::dict> encode_table(
    const OwnedNativeEpisode &episode, nanobind::handle plan,
    std::int64_t start, std::int64_t limit);
// commands_applied row dicts — the ordinary_commands mirror's Python
// face over the native AppliedCommandRow records.
nanobind::list applied_command_rows(const OwnedNativeEpisode &episode,
                                    std::int64_t start, std::int64_t limit);
// derived-transition row dicts over the native DerivedEventRow records;
// `plan` is the {'fields': [{'kind', 'path'}, ...]} dict native.py
// compiles from _DERIVED_FIELDS.
nanobind::list derived_event_rows(const OwnedNativeEpisode &episode,
                                  nanobind::handle plan,
                                  std::int64_t start, std::int64_t limit);
// runtime_identity() module registration — the provenance boundary's
// optional Python face over the pure RuntimeIdentity record.
void bind_provenance(nanobind::module_ &module);

} // namespace runtime
