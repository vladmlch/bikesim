// bootstrap_io.hpp — the filesystem face of the owned bootstrap (T09).
//
// read_bootstrap() loads one capture_bootstrap directory — config.json,
// state.json and model.mjb — into the owned RuntimeBootstrap. The fixture
// JSON carries externalized payloads as capture tags ({"$array": path,
// dtype, order, shape, sha256}, {"$nonfinite": ...}, {"$tuple": [...]},
// {"$dataclass": ..., "fields": {...}}, {"$bytes": path, sha256, bytes});
// this layer resolves them before the staged decode ever runs, verifying
// every declared digest/shape against the payload bytes.
//
// Array tag paths are capture-root-relative: the written fixture tree is
// <root>/bootstrap/<name>/..., and a tag records
// "bootstrap/<name>/state.arrays/aNNN.f64". The resolver climbs the
// bootstrap directory's ancestors — the nearest ancestor under which the
// declared path exists is the capture root; an unresolvable or escaping
// path fails closed.
#pragma once

#include <filesystem>

#include "../io/value.hpp"
#include "runtime.hpp"

namespace runtime {

// Resolves the externalized capture tags in one parsed bootstrap document
// — the read_bootstrap() half exposed separately so a fixture-level fault
// document composes against a known-good model/state pair at the same
// contract position (digest, payload and shape checks all run).
[[nodiscard]] bike::io::Value
resolve_bootstrap_document(const bike::io::Value &document,
                           const std::filesystem::path &directory);

} // namespace runtime
