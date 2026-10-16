// Immutable runtime identity compiled into the binary, never a sidecar
// claim (task T11: the pure half of the provenance boundary). The record
// carries the fields the former nb::dict produced, in declaration order —
// encode_binding.cpp's bind_provenance() is the optional module face;
// native consumers (run capture, recording metadata) read the record
// directly without a Python round-trip.
#pragma once

#include <string>

namespace runtime {

// The loaded engine + compiled-in build identity at call time.
struct RuntimeIdentity {
    std::string mujoco_version;
    std::string mujoco_library_path;
    std::string native_source_sha256;
    std::string build_context_json;
};

// Throws std::runtime_error when the loaded MuJoCo library cannot be
// identified — the same failure the module binding surfaced.
[[nodiscard]] RuntimeIdentity runtime_identity();

} // namespace runtime
