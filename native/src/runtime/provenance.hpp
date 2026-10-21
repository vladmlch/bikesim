// Immutable runtime identity compiled into the binary, never a sidecar
// claim (task T11: the pure half of the provenance boundary). The record
// carries the fields the former nb::dict produced, in declaration order —
// encode_binding.cpp's bind_provenance() is the optional module face;
// native consumers (run capture, recording metadata) read the record
// directly without a Python round-trip.
#pragma once

#include "provider_images.hpp"

#include <string>
#include <vector>

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

// The qualified provider policy the build was configured with (JSON text).
// The Linux root build may substitute exact adjacent installed paths after
// verifying their committed SHA-256 and actual loader membership. All other
// contexts return the compile-time constant unchanged.
[[nodiscard]] std::string numerics_provider_policy_json();

// The provider images actually loaded into this process, checked against the
// configure-time list (or the verified Linux root install paths). Loader
// inspection: identity boundaries only, never
// inside a step loop. Kept out of runtime_identity() so that a missing
// provider image cannot make every identity consumer fail. Throws
// std::runtime_error when a required image is not loaded or the platform
// cannot inspect loaded images.
[[nodiscard]] std::vector<ProviderImage> provider_images();

} // namespace runtime
