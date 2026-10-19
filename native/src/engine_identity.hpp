// engine_identity.hpp — resolved engine/library/provider identity (T04).
//
// engine::identity() reports the engine actually loaded into THIS process:
// mj_versionString() returns a static char array inside the mapped MuJoCo
// image, so dladdr on that data address names the resolved library file,
// whose bytes are then SHA-256'd. This is the runtime second layer behind
// T03's configure-time acquisition probes (version + artifact sha256 +
// private-ABI symbols): configure proves what SHOULD load, identity()
// proves what DID load. It deliberately does not consult any Python
// package or wheel directory — there may be none.
//
// Identity.numeric_provider is the recorded supplier name of the pinned
// binary64 numeric surface (the Apple Accelerate CBLAS entry points in
// cblas_abi.hpp, the platform libm calls behind numeric_sincos.hpp, and
// the ported NumPy ufunc kernels in numeric_numpy.hpp): blas::provider_name
// of the backend this build links — "accelerate" on Darwin,
// "scipy-openblas" on Linux. The field pins the supplier identity as
// recorded; the bitwise fixtures pin the numerical results themselves.
#pragma once

#include <string>

namespace engine {

// Resolved identity triple. Every field is a verbatim string: no
// normalization, no wildcard matching, no tolerance.
struct Identity {
    std::string mujoco_version;
    std::string library_sha256;
    std::string numeric_provider;
};

// Resolves the engine currently loaded in this process. Throws
// std::runtime_error when the loaded MuJoCo image cannot be identified
// (no version string, no image path) or its file bytes cannot be hashed.
[[nodiscard]] Identity identity();

// Field-by-field verbatim comparison of `required` against the resolved
// identity. A mismatch throws std::invalid_argument naming the field and
// both values; an equal identity returns silently.
void verify_identity(const Identity &required);

} // namespace engine
