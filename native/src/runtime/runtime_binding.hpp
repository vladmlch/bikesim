// runtime_binding.hpp — the nanobind adapter boundary over the pure
// runtime::NativeRideRuntime (task T09).
//
// The ride runtime itself is Python-free (runtime.hpp); this header
// declares only the adapter seams: the handle materializer/compose bridge
// the two ctor sites share, the held-GIL warning emission, and the module
// registration.
#pragma once
#include <nanobind/nanobind.h>

#include "runtime.hpp"

namespace runtime {

// The adapter half of the old (model_path, config, state) ctor contract:
// os.fspath resolution for the artifact, the bytes read, the Python
// documents materialized into owned bike::io::Value trees, then the shared
// compose_bootstrap validation order — config envelope, artifact digest,
// writer-config decode, model image load, state sections.
[[nodiscard]] RuntimeBootstrap
compose_bootstrap_handles(nanobind::handle model_path, nanobind::handle config,
                          nanobind::handle state);

// Held-GIL emission of the pending non-strict reference rejection — the
// RuntimeWarning the pure runtime holds as state
// (NativeRideRuntime::take_pending_warning) until the adapter surfaces it.
void emit_reference_warning(NativeRideRuntime &runtime);

void bind_runtime(nanobind::module_ &module);

} // namespace runtime
