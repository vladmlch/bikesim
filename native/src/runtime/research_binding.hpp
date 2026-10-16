// research_binding.hpp — the nanobind adapter boundary over the pure
// runtime::NativeResearchRuntime (task T10).
//
// The research runtime itself is Python-free (research.hpp); this header
// declares only the adapter seams: the held-GIL warning emission and the
// module registration. The owned bootstrap/composition face the Python
// facade could hand documents to lives in research_io.hpp.
#pragma once
#include <nanobind/nanobind.h>

namespace runtime {
class NativeResearchRuntime;

// Held-GIL emission of the pending non-strict reference rejection — the
// RuntimeWarning the pure research runtime holds as state
// (NativeResearchRuntime::take_pending_warning) until the adapter
// surfaces it after advance_control.
void emit_research_warning(NativeResearchRuntime &runtime);

void bind_research(nanobind::module_ &module);
} // namespace runtime
