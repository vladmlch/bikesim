// record_initial_state.hpp — BIKE_GOLDEN_RECORD support for schema-2
// physical initial-state documents (initialization::PhysicalInitialState).
// Nothing else in this checkout writes these documents: they are captured by
// SetupOwner::capture_initial_state() after the same fresh-start settle the
// consuming path performs. Outside record mode every entry point is a no-op.
#pragma once

#include <filesystem>

namespace bike::research { struct ResearchRequest; }

namespace bt {

// Regenerates the document the request's `initial_state_file` resolves to on
// the running platform — the same platform-keyed sibling resolution the
// request loaders apply (initialization/request.cpp, research/
// configuration.cpp). The request is read the way its consumer reads it: a
// request whose top-level keys fit the strict setup-request schema is loaded
// with initialization::read_setup_request(skip_initial_state = true); any
// other request goes through research::read_research_request and gets the
// setup mutations prepare_environment applies before its settle.
void record_initial_state(const std::filesystem::path& request);

// The same regeneration for a request assembled in code: a copy of
// request.setup is settled after the research pre-settle mutations and the
// capture is written to `document` verbatim.
void record_initial_state(const bike::research::ResearchRequest& request,
                          const std::filesystem::path& document);

} // namespace bt
