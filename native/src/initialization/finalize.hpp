#pragma once

namespace runtime { struct RuntimeConfig; }
namespace bike::initialization {
struct SetupContext;
struct ResearchStartup;
class PhysicalInitialState;
void finalize_initial_state(SetupContext&, const runtime::RuntimeConfig&);
[[nodiscard]] ResearchStartup prepare_research_startup(SetupContext&);
} // namespace bike::initialization
