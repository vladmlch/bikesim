#pragma once

namespace runtime { struct RuntimeConfig; }
namespace bike::initialization {
struct SetupContext;
class PhysicalInitialState;
void finalize_initial_state(SetupContext&, const runtime::RuntimeConfig&);
} // namespace bike::initialization
