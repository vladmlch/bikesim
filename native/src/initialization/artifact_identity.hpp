#pragma once
#include "../io/value.hpp"

namespace bike::initialization {
// Historical schema-2 execution identity is data only. Validation never
// imports an extension, resolves a recorded path or instantiates a policy.
// Native running-image provenance is owned by platform::build_identity and
// engine::identity; Python extension selection has no standalone analogue.
void validate_recorded_execution(const io::Value&);
} // namespace bike::initialization
