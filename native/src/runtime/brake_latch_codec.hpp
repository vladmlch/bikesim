// runtime/brake_latch_codec.hpp — the state.runtime.brake_latch wire
// shape: {"front": number|null, "rear": number|null}. Both directions
// are shared by the capture emitter (initialization/capture.cpp) and the
// step-state decoder (runtime.cpp) so the latch document has exactly one
// spelling — and by the task-6 test, which round-trips the real codec
// instead of a typed State copy (review P9).
#pragma once

#include <string_view>

#include "../io/value.hpp"
#include "static_brake.hpp"

namespace runtime {

[[nodiscard]] bike::io::Value
encode_brake_latch(const StaticBrake::State &state);

// Strict decode: the section must be a mapping with exactly {front, rear}
// and each member must be null or a finite real. A violation throws
// std::invalid_argument before any latch commits — PhysicalStep::restore
// never sees a half-decoded bite.
[[nodiscard]] StaticBrake::State
decode_brake_latch(const bike::io::Value &value, std::string_view path);

} // namespace runtime
