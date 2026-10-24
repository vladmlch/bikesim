#pragma once

#include "presentation.hpp"
#include "../io/value.hpp"

namespace runtime {
// Private cold continuation codec. All numbers are saved values; no endpoint,
// force, sensor, policy or engine evaluation is performed during decoding.
[[nodiscard]] bike::io::Value encode_presentation_checkpoint(const PresentationState&);
[[nodiscard]] PresentationState decode_presentation_checkpoint(
    const bike::io::Value&, const PresentationState& model_bound_layout);
}
