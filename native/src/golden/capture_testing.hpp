#pragma once

#include "episode.hpp"

namespace bike::golden::testing {
// Cold regression seam for the original golden-capture monkeypatch test.
// The callback runs synchronously after the real apply_forces returns and
// before capture copies live qfrc_applied. It receives only that bounded
// buffer, never a model/data pointer; it must not retain the borrowed span.
// Preallocate any observation storage: the callback is nonthrowing.
using ForceExitMutation = void (*)(void *, std::span<double>) noexcept;

[[nodiscard]] EpisodeArtifact capture_with_force_exit_mutation(
    runtime::NativeResearchRuntime &env, std::size_t steps,
    const runtime::RideControl &control, void *context, ForceExitMutation mutation);

// Calls the same typed preflight used before spatial golden serialization.
// No runtime/model owner is needed to distinguish numeric NaN from text "nan".
void validate_spatial_numeric_evidence(const runtime::Wire&);
} // namespace bike::golden::testing
