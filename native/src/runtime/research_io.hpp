// research_io.hpp — the owned research bootstrap boundary (task T10;
// migration/interface-contracts.md §research).
//
// A research bootstrap is one capture directory: the T09 ride trio
// (config.json / state.json / model.mjb) plus research.json carrying the
// research envelope, the setup-generated noise tape and the startup
// sensor state — the exact owned records the nanobind ctor parsed out of
// Python objects. read_research_bootstrap() resolves the externalized
// capture tags through the same capture-root convention as
// read_bootstrap() and decodes them with the same acceptance set and
// "path: reason" diagnostics as the nanobind readers (vread:: is the
// owned twin of wire::). make_research() composes the four owned records
// into the runtime — the research runtime consumes nothing borrowed.
#pragma once

#include <filesystem>
#include <memory>

#include "../io/value.hpp"
#include "research.hpp"
#include "runtime.hpp"

namespace runtime {

// One fully staged research bootstrap: the owned ride bootstrap plus the
// three research records the episode consumes.
struct ResearchBootstrap {
    RuntimeBootstrap ride;
    ResearchConfig config;
    NoiseTape tape;
    SensorState startup;
};

// Loads one research capture directory — the ride documents via
// read_bootstrap(), research.json via resolve_bootstrap_document() — into
// the owned ResearchBootstrap. The section decoders run in the adapter's
// ordinal order (research config, noise tape, sensor state, then the ride
// composition) and reject with the same diagnostics.
[[nodiscard]] ResearchBootstrap
read_research_bootstrap(const std::filesystem::path &directory);

// Composes a NativeResearchRuntime over its owned ride runtime from an
// owned ResearchBootstrap — the pure twin of the binding ctor's
// (handles -> records) half.
[[nodiscard]] std::unique_ptr<NativeResearchRuntime>
make_research(ResearchBootstrap bootstrap);

// Document-level decoders, exposed like resolve_bootstrap_document() so a
// fixture-level fault document composes against known-good peers at the
// same contract position.
[[nodiscard]] ResearchConfig
decode_research_config(const bike::io::Value &document);
[[nodiscard]] NoiseTape
decode_noise_tape(const bike::io::Value &noise,
                  const bike::io::Value &dropout_uniform);
[[nodiscard]] SensorState
decode_sensor_state(const bike::io::Value &document);

} // namespace runtime
