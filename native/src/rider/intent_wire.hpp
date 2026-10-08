// rider/intent_wire.hpp — dict <-> struct conversion for the seated-climb
// intent stack's config, signals, control and bootstrap-state surfaces.
//
// The wire shapes are the production ones: `config.rider_intent` is
// `plain(asdict(SeatedClimbConfig))`; `state.rider_intent` is exactly
// RiderIntentResolver.state_dict() output; `state.signals` is the
// setup boundary's _signals_dict. The `control` shape is the runtime
// boundary's _control_dict. Unknown keys, missing keys and out-of-schema
// values are rejected — the wire never guesses.
#pragma once
#include <nanobind/nanobind.h>

#include <string_view>

#include "intent.hpp"

namespace rider {

namespace nb = nanobind;

[[nodiscard]] IntentConfig parse_intent_config(nb::handle value,
                                               std::string_view path);
[[nodiscard]] IntentSignals parse_intent_signals(nb::handle value,
                                                 std::string_view path);
// rider_intent.py `type(step) is not int` — strict PyLong, no bools,
// numpy integers or int subclasses.
[[nodiscard]] std::int64_t parse_intent_step(nb::handle value,
                                             std::string_view path);
[[nodiscard]] IntentControl parse_intent_control(nb::handle value,
                                                 std::string_view path);
[[nodiscard]] RiderIntent::State parse_intent_state(nb::handle value,
                                                    std::string_view path);

// The exact state_dict()/_signals_dict()/_control_dict shapes.
[[nodiscard]] nb::dict intent_state_dict(const RiderIntent::State &state);
[[nodiscard]] nb::dict intent_signals_dict(const IntentSignals &signals);
[[nodiscard]] nb::dict intent_control_dict(const IntentControl &control);
[[nodiscard]] nb::dict rider_posture_dict(const RiderPosture &posture);
[[nodiscard]] nb::dict seated_intent_dict(const Intent &intent);

} // namespace rider
