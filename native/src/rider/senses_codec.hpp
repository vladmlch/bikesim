// senses_codec.hpp — strict owned-Value codecs for the rider senses
// channel (launch part 1, task 2). The documents ride inside the
// bootstrap state (`state.senses`, beside `signals`) and the runtime
// config envelope (`config.surface_labels`, beside `rider_intent`).
// Task 3 adds the delayed senses adapter's restorable state document.
#pragma once

#include "../io/value.hpp"
#include "front_lift_reaction.hpp"
#include "launch_plan.hpp"
#include "perception.hpp"
#include "senses.hpp"

namespace rider {

// state.senses — six exact keys; finite reals, integer rear_teeth >= 3,
// surface labels from the closed SurfaceLabel domain.
[[nodiscard]] bike::io::Value encode_rider_senses(const RiderSenses &s);
[[nodiscard]] RiderSenses decode_rider_senses(const bike::io::Value &v,
                                              std::string_view path);
// config.surface_labels — {"surface": <default label>, "sections":
// [{"start_m", "end_m", "surface": <label>}]}; the section list must be
// sorted and non-overlapping like SurfaceMap's constructor enforces.
[[nodiscard]] bike::io::Value
encode_surface_labels(const SurfaceLabels &s);
[[nodiscard]] SurfaceLabels
decode_surface_labels(const bike::io::Value &v, std::string_view path);

// The perception adapter's restorable document (launch part 1, task 3):
// {"tick", "samples": [[t, {"signals", "senses"}]], "delayed":
// {"signals", "senses"} | null, "pitch_rad"} — exact keys; `tick` and
// the row stamps are nonnegative int64 integers (follow-up 2: the
// integer clock replaced the accumulated double). The reachable-state
// invariants belong to Perception::restore, not the wire.
[[nodiscard]] bike::io::Value
encode_perception_state(const Perception::State &state);
[[nodiscard]] Perception::State
decode_perception_state(const bike::io::Value &v, std::string_view path);

// The reaction's restorable document: {"phase", "predicate",
// "decision_ticks", "clear_ticks", "effort_factor", "lean_add_rad",
// "percepts_seen", "hand_n", "crank_angle_rad", "hand_average_n",
// "crank_angle_total_rad"} — exact keys, phase one of
// idle/deciding/acting/releasing, int64 counters, finite reals and the
// bounded criterion-B history arrays. Reachability belongs to
// FrontLiftReaction::restore.
[[nodiscard]] bike::io::Value encode_reaction_state(const FrontLiftReaction::State &state);
[[nodiscard]] FrontLiftReaction::State decode_reaction_state(const bike::io::Value &v, std::string_view path);
// The runtime document of [front_lift_reaction]: the 14 keys, exact, then
// validate_reaction_table.
[[nodiscard]] bike::io::Value encode_reaction_config(const FrontLiftReactionConfig &config);
[[nodiscard]] FrontLiftReactionConfig decode_reaction_config(const bike::io::Value &v, std::string_view path);

// The plan's restorable document: {"phase", "phase_ticks", "launch_ticks",
// "end_ok_ticks", "blend_ticks", "effort_nm", "brake", "lean_target_rad",
// "shift_up", "request_count", "request_teeth", "blocked_teeth",
// "preload_missed"} — exact keys, the phase by name.
[[nodiscard]] bike::io::Value encode_launch_state(const LaunchPlan::State &state);
[[nodiscard]] LaunchPlan::State decode_launch_state(const bike::io::Value &v, std::string_view path);
// The runtime document of [rider_launch]: the 24 keys, exact, then
// validate_launch_table.
[[nodiscard]] bike::io::Value encode_launch_config(const LaunchConfig &config);
[[nodiscard]] LaunchConfig decode_launch_config(const bike::io::Value &v, std::string_view path);

} // namespace rider
