// rider/spindle_wire.hpp — dict <-> struct conversion for the spindle
// controller's config, command, and bootstrap-state surfaces.
//
// The wire shapes are the production ones: `config.rider_controller` is
// `plain(asdict(ArticulatedConfig))` plus the resolved strength/envelope
// tables appended by bike_sim.native.setup; `state` sections are exactly
// ArticulatedRiderController.state_dict() output. Unknown keys, missing
// keys, and out-of-schema values are rejected — the wire never guesses.
#pragma once
#include <nanobind/nanobind.h>

#include <string_view>

#include "spindle_controller.hpp"

namespace rider {

namespace nb = nanobind;

[[nodiscard]] SpindlePose parse_spindle_pose(nb::handle value,
                                             std::string_view path);
[[nodiscard]] SpindleConfig parse_spindle_config(nb::handle value,
                                                 std::string_view path);
[[nodiscard]] RiderPosture parse_rider_posture(nb::handle value,
                                               std::string_view path);
[[nodiscard]] RiderCommand parse_rider_command(nb::handle value,
                                               std::string_view path);
[[nodiscard]] SpindleController::State
parse_spindle_state(nb::handle value, std::string_view path);

// Ordered joint-name -> scalar dict decode — the runtime bootstrap's
// held_control/held_rider_terms sections carry the same wire shapes.
[[nodiscard]] NamedEntries<double>
parse_named_reals(nb::handle value, std::string_view path);
[[nodiscard]] JointTerms parse_joint_terms(nb::handle value,
                                           std::string_view path);

// The exact ArticulatedRiderController.state_dict() shape.
[[nodiscard]] nb::dict
spindle_state_dict(const SpindleController::State &state);
// Diagnostics emission matching the Python dicts key-for-key.
[[nodiscard]] nb::dict
effort_diagnostics_dict(const EffortDiagnostics &diagnostics);
[[nodiscard]] nb::dict
allocation_diagnostics_dict(const AllocationDiagnostics &diagnostics);
[[nodiscard]] nb::dict
support_diagnostics_dict(const SupportDiagnostics &diagnostics);
[[nodiscard]] nb::dict sole_goal_diagnostics_dict(
    const NamedEntries<SoleGoalDiagnostics> &diagnostics);
[[nodiscard]] nb::dict last_terms_dict(
    const NamedEntries<JointTerms> &terms);

} // namespace rider
