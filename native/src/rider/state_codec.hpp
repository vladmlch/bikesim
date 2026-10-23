// rider/state_codec.hpp — the owned bike::io::Value state codecs for the
// rider boundary (task T08; migration/interface-contracts.md §1/§2).
//
// These are the Python-free twins of the nanobind wire readers in
// spindle_wire.cpp, intent_wire.cpp, attachment_binding.cpp and
// rider_contact_binding.cpp — the typed records stay owned by
// spindle_controller.hpp / intent.hpp / rider_posture.hpp /
// attachment_wrench.hpp / writers/rider_contacts.hpp; nothing binding-
// specific enters this header. Every decoder is a pure function of the
// wire document: a malformed member, a foreign topology layout, a bad
// label or a width mismatch throws std::invalid_argument naming the
// public path before any live state is touched.
//
// The schemas are closed and mirrored exactly:
//   * `config.rider_controller` — the 66-key ArticulatedConfig document
//     plus the resolved strength/envelope tables (decode_spindle_config).
//   * `state.rider_controller` — SpindleController::State: terms, IK
//     saturates, pedal recovery and the four diagnostics sections.
//   * `state.rider_intent` / `state.signals` / the resolve() `control`
//     document — the seated-climb intent stack.
//   * the attachment records (AttachmentGeometry, AttachmentRaw,
//     AttachmentSample) and the contacts writer's serializable state —
//     topology-gated against the live RiderContactWriter exactly like
//     parse_rider_contacts_state().
//
// Path arguments spell the document's public position
// ("state.rider_controller", "control", ...) so diagnostics match the
// binding's verbatim; the pathless produced entry point uses
// "config.rider_controller".
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../io/value.hpp"
#include "../writers/rider_contacts.hpp"
#include "attachment_wrench.hpp"
#include "intent.hpp"
#include "rider_posture.hpp"
#include "spindle_controller.hpp"

namespace rider {

    // ---- produced boundary ------------------------------------------------

    // `config.rider_controller` -> SpindleConfig. Runs the wire's exact
    // key gate, every scalar's finite-real/boolean/string check, the
    // unmodeled-key type validation, physical attachment-mode gates,
    // and the torque-curve/joint-envelope
    // domain validators — the same acceptance set as the binding's
    // parse_spindle_config.
    [[nodiscard]] SpindleConfig
    decode_spindle_config(const bike::io::Value &value);
    [[nodiscard]] SpindleConfig
    decode_spatial_spindle_config(const bike::io::Value &value);
    // Plain ArticulatedConfig members (without strength/envelope tables).
    [[nodiscard]] bike::configuration::ArticulatedConfig
    decode_articulated_config(const bike::io::Value &value);

    // ---- spindle controller state wire ------------------------------------

    [[nodiscard]] SpindlePose
    decode_spindle_pose(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] SpindlePose
    decode_spatial_spindle_pose(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] RiderPosture
    decode_rider_posture(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] RiderCommand
    decode_rider_command(const bike::io::Value &value, std::string_view path);
    // Ordered joint-name -> scalar dict decode — the held_control/
    // held_rider_terms wire shapes the runtime bootstrap reads.
    [[nodiscard]] NamedEntries<double>
    decode_named_reals(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] JointTerms
    decode_joint_terms(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] SoleGoalDiagnostics
    decode_sole_goal(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] bike::io::Value
    encode_sole_goal(const SoleGoalDiagnostics &goal);
    // The exact ArticulatedRiderController.state_dict() document.
    [[nodiscard]] SpindleController::State
    decode_spindle_state(const bike::io::Value &value, std::string_view path);
    // The state_dict() inverse — same key set and emission order.
    [[nodiscard]] bike::io::Value
    encode_spindle_state(const SpindleController::State &state);
    [[nodiscard]] bike::io::Value
    encode_rider_posture(const RiderPosture &posture);

    // ---- intent stack wires ----------------------------------------------

    [[nodiscard]] IntentConfig
    decode_intent_config(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] IntentSignals
    decode_intent_signals(const bike::io::Value &value,
                          std::string_view path);
    // rider_intent.py resolve()'s `type(step) is not int` gate — a
    // float-spelled integer lexeme (2.0) or a bool rejects exactly like
    // the binding's PyLong_CheckExact.
    [[nodiscard]] std::int64_t
    decode_intent_step(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] IntentControl
    decode_intent_control(const bike::io::Value &value,
                          std::string_view path);
    [[nodiscard]] RiderIntent::State
    decode_intent_state(const bike::io::Value &value, std::string_view path);
    [[nodiscard]] bike::io::Value
    encode_intent_state(const RiderIntent::State &state);
    [[nodiscard]] bike::io::Value
    encode_intent_signals(const IntentSignals &signals);
    [[nodiscard]] bike::io::Value
    encode_intent_control(const IntentControl &control);

    // ---- attachment record wires -------------------------------------------

    // The AttachmentGeometry document: 'rider_jac'/'bike_jac' decode from
    // the wire's nested-row matrix spelling into DenseMatrix; observable/
    // rotational take JSON-truthy values like the oracle's PyObject_IsTrue.
    [[nodiscard]] AttachmentGeometry
    decode_attachment_geometry(const bike::io::Value &value,
                               std::string_view path);
    // One last_attachment_samples entry — 'rider_jac' membership selects
    // the raw block over the budget sample, exactly like the binding.
    [[nodiscard]] writers::RiderAttachmentEntry
    decode_attachment_entry(const bike::io::Value &value,
                            std::string_view path);
    [[nodiscard]] bike::io::Value
    encode_attachment_geometry(const AttachmentGeometry &geometry);
    [[nodiscard]] bike::io::Value
    encode_attachment_entry(const writers::RiderAttachmentEntry &entry);

    // ---- rider contacts state wire -----------------------------------------

    // The contacts-state document — topology gates (linked saddle,
    // linked pedals, welded grip) come from the live writer exactly like
    // parse_rider_contacts_state(); the decode itself never mutates it.
    [[nodiscard]] std::pair<writers::RiderContactsState,
                            std::optional<writers::RiderContactsProbe>>
    decode_rider_contacts_state(const bike::io::Value &value,
                                const writers::RiderContactWriter &writer);
    [[nodiscard]] bike::io::Value encode_rider_contacts_state(
        const writers::RiderContactsState &state,
        const std::optional<writers::RiderContactsProbe> &probe);

} // namespace rider
