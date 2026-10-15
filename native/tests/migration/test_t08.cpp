// test_t08.cpp — rider/drivetrain state-wire codec regressions (task
// T08): the produced bike::io::Value decoders/encoders —
// rider::decode_spindle_config, drivetrain::decode_drive_snapshot /
// encode_drive_snapshot and the surrounding spindle/intent/attachment/
// contacts wires — exercised in the Python-free graph (this target's
// compile context carries no Python/nanobind include, so compiling the
// codec headers IS the include-closure check; binding headers are
// deliberately absent).
//
// The oracle is the captured wire document itself: canonical_json emits
// number lexemes verbatim and sorts object members, so
// canonical_json(encode(decode(input))) == canonical_json(input) proves
// the codec round-trips every member, every optional/null distinction
// and every float spelling — signed zero included — before the
// checked-in golden pins the bytes. Emission ORDER (the binding's dict-
// literal update order) is verified separately on the Value's member
// sequence, which canonical sorting would otherwise hide.
//
// Rejections are the wire contract: every malformed member throws
// std::invalid_argument before any live state is touched — the decoders
// are pure functions, so nothing partial can escape.
#include "support/migration_test.hpp"
#include "drivetrain/state_codec.hpp"
#include "rider/state_codec.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "writer_config.hpp"
#include "stepper.hpp"
#include "diag.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace {

namespace bio = bike::io;

bio::Value parse(std::string_view text) { return bio::parse_json(text); }

bio::Value parse_fixture(std::string_view relative) {
    return parse(bt::text(bt::fixture(relative)));
}

[[nodiscard]] const bio::Value::Object &object_of(const bio::Value &v) {
    const auto *const out = std::get_if<bio::Value::Object>(&v.data);
    bt::require(out != nullptr, "encoded value must be an object");
    return *out;
}

[[nodiscard]] std::vector<std::string_view> member_names(
    const bio::Value::Object &object) {
    std::vector<std::string_view> out;
    out.reserve(object.size());
    for (const auto &[name, member]: object) out.push_back(name);
    return out;
}

// Member lookup with a fail-closed precondition — the fixture is declared
// shape-correct, so absence is an acquisition error, not a skipped check.
[[nodiscard]] const bio::Value &require_member(
    const bio::Value::Object &object, std::string_view key) {
    const bio::Value *const member = bio::find_member(object, key);
    if (member == nullptr)
        throw std::invalid_argument("fixture member missing: " +
                                    std::string(key));
    return *member;
}

// Number equality under the wire's semantic contract: integer-spelled
// lexemes compare exactly, float-spelled lexemes compare by IEEE-754 bit
// payload (the codec re-spells decoded doubles in Python repr form —
// e.g. a captured -1000000000000.0 re-emits as -1e+12 — while the value
// and the integer/float distinction are preserved exactly).
[[nodiscard]] bool numbers_equal(const bio::Number &a, const bio::Number &b) {
    const auto float_spelled = [](std::string_view lexeme) noexcept {
        return lexeme.find('.') != std::string_view::npos ||
               lexeme.find('e') != std::string_view::npos ||
               lexeme.find('E') != std::string_view::npos;
    };
    const bool af = float_spelled(a.lexeme), bf = float_spelled(b.lexeme);
    if (af != bf) return false;
    if (!af) return a.lexeme == b.lexeme;
    bool oka = false, okb = false;
    const double da = bio::detail::float_from_lexeme(a.lexeme, oka);
    const double db = bio::detail::float_from_lexeme(b.lexeme, okb);
    return oka && okb &&
           std::bit_cast<std::uint64_t>(da) == std::bit_cast<std::uint64_t>(db);
}

// Deep value equality: arrays order-sensitive, objects member-set
// (insertion ORDER is verified separately where the wire pins it), the
// leaf kinds compare directly.
// NOLINTNEXTLINE(misc-no-recursion) the Value tree is recursive by shape — depth is parser-bounded
[[nodiscard]] bool values_equal(const bio::Value &a, const bio::Value &b) {
    if (a.data.index() != b.data.index()) return false;
    if (const auto *const an = std::get_if<bio::Number>(&a.data))
        return numbers_equal(*an, std::get<bio::Number>(b.data));
    if (const auto *const aa = std::get_if<bio::Value::Array>(&a.data)) {
        const auto &ba = std::get<bio::Value::Array>(b.data);
        if (aa->size() != ba.size()) return false;
        for (std::size_t i = 0; i < aa->size(); ++i)
            if (!values_equal((*aa)[i], ba[i])) return false;
        return true;
    }
    if (const auto *const ao = std::get_if<bio::Value::Object>(&a.data)) {
        const auto &bo = std::get<bio::Value::Object>(b.data);
        if (ao->size() != bo.size()) return false;
        for (const auto &[name, av]: *ao) {
            const bio::Value *const bv = bio::find_member(bo, name);
            if (bv == nullptr || !values_equal(av, *bv)) return false;
        }
        return true;
    }
    // Remaining arms compare directly — Value itself carries no == since
    // the recursive variant cannot synthesize one.
    if (std::holds_alternative<std::monostate>(a.data)) return true;
    if (const auto *const ab = std::get_if<bool>(&a.data))
        return *ab == std::get<bool>(b.data);
    if (const auto *const as = std::get_if<std::string>(&a.data))
        return *as == std::get<std::string>(b.data);
    if (const auto *const ay = std::get_if<bio::Bytes>(&a.data))
        return ay->value == std::get<bio::Bytes>(b.data).value;
    const auto &at = std::get<bio::Tagged>(a.data);
    const auto &btt = std::get<bio::Tagged>(b.data);
    return at.kind == btt.kind && at.fields == btt.fields;
}

// Two round-trip proofs: golden_text byte-pins the canonical encode
// output, and values_equal proves every member survives the decode→
// encode path numerically (float spellings normalize to Python repr
// form; the signed-zero and integer/float distinctions are checked
// bit-exactly by numbers_equal).
void expect_round_trip(std::string_view golden_name,
                       const bio::Value &input, const bio::Value &encoded) {
    bt::golden_text(golden_name, bio::canonical_json(encoded));
    bt::require(values_equal(input, encoded),
                "round-trip must reproduce every captured member exactly");
}

[[nodiscard]] bio::Value encode_back_drive(
    const drivetrain::DriveSnapshot &state) {
    return drivetrain::encode_drive_snapshot(state);
}

void checks_drive_welded() {
    // ---- the welded capture: decode, inspect, encode, golden ----------
    // The decoded snapshots are box-held: DriveSnapshot carries the typed
    // 67-lane telemetry tables and pushes past the 8 KiB frame budget.
    const bio::Value input = parse_fixture("state/drive.json");
    const auto state = std::make_unique<const drivetrain::DriveSnapshot>(
        drivetrain::decode_drive_snapshot(input));
    // Signed zero survives the decode verbatim (IEEE-754 bit payload).
    bt::require(state->angles.has_value() && state->angles->count == 1 &&
                    std::bit_cast<std::uint64_t>(state->angles->values[0]) ==
                        0x8000000000000000ULL,
                "drive angle -0.0 must keep its sign bit");
    bt::require(state->reference.has_value() && *state->reference == 0.0,
                "reference 0.0 decodes");
    bt::require(!state->shift_time_s.has_value() &&
                    !state->last_time_s.has_value() &&
                    !state->psi.has_value(),
                "absent-valued wire members decode to absent optionals");
    bt::require(!state->pending_actuation.has_value() &&
                    !state->clutch.has_value() && !state->freewheel.has_value(),
                "null transmissions/actuation decode to empty optionals");
    bt::require(state->ideal_hub.has_value() &&
                    state->ideal_hub->rear_teeth == 51 &&
                    !state->ideal_hub->shift_pending,
                "ideal_hub transmission decodes");
    // Typed telemetry lanes: integer floor and closed labels ride the
    // schema; explicit presence states are observable.
    bt::require(
        state->last.present(drivetrain::TelemetryField::gear_front_teeth) &&
            std::get<int>(state->last.at("gear_front_teeth")) == 22,
        "integer telemetry lane decodes");
    bt::require(
        state->last.present(drivetrain::TelemetryField::gear_ratio),
        "real telemetry lane decodes");
    bt::require(state->probe_last.has_value() &&
                    state->probe_last->present(
                        drivetrain::TelemetryField::assist_mode) &&
                    std::get<std::string>(
                        state->probe_last->at("assist_mode")) == "turbo",
                "open telemetry lane decodes");
    if (state->probe_last) {
        bt::require(state->probe_last->presence(
                        drivetrain::TelemetryField::shift_time_s) == 2,
                    "explicit-null telemetry lane decodes as presence 2");
    } else {
        bt::require(false, "probe_last must be present");
    }

    // Independent-instance lifetime: a moved-out copy remains complete
    // and a later decode of the same document is unaffected.
    auto copy = std::make_unique<drivetrain::DriveSnapshot>(*state);
    const auto moved = std::make_unique<const drivetrain::DriveSnapshot>(
        std::move(*copy));
    bt::require(moved->ideal_hub.has_value() &&
                    moved->ideal_hub->rear_teeth == 51,
                "moved snapshot keeps transmission state");
    const auto again = std::make_unique<const drivetrain::DriveSnapshot>(
        drivetrain::decode_drive_snapshot(input));
    bt::require(again->ideal_hub.has_value() &&
                    again->ideal_hub->rear_teeth == 51,
                "repeat decode is independent of the first");

}

void checks_drive_roundtrip() {
    // Round-trip: canonical bytes equal the captured input exactly, and
    // the member sequence reproduces state_dict()'s update order.
    const bio::Value input = parse_fixture("state/drive.json");
    const auto state = std::make_unique<const drivetrain::DriveSnapshot>(
        drivetrain::decode_drive_snapshot(input));
    const bio::Value encoded = encode_back_drive(*state);
    expect_round_trip("state/drive.canonical.json", input, encoded);
    constexpr std::array<std::string_view, 12> kDriveOrder{
        "policies",   "shift_time_s", "last_time_s", "reference",
        "psi",        "angles",       "last",        "probe_last",
        "pending_actuation", "ideal_hub", "clutch", "freewheel"};
    bt::require(member_names(object_of(encoded)) ==
                    std::vector<std::string_view>(kDriveOrder.begin(),
                                                  kDriveOrder.end()),
                "drive emission must mirror state_dict key order");
    // Telemetry lanes emit in schema order — the wire's sorted sequence.
    if (const bio::Value *const last =
            bio::find_member(object_of(encoded), "last")) {
        const auto names = member_names(object_of(*last));
        bt::require(std::ranges::is_sorted(names),
                    "telemetry members emit in schema order");
    } else {
        bt::require(false, "encoded drive must carry last");
    }
    // The policies object's dict-literal order is part of the wire shape.
    if (const bio::Value *const policies =
            bio::find_member(object_of(encoded), "policies")) {
        constexpr std::array<std::string_view, 5> kPolicyOrder{
            "pedaling", "shifting", "assist", "battery", "hub"};
        bt::require(member_names(object_of(*policies)) ==
                        std::vector<std::string_view>(kPolicyOrder.begin(),
                                                      kPolicyOrder.end()),
                    "policy emission must mirror state_dict key order");
    } else {
        bt::require(false, "encoded drive must carry policies");
    }

}

void checks_drive_minimal() {
    // ---- absent-optional layout: every optional member is null --------
    const bio::Value minimal_input =
        parse_fixture("state/drive_minimal.json");
    const auto minimal = std::make_unique<const drivetrain::DriveSnapshot>(
        drivetrain::decode_drive_snapshot(minimal_input));
    bt::require(!minimal->angles.has_value() &&
                    !minimal->probe_last.has_value() &&
                    minimal->last.empty(),
                "minimal drive state decodes to absent optionals");
    expect_round_trip("state/drive_minimal.canonical.json", minimal_input,
                      encode_back_drive(*minimal));

}

void checks_drive_invalid() {
    // ---- rejection family: each fixture carries one defect ------------
    for (const char *name:
         {"state/invalid/drive_angle_width.json",   // 3 drive angles > 2
          "state/invalid/drive_label.json",         // closed-label value
          "state/invalid/drive_schema.json",        // missing required key
          "state/invalid/drive_unknown_key.json",   // foreign top-level key
          "state/invalid/drive_nonfinite.json",     // 1e400 lexeme overflow
          "state/invalid/drive_telemetry_range.json",  // integer under min
          "state/invalid/drive_pending_dt.json"}) {  // dt <= 0
        bt::invalid([&] {
            (void)drivetrain::decode_drive_snapshot(parse_fixture(name));
        });
    }
    // A rejecting decode leaves no residue: the valid document still
    // decodes afterwards, identical to before.
    const auto after = std::make_unique<const drivetrain::DriveSnapshot>(
        drivetrain::decode_drive_snapshot(parse_fixture("state/drive.json")));
    bt::require(after->ideal_hub.has_value() &&
                    after->ideal_hub->rear_teeth == 51,
                "decode after a rejection must still succeed");

    // Integer-vs-float spelling is a wire distinction: a float-spelled
    // tooth count is not an integer.
    bt::invalid([] {
        (void)drivetrain::decode_drive_snapshot(parse(
            R"({"policies":{"pedaling":{"coasting":false,
             "target_phase_rad":null,"target_rate_rad_s":0.0,
             "deceleration_rad_s2":0.0,"_effort":0.0,"_cadence_ema":null},
             "shifting":{"rear_teeth":22.0,"from_teeth":22,"shift_count":0,
             "cooldown_s":0.0,"cut_remaining_s":0.0,"direction":"none",
             "cadence_ema":null,"required_ema":null},
             "assist":{"torque":0.0,"pedaling":false,"last_gain":0.0},
             "battery":{"initial_energy_j":0.0,"energy_j":0.0,
             "drawn_energy_j":0.0},"hub":null},
             "shift_time_s":null,"last_time_s":null,"reference":0.0,
             "psi":null,"angles":null,"last":{},"probe_last":null,
             "pending_actuation":null,"ideal_hub":null,"clutch":null,
             "freewheel":null})"));
    });
}

void checks_spindle() {
    // The 66-key controller config plus resolved strength/envelope tables.
    const bio::Value config_doc =
        parse_fixture("config/rider_controller.json");
    const rider::SpindleConfig config =
        rider::decode_spindle_config(config_doc);
    bt::require(config.joint_kp_nm_rad == 600. &&
                    config.activation_tau_s == 0. &&
                    config.active_positive_power_limit_w.has_value() &&
                    *config.active_positive_power_limit_w == 450.,
                "spindle config decodes modeled scalars");
    bt::require(config.has_strength && !config.strength.empty() &&
                    !config.envelopes.empty(),
                "strength table and joint envelopes decode");
    // Copy/move lifetimes: the config is a plain value.
    rider::SpindleConfig copy = config;
    const rider::SpindleConfig moved = std::move(copy);
    bt::require(moved.has_strength, "moved config keeps strength table");

    // Controller state: decode then encode must reproduce the capture.
    const bio::Value state_input =
        parse_fixture("state/rider_controller.json");
    const rider::SpindleController::State spindle_state =
        rider::decode_spindle_state(state_input, "state.rider_controller");
    bt::require(spindle_state.enabled && spindle_state.command_enabled,
                "spindle state flags decode");
    bt::require(spindle_state.active_state.size() == 9,
                "the welded spindle active_state carries 9 values");
    bt::require(spindle_state.pedal_recovery.size() == 2 &&
                    spindle_state.last_terms.size() == 9,
                "pedal recovery and joint terms decode");
    expect_round_trip("state/rider_controller.canonical.json", state_input,
                      rider::encode_spindle_state(spindle_state));

    // The pd_split warm-carry rejection (spindle holds no PD split).
    bt::invalid([] {
        const bio::Value bad =
            parse_fixture("state/invalid/spindle_pd_split.json");
        (void)rider::decode_spindle_state(bad, "state.rider_controller");
    });
    // An unknown posture member rejects at the section gate.
    bt::invalid([] {
        (void)rider::decode_rider_posture(
            parse(R"({"torso_lean_rad":0.1,"pelvis_pitch_rad":0.0,
                      "pelvis_offset_m":null,"use_saddle":true,
                      "bogus":0})"),
            "posture");
    });
}

void checks_intent() {
    const bio::Value state_input = parse_fixture("state/rider_intent.json");
    const rider::RiderIntent::State intent_state =
        rider::decode_intent_state(state_input, "state.rider_intent");
    bt::require(intent_state.intent.effort_ceiling_nm >= 0.,
                "intent block decodes");
    expect_round_trip("state/rider_intent.canonical.json", state_input,
                      rider::encode_intent_state(intent_state));

    const bio::Value signals_input = parse_fixture("state/signals.json");
    const rider::IntentSignals signals =
        rider::decode_intent_signals(signals_input, "state.signals");
    expect_round_trip("state/signals.canonical.json", signals_input, rider::encode_intent_signals(signals));

    // resolve()'s control document: a negative optional real rejects.
    bt::invalid([] {
        (void)rider::decode_intent_control(
            parse_fixture("state/invalid/intent_negative_control.json"),
            "control");
    });
    // `type(step) is not int`: a float-spelled lexeme is not a step.
    bt::invalid([] {
        (void)rider::decode_intent_step(parse("2.0"), "control.step");
    });
    {
        const std::int64_t step =
            rider::decode_intent_step(parse("7"), "control.step");
        bt::require(step == 7, "integer step decodes");
    }
}

void checks_attachments() {
    {
        const bio::Value input =
            parse_fixture("state/attachment_geometry.json");
        const rider::AttachmentGeometry geometry =
            rider::decode_attachment_geometry(input, "attachment");
        bt::require(geometry.eq_id >= 0 && !geometry.rider_jac.values.empty(),
                    "attachment geometry decodes");
        expect_round_trip("state/attachment_geometry.canonical.json", input,
                          rider::encode_attachment_geometry(geometry));
    }
    for (const auto &[input_name, golden_name]: {
             std::pair{"state/attachment_raw.json",
                       "state/attachment_raw.canonical.json"},
             std::pair{"state/attachment_sample.json",
                       "state/attachment_sample.canonical.json"}}) {
        const bio::Value input = parse_fixture(input_name);
        const writers::RiderAttachmentEntry entry =
            rider::decode_attachment_entry(input, input_name);
        expect_round_trip(golden_name, input,
                          rider::encode_attachment_entry(entry));
    }
    // A ragged matrix member is not a 2-D float64 array.
    bt::invalid([] {
        (void)rider::decode_attachment_geometry(
            parse(R"({"eq_id":0,"rider_jac":[[1.0,2.0],[3.0]],
                      "rider_columns":[0],"bike_jac":[[1.0],[2.0]],
                      "bike_columns":[0],"observable":true,
                      "normal":[0.0,0.0,1.0],"kind":"pedal",
                      "rotational":false,"half_patch_m":0.025,
                      "pull_direction":null})"),
            "attachment");
    });
    // A non-integer eq_id is the record's own diagnostic family.
    bt::invalid([] {
        (void)rider::decode_attachment_geometry(
            parse(R"({"eq_id":1.5,"rider_jac":[],"rider_columns":[],
                      "bike_jac":[],"bike_columns":[],"observable":true,
                      "normal":[0.0,0.0,1.0],"kind":"pedal",
                      "rotational":false,"half_patch_m":0.025,
                      "pull_direction":null})"),
            "attachment");
    });
}

void checks_contacts_welded(const writers::RiderContactWriter &);
void checks_contacts_minimal(const writers::RiderContactWriter &);
void checks_contacts_invalid(const writers::RiderContactWriter &);

// The topology gate needs the live writer — welded_pinned's config
// carries linked saddle, spindle pedals and the welded connect grip,
// matching the captured state document. The setup lives in its own
// frame: WriterConfig + Stepper model/data over the 8 KiB budget.
[[nodiscard]] std::unique_ptr<Stepper> contacts_stepper() {
    const bio::Value config_doc =
        parse_fixture("bootstrap/welded_pinned/config.json");
    const auto config = std::make_unique<const nativecfg::WriterConfig>(
        nativecfg::decode(
            require_member(object_of(config_doc), "writer_config")));
    bt::require(config->rider_contacts.has_value(),
                "welded_pinned config must carry rider_contacts");
    auto stepper = std::make_unique<Stepper>(
        bt::fixture("bootstrap/welded_pinned/model.mjb").string(), *config);
    bt::require(stepper->has_rider_contacts(),
                "welded_pinned config must install the contacts writer");
    const writers::RiderContactWriter &writer = stepper->rider_contacts();
    bt::require(writer.linked_saddle() && writer.linked_pedals() &&
                    writer.welded_grip(),
                "welded_pinned topology must be fully linked");
    return stepper;
}

void checks_contacts() {
    const auto stepper = contacts_stepper();
    const writers::RiderContactWriter &writer = stepper->rider_contacts();
    checks_contacts_welded(writer);
    checks_contacts_minimal(writer);
    checks_contacts_invalid(writer);
}

void checks_contacts_welded(const writers::RiderContactWriter &writer) {
    const bio::Value input = parse_fixture("state/rider_contacts.json");
    // The decoded (state, probe) pair is box-held: the contacts state
    // carries attachment-sample and support-diagnostics tables that push
    // the frame past the 8 KiB budget.
    const auto decoded = std::make_unique<
        std::pair<writers::RiderContactsState,
                  std::optional<writers::RiderContactsProbe>>>(
        rider::decode_rider_contacts_state(input, writer));
    const writers::RiderContactsState &state = decoded->first;
    const auto &probe = decoded->second;
    bt::require(probe.has_value(),
                "the welded capture carries the probe triple");
    bt::require(state.settled_welds.has_value() &&
                    !state.settled_welds->entries.empty(),
                "settled welds decode against the writer topology");
    bt::require(state.attachment_samples.size() == 5,
                "five attachment sample entries decode");
    bt::require(state.diagnostics.evaluated &&
                    state.diagnostics.supports[0].detailed,
                "detailed welded diagnostics decode");
    const bio::Value encoded =
        rider::encode_rider_contacts_state(state, probe);
    expect_round_trip("state/rider_contacts.canonical.json", input, encoded);

    // Lifetime: the decoded state is an independent owner — a moved-out
    // copy keeps every attachment sample.
    const writers::RiderContactsState detached = std::move(decoded->first);
    bt::require(detached.attachment_samples.size() == 5,
                "moved contacts state keeps its samples");
}

void checks_contacts_minimal(const writers::RiderContactWriter &writer) {
    // The minimal document — no probe triple, no settled welds, the
    // unevaluated {} diagnostics — decodes and re-emits identically.
    const bio::Value minimal_input =
        parse_fixture("state/rider_contacts_minimal.json");
    const auto minimal_decoded = std::make_unique<
        std::pair<writers::RiderContactsState,
                  std::optional<writers::RiderContactsProbe>>>(
        rider::decode_rider_contacts_state(minimal_input, writer));
    bt::require(!minimal_decoded->second.has_value() &&
                    !minimal_decoded->first.settled_welds.has_value() &&
                    !minimal_decoded->first.diagnostics.evaluated,
                "minimal contacts state decodes to absent sections");
    expect_round_trip("state/rider_contacts_minimal.canonical.json",
                      minimal_input,
                      rider::encode_rider_contacts_state(
                          minimal_decoded->first, minimal_decoded->second));

}

void checks_contacts_invalid(const writers::RiderContactWriter &writer) {
    // Topology-gated rejections: a partial probe triple and a foreign
    // settle-weld name both fail closed.
    bt::invalid([&] {
        (void)rider::decode_rider_contacts_state(
            parse_fixture("state/invalid/contacts_probe_partial.json"),
            writer);
    });
    bt::invalid([&] {
        (void)rider::decode_rider_contacts_state(
            parse_fixture("state/invalid/contacts_weld_name.json"), writer);
    });
}

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(checks_drive_welded);
        failures += bt::run(checks_drive_roundtrip);
        failures += bt::run(checks_drive_minimal);
        failures += bt::run(checks_drive_invalid);
        failures += bt::run(checks_spindle);
        failures += bt::run(checks_intent);
        failures += bt::run(checks_attachments);
        failures += bt::run(checks_contacts);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
