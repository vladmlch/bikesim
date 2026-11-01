// Completion 3.15 (T52). Source: tests/reference/test_native_rider_intent.py.
// The seated-climb intent stack — RiderIntent (rider_intent.py
// RiderIntentResolver), SeatedClimbPolicy and SeatedPostureProgram —
// replayed against the platform goldens recorded from the oracle file.
// Every node is a recorded NativeTestAdapter `rider_intent` session:
// `input.<op>.k` entries carry the canonical-JSON argument document (one
// UTF-8 byte per f64 lane), `output.<leaf>.k` entries are the oracle's
// canonical-JSON or f64-scalar results compared byte/bit-for-bit, and
// `error.<op>.k` empty |u1 markers freeze "the wire call raised" outcomes.
// peek_label() selects the branch exactly like the recorded pytest.raises
// blocks.

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "io/float_format.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "rider/intent.hpp"
#include "rider/state_codec.hpp"
#include "support/golden.hpp"
#include "support/migration_test.hpp"

namespace {
namespace bio = bike::io;

constexpr std::string_view kFile =
    "tests/reference/test_native_rider_intent.py::";

// Per-tag counters mirror the source file's _SEQ — a label mismatch is
// itself a contract failure.
struct Seq {
    std::map<std::string, std::size_t> counters;

    std::string next(std::string_view tag) {
        const std::size_t index = counters[std::string(tag)]++;
        return std::string(tag) + "." + std::to_string(index);
    }
};

// ---- captured-input plumbing --------------------------------------------------

// input.* entries are canonical-JSON text carried one byte per f64 lane.
std::string decode_f64_text(const std::vector<double> &encoded) {
    std::string out;
    out.reserve(encoded.size());
    for (const double code : encoded)
        out.push_back(static_cast<char>(static_cast<unsigned char>(code)));
    return out;
}

bio::Value input_doc(bt::oracle::Node &golden, Seq &seq, const char *op) {
    const std::vector<double> encoded =
        golden.f64_input(seq.next(std::string("input.") + op));
    return bio::parse_recording_json(decode_f64_text(encoded));
}

const bio::Value &member(const bio::Value &doc, const char *key) {
    const auto &object = bio::as_object(doc, "input doc");
    const bio::Value *found = bio::find_member(object, key);
    bt::require(found != nullptr,
                std::string("input doc carries '") + key + "'");
    return *found;
}

// The recorded call documents freeze their argument set; a foreign key is a
// fixture defect, not a contract branch.
void require_keys(const bio::Value &doc,
                  std::initializer_list<const char *> keys) {
    const auto &object = bio::as_object(doc, "input doc");
    bt::require(object.size() == keys.size(),
                "input doc carries exactly the recorded argument set");
    for (const char *key : keys)
        bt::require(bio::find_member(object, key) != nullptr,
                    std::string("input doc carries '") + key + "'");
}

// wire::real analogs for the recorded documents: Number lexemes convert
// with float() semantics; the float64 recording tag admits the oracle's
// nonfinite spellings (nan, +inf, -inf) to the lazily-validated scalars.
double real_arg(const bio::Value &doc, const char *key) {
    return bio::tagged_double(member(doc, key), key);
}

std::optional<double> optional_arg(const bio::Value &doc, const char *key) {
    const bio::Value &value = member(doc, key);
    return bio::is_null(value)
               ? std::nullopt
               : std::optional<double>(bio::tagged_double(value, key));
}

bool bool_arg(const bio::Value &doc, const char *key) {
    return bio::as_bool(member(doc, key), key);
}

// ---- recorded-output plumbing ------------------------------------------------

void json_out(bt::oracle::Node &golden, Seq &seq, const char *tag,
              const bio::Value &doc) {
    const std::string text = bio::recording_json(doc);
    const std::array<std::size_t, 1> shape{text.size()};
    golden.bytes(std::as_bytes(std::span(text)), "|u1", shape,
                 seq.next(std::string("output.") + tag));
}

void state_out(bt::oracle::Node &golden, Seq &seq,
               const rider::RiderIntent &intent) {
    json_out(golden, seq, "state",
             rider::encode_intent_state(intent.state()));
}

void scalar_out(bt::oracle::Node &golden, Seq &seq, const char *tag,
                double value) {
    const std::array<double, 1> actual{value};
    golden.f64(actual, std::span<const std::size_t>{},
               seq.next(std::string("output.") + tag));
}

void consumed_error(bt::oracle::Node &golden, Seq &seq, const char *op) {
    const std::array<std::size_t, 1> shape{0};
    golden.bytes({}, "|u1", shape,
                 seq.next(std::string("error.") + op));
}

bio::Value real_leaf(double value) {
    if (std::isnan(value))
        return bio::Value{bio::Tagged{.kind = "float64",
                                      .fields = {{"value", "nan"}}}};
    if (std::isinf(value))
        return bio::Value{bio::Tagged{
            .kind = "float64",
            .fields = {{"value", value < 0. ? "-inf" : "+inf"}}}};
    return bio::Value{bio::Number{bio::repr_binary64(value)}};
}

// ---- the recorded session ----------------------------------------------------

// The adapter's `rider_intent` section lands here: construction is one
// recorded call, every later op replays through the engaged resolver.
struct Plant {
    std::optional<rider::RiderIntent> intent;
    // The last resolve() result — the source's `native` dict asserts.
    std::optional<rider::IntentControl> last_control;
};

rider::RiderIntent &require_intent(Plant &plant) {
    bt::require(plant.intent.has_value(),
                "the recorded adapter construction succeeded");
    return (plant.intent).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
}

std::string state_text(const rider::RiderIntent &intent) {
    return bio::recording_json(rider::encode_intent_state(intent.state()));
}

void op_adapter(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "adapter");
    require_keys(doc, {"config", "dt_s"});
    try {
        plant.intent.emplace(
            rider::decode_intent_config(member(doc, "config"),
                                        "adapter.rider_intent.config"),
            real_arg(doc, "dt_s"));
    } catch (const std::exception &) {
        consumed_error(golden, seq, "adapter");
    }
}

void op_resolve(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "resolve");
    require_keys(doc, {"control", "signals", "step", "road_grade",
                       "preview_grade", "lean_limit_rad", "active",
                       "advance"});
    auto &intent = require_intent(plant);
    std::optional<rider::IntentControl> result;
    try {
        // Adapter order (test_adapter.cpp::intent_resolve): control,
        // signals, the lazily-validated real scalars, then — gated on
        // enabled/active exactly like rider_intent.py — the strict step.
        const auto control = rider::decode_intent_control(
            member(doc, "control"), "resolve.control");
        const auto signals = rider::decode_intent_signals(
            member(doc, "signals"), "resolve.signals");
        const double grade = real_arg(doc, "road_grade");
        const auto preview = optional_arg(doc, "preview_grade");
        const auto lean = optional_arg(doc, "lean_limit_rad");
        const bool active = bool_arg(doc, "active");
        const bool advance = bool_arg(doc, "advance");
        const std::int64_t step =
            (intent.config().enabled && active)
                ? rider::decode_intent_step(member(doc, "step"),
                                            "resolve.step")
                : 0;
        result = intent.resolve(control, signals, step, grade, preview,
                                lean, active, advance);
    } catch (const std::exception &) {
        consumed_error(golden, seq, "resolve");
        return;
    }
    plant.last_control = result;
    json_out(golden, seq, "control",
             rider::encode_intent_control(*result));
    state_out(golden, seq, intent);
}

void op_update(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "update");
    require_keys(doc, {"signals", "dt_s", "road_grade", "preview_grade",
                       "lean_limit_rad"});
    auto &intent = require_intent(plant);
    std::optional<rider::Intent> result;
    try {
        const auto signals = rider::decode_intent_signals(
            member(doc, "signals"), "update.signals");
        result = intent.policy().update(
            signals, real_arg(doc, "dt_s"), real_arg(doc, "road_grade"),
            optional_arg(doc, "preview_grade"),
            optional_arg(doc, "lean_limit_rad"));
    } catch (const std::exception &) {
        consumed_error(golden, seq, "update");
        return;
    }
    bio::Value::Object out;
    out.emplace_back("posture",
                     rider::encode_rider_posture(result->posture));
    out.emplace_back("effort_ceiling_nm",
                     real_leaf(result->effort_ceiling_nm));
    json_out(golden, seq, "intent", bio::Value{std::move(out)});
}

void op_power(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "power");
    require_keys(doc, {"preview_grade", "dt_s"});
    auto &intent = require_intent(plant);
    std::optional<double> result;
    try {
        result = intent.policy().power_target_w(
            real_arg(doc, "preview_grade"), real_arg(doc, "dt_s"));
    } catch (const std::exception &) {
        consumed_error(golden, seq, "power");
        return;
    }
    scalar_out(golden, seq, "power", *result);
}

void op_program(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "program");
    require_keys(doc, {"time_s", "road_grade", "dt_s", "front_load_share",
                       "lean_limit_rad", "dead_time_s"});
    auto &intent = require_intent(plant);
    std::optional<double> result;
    try {
        result = intent.policy().program().update(
            real_arg(doc, "time_s"), real_arg(doc, "road_grade"),
            real_arg(doc, "dt_s"), optional_arg(doc, "front_load_share"),
            optional_arg(doc, "lean_limit_rad"),
            optional_arg(doc, "dead_time_s"));
    } catch (const std::exception &) {
        consumed_error(golden, seq, "program");
        return;
    }
    scalar_out(golden, seq, "program", *result);
}

void op_pulse(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "pulse");
    require_keys(doc, {"start_s", "duration_s", "amplitude_rad"});
    try {
        require_intent(plant).schedule_pulse(real_arg(doc, "start_s"),
                                             real_arg(doc, "duration_s"),
                                             real_arg(doc, "amplitude_rad"));
    } catch (const std::exception &) {
        consumed_error(golden, seq, "pulse");
    }
}

void op_restore(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const bio::Value doc = input_doc(golden, seq, "restore");
    require_keys(doc, {"state"});
    try {
        require_intent(plant).restore(rider::decode_intent_state(
            member(doc, "state"), "state"));
    } catch (const std::exception &) {
        consumed_error(golden, seq, "restore");
    }
}

// Consume exactly one recorded event: one input.<op>.k call plus its
// recorded outputs or error marker, or one bare output.state.k leaf.
void event(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    const std::string_view label = golden.peek_label();
    bt::require(!label.empty(), "event() called past the node end");
    if (label.starts_with("output.state.")) {
        state_out(golden, seq, require_intent(plant));
        return;
    }
    bt::require(label.starts_with("input."),
                "a recorded event starts with input.* or output.state.*");
    const std::size_t split = label.rfind('.');
    const std::string_view op = label.substr(6, split - 6);
    if (op == "adapter") return op_adapter(plant, golden, seq);
    if (op == "resolve") return op_resolve(plant, golden, seq);
    if (op == "update") return op_update(plant, golden, seq);
    if (op == "power") return op_power(plant, golden, seq);
    if (op == "program") return op_program(plant, golden, seq);
    if (op == "pulse") return op_pulse(plant, golden, seq);
    if (op == "restore") return op_restore(plant, golden, seq);
    bt::require(false, "recorded op is part of the rider_intent contract");
}

void drain(Plant &plant, bt::oracle::Node &golden, Seq &seq) {
    while (golden.has_next())
        event(plant, golden, seq);
}

// pytest.approx(rel=1e-12, abs=1e-12) for the handful of native-side
// member asserts the source keeps next to the byte comparisons.
bool approx12(double actual, double expected) {
    return actual == expected ||
           std::abs(actual - expected) <=
               1e-12 + 1e-12 * std::abs(expected);
}

// ---- the 26 source nodes ---------------------------------------------------

void test_resolve_matches_oracle_over_mixed_steps() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 35; ++i) {
        event(plant, golden, seq);  // _resolve: control + state
        event(plant, golden, seq);  // _state_cmp leaf
    }
    golden.done();
}

void test_resolve_disabled_passthrough() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);
    for (int i = 0; i < 3; ++i)
        event(plant, golden, seq);
    event(plant, golden, seq);  // _state_cmp
    golden.done();
}

void test_resolve_merges_only_unset_fields() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
    const auto &control = (plant.last_control).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
    bt::require(control.posture.has_value() &&
                    approx12(control.posture->torso_lean_rad, .12),
                "resolve keeps the held torso_lean_rad");
    bt::require(control.human_torque_nm.has_value() &&
                    approx12(*control.human_torque_nm, 55.),
                "resolve keeps the held human_torque_nm");
}

void test_resolve_partial_merge() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
    bt::require(plant.last_control.value().posture.has_value(), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "resolve fills the unset posture from the intent");
    bt::require(plant.last_control.value().human_torque_nm.has_value() && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    approx12((plant.last_control.value().human_torque_nm).value(), 25.), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "resolve keeps the held human_torque_nm");
}

void test_resolve_rider_disabled_keeps_control_but_ticks() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
    bt::require(!plant.last_control.value().posture.has_value() && // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    !plant.last_control.value().human_torque_nm.has_value(), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                "rider_enabled=false suppresses the merge");
    bt::require(require_intent(plant).state().last_tick_step == 0,
                "the acquisition tick still ran");
}

void test_resolve_inactive_returns_control() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
    bt::require(require_intent(plant).state().last_tick_step == -1,
                "an inactive call never ticks");
}

void test_probe_advance_preserves_state() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);                    // adapter
    event(plant, golden, seq);                    // resolve step 0
    const std::string before =
        state_text(require_intent(plant));
    event(plant, golden, seq);                    // before snapshot
    event(plant, golden, seq);                    // resolve step 10, probe
    bt::require(state_text(require_intent(plant)) == before,
                "a probing resolve preserves the resolver state");
    event(plant, golden, seq);                    // state after the probe
    event(plant, golden, seq);                    // resolve step 10, real
    golden.done();
}

void test_resolve_step_validation() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 5; ++i)
        event(plant, golden, seq);  // the five malformed steps
    golden.done();
}

void test_resolve_skipped_acquisition() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
}

void test_resolve_rewind_requires_reset() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    event(plant, golden, seq);  // step 0
    event(plant, golden, seq);  // step 10
    event(plant, golden, seq);  // step 5 — rewound clock raises
    require_intent(plant).reset();  // unrecorded intent_reset()
    event(plant, golden, seq);      // step 0 again
    golden.done();
}

void test_period_must_be_integer_multiple() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // enabled resolver rejects period_s
    event(plant, golden, seq);  // a disabled resolver tolerates it
    golden.done();
}

void test_config_validation() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    for (int i = 0; i < 9; ++i)
        event(plant, golden, seq);  // each malformed config raises
    golden.done();
}

void test_signals_validation() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 6; ++i)
        event(plant, golden, seq);  // each malformed signals doc raises
    golden.done();
}

void test_control_validation() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 5; ++i)
        event(plant, golden, seq);  // each malformed control doc raises
    golden.done();
}

void test_road_grade_validation_is_lazy() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    event(plant, golden, seq);  // step 0 tick
    event(plant, golden, seq);  // step 1 — nan window passes non-tick
    event(plant, golden, seq);  // step 10 — nan grade raises on a tick
    golden.done();
}

void test_update_validates_before_enabled_check() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
}

void test_policy_update_parity_with_delays() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 79; ++i)
        event(plant, golden, seq);  // the recorded update timeline
    event(plant, golden, seq);      // _state_cmp
    golden.done();
}

void test_resolve_enabled_delayed_none_front_load() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    auto const &intent = require_intent(plant);
    bool saw_delayed_none = false;
    for (int step = 0; step < 450; ++step) {
        event(plant, golden, seq);  // _resolve
        event(plant, golden, seq);  // _state_cmp
        const auto &program = intent.state().program;
        if (!program.delayed_load_share.has_value() && step >= 220) {
            saw_delayed_none = true;
            // A delivered None skips the trim integrator entirely.
            bt::require(program.trim_rad == 0.,
                        "a delivered None share skips the trim integrator");
        }
    }
    bt::require(saw_delayed_none, "the delayed None window was exercised");
    // Once real shares deliver, the trim integrator runs again.
    bt::require(intent.state().program.trim_rad != 0.,
                "real shares re-arm the trim integrator");
    golden.done();
}

void test_policy_inclination_norm_gate() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 40; ++i)
        event(plant, golden, seq);  // specific-force outside the norm gate
    event(plant, golden, seq);      // _state_cmp
    golden.done();
    bt::require(require_intent(plant).policy().inclination_rad() != 0.,
                "the pitch-rate integration still runs past the norm gate");
}

void test_surge_budget_spend_and_recovery() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 8; ++i)
        event(plant, golden, seq);  // four spends + four recovery calls
    event(plant, golden, seq);      // _state_cmp
    golden.done();
}

void test_program_update_parity() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 5; ++i)
        event(plant, golden, seq);  // the five program.update cases
    event(plant, golden, seq);      // _state_cmp
    golden.done();
}

void test_program_validation() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    for (int i = 0; i < 7; ++i)
        event(plant, golden, seq);  // each malformed update raises
    golden.done();
}

void test_schedule_pulse_and_reset() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    event(plant, golden, seq);  // resolve step 0
    event(plant, golden, seq);  // schedule_pulse(.015, .02, .25)
    for (int i = 0; i < 4; ++i)
        event(plant, golden, seq);  // resolve steps 10..40
    event(plant, golden, seq);      // _state_cmp with the queued pulse
    require_intent(plant).reset();  // unrecorded intent_reset()
    event(plant, golden, seq);      // _state_cmp — pulse cleared
    event(plant, golden, seq);      // nan start raises
    event(plant, golden, seq);      // zero duration raises
    golden.done();
}

void test_state_roundtrip() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    // resolver.schedule_pulse(.03, .02, .2) runs on the oracle before the
    // loop; the recorded output.state leaves carry its pulses member, so
    // the native resolver schedules the same pulse here — the recorded
    // control leaves are unchanged either way (body_pulse(t==start)==0).
    require_intent(plant).schedule_pulse(.03, .02, .2);
    for (int i = 0; i < 4; ++i)
        event(plant, golden, seq);  // resolve steps 0..30
    event(plant, golden, seq);      // restore the oracle snapshot
    event(plant, golden, seq);      // _state_cmp
    const std::string again = state_text(require_intent(plant));
    event(plant, golden, seq);      // restore the native snapshot back
    event(plant, golden, seq);      // _state_cmp
    bt::require(state_text(require_intent(plant)) == again,
                "native state -> restore is a fixed point");
    event(plant, golden, seq);      // resolve step 40 keeps the clock
    golden.done();
}

void test_state_rejects_malformed() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    event(plant, golden, seq);  // adapter
    event(plant, golden, seq);  // the reference intent_state() leaf
    for (int i = 0; i < 7; ++i)
        event(plant, golden, seq);  // each corrupted state raises
    golden.done();
}

void test_state_unknown_signals_keys_rejected() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Plant plant;
    Seq seq;
    drain(plant, golden, seq);
    golden.done();
}

} // namespace

int main() {
    return bt::run([] {
        test_resolve_matches_oracle_over_mixed_steps();
        test_resolve_disabled_passthrough();
        test_resolve_merges_only_unset_fields();
        test_resolve_partial_merge();
        test_resolve_rider_disabled_keeps_control_but_ticks();
        test_resolve_inactive_returns_control();
        test_probe_advance_preserves_state();
        test_resolve_step_validation();
        test_resolve_skipped_acquisition();
        test_resolve_rewind_requires_reset();
        test_period_must_be_integer_multiple();
        test_config_validation();
        test_signals_validation();
        test_control_validation();
        test_road_grade_validation_is_lazy();
        test_update_validates_before_enabled_check();
        test_policy_update_parity_with_delays();
        test_resolve_enabled_delayed_none_front_load();
        test_policy_inclination_norm_gate();
        test_surge_budget_spend_and_recovery();
        test_program_update_parity();
        test_program_validation();
        test_schedule_pulse_and_reset();
        test_state_roundtrip();
        test_state_rejects_malformed();
        test_state_unknown_signals_keys_rejected();
    });
}
