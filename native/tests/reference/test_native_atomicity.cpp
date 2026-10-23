// Completion 3.28. Source: tests/reference/test_native_atomicity.py.
// Every mutating drivetrain path — shift, prepare, reset, restore, compute,
// probe and settle — stages the candidate and only commits fully validated
// state. These ports pin the observable FFI contract: the drive snapshot,
// engine qpos/qvel, ctrl and the prepared-storage generations must be
// bit-identical after any rejection, a repeated attempt must reject
// identically, and a successful commit lands atomically without
// reallocating the prepared buffers.
//
// All comparisons are self-contained (native frozen state vs native frozen
// state, literals or analytic values) — no Python oracle is consulted, so
// this file holds no golden nodes. The five `test_python_*` source records
// are oracle-mirror twins of the ported contracts and are dispositioned
// `drop` in the port map.
#include <array>
#include <algorithm>
#include <ranges>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "diag.hpp"
#include "drivetrain_reference_support.hpp"
#include "io/json.hpp"
#include "rider/state_codec.hpp"

namespace {
using namespace drivetrain_reference;
using bike::io::canonical_json;
using policy_reference::field;
using policy_reference::member;
using policy_reference::put;
using policy_reference::set;

constexpr std::string_view kGeometric = "geometric_ideal_mid_drive";

cfg::ShiftingConfig invalid_cassette() {
    // A parsed cassette whose next sprocket makes tangent geometry invalid.
    cfg::ShiftingConfig result;
    result.enabled = true; result.cassette = {24, 1000};
    result.cadence_smoothing_tau_s = 0.;
    result.target_cadence_min_rpm = 100.; result.target_cadence_max_rpm = 1000.;
    return result;
}
cfg::ShiftingConfig valid_cassette() {
    cfg::ShiftingConfig result;
    result.enabled = true; result.cassette = {24, 28};
    result.cadence_smoothing_tau_s = 0.;
    result.target_cadence_min_rpm = 100.; result.target_cadence_max_rpm = 1000.;
    return result;
}
std::unique_ptr<Stepper> shifted_pair(std::string_view kind, const cfg::ShiftingConfig& shifting,
                                      std::string_view topology = "plain") {
    auto input = source(kind, topology); input.shifting = shifting;
    return pair(input, "crank_effort", topology);
}

// _spin: write the two wheel rates into the owner's data and republish the
// whole engine state through set_state + forward, like the source.
void spin(Stepper& owner, double crank = 4., double wheel = 1.) {
    velocity(owner, "crank_spin", crank); velocity(owner, "rear_wheel_spin", wheel);
    reset_engine_state(owner);
}

// Prepared-storage telemetry mirroring native._drive_prepared_storage():
// None where no transmission exists (elastic chain) — the {0,0,0,0}
// sentinel keeps both sides of a frozen comparison equally absent.
std::array<std::uint64_t, 4> storage_or_absent(const Stepper& owner) {
    const auto* transmission = owner.drive().transmission_storage();
    const auto* prepared = transmission != nullptr ? transmission->prepared_storage() : nullptr;
    if (prepared == nullptr) return {0, 0, 0, 0};
    const auto generations = transmission->prepared_generations();
    return {generations.first, generations.second,
            static_cast<std::uint64_t>(prepared->jacobian.capacity()),
            static_cast<std::uint64_t>(prepared->qpos.capacity())};
}

// _frozen: everything a rejected transaction must preserve, bit-exact —
// drive_state, engine qpos/qvel, the actuator surface and the storage
// generations. The snapshot travels through canonical_json because the
// wire encoding is already the freeze() canonical form.
struct Frozen {
    std::string state;
    std::vector<double> qpos, qvel, ctrl;
    std::array<std::uint64_t, 4> storage;
};
Frozen frozen(const Stepper& owner) {
    return {canonical_json(state(owner)),
            {owner.qpos().begin(), owner.qpos().end()},
            {owner.qvel().begin(), owner.qvel().end()},
            {owner.ctrl().begin(), owner.ctrl().end()}, storage_or_absent(owner)};
}
bool same_bits(std::span<const double> a, std::span<const double> b) {
    // Byte-exact: NaN payloads and -0.0 stay distinct, unlike operator==.
    return a.size() == b.size() &&
           std::ranges::equal(std::as_bytes(a), std::as_bytes(b));
}
bool operator==(const Frozen& a, const Frozen& b) {
    return a.state == b.state && same_bits(a.qpos, b.qpos) &&
           same_bits(a.qvel, b.qvel) && same_bits(a.ctrl, b.ctrl) &&
           a.storage == b.storage;
}

// Runs action(), requires rejection of type Exception (message substring
// optional) and returns the exception text so identical-retry checks can
// compare messages.
template <class Exception, class F>
std::string rejected(F&& action, std::string_view message = {}) {
    std::string out;
    try {
        std::forward<F>(action)();
    } catch (const Exception& failure) {
        out = failure.what();
        bt::require(message.empty() || out.find(message) != std::string::npos,
                    "native exception retains the source diagnostic");
    }
    bt::require(!out.empty(), "native operation rejects the original invalid request");
    return out;
}

// Source: test_native_atomicity.py::test_failed_geometric_shift_is_atomic
void test_failed_geometric_shift_is_atomic() {
    auto owner = shifted_pair(kGeometric, invalid_cassette());
    spin(*owner);
    const auto before = frozen(*owner);
    const auto first = rejected<std::invalid_argument>(
        [&] { (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {}); });
    bt::require(frozen(*owner) == before, "rejected shift publishes nothing");
    // A repeated attempt meets the same candidate, the same rejection, and
    // still publishes nothing — the shifter roll-back keeps the retry legal.
    const auto second = rejected<std::invalid_argument>(
        [&] { (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {}); });
    bt::require(frozen(*owner) == before, "repeated rejection still publishes nothing");
    bt::require(first == second, "repeated rejection emits the identical diagnostic");
}

// Source: test_native_atomicity.py::test_valid_shift_commits_atomically
void test_valid_shift_commits_atomically() {
    for (const std::string_view kind : std::array<std::string_view, 2>{"ideal_mid_drive", kGeometric}) {
        auto owner = shifted_pair(kind, valid_cassette());
        spin(*owner);
        const auto stored = storage_or_absent(*owner);
        const auto result = owner->drive().prepare({}, .1, false, true, true, true, {}, {});
        const auto after = state(*owner);
        // Cadence ~38 rpm is below the 100 rpm floor: downshift 24 -> 28.
        bt::require(bike::io::as_double(field(field(after, "ideal_hub"), "ratio"), "ratio") == 34. / 28.,
                    "committed shift lands the 34/28 ratio atomically");
        // Geometric hubs re-derive their sprocket; ideal hubs keep the
        // configured rear_teeth — only the wrap coefficient carries it.
        const bool geometric = kind == kGeometric;
        bt::require(bike::io::as_int64(field(field(after, "ideal_hub"), "rear_teeth"), "rear_teeth") ==
                        (geometric ? 28 : 24),
                    "committed shift carries the topology's rear_teeth mirror");
        bt::require(!bike::io::is_null(field(field(after, "ideal_hub"), "shift_pending")) &&
                        std::get<bool>(field(field(after, "ideal_hub"), "shift_pending").data) == geometric,
                    "committed shift publishes the topology's pending flag");
        const auto& policies_shifting = field(field(after, "policies"), "shifting");
        bt::require(bike::io::as_int64(field(policies_shifting, "rear_teeth"), "rear_teeth") == 28,
                    "policy snapshot shows the shifted sprocket");
        bt::require(bike::io::as_int64(field(policies_shifting, "shift_count"), "shift_count") == 1,
                    "policy snapshot counts exactly one shift");
        bt::require(bike::io::as_string(field(policies_shifting, "direction"), "direction") == "down",
                    "policy snapshot names the down shift");
        bt::require(bike::io::as_double(field(after, "shift_time_s"), "shift_time_s") == owner->data()->time,
                    "shift timestamp lands the engine clock");
        if (geometric) {
            // Successful commits copy into persistent prepared storage:
            // vector identity (generation) and capacity are stable.
            bt::require(storage_or_absent(*owner) == stored,
                        "commit preserves prepared-buffer generations and capacity");
            bt::require(!bike::io::is_null(field(field(after, "ideal_hub"), "prepared")),
                        "geometric commit republishes the prepared block");
        }
        // The returned pedaling state is a plain snapshot of the transaction.
        bt::require(std::isfinite(result.effort_nm), "pedaling result carries a finite effort");
    }
}

// Source: test_native_atomicity.py::test_rejected_then_reset_prepare_and_restore
void test_rejected_then_reset_prepare_and_restore() {
    for (const std::string_view kind : std::array<std::string_view, 2>{"ideal_mid_drive", kGeometric}) {
        auto owner = shifted_pair(kind, invalid_cassette());
        spin(*owner);
        if (kind == kGeometric)
            (void)rejected<std::invalid_argument>(
                [&] { (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {}); });
        // Reset after the rejection re-stages from live state: the hub
        // returns to the configured gear and the snapshot round-trips.
        owner->drive().reset();
        const auto reset_state = state(*owner);
        bt::require(bike::io::as_double(field(field(reset_state, "ideal_hub"), "ratio"), "ratio") == 34. / 24.,
                    "reset returns the hub to the configured gear");
        bt::require(bike::io::as_int64(
                        field(field(field(reset_state, "policies"), "shifting"), "shift_count"), "shift_count") == 0,
                    "reset clears the shift counter");
        restore(*owner, reset_state);
        bt::require(canonical_json(state(*owner)) == canonical_json(reset_state),
                    "reset snapshot round-trips through restore");
        // A non-shifting prepare commits and only touches prepared/boundary.
        velocity(*owner, "crank_spin", 12.);  // ~115 rpm: no shift
        owner->data()->time += .1;
        reset_engine_state(*owner);
        (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {});
        bt::require(bike::io::as_double(field(field(state(*owner), "ideal_hub"), "ratio"), "ratio") == 34. / 24.,
                    "non-shifting prepare keeps the configured gear");
    }
}

// Source: test_native_atomicity.py::test_invalid_restore_is_atomic
void test_invalid_restore_is_atomic() {
    auto owner = pair(kGeometric);
    spin(*owner);
    (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {});
    const auto before = frozen(*owner);
    auto inconsistent = state(*owner);
    set(member(inconsistent, "ideal_hub"), "ratio", real(9.9));  // inconsistent with the gearing row
    (void)rejected<std::invalid_argument>([&] { restore(*owner, inconsistent); });
    bt::require(frozen(*owner) == before, "inconsistent restore publishes nothing");
    auto malformed = state(*owner);
    set(member(malformed, "ideal_hub"), "boundary", Value{bike::io::Number{"nan"}});
    (void)rejected<std::invalid_argument>([&] { restore(*owner, malformed); });
    bt::require(frozen(*owner) == before, "non-finite restore publishes nothing");
}

// Source: test_native_atomicity.py::test_rejected_timestamp_advance_is_atomic
void test_rejected_timestamp_advance_is_atomic() {
    auto owner = pair(kGeometric);
    spin(*owner);
    (void)components(*owner, .002, 2.);  // publishes last_time_s
    const auto before = frozen(*owner);
    (void)rejected<std::invalid_argument>([&] { (void)components(*owner, .002, 2.); },
                                          "once per timestamp");
    bt::require(frozen(*owner) == before, "duplicate timestamp publishes nothing");
}

// Source: test_native_atomicity.py::test_rejected_shift_preserves_auxiliary_transmissions
void test_rejected_shift_preserves_auxiliary_transmissions() {
    for (auto topology : {"clutch", "rotor"}) {
        auto owner = shifted_pair(kGeometric, invalid_cassette(), topology);
        spin(*owner);
        const auto before = frozen(*owner);
        (void)rejected<std::invalid_argument>(
            [&] { (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {}); });
        bt::require(frozen(*owner) == before, "rejected shift preserves auxiliary transmissions");
        const auto auxiliary = std::string_view(topology) == "clutch" ? "clutch" : "freewheel";
        bt::require(bike::io::as_double(field(field(state(*owner), auxiliary), "ratio"), "ratio") == 1.,
                    "auxiliary transmission stays at its unit ratio");
    }
}

// Source: test_native_atomicity.py::test_probe_controls_do_not_publish
void test_probe_controls_do_not_publish() {
    auto owner = shifted_pair(kGeometric, invalid_cassette());
    spin(*owner);
    (void)rejected<std::invalid_argument>(
        [&] { (void)owner->drive().prepare({}, .1, false, true, true, true, {}, {}); });
    const auto before = state(*owner);
    // Probe evaluation (advance=False) must not touch ratchets or live
    // policy state even right after a rejected transaction.
    (void)components(*owner, .002, 2., {}, true, false, false);
    auto after = state(*owner);
    set(after, "probe_last", field(before, "probe_last"));  // diagnostics channel only
    bt::require(canonical_json(after) == canonical_json(before),
                "probe after rejection publishes only its declared channel");
}

// Source: test_native_atomicity.py::test_rejected_prepare_keeps_policy_state
void test_rejected_prepare_keeps_policy_state() {
    // A ceiling rejection discards the candidate: pedaling, shifter EMAs,
    // shift clock and staged model updates all stay at their pre-call
    // values — and a valid retry equals one fresh call.
    for (const std::string_view kind : std::array<std::string_view, 2>{"ideal_mid_drive", kGeometric}) {
        auto owner = pair(kind);
        spin(*owner);
        const auto before = frozen(*owner);
        for (int i = 0; i < 3; ++i) {
            (void)rejected<std::invalid_argument>(
                [&] { (void)owner->drive().prepare({}, .01, false, true, true, true, {}, -1.); });
            bt::require(frozen(*owner) == before, "ceiling rejection publishes nothing");
        }
        // A valid call after the rejections must land exactly the state a
        // single call on an untouched fixture lands — never a retry sum.
        auto fresh = pair(kind);
        spin(*fresh);
        const auto result = owner->drive().prepare({}, .01, false, true, true, true, {}, {});
        const auto expected = fresh->drive().prepare({}, .01, false, true, true, true, {}, {});
        bt::require(canonical_json(policy_reference::pedaling_result(result)) ==
                        canonical_json(policy_reference::pedaling_result(expected)),
                    "valid retry returns the single-call pedaling snapshot");
        bt::require(canonical_json(state(*owner)) == canonical_json(state(*fresh)),
                    "valid retry lands the single-call drive state");
    }
}

// Source: test_native_atomicity.py::test_rejected_compute_keeps_everything
void test_rejected_compute_keeps_everything() {
    // Wire-level and staging-level rejections publish nothing: not the
    // policy candidates, not ctrl, not the diagnostics or timestamp state.
    // The ('control', {'rider_enabled': 1}) member is an original-type check
    // — the owned public-control decoder carries the same contract.
    for (const auto& item : std::initializer_list<std::pair<std::string_view, double>>{
             {"motor_limit_nm", -1.}, {"human_torque_nm", std::numeric_limits<double>::quiet_NaN()},
             {"rider_enabled", 1.}, {"sensed_nan", std::numeric_limits<double>::quiet_NaN()},
             {"sensed_inf", std::numeric_limits<double>::infinity()}}) {
        auto owner = pair(kGeometric);
        spin(*owner);
        (void)components(*owner, .002, 2.);
        spin(*owner, 4.5);
        const auto before = frozen(*owner);
        const auto& [field_name, value] = item;
        if (field_name == "rider_enabled") {
            auto control = rider::encode_intent_control(rider::IntentControl{});
            set(control, "rider_enabled", integer(1));
            (void)rejected<std::invalid_argument>(
                [&] { (void)rider::decode_intent_control(control, "control"); });
        } else {
            drivetrain::TickInputs input;
            input.dt = .002; input.speed = 2.;
            if (field_name == "motor_limit_nm") input.control.motor_limit_nm = -1.;
            else if (field_name == "human_torque_nm")
                input.control.human_torque_nm = std::numeric_limits<double>::quiet_NaN();
            else input.sensed = value;
            (void)rejected<std::invalid_argument>([&] { (void)owner->drive().stage_components(input); });
        }
        bt::require(frozen(*owner) == before, "rejected compute publishes nothing");
    }
}

// Source: test_native_atomicity.py::test_unsettled_pending_rejects_before_policy_advance
void test_unsettled_pending_rejects_before_policy_advance() {
    // An unsettled actuation rejects before pedaling/assist candidates are
    // built — retries can never accumulate effort or EMA state.
    auto owner = pair(kGeometric);
    spin(*owner);
    (void)components(*owner, .002, 2.);  // reserves the pending actuation
    owner->data()->time += .002;
    spin(*owner);
    const auto before = frozen(*owner);
    for (int i = 0; i < 2; ++i) {
        (void)rejected<std::runtime_error>([&] { (void)components(*owner, .002, 2.); }, "not settled");
        bt::require(frozen(*owner) == before, "unsettled rejection publishes nothing");
    }
    // Clearing pending through a snapshot round-trip restores forward motion.
    auto cleared = state(*owner);
    set(cleared, "pending_actuation", Value{});
    restore(*owner, cleared);
    (void)components(*owner, .002, 2.);
}

// Source: test_native_atomicity.py::test_prepare_duplicate_timestamp_is_atomic
void test_prepare_duplicate_timestamp_is_atomic() {
    // The interval clock is shared: a successful compute makes a later
    // same-timestamp prepare a rejection, and the rejection writes nothing.
    auto owner = pair(kGeometric);
    spin(*owner);
    (void)components(*owner, .002, 2.);
    const auto before = frozen(*owner);
    (void)rejected<std::invalid_argument>(
        [&] { (void)owner->drive().prepare({}, .002, false, true, true, true, {}, {}); },
        "once per timestamp");
    bt::require(frozen(*owner) == before, "duplicate prepare timestamp publishes nothing");
}

// Source: test_native_atomicity.py::test_probe_atomicity
void test_probe_atomicity() {
    // A probe publishes only its declared telemetry — the force rows and
    // the probe diagnostics channel. Policy, clock, pending and model state
    // are bitwise untouched even while pending actuation is reserved, and a
    // rejecting probe does not even touch probe_last.
    for (const std::string_view kind : std::array<std::string_view, 2>{"ideal_mid_drive", kGeometric}) {
        auto owner = pair(kind);
        spin(*owner);
        (void)components(*owner, .002, 2.);  // set pending + interval clock
        spin(*owner);
        const auto before = state(*owner);
        const auto rows = components(*owner, .002, 2., {}, true, false, false);
        bt::require(!rows.empty(), "probe still returns the candidate force rows");
        const auto after = state(*owner);
        bt::require(!bike::io::is_null(field(after, "probe_last")), "probe publishes probe_last");
        auto probe = after;
        set(probe, "probe_last", field(before, "probe_last"));
        bt::require(canonical_json(probe) == canonical_json(before),
                    "probe leaves policy, clock, pending and model state untouched");
        // A rejected probe leaves the whole snapshot, probe channel included.
        (void)rejected<std::invalid_argument>(
            [&] {
                (void)components(*owner, .002, 2., {}, true, false, false,
                                 std::numeric_limits<double>::quiet_NaN());
            });
        bt::require(canonical_json(state(*owner)) == canonical_json(after),
                    "rejected probe publishes nothing at all");
    }
}

// Source: test_native_atomicity.py::test_prepare_then_components_matches_internal_prepare
void test_prepare_then_components_matches_internal_prepare() {
    // prepare→compute(supplied state) equals compute's internal prepare:
    // the staged pedaling advance is identical whether it commits early or
    // is supplied back to the same tick.
    for (const std::string_view kind : std::array<std::string_view, 2>{"ideal_mid_drive", kGeometric}) {
        auto supplied = pair(kind);
        auto internal = pair(kind);
        spin(*supplied);
        spin(*internal);
        const auto pedaling = supplied->drive().prepare({}, .002, false, true, true, true, {}, {});
        (void)components(*supplied, .002, 2., {}, true, false, true, 0., pedaling);
        (void)components(*internal, .002, 2.);
        bt::require(canonical_json(state(*supplied)) == canonical_json(state(*internal)),
                    "supplied pedaling equals the internal prepare");
    }
}

// _solved_fixture: arm a pending actuation whose reserved effort matches
// the solve — components() reserves the delivered torque in the pending
// row; a restored pending row plus a matching ctrl row reproduces that
// arming deterministically.
Value solved_fixture(Stepper& owner, std::vector<double>& control) {
    (void)components(owner, .002, 2., {}, true, false, true, 20.);
    auto armed = state(owner);
    bt::require(!bike::io::is_null(field(armed, "pending_actuation")), "components reserves the pending row");
    set(armed, "pending_actuation",
        Value{Value::Object{{"requested", real(1.)}, {"omega", real(4.)},
                            {"dt", real(.002)}, {"enabled", Value{true}}}});
    restore(owner, armed);
    control.assign(owner.ctrl().begin(), owner.ctrl().end());
    control[static_cast<std::size_t>(
        bike::model::required_id(*owner.model(), mjOBJ_ACTUATOR, "mid_drive"))] = 1.;
    owner.set_inputs(control, std::vector<double>(owner.qvel().size()));
    owner.step();
    armed = state(owner);
    bt::require(bike::io::as_double(field(field(armed, "pending_actuation"), "requested"),
                                    "requested") == 1.,
                "armed pending survives the solved step");
    return armed;
}

// Source: test_native_atomicity.py::test_rejected_settle_keeps_everything
void test_rejected_settle_keeps_everything() {
    // A settle rejection publishes nothing — not even transmission solve
    // telemetry: pending validation runs ahead of every solve, so a refused
    // settle leaves battery, pending, diagnostics and hub state frozen.
    for (auto kind : kinds) {
        auto owner = pair(kind);
        std::vector<double> control;
        const auto armed = solved_fixture(*owner, control);
        // A pending reservation below the solved effort violates the
        // reserved ceiling — the rejection must land before publication.
        auto corrupted = armed;
        set(corrupted, "pending_actuation",
            Value{Value::Object{{"requested", real(.001)}, {"omega", real(4.)},
                                {"dt", real(.002)}, {"enabled", Value{true}}}});
        restore(*owner, corrupted);
        const auto before = frozen(*owner);
        for (int i = 0; i < 2; ++i) {
            (void)rejected<drivetrain::ArithmeticError>([&] { (void)owner->drive().settle(); },
                                                      "reserved effort ceiling");
            bt::require(frozen(*owner) == before, "refused settle publishes nothing");
        }
        // Retry from the honest armed state: bitwise-equal to a clean
        // drivetrain settling from the same snapshot — the control steps
        // from the same pre-step engine state so the recorded constraint
        // forces match.
        restore(*owner, armed);
        const auto solved = owner->drive().settle();
        const std::vector<double> out(solved.begin(), solved.end());
        auto fresh = pair(kind);
        restore(*fresh, armed);
        fresh->set_inputs(control, std::vector<double>(fresh->qvel().size()));
        fresh->step();
        const auto expected = fresh->drive().settle();
        bt::require(same_bits(out, expected), "honest settle is bitwise the clean drivetrain's");
        bt::require(canonical_json(state(*owner)) == canonical_json(state(*fresh)),
                    "honest settle lands the clean drivetrain's state");
    }
}

// Source: test_native_atomicity.py::test_battery_budget_rejection_is_atomic
void test_battery_budget_rejection_is_atomic() {
    // The store check rejects before any publication too — and the retry
    // with an honest reservation debits exactly once.
    auto owner = pair();
    std::vector<double> control;
    const auto armed = solved_fixture(*owner, control);
    auto corrupted = armed;
    set(corrupted, "pending_actuation",
        Value{Value::Object{{"requested", real(1000.)}, {"omega", real(4.)},
                            {"dt", real(.002)}, {"enabled", Value{true}}}});
    set(member(member(corrupted, "policies"), "battery"), "energy_j", real(.001));
    restore(*owner, corrupted);
    const auto before = frozen(*owner);
    (void)rejected<drivetrain::ArithmeticError>([&] { (void)owner->drive().settle(); },
                                              "battery storage");
    bt::require(frozen(*owner) == before, "battery rejection publishes nothing");
    restore(*owner, armed);
    (void)owner->drive().settle();
    const auto settled = state(*owner);
    const double debit =
        bike::io::as_double(field(field(field(settled, "policies"), "battery"), "drawn_energy_j"),
                            "drawn_energy_j") -
        bike::io::as_double(field(field(field(armed, "policies"), "battery"), "drawn_energy_j"),
                            "drawn_energy_j");
    bt::require(debit > 0. && bike::io::is_null(field(settled, "pending_actuation")),
                "honest retry debits exactly once and consumes pending");
}

// Source: test_native_atomicity.py::test_sparse_last_settle_debits_once
void test_sparse_last_settle_debits_once() {
    // The A12 shape at the FFI: a restored sparse last={} cannot change
    // exactly-once settlement — the detached map republishes the telemetry,
    // the battery moves exactly once, and a second settle debits nothing.
    const std::array<std::pair<std::string_view, std::string_view>, 7> combos{{
        {"elastic_chain", "plain"},
        {"ideal_mid_drive", "plain"}, {"ideal_mid_drive", "clutch"}, {"ideal_mid_drive", "rotor"},
        {kGeometric, "plain"}, {kGeometric, "clutch"}, {kGeometric, "rotor"}}};
    for (const auto& [kind, topology] : combos) {
        auto owner = pair(kind, "crank_effort", topology);
        std::vector<double> control;
        const auto armed = solved_fixture(*owner, control);
        auto sparse = armed;
        set(sparse, "last", Value{Value::Object{}});
        restore(*owner, sparse);
        bt::require(policy_reference::sorted_keys(field(state(*owner), "last")).empty(),
                    "restored sparse diagnostics stay sparse");
        (void)owner->drive().settle();
        const auto settled = state(*owner);
        const double debit =
            bike::io::as_double(field(field(field(settled, "policies"), "battery"), "drawn_energy_j"),
                                "drawn_energy_j") -
            bike::io::as_double(field(field(field(armed, "policies"), "battery"), "drawn_energy_j"),
                                "drawn_energy_j");
        bt::require(debit > 0., "sparse settle debits the battery once");
        bt::require(bike::io::is_null(field(settled, "pending_actuation")), "sparse settle consumes pending");
        bt::require(!policy_reference::sorted_keys(field(settled, "last")).empty(),
                    "sparse settle republishes telemetry through the detached map");
        // Control: the same armed snapshot with full diagnostics settles
        // into the same battery/pending/hub outcome.
        auto control_owner = pair(kind, "crank_effort", topology);
        restore(*control_owner, armed);
        control_owner->set_inputs(control, std::vector<double>(control_owner->qvel().size()));
        control_owner->step();
        (void)control_owner->drive().settle();
        const auto control_state = state(*control_owner);
        bt::require(canonical_json(field(settled, "policies")) ==
                        canonical_json(field(control_state, "policies")),
                    "sparse settle lands identical policy state");
        for (auto name : {"ideal_hub", "clutch", "freewheel"})
            bt::require(canonical_json(field(settled, name)) ==
                            canonical_json(field(control_state, name)),
                        "sparse settle lands identical transmission state");
        // Exactly once: settling again moves the battery by nothing at all.
        (void)owner->drive().settle();
        const auto again = state(*owner);
        bt::require(canonical_json(field(field(again, "policies"), "battery")) ==
                        canonical_json(field(field(settled, "policies"), "battery")),
                    "second settle cannot debit the battery twice");
        bt::require(bike::io::is_null(field(again, "pending_actuation")), "second settle stays consumed");
    }
}
} // namespace

// The 16 test bodies inline into this one lambda; the aggregate frame
// exceeds the 8 KiB policy under clang even though each callee stays
// within budget, so the suppression wraps the dispatcher only.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_failed_geometric_shift_is_atomic();
        test_valid_shift_commits_atomically();
        test_rejected_then_reset_prepare_and_restore();
        test_invalid_restore_is_atomic();
        test_rejected_timestamp_advance_is_atomic();
        test_rejected_shift_preserves_auxiliary_transmissions();
        test_probe_controls_do_not_publish();
        test_rejected_prepare_keeps_policy_state();
        test_rejected_compute_keeps_everything();
        test_unsettled_pending_rejects_before_policy_advance();
        test_prepare_duplicate_timestamp_is_atomic();
        test_probe_atomicity();
        test_prepare_then_components_matches_internal_prepare();
        test_rejected_settle_keeps_everything();
        test_battery_budget_rejection_is_atomic();
        test_sparse_last_settle_debits_once();
    });
}
NATIVE_DIAG_POP
