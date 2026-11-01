// test_monitor_strictness.cpp — native port of
// tests/reference/test_monitor_strictness.py (task C323, batch 3.23).
//
// Run-mode checks against neutral raw intervals, plus the runtime/research
// ownership and failure-evidence contracts the oracle exercised through
// Python-only seams:
//   * PeriodAccounting plays the oracle's runtime_for_raws() stand-in —
//     neutral RawStep values, decimation 1/400, strict on/off;
//   * the delayed_bootstrap research fixture supplies the real
//     begin/advance/failure/reset orchestration, with test_fail_at_step()
//     as the engine-failure probe and the weld-attachment budget as the
//     real strict-rejection trigger;
//   * the Python-only injection seams (monkeypatched recorder, Prepared
//     exception inside reset, headless CLI plumbing) are ported to the
//     closest reachable native contract and documented per record.
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "diag.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "support/migration_test.hpp"
#include "strictness_reference_support.hpp"
#include "runtime/accounting.hpp"
#include "runtime/bootstrap.hpp"
#include "runtime/research.hpp"
#include "runtime/research_io.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/status.hpp"
#include "runtime/step.hpp"

namespace {

namespace bio = bike::io;
namespace wire = runtime::sample_wire;

// neutral_raw/runtime_for_raws/kDt live in strictness_reference_support.hpp —
// the shared header mirrors the oracle's
// `from test_monitor_strictness import runtime_for_raws, neutral_raw`
// in test_energy_ledger.py.
using strictness_support::kDt;
using strictness_support::neutral_raw;
using strictness_support::runtime_for_raws;

[[nodiscard]] std::filesystem::path dir() {
    return bt::fixture("research/delayed_bootstrap");
}

// ---- research fixture seams -------------------------------------------------

void set_doc_bool(bio::Value::Object &object, const char *key, bool value) {
    for (auto &[name, member] : object)
        if (name == key) {
            member.data = value;
            return;
        }
    object.emplace_back(key, bio::Value{.data = value});
}
void set_doc_real(bio::Value::Object &object, const char *key,
                  std::string_view lexeme) {
    const bio::Value replacement{.data = bio::Number{std::string(lexeme)}};
    for (auto &[name, member] : object)
        if (name == key) {
            member = replacement;
            return;
        }
    object.emplace_back(key, replacement);
}
[[nodiscard]] bio::Value::Object &doc_object(bio::Value &value, const char *path) {
    if (!std::holds_alternative<bio::Value::Object>(value.data))
        throw std::invalid_argument(std::string("expected object at ") + path);
    return std::get<bio::Value::Object>(value.data);
}
[[nodiscard]] bio::Value &doc_member(bio::Value &value, const char *key) {
    auto &object = doc_object(value, key);
    for (auto &[name, member] : object)
        if (name == key) return member;
    throw std::invalid_argument(std::string("missing document member ") + key);
}

// The fixture ships strict=false; a strict research episode rewrites the
// config envelope AND the persisted monitor mode (construction rejects a
// mismatched pair), then prices the weld load bound out of reach so the
// first observed foot attachment produces a real *.normal violation.
[[nodiscard]] runtime::ResearchBootstrap strict_bootstrap() {
    auto boot = runtime::read_research_bootstrap(dir());
    boot.ride.config.strict = true;
    boot.config.stop_on_model_violation = true;
    // The oracle's replace(env.config, record_decimation=1) flows through
    // reset() into both envelopes — the research decimation and the
    // physical measurement cadence must agree (research.cpp:96-98).
    boot.config.record_decimation = 1;
    boot.ride.config.measurement_decimation = 1;
    // The persisted state.runtime inventory is validated against the config
    // envelope at construction — the recorded decimation must move in
    // lockstep or the bootstrap decode rejects the pair.
    set_doc_real(doc_object(boot.ride.state.runtime, "state.runtime"),
                 "measurement_decimation", "1");
    set_doc_bool(doc_object(doc_member(boot.ride.state.runtime, "monitor"),
                            "monitor"),
                 "strict", true);
    set_doc_real(doc_object(boot.ride.config.rider_controller,
                            "rider_controller"),
                 "weld_foot_min_normal_n", "1e9");
    return boot;
}

[[nodiscard]] std::unique_ptr<runtime::NativeResearchRuntime>
make_env(runtime::ResearchBootstrap boot) {
    return runtime::make_research(std::move(boot));
}

[[nodiscard]] const runtime::WireObject &section(const runtime::WireObject &o,
                                                 std::string_view key) {
    return wire::object(wire::required(o, key));
}

// ---- record 1: defaults + strict must be a real bool -------------------------

void defaults_to_strict_and_validates_mode() {
    // Python: inspect.signature(PhysicalRuntime)['strict'].default is True.
    // The native config carries strict through the document envelope —
    // "default" maps to the headless check-policy default (cli
    // configuration.cpp defaults --check-policy to "strict"); the runtime
    // decode itself bool-gates the field. Assert both halves that are
    // spellable here: a real bool decodes and every non-bool is rejected.
    const auto config_path = dir() / "config.json";
    const auto envelope = bio::parse_json(bt::text(config_path));
    const auto decoded = runtime::decode_runtime_config(envelope);
    bt::require(!decoded.strict, "fixture envelope decodes its own strict flag");
    for (const bio::Value &bad :
         {bio::Value{.data = bio::Number{"0"}},
          bio::Value{.data = bio::Number{"1"}}, bio::Value{},
          bio::Value{.data = std::string{"strict"}}}) {
        auto edited = envelope;
        for (auto &[name, member] : doc_object(edited, "config"))
            if (name == "strict") member = bad;
        // non-bool strict is rejected at the config boundary.
        bt::invalid([&] { (void)runtime::decode_runtime_config(edited); });
    }
}

// ---- record 2: first failure time + sticky status ignore decimation ---------

void raw_failure_time_and_sticky_status_ignore_decimation(bool strict,
                                                          int decimation) {
    const auto runtime = runtime_for_raws(strict, decimation);
    for (std::int64_t i = 0; i < 10; ++i)
        runtime->push(neutral_raw(
            i, i == 3 ? std::vector<std::string>{"foot_front.normal"}
                      : std::vector<std::string>{}));
    if (strict) {
        bool rejected = false;
        std::string message;
        try {
            runtime->flush();
        } catch (const runtime::InvalidReferenceRun &failure) {
            rejected = true;
            message = failure.what();
        }
        bt::require(rejected, "strict flush raises InvalidReferenceRun");
        bt::require(message.find("foot_front.normal") != std::string::npos,
                    "strict rejection names the violating channel");
    } else {
        runtime->flush();
        // The RuntimeWarning surface: pending once, consumed once.
        const auto warning = runtime->take_warning();
        bt::require(warning.has_value() &&
                        warning->find("foot_front.normal") != std::string::npos,
                    "non-strict flush holds exactly one pending warning");
        bt::require(!runtime->take_warning().has_value(),
                    "the pending warning is consumed exactly once");
    }
    const auto &first = runtime->state().monitor.first_failure;
    bt::require(first.has_value() && first->time_s == .002 &&
                    first->violations == std::vector<std::string>{"foot_front.normal"},
                "first failure latches the violating interval's own end time");
    bt::require(runtime->pending_samples().size() == 10,
                "the committed period publishes every interval");
    bt::require(std::abs(runtime->state().history.duration_s - .005) < 1e-15,
                "work history integrates the committed prefix");
    const auto status_of = [](const runtime::SamplePtr &sample) {
        return section(sample->channels, "model_status");
    };
    bt::require(wire::boolean(wire::required(
                    status_of(runtime->pending_samples()[2]), "model_valid")),
                "the pre-violation sample stays model-valid");
    bt::require(!wire::boolean(wire::required(
                    status_of(runtime->pending_samples().back()), "model_valid")),
                "the closing sample is model-invalid");
    const auto &first_status = runtime->state().model_status.first;
    bt::require(first_status.has_value() &&
                    wire::integer(wire::required(*first_status, "interval_id")) == 3,
                "model status latches the first violating interval");
    bt::require(!runtime->has_raws(), "flush drains the raw period");
}

// ---- record 3: unmeasured tail forwards the ledger datum --------------------

void unmeasured_partial_period_keeps_previous_energy_datum() {
    // The oracle preseeded runtime.energy = {mechanical 42., elastic {x:1}};
    // the native equivalent is the committed EnergyState, so build the
    // accounting with the datum already committed.
    runtime::AccountingConfig config;
    config.timestep_s = kDt;
    config.period_steps = 10;
    config.measurement_decimation = 400;
    config.retention = runtime::RetentionMode::legacy;
    runtime::AccountingState initial;
    initial.energy.initial_energy_j = 0.;
    initial.energy.initial_battery_j = 0.;
    initial.energy.energy_scale_j = 1.;
    initial.energy.mechanical_energy_j = 42.;
    initial.energy.elastic_energy_j = {{"x", runtime::Wire(1.)}};
    runtime::PeriodAccounting rt(config, initial, 1, nullptr, nullptr);
    for (std::int64_t i = 0; i < 3; ++i) {
        auto raw = neutral_raw(i);
        raw.mechanical_energy_j = std::nullopt;
        raw.elastic_energy_j = std::nullopt;
        rt.push(std::move(raw));
    }
    rt.flush();
    bt::require(rt.pending_samples().size() == 3,
                "the unmeasured tail publishes every raw");
    for (const auto &sample : rt.pending_samples()) {
        const auto &energy = section(sample->channels, "energy");
        bt::require(wire::number(wire::required(energy, "mechanical_energy_j")) == 42.,
                    "unmeasured rows forward the committed mechanical datum");
        const auto &elastic = section(energy, "elastic_energy_j");
        bt::require(wire::number(wire::required(elastic, "x")) == 1.,
                    "unmeasured rows forward the committed elastic datum");
    }
}

// ---- record 4: closing measured datum on every raw --------------------------

void partial_period_reports_last_measured_energy_on_every_raw() {
    const auto runtime = runtime_for_raws(false, 400);
    for (std::int64_t i = 0; i < 3; ++i) {
        auto raw = neutral_raw(i);
        if (i == 1) {
            raw.mechanical_energy_j = 1.;
            raw.elastic_energy_j = runtime::WireObject{{"k", runtime::Wire(1.)}};
            // measured endpoint datum the tail forwards
            for (auto &[name, member] : raw.diagnostics)
                if (name == "endpoint_mass")
                    member = runtime::Wire(runtime::WireObject{
                        {"com_m", runtime::Wire(runtime::WireArray{
                                      runtime::Wire(1.), runtime::Wire(0.),
                                      runtime::Wire(0.)})}});
        } else {
            raw.mechanical_energy_j = std::nullopt;
            raw.elastic_energy_j = std::nullopt;
        }
        runtime->push(std::move(raw));
    }
    runtime->flush();
    bt::require(runtime->pending_samples().size() == 3,
                "the partial period publishes every raw");
    for (const auto &sample : runtime->pending_samples()) {
        const auto &energy = section(sample->channels, "energy");
        bt::require(wire::number(wire::required(energy, "mechanical_energy_j")) == 1.,
                    "every row reports the period's closing measurement");
        const auto &elastic = section(energy, "elastic_energy_j");
        bt::require(wire::number(wire::required(elastic, "k")) == 1.,
                    "every row reports the closing elastic datum");
    }
    const auto &endpoint = section(runtime->pending_samples().back()->channels,
                                   "endpoint_mass");
    const auto &com = wire::array(wire::required(endpoint, "com_m"));
    bt::require(com.size() == 3 && wire::number(com[0]) == 1. &&
                    wire::number(com[1]) == 0. && wire::number(com[2]) == 0.,
                "the tail raw forwards the last measured endpoint mass");
}

// ---- record 5: mode cannot change after a step ------------------------------

void monitor_mode_cannot_change_after_a_step() {
    // Python: set_strict(non-bool) -> ValueError('bool'), and after
    // sim.steps advanced -> RuntimeError('first step'). The native monitor
    // mode is bound at construction; a persisted state claiming a
    // different mode than the config is the post-step analogue and is
    // rejected at the bootstrap seam. The non-bool leg is the config
    // decode gate shared with record 1.
    const auto envelope = bio::parse_json(bt::text(dir() / "config.json"));
    for (const bio::Value &bad :
         {bio::Value{.data = bio::Number{"0"}}, bio::Value{},
          bio::Value{.data = std::string{"false"}}}) {
        auto edited = envelope;
        for (auto &[name, member] : doc_object(edited, "config"))
            if (name == "strict") member = bad;
        bt::invalid([&] { (void)runtime::decode_runtime_config(edited); });
    }
    // Post-step flip: persisted monitor mode disagrees with the config.
    auto boot = runtime::read_bootstrap(dir());
    set_doc_bool(doc_object(doc_member(boot.state.runtime, "monitor"),
                            "monitor"),
                 "strict", true);
    // A strict-mode flip against persisted state is rejected.
    bt::invalid([&] { (void)runtime::NativeRideRuntime(std::move(boot)); });
}

// ---- record 6: reset enters the fresh episode -------------------------------

void reset_enters_new_episode() {
    // The oracle's leg through a throwing prepare() callback is a
    // Python-only seam (the seed object exists only to observe ordering).
    // The ported contract: after committed advancement, reset() restores
    // the t=0 boundary — step/time zero, monitor evidence cleared — and
    // the generation advances.
    auto run = std::make_unique<runtime::NativeRideRuntime>(
        runtime::read_bootstrap(dir()));
    const int generation0 = run->snapshot().generation;
    runtime::RideControl const control;
    (void)run->advance(3, control, 0., 0., std::nullopt);
    bt::require(run->snapshot().step == 3 && run->snapshot().time_s > 0.,
                "the runtime commits intervals before reset");
    run->reset();
    const auto snap = run->snapshot();
    bt::require(snap.step == 0 && snap.time_s == 0.,
                "reset restores the t=0 committed boundary");
    bt::require(snap.generation == generation0 + 1,
                "reset enters a new episode generation");
    bt::require(!snap.first_failure.has_value(),
                "reset clears the first-failure evidence");
    run->close();
}

// ---- records 7/8/9: failure evidence through the research owner --------------

// The shared research-failure fixture: strict mode + priced-out weld
// bound produce a real *.normal rejection; an optional test_fail_at_step
// arm injects the engine failure at the committed boundary.
[[nodiscard]] std::unique_ptr<runtime::NativeResearchRuntime>
armed_env(std::optional<std::int64_t> fail_step) {
    auto env = make_env(strict_bootstrap());
    if (fail_step.has_value()) env->test_fail_at_step(*fail_step);
    return env;
}

void headless_tail_failure_preserves_primary_error() {
    // Python parametrizes over tail exception classes; the native contract
    // is one path — handle_failure() keeps the primary exception and
    // records the trailing-period cleanup result as evidence. Armed both
    // ways (strict violations pending in the tail + engine failure), the
    // trailing interval check fails inside cleanup and lands in
    // cleanup_notes without replacing the primary RuntimeError. The
    // no-output-files leg is adapter plumbing (the run dir is a CLI
    // artifact); the ported evidence is "no transition is fabricated".
    auto env = armed_env(2);
    runtime::RideControl const control;
    env->begin_control(control, 0., 0.);
    std::string primary;
    try {
        (void)env->advance_control(std::nullopt, std::nullopt);
    } catch (const std::runtime_error &failure) {
        primary = failure.what();
    }
    bt::require(!primary.empty(), "the primary engine failure propagates");
    const auto status = env->status();
    bt::require(wire::string(wire::required(status, "reason")) == "simulation_error",
                "the terminal reason stays simulation_error");
    const auto &error = wire::string(wire::required(status, "error"));
    bt::require(error.starts_with("RuntimeError: "),
                "the error keeps the primary exception class prefix");
    const auto &notes = wire::array(wire::required(status, "cleanup_notes"));
    bool trailing_interval = false;
    for (const auto &note : notes)
        trailing_interval = trailing_interval ||
            wire::string(note).find("trailing interval check") != std::string::npos;
    bt::require(trailing_interval,
                "the trailing strict flush failure is recorded, not raised");
    bt::require(env->transitions().empty(),
                "no transition is fabricated for the failed control");
    env->close();
}

void research_cleanup_keeps_primary_exception_and_terminal_metadata(bool reference) {
    auto env = reference ? armed_env(std::nullopt) : armed_env(2);
    runtime::RideControl const control;
    env->begin_control(control, 0., 0.);
    std::string captured;
    try {
        (void)env->advance_control(std::nullopt, std::nullopt);
    } catch (const runtime::InvalidReferenceRun &failure) {
        captured = std::string("InvalidReferenceRun: ") + failure.what();
    } catch (const std::runtime_error &failure) {
        captured = std::string("RuntimeError: ") + failure.what();
    }
    bt::require(!captured.empty(), "the primary exception propagates");
    const auto status = env->status();
    bt::require(wire::boolean(wire::required(status, "terminated")),
                "the episode terminates on the primary failure");
    const auto &reason = wire::string(wire::required(status, "reason"));
    const auto &error = wire::string(wire::required(status, "error"));
    if (reference) {
      bt::require(reason == "invalid_controller" &&
                      error.starts_with("InvalidReferenceRun: "),
                  "a strict rejection is terminal invalid_controller evidence");
    } else {
      bt::require(reason == "simulation_error" &&
                      error.starts_with("RuntimeError: "),
                  "an engine failure is terminal simulation_error evidence");
    }
    // The Python __notes__ analogue: cleanup evidence is an owned list
    // (populated whenever a trailing cleanup step itself fails).
    (void)wire::array(wire::required(status, "cleanup_notes"));
    env->close();
}

void failed_sample_consumption_is_not_retried() {
    // The oracle monkeypatched the recorder to throw mid-consume — a
    // Python seam. The ported contract: the consume cursor is claimed
    // before side effects and acknowledged through the claim, so every
    // published sample integrates exactly once and nothing stays pending
    // for a retry. Setup: identical fixture envs — one clean first window,
    // one armed to fail after the window boundary — share the same
    // published prefix; the failed run's trailing consume pass must land
    // the identical torque integral (a retried sample would double it).
    runtime::RideControl control;
    control.motor_torque_nm = 10.;
    // Non-strict fixture env: the reference integral over the first
    // period's samples for the identical command.
    auto clean = make_env(runtime::read_research_bootstrap(dir()));
    clean->begin_control(control, 0., 0.);
    (void)clean->advance_control(std::nullopt, std::nullopt);
    const double clean_torque =
        wire::number(wire::required(clean->status(), "torque_delivered_nms"));
    clean->close();

    // Same fixture, armed past the first period close: the published
    // prefix is already consumed when the engine failure lands, then
    // handle_failure runs the trailing consume pass over the same cursor.
    auto failed = make_env(runtime::read_research_bootstrap(dir()));
    failed->test_fail_at_step(9);
    failed->begin_control(control, 0., 0.);
    (void)failed->advance_control(std::nullopt, std::nullopt);
    failed->acknowledge_control();
    failed->begin_control(control, 0., 0.);
    bool threw = false;
    try {
        (void)failed->advance_control(std::nullopt, std::nullopt);
    } catch (const std::runtime_error &) {
        threw = true;
    }
    bt::require(threw, "the armed engine failure propagates");
    const auto status = failed->status();
    bt::require(wire::integer(wire::required(
                    section(status, "accounting"), "pending_intervals")) == 0,
                "claimed samples are never left pending for a retry");
    const double failed_torque =
        wire::number(wire::required(status, "torque_delivered_nms"));
    bt::require(failed_torque == clean_torque,
                "the consumed prefix is integrated exactly once");
    failed->close();
}

// ---- record 10: rejection keeps one owner, strict mode, saved failure -------

void research_rejection_keeps_single_owner_reset_mode_and_saved_failure() {
    auto env = armed_env(std::nullopt); // strict, real weld violation
    runtime::RideControl const control;
    try {
        env->begin_control(control, 0., 0.);
        (void)env->advance_control(std::nullopt, std::nullopt);
        bt::require(false, "the strict rejection must propagate");
    } catch (const runtime::InvalidReferenceRun &) { // NOLINT(bugprone-empty-catch) the expected rejection is consumed here; falling through is the assertion
    }
    const auto status = env->status();
    bt::require(wire::string(wire::required(status, "reason")) == "invalid_controller",
                "a strict rejection is the invalid_controller terminal");
    const auto &accounting = section(status, "accounting");
    const auto &monitor = section(accounting, "monitor");
    bt::require(wire::boolean(wire::required(monitor, "strict")),
                "the monitor owner stays strict");
    const auto &first = wire::array(wire::required(monitor, "first_failure"));
    const auto steps = wire::integer(wire::required(status, "step"));
    const double dt = steps > 0
                          ? wire::number(wire::required(status, "time_s")) /
                                static_cast<double>(steps)
                          : 0.;
    bt::require(first.size() == 2 && wire::number(first[0]) > 0. &&
                    wire::number(first[0]) <= dt * (1. + 1e-9),
                "the saved first failure lands on its own violating interval");
    const auto &model_status = section(accounting, "model_status");
    bt::require(!wire::boolean(wire::required(model_status, "model_valid")),
                "the saved evidence keeps the invalid model status");
    bt::require(wire::integer(wire::required(status, "recorded_rows")) ==
                    wire::integer(wire::required(status, "step")),
                "recorded rows track committed steps at decimation 1");
    // reset(): same owner (the env object persists), strict preserved,
    // first_failure cleared — the oracle's identity check maps to the
    // single-owner contract on the accounting state.
    env->reset();
    const auto after = env->status();
    const auto &monitor_after =
        section(section(after, "accounting"), "monitor");
    bt::require(wire::boolean(wire::required(monitor_after, "strict")) &&
                    wire::is_none(wire::required(monitor_after, "first_failure")),
                "reset keeps the strict owner and clears the failure");
    env->close();
}

} // namespace

// The research-fixture legs plus the neutral-raw accounting legs inline
// into this one dispatcher; the aggregate frame exceeds the 8 KiB policy
// even though each callee stays within budget, so the suppression wraps
// main only (same pattern as test_period_buffer.cpp).
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        defaults_to_strict_and_validates_mode();
        for (const bool strict : {true, false})
            for (const int decimation : {1, 400})
                raw_failure_time_and_sticky_status_ignore_decimation(strict,
                                                                     decimation);
        unmeasured_partial_period_keeps_previous_energy_datum();
        partial_period_reports_last_measured_energy_on_every_raw();
        monitor_mode_cannot_change_after_a_step();
        reset_enters_new_episode();
        headless_tail_failure_preserves_primary_error();
        research_cleanup_keeps_primary_exception_and_terminal_metadata(false);
        research_cleanup_keeps_primary_exception_and_terminal_metadata(true);
        failed_sample_consumption_is_not_retried();
        research_rejection_keeps_single_owner_reset_mode_and_saved_failure();
    });
}
NATIVE_DIAG_POP
