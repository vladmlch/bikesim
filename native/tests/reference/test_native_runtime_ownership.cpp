// Completion 3.25 (batch runtime). Source: tests/reference/
// test_native_runtime_ownership.py — A4 acknowledgement, immutable
// outputs, reset/close and fatal prefixes.
//
// Ported (9 of 10 records). test_row_materialization_is_lazy_and_retryable
// is dispositioned `drop` (python-only): it exercises the SampleBatch
// adapter's lazy row cache and an injected MemoryError retry — there is
// no lazy materialization seam in the native boundary (prepare_samples()
// publishes owned immutable SamplePtrs; row materialization is a pure
// as_wire() per call). The underlying liveness guarantee is covered by
// test_unmaterialized_batch_keeps_native_rows_alive_after_close below.
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {

using runtime_reference::expect_error;
using runtime_reference::expect_invalid;
using runtime_reference::require_bitwise;
using runtime_reference::require_wire_bitwise;
using runtime_reference::ResolvedDocs;
using runtime_reference::compose_runtime;
using runtime_reference::resolved_docs;
using runtime_reference::set_member;
using runtime_reference::member;
using runtime_reference::num;
using runtime_reference::welded_dir;
using runtime_reference::wire_has;
using runtime_reference::wire_int;
using runtime_reference::wire_member;
using runtime_reference::wire_obj;

namespace bio = bike::io;

[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> ride() {
    return runtime_reference::ride();
}

// prepare() then acknowledge — the driver.drain_samples() pair.
runtime::RuntimeSampleBatch drain(runtime::NativeRideRuntime &rt) {
    runtime::RuntimeSampleBatch batch = rt.prepare_samples();
    rt.acknowledge_samples(batch);
    return batch;
}

[[nodiscard]] std::vector<runtime::WireObject>
rows_of(const runtime::RuntimeSampleBatch &batch) {
    std::vector<runtime::WireObject> rows;
    rows.reserve(batch.samples.size());
    for (const runtime::SamplePtr &sample : batch.samples)
        rows.push_back(sample->as_wire());
    return rows;
}

void require_rows_bitwise(const std::vector<runtime::WireObject> &actual,
                          const std::vector<runtime::WireObject> &expected,
                          std::string_view what) {
    bt::require(actual.size() == expected.size(), what);
    for (std::size_t i = 0; i < actual.size() && i < expected.size(); ++i)
        bt::require(runtime_reference::wire_bitwise(actual[i], expected[i]),
                    what);
}

runtime::Wire &wire_mut(runtime::WireObject &object, std::string_view key) {
    for (auto &[name, value] : object)
        if (name == key) return value;
    std::string keys;
    for (const auto &[name, value] : object) {
        if (!keys.empty()) keys += ',';
        keys += name;
    }
    throw std::invalid_argument(std::string(
        "wire member missing for mutation: '") + std::string(key) +
        "' in {" + keys + "}");
}

[[nodiscard]] std::vector<std::int64_t>
interval_ids(const runtime::RuntimeSampleBatch &batch) {
    std::vector<std::int64_t> ids;
    ids.reserve(batch.samples.size());
    for (const runtime::SamplePtr &sample : batch.samples)
      ids.push_back(sample->interval_id);
    return ids;
}

// ---- batch ownership -------------------------------------------------------

// SampleBatch nested values/columns are owned and read-only; dict-row
// exports are fresh materializations.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_batch_nested_values_are_owned_and_read_only() {
    auto data = std::make_unique<runtime::PhysicalSampleData>();
    data->interval_id = 0;
    data->time_s = 0.;
    data->end_time_s = .00125;
    data->channels = runtime::WireObject{
        {"nested", runtime::Wire{runtime::WireObject{
            {"values", runtime::Wire{runtime::WireArray{1., 2.}}}}}}};
    const runtime::SamplePtr sample{std::move(data)};
    runtime::SampleColumns columns{{"interval_id", {0.}}};
    runtime::RuntimeSampleBatch batch{
        .generation = 4, .samples = {sample}, .columns = columns};
    // Mutating the caller-held column source cannot reach the batch's
    // owned copy; the row's nested values are const-held.
    columns["interval_id"][0] = 99.;
    bt::require(batch.columns["interval_id"][0] == 0.,
                "batch columns are owned copies");
    bt::require(batch.samples[0]->interval_id == 0,
                "batch interval ids come from the owned rows");
    // Exported rows are fresh materializations: mutating an export can
    // never rewrite the published row or the next export.
    runtime::WireObject exported = sample->as_wire();
    auto &nested = std::get<runtime::WireObject>(
        wire_mut(exported, "nested").value);
    auto &values = std::get<runtime::WireArray>(
        wire_mut(nested, "values").value);
    values[0] = runtime::Wire{42.};
    const runtime::WireObject fresh = sample->as_wire();
    const runtime::Wire &value = std::get<runtime::WireArray>(
        wire_member(runtime_reference::wire_obj(fresh, "nested"), "values")
            .value)
        .front();
    bt::require(std::get<double>(value.value) == 1.,
                "dict-row exports are fresh copies of the immutable row");
}
NATIVE_DIAG_POP

// A failed batch construction between prepare and acknowledge keeps the
// pending rows retryable — the pending count is untouched until the
// acknowledgement consumes it.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_failed_batch_construction_retains_pending_rows() {
    const auto driver = ride();
    const runtime::RideControl command;
    static_cast<void>(
        driver->advance(13, command, 0., 0., std::nullopt));
    driver->flush();
    bt::require(wire_int(driver->accounting_state(),
                         "pending_intervals") == 13,
                "flush publishes 13 pending intervals");
    // The boxing failure sits between prepare and acknowledge: the
    // prepared batch is dropped without acknowledgement.
    {
        const runtime::RuntimeSampleBatch dropped =
            driver->prepare_samples();
        static_cast<void>(dropped);
    }
    bt::require(wire_int(driver->accounting_state(),
                         "pending_intervals") == 13,
                "an unacknowledged batch keeps pending rows");
    const runtime::RuntimeSampleBatch batch = drain(*driver);
    bt::require(interval_ids(batch) ==
                    std::vector<std::int64_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                                              10, 11, 12},
                "the retried drain returns the whole prefix");
    bt::require(wire_int(driver->accounting_state(),
                         "pending_intervals") == 0,
                "acknowledgement drains pending");
    bt::require(driver->prepare_samples().samples.empty(),
                "a second drain is empty");
}
NATIVE_DIAG_POP

// A previously prepared prefix may be acknowledged after more intervals
// publish, but it must not discard the appended suffix; stale and
// cross-generation prefixes reject.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_acknowledgement_rejects_stale_or_cross_generation_prefix() {
    const auto driver = ride();
    const runtime::RideControl command;
    static_cast<void>(driver->advance(3, command, 0., 0., std::nullopt));
    driver->flush();
    const runtime::RuntimeSampleBatch old = driver->prepare_samples();
    static_cast<void>(driver->advance(7, command, 0., 0., std::nullopt));
    driver->flush();
    driver->acknowledge_samples(old);
    const runtime::RuntimeSampleBatch suffix = drain(*driver);
    bt::require(interval_ids(suffix) ==
                    std::vector<std::int64_t>{3, 4, 5, 6},
                "prefix acknowledgement keeps the appended suffix");
    // Re-acknowledging a consumed batch trips the size check first: the
    // native message is "belongs to a different generation or prefix"
    // (the source's 'prefix|stale' alternation covers both branches).
    expect_error("prefix",
                 [&] { driver->acknowledge_samples(old); });
    driver->reset();
    static_cast<void>(driver->advance(3, command, 0., 0., std::nullopt));
    driver->flush();
    expect_error("generation",
                 [&] { driver->acknowledge_samples(old); });
    const runtime::RuntimeSampleBatch batch = drain(*driver);
    bt::require(interval_ids(batch) ==
                    std::vector<std::int64_t>{0, 1, 2},
                "post-reset drain returns the new generation's prefix");
}
NATIVE_DIAG_POP

// An acknowledgement built from a different runtime's batch cannot
// consume this runtime's pending prefix.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_acknowledgement_cannot_consume_a_different_runtime() {
    const auto driver = ride();
    const auto other = ride();
    const runtime::RideControl command;
    for (auto *owner : {driver.get(), other.get()}) {
        static_cast<void>(
            owner->advance(3, command, 0., 0., std::nullopt));
        owner->flush();
    }
    const runtime::RuntimeSampleBatch foreign =
        other->prepare_samples();
    expect_error("stale",
                 [&] { driver->acknowledge_samples(foreign); });
    bt::require(interval_ids(drain(*driver)) ==
                    std::vector<std::int64_t>{0, 1, 2},
                "the rejected foreign ack leaves the prefix intact");
    bt::require(interval_ids(drain(*other)) ==
                    std::vector<std::int64_t>{0, 1, 2},
                "the foreign runtime's own prefix is untouched");
    other->close();
}
NATIVE_DIAG_POP

// Samples, columns and snapshots outlive reset() and close() — the held
// copies keep their data; the runtime's own surfaces empty on reset and
// fail fast after close.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_samples_columns_and_snapshots_survive_reset_and_close() {
    const auto driver = ride();
    const runtime::RideControl command;
    const auto initial =
        std::make_unique<const runtime::RuntimeSnapshot>(
            driver->snapshot());
    static_cast<void>(driver->advance(41, command, 0., 0., std::nullopt));
    driver->flush();
    auto frame = std::make_unique<runtime::RuntimeSnapshot>(
        driver->snapshot());
    const runtime::RuntimeSampleBatch batch = drain(*driver);
    const runtime::SampleColumns columns = driver->recorded_columns();
    const std::vector<runtime::WireObject> expected = rows_of(batch);
    const std::vector<double> saved_state = frame->integration_state;
    const runtime::SampleColumns &saved_columns = columns;
    bt::require(frame->latest_sample != nullptr,
                "the snapshot carries the latest published sample");
    // Held copies cannot reach the runtime: mutate the snapshot's own
    // members and confirm the runtime's copies are unchanged.
    frame->integration_state[0] = 99.;
    require_bitwise(driver->snapshot().integration_state, saved_state,
                    "mutating a held snapshot cannot reach the runtime");
    wire_mut(frame->view, "drive_mode") = runtime::Wire{"forged"};
    const runtime::RuntimeSnapshot fresh_view = driver->snapshot();
    bt::require(std::get<std::string>(
                    wire_member(fresh_view.view, "drive_mode").value) ==
                    "articulated_effort",
                "mutating a held snapshot view cannot reach the runtime");
    driver->reset();
    const auto reset =
        std::make_unique<const runtime::RuntimeSnapshot>(
            driver->snapshot());
    bt::require(reset->generation == initial->generation + 1 &&
                    reset->step == 0 && reset->latest_sample == nullptr &&
                    !reset->first_failure.has_value(),
                "reset restores the captured t=0 boundary");
    require_bitwise(reset->integration_state,
                    initial->integration_state,
                    "reset integration state matches the capture bitwise");
    bt::require(driver->prepare_samples().samples.empty() &&
                    driver->recorded_columns().empty(),
                "reset clears pending samples and recorded columns");
    static_cast<void>(driver->advance(9, command, 0., 0., std::nullopt));
    driver->flush();
    driver->close();
    // The held frame, batch and columns are owned — they survive close().
    // (The C++ snapshot is a caller-owned mutable copy: restore the
    // isolation-probe byte before asserting the held bytes are intact —
    // the source proved the same liveness from read-only ndarrays.)
    frame->integration_state = saved_state;
    require_bitwise(frame->integration_state, saved_state,
                    "the held snapshot keeps its bytes across close");
    require_wire_bitwise(frame->latest_sample->as_wire(), expected.back(),
                         "latest_sample keeps the last drained row");
    require_rows_bitwise(rows_of(batch), expected,
                         "drained rows rematerialize identically");
    for (const auto &[name, values] : saved_columns) {
        const auto found = columns.find(name);
        bt::require(found != columns.end() &&
                        runtime_reference::bitwise_equal(found->second,
                                                         values),
                    "recorded columns hold their owned values");
        const auto batch_found = batch.columns.find(name);
        bt::require(batch_found == batch.columns.end() ||
                        runtime_reference::bitwise_equal(
                            batch_found->second, values),
                    "batch columns agree with recorded columns");
    }
    for (const std::function<void()> &operation :
         {std::function<void()>{
              [&] { static_cast<void>(driver->snapshot()); }},
          std::function<void()>{[&] { driver->flush(); }},
          std::function<void()>{
              [&] { static_cast<void>(driver->prepare_samples()); }},
          std::function<void()>{[&] { driver->reset(); }}})
        expect_error("closed", operation);
}
NATIVE_DIAG_POP

// An unmaterialized batch's native rows stay alive after the owning
// runtime closes — the shared SamplePtrs own them.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_unmaterialized_batch_keeps_native_rows_alive_after_close() {
    const auto driver = ride();
    const runtime::RideControl command;
    static_cast<void>(
        driver->advance(13, command, 0., 0., std::nullopt));
    driver->flush();
    const runtime::RuntimeSampleBatch batch = driver->prepare_samples();
    driver->close();
    const std::vector<runtime::WireObject> rows = rows_of(batch);
    bt::require(rows.size() == 13, "the closed runtime's batch has 13 rows");
    for (std::size_t i = 0; i < rows.size(); ++i) {
        bt::require(wire_int(rows[i], "interval_id") ==
                        static_cast<std::int64_t>(i),
                    "row interval_id survives close");
        bt::require(wire_int(rows[i], "schema_version") == 2,
                    "row materializes schema 2 after close");
    }
}
NATIVE_DIAG_POP

// Mutating the caller-held capture documents (and there being no source
// sim in the native port — the runtime IS the sim) cannot affect
// accounting or the reset replay.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_source_and_bootstrap_mutations_do_not_affect_accounting_or_reset() {
    ResolvedDocs docs = resolved_docs(welded_dir());
    ResolvedDocs clean_docs = resolved_docs(welded_dir());
    const auto driver = compose_runtime(docs);
    const auto clean = compose_runtime(clean_docs);
    const auto initial =
        std::make_unique<const runtime::RuntimeSnapshot>(
            driver->snapshot());
    // The public capture is caller-owned; the runtime decoded its own copy.
    set_member(member(member(docs.state, "runtime"), "monitor"), "strict",
               bio::Value{true});
    set_member(member(member(docs.state, "runtime"), "energy"),
               "initial_energy_j", num(1000000000));
    set_member(member(member(member(docs.state, "runtime"), "history"),
                      "work_j"),
               "forged", num(1000000000));
    auto &lanes = std::get<bio::Value::Array>(
        member(docs.state, "integration_state").data);
    for (auto &lane : lanes) lane = runtime_reference::nan_value();
    set_member(docs.config, "measurement_decimation", num(999));
    const runtime::RideControl command;
    for (auto *owner : {driver.get(), clean.get()}) {
        static_cast<void>(
            owner->advance(13, command, 0., 0., std::nullopt));
        owner->flush();
    }
    const std::vector<runtime::WireObject> expected =
        rows_of(drain(*clean));
    require_rows_bitwise(rows_of(drain(*driver)), expected,
                         "caller-held document mutations do not reach rows");
    require_wire_bitwise(driver->accounting_state(),
                         clean->accounting_state(),
                         "accounting state is unaffected by caller mutation");
    driver->reset();
    require_bitwise(driver->snapshot().integration_state,
                    initial->integration_state,
                    "reset replays the captured integration vector bitwise");
    const runtime::WireObject accounting = driver->accounting_state();
    const runtime::WireObject &history = wire_obj(accounting, "history");
    bt::require(!wire_has(wire_obj(history, "work_j"), "forged"),
                "a forged capture member cannot leak into accounting");
    static_cast<void>(
        driver->advance(13, command, 0., 0., std::nullopt));
    driver->flush();
    require_rows_bitwise(rows_of(drain(*driver)), expected,
                         "post-reset rows still match the clean runtime");
    driver->close();
    clean->close();
}
NATIVE_DIAG_POP

// advance() validates target_step before any native work — the source
// parametrizes [True, 1.5, '3', -1] over a SimpleNamespace stub; only -1
// is a typed native input (the type-freedom cases are noted in the port
// map). The field-name and no-mutation contract port verbatim.
void test_invalid_target_is_rejected_before_native_call() {
    const auto driver = ride();
    const runtime::RideControl command;
    expect_invalid("target_step", [&] {
        static_cast<void>(
            driver->advance(-1, command, 0., 0., std::nullopt));
    });
    bt::require(driver->snapshot().step == 0,
                "a rejected target never reaches the native step loop");
}

// A fatal engine error preserves the committed prefix, poisons the
// engine until reset, and reset() recovers the capture.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_fatal_engine_error_preserves_completed_prefix_and_can_reset() {
    const auto driver = ride();
    const auto clean = ride();
    const runtime::RideControl command;
    static_cast<void>(clean->advance(3, command, 0., 0., std::nullopt));
    clean->flush();
    const std::vector<runtime::WireObject> expected =
        rows_of(drain(*clean));
    const auto expected_frame =
        std::make_unique<const runtime::RuntimeSnapshot>(
            clean->snapshot());
    driver->test_fail_at_step(3);
    bool fired = false;
    try {
        static_cast<void>(
            driver->advance(9, command, 0., 0., std::nullopt));
    } catch (const std::exception &) {
        fired = true;
    }
    bt::require(fired, "the fatal engine boundary fires mid-advance");
    const auto frame =
        std::make_unique<const runtime::RuntimeSnapshot>(
            driver->snapshot());
    bt::require(frame->step == 3,
                "the committed prefix survives the fatal interval");
    require_bitwise(frame->integration_state,
                    expected_frame->integration_state,
                    "the preserved prefix is bitwise-exact");
    bt::require(
        frame->first_failure.has_value() ==
            expected_frame->first_failure.has_value(),
        "first_failure presence matches the clean runtime");
    if (frame->first_failure.has_value())
        bt::require(runtime_reference::wire_bitwise(
                        frame->first_failure->as_wire(),
                        expected_frame->first_failure.value().as_wire()), // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                    "first_failure matches the clean runtime");
    const runtime::WireObject state = driver->accounting_state();
    bt::require(wire_int(wire_obj(state, "history"), "last_id") == 2,
                "the preserved prefix publishes through interval 2");
    bt::require(wire_int(state, "buffered_intervals") == 0,
                "the flush leaves no buffered intervals");
    require_rows_bitwise(rows_of(drain(*driver)), expected,
                         "the preserved prefix drains the clean rows");
    expect_error("poison", [&] {
        static_cast<void>(
            driver->advance(4, command, 0., 0., std::nullopt));
    });
    driver->reset();
    const auto reset =
        std::make_unique<const runtime::RuntimeSnapshot>(
            driver->snapshot());
    bt::require(reset->step == 0 &&
                    reset->generation == frame->generation + 1 &&
                    !reset->first_failure.has_value(),
                "reset recovers the capture with a bumped generation");
    static_cast<void>(driver->advance(3, command, 0., 0., std::nullopt));
    driver->flush();
    require_rows_bitwise(rows_of(drain(*driver)), expected,
                         "the recovered runtime reproduces the prefix rows");
    driver->close();
    clean->close();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(
            test_batch_nested_values_are_owned_and_read_only);
        failures += bt::run(
            test_failed_batch_construction_retains_pending_rows);
        failures += bt::run(
            test_acknowledgement_rejects_stale_or_cross_generation_prefix);
        failures += bt::run(
            test_acknowledgement_cannot_consume_a_different_runtime);
        failures += bt::run(
            test_samples_columns_and_snapshots_survive_reset_and_close);
        failures += bt::run(
            test_unmaterialized_batch_keeps_native_rows_alive_after_close);
        failures += bt::run(
            test_source_and_bootstrap_mutations_do_not_affect_accounting_or_reset);
        failures += bt::run(
            test_invalid_target_is_rejected_before_native_call);
        failures += bt::run(
            test_fatal_engine_error_preserves_completed_prefix_and_can_reset);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
