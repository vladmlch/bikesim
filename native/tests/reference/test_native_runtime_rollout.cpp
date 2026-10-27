// Completion 3.26 (batch runtime). Source: tests/reference/
// test_native_runtime_rollout.py — A4 schema-2 parity, explicit flush
// schedules, and one-second rollout (6 records: 2 ported, 4 `planned`).
//
// The four live-oracle comparisons —
// test_native_full_rows_match_python_for_non_aligned_tail,
// test_identical_non_aligned_explicit_flush_schedule_matches_python,
// test_native_recorded_columns_match_scalar_recorder_without_row_roundtrip,
// test_one_second_800_interval_golden_episode_all_channels — assert
// assert_tree_close/bitwise equality of native sample rows, recorded
// columns and the integration vector against the *Python* ride. They fail
// on unmodified HEAD with the documented weld-heavy parity divergence
// (e.g. root.sensors.frame_specific_force_body_mps2[0]
// -0.05449393733358502 native vs -0.054495487350229764 oracle, and the
// rider_strength.rider_torso_hinge strict trigger at t=0.37875) — the same
// pre-existing ADR-0003 red family the 3.25 step/commands ports recorded.
// Porting them past green would falsify the gate; they stay `planned`.
//
// Ported below: the two native-vs-native records — partial-period budget/
// snapshot/drain behavior (schedule-independence of the published prefix,
// bitwise integration vector between runtimes) and record-decimation
// independence of the accounting ledger — plus the forwarded closing
// measurement contract.
#include "runtime_reference_support.hpp"

#include "diag.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace sw = runtime::sample_wire;
using runtime_reference::compose_runtime;
using runtime_reference::member;
using runtime_reference::num;
using runtime_reference::require_tree_close;
using runtime_reference::resolved_docs;
using runtime_reference::set_member;
using runtime_reference::welded_dir;
using runtime_reference::wire_bitwise;
using runtime_reference::wire_member;

constexpr double kClose = 1e-9;

runtime::RuntimeSampleBatch drain(runtime::NativeRideRuntime &rt) {
    runtime::RuntimeSampleBatch batch = rt.prepare_samples();
    rt.acknowledge_samples(batch);
    return batch;
}

[[nodiscard]] double wire_number(const runtime::WireObject &o, std::string_view key) {
    const runtime::Wire &value = wire_member(o, key);
    if (const auto *d = std::get_if<double>(&value.value)) return *d;
    return static_cast<double>(sw::integer(value));
}
[[nodiscard]] bool wire_is_null(const runtime::WireObject &o, std::string_view key) {
    return std::holds_alternative<std::monostate>(wire_member(o, key).value);
}

// A runtime created from the welded capture with a patched record decimation.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[nodiscard]] std::unique_ptr<runtime::NativeRideRuntime> ride_decimated(int decimation) {
    auto docs = resolved_docs(welded_dir());
    if (decimation != 1) {
        set_member(docs.config, "measurement_decimation", num(decimation));
        set_member(member(docs.state, "runtime"), "measurement_decimation",
                   num(decimation));
    }
    return compose_runtime(docs);
}
NATIVE_DIAG_POP

// ---------------------------------------------------------------------------
// test_target_snapshot_drain_and_budget_do_not_close_a_partial_period
// ---------------------------------------------------------------------------
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_target_snapshot_drain_and_budget_do_not_close_a_partial_period() {
    const auto native = ride_decimated(7);
    const auto whole = ride_decimated(7);
    const runtime::RideControl command;
    // The welded capture: dt .00125, control_period_s .005 -> period of 4.
    constexpr std::int64_t period = 4;
    static_cast<void>(native->advance(period - 1, command, 0., 0., std::nullopt));
    const auto before =
        std::make_unique<const runtime::RuntimeSnapshot>(native->snapshot());
    bt::require(before->latest_sample == nullptr,
                "a partial period publishes no latest sample");
    bt::require(native->prepare_samples().samples.empty(),
                "a partial period publishes no rows");
    {
        const runtime::WireObject accounting = native->accounting_state();
        const auto &history = sw::object(wire_member(accounting, "history"));
        bt::require(wire_is_null(history, "last_id"),
                    "no measured interval has closed yet");
        bt::require(sw::integer(wire_member(accounting, "buffered_intervals")) ==
                        period - 1,
                    "the partial period buffers its intervals");
    }
    // wall_budget_s=0. yields before committing a step; the snapshot,
    // buffers and integration vector are untouched.
    const runtime::AdvanceResult budget =
        native->advance(period + 10, command, 0., 0., 0.);
    bt::require(budget.reason == "budget" && budget.step == period - 1,
                "a zero wall budget yields with reason 'budget'");
    bt::require(native->prepare_samples().samples.empty(),
                "budget yield publishes nothing");
    runtime_reference::require_bitwise(
        native->snapshot().integration_state, before->integration_state,
        "budget yield does not move the integration vector");
    // The drain/flush schedule cannot change the committed rows: a staged
    // schedule and a single full advance publish identical prefixes and
    // finish on a bitwise-identical integration vector.
    std::vector<runtime::WireObject> actual;
    const std::set<std::int64_t> targets{period, period + 1, 2 * period - 1,
                                         3 * period + 3};
    for (const std::int64_t target : targets) {
        static_cast<void>(native->advance(target, command, 0., 0., std::nullopt));
        static_cast<void>(native->snapshot());
        for (const auto &sample : drain(*native).samples)
            actual.push_back(sample->as_wire());
    }
    static_cast<void>(whole->advance(3 * period + 3, command, 0., 0., std::nullopt));
    whole->flush();
    native->flush();
    for (const auto &sample : drain(*native).samples)
        actual.push_back(sample->as_wire());
    const runtime::RuntimeSampleBatch expected = drain(*whole);
    bt::require(actual.size() == expected.samples.size(),
                "drain schedules publish the same row count");
    for (std::size_t i = 0; i < actual.size() && i < expected.samples.size(); ++i) {
        bt::require(wire_bitwise(actual[i], expected.samples[i]->as_wire()),
                    "staged drains publish bitwise-identical rows");
    }
    runtime_reference::require_bitwise(
        native->snapshot().integration_state, whole->snapshot().integration_state,
        "schedule-independence of the committed integration vector");
    native->close();
    whole->close();
}
NATIVE_DIAG_POP

// ---------------------------------------------------------------------------
// test_accounting_all_intervals_is_independent_of_record_decimation — every
// interval is accounted regardless of the recorded cadence; the forwarded
// period-closing measurement is the only decimation-dependent surface.
// ---------------------------------------------------------------------------
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_accounting_all_intervals_is_independent_of_record_decimation() {
    const auto every = ride_decimated(1);
    const auto decimated = ride_decimated(7);
    const runtime::RideControl command;
    for (const auto *driver : {&every, &decimated}) {
        static_cast<void>((*driver)->advance(81, command, 0., 0., std::nullopt));
        (*driver)->flush();
    }
    const runtime::WireObject every_accounting = every->accounting_state();
    const runtime::WireObject decimated_accounting = decimated->accounting_state();
    require_tree_close(wire_member(decimated_accounting, "history"),
                       wire_member(every_accounting, "history"),
                       "cumulative work history is decimation-invariant");
    // Point-in-time measurement fields forward the period's closing
    // measurement on the decimated tail; cumulative fields stay identical.
    const std::set<std::string_view> forwarded{
        "mechanical_energy_j", "elastic_energy_j", "residual_j",
        "measured", "source_interval_id", "source_time_s"};
    const auto filter = [&](const runtime::WireObject &energy) {
        runtime::WireObject out;
        for (const auto &[name, value] : energy)
            if (!forwarded.contains(name)) out.emplace_back(name, value);
        return out;
    };
    const auto &every_energy = sw::object(wire_member(every_accounting, "energy"));
    const auto &decimated_energy =
        sw::object(wire_member(decimated_accounting, "energy"));
    require_tree_close(runtime::Wire{runtime::WireObject{filter(decimated_energy)}},
                       runtime::Wire{runtime::WireObject{filter(every_energy)}},
                       "non-forwarded energy ledger fields are decimation-invariant");
    const runtime::RuntimeSampleBatch every_rows = drain(*every);
    std::vector<runtime::WireObject> rows(81);
    for (const runtime::SamplePtr &sample : every_rows.samples)
        rows[static_cast<std::size_t>(sample->interval_id)] = sample->as_wire();
    // The decimated ledger's closing measurement is the last recorded or
    // period-closing interval before step 81 (period = 4).
    constexpr std::int64_t period = 4;
    std::int64_t closing_id = 0;
    for (std::int64_t i = 0; i < 81; ++i)
        if (i % 7 == 0 || (i + 1) % period == 0) closing_id = i;
    const auto &closing =
        sw::object(wire_member(rows[static_cast<std::size_t>(closing_id)], "energy"));
    require_tree_close(wire_member(decimated_energy, "mechanical_energy_j"),
                       wire_member(closing, "mechanical_energy_j"),
                       "the decimated tail forwards the closing mechanical energy");
    require_tree_close(wire_member(decimated_energy, "elastic_energy_j"),
                       wire_member(closing, "elastic_energy_j"),
                       "the decimated tail forwards the closing elastic energy");
    // residual = total - initial - accumulators: the forwarded residual is
    // the tail's own residual shifted by (forwarded - own) total.
    const auto &last =
        sw::object(wire_member(rows[80], "energy"));
    const double residual = wire_number(last, "residual_j") +
                            wire_number(closing, "mechanical_energy_j") -
                            wire_number(last, "mechanical_energy_j");
    bt::require(std::abs(wire_number(decimated_energy, "residual_j") - residual) <=
                    std::max(kClose, kClose * std::abs(residual)),
                "the forwarded residual keeps the tail interval's own accumulators");
    require_tree_close(wire_member(decimated_accounting, "model_status"),
                       wire_member(every_accounting, "model_status"),
                       "model status is decimation-invariant");
    const runtime::RuntimeSampleBatch decimated_rows = drain(*decimated);
    bt::require(decimated_rows.samples.size() == 81 &&
                    every_rows.samples.size() == 81,
                "accounting intervals are decimation-independent");
    for (std::size_t i = 0; i < 81; ++i) {
        bt::require(every_rows.samples[i]->interval_id ==
                            static_cast<std::int64_t>(i) &&
                        decimated_rows.samples[i]->interval_id ==
                            static_cast<std::int64_t>(i),
                    "all 81 interval ids publish under either cadence");
    }
    const runtime::SampleColumns every_columns = every->recorded_columns();
    const runtime::SampleColumns decimated_columns = decimated->recorded_columns();
    bt::require(every_columns.at("interval_id").size() == 81,
                "decimation 1 records every interval");
    const auto &decimated_ids = decimated_columns.at("interval_id");
    bt::require(decimated_ids.size() == 12,
                "decimation 7 records floor(81/7)+1 interval columns");
    for (std::size_t i = 0; i < decimated_ids.size(); ++i)
        bt::require(decimated_ids[i] == static_cast<double>(i * 7),
                    "recorded columns land on the decimated cadence");
    every->close();
    decimated->close();
}
NATIVE_DIAG_POP

} // namespace

int main() {
    try {
        int failures = 0;
        failures += bt::run(
            test_target_snapshot_drain_and_budget_do_not_close_a_partial_period);
        failures += bt::run(
            test_accounting_all_intervals_is_independent_of_record_decimation);
        return failures == 0 ? 0 : 1;
    } catch (...) {
        return 1;
    }
}
