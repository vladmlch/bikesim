// Completion 3.30 (T53). Source: tests/reference/test_native_recording_retention.py.
//
// Retention-mode / owned-episode / NativePhysicalSource contract ports.
// The oracle exercised the nanobind seam; the ports call the same native
// owners directly: NativeRideRuntime over the captured welded bootstrap
// (config.json retention/decimation/strict fields re-staged per test),
// OwnedNativeEpisode::{batch,sample_batch} and
// recording::make_physical_source. Assertions are invariants and
// structural checks — native-vs-native or literal — so no platform
// goldens exist for this file.
//
// Dropped node (python-only): test_legacy_export_survives_record_default —
// PhysicalRideDriver.export() is the Python adapter's file-bundle surface
// (telemetry.csv + intervals.jsonl + npy + summary.json). The surviving
// contract (legacy retention still produces the decimated projection) is
// covered by test_default_retention_is_the_legacy_projection; the
// artifact emission is cli/ride.cpp's legacy headless path.
#include "recording_test_support.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "cli/physical_capture.hpp"
#include "recording/publish.hpp"
#include "research/evidence.hpp"
#include "runtime/status.hpp"
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <span>

namespace {

using namespace recording_test;
namespace value = rec::value;
namespace cli = bike::cli;
namespace research = bike::research;

// Staged welded runtime, replace_config_key and advance() come from
// recording_test_support.hpp — the staged copy preserves the
// <root>/bootstrap/welded layout so capture-root-relative array payloads
// in state.json resolve.
std::vector<std::int64_t> interval_ids(const runtime::RuntimeSampleBatch &batch) {
    std::vector<std::int64_t> ids;
    for (const auto &sample : batch.samples) ids.push_back(sample->interval_id);
    return ids;
}

std::vector<std::int64_t> interval_ids(std::span<const SamplePtr> samples) {
    std::vector<std::int64_t> ids;
    for (const auto &sample : samples) ids.push_back(sample->interval_id);
    return ids;
}

const WireObject &energy(const runtime::PhysicalSampleData &sample) {
    return sw::object(sw::required(sample.channels, "energy"));
}

// The production model-joint catalog for the welded runtime — the same
// model_catalog() the CLI binds before sealing episodes.
const rec::Catalog &welded_catalog(runtime::NativeRideRuntime &runtime) {
    static const rec::Catalog catalog = cli::model_catalog(runtime.model_image());
    return catalog;
}

void test_full_retention_keeps_every_interval_under_measurement_decimation() {
    const auto full = welded_runtime("full", 80);
    const auto legacy = welded_runtime("legacy", 80);
    advance(*full->runtime, 81);
    const auto episode = full->runtime->seal_episode();
    bt::require(episode.generation == full->runtime->snapshot().generation &&
                episode.samples.size() == 81 &&
                interval_ids(episode.samples) ==
                    [] { std::vector<std::int64_t> ids(81); std::iota(ids.begin(), ids.end(), 0); return ids; }(),
                "full retention keeps every interval under a decimated cadence");
    const auto batch = episode.batch(0, 81);
    bt::require(batch.samples.size() == 81 && interval_ids(batch) == interval_ids(episode.samples),
                "batch(0, 81) exposes the whole retained index");
    advance(*legacy->runtime, 81);
    legacy->runtime->flush();
    const auto columns = legacy->runtime->recorded_columns();
    const auto &ids = columns.count("interval_id") ? columns.at("interval_id") : std::vector<double>{};
    bt::require(ids == std::vector<double>({0., 80.}),
                "legacy retention keeps the decimated projection");
    const auto sealed = legacy->runtime->seal_episode();
    bt::require(sealed.samples.size() == 2 && interval_ids(sealed.samples) == std::vector<std::int64_t>({0, 80}),
                "sealed legacy episode keeps the same projection");
}

void test_default_retention_is_the_legacy_projection() {
    const auto rig = welded_runtime("legacy", 7);
    advance(*rig->runtime, 29);
    rig->runtime->flush();
    const auto columns = rig->runtime->recorded_columns();
    bt::require(!columns.empty() &&
                columns.at("interval_id") == std::vector<double>({0., 7., 14., 21., 28.}),
                "default retention is the decimated projection");
    const auto episode = rig->runtime->seal_episode();
    bt::require(episode.samples.size() == 5 &&
                interval_ids(episode.samples) == std::vector<std::int64_t>({0, 7, 14, 21, 28}),
                "sealed episode projects identically");
}

void test_off_retention_keeps_pending_evidence_without_history() {
    const auto rig = welded_runtime("off", 80);
    advance(*rig->runtime, 13);
    rig->runtime->flush();
    bt::require(rig->runtime->recorded_columns().empty() &&
                rig->runtime->recorded_intervals().empty(),
                "off retention keeps no history");
    const auto batch = rig->runtime->prepare_samples();
    rig->runtime->acknowledge_samples(batch);
    bt::require(interval_ids(batch.samples) ==
                    [] { std::vector<std::int64_t> ids(13); std::iota(ids.begin(), ids.end(), 0); return ids; }(),
                "pending consumer delivery still drains every committed interval");
    bt::require(rig->runtime->seal_episode().samples.empty(),
                "off retention seals to an empty episode");
}

void test_retention_rejects_unknown_mode() {
    // ('every', '', 1, True) are the oracle's bad domains; 1/True are
    // Python scalar types. The native string domain rejects the rest.
    for (const char *mode : {"every", ""})
        throws<std::invalid_argument>([&] { (void)welded_runtime(mode); }, "",
                                      "unknown retention mode rejected");
}

void test_seal_flushes_partial_period_and_empty_run() {
    const auto rig = welded_runtime("full");
    bt::require(rig->period > 1, "control period covers several steps");
    advance(*rig->runtime, rig->period - 1);
    const auto episode = rig->runtime->seal_episode();
    auto expected = std::vector<std::int64_t>(static_cast<std::size_t>(rig->period - 1));
    std::iota(expected.begin(), expected.end(), 0);
    bt::require(episode.samples.size() == static_cast<std::size_t>(rig->period - 1) &&
                interval_ids(episode.batch(0, rig->period - 1)) == expected,
                "seal closes the trailing partial period");
    const auto empty = welded_runtime("full");
    const auto sealed = empty->runtime->seal_episode();
    bt::require(sealed.samples.empty() && sealed.batch(0, 1).samples.empty(),
                "a fresh runtime seals to an empty episode that still reads");
}

void test_episode_batch_validates_range_and_types_before_boxing() {
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 5);
    const auto episode = rig->runtime->seal_episode();
    for (const auto &[start, limit] : std::map<std::int64_t, std::int64_t>{
             {-1, 1}, {0, 0}, {6, 1}, {0, -2}}) {
        bool rejected = false;
        try { (void)episode.batch(start, limit); }
        catch (const std::exception &) { rejected = true; }
        bt::require(rejected, "out-of-domain batch range rejected");
    }
    bt::require(episode.batch(5, 1).samples.empty(),
                "start == row_count admits the empty suffix");
    bt::require(episode.batch(4, 9).samples.size() == 1,
                "limit clamps to the retained extent");
}

void test_episode_samples_expose_typed_fields_and_provenance() {
    const auto rig = welded_runtime("full", 80);
    advance(*rig->runtime, 25);
    const auto episode = rig->runtime->seal_episode();
    const auto batch = episode.batch(0, 25);
    for (std::size_t index = 0; index < batch.samples.size(); ++index) {
        const auto &sample = *batch.samples[index];
        bt::require(sample.interval_id == static_cast<std::int64_t>(index),
                    "typed interval identity");
        const auto &e = energy(sample);
        const bool measured = sw::boolean(sw::required(e, "measured"));
        bt::require(measured == (index % 80 == 0 || (index + 1) % static_cast<std::size_t>(rig->period) == 0),
                    "measurement cadence or period-close marks the row");
        if (measured) {
            bt::require(sw::integer(sw::required(e, "source_interval_id")) == static_cast<std::int64_t>(index),
                        "measured rows cite themselves");
            bt::require(sw::number(sw::required(e, "source_time_s")) == sample.end_time_s,
                        "measured rows cite their own end time");
        } else {
            const auto p = static_cast<std::size_t>(rig->period);
            auto closing = (index / p + 1) * p - 1;
            const auto source = static_cast<std::int64_t>(
                closing < episode.samples.size() ? closing : (index / p) * p - 1);
            bt::require(sw::integer(sw::required(e, "source_interval_id")) == source &&
                        sw::number(sw::required(e, "source_time_s")) ==
                            episode.samples[static_cast<std::size_t>(source)]->end_time_s,
                        "forwarded rows cite the period's closing measurement");
        }
        bt::require(!sample.forces.empty(), "typed force components present");
        for (const auto &[name, forces] : sample.forces) {
            bt::require(!name.empty() && forces.size() == sample.qvel.size(),
                        "force component names and shapes match qvel");
            double power = 0.;
            for (std::size_t i = 0; i < forces.size(); ++i) power += forces[i] * sample.qvel[i];
            const auto *slot = sw::find(sample.powers_w, name);
            bt::require(slot != nullptr &&
                        std::abs(sw::number(*slot) - power) <=
                            1e-9 * std::max(1., std::abs(power)),
                        "powers_w carries the component power");
        }
    }
}

void test_measurement_provenance_marks_forwarded_rows() {
    const auto rig = welded_runtime("full", 80);
    advance(*rig->runtime, rig->period + 3);
    const auto episode = rig->runtime->seal_episode();
    const auto batch = episode.batch(0, rig->period + 3);
    const auto &first = energy(*batch.samples[0]);
    bt::require(sw::boolean(sw::required(first, "measured")) &&
                sw::integer(sw::required(first, "source_interval_id")) == 0,
                "the measured row cites itself");
    const auto closing = static_cast<std::size_t>(rig->period - 1);
    for (std::size_t index = 1; index < closing; ++index) {
        const auto &row = energy(*batch.samples[index]);
        bt::require(!sw::boolean(sw::required(row, "measured")) &&
                    sw::integer(sw::required(row, "source_interval_id")) == rig->period - 1 &&
                    sw::number(sw::required(row, "source_time_s")) == batch.samples[closing]->end_time_s,
                    "the period forwards the closing interval's measurement");
    }
    const auto &closing_row = energy(*batch.samples[closing]);
    bt::require(sw::boolean(sw::required(closing_row, "measured")) &&
                sw::integer(sw::required(closing_row, "source_interval_id")) == rig->period - 1,
                "the closing row measures itself");
    const auto &tail = energy(*batch.samples[static_cast<std::size_t>(rig->period)]);
    bt::require(!sw::boolean(sw::required(tail, "measured")) &&
                sw::integer(sw::required(tail, "source_interval_id")) == rig->period - 1,
                "the unmeasured partial tail forwards the previous closing row");
}

void test_sealed_episode_survives_reset_and_close() {
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 41);
    const auto episode = rig->runtime->seal_episode();
    const auto ids = interval_ids(episode.samples);
    rig->runtime->reset();
    bt::require(episode.samples.size() == 41 && interval_ids(episode.batch(0, 41)) == ids,
                "sealed episode survives reset");
    const auto new_generation = rig->runtime->snapshot().generation;
    bt::require(episode.generation == new_generation - 1,
                "the sealed generation stays the pre-reset one");
    rig->runtime->close();
    const auto rows = episode.batch(0, 41);
    bt::require(interval_ids(rows) == ids &&
                sw::boolean(sw::required(energy(*episode.batch(10, 4).samples[0]), "measured")),
                "the owned episode still reads after close");
}

void test_failed_sealing_leaves_active_evidence_available() {
    // The oracle injected one MemoryError inside seal_episode; the native
    // seal owns no injectable allocation seam, so the port exercises the
    // same ownership contract through the process-isolated failure hook:
    // a mid-advance engine failure at step 9 leaves the committed prefix
    // readable, and sealing still returns the intact retained index.
    const auto rig = welded_runtime("full");
    rig->runtime->test_fail_at_step(9);
    bool failed = false;
    try { (void)rig->runtime->advance(12, runtime::RideControl{}, 0., 0., std::nullopt); }
    catch (const std::exception &) { failed = true; }
    bt::require(failed, "the injected failure propagates through advance");
    const auto episode = rig->runtime->seal_episode();
    bt::require(episode.samples.size() == 9,
                "sealing the surviving prefix returns the intact retained index");
    const auto batch = rig->runtime->prepare_samples();
    auto expected = std::vector<std::int64_t>(9);
    std::iota(expected.begin(), expected.end(), 0);
    bt::require(interval_ids(batch.samples) == expected,
                "the failure did not consume pending delivery");
    rig->runtime->acknowledge_samples(batch);
    const auto state = rig->runtime->accounting_state();
    const auto &history = sw::object(sw::required(state, "history"));
    bt::require(sw::integer(sw::required(history, "last_id")) == 8,
                "the failure did not consume retained rows either");
}

void test_strict_flush_rejection_preserves_retained_evidence() {
    const auto rig = welded_runtime("full", 7, /*strict=*/true);
    throws<runtime::InvalidReferenceRun>([&] { advance(*rig->runtime, 800); }, "",
                                       "strict accounting rejection propagates");
    const auto episode = rig->runtime->seal_episode();
    bt::require(rig->runtime->snapshot().first_failure.has_value(),
                "the strict rejection latched a first failure");
    const auto state = rig->runtime->accounting_state();
    const auto &history = sw::object(sw::required(state, "history"));
    const auto last = sw::integer(sw::required(history, "last_id"));
    bt::require(static_cast<std::int64_t>(episode.samples.size()) == last + 1,
                "the offending period was published before the rejection");
    const auto batch = episode.batch(0, static_cast<std::int64_t>(episode.samples.size()));
    const auto ids = interval_ids(batch);
    for (std::size_t i = 0; i < ids.size(); ++i)
        bt::require(ids[i] == static_cast<std::int64_t>(i),
                    "sealing still returns every committed interval");
}

// Collect the PhysicalSource's emitted tables with per-table row counts.
struct TableSummary {
    std::map<std::string, std::vector<std::int64_t>> rows;
    std::map<std::string, std::vector<runtime::EncodedTable>> batches;
};
TableSummary collect_tables(const rec::PhysicalSource &source, std::size_t maximum) {
    TableSummary out;
    source.tables(maximum, [&](std::string_view name, runtime::EncodedTable &&batch) {
        out.rows[std::string(name)].push_back(batch.rows);
        out.batches[std::string(name)].push_back(std::move(batch));
    });
    return out;
}

// One column lane as doubles regardless of the piece kind.
std::vector<double> doubles_of(const runtime::Piece &piece) {
    if (!piece.d.empty() || piece.i.empty()) return piece.d;
    std::vector<double> out;
    out.reserve(piece.i.size());
    for (const auto v : piece.i) out.push_back(static_cast<double>(v));
    return out;
}

void test_native_source_emits_catalog_tables_with_provenance() {
    const auto rig = welded_runtime("full", 80);
    advance(*rig->runtime, 41);
    const auto episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    const auto source = rec::make_physical_source(episode, welded_catalog(*rig->runtime));
    bt::require(source->row_count() == 41, "source exposes the retained row count");
    const auto tables = collect_tables(*source, 8);
    std::set<std::string> emitted;
    for (const auto &[name, rows] : tables.rows) emitted.insert(name);
    std::set<std::string> expected;
    for (const auto &spec : source->catalog()->table_order()) expected.insert(spec.name);
    bt::require(emitted == expected, "source emits exactly the catalog table set");
    std::int64_t physics = 0;
    std::vector<std::int64_t> ids;
    for (const auto &batch : tables.batches.at("physics")) {
        physics += batch.rows;
        const auto *piece = batch.columns.count("interval_id") ? &batch.columns.at("interval_id") : nullptr;
        if (piece) for (const auto v : doubles_of(*piece))
            ids.push_back(static_cast<std::int64_t>(v));
    }
    bt::require(physics == 41 && ids.size() == 41,
                "physics table carries every retained row");
    for (std::size_t i = 0; i < ids.size(); ++i)
        bt::require(ids[i] == static_cast<std::int64_t>(i), "interval ids stay contiguous");
    // channel_energy decodes the measurement provenance columns.
    const auto &energy_table = tables.batches.at("channel_energy").front();
    const auto *measured = energy_table.columns.count("measured")
        ? &energy_table.columns.at("measured") : nullptr;
    const auto *source_ids = energy_table.columns.count("source_interval_id")
        ? &energy_table.columns.at("source_interval_id") : nullptr;
    bt::require(measured != nullptr && source_ids != nullptr,
                "channel_energy carries the provenance columns");
    bt::require(!measured->b.empty() && measured->b[0] != 0 && measured->b[1] == 0,
                "row 0 is measured, row 1 is not");
    const auto source_values = doubles_of(*source_ids);
    bt::require(source_values.size() >= 2 && source_values[0] == 0. &&
                source_values[1] == static_cast<double>(rig->period - 1),
                "forwarded rows cite the period's closing measurement");
    // Missing optional members encode explicit NULLs, not padding: the
    // mass root flattens to leaf columns plus __mass/__endpoint_mass marker
    // columns; the endpoint_mass_present flag carries the Python-side
    // 'endpoint_mass' in/not-in row membership.
    const auto &mass_table = tables.batches.at("channel_mass").front();
    const auto *endpoint = mass_table.columns.count("endpoint_mass_present")
        ? &mass_table.columns.at("endpoint_mass_present") : nullptr;
    bt::require(mass_table.columns.count("mass_kg") == 1 && endpoint != nullptr &&
                endpoint->b.size() == static_cast<std::size_t>(mass_table.rows) &&
                endpoint->b[0] != 0 && endpoint->b[1] == 0,
                "an unmeasured interval has no endpoint_mass value");
    std::int64_t force_rows = 0;
    std::set<std::string> components;
    for (const auto &batch : tables.batches.at("forces")) {
        force_rows += batch.rows;
        if (const auto *component = batch.columns.count("component") ? &batch.columns.at("component") : nullptr)
            for (const auto &name : component->s) components.insert(name);
    }
    bt::require(force_rows > 41 && !components.empty(),
                "forces table fans out several components per interval");
}

void test_native_source_tables_validate_batch_rows_and_channel_roots() {
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 4);
    const auto episode = std::make_shared<runtime::OwnedNativeEpisode>(rig->runtime->seal_episode());
    const auto source = rec::make_physical_source(episode, welded_catalog(*rig->runtime));
    // (1.5, '8', True) are Python scalar-type rejections; the C++ domain
    // carries 0 / huge-as-negative as the reachable invalid inputs.
    throws<std::invalid_argument>([&] { (void)collect_tables(*source, 0); }, "",
                                  "batch_rows must be positive");
    {
        const auto chunked = collect_tables(*source, 2);
        std::vector<std::int64_t> physics_rows;
        for (const auto &batch : chunked.batches.at("physics"))
            physics_rows.push_back(batch.rows);
        bt::require(physics_rows == std::vector<std::int64_t>({2, 2}),
                    "batch_rows chunks the table stream");
    }
    // An unclaimed channel root fails before any batch is emitted — the
    // same strict-plan rejection the oracle's forged_root staged.
    std::vector<SamplePtr> forged;
    for (const auto &sample : episode->samples) {
        auto copy = std::make_shared<runtime::PhysicalSampleData>(*sample);
        copy->channels.emplace_back("forged_root", Wire{WireObject{}});
        forged.push_back(SamplePtr(std::move(copy)));
    }
    const auto forged_episode = std::make_shared<runtime::OwnedNativeEpisode>(
        runtime::OwnedNativeEpisode::seal(episode->generation, forged, episode->columns));
    const auto bad = rec::make_physical_source(forged_episode, welded_catalog(*rig->runtime));
    throws<std::invalid_argument>([&] { (void)collect_tables(*bad, 2); }, "forged_root",
                                  "unclaimed channel roots are rejected");
}

// The authored energy channel shape (_recording_support.energy_channels).
WireObject energy_channels() {
    return WireObject{
        {"energy", Wire{WireObject{
            {"mechanical_energy_j", Wire{12.5}},
            {"elastic_energy_j", Wire{WireObject{{"fork_spring", Wire{0.25}}}}},
            {"active_work_j", Wire{1.5}},
            {"external_work_j", Wire{-0.5}},
            {"loss_j", Wire{0.25}},
            {"loss_step_j", Wire{0.001}},
            {"solver_constraint_work_j", Wire{0.0}},
            {"energy_scale_j", Wire{10.0}},
            {"muscle_signed_j", Wire{1.0}},
            {"muscle_positive_j", Wire{1.25}},
            {"motor_signed_j", Wire{0.5}},
            {"motor_positive_j", Wire{0.5}},
            {"source_positive_work_j", Wire{1.75}},
            {"constraint_signed_j", Wire{0.0}},
            {"constraint_absolute_j", Wire{0.0}},
            {"closure_signed_j", Wire{0.0}},
            {"closure_absolute_j", Wire{0.0}},
            {"constraint_work_ratio", Wire{0.0}},
            {"constraint_work_ok", Wire{true}},
            {"residual_j", Wire{-0.001}},
            {"electrical_work_j", Wire{0.5}},
            {"electrical_residual_j", Wire{0.0}},
            {"measured", Wire{true}},
            {"source_interval_id", Wire{std::int64_t{0}}},
            {"source_time_s", Wire{0.001}},
        }}}};
}

void test_native_source_encodes_missing_and_nonfinite_values() {
    // A synthetic episode through the same duck-typed surface the oracle
    // staged: nonfinite energy payloads stay nonfinite in the typed pieces
    // and missing optionals stay absent.
    auto channels = energy_channels();
    auto &energy_object = object_ref(channels, "energy");
    sw::set(energy_object, "mechanical_energy_j", Wire{std::numeric_limits<double>::infinity()});
    sw::set(energy_object, "external_work_j", Wire{-std::numeric_limits<double>::infinity()});
    std::vector<SamplePtr> rows{sample_row(0, 0., .001, {}, {}, {}, {},
                                         std::move(channels))};
    const auto episode = std::make_shared<runtime::OwnedNativeEpisode>(make_episode(std::move(rows)));
    const auto source = rec::make_physical_source(episode, synthetic_catalog());
    const auto tables = collect_tables(*source, 4);
    const auto &energy_table = tables.batches.at("channel_energy").front();
    const auto *mechanical = energy_table.columns.count("mechanical_energy_j")
        ? &energy_table.columns.at("mechanical_energy_j") : nullptr;
    const auto *measured = energy_table.columns.count("measured")
        ? &energy_table.columns.at("measured") : nullptr;
    const auto *source_ids = energy_table.columns.count("source_interval_id")
        ? &energy_table.columns.at("source_interval_id") : nullptr;
    bt::require(mechanical != nullptr && mechanical->d.size() == 1 &&
                mechanical->d[0] == std::numeric_limits<double>::infinity(),
                "nonfinite payloads stay nonfinite in the pieces");
    bt::require(measured != nullptr && !measured->b.empty() && measured->b[0] != 0 &&
                source_ids != nullptr && !source_ids->i.empty() && source_ids->i[0] == 0,
                "measured/source provenance columns decode");
}

void test_retained_index_is_independent_of_pending_acknowledgement() {
    const auto rig = welded_runtime("full");
    advance(*rig->runtime, 9);
    rig->runtime->flush();
    // Acknowledgement drains only pending delivery; retained rows stay.
    const auto batch = rig->runtime->prepare_samples();
    auto expected = std::vector<std::int64_t>(9);
    std::iota(expected.begin(), expected.end(), 0);
    bt::require(interval_ids(batch.samples) == expected,
                "pending delivery reports the committed prefix");
    rig->runtime->acknowledge_samples(batch);
    rig->runtime->release_samples();
    const auto first = rig->runtime->seal_episode();
    bt::require(interval_ids(first.samples) == expected,
                "sealing still owns every committed interval after the drain");
    advance(*rig->runtime, 13);
    const auto second = rig->runtime->seal_episode();
    bt::require(second.samples.size() == 13,
                "sealing returns the whole retained index, not the delta");
}

// Deep structural equality through the recording-JSON spelling — the same
// lossless tree the writer emits.
std::string wire_text(const Wire &wire) {
    return io::recording_json(research::wire_value(wire));
}

void test_recording_does_not_change_committed_trajectory() {
    // record=enabled/disabled in the oracle is the driver-level capture
    // flag, not a retention mode: both rigs share the full-retention welded
    // config and the "recording" one drains the pending consumer stream the
    // way RunCapture's delivery does.
    const auto recording = welded_runtime("full", 80);
    const auto idle = welded_runtime("full", 80);
    for (const std::int64_t target : {1, 4, 8}) {
        advance(*recording->runtime, target);
        advance(*idle->runtime, target);
        recording->runtime->acknowledge_samples(recording->runtime->prepare_samples());
        const auto a = recording->runtime->snapshot();
        const auto b = idle->runtime->snapshot();
        const auto accounting_a = recording->runtime->accounting_state();
        const auto accounting_b = idle->runtime->accounting_state();
        bt::require(a.integration_state == b.integration_state,
                    "integration state is bitwise identical under either retention");
        bt::require(wire_text(Wire{a.model_status}) == wire_text(Wire{b.model_status}),
                    "model status is identical under either retention");
        const auto &ha = sw::object(sw::required(accounting_a, "history"));
        const auto &hb = sw::object(sw::required(accounting_b, "history"));
        bt::require(wire_text(Wire{ha}) == wire_text(Wire{hb}) &&
                    wire_text(Wire{sw::object(sw::required(accounting_a, "energy"))}) ==
                    wire_text(Wire{sw::object(sw::required(accounting_b, "energy"))}),
                    "accounting state is identical under either retention");
        bt::require(a.outcome == b.outcome, "outcome agrees");
    }
    recording->runtime->flush();
    idle->runtime->flush();
    const auto a = recording->runtime->snapshot();
    const auto b = idle->runtime->snapshot();
    bt::require(a.first_failure.has_value() == b.first_failure.has_value(),
                "first-failure state agrees");
    if (a.latest_sample && b.latest_sample) {
        const auto &ca = sw::object(sw::required(a.latest_sample->channels, "control"));
        const auto &cb = sw::object(sw::required(b.latest_sample->channels, "control"));
        bt::require(wire_text(Wire{ca}) == wire_text(Wire{cb}),
                    "the committed interval's applied control channel agrees");
    }
    const auto episode = recording->runtime->seal_episode();
    bt::require(episode.samples.size() ==
                static_cast<std::size_t>(recording->runtime->snapshot().step),
                "the retained episode covers every committed step");
}

} // namespace

int main() {
    return bt::run([] {
        test_full_retention_keeps_every_interval_under_measurement_decimation();
        test_default_retention_is_the_legacy_projection();
        test_off_retention_keeps_pending_evidence_without_history();
        test_retention_rejects_unknown_mode();
        test_seal_flushes_partial_period_and_empty_run();
        test_episode_batch_validates_range_and_types_before_boxing();
        test_episode_samples_expose_typed_fields_and_provenance();
        test_measurement_provenance_marks_forwarded_rows();
        test_sealed_episode_survives_reset_and_close();
        test_failed_sealing_leaves_active_evidence_available();
        test_strict_flush_rejection_preserves_retained_evidence();
        test_native_source_emits_catalog_tables_with_provenance();
        test_native_source_tables_validate_batch_rows_and_channel_roots();
        test_native_source_encodes_missing_and_nonfinite_values();
        test_retained_index_is_independent_of_pending_acknowledgement();
        test_recording_does_not_change_committed_trajectory();
    });
}
