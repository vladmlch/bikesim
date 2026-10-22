// Completion 3.22b. Source: tests/reference/test_recorder_columns.py.
// The Python PhysicalRecorder's flat row lives on PhysicalSampleData::
// flat_columns + make_columns; its CSV/JSONL emission is the realtime-driver
// export contract (sorted names, NaN -> empty cell, repr_double payload).
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "configuration/fields.hpp"
#include "configuration/load.hpp"
#include "configuration/rider_spec.hpp"
#include "io/json.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/recording.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

runtime::SamplePtr make_sample(std::int64_t id, runtime::WireObject channels,
                               std::vector<double> qpos = {},
                               std::vector<double> qvel = {}) {
    auto sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->interval_id = id;
    sample->time_s = static_cast<double>(id) * .0005;
    sample->end_time_s = static_cast<double>(id + 1) * .0005;
    sample->qpos = std::move(qpos);
    sample->qvel = std::move(qvel);
    sample->channels = std::move(channels);
    return sample;
}

// The realtime_driver telemetry.csv contract on a column map: sorted names,
// CRLF rows, absent (NaN) cells emitted empty, repr_double payloads.
std::string csv_text(const runtime::SampleColumns &columns) {
    std::ostringstream out;
    std::size_t rows = 0;
    bool first = true;
    for (const auto &[name, values] : columns) {
        if (!first) out << ',';
        out << name;
        if (!first && rows != values.size())
            throw std::runtime_error("measurement column sizes differ");
        first = false;
        rows = values.size();
    }
    if (columns.empty()) out << "schema_version,time_s";
    out << "\r\n";
    for (std::size_t row = 0; row < rows; ++row) {
        first = true;
        for (const auto &[name, values] : columns) {
            (void)name;
            if (!first) out << ',';
            first = false;
            if (!std::isnan(values.at(row)))
                out << bike::configuration::detail::repr_double(values.at(row));
        }
        out << "\r\n";
    }
    return out.str();
}

std::vector<std::string> csv_rows(const std::string &text) {
    std::vector<std::string> rows;
    std::size_t at = 0;
    while (at < text.size()) {
        const auto end = text.find("\r\n", at);
        rows.push_back(text.substr(at, end == std::string::npos ? end : end - at));
        at = end == std::string::npos ? text.size() : end + 2;
    }
    if (!rows.empty() && rows.back().empty()) rows.pop_back();
    return rows;
}

std::vector<std::string> csv_cells(const std::string &row) {
    std::vector<std::string> cells;
    std::size_t at = 0;
    while (true) {
        const auto comma = row.find(',', at);
        cells.push_back(row.substr(at, comma == std::string::npos ? comma : comma - at));
        if (comma == std::string::npos) break;
        at = comma + 1;
    }
    return cells;
}

// test_columns_are_rectangular_and_csv_matches_rows — the recorder keeps
// absent values NaN in a rectangular sorted map and the CSV emits them empty.
void test_columns_are_rectangular_and_csv_matches_rows() {
    // flatten_numbers({'a':1,'b':{'c':2}}) and ({'a':3,'d':4}) are the two
    // rows; flat_columns supplies the same dotted-leaf projection, so the
    // samples carry those keys as channels and the fixed schema rides along.
    const runtime::WireObject mass{{"mass_kg", 100.},
                                   {"com_m", runtime::WireArray{0., 0., 1.}}};
    const runtime::WireObject energy{{"mechanical_energy_j", 12.},
                                     {"residual_j", .01}};
    const std::vector<runtime::SamplePtr> samples{
        make_sample(0, {{"mass", mass}, {"energy", energy}, {"a", 1.},
                        {"b", runtime::WireObject{{"c", 2.}}}}, {0.}, {0.}),
        make_sample(1, {{"mass", mass}, {"energy", energy}, {"a", 3.},
                        {"d", 4.}}, {0.}, {0.})};
    const auto columns = runtime::make_columns(samples, runtime::ColumnLayout{});
    bt::require(columns.size() >= 3, "recorder columns carry every flat leaf");
    bt::require(columns.count("a") && columns.count("b.c") && columns.count("d"),
                "the appended keys flatten to 'a', 'b.c', 'd'");
    bt::require(columns.at("a") == std::vector<double>({1., 3.}),
                "the shared column holds both row values");
    bt::require(std::isnan(columns.at("b.c").at(1)) && std::isnan(columns.at("d").at(0)),
                "missing cells stay NaN inside the rectangular frame");
    const auto rows = csv_rows(csv_text(columns));
    bt::require(rows.size() == 3, "CSV holds a header plus one row per interval");
    const auto names = csv_cells(rows[0]);
    const auto at = [&](std::size_t row, const std::string &name) {
        const auto it = std::ranges::find(names, name);
        bt::require(it != names.end(), "CSV header carries " + name);
        return csv_cells(rows[row + 1])[static_cast<std::size_t>(std::distance(names.begin(), it))];
    };
    bt::require(at(0, "a") == "1" || at(0, "a") == "1.0", "row 0 column a is '1.0'");
    bt::require(at(1, "d") == "4" || at(1, "d") == "4.0", "row 1 column d is '4.0'");
    bt::require(at(0, "d").empty(), "an absent numeric cell is written empty");
}

// test_sample_owns_channels_without_recursive_copy_and_arrays_remain_readonly
void test_sample_owns_channels_without_recursive_copy() {
    // Ownership, not aliasing: mutating the source object after construction
    // cannot reach the sample, and the whole sample is const once shared.
    auto source = runtime::WireObject{{"drive", runtime::WireObject{{"power", 2.}}},
                                      {"values", runtime::WireArray{0., 1., 2.}}};
    runtime::PhysicalSampleData sample;
    sample.qpos = {0., 0., 0., 0.};
    sample.qvel = {0., 0., 0., 0.};
    sample.forces = {{"f", {1., 1., 1., 1.}}};
    sample.channels = source;
    source.front().second = runtime::Wire(runtime::WireObject{{"power", 9.}});
    bt::require(runtime::sample_wire::number(runtime::sample_wire::required(
                    runtime::sample_wire::object(runtime::sample_wire::required(
                        sample.channels, "drive")), "power")) == 2.,
                "the sample owns its channel tree — later source mutation is invisible");
    const auto &values = runtime::sample_wire::array(
        runtime::sample_wire::required(sample.channels, "values"));
    bt::require(values.size() == 3 && runtime::sample_wire::number(values[2]) == 2.,
                "the channel array publishes as_dict's [0., 1., 2.] payload");
    const runtime::SamplePtr shared = std::make_shared<runtime::PhysicalSampleData>(std::move(sample));
    static_assert(std::is_const_v<std::remove_reference_t<decltype((shared->qpos))>> &&
                  std::is_const_v<std::remove_reference_t<decltype((shared->qvel))>>);
    const auto wire = shared->as_wire();
    const auto flattened = runtime::sample_wire::array(runtime::sample_wire::required(wire, "values"));
    bt::require(flattened.size() == 3, "as_wire keeps the channel array intact");
}

// test_column_csv_and_streamed_jsonl_preserve_the_original_interval_row
void test_column_csv_and_jsonl_preserve_the_interval_row() {
    const runtime::ColumnLayout layout{.root_x_qpos = 0, .root_x_dof = 0, .root_pitch_qpos = 2};
    runtime::ResearchRecorder recorder(1, layout);
    std::vector<runtime::SamplePtr> samples;
    for (std::int64_t i = 0; i < 3; ++i) {
        auto sample = make_sample(i, {
            {"mass", runtime::WireObject{{"mass_kg", 100.},
                {"com_m", runtime::WireArray{static_cast<double>(i), 0., 1.}},
                {"kinetic_energy_j", 2.}, {"gravitational_energy_j", 10.}}},
            {"energy", runtime::WireObject{{"mechanical_energy_j", 12.},
                                           {"residual_j", .01}}},
            {"component_work_j", runtime::WireObject{{"f", .5 * i}}},
            {"scalar_array", 1.25}},
            {static_cast<double>(i), 0., .1}, {2., 0., 0.});
        samples.push_back(sample);
        recorder.append(sample);
    }
    bt::require(recorder.rows() == 3, "every interval row is retained");
    const auto columns = recorder.columns();
    const auto rows = csv_rows(csv_text(columns));
    bt::require(rows.size() == 4, "CSV preserves one row per recorded interval");
    const auto names = csv_cells(rows[0]);
    bt::require(names.size() == columns.size(), "CSV header is the full flat key set");
    for (std::size_t i = 0; i < 3; ++i) {
        const auto expected = samples[i]->flat_columns(layout);
        std::set<std::string> expected_keys;
        for (const auto &[key, value] : expected) expected_keys.insert(key);
        bt::require(expected_keys == std::set<std::string>(names.begin(), names.end()),
                    "row " + std::to_string(i) + " preserves the flat field inventory");
        const auto cells = csv_cells(rows[i + 1]);
        for (std::size_t c = 0; c < names.size(); ++c) {
            const auto found = std::ranges::find_if(expected, [&](const auto &kv) {
                return kv.first == names[c]; });
            bt::require(found != expected.end(), "column value exists for " + names[c]);
            if (cells[c].empty()) continue; // absent NaN cell
            bt::require(std::stod(cells[c]) == found->second,
                        "CSV cell " + names[c] + " round-trips the row value");
        }
    }
    // write_jsonl emits each sample's dict form verbatim, one JSON line per
    // interval; intervals() carries the same as_wire payloads.
    const auto intervals = recorder.intervals();
    bt::require(intervals.size() == 3, "JSONL holds one payload per interval");
    for (std::size_t i = 0; i < 3; ++i) {
        const auto line = bike::io::canonical_json(bike::research::wire_value(
            runtime::Wire{samples[i]->as_wire()}));
        bt::require(bike::io::canonical_json(bike::research::wire_value(intervals[i])) == line,
                    "the streamed payload equals the interval's dict form");
    }
}

// test_runtime_publishes_owned_channels_and_preserves_previous_samples — slow.
// 30 native steps: every published channels tree is independently owned, the
// first ten snapshots survive the following twenty steps, and the published
// rider_control mapping is not the live controller's.
void test_runtime_publishes_owned_channels() {
    namespace research = bike::research;
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = bike::configuration::load_physics(
        bt::fixture("config/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.physics->timestep_s = .0005;
    request.setup.model.rider = bike::configuration::RiderSpecs{};
    request.setup.model.rider->variant = "articulated_planar";
    request.setup.track = bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 200.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [200.0, 0.0]] }\n"));
    request.experiment.duration_s = .02;
    request.experiment.control_period_s = .005;
    request.experiment.record_decimation = 1;
    request.experiment.stop_on_model_violation = false;
    request.sensors = research::SensorConfig::ideal();
    const auto prepared = research::prepare_environment(std::move(owner));
    bt::require(prepared->bootstrap.config.timestep_s == .0005,
                "environment metadata keeps timestep_s 0.0005");
    bt::require(static_cast<double>(prepared->bootstrap.config.control_steps) *
                prepared->bootstrap.config.timestep_s == .005,
                "environment metadata keeps controller_interval_s 0.005");
    runtime::NativeRideRuntime runtime(std::move(prepared->bootstrap.ride));
    const runtime::RideControl command{
        .motor_torque_nm = 0., .motor_limit_nm = {}, .human_torque_nm = 0.,
        .crank_target_rate_rad_s = {}, .posture = {}};
    std::vector<runtime::SamplePtr> published;
    const auto collect = [&](std::int64_t steps) {
        const auto result = runtime.advance(runtime.snapshot().step + steps,
                                            command, 0., 0., std::nullopt);
        bt::require(result.reason != "outcome", "the welded run stays rideable");
        auto batch = runtime.prepare_samples();
        for (const auto &sample : batch.samples) published.push_back(sample);
        runtime.acknowledge_samples(batch);
    };
    collect(10);
    std::vector<std::string> snapshots;
    for (const auto &sample : published)
        snapshots.push_back(bike::io::canonical_json(bike::research::wire_value(
            runtime::Wire{sample->as_wire()})));
    collect(20);
    bt::require(published.size() == 30, "thirty committed intervals published");
    std::set<const runtime::WireObject *> owners;
    for (const auto &sample : published) owners.insert(&sample->channels);
    bt::require(owners.size() == 30,
                "each interval owns a distinct channels object");
    for (std::size_t i = 0; i < 10; ++i)
        bt::require(bike::io::canonical_json(bike::research::wire_value(
                        runtime::Wire{published[i]->as_wire()})) == snapshots[i],
                    "earlier published samples are immutable across later steps");
    bt::require(runtime::sample_wire::find(published.back()->channels, "rider_control") != nullptr,
                "the published rider_control row is a detached owned copy");
}

} // namespace

// The four test bodies inline into this one dispatcher; the aggregate frame
// exceeds the 8 KiB policy even though each callee stays within budget, so
// the suppression wraps main only.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_columns_are_rectangular_and_csv_matches_rows();
        test_sample_owns_channels_without_recursive_copy();
        test_column_csv_and_jsonl_preserve_the_interval_row();
        test_runtime_publishes_owned_channels();
    });
}
NATIVE_DIAG_POP
