// Completion 3.31 (T53). Source: tests/reference/test_recording_analysis.py.
//
// Analyst surface over a sealed run: the recording JSON boundary, bounded
// queries/exports, windows, field selection, exact/summary comparison and
// schema-compatibility refusals — the same Value trees the `bike-recording`
// CLI prints. The oracle's 16th record
// (test_exact_comparison_rejects_incompatible_units_and_phases) is dropped:
// units/phases derive from joint_type inside build_catalog with no public
// seam to author divergent field metadata between two sealed recordings,
// so the schema check can only be exercised through coordinate identity
// natively. Ledger disposition records the substitution.
#include "recording_test_support.hpp"
#include "recording/export.hpp"
#include "recording/publish.hpp"
#include "recording/report.hpp"
#include "cli/recording.hpp"
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

namespace {

using namespace recording_test;
namespace detail = rec::detail;
using Object = io::Value::Object;
using Array = io::Value::Array;
using rec::Recording;

constexpr double kDt = 1. / 8;

std::vector<SamplePtr> rows(std::size_t count = 3, double offset = 0.,
                            bool nan = false) {
    std::vector<SamplePtr> out;
    for (std::size_t i = 0; i < count; ++i)
        out.push_back(sample_row(
            static_cast<std::int64_t>(i), static_cast<double>(i) * kDt + offset,
            kDt, {}, {}, {}, {},
            energy_channels(WireObject{
                {"residual_j",
                 Wire{nan ? std::numeric_limits<double>::quiet_NaN()
                          : static_cast<double>(i) * .1}}})));
    return out;
}

std::filesystem::path seal(bt::TemporaryDirectory &dir,
                           std::vector<SamplePtr> samples,
                           std::string_view name = "run",
                           SyntheticCapture options = {}) {
    options.episodes = {std::move(samples)};
    auto capture = synthetic_capture({}, std::move(options));
    return rec::finalize(*capture, dir.path() / name).directory;
}

const Value &member_at(const Value &object, std::string_view name) {
    return detail::field(object, name);
}
const Array &members(const Value &object, std::string_view name) {
    return io::as_array(member_at(object, name), "member");
}
const Value &row_at(const Value &result, std::size_t index) {
    return members(result, "rows")[index];
}
std::int64_t field_int(const Value &object, std::string_view name) {
    return io::as_int64(member_at(object, name), "member");
}
double field_real(const Value &object, std::string_view name) {
    return io::tagged_double(member_at(object, name), "member");
}
bool field_bool(const Value &object, std::string_view name) {
    return io::as_bool(member_at(object, name), "member");
}
std::string field_str(const Value &object, std::string_view name) {
    return io::as_string(member_at(object, name), "member");
}

void test_recording_json_boundary_round_trips_typed_literals() {
    const std::string tag{io::kRecordingTag};
    Value payload{Object{
        {"nan", rec::value::real(std::numeric_limits<double>::quiet_NaN())},
        {"positive",
         rec::value::real(std::numeric_limits<double>::infinity())},
        {"negative",
         rec::value::real(-std::numeric_limits<double>::infinity())},
        {"zero", rec::value::real(-0.)},
        {"null", Value{}},
        {"exists", Value{false}},
        {"literal", Value{Object{{tag, text("float64")},
                                 {"value", text("nan")}}}},
        {"decimal", Value{io::Tagged{"decimal", {{"value", "1.2300"}}}}},
        {"timestamp",
         Value{io::Tagged{"datetime", {{"value", "2026-01-01T12:00:00+00:00"}}}}},
    }};
    const auto document = io::recording_json(payload);
    const auto result = io::parse_recording_json(document);
    bt::require(std::isnan(field_real(result, "nan")) &&
                field_real(result, "positive") ==
                    std::numeric_limits<double>::infinity() &&
                field_real(result, "negative") ==
                    -std::numeric_limits<double>::infinity(),
                "tagged float64 sentinels round-trip");
    bt::require(field_real(result, "zero") == 0. && std::signbit(field_real(result, "zero")),
                "signed zero survives the JSON boundary");
    bt::require(io::is_null(member_at(result, "null")) &&
                field_bool(result, "exists") == false,
                "null and false stay distinct kinds");
    const auto &literal = io::as_object(member_at(result, "literal"), "l");
    bt::require(literal.size() == 2 &&
                io::as_string(detail::field(literal, tag), "k") == "float64" &&
                io::as_string(detail::field(literal, "value"), "v") == "nan",
                "a reserved-tag object escapes and decodes literally");
    const auto &decimal = member_at(result, "decimal");
    const auto *decimal_tag = std::get_if<io::Tagged>(&decimal.data);
    bt::require(decimal_tag && decimal_tag->kind == "decimal" &&
                decimal_tag->fields.size() == 1 &&
                decimal_tag->fields[0] ==
                    io::Tagged::Field{"value", "1.2300"},
                "decimal keeps its exact authored lexeme");
    const auto *timestamp =
        std::get_if<io::Tagged>(&member_at(result, "timestamp").data);
    bt::require(timestamp && timestamp->kind == "datetime" &&
                timestamp->fields.size() == 1 &&
                timestamp->fields[0] ==
                    io::Tagged::Field{"value", "2026-01-01T12:00:00+00:00"},
                "timezone-aware datetime canonicalizes through the tag");
    for (const char *malformed : {R"({"x":1,"x":2})", R"({"x":NaN})",
                                  R"({"x":1e9999})"})
        throws<std::invalid_argument>(
            [&] { (void)io::parse_recording_json(malformed); }, "",
            "malformed recording JSON is refused at the boundary");
}

void test_window_returns_interval_overlap_and_selected_fields() {
    bt::TemporaryDirectory dir("analysis-window");
    const auto reader = sealed(seal(dir, rows()));
    const auto window = rec::window(*reader, 0, .0625, .2,
                                    std::optional<std::string>(
                                        "physics.time_s,physics.qvel"));
    const auto &selected = members(window, "rows");
    bt::require(selected.size() == 1 &&
                field_int(selected[0], "interval_id") == 1,
                "the window selects the interval opening inside it");
    bt::require(field_bool(selected[0], "overlaps_end"),
                "the selected interval reports its partial end overlap");
    bt::require(io::as_array(member_at(selected[0], "physics.qvel"), "q").size()
                    == 8,
                "selected vector fields keep their shape");
    const auto &before = members(member_at(window, "overlap_before_start"),
                                 "rows");
    bt::require(before.size() == 1 &&
                field_int(before[0], "interval_id") == 0,
                "overlap_before_start names the interval open at start");
    const auto identity = rec::window(
        *reader, 0, .125, .25, std::optional<std::string>("physics.interval_id"));
    bt::require(field_int(members(identity, "rows")[0], "physics.interval_id") ==
                    1,
                "field selection re-projects per call");
}

void test_bounded_query_and_export_report_truncation_and_row_totals() {
    bt::TemporaryDirectory dir("analysis-bounded");
    const auto path = seal(dir, rows(211));
    const auto reader = sealed(path);
    const auto bounded = reader->query(
        "SELECT interval_id FROM physics ORDER BY interval_id");
    bt::require(members(bounded, "rows").size() == 200 &&
                field_bool(bounded, "truncated") && field_bool(bounded, "has_more") &&
                field_int(bounded, "limit") == 200,
                "bounded queries cap and flag truncation");
    const auto destination = dir.path() / "out.jsonl";
    const auto exported =
        rec::export_data(*reader, 0, "telemetry", "jsonl", destination);
    bt::require(field_int(exported, "rows") == 211,
                "export reports the unbounded row total");
    std::size_t lines = 0;
    {   std::ifstream in(destination);
        std::string line;
        while (std::getline(in, line)) ++lines; }
    bt::require(lines == 211, "every exported row is one JSONL line");
    throws<std::exception>(
        [&] { (void)rec::export_data(*reader, 0, "telemetry", "jsonl",
                                     destination); },
        "", "exclusive creation refuses an existing export");
    // The oracle's trailing main(['sql', path, '--query', ...]) leg: the same
    // bounded row tree through the analyst console's JSON envelope.
    std::ostringstream out, error;
    const std::string path_text = path.string();
    const std::vector<std::string_view> argv{
        "sql", path_text, "--query", "SELECT CAST(1.25 AS DOUBLE) AS amount",
        "--json"};
    bt::require(bike::cli::recording_command(argv, out, error) == 0,
                "the analyst sql subcommand accepts a sealed path");
    const auto emitted = io::parse_recording_json(out.str());
    bt::require(members(emitted, "rows").size() == 1 &&
                field_real(row_at(emitted, 0), "amount") == 1.25,
                "the JSON envelope decodes the scalar row");
}

void test_sql_cannot_mutate_through_the_reader() {
    bt::TemporaryDirectory dir("analysis-sql");
    const auto reader = sealed(seal(dir, rows()));
    for (const char *sql : {
             "DELETE FROM physics",
             "INSERT INTO physics DEFAULT VALUES",
             "UPDATE physics SET interval_id=0",
             "DROP TABLE physics",
             "ALTER TABLE physics ADD COLUMN extra BIGINT",
             "CREATE TABLE rogue(id BIGINT)",
             "ATTACH 'other.db' AS other",
             "PRAGMA writable_schema=ON",
         })
        throws<std::exception>([&] { (void)reader->query(sql); }, "",
                               "mutating SQL never reaches storage");
}

void test_exact_comparison_refuses_mismatched_clocks_before_values() {
    bt::TemporaryDirectory dir("analysis-clock");
    const auto left = sealed(seal(dir, rows(), "a"));
    const auto right = sealed(seal(dir, rows(3, .125), "b"));
    const auto result = rec::compare(*left, *right, 0, 0);
    bt::require(!field_bool(result, "compatible") && !field_bool(result, "agrees"),
                "mismatched clocks refuse agreement");
    const auto &reasons = members(result, "incompatibilities");
    bt::require(!reasons.empty() &&
                field_str(reasons[0], "reason") == "time_grid",
                "the first incompatibility names the time grid");
    const auto summary = rec::compare(*left, *right, 0, 0, "summary");
    bt::require(field_str(summary, "alignment") == "none" &&
                field_int(member_at(summary, "left"), "physical_rows") == 3 &&
                field_int(member_at(summary, "right"), "physical_rows") == 3,
                "summary mode reports both extents without alignment");
}

void test_matching_nonfinite_values_are_agreeing_invalid_evidence() {
    bt::TemporaryDirectory dir("analysis-nonfinite");
    const auto left = sealed(seal(dir, rows(3, 0., true), "a"));
    const auto right = sealed(seal(dir, rows(3, 0., true), "b"));
    const auto result = rec::compare(
        *left, *right, 0, 0, "exact",
        std::optional<std::string>("channel_energy.residual_j"));
    bt::require(field_bool(result, "compatible") && field_bool(result, "agrees") &&
                field_int(result, "matching_invalid_pairs") == 3 &&
                field_str(result, "interpretation").find("INVALID") !=
                    std::string::npos,
                "matching NaN pairs agree as invalid evidence");
}

void test_report_excludes_forwarded_energy_extrema() {
    bt::TemporaryDirectory dir("analysis-report");
    std::vector<SamplePtr> samples;
    samples.push_back(sample_row(0, 0., kDt, {}, {}, {}, {},
                                 energy_channels(
                                     WireObject{{"residual_j", Wire{.1}}})));
    samples.push_back(sample_row(
        1, kDt, kDt, {}, {}, {}, {},
        energy_channels(WireObject{{"residual_j", Wire{9999999.}},
                                   {"measured", Wire{false}}})));
    const auto path = seal(dir, std::move(samples));
    const auto summary = bt::text(path / "summary.md");
    const auto line_at = summary.find("channel_energy`.residual_j");
    bt::require(line_at != std::string::npos,
                "the extrema table lists channel_energy.residual_j");
    const auto line_end = summary.find('\n', line_at);
    bt::require(summary.substr(line_at, line_end - line_at)
                        .find("9999999") == std::string::npos,
                "forwarded (unmeasured) extrema stay out of the report");
    bt::require(summary.find("measured IS TRUE") != std::string::npos,
                "the report documents the measured-domain filter");
}

void test_exact_comparison_reports_first_mismatch_with_stable_keys() {
    bt::TemporaryDirectory dir("analysis-mismatch");
    auto right_rows = rows();
    // right row 1: residual .1 + 1e-6 — beyond atol=rtol=1e-9.
    auto shifted = energy_channels(
        WireObject{{"residual_j", Wire{.1 + 1e-6}}});
    right_rows[1] = sample_row(1, kDt, kDt, {}, {}, {}, {}, std::move(shifted));
    const auto left = sealed(seal(dir, rows(), "a"));
    const auto right = sealed(seal(dir, std::move(right_rows), "b"));
    const auto result = rec::compare(
        *left, *right, 0, 0, "exact",
        std::optional<std::string>("channel_energy.residual_j"));
    bt::require(field_bool(result, "compatible") && !field_bool(result, "agrees"),
                "a single drifted value disagrees without incompatibility");
    const auto &first = member_at(result, "first_mismatch");
    bt::require(field_str(first, "kind") == "float" &&
                field_str(first, "path") == "channel_energy.residual_j" &&
                field_int(first, "interval_id") == 1 &&
                field_real(first, "time_s") == .125 &&
                field_real(first, "left") == .1 &&
                std::abs(field_real(first, "right") - (.1 + 1e-6)) < 1e-15,
                "first_mismatch keeps stable keys and authored values");
    bt::require(field_int(result, "mismatched_field_rows") == 1 &&
                field_int(result, "compared_intervals") == 3 &&
                field_str(result, "tolerance_rule") ==
                    "abs(a-b) <= atol + rtol*max(abs(a),abs(b))",
                "tolerance accounting stays named in the result");
}

void test_exact_comparison_reports_prefix_extent_mismatch() {
    bt::TemporaryDirectory dir("analysis-extent");
    const auto left = sealed(seal(dir, rows(3), "a"));
    const auto right = sealed(seal(dir, rows(2), "b"));
    const auto result = rec::compare(*left, *right, 0, 0);
    bt::require(field_int(result, "compared_intervals") == 2,
                "the shared prefix compares fully");
    const auto &extents = member_at(result, "unmatched_extents");
    bt::require(field_int(extents, "left_intervals") == 1 &&
                field_int(extents, "right_intervals") == 0,
                "unmatched extents count per side");
    const auto &first = member_at(extents, "first");
    bt::require(field_int(first, "left_first_interval") == 2 &&
                io::is_null(member_at(first, "right_first_interval")),
                "the first unmatched interval names the left tail");
    bt::require(io::is_null(member_at(result, "first_mismatch")) &&
                !field_bool(result, "agrees"),
                "extent mismatch disagrees without a field diff");
}

void test_exact_comparison_rejects_incompatible_dof_coordinate_identity() {
    bt::TemporaryDirectory dir("analysis-coords");
    auto renamed = synthetic_joints();
    renamed[0].name = "renamed_" + renamed[0].name;
    const auto other =
        rec::build_catalog(renamed, std::span<const rec::ScalarAlias>{});
    const auto left = sealed(seal(dir, rows(), "a"));
    SyntheticCapture options;
    options.catalog = &other;
    const auto right = sealed(seal(dir, rows(), "b", std::move(options)));
    const auto result = rec::compare(*left, *right, 0, 0);
    const auto &reasons = members(result, "incompatibilities");
    bool coordinate_identity = false;
    for (const auto &entry : reasons)
        coordinate_identity = coordinate_identity ||
            field_str(entry, "reason") == "coordinate_identity";
    bt::require(coordinate_identity &&
                field_int(result, "compared_intervals") == 0,
                "renamed DOF identity refuses the comparison");
}

void test_exact_comparison_rejects_differing_measurement_provenance() {
    bt::TemporaryDirectory dir("analysis-provenance");
    auto right_rows = rows();
    right_rows[2] = sample_row(
        2, 2 * kDt, kDt, {}, {}, {}, {},
        energy_channels(WireObject{{"residual_j", Wire{.2}},
                                   {"measured", Wire{false}},
                                   {"source_interval_id", Wire{std::int64_t{1}}},
                                   {"source_time_s", Wire{.25}}}));
    const auto left = sealed(seal(dir, rows(), "a"));
    const auto right = sealed(seal(dir, std::move(right_rows), "b"));
    const auto result = rec::compare(*left, *right, 0, 0);
    const auto &reasons = members(result, "incompatibilities");
    bt::require(reasons.size() == 1 &&
                field_str(reasons[0], "reason") == "measurement_provenance" &&
                field_int(reasons[0], "interval_id") == 2,
                "forwarded provenance is an incompatibility, not a diff");
}

void test_exact_comparison_numeric_tolerance_boundary() {
    bt::TemporaryDirectory dir("analysis-tolerance");
    const std::array<double, 3> deltas{5e-10, 1e-9 / (1. - 1e-9), 1e-8};
    const std::array<bool, 3> expected{true, true, false};
    const auto left = sealed(seal(dir, rows(), "a"));
    for (std::size_t k = 0; k < 3; ++k) {
        auto right_rows = rows();
        right_rows[0] = sample_row(
            0, 0., kDt, {}, {}, {}, {},
            energy_channels(WireObject{{"residual_j", Wire{deltas.at(k)}}}));
        const auto right = sealed(
            seal(dir, std::move(right_rows), "b" + std::to_string(k)));
        const auto result = rec::compare(
            *left, *right, 0, 0, "exact",
            std::optional<std::string>("channel_energy.residual_j"),
            1e-9, 1e-9);
        bt::require(field_bool(result, "agrees") == expected.at(k),
                    "the |a-b| <= atol + rtol*max boundary is exact");
        const auto &deviation =
            members(member_at(result, "deviations"), "rows");
        bool found = false;
        for (const auto &row : deviation)
            if (field_str(row, "field") == "channel_energy.residual_j") {
                found = true;
                bt::require(std::abs(field_real(row, "max_absolute_error") -
                                     deltas[k]) < 1e-20,
                            "deviations report the observed absolute error");
            }
        bt::require(found, "deviations cover the compared field");
        const auto &first = member_at(result, "first_mismatch");
        if (expected.at(k))
            bt::require(io::is_null(first), "inside tolerance: no mismatch");
        else
            bt::require(field_str(first, "kind") == "float" &&
                        field_int(first, "interval_id") == 0 &&
                        std::abs(field_real(first, "absolute_error") - deltas.at(k)) <
                            1e-20,
                        "outside tolerance: a float mismatch on interval 0");
    }
}

void test_exact_comparison_distinguishes_null_from_missing() {
    bt::TemporaryDirectory dir("analysis-presence");
    auto left_energy = energy_channels(
        WireObject{{"residual_j", Wire{nullptr}}});
    auto right_energy = energy_channels();
    wire_erase(object_ref(right_energy, "energy"), "residual_j");
    auto tail = rows(2);
    const auto left = sealed(
        seal(dir, {sample_row(0, 0., kDt, {}, {}, {}, {},
                              std::move(left_energy)),
                   tail[1]}, "a"));
    const auto right = sealed(
        seal(dir, {sample_row(0, 0., kDt, {}, {}, {}, {},
                              std::move(right_energy)),
                   tail[1]}, "b"));
    const auto result = rec::compare(
        *left, *right, 0, 0, "exact",
        std::optional<std::string>("channel_energy.residual_j"));
    bt::require(field_bool(result, "compatible") && !field_bool(result, "agrees"),
                "explicit null and absent are not equal evidence");
    const auto &first = member_at(result, "first_mismatch");
    bt::require(field_int(first, "interval_id") == 0 &&
                field_real(first, "time_s") == 0. &&
                field_str(first, "path") == "channel_energy.residual_j_present" &&
                field_str(first, "kind") == "category" &&
                field_bool(first, "left") && !field_bool(first, "right"),
                "the presence companion reports the null/missing split");
}

void test_exact_comparison_reports_categorical_change() {
    bt::TemporaryDirectory dir("analysis-category");
    const auto with_status = [](std::string_view verdict) {
        return energy_channels(
            {}, WireObject{{"model_status",
                            Wire{WireObject{
                                {"model_valid", Wire{true}},
                                {"model_violation_counts", Wire{WireObject{}}},
                                {"numerically_valid",
                                 Wire{std::string(verdict)}},
                                {"calibration_status",
                                 Wire{std::string("parameterized_unvalidated")}}}}}});
    };
    std::vector<SamplePtr> left_rows, right_rows;
    for (std::int64_t i = 0; i < 3; ++i) {
        left_rows.push_back(sample_row(i, static_cast<double>(i) * kDt, kDt,
                                       {}, {}, {}, {}, with_status("true")));
        right_rows.push_back(sample_row(i, static_cast<double>(i) * kDt, kDt,
                                        {}, {}, {}, {},
                                        with_status("not_evaluated")));
    }
    const auto left = sealed(seal(dir, std::move(left_rows), "a"));
    const auto right = sealed(seal(dir, std::move(right_rows), "b"));
    const auto result = rec::compare(
        *left, *right, 0, 0, "exact",
        std::optional<std::string>("channel_model_status.model_status"));
    const auto &first = member_at(result, "first_mismatch");
    bt::require(field_str(first, "kind") == "category" &&
                field_str(first, "path") ==
                    "channel_model_status.model_status.numerically_valid" &&
                field_str(first, "left") == "true" &&
                field_str(first, "right") == "not_evaluated",
                "categorical changes name the leaf path and both spellings");
    bt::require(field_int(result, "mismatched_field_rows") == 3,
                "every interval carries the categorical change");
}

void test_summary_mode_reports_parameter_changes_without_alignment() {
    bt::TemporaryDirectory dir("analysis-summary");
    const auto summary_for = [](std::int64_t stiffness, double scale) {
        Object frame{{"seat_stiffness_n_mm",
                      rec::value::integer(stiffness)}};
        Object power{{"scale", rec::value::real(scale)}};
        Object config{{"frame", Value{std::move(frame)}},
                      {"power", Value{std::move(power)}}};
        Object metrics{{"duration_s", rec::value::real(4.)},
                       {"events", rec::value::integer(2)}};
        return Value{Object{{"resolved_config", Value{std::move(config)}},
                            {"episode_metrics", Value{std::move(metrics)}}}};
    };
    SyntheticCapture left_options, right_options;
    left_options.summaries = {summary_for(120, 1.)};
    right_options.summaries = {summary_for(150, 1.5)};
    const auto left = sealed(seal(dir, rows(), "a", std::move(left_options)));
    const auto right = sealed(seal(dir, rows(), "b", std::move(right_options)));
    const auto result = rec::compare(*left, *right, 0, 0, "summary");
    const auto &config = io::as_object(
        member_at(member_at(result, "left"), "resolved_config"), "c");
    bt::require(io::as_int64(
                    detail::field(
                        io::as_object(detail::field(config, "frame"), "f"),
                        "seat_stiffness_n_mm"),
                    "s") == 120 &&
                io::as_double(
                    detail::field(
                        io::as_object(detail::field(config, "power"), "p"),
                        "scale"),
                    "s") == 1.,
                "summary mode surfaces the left resolved config verbatim");
    bt::require(field_str(result, "interpretation").find("no automatic alignment")
                    != std::string::npos,
                "summary mode refuses implicit alignment");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_recording_json_boundary_round_trips_typed_literals();
        test_window_returns_interval_overlap_and_selected_fields();
        test_bounded_query_and_export_report_truncation_and_row_totals();
        test_sql_cannot_mutate_through_the_reader();
        test_exact_comparison_refuses_mismatched_clocks_before_values();
        test_matching_nonfinite_values_are_agreeing_invalid_evidence();
        test_report_excludes_forwarded_energy_extrema();
        test_exact_comparison_reports_first_mismatch_with_stable_keys();
        test_exact_comparison_reports_prefix_extent_mismatch();
        test_exact_comparison_rejects_incompatible_dof_coordinate_identity();
        test_exact_comparison_rejects_differing_measurement_provenance();
        test_exact_comparison_numeric_tolerance_boundary();
        test_exact_comparison_distinguishes_null_from_missing();
        test_exact_comparison_reports_categorical_change();
        test_summary_mode_reports_parameter_changes_without_alignment();
    });
}
