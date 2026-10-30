// Port of tests/test_batch.py — bike-batch spec expansion, CPU policy and
// actual serial/pool equivalence. expand_spec keeps insertion order, slugs
// overrides into job names, reserves `out`, and the CLI validates every job's
// argv with the exact bike-research parser before any process or output is
// created. `--serial`/`--jobs N` must not perform CPU discovery, the report
// carries workers/thread_policy, and a real two-job episode writes ordered
// batch_results.jsonl records through isolated bike-research children.
#include "../support/migration_test.hpp"
#include "../support/temporary_directory.hpp"
#include "batch/plan.hpp"
#include "cli/application.hpp"
#include "cli/arguments.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"
#include "platform/process.hpp"
#include "recording/value_util.hpp"
#include <array>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <system_error>
#include <vector>

namespace {
namespace batch = bike::batch;
using Value = bike::io::Value;
using Object = Value::Object;
using Array = Value::Array;
using bike::recording::value::integer;
using bike::recording::value::real;
using bike::recording::value::text;

using Members = std::initializer_list<std::pair<std::string, Value>>;
[[nodiscard]] Value spec(Members root) {
    return Value{Object{std::vector<std::pair<std::string, Value>>{root}}};
}
[[nodiscard]] Value spec(const Object& root) { return Value{root}; }
[[nodiscard]] Value obj(Members members) { return spec(members); }

// test_grid_cartesian_product_and_order — seed-major row order and argv
// spelled exactly as the reference emits it.
void grid_cartesian_product_and_order() {
    const auto jobs = batch::expand_spec(spec({
        {"base", obj({{"scenario", text("rough_uphill")}})},
        {"grid", obj({{"seed", Value{Array{integer(1), integer(2)}}},
                      {"rider-mass", Value{Array{real(60.), real(80.)}}}})},
    }));
    bt::require(jobs.size() == 4, "the grid expands to the full cartesian product");
    const std::array seeds{1., 1., 2., 2.};
    const std::array masses{60., 80., 60., 80.};
    for (std::size_t i = 0; i < 4; ++i) {
        bt::require(bike::io::as_double(
            *bike::io::find_member(bike::io::as_object(jobs.at(i).params, "params"), "seed"),
            "params.seed") == seeds.at(i), "grid rows iterate seed-major");
        bt::require(bike::io::as_double(
            *bike::io::find_member(bike::io::as_object(jobs.at(i).params, "params"), "rider-mass"),
            "params.rider-mass") == masses.at(i), "rider-mass iterates inside seed");
    }
    bt::require((jobs[0].argv == std::vector<std::string>{
        "--scenario", "rough_uphill", "--seed", "1", "--rider-mass", "60.0"}),
        "argv spells flags and values in spec order");
}

// test_bool_flags_and_job_overrides — true emits a bare flag, false omits it;
// explicit [job] rows merge onto the base.
void bool_flags_and_job_overrides() {
    const auto jobs = batch::expand_spec(spec({
        {"base", obj({{"assist", Value{true}}, {"ideal-sensors", Value{false}}})},
        {"job", Value{Array{obj({{"seed", integer(7)}, {"motor-torque", real(120.)}})}}},
    }));
    bt::require(jobs.size() == 1, "one explicit job row expands once");
    bt::require((jobs[0].argv == std::vector<std::string>{
        "--assist", "--seed", "7", "--motor-torque", "120.0"}),
        "true flags are bare, false flags vanish, job values override");
}

// test_range_tables — {start,count} and {start,stop,step} integer ranges.
void range_tables() {
    const auto jobs = batch::expand_spec(spec({
        {"grid", obj({
            {"seed", obj({{"start", integer(10)}, {"count", integer(3)}})},
            {"rear-teeth", obj({{"start", integer(40)}, {"stop", integer(52)}, {"step", integer(5)}})},
        })},
    }));
    bt::require(jobs.size() == 9, "3 seeds x 3 rear-teeth");
    const std::array seeds{10., 10., 10., 11., 11., 11., 12., 12., 12.};
    const std::array teeth{40., 45., 50., 40., 45., 50., 40., 45., 50.};
    for (std::size_t i = 0; i < 9; ++i) {
        const auto& params = bike::io::as_object(jobs.at(i).params, "params");
        bt::require(bike::io::as_double(*bike::io::find_member(params, "seed"), "seed") == seeds.at(i),
            "count ranges iterate inside");
        bt::require(bike::io::as_double(*bike::io::find_member(params, "rear-teeth"), "rear-teeth") == teeth.at(i),
            "stop ranges step exclusively");
    }
}

// test_out_is_reserved_everywhere — `out` in base, grid or job raises.
void out_is_reserved_everywhere() {
    const std::vector<Object> sections{
        Object{{{"base", obj({{"out", text("x")}})}}},
        Object{{{"grid", obj({{"out", Value{Array{text("x")}}}})}}},
        Object{{{"job", Value{Array{obj({{"out", text("x")}})}}}}},
    };
    for (const Object& section : sections) {
        bool rejected = false;
        try { (void)batch::expand_spec(spec(section)); }
        catch (const std::invalid_argument& error) {
            rejected = std::string(error.what()).find("out") != std::string::npos;
        }
        bt::require(rejected, "'out' is reserved in every spec section");
    }
}

// test_bad_values_and_sections_are_named — unknown sections, empty lists and
// non-scalar flags name themselves; an empty spec is one default job.
void bad_values_and_sections_are_named() {
    auto rejects_named = [](Object root, std::string_view needle) {
        try { (void)batch::expand_spec(spec(std::move(root))); return false; }
        catch (const std::invalid_argument& error) {
            return std::string(error.what()).find(needle) != std::string::npos;
        }
    };
    bt::require(rejects_named({{"junk", obj({})}}, "unknown spec sections"),
        "unknown spec sections are named");
    bt::require(rejects_named({{"grid", obj({{"seed", Value{Array{}}}})}}, "grid.seed"),
        "an empty grid list names grid.seed");
    bt::require(rejects_named({{"base", obj({{"rider-mass", Value{Array{real(60.), real(80.)}}}})}}, "rider-mass"),
        "a non-scalar base flag names itself");
    bt::require(batch::expand_spec(spec({{"job", Value{Array{}}}})).size() == 1,
        "an empty job list is one default episode");
    bt::require(batch::expand_spec(spec({})).size() == 1,
        "an empty spec is one default episode, not an error");
}

// test_slug_reflects_overrides — the job id carries index plus overridden keys.
void slug_reflects_overrides() {
    const auto jobs = batch::expand_spec(spec({
        {"base", obj({{"duration", real(5.)}})},
        {"grid", obj({{"seed", Value{Array{integer(42)}}}})},
    }));
    bt::require(jobs.size() == 1 && jobs[0].id == "0000-seed42",
        "the slug appends changed flags to the zero-padded index");
}

// test_validate_rejects_flags_research_does_not_know — every expanded job's
// argv goes through the exact bike-research parser before output exists.
void validate_rejects_flags_research_does_not_know() {
    const auto jobs = batch::expand_spec(spec({{"base", obj({{"not-a-flag", integer(3)}})}}));
    bt::require(jobs.size() == 1, "expansion itself does not screen flags");
    bool rejected = false;
    try {
        const std::vector<std::string_view> views{jobs[0].argv.begin(), jobs[0].argv.end()};
        const auto parsed = bike::cli::parse_arguments("bike-research", views);
        if (!parsed.help) bike::cli::validate_research_arguments(parsed);
    } catch (const std::exception& error) {
        rejected = std::string(error.what()).find("not-a-flag") != std::string::npos;
    }
    bt::require(rejected, "an unknown flag is rejected by the research parser");
}

[[nodiscard]] std::string welded_toml() {
    // bt::fixture lands in <repo>/tests/fixtures/migration — five parents up is
    // the repository root that owns examples/research/.
    auto root = bt::fixture("bootstrap/welded");
    for (int i = 0; i < 5; ++i) root = root.parent_path();
    return (root / "examples/research/viewer_physics_welded.toml").string();
}

[[nodiscard]] std::filesystem::path batch_binary() {
    // bike-batch lands at <build-root>/bin; this test's binary is four levels
    // down in native/tests/reference (same layout as test_replay_playback).
    return bike::cli::executable_path()
        .parent_path().parent_path().parent_path().parent_path() / "bin" / "bike-batch";
}

[[nodiscard]] std::vector<Value> read_results(const std::filesystem::path& path) {
    std::ifstream stream(path);
    bt::require(stream.good(), "batch_results.jsonl was published");
    std::vector<Value> records;
    for (std::string line; std::getline(stream, line);)
        if (!line.empty()) records.push_back(bike::io::parse_recording_json(line));
    return records;
}

void write_track(const std::filesystem::path& path) {
    std::ofstream stream(path);
    stream << "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n";
}

// test_serial_two_job_episode — two minimal episodes through the real
// pipeline, sequentially: 2 ordered records, job directories, exit plumbing.
void serial_two_job_episode() {
    const bt::TemporaryDirectory tmp_("batch");
    const auto& tmp = tmp_.path();
    const auto spec_path = tmp / "spec.toml";
    const auto track = tmp / "track.toml";
    write_track(track);
    {
        std::ofstream stream(spec_path);
        stream << "[base]\n"
            "physics-config = \"" << welded_toml() << "\"\n"
            << "track-file = \"" << track.string() << "\"\n"
            "duration = 0.2\ninitial-speed = 0\nenergy-tolerance = 1e9\n"
            "diagnostic-model-limits = true\nrecord-decimation = 400\n\n"
            "[grid]\nseed = [0, 1]\n";
    }
    const auto out = tmp / "batch";
    const auto binary = batch_binary();
    bt::require(std::filesystem::exists(binary), "the bike-batch artifact is built");
    bike::platform::ProcessOptions options;
    options.timeout_s = 300.;
    const auto ran = bike::platform::run({binary.string(), "--spec", spec_path.string(),
        "--out", out.string(), "--serial"}, tmp, options);
    const auto records = read_results(out / "batch_results.jsonl");
    bt::require(records.size() == 2, "two jobs write two records");
    std::set<std::string> names;
    for (const auto& record : records) {
        const auto& fields = bike::io::as_object(record, "record");
        names.insert(bike::io::as_string(*bike::io::find_member(fields, "name"), "name"));
        const auto exit_code = bike::io::as_double(*bike::io::find_member(fields, "exit"), "exit");
        bt::require(exit_code == 0. || exit_code == 2. || exit_code == 3.,
            "plumbing exit codes, not physics");
    }
    bt::require((names == std::set<std::string>{"0000-seed0", "0001-seed1"}),
        "records are named by index plus slug");
    for (const auto& name : names)
        bt::require(std::filesystem::is_directory(out / name), "each job output directory exists");
    const bool all_zero = std::ranges::all_of(records, [](const Value& record) {
        const auto& fields = bike::io::as_object(record, "record");
        return bike::io::as_double(*bike::io::find_member(fields, "exit"), "exit") == 0.;
    });
    bt::require(ran.exit_code == (all_zero ? 0 : 1),
        "the batch exit code mirrors all-zero job results");
}

// test_nonpositive_worker_count_is_rejected_before_output — --jobs 0 / -2
// exit 2 with no output directory created.
void nonpositive_worker_count_is_rejected_before_output() {
    const bt::TemporaryDirectory tmp_("batch");
    const auto& tmp = tmp_.path();
    const auto binary = batch_binary();
    bt::require(std::filesystem::exists(binary), "the bike-batch artifact is built");
    for (const char* count : {"0", "-2"}) {
        const auto out = tmp / (std::string("out") + count);
        bike::platform::ProcessOptions options;
        options.timeout_s = 60.;
        const auto ran = bike::platform::run({binary.string(), "--spec",
            (tmp / "absent.toml").string(), "--out", out.string(), "--jobs", count},
            tmp, options);
        bt::require(ran.exit_code == 2, "a nonpositive --jobs exits 2");
        bt::require(!std::filesystem::exists(out), "no output is created before rejection");
    }
}

// test_explicit_serial_job_count_avoids_discovery_and_reports_threads — an
// explicit --jobs never performs CPU discovery and the printed summary
// reports workers=1 plus the OPENBLAS_NUM_THREADS policy.
void explicit_serial_job_count_avoids_discovery_and_reports_threads() {
    const bt::TemporaryDirectory tmp_("batch");
    const auto& tmp = tmp_.path();
    const auto spec_path = tmp / "spec.toml";
    { std::ofstream stream(spec_path); stream << "[base]\nseed = 9\nduration = 0.01\nphysics-config = \"" << welded_toml() << "\"\n"; }
    const auto out = tmp / "batch";
    const auto binary = batch_binary();
    bt::require(std::filesystem::exists(binary), "the bike-batch artifact is built");
    bike::platform::ProcessOptions options;
    options.timeout_s = 300.;
    // platform::run inherits this process's environment verbatim.
    const char* previous = std::getenv("OPENBLAS_NUM_THREADS");
    const std::string saved = previous ? previous : "";
    ::setenv("OPENBLAS_NUM_THREADS", "2", 1);
    const auto ran = bike::platform::run({binary.string(), "--spec", spec_path.string(),
        "--out", out.string(), "--jobs", "1"}, tmp, options);
    bt::require(ran.exit_code == 0 || ran.exit_code == 1,
        "the explicit worker count runs without discovery: " + ran.stderr_text);
    bt::require(ran.stdout_text.find("workers=1") != std::string::npos &&
        ran.stdout_text.find("start_method=in-process") != std::string::npos,
        "the run announces workers=1 start_method=in-process");
    const auto report = bike::io::parse_recording_json(ran.stdout_text.substr(ran.stdout_text.rfind("\n{") + 1));
    const auto& fields = bike::io::as_object(report, "report");
    bt::require(bike::io::as_double(*bike::io::find_member(fields, "workers"), "workers") == 1.,
        "the report records workers=1");
    const auto* policy = bike::io::find_member(fields, "thread_policy");
    bt::require(policy != nullptr, "the report carries thread_policy");
    const auto& policy_object = bike::io::as_object(*policy, "thread_policy");
    const auto* threads = bike::io::find_member(policy_object, "OPENBLAS_NUM_THREADS");
    bt::require(threads != nullptr &&
        bike::io::as_string(*threads, "OPENBLAS_NUM_THREADS") == "2",
        "thread_policy echoes OPENBLAS_NUM_THREADS=2");
    // The Python contract asserts the variable is still '2' after the run:
    // batch reports the inherited policy without clobbering it.
    const char* after = std::getenv("OPENBLAS_NUM_THREADS");
    bt::require(after != nullptr && std::string(after) == "2",
        "the parent environment is untouched");
    if (previous) ::setenv("OPENBLAS_NUM_THREADS", saved.c_str(), 1);
    else ::unsetenv("OPENBLAS_NUM_THREADS");
}

// test_actual_two_worker_batch_matches_serial_jobs — the real pool matches
// serial execution: identical name/exit/outcome/metrics/params/argv rows.
void actual_two_worker_batch_matches_serial_jobs() {
    const bt::TemporaryDirectory tmp_("batch");
    const auto& tmp = tmp_.path();
    const auto spec_path = tmp / "spec.toml";
    const auto track = tmp / "track.toml";
    write_track(track);
    {
        std::ofstream stream(spec_path);
        stream << "[base]\n"
            "physics-config = \"" << welded_toml() << "\"\n"
            << "track-file = \"" << track.string() << "\"\n"
            "duration = 0.01\ncheck-policy = \"diagnostic\"\nmotor-torque = 0\n"
            "human-torque = 0\n\n[grid]\nseed = [0, 1]\n";
    }
    const auto binary = batch_binary();
    bt::require(std::filesystem::exists(binary), "the bike-batch artifact is built");
    std::vector<std::map<std::int64_t, std::string>> runs;
    for (const auto& mode : {std::pair{"serial", std::vector<std::string>{"--serial"}},
                             std::pair{"pool", std::vector<std::string>{"--jobs", "2"}}}) {
        const auto out = tmp / mode.first;
        bike::platform::ProcessOptions options;
        options.timeout_s = 300.;
        std::vector<std::string> argv{binary.string(), "--spec", spec_path.string(),
            "--out", out.string()};
        argv.insert(argv.end(), mode.second.begin(), mode.second.end());
        const auto ran = bike::platform::run(argv, tmp, options);
        const auto diagnostic = ran.stdout_text + ran.stderr_text;
        bt::require(ran.exit_code == 0 || ran.exit_code == 1,
            "batch completes in " + std::string(mode.first) + " mode: " + diagnostic);
        const auto rows = read_results(out / "batch_results.jsonl");
        bt::require(rows.size() == 2, "two records per mode: " + diagnostic);
        std::map<std::int64_t, std::string> by_index;
        std::set<std::string> outs;
        for (const auto& row : rows) {
            const auto& fields = bike::io::as_object(row, "record");
            const auto exit_code = bike::io::as_double(*bike::io::find_member(fields, "exit"), "exit");
            const auto* outcome = bike::io::find_member(fields, "outcome");
            bt::require((exit_code == 0. || exit_code == 2.) && outcome != nullptr &&
                bike::io::as_string(*outcome, "outcome") == "duration" &&
                bike::io::find_member(fields, "error") == nullptr,
                "each job exits cleanly with the duration outcome: " + diagnostic);
            Object kept;
            for (const char* key : {"name", "exit", "outcome", "metrics", "params", "argv"})
                if (const auto* member = bike::io::find_member(fields, key)) kept.emplace_back(key, *member);
            by_index.emplace(static_cast<std::int64_t>(bike::io::as_double(
                *bike::io::find_member(fields, "index"), "index")),
                bike::io::canonical_json(Value{std::move(kept)}));
            outs.insert(bike::io::as_string(*bike::io::find_member(fields, "out"), "out"));
        }
        for (const auto& entry : outs)
            bt::require(std::filesystem::path(entry).parent_path() == out,
                "job outputs stay under the batch root");
        bt::require(outs.size() == 2, "each job owns a distinct output directory");
        runs.push_back(std::move(by_index));
    }
    bt::require(runs[0] == runs[1],
        "serial and two-worker runs produce identical result rows");
}
} // namespace

int main() {
    return bt::run([] {
        grid_cartesian_product_and_order();
        bool_flags_and_job_overrides();
        range_tables();
        out_is_reserved_everywhere();
        bad_values_and_sections_are_named();
        slug_reflects_overrides();
        validate_rejects_flags_research_does_not_know();
        serial_two_job_episode();
        nonpositive_worker_count_is_rejected_before_output();
        explicit_serial_job_count_avoids_discovery_and_reports_threads();
        actual_two_worker_batch_matches_serial_jobs();
    });
}
