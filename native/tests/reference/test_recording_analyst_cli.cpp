// Completion 3.31 (T53). Source: tests/reference/test_recording_analyst_cli.py.
//
// The `bike-recording` console contract: JSON result envelopes on stdout,
// RECORDING ERROR diagnostics on stderr, exit code 0/1/2 — driven through
// bike::cli::recording_command with captured streams, the oracle's main()
// + capsys seam. Recordings are sealed synthetic captures; no simulator
// ever runs.
#include "recording_test_support.hpp"
#include "cli/recording.hpp"
#include "recording/publish.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace recording_test;
namespace detail = rec::detail;
using Object = io::Value::Object;

constexpr double kDt = 1. / 8;

struct Invocation {
    int status;
    std::string out, error;
};

// The oracle's `main(argv)` console seam: one recording_command call with
// captured streams — argv excludes the program name, like bike-recording's
// own entry point.
Invocation invoke(std::initializer_list<std::string_view> arguments) {
    std::ostringstream out, error;
    const std::vector<std::string_view> argv{arguments};
    const int status = bike::cli::recording_command(argv, out, error);
    return {.status = status, .out = out.str(), .error = error.str()};
}

Value json(const std::string &captured) {
    std::string source{captured};
    if (!source.empty() && source.back() == '\n') source.pop_back();
    return io::parse_recording_json(source);
}

std::vector<SamplePtr> episode_rows(std::int64_t count, double dt = kDt) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    std::vector<SamplePtr> out;
    out.reserve(static_cast<std::size_t>(count));
    for (std::int64_t i = 0; i < count; ++i)
      out.push_back(sample_row(i, static_cast<double>(i) * dt, dt));
    return out;
}

std::int64_t integer(const Value &row_, std::string_view name) {
    return io::as_int64(detail::field(row_, name), "member");
}
std::string str(const Value &row_, std::string_view name) {
    return io::as_string(detail::field(row_, name), "member");
}

void test_empty_result_sets_are_stable_json_envelopes() {
  bt::TemporaryDirectory const dir("cli-empty");
  auto capture = capture_rows({sample_row(0, 0., kDt)});
  const auto path = rec::finalize(*capture, dir.path()).directory.string();
  {
    const auto call = invoke({"events", path, "--episode", "0", "--json"});
    bt::require(call.status == 0, "events accepts an empty journal");
    const auto result = json(call.out);
    bt::require(io::as_array(detail::field(result, "rows"), "rows").empty() &&
                    io::as_int64(detail::field(result, "total"), "t") == 0 &&
                    !io::as_bool(detail::field(result, "truncated"), "t") &&
                    !io::as_bool(detail::field(result, "has_more"), "h"),
                "empty events keep the bounded envelope");
  }
    {   const auto call = invoke({"fields", path, "--pattern",
                                "no_such_field_", "--json"});
        bt::require(call.status == 0, "an unmatched pattern is not an error");
        const auto result = json(call.out);
        bt::require(io::as_array(detail::field(result, "rows"), "rows")
                        .empty() &&
                    io::as_int64(detail::field(result, "total"), "t") == 0,
                    "fields reports total zero for an unmatched pattern"); }
    {   const auto call = invoke({"window", path, "--episode", "0",
                                "--start", "5", "--end", "6",
                                "--fields", "physics.time_s", "--json"});
        bt::require(call.status == 0 &&
                    io::as_array(detail::field(json(call.out), "rows"), "rows")
                        .empty(),
                    "an out-of-extent window is an empty selection"); }
}

void test_failures_surface_recording_error_without_a_traceback() {
  bt::TemporaryDirectory const dir("cli-errors");
  auto capture = capture_rows({sample_row(0, 0., kDt)});
  const auto path = rec::finalize(*capture, dir.path()).directory.string();
  {
    const auto call = invoke({"window", path, "--episode", "0", "--start", "0",
                              "--end", "1", "--fields", "invented.field"});
    bt::require(call.status == 1 &&
                    call.error.find("invented.field") != std::string::npos,
                "an unknown field names itself on stderr");
  }
    {   const auto call = invoke({"sql", path, "--query",
                                "SELECT no_such_column FROM physics"});
        bt::require(call.status == 1 &&
                    call.error.find("no_such_column") != std::string::npos,
                    "a bad SQL column names itself on stderr"); }
}

void test_multi_episode_selection_stays_episode_scoped() {
  bt::TemporaryDirectory const dir("cli-multi");
  SyntheticCapture options;
  options.episodes = {episode_rows(2), episode_rows(3)};
  options.events = {
      input_event(0, 0, "episode_start", .1, 0),
      input_event(0, 1, "quit", .2, 1, Value{Object{{"reason", text("user")}}}),
      input_event(1, 0, "episode_start", 1.1, 0),
      input_event(1, 1, "quit", 1.2, 1,
                  Value{Object{{"reason", text("user")}}})};
  auto capture = synthetic_capture({}, std::move(options));
  const auto path = rec::finalize(*capture, dir.path()).directory.string();
  {
    const auto call = invoke({"overview", path, "--json"});
    bt::require(call.status == 0, "overview accepts a multi-episode run");
    const auto result = json(call.out);
    const auto &rows =
        io::as_array(detail::field(detail::field(result, "episodes"), "rows"),
                     "episodes.rows");
    bt::require(rows.size() == 2 && integer(rows[0], "episode_id") == 0 &&
                    integer(rows[1], "episode_id") == 1 &&
                    integer(rows[0], "physical_rows") == 2 &&
                    integer(rows[1], "physical_rows") == 3,
                "overview lists both episodes with their extents");
  }
    {   const auto call =
            invoke({"events", path, "--episode", "1", "--json"});
        const auto result = json(call.out);
        const auto &rows =
            io::as_array(detail::field(result, "rows"), "rows");
        bt::require(rows.size() == 2 &&
                    integer(rows[0], "event_id") == 0 &&
                    str(rows[0], "kind") == "episode_start" &&
                    integer(rows[1], "event_id") == 1 &&
                    str(rows[1], "kind") == "quit" &&
                    integer(rows[0], "episode_id") == 1 &&
                    integer(rows[1], "episode_id") == 1,
                    "events stay scoped to the requested episode"); }
    {   const auto call = invoke({"sql", path, "--query",
                                "SELECT interval_id FROM physics " // NOLINT(bugprone-suspicious-missing-comma) adjacent literals intentionally join the fragments
                                "WHERE episode_id=1 ORDER BY interval_id",
                                "--json"});
        const auto result = json(call.out);
        const auto &rows =
            io::as_array(detail::field(result, "rows"), "rows");
        bt::require(rows.size() == 3 && integer(rows[0], "interval_id") == 0 &&
                    integer(rows[2], "interval_id") == 2,
                    "episode-scoped SQL sees only that episode's rows"); }
}

void test_csv_export_writes_bounded_rows_and_refuses_overwrite() {
  bt::TemporaryDirectory const dir("cli-export");
  auto capture = capture_rows(episode_rows(3));
  const auto path = rec::finalize(*capture, dir.path()).directory.string();
  const auto destination = dir.path() / "out.csv";
  {
    const auto call = invoke({"export", path, "--episode", "0", "--format",
                              "csv", "--output", destination.string()});
    bt::require(call.status == 0, "csv export succeeds");
    const auto result = json(call.out);
    bt::require(io::as_int64(detail::field(result, "rows"), "r") == 3 &&
                    io::as_string(detail::field(result, "format"), "f") ==
                        "csv",
                "export reports the row total and format");
    std::ifstream in(destination);
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);)
      lines.push_back(line);
    bt::require(lines.size() == 4 && lines[0].starts_with("episode_id"),
                "csv carries a header plus one line per row");
    const auto header = lines[0];
    const auto interval_at = header.find("interval_id");
    bt::require(interval_at != std::string::npos,
                "interval_id is an exported column");
    // Column index of interval_id within the comma-separated header.
    std::size_t column = 0;
    for (std::size_t at = 0; at < interval_at; ++at)
      if (header[at] == ',')
        ++column;
    for (std::size_t i = 1; i < 4; ++i) {
      std::size_t begin = 0;
      for (std::size_t c = 0; c < column; ++c)
        begin = lines[i].find(',', begin) + 1;
      const auto end = lines[i].find(',', begin);
      bt::require(lines[i].substr(begin, end - begin) == std::to_string(i - 1),
                  "interval ids export in order");
    } }
    {   const auto call = invoke({"export", path, "--episode", "0",
                                "--format", "csv", "--output",
                                destination.string()});
        bt::require(call.status == 1 &&
                    call.error.find("exist") != std::string::npos,
                    "an existing export destination refuses overwrite"); }
    {   const auto bounded = dir.path() / "bounded.csv";
        const auto call = invoke({"export", path, "--episode", "0",
                                "--format", "csv", "--output",
                                bounded.string(), "--limit", "1"});
        bt::require(call.status == 0, "bounded csv export succeeds");
        std::ifstream in(bounded);
        std::size_t lines = 0;
        for (std::string line; std::getline(in, line);) ++lines;
        bt::require(lines == 2, "limit bounds the exported rows"); }
}

void test_bad_sql_and_missing_recordings_fail_with_recording_error() {
  bt::TemporaryDirectory const dir("cli-bad");
  auto capture = capture_rows({sample_row(0, 0., kDt)});
  const auto path = rec::finalize(*capture, dir.path()).directory.string();
  for (const char *sql :
       {"DELETE FROM physics", "SELECT 1; SELECT 2",
        "INSERT INTO physics DEFAULT VALUES", "not sql at all"}) {
    const auto call = invoke({"sql", path, "--query", sql});
    bt::require(call.status == 1 &&
                    call.error.find("RECORDING ERROR") != std::string::npos,
                "mutating or malformed SQL surfaces RECORDING ERROR");
  }
    {   const auto missing = dir.path() / "missing-run";
        const auto call = invoke({"overview", missing.string()});
        bt::require(call.status == 1 &&
                    call.error.find("RECORDING ERROR") != std::string::npos,
                    "a missing recording refuses without a traceback"); }
    {   const auto call = invoke({"sql", path, "--query", "SELECT 1"});
        bt::require(call.status == 0, "a read-only query succeeds"); }
}

void test_sql_json_marks_the_truncation_boundary() {
  bt::TemporaryDirectory const dir("cli-truncated");
  auto capture = capture_rows(episode_rows(211));
  const auto path = rec::finalize(*capture, dir.path()).directory.string();
  {
    const auto call = invoke({"sql", path, "--query",
                              "SELECT interval_id FROM physics " // NOLINT(bugprone-suspicious-missing-comma) adjacent literals intentionally join the fragments
                              "ORDER BY interval_id",
                              "--json"});
    bt::require(call.status == 0, "the bounded query succeeds");
    const auto result = json(call.out);
    bt::require(io::as_array(detail::field(result, "rows"), "rows").size() ==
                        200 &&
                    io::as_bool(detail::field(result, "truncated"), "t") &&
                    io::as_bool(detail::field(result, "has_more"), "h") &&
                    io::as_int64(detail::field(result, "limit"), "l") == 200,
                "the 200-row cap flags truncation");
  }
    {   const auto call = invoke({"sql", path, "--query",
                                "SELECT count(*) AS n FROM physics", "--json"});
        const auto result = json(call.out);
        const auto &rows =
            io::as_array(detail::field(result, "rows"), "rows");
        bt::require(rows.size() == 1 && integer(rows[0], "n") == 211,
                    "aggregate queries still see the full extent"); }
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_empty_result_sets_are_stable_json_envelopes();
        test_failures_surface_recording_error_without_a_traceback();
        test_multi_episode_selection_stays_episode_scoped();
        test_csv_export_writes_bounded_rows_and_refuses_overwrite();
        test_bad_sql_and_missing_recordings_fail_with_recording_error();
        test_sql_json_marks_the_truncation_boundary();
    });
}
