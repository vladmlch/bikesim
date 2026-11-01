// Completion 3.33b. Source: tests/reference/test_research_sweep.py.
// The bike-research-sweep fan-out: dry-run listing, seed-list parsing and
// the per-seed return-code summary.
//
// Substitution notes:
//  * tools.research_sweep.main forwards to the bike-research-sweep binary;
//    the dry-run port runs that artifact as a child process and reads its
//    captured stdout — exactly the capsys probe, one layer more faithful.
//  * The run_one monkeypatch is the executable seam: a stub child exits 0
//    iff its argv names --seed 2, injecting the same deterministic return
//    codes without real episodes.
#include "research_reference_support.hpp"
#include "batch/plan.hpp"
#include "cli/application.hpp"
#include "io/json.hpp"
#include "platform/process.hpp"
#include <array>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace {
using namespace research_reference;
namespace io = bike::io;
namespace rd = bike::recording::detail;

[[nodiscard]] std::filesystem::path sweep_binary() {
    // bike-research-sweep lands at <build-root>/bin; this test's binary is
    // four levels down in native/tests/reference (the test_batch layout).
    return bike::cli::executable_path()
        .parent_path().parent_path().parent_path().parent_path() /
        "bin" / "bike-research-sweep";
}

// tests/reference/test_research_sweep.py::test_dry_run_lists_one_command_per_seed
void test_dry_run_lists_one_command_per_seed() {
  Directory const directory;
  const auto sweep = sweep_binary();
  bt::require(std::filesystem::exists(sweep),
              "the bike-research-sweep artifact is built");
  const auto result = bike::platform::run(
      {sweep.string(), "--out", directory.path.string(), "--seeds", "3-5",
       "--dry-run", "--", "--duration", "1"},
      directory.path);
  bt::require(result.exit_code == 0, "a dry run exits 0: " + result.err);
  // lines = [l for l in capsys...out.splitlines() if 'bike-research' in l]
  std::vector<std::string> lines;
  std::istringstream stream(result.out);
  for (std::string line; std::getline(stream, line);)
    if (line.find("bike-research") != std::string::npos)
      lines.push_back(line);
  bt::require(lines.size() == 3, "one listed command per seed");
  std::vector<std::string> seeds;
  for (const auto &line : lines) {
    bt::require(line.find("--duration 1") != std::string::npos,
                "the passthrough arguments reach every command");
    // l.split('--seed ')[1].split()[0]
    const auto at = line.find("--seed ");
    bt::require(at != std::string::npos, "each command carries its seed");
    const auto begin = at + std::string_view("--seed ").size();
    const auto end = line.find(' ', begin);
    seeds.push_back(
        line.substr(begin, end == std::string::npos ? end : end - begin));
  }
    bt::require(seeds == std::vector<std::string>({"3", "4", "5"}),
                "the seed fan-out preserves range order");
}

// tests/reference/test_research_sweep.py::test_seed_ranges_and_lists_parse
void test_seed_ranges_and_lists_parse() {
    bt::require(bike::batch::parse_seeds("1-3,7") ==
                    std::vector<std::string>({"1", "2", "3", "7"}),
                "ranges and comma lists expand in order");
    bt::require(bike::batch::parse_seeds("4") == std::vector<std::string>({"4"}),
                "a lone seed parses");
}

// tests/reference/test_research_sweep.py::test_summary_records_return_codes
void test_summary_records_return_codes() {
  Directory const directory;
  // run_one twin: exit 0 iff the child command line names --seed 2.
  const auto stub = directory.path / "stub-research.sh";
  {
    std::ofstream script(stub);
    script << "#!/bin/sh\ncase \" $* \" in *\" --seed 2 \"*) exit 0;; *) exit "
              "3;; esac\n";
  }
    std::filesystem::permissions(stub, std::filesystem::perms::owner_all);
    // --out must be empty at dispatch (the stub script already occupies the
    // temp root, so the run root is a fresh child).
    const auto out = (directory.path / "out").string();
    const std::array<std::string_view, 6> argv{
        "--out", std::string_view{out}, "--seeds", "1-2", "--jobs", "2"};
    std::ostringstream const captured;
    auto *const restore = std::cout.rdbuf(captured.rdbuf());
    const auto code = bike::batch::research_sweep(argv, stub);
    std::cout.rdbuf(restore);
    bt::require(code == 1, "a nonzero child propagates into the sweep exit code");
    const auto summary =
        io::parse_recording_json(bt::text(directory.path / "out" / "sweep.json"));
    // {row['seed']: row['returncode'] for row in summary['runs']}
    std::map<std::int64_t, std::int64_t> runs;
    for (const auto &row : io::as_array(rd::field(summary, "runs"), "runs"))
        runs.emplace(io::as_int64(rd::field(row, "seed"), "seed"),
                     io::as_int64(rd::field(row, "returncode"), "returncode"));
    bt::require(runs == std::map<std::int64_t, std::int64_t>({{1, 3}, {2, 0}}),
                "the summary records every seed's return code");
}

} // namespace

int main() {
    return bt::run([] {
        test_dry_run_lists_one_command_per_seed();
        test_seed_ranges_and_lists_parse();
        test_summary_records_return_codes();
    });
}
