// Completion 3.31 (T53). Source: tests/reference/test_recording_storage.py.
//
// Container/publication regressions — no simulator ever runs. The oracle's
// ``synthetic_capture``/``TreePhysicalSource`` seam is the production
// OwnedNativeEpisode + make_physical_source pair; ``capture.finalize`` is
// rec::finalize over a real RunCapture, and the oracle's
// os.link/``_reporter`` monkeypatches land on FinalizeOptions::before_stage
// / FinalizeOptions::report — the cold-path fault-injection hooks the
// publisher ships for exactly this purpose.
//
// The oracle's ``duckdb.connect`` + manifest re-hash reseal path is
// recording_test::mutate_database: a real read-write Database connection
// plus file_sha256 re-cover, so only semantic validators can still refuse.
#include "recording_test_support.hpp"
#include "recording/publish.hpp"
#include "platform/files.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace {

using namespace recording_test;
namespace detail = rec::detail;
using Object = io::Value::Object;

std::vector<SamplePtr> energy_rows(std::span<const double> residuals,
                                   double dt = 1. / 8) {
    std::vector<SamplePtr> rows;
    rows.reserve(residuals.size());
    for (std::size_t i = 0; i < residuals.size(); ++i)
      rows.push_back(sample_row(
          static_cast<std::int64_t>(i), static_cast<double>(i) * dt, dt, {}, {},
          {}, {},
          energy_channels(WireObject{{"residual_j", Wire{residuals[i]}}})));
    return rows;
}

// {p.name: p.read_bytes()} — the read-only publication snapshot the oracle
// diffs across a Recording session.
std::map<std::string, std::string> snapshot(const std::filesystem::path &dir) {
    std::map<std::string, std::string> out;
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
        std::ifstream in(entry.path(), std::ios::binary);
        out[entry.path().filename().string()] =
            std::string(std::istreambuf_iterator<char>(in), {});
    }
    return out;
}

std::vector<std::string> names(const std::filesystem::path &dir) {
    std::vector<std::string> out;
    for (const auto &entry : std::filesystem::directory_iterator(dir))
        out.push_back(entry.path().filename().string());
    std::ranges::sort(out);
    return out;
}

void test_roundtrip_typed_nonfinite_presence_and_owned_asset() {
  bt::TemporaryDirectory const dir("storage-roundtrip");
  const double residuals[] = {0., -0., std::numeric_limits<double>::quiet_NaN(), // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()};
  const std::string payload =
      "captured input, not a reference to a mutable pathname";
  const auto payload_bytes = std::as_bytes(std::span{payload});
  const std::vector<std::byte> bytes(payload_bytes.begin(),
                                     payload_bytes.end());
  auto capture = capture_rows(
      energy_rows(residuals), "viewer",
      Value{Object{{"outcome", Value{Object{{"reason", text("synthetic")}}}}}},
      {rec::asset("test.txt", "text/plain", bytes)});
  const auto published = rec::finalize(*capture, dir.path());
  bt::require(names(published.directory) ==
                  std::vector<std::string>(
                      {"manifest.json", "run.duckdb", "summary.md"}),
              "sealed directory carries exactly the fixed payload files");
  const auto reader = sealed(published.directory);
  const auto stored = reader->rows(
      "SELECT residual_j FROM channel_energy ORDER BY interval_id");
  bt::require(stored.size() == 5, "five stored residual rows");
  bt::require(
      io::tagged_double(detail::field(stored[0], "residual_j"), "r") == 0. &&
          std::signbit(
              io::tagged_double(detail::field(stored[1], "residual_j"), "r")) &&
          std::isnan(
              io::tagged_double(detail::field(stored[2], "residual_j"), "r")) &&
          io::tagged_double(detail::field(stored[3], "residual_j"), "r") ==
              std::numeric_limits<double>::infinity() &&
          io::tagged_double(detail::field(stored[4], "residual_j"), "r") ==
              -std::numeric_limits<double>::infinity(),
      "typed nonfinite and signed-zero round trip");
  const auto asset_row = reader->rows("SELECT content FROM assets");
  bt::require(asset_row.size() == 1, "one stored asset");
  const auto *content =
      std::get_if<io::Bytes>(&detail::field(asset_row[0], "content").data);
  bt::require(content && content->value == bytes,
              "asset bytes are owned data, not a pathname");
  const auto episode = reader->episode();
  bt::require(
      io::as_int64(detail::field(episode, "physical_rows"), "rows") == 5 &&
          io::is_null(detail::field(episode, "replay_contract_version")),
      "episode extent and absent replay contract");
  bt::require(io::as_string(detail::field(reader->manifest(), "report_status"),
                            "status") == "complete",
              "complete report status on a healthy publication");
}

void test_default_publisher_handles_empty_run_and_zero_step_episode() {
  bt::TemporaryDirectory const dir("storage-empty");
  rec::RunCapture empty(run_metadata(), true);
  const auto empty_path = rec::finalize(empty, dir.path() / "no-episodes");
  {
    const auto reader = sealed(empty_path.directory);
    bt::require(reader->episode_ids().empty(),
                "episode-free run seals with no episode ids");
  }
    auto zero = capture_rows({});
    const auto zero_path = rec::finalize(*zero, dir.path() / "zero-step");
    {   const auto reader = sealed(zero_path.directory);
        bt::require(reader->episode_ids() == std::vector<std::int64_t>{0} &&
                    io::as_int64(detail::field(reader->episode(), "physical_rows"),
                                 "rows") == 0,
                    "zero-step episode seals with an empty extent"); }
}

void test_disabled_capture_does_not_create_output() {
  bt::TemporaryDirectory const dir("storage-disabled");
  const auto path = dir.path() / "must-not-exist";
  rec::RunCapture capture(run_metadata(), false);
  const auto result = rec::finalize(capture, path);
  bt::require(result.directory.empty() && result.report_status == "disabled" &&
                  !std::filesystem::exists(path),
              "disabled capture publishes nothing");
}

void test_publication_collision_does_not_adopt_even_an_empty_directory() {
  bt::TemporaryDirectory const dir("storage-collision");
  auto first_capture = capture_rows({});
  const auto first = rec::finalize(*first_capture, dir.path()).directory;
  const auto before = snapshot(first);
  const auto occupied =
      first.parent_path() / (first.filename().string() + "-1");
  std::filesystem::create_directory(occupied);
  auto second_capture = capture_rows({});
  const auto second = rec::finalize(*second_capture, dir.path()).directory;
  bt::require(second.filename().string() == first.filename().string() + "-2",
              "collision bumps the publication suffix, never adopts");
  bt::require(std::filesystem::is_empty(occupied),
              "the occupied directory stays untouched");
  bt::require(snapshot(first) == before, "first publication is byte-stable");
}

void test_report_failure_publishes_valid_database_with_explicit_status() {
  bt::TemporaryDirectory const dir("storage-report-fail");
  auto capture = capture_rows({});
  rec::FinalizeOptions options;
  options.report = [](const std::filesystem::path &database) -> std::string {
    // The oracle's _reporter wrote a partial destination then raised.
    std::ofstream(database.parent_path() / "summary.md") << "partial report";
    throw std::runtime_error("report failure injection");
  };
  const auto published = rec::finalize(*capture, dir.path(), options);
  const auto reader = sealed(published.directory);
  bt::require(
      io::as_string(detail::field(reader->manifest(), "report_status"),
                    "status") == "failed" &&
          io::as_string(detail::field(reader->manifest(), "report_error"),
                        "error")
                  .find("report failure injection") != std::string::npos,
      "failed report keeps an explicit status and error");
  const auto &hashes =
      io::as_object(detail::field(reader->manifest(), "file_sha256"), "hashes");
  bt::require(hashes.size() == 1 && detail::find(hashes, "run.duckdb"),
              "only the database payload is sealed on report failure");
  bt::require(!std::filesystem::exists(published.directory / "summary.md"),
              "no partial report is published");
}

void test_manifest_is_last_and_failed_link_keeps_owned_evidence_retryable() {
    bt::TemporaryDirectory dir("storage-manifest");
    auto capture = capture_rows({sample_row(0, 0.)});
    std::vector<std::string> calls;
    rec::FinalizeOptions options;
    options.before_stage = [&](std::string_view stage) {
        if (stage == "link_run.duckdb" || stage == "link_summary.md" ||
            stage == "manifest_link")
            calls.emplace_back(stage);
        if (stage == "manifest_link")
            throw std::runtime_error("manifest publication injection");
    };
    throws<std::exception>(
        [&] { (void)rec::finalize(*capture, dir.path(), options); },
        "manifest publication injection", "manifest link failure propagates");
    // The oracle observed the link order run.duckdb, summary.md,
    // manifest.json; natively the staged link hooks carry the same order —
    // the sole sealing marker lands LAST.
    bt::require(calls == std::vector<std::string>(
                             {"link_run.duckdb", "link_summary.md",
                              "manifest_link"}),
                "payloads link before the manifest marker");
    bt::require(capture->state() == rec::RunCapture::State::publication_failed,
                "publication failure is named on the capture state");
    bt::require(capture->episodes().front()->physical->row_count() == 1,
                "owned evidence stays reachable for a retry");
    for (const auto &candidate :
         std::filesystem::directory_iterator(dir.path()))
        if (candidate.is_directory())
            throws<rec::RecordingError>(
                [&] { (void)rec::Recording(candidate.path()); }, "unsealed",
                "unsealed staging and half-published destinations refuse open");
    const auto published = rec::finalize(*capture, dir.path());
    const auto reader = sealed(published.directory);
    bt::require(io::as_int64(detail::field(reader->episode(), "physical_rows"),
                             "rows") == 1,
                "the retried publication seals the retained evidence");
}

void test_reader_rejects_unsealed_or_tampered_payload() {
  bt::TemporaryDirectory const dir("storage-tamper");
  { // missing_manifest
    auto capture = capture_rows({});
    const auto path = rec::finalize(*capture, dir.path() / "a").directory;
    std::filesystem::remove(path / "manifest.json");
    throws<rec::RecordingError>([&] { (void)rec::Recording(path); }, "unsealed",
                                "missing manifest refuses open");
  }
    {   // changed_database
        auto capture = capture_rows({});
        const auto path = rec::finalize(*capture, dir.path() / "b").directory;
        {   std::ofstream stream(path / "run.duckdb", std::ios::app);
            stream << "checksum mismatch"; }
        throws<rec::RecordingError>(
            [&] { (void)rec::Recording(path); }, "checksum",
            "payload checksum mismatch refuses open"); }
    {   // symlink_database
        auto capture = capture_rows({});
        const auto path = rec::finalize(*capture, dir.path() / "c").directory;
        const auto moved = path / "elsewhere.db";
        std::filesystem::rename(path / "run.duckdb", moved);
        std::filesystem::create_symlink(moved, path / "run.duckdb");
        throws<std::system_error>(
            [&] { (void)rec::Recording(path); }, "",
            "a symlinked payload refuses open"); }
}

void test_resealed_catalog_tamper_is_rejected_before_analysis() {
  bt::TemporaryDirectory const dir("storage-catalog-tamper");
  auto capture = capture_rows({});
  const auto path = rec::finalize(*capture, dir.path()).directory;
  mutate_database(path, "UPDATE field_catalog SET unit='invented' "
                        "WHERE table_name='physics' AND field_name='time_s'");
  throws<std::invalid_argument>(
      [&] { (void)rec::Recording(path); }, "field catalog",
      "rehashed catalog tamper is rejected before analysis");
}

void test_assets_are_verified_before_publication() {
    bt::TemporaryDirectory dir("storage-assets");
    const std::string payload = "not the declared digest";
    const auto payload_bytes = std::as_bytes(std::span{payload});
    const std::vector<std::byte> bytes(payload_bytes.begin(),
                                       payload_bytes.end());
    auto capture = capture_rows({}, "viewer", Value{Object{}},
                                {rec::Asset{.name = "bad.txt",
                                            .media_type = "text/plain",
                                            .sha256 = std::string(64, '0'),
                                            .content = bytes}});
    throws<std::runtime_error>(
        [&] { (void)rec::finalize(*capture, dir.path()); }, "asset checksum",
        "a declared-digest mismatch refuses publication");
    for (const auto &candidate :
         std::filesystem::recursive_directory_iterator(dir.path()))
        bt::require(candidate.path().filename() != "manifest.json",
                    "no sealed manifest survives a refused publication");
}

void test_multi_episode_round_trip_keeps_extents_and_events_isolated() {
  bt::TemporaryDirectory const dir("storage-multi");
  std::vector<SamplePtr> first, second;
  for (int i = 0; i < 3; ++i)
    first.push_back(sample_row(i, static_cast<double>(i) / 8, 1. / 8));
  for (int i = 0; i < 2; ++i)
    second.push_back(sample_row(i, static_cast<double>(i) / 4, 1. / 4));
  SyntheticCapture options;
  options.episodes = {std::move(first), std::move(second)};
  options.events = {
      input_event(0, 0, "episode_start", .1, 0),
      input_event(0, 1, "quit", .2, 2, Value{Object{{"reason", text("user")}}}),
      input_event(1, 0, "episode_start", .3, 0)};
  options.summaries = {
      Value{Object{{"outcome", Value{Object{{"reason", text("first")}}}}}},
      Value{Object{{"outcome", Value{Object{{"reason", text("second")}}}}}}};
  auto capture = synthetic_capture({}, std::move(options));
  const auto path = rec::finalize(*capture, dir.path()).directory;
  const auto reader = sealed(path);
  bt::require(reader->episode_ids() == std::vector<std::int64_t>({0, 1}),
              "two episodes seal in order");
  const auto extents =
      reader->rows("SELECT episode_id,count(*) AS n,min(interval_id) AS lo,"
                   "max(interval_id) AS hi,min(time_s) AS t0,"
                   "max(interval_end_s) AS t1 FROM physics GROUP BY episode_id "
                   "ORDER BY episode_id");
  bt::require(extents.size() == 2, "one extent row per episode");
  bt::require(io::as_int64(detail::field(extents[0], "n"), "n") == 3 &&
                  io::as_int64(detail::field(extents[0], "lo"), "lo") == 0 &&
                  io::as_int64(detail::field(extents[0], "hi"), "hi") == 2 &&
                  io::as_double(detail::field(extents[0], "t0"), "t0") == 0. &&
                  io::as_double(detail::field(extents[0], "t1"), "t1") == .375,
              "episode 0 keeps its own extent");
  bt::require(io::as_int64(detail::field(extents[1], "episode_id"), "e") == 1 &&
                  io::as_int64(detail::field(extents[1], "n"), "n") == 2 &&
                  io::as_double(detail::field(extents[1], "t1"), "t1") == .5,
              "episode 1 keeps its own extent");
  const auto journal =
      reader->rows("SELECT episode_id,event_id,kind,values_json "
                   "FROM events ORDER BY episode_id,event_id");
  bt::require(
      journal.size() == 3 &&
          io::as_int64(detail::field(journal[0], "event_id"), "e") == 0 &&
          io::as_string(detail::field(journal[0], "kind"), "k") ==
              "episode_start" &&
          io::as_string(detail::field(journal[1], "kind"), "k") == "quit" &&
          io::as_int64(detail::field(journal[2], "episode_id"), "e") == 1,
      "journaled events keep episode isolation and order");
  const auto values = io::parse_recording_json(
      io::as_string(detail::field(journal[1], "values_json"), "v"));
  bt::require(io::as_string(detail::field(values, "reason"), "r") == "user",
              "event values round-trip through recording JSON");
  const auto count =
      reader->rows("SELECT count(*) AS n FROM events WHERE episode_id=1");
  bt::require(io::as_int64(detail::field(count[0], "n"), "n") == 1,
              "episode 1 owns exactly its start event");
  const auto summaries = reader->rows(
      "SELECT episode_id,summary_json FROM episodes ORDER BY episode_id");
  for (std::size_t i = 0; i < 2; ++i) {
    const auto summary = io::parse_recording_json(
        io::as_string(detail::field(summaries[i], "summary_json"), "s"));
    bt::require(io::as_string(
                    detail::field(detail::field(summary, "outcome"), "reason"),
                    "r") == (i == 0 ? "first" : "second"),
                "per-episode summaries stay aligned");
  }
}

void test_forces_round_trip_records_components_and_checks_qfrc_width() {
    bt::TemporaryDirectory dir("storage-forces");
    std::vector<SamplePtr> rows;
    rows.reserve(2);
    for (int i = 0; i < 2; ++i)
      rows.push_back(
          sample_row(i, static_cast<double>(i) / 8, 1. / 8, {},
                     std::vector<double>{2., 0., 0., 0., 0., 0., 0., 0.}));
    SyntheticCapture options;
    options.forces = true;
    auto capture = synthetic_capture(rows, options);
    const auto path = rec::finalize(*capture, dir.path()).directory;
    {   const auto reader = sealed(path);
        const auto stored = reader->rows(
            "SELECT episode_id,interval_id,component,qfrc,power_w "
            "FROM forces ORDER BY interval_id,component");
        bt::require(stored.size() == 4, "two components x two intervals");
        for (std::size_t i = 0; i < stored.size(); ++i) {
            const auto interval = i / 2;
            const bool chain = i % 2 == 0;
            bt::require(io::as_int64(detail::field(stored[i], "interval_id"), "i")
                            == static_cast<std::int64_t>(interval) &&
                        io::as_string(detail::field(stored[i], "component"), "c")
                            == (chain ? "chain" : "tires"),
                        "components explode per interval in order");
            const auto &vector = io::as_array(
                detail::field(stored[i], "qfrc"), "qfrc");
            bt::require(vector.size() == 8 &&
                        io::as_double(vector[0], "q") == (chain ? 3. : -1.),
                        "qfrc keeps the authored vector head");
            for (std::size_t k = 1; k < 8; ++k)
                bt::require(io::as_double(vector[k], "q") == 0.,
                            "qfrc keeps the authored vector tail");
            bt::require(io::tagged_double(detail::field(stored[i], "power_w"),
                                          "p") == (chain ? 6. : -2.),
                        "power_w is the authored qfrc*qvel product");
        }
        const auto work = reader->rows(
            "SELECT component_work_j FROM channel_energy ORDER BY interval_id");
        bt::require(work.size() == 2, "one work map per interval");
        for (const auto &row : work) {
            const auto &map = io::as_object(
                detail::field(row, "component_work_j"), "work");
            bt::require(io::tagged_double(detail::field(map, "chain"), "c") ==
                            .75 &&
                        io::tagged_double(detail::field(map, "tires"), "t") ==
                            -.25,
                        "component work is power * dt");
        } }
    // A malformed qfrc width reaches the storage validator, never the file.
    SyntheticCapture bad_options;
    auto bad = synthetic_capture(
        {sample_row(0, 0., .001, {}, {},
                    runtime::ForceComponents{{"chain", {1., 2.}}},
                    WireObject{{"chain", Wire{1.5}}})},
        std::move(bad_options));
    throws<std::runtime_error>(
        [&] { (void)rec::finalize(*bad, dir.path() / "bad"); },
        "force coordinate width",
        "qfrc width outside the DOF map is rejected");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_roundtrip_typed_nonfinite_presence_and_owned_asset();
        test_default_publisher_handles_empty_run_and_zero_step_episode();
        test_disabled_capture_does_not_create_output();
        test_publication_collision_does_not_adopt_even_an_empty_directory();
        test_report_failure_publishes_valid_database_with_explicit_status();
        test_manifest_is_last_and_failed_link_keeps_owned_evidence_retryable();
        test_reader_rejects_unsealed_or_tampered_payload();
        test_resealed_catalog_tamper_is_rejected_before_analysis();
        test_assets_are_verified_before_publication();
        test_multi_episode_round_trip_keeps_extents_and_events_isolated();
        test_forces_round_trip_records_components_and_checks_qfrc_width();
    });
}
