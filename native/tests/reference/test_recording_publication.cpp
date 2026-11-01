// Completion 3.30 (T53). Source: tests/reference/test_recording_publication.py.
//
// Fault-injection contracts for final-only publication. The oracle
// monkeypatched storage._connect / os.link; the native seam is
// FinalizeOptions::before_stage, the same cold-path hook the production
// writers expose for tests. Self-contained contract assertions — no
// goldens.
#include "recording_test_support.hpp"
#include "support/temporary_directory.hpp"
#include "diag.hpp"
#include "recording/publish.hpp"
#include "recording/reader.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>

namespace {

using namespace recording_test;

// The oracle's ['insert', 'COMMIT', 'CHECKPOINT', 'close'] phases map to
// the write_database stage names the before_stage hook receives.
void test_database_failure_retains_capture_and_retry_uses_new_directory() {
    for (const auto &[phase, stage] : std::map<std::string, std::string>{
             {"insert", "append"}, {"COMMIT", "commit"},
             {"CHECKPOINT", "checkpoint"}, {"close", "database_close"}}) {
        bt::TemporaryDirectory root("pub-db-failure-" + phase);
        const auto capture = capture_rows({sample_row(0, 0.)});
        bool armed = true;
        rec::FinalizeOptions options;
        options.before_stage = [&](std::string_view name) {
            if (armed && name == stage) {
                armed = false;
                throw std::runtime_error("injected " + phase);
            }
        };
        throws<std::runtime_error>([&] { (void)rec::finalize(*capture, root.path(), options); },
                                   "injected", phase + " failure propagates");
        bt::require(capture->state() == rec::RunCapture::State::publication_failed,
                    "capture stays retryable after " + phase + " failure");
        bt::require(capture->episodes().size() == 1 &&
                    capture->episodes()[0]->physical->row_count() == 1,
                    "owned evidence is intact after " + phase + " failure");
        bool sealed = false;
        for (const auto &entry : std::filesystem::directory_iterator(root.path()))
            sealed |= std::filesystem::exists(entry.path() / "manifest.json");
        bt::require(!sealed, "no manifest sealed on " + phase + " failure");
        // Retry publishes into a fresh destination with the intact evidence.
        const auto publication = rec::finalize(*capture, root.path(), options);
        bt::require(capture->state() == rec::RunCapture::State::published &&
                    std::filesystem::exists(publication.directory / "manifest.json"),
                    "retry seals a new directory after " + phase + " failure");
        bt::require(rec::finalize(*capture, root.path() / "ignored-after-success", options).directory ==
                    publication.directory,
                    "a successful publication is cached");
        rec::Recording saved(publication.directory);
        const auto episode = saved.episode(0);
        bt::require(as_integer(*member(as_object(episode), "physical_rows")) == 1,
                    "retried publication carries the retained row");
        saved.close();
    }
}

void test_payload_failure_does_not_seal_or_replace_existing_files() {
    bt::TemporaryDirectory root("pub-payload-failure");
    const auto unrelated = root.path() / "notes.txt";
    {
        std::ofstream out(unrelated);
        out << "belongs to the user";
    }
    const auto capture = capture_rows({sample_row(0, 0.)});
    rec::FinalizeOptions options;
    options.before_stage = [](std::string_view stage) {
        if (stage == "link_run.duckdb")
            throw std::runtime_error("payload publication injection");
    };
    throws<std::runtime_error>([&] { (void)rec::finalize(*capture, root.path(), options); },
                               "payload publication", "payload link failure propagates");
    std::ifstream in(unrelated);
    std::string const contents((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
    bt::require(contents == "belongs to the user",
                "unrelated files are never replaced");
    bool sealed = false;
    for (const auto &entry : std::filesystem::directory_iterator(root.path()))
        sealed |= std::filesystem::exists(entry.path() / "manifest.json");
    bt::require(!sealed, "no manifest sealed on payload failure");
    bt::require(capture->state() == rec::RunCapture::State::publication_failed,
                "capture stays retryable after payload failure");
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_database_failure_retains_capture_and_retry_uses_new_directory();
        test_payload_failure_does_not_seal_or_replace_existing_files();
    });
}
