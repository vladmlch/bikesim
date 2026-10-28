// Completion 3.31 (T53). Source: tests/reference/test_recording_report.py.
//
// Report regeneration is read-only and byte-idempotent: rendering a sealed
// recording reproduces the shipped summary.md exactly, twice, without
// touching a single payload byte. The oracle's `reader.render()` seam is
// recording::render_summary over the opened container.
#include "recording_test_support.hpp"
#include "recording/publish.hpp"
#include "recording/report.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace {

using namespace recording_test;
using Object = io::Value::Object;

std::map<std::string, std::string> snapshot(const std::filesystem::path &dir) {
    std::map<std::string, std::string> out;
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
        std::ifstream in(entry.path(), std::ios::binary);
        out[entry.path().filename().string()] =
            std::string(std::istreambuf_iterator<char>(in), {});
    }
    return out;
}

void test_report_regeneration_is_read_only_and_idempotent() {
    bt::TemporaryDirectory dir("report-idempotent");
    auto capture = capture_rows(
        {sample_row(0, 0., .001, {}, {}, {}, {}, energy_channels())},
        "viewer",
        Value{Object{
            {"outcome",
             Value{Object{{"reason", text("synthetic")}}}},
            {"first_failure", Value{}}}});
    const auto path = rec::finalize(*capture, dir.path()).directory;
    const auto before = snapshot(path);
    const auto reader = sealed(path);
    const auto regenerated = rec::render_summary(*reader);
    bt::require(regenerated == bt::text(path / "summary.md"),
                "the regenerated report is the shipped summary, byte-exact");
    bt::require(rec::render_summary(*reader) == regenerated,
                "a second render is identical");
    bt::require(snapshot(path) == before,
                "rendering never writes into the sealed container");
    for (const std::string_view marker :
         {"saved", "Episode 0", "first_failure", "SELECT",
          "channel_energy`.residual_j"})
        bt::require(regenerated.find(marker) != std::string::npos,
                    "report keeps the reviewed evidence language: " +
                        std::string(marker));
}

} // namespace

int main() {
    return bt::run([] {
        test_report_regeneration_is_read_only_and_idempotent();
    });
}
