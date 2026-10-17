// Authored native evidence for T38–T46 contract tests. This helper does not
// run a simulator or claim to replace the acquired Python oracle fixtures.
#pragma once
#include "recording/capture.hpp"
#include "recording/publish.hpp"
#include "recording/value_util.hpp"
#include <array>
#include <memory>

namespace recording_fixture {
namespace rec = bike::recording;
namespace io = bike::io;
inline rec::RunMetadata metadata(std::string uuid = "00000000-0000-4000-8000-000000000038") {
    return {std::move(uuid), "2026-10-17T12:00:00.123456+00:00", "native", "ride",
        io::Value{io::Value::Object{{"fixture", io::Value{true}}}}, 0.};
}
inline rec::Catalog catalog() {
    const std::array<rec::JointCoords,2> joints{{{"root_x", "slide", 0, 0},
                                              {"root_pitch", "hinge", 1, 1}}};
    return rec::build_catalog(joints);
}
inline runtime::OwnedNativeEpisode episode(std::size_t count = 3) {
    std::vector<runtime::SamplePtr> samples;
    for (std::size_t i = 0; i < count; ++i) {
        auto sample = std::make_shared<runtime::PhysicalSampleData>();
        sample->interval_id = static_cast<std::int64_t>(i);
        sample->time_s = static_cast<double>(i) * .001;
        sample->end_time_s = static_cast<double>(i+1) * .001;
        sample->qpos = {static_cast<double>(i) * .002, -0.};
        sample->qvel = {2., .25};
        sample->forces = {{"chain", {3., -4.}}};
        sample->powers_w = {{"chain", runtime::Wire{5.}}};
        // Absent roots are deliberate: every channel table still has a row,
        // with explicit false parent masks and NULL leaf values.
        samples.push_back(std::move(sample));
    }
    return runtime::OwnedNativeEpisode::seal(1, samples,
        runtime::ColumnLayout{.root_x_qpos=0, .root_x_dof=0, .root_pitch_qpos=1});
}
inline std::shared_ptr<const rec::EpisodeEvidence> evidence(std::size_t count = 3) {
    auto result = std::make_shared<rec::EpisodeEvidence>();
    result->episode_id = 0;
    result->generation = 1;
    result->physical = rec::make_physical_source(episode(count), catalog());
    result->summary = io::Value{io::Value::Object{
        {"duration_s", rec::value::real(static_cast<double>(count)*.001)},
        {"outcome", io::Value{io::Value::Object{{"reason", rec::value::text("fixture")}}}},
        {"episode_metrics", io::Value{io::Value::Object{
            {"iterations", rec::value::integer(9007199254740993LL)},
            {"passed", io::Value{true}}, {"label", rec::value::text("quote,\" and\nUTF-8: велосипед")}}}}}};
    result->assets.push_back(rec::json_asset("fixture.json", io::Value{io::Value::Object{{"schema", rec::value::integer(1)}}}));
    return result;
}
inline std::unique_ptr<rec::RunCapture> capture(std::size_t count = 3,
    bool enabled = true, std::string uuid = "00000000-0000-4000-8000-000000000038") {
    auto result = std::make_unique<rec::RunCapture>(metadata(std::move(uuid)), enabled);
    result->add_episode(evidence(count));
    result->event({0,0,"episode_ended",.25,static_cast<std::int64_t>(count),{},
        io::Value{io::Value::Object{{"reason",rec::value::text("fixture")}}}});
    return result;
}
inline rec::Publication publish(const std::filesystem::path& root, std::size_t count = 3) {
    const auto run = capture(count);
    return rec::finalize(*run, root);
}
} // namespace recording_fixture
