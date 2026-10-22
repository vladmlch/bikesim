// Faithful source regressions for all nine original episode functions.
// The live +.125 exit-buffer mutation uses the cold capture-only seam;
// synthetic codec data is not used in place of physical capture.
// No test in this file creates an oracle fixture.
#include "support/golden.hpp"
#include "support/model_fixture.hpp"
#include "support/temporary_directory.hpp"
#include "golden/episode.hpp"
#include "golden/capture_testing.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"
#include "research/environment.hpp"
#include "research/evidence.hpp"
#include "runtime/research.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <memory>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace io = bike::io;
namespace golden = bike::golden;
std::unique_ptr<runtime::NativeResearchRuntime> fresh() {
    return bike::research::make_environment(bt::oracle::root() / "episode/pinned_savage_40/request.toml");
}
golden::EpisodeArtifact capture(runtime::NativeResearchRuntime &environment, std::size_t steps) {
    runtime::RideControl control; control.human_torque_nm = 35.;
    return golden::capture_episode(environment, steps, control);
}
void same(const io::NpyArray &left, const io::NpyArray &right, std::string_view name) {
    bt::require(left.dtype == right.dtype && left.shape == right.shape &&
        left.fortran_order == right.fortran_order && left.bytes == right.bytes,
        "episode field dtype/shape/bits: " + std::string(name));
}
void state_and_force_fields(const golden::EpisodeArtifact &left, const golden::EpisodeArtifact &right) {
    same(left.state_qpos, right.state_qpos, "state_qpos");
    same(left.state_qvel, right.state_qvel, "state_qvel");
    same(left.state_act, right.state_act, "state_act");
    same(left.state_warmstart, right.state_warmstart, "state_warmstart");
    same(left.state_time, right.state_time, "state_time");
    same(left.forces, right.forces, "forces");
    same(left.ctrl_written, right.ctrl_written, "ctrl_written");
    same(left.tire_state, right.tire_state, "tire_state");
    same(left.qfrc_applied, right.qfrc_applied, "qfrc_applied");
}
io::Value first_failure(const runtime::NativeResearchRuntime &environment) {
    const auto state = environment.snapshot();
    return state.first_failure ? bike::research::wire_value(state.first_failure->as_wire()) : io::Value{};
}

// tests/reference/test_golden_episode.py::test_capture_is_deterministic
void test_capture_is_deterministic() {
    auto first = fresh(), second = fresh();
    const auto a = capture(*first, 50), b = capture(*second, 50);
    bt::require(a.channel_names == b.channel_names && a.rows.size() == 50 && b.rows.size() == 50,
        "fifty-step captures have identical channel schema and row count");
    // Same typed in-memory rows: exact equality, without compare tolerances.
    bt::require(a.rows == b.rows, "every recorder leaf is exactly deterministic");
    bt::require(io::canonical_json(io::Value{a.stored_rows}) == io::canonical_json(io::Value{b.stored_rows}),
        "nested non_numeric/list channels are exactly deterministic");
    bt::require(golden::same_failure(a.first_failure, b.first_failure, 0., 0.), "first monitor failure is exactly deterministic");
}

// tests/reference/test_golden_episode.py::test_saved_artifact_roundtrips
void test_saved_artifact_roundtrips() {
    bt::TemporaryDirectory const temporary("golden-fifty-roundtrip");
    auto environment = fresh(); const auto episode = capture(*environment, 50);
    const auto path = temporary.path() / "golden";
    golden::save(episode, path);
    const auto loaded = golden::load_episode(path);
    bt::require(loaded.channel_names == episode.channel_names && episode.rows.size() == 50, "fifty-step artifact schema retained");
    bt::require(golden::compare_rows(episode.rows, loaded.rows, 0., 0.).empty(), "all flattened saved rows round trip exactly");
    bt::require(golden::same_failure(episode.first_failure, loaded.first_failure, 0., 0.), "saved first failure retained");
    bt::require(std::filesystem::is_regular_file(path / "model.mjb") && !loaded.model_image.empty(), "saved artifact contains actual model image");
}

// tests/reference/test_episode_compare.py::test_replay_matches_golden_bitwise
void test_replay_matches_golden_bitwise() {
    auto original = fresh(); const auto episode = capture(*original, 50);
    auto replay = fresh(); const auto rows = golden::replay_episode(*replay, episode);
    bt::require(episode.rows.size() == 50 && golden::compare_rows(episode.rows, rows).empty(), "fifty-step replay reproduces every recorder row");
    bt::require(golden::same_failure(first_failure(*original), first_failure(*replay), 0., 0.), "replay has exactly the same first monitor failure");
}

// tests/reference/test_episode_compare.py::test_compare_episode_on_saved_artifact
void test_compare_episode_on_saved_artifact() {
    bt::TemporaryDirectory const temporary("golden-fifty-compare");
    auto original = fresh(); const auto episode = capture(*original, 50);
    const auto path = temporary.path() / "golden"; golden::save(episode, path);
    auto replay = fresh();
    bt::require(episode.rows.size() == 50 && golden::compare_episode(golden::load_episode(path), *replay).empty(),
        "saved fifty-step artifact replays through the full comparison entry point");
}

// tests/reference/test_episode_compare.py::test_compare_reports_first_divergence
void test_compare_reports_first_divergence() {
    const std::vector<golden::Row> a{{{"x", 1.}, {"y", 2.}}};
    const std::vector<golden::Row> b{{{"x", 1.}, {"y", 2. + 1e-9}}};
    const auto differences = golden::compare_rows(a, b);
    bt::require(!differences.empty() && differences.front().find("step=0") != std::string::npos &&
        differences.front().find("channel=y") != std::string::npos, "first divergence identifies the original step and channel");
}

// tests/reference/test_golden_episode_v2.py::test_v2_state_and_forces_recorded
void test_v2_state_and_forces_recorded() {
    auto environment = fresh();
    const auto empty = capture(*environment, 0);
    bike::model::ModelOwner model(engine::load_model_image(empty.model_image));
    bt::require(model != nullptr, "capture supplies a real loadable engine model");
    const auto nv = static_cast<std::size_t>(model->nv), nq = static_cast<std::size_t>(model->nq);
    bt::require(empty.qfrc_applied.shape == std::vector<std::size_t>{0, nv}, "zero-step capture has the actual model force width");
    const auto episode = capture(*environment, 40);
    bt::require(episode.state_qpos.shape == std::vector<std::size_t>{40, nq}, "forty entry qpos rows");
    bt::require(episode.state_qvel.shape == std::vector<std::size_t>{40, nv}, "forty entry qvel rows");
    bt::require(episode.state_warmstart.shape == std::vector<std::size_t>{40, nv}, "forty entry warmstart rows");
    bt::require(episode.forces.shape == std::vector<std::size_t>{40, episode.force_names.size(), nv}, "full forty-step component matrix");
    bt::require(episode.qfrc_applied.shape == std::vector<std::size_t>{40, nv}, "independently captured live exit totals");
    const auto *snapshots = io::find_member(io::as_object(episode.manifest, "manifest"), "tire_snapshots");
    bt::require(snapshots != nullptr && io::as_array(*snapshots, "tire_snapshots").size() == 40, "one tyre snapshot record per physics step");
    for (const auto name : {"fork_spring", "shock_coil", "shock_damper"})
        bt::require(std::ranges::find(episode.force_names, name) != episode.force_names.end(), "suspension component present: " + std::string(name));
    const auto forces = episode.forces.doubles(), totals = episode.qfrc_applied.doubles();
    for (std::size_t step = 0; step < 40; ++step) {
        std::vector<double> total(nv, 0.);
        for (std::size_t component = 0; component < episode.force_names.size(); ++component)
            for (std::size_t dof = 0; dof < nv; ++dof)
                total[dof] += forces[(step * episode.force_names.size() + component) * nv + dof];
        for (std::size_t dof = 0; dof < nv; ++dof)
            bt::require(std::bit_cast<std::uint64_t>(total[dof]) == std::bit_cast<std::uint64_t>(totals[step * nv + dof]),
                "exit total step " + std::to_string(step) + " dof " + std::to_string(dof) + " equals ordered component fold bitwise");
    }
}

// tests/reference/test_golden_episode_v2.py::test_v2_roundtrip_and_determinism
void test_v2_roundtrip_and_determinism() {
    bt::TemporaryDirectory const temporary("golden-v2-roundtrip");
    auto first = fresh(), second = fresh();
    const auto episode = capture(*first, 40), other = capture(*second, 40);
    state_and_force_fields(episode, other);
    const auto path = temporary.path() / "golden"; golden::save(episode, path);
    const auto loaded = golden::load_episode(path);
    state_and_force_fields(loaded, episode);
    bt::require(loaded.force_names == episode.force_names, "all saved force component names retain order");
}

// tests/reference/test_golden_episode_v2.py::test_exit_total_is_captured_independently_of_components
void test_exit_total_is_captured_independently_of_components() {
    bt::TemporaryDirectory const temporary("golden-live-exit");
    auto environment = fresh();
    const auto empty = capture(*environment, 0);
    bike::model::ModelOwner model(engine::load_model_image(empty.model_image));
    bt::require(model != nullptr && model->nv > 0, "live-exit fixture has a generalized force buffer");
    const auto nv = static_cast<std::size_t>(model->nv);
    bt::require(empty.qfrc_applied.shape == std::vector<std::size_t>{0, nv}, "empty independent total retains model width");
    struct Observation {
        std::vector<double> values;
        std::size_t width, count = 0;
        bool invalid_span = false;
    } observed{.values=std::vector<double>(3 * nv), .width=nv, .count=0, .invalid_span=false};
    // Preallocated storage and a noexcept callback keep this regression
    // from introducing a new throwing operation into the physical step.
    // The borrowed live span is never retained after the callback returns.
    const auto changed_exit = [](void *context, std::span<double> live) noexcept {
        auto &out = *static_cast<Observation *>(context);
        if (live.empty() || live.size() != out.width || out.count >= 3) {
            out.invalid_span = true;
            return;
        }
        live[0] += 0.125;
        std::ranges::copy(live, std::span(out.values).subspan(out.count * out.width, out.width).begin());
        ++out.count;
    };
    runtime::RideControl control; control.human_torque_nm = 35.;
    const auto episode = golden::testing::capture_with_force_exit_mutation(
        *environment, 3, control, &observed, changed_exit);
    bt::require(!observed.invalid_span && observed.count == 3, "three live post-apply_forces observations");
    same(episode.qfrc_applied, io::double_array(observed.values, {3, nv}), "live mutated exit observations");
    bt::require(episode.forces.shape == std::vector<std::size_t>{3, episode.force_names.size(), nv},
        "changed capture keeps all three independent component matrices");
    const auto forces = episode.forces.doubles(), exits = episode.qfrc_applied.doubles();
    std::vector<double> reconstructed(nv, 0.);
    for (std::size_t component = 0; component < episode.force_names.size(); ++component)
        for (std::size_t dof = 0; dof < nv; ++dof)
            reconstructed[dof] += forces[(2 * episode.force_names.size() + component) * nv + dof];
    bool differs = false;
    for (std::size_t dof = 0; dof < nv; ++dof)
        differs = differs || std::bit_cast<std::uint64_t>(reconstructed[dof]) !=
            std::bit_cast<std::uint64_t>(exits[2 * nv + dof]);
    bt::require(differs, "actual changed exit cannot be reconstructed from its unchanged component matrix");
    const auto path = temporary.path() / "changed"; golden::save(episode, path);
    same(golden::load_episode(path).qfrc_applied, episode.qfrc_applied, "saved live changed exit");
}

// tests/reference/test_golden_episode_v2.py::test_load_old_artifact_without_exit_total[version]
void test_load_old_artifact_without_exit_total(std::string_view version) {
    bt::TemporaryDirectory const temporary("golden-old-artifact");
    bt::ModelFixture model(R"(<mujoco><worldbody><body><joint type="slide"/><geom size=".1"/></body></worldbody></mujoco>)");
    golden::EpisodeArtifact episode;
    const auto add = [&](std::string name, const auto &values) {
        episode.initial.emplace_back(std::move(name), io::double_array(values, {values.size()}));
    };
    add("qpos", model_access::readonly_buffer(model.data->qpos, model.model->nq));
    add("qvel", model_access::readonly_buffer(model.data->qvel, model.model->nv));
    add("act", model_access::readonly_buffer(model.data->act, model.model->na));
    add("ctrl", model_access::readonly_buffer(model.data->ctrl, model.model->nu));
    add("qacc_warmstart", model_access::readonly_buffer(model.data->qacc_warmstart, model.model->nv));
    const std::array time{0.}; add("time", time);
    const auto nv = static_cast<std::size_t>(model.model->nv);
    episode.force_names = {"old"}; episode.forces = io::double_array(std::vector<double>(2 * nv, 1.), {2, 1, nv});
    const auto path = temporary.path() / "golden";
    golden::save(episode, path, model.model.get());
    auto archive = io::parse_npz(io::read_numpy_bytes(path / "episode.npz"));
    std::erase_if(archive, [&](const auto &member) {
        const auto &key = member.first;
        return key == "qfrc_applied" || (version == "v1" &&
            (key.starts_with("state_") || key.starts_with("tire_") || key == "ctrl_written" || key == "forces" || key == "force_names"));
    });
    const auto bytes = io::encode_npz(archive);
    std::filesystem::remove(path / "episode.npz");
    bike::platform::write_exclusive(path / "episode.npz", bytes);
    const auto loaded = golden::load_episode(path);
    bt::require(loaded.qfrc_applied.shape == std::vector<std::size_t>{0, nv} && loaded.qfrc_applied.bytes.empty(),
        "legacy " + std::string(version) + " does not synthesize missing independent totals");
    if (version == "v2") same(loaded.forces, episode.forces, "legacy v2 components");
    else bt::require(loaded.forces.shape == std::vector<std::size_t>{0, 0, nv} && loaded.forces.bytes.empty(),
        "legacy v1 retains the original empty force-matrix shape");
}
} // namespace
int main() { return bt::run([] {
    test_capture_is_deterministic();
    test_saved_artifact_roundtrips();
    test_replay_matches_golden_bitwise();
    test_compare_episode_on_saved_artifact();
    test_compare_reports_first_divergence();
    test_v2_state_and_forces_recorded();
    test_v2_roundtrip_and_determinism();
    test_exit_total_is_captured_independently_of_components();
    for (const auto version : {"v1", "v2"}) test_load_old_artifact_without_exit_total(version);
}); }
